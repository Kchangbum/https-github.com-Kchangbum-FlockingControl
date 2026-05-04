%% =======================================================================
%  Hybrid Flocking — DDPG (Parameter Sharing + 안정화 + Cascade 방지)
%
%  주요 변경점 (이전 버전 대비)
%   1) 단일 sharedAgent → 5개 RL Agent 블록이 같은 객체 참조 (parameter sharing)
%   2) Actor/Critic 네트워크에 LayerNormalization 삽입 (DDPG 안정화)
%   3) TargetSmoothFactor 5e-3 → 1e-3 (느린 타겟 갱신, Q-overestimation 완화)
%   4) Observation 10차원 → 14차원 (k=2 nearest neighbors, cascade 방지)
%   5) Reward = 0.7 × 개별 + 0.3 × 팀평균 (협응 학습)
%
%  ChangBum / ICAS2026
%% =======================================================================
clc; clear; close all;
d2r = pi/180;

%% 1. 모델/경로 설정
mdl = 'test_flocking_algorithm';
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
cd(this_dir);

if isempty(which([mdl '.slx']))
    error('Simulink 모델 ''%s.slx''을(를) 현재 폴더에서 찾지 못했습니다.\n현재 폴더: %s', mdl, this_dir);
end
if ~bdIsLoaded(mdl), load_system(mdl); end

%% 2. 공통 하이퍼파라미터
N_agents      = 5;
Ts            = 0.1;
Tf            = 200;

R_orbit       = 500;
V_target      = 20;
R_form        = 75;
R_safe        = 30;
R_collision   = 12;
R_swarm_max   = 200;

assignin('base','R_orbit',     R_orbit);
assignin('base','V_target',    V_target);
assignin('base','R_form',      R_form);
assignin('base','R_safe',      R_safe);
assignin('base','R_collision', R_collision);
assignin('base','R_swarm_max', R_swarm_max);
assignin('base','N_agents',    N_agents);
assignin('base','Ts',          Ts);
assignin('base','Tf',          Tf);

% RL Agent 블록 경로 (5개 그대로 유지, 단 모두 같은 sharedAgent 참조하도록)
blks = strings(1, N_agents);
for k = 1:N_agents
    blks(k) = sprintf('%s/2-Dimensional model/RL Agent%d', mdl, k);
end

%% 3. Observation / Action Specs (14차원으로 확장)
%  채널 1-8  : 기존 (drel_x/y, dV, sP, cP, e_radial, sE, cE)
%  채널 9-11 : nearest neighbor 1 (rel_x/R_form, rel_y/R_form, dist/R_safe)
%  채널 12-14: nearest neighbor 2 (rel_x/R_form, rel_y/R_form, dist/R_safe)
oinfo = rlNumericSpec([14 1]);
oinfo.Name        = 'observation';
oinfo.Description = '14-dim: centroid-rel + path-err + 2-nearest';

ainfo = rlNumericSpec([3 1], 'LowerLimit', 0, 'UpperLimit', 1);
ainfo.Name = 'gains_VFG_ACS_PF';

%% 4. Networks (LayerNorm 적용)
rng(0,'twister');
obsDim = oinfo.Dimension(1);
actDim = ainfo.Dimension(1);

% --- Critic Q(s,a) with LayerNorm ---
obsPath = [
    featureInputLayer(obsDim, 'Name','obsIn')
    fullyConnectedLayer(256, 'Name','co1')
    layerNormalizationLayer('Name','cln1')
    reluLayer('Name','cr1')
    fullyConnectedLayer(128, 'Name','co2')
    layerNormalizationLayer('Name','cln2')
    ];
actPath = [
    featureInputLayer(actDim, 'Name','actIn')
    fullyConnectedLayer(128, 'Name','ca1')
    layerNormalizationLayer('Name','cln_a')
    ];
mergePath = [
    additionLayer(2, 'Name','add')
    reluLayer('Name','cr2')
    fullyConnectedLayer(64, 'Name','cf')
    layerNormalizationLayer('Name','cln3')
    reluLayer('Name','cr3')
    fullyConnectedLayer(1, 'Name','QValue')
    ];
criticNet = dlnetwork();
criticNet = addLayers(criticNet, obsPath);
criticNet = addLayers(criticNet, actPath);
criticNet = addLayers(criticNet, mergePath);
criticNet = connectLayers(criticNet, 'cln2',  'add/in1');
criticNet = connectLayers(criticNet, 'cln_a', 'add/in2');
criticNet = initialize(criticNet);
critic = rlQValueFunction(criticNet, oinfo, ainfo, ...
    'ObservationInputNames','obsIn', 'ActionInputNames','actIn');

% --- Actor μ(s) with LayerNorm ---
actorNet = [
    featureInputLayer(obsDim, 'Name','aObsIn')
    fullyConnectedLayer(256)
    layerNormalizationLayer
    reluLayer
    fullyConnectedLayer(128)
    layerNormalizationLayer
    reluLayer
    fullyConnectedLayer(actDim, 'Name','aOut')
    sigmoidLayer('Name','sig')
    ];
actorNet = dlnetwork(actorNet);
actor = rlContinuousDeterministicActor(actorNet, oinfo, ainfo);

%% 5. DDPG Agents — 5개 별도 객체 (R2025b 호환), 동일 템플릿에서 출발
%  R2025b는 multiple RL Agent 블록이 같은 객체를 참조하는 것을 금지함.
%  대신 5개 별도 객체를 만들되, 같은 actor/critic 템플릿에서 출발 + 학습 후
%  주기적으로 가중치를 동기화하는 sync 함수(아래 sync_actor_critic) 사용.
agents = [];
for i = 1:N_agents
    a = rlDDPGAgent(actor, critic);   % 매번 새 객체 (R2025b 요구사항)
    a.AgentOptions.SampleTime              = Ts;
    a.AgentOptions.DiscountFactor          = 0.99;
    a.AgentOptions.MiniBatchSize           = 256;
    a.AgentOptions.ExperienceBufferLength  = 1e6;
    a.AgentOptions.TargetSmoothFactor      = 1e-3;     % 5e-3 → 1e-3 (안정화)
    a.AgentOptions.NumStepsToLookAhead     = 1;

    a.AgentOptions.ActorOptimizerOptions = rlOptimizerOptions( ...
        'LearnRate', 5e-5, 'GradientThreshold', 1);
    a.AgentOptions.CriticOptimizerOptions = rlOptimizerOptions( ...
        'LearnRate', 5e-4, 'GradientThreshold', 1);

    % 영구 탐색 OU 노이즈
    a.AgentOptions.NoiseOptions.MeanAttractionConstant     = 0.15;
    a.AgentOptions.NoiseOptions.StandardDeviation          = 0.50;
    a.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-6;
    a.AgentOptions.NoiseOptions.StandardDeviationMin       = 0.10;

    agents = [agents; a]; %#ok<AGROW>
end

% 각 에이전트를 base workspace에 등록 (Simulink 블록이 각각 참조)
for i = 1:N_agents
    assignin('base', sprintf('agentObj%d', i), agents(i));
end

fprintf('[OK] %d separate DDPG agents created (identical init).\n', N_agents);
fprintf('     Centralized learning + shared experience buffer 적용.\n');

%% 6. 환경 / 학습 옵션
env = rlSimulinkEnv(mdl, blks);
[env.ResetFcn]       = deal(@localResetFcn);
[env.UseFastRestart] = deal('on');
fprintf('[OK] Environment ready.\n');

trainOpts = rlMultiAgentTrainingOptions( ...
    'AgentGroups',                {1:N_agents}, ...
    'LearningStrategy',           'centralized', ...
    'MaxEpisodes',                1500, ...        % 충분한 학습 시간 확보
    'MaxStepsPerEpisode',         ceil(Tf/Ts), ...
    'ScoreAveragingWindowLength', 30, ...
    'StopTrainingCriteria',       'AverageReward', ...
    'StopTrainingValue',          -0.3, ...        % 0에 충분히 근접하면 종료 (-10에서 시작)
    'Plots',                      'training-progress', ...
    'Verbose',                    false);

evl = rlEvaluator('EvaluationFrequency', 25, 'NumEpisodes', 3);

%% 7. 학습 실행
%  옵션 A: 일반 train() — 5개 에이전트가 독립 학습 (centralized로 buffer 공유)
%  옵션 B: 청크 학습 + 주기적 가중치 동기화 (진짜 parameter sharing 흉내)
%
%  먼저 옵션 A로 시도하고, 1000 에피소드까지 비대칭 분기 보이면 옵션 B 활성화.
useChunkedSync = false;     % true로 바꾸면 옵션 B (sync 모드)
syncEvery      = 25;        % 25 에피소드마다 가중치 평균 → 5명에게 재분배

doTrain = true;
if doTrain
    if ~useChunkedSync
        % --- 옵션 A: 표준 학습 ---
        trainingStats = train(agents, env, trainOpts, 'Evaluator', evl);
    else
        % --- 옵션 B: 청크 학습 + sync ---
        totalEps      = trainOpts.MaxEpisodes;
        chunks        = ceil(totalEps / syncEvery);
        trainingStats = [];
        for c = 1:chunks
            epsThisChunk           = min(syncEvery, totalEps - (c-1)*syncEvery);
            chunkOpts              = trainOpts;
            chunkOpts.MaxEpisodes  = epsThisChunk;
            chunkOpts.Plots        = 'training-progress';

            chunkStats   = train(agents, env, chunkOpts, 'Evaluator', evl);
            trainingStats = [trainingStats; chunkStats]; %#ok<AGROW>

            % --- 가중치 동기화: agents(1) → 나머지 ---
            sync_actor_critic(agents);
            fprintf('[SYNC] After chunk %d/%d (ep %d): weights synced.\n', ...
                    c, chunks, c*syncEvery);
        end
    end
    save(fullfile(this_dir,'DDPG_HybridFlocking_agents.mat'), 'agents');
    save(fullfile(this_dir,'DDPG_HybridFlocking_stats.mat'),  'trainingStats');
    fprintf('[DONE] Training complete. Saved in %s\n', this_dir);
end

%% =======================================================================
%  Reset Function
%% =======================================================================
function in = localResetFcn(in)
    N         = evalin('base','N_agents');
    R_orbit   = evalin('base','R_orbit');
    V_target  = evalin('base','V_target');

    R_form_init   = 60;
    min_dist_init = 25;

    radial0 = R_orbit + (rand()-0.5) * 200;
    th0     = rand() * 2 * pi;
    cx      = radial0 * cos(th0);
    cy      = radial0 * sin(th0);
    psi_tan = th0 + pi/2;

    state0 = zeros(5, N);
    x0 = zeros(1,N); y0 = zeros(1,N);
    for i = 1:N
        valid = false;
        while ~valid
            r_ = R_form_init * sqrt(rand());
            t_ = rand() * 2 * pi;
            xi = cx + r_ * cos(t_);
            yi = cy + r_ * sin(t_);
            if i == 1
                valid = true;
            else
                d = hypot(x0(1:i-1) - xi, y0(1:i-1) - yi);
                if all(d >= min_dist_init), valid = true; end
            end
        end
        x0(i) = xi; y0(i) = yi;
    end

    state0(1,:) = x0;
    state0(2,:) = y0;
    state0(3,:) = V_target;
    state0(4,:) = psi_tan + 0.1*(rand(1,N)-0.5);
    state0(5,:) = 0;

    in = setVariable(in, 'init_state', state0);
end

%% =======================================================================
%  Sync Function — 5개 에이전트 가중치를 평균내서 모두에게 재분배
%   (Parameter Sharing 흉내)
%% =======================================================================
function sync_actor_critic(agents)
    N = numel(agents);
    if N < 2, return; end

    % --- Actor 가중치 평균 ---
    actorParamsList = cell(N, 1);
    for i = 1:N
        actorParamsList{i} = getLearnableParameters(getActor(agents(i)));
    end
    actorAvg = average_param_cells(actorParamsList);
    for i = 1:N
        actor_i = getActor(agents(i));
        actor_i = setLearnableParameters(actor_i, actorAvg);
        setActor(agents(i), actor_i);
    end

    % --- Critic 가중치 평균 ---
    criticParamsList = cell(N, 1);
    for i = 1:N
        criticParamsList{i} = getLearnableParameters(getCritic(agents(i)));
    end
    criticAvg = average_param_cells(criticParamsList);
    for i = 1:N
        critic_i = getCritic(agents(i));
        critic_i = setLearnableParameters(critic_i, criticAvg);
        setCritic(agents(i), critic_i);
    end
end

function avg = average_param_cells(cellList)
    %  cellList{i} = i번째 에이전트의 learnable params (cell of dlarray)
    N    = numel(cellList);
    avg  = cellList{1};
    nP   = numel(avg);
    for k = 1:nP
        s = avg{k};
        for i = 2:N
            s = s + cellList{i}{k};
        end
        avg{k} = s / N;
    end
end
