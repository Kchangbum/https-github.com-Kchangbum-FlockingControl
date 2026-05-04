%% =======================================================================
%  Hybrid Flocking — DDPG (centroid path-following + flexible local motion)
%  - 군집 중점이 원형 궤도(R_orbit)를 추종
%  - 각 에이전트는 중점 주위 R_form 반경 안에서 자유롭게 비행
%  - 정규화 제곱오차 기반 연속 cost 보상으로 충돌 회피 학습 가속
%  - 보상은 음수 누적형: episode 총합 ≈ -10 → 0 (우상향)
%  ChangBum / ICAS2026
%% =======================================================================
clc; clear; close all;
d2r = pi/180;

%% 1. 모델/경로 설정 (이 컴퓨터에 맞춰 자동 인식)
mdl = 'test_flocking_algorithm';
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);                                   % 보상/관측 함수 경로 추가
cd(this_dir);                                        % 작업 디렉토리 고정

if isempty(which([mdl '.slx']))
    error('Simulink 모델 ''%s.slx''을(를) 현재 폴더에서 찾지 못했습니다.\n현재 폴더: %s', mdl, this_dir);
end
if ~bdIsLoaded(mdl), load_system(mdl); end

%% 2. 공통 하이퍼파라미터 (Simulink 보상 블록과 공유)
N_agents      = 5;            % 에이전트 수
Ts            = 0.1;          % 샘플링 시간 [s]
Tf            = 200;          % 1 에피소드 길이 [s]

R_orbit       = 500;          % 목표 군집 중점 궤도 반경 [m]
V_target      = 20;           % 목표 속도 [m/s]
R_form        = 75;           % 각 에이전트가 자유롭게 비행 가능한 반경 [m]
R_safe        = 50;           % 안전 이격 거리 [m]
R_collision   = 12;           % 충돌 임계 [m]   (이 값 미만이면 큰 페널티)
R_swarm_max   = 200;          % 군집 최대 산개 [m]

% Simulink 워크스페이스로 전달
assignin('base','R_orbit',     R_orbit);
assignin('base','V_target',    V_target);
assignin('base','R_form',      R_form);
assignin('base','R_safe',      R_safe);
assignin('base','R_collision', R_collision);
assignin('base','R_swarm_max', R_swarm_max);
assignin('base','N_agents',    N_agents);
assignin('base','Ts',          Ts);
assignin('base','Tf',          Tf);

% RL Agent 블록 경로
blks = strings(1, N_agents);
for k = 1:N_agents
    blks(k) = sprintf('%s/2-Dimensional model/RL Agent%d', mdl, k);
end

%% 3. Observation / Action Specs
% Observation (10차원, 모두 정규화)
%   1) (x_i - x_c)/R_form           : 군집 중점 기준 상대 X
%   2) (y_i - y_c)/R_form           : 군집 중점 기준 상대 Y
%   3) (V_i - V_target)/V_target    : 속도 오차
%   4) sin(psi_i)                   : 헤딩 sin
%   5) cos(psi_i)                   : 헤딩 cos
%   6) (|p_c| - R_orbit)/R_orbit    : 군집 중점의 궤도 반경 오차
%   7) sin(psi_des - psi_i)         : 접선 방향 헤딩 오차 sin
%   8) cos(psi_des - psi_i)         : 접선 방향 헤딩 오차 cos
%   9) (Δx_nearest)/R_form          : 가장 가까운 이웃 상대 X
%  10) d_min / R_safe               : 가장 가까운 이웃까지 거리(정규화)
oinfo = rlNumericSpec([10 1]);
oinfo.Name        = 'observation';
oinfo.Description = 'centroid-rel + path-err + nearest-neighbor';

% Action: [k_VFG, k_ACS, k_PF] ∈ [0, 1]   ←  하이브리드 게인 학습
ainfo = rlNumericSpec([3 1], 'LowerLimit', 0, 'UpperLimit', 1);
ainfo.Name = 'gains_VFG_ACS_PF';

%% 4. 신경망 (Shared Policy)
rng(0,'twister');
obsDim = oinfo.Dimension(1);
actDim = ainfo.Dimension(1);

% --- Critic Q(s,a) ------------------------------------------------------
obsPath = [
    featureInputLayer(obsDim, 'Name','obsIn')
    fullyConnectedLayer(256,  'Name','co1'); reluLayer('Name','cr1')
    fullyConnectedLayer(128,  'Name','co2')
    ];
actPath = [
    featureInputLayer(actDim, 'Name','actIn')
    fullyConnectedLayer(128,  'Name','ca1')
    ];
mergePath = [
    additionLayer(2,          'Name','add')
    reluLayer('Name','cr2')
    fullyConnectedLayer(64,   'Name','cf'); reluLayer('Name','cr3')
    fullyConnectedLayer(1,    'Name','QValue')
    ];
criticNet = dlnetwork();
criticNet = addLayers(criticNet, obsPath);
criticNet = addLayers(criticNet, actPath);
criticNet = addLayers(criticNet, mergePath);
criticNet = connectLayers(criticNet, 'co2', 'add/in1');
criticNet = connectLayers(criticNet, 'ca1', 'add/in2');
criticNet = initialize(criticNet);
critic = rlQValueFunction(criticNet, oinfo, ainfo, ...
    'ObservationInputNames','obsIn', 'ActionInputNames','actIn');

% --- Actor μ(s) ∈ [0,1] (sigmoid) --------------------------------------
actorNet = [
    featureInputLayer(obsDim, 'Name','aObsIn')
    fullyConnectedLayer(256); reluLayer
    fullyConnectedLayer(128); reluLayer
    fullyConnectedLayer(actDim, 'Name','aOut')
    sigmoidLayer('Name','sig')
    ];
actorNet = dlnetwork(actorNet);
actor = rlContinuousDeterministicActor(actorNet, oinfo, ainfo);

%% 5. DDPG 에이전트 N개 (공유 템플릿)
agents = [];
for i = 1:N_agents
    a = rlDDPGAgent(actor, critic);
    a.AgentOptions.SampleTime              = Ts;
    a.AgentOptions.DiscountFactor          = 0.99;
    a.AgentOptions.MiniBatchSize           = 256;
    a.AgentOptions.ExperienceBufferLength  = 1e6;
    a.AgentOptions.TargetSmoothFactor      = 5e-3;     % soft target update
    a.AgentOptions.NumStepsToLookAhead     = 1;

    a.AgentOptions.ActorOptimizerOptions   = rlOptimizerOptions( ...
        'LearnRate', 5e-5, 'GradientThreshold', 1);   % 살짝 낮춰서 분기 완화
    a.AgentOptions.CriticOptimizerOptions  = rlOptimizerOptions( ...
        'LearnRate', 5e-4, 'GradientThreshold', 1);

    % OU 노이즈: 탐색 유지 (정책 붕괴 방지)
    %  - 표준편차를 0.5로 키우고 감쇠를 매우 천천히
    %  - σ_min도 0.1로 올려 영구 탐색 보장 (local min 탈출용)
    a.AgentOptions.NoiseOptions.MeanAttractionConstant     = 0.15;
    a.AgentOptions.NoiseOptions.StandardDeviation          = 0.50;
    a.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-6;   % 10배 느림
    a.AgentOptions.NoiseOptions.StandardDeviationMin       = 0.10;   % 영구 탐색

    agents = [agents; a]; %#ok<AGROW>
end

%% 6. 환경 / 학습 옵션
env = rlSimulinkEnv(mdl, blks);
[env.ResetFcn]       = deal(@localResetFcn);
[env.UseFastRestart] = deal('on');
fprintf('환경 생성 완료 (agents = %d)\n', N_agents);

trainOpts = rlMultiAgentTrainingOptions( ...
    'AgentGroups',                {1:N_agents}, ...
    'LearningStrategy',           'centralized', ...
    'MaxEpisodes',                5000, ...
    'MaxStepsPerEpisode',         ceil(Tf/Ts), ...
    'ScoreAveragingWindowLength', 30, ...        % 30 에피소드 이동평균 → 그래프 매끈
    'StopTrainingCriteria',       'AverageReward', ...
    'StopTrainingValue',          1e6, ...      % 0에 충분히 근접 → 학습 종료
    'Plots',                      'training-progress', ...
    'Verbose',                    false);

evl = rlEvaluator('EvaluationFrequency', 25, 'NumEpisodes', 3);

%% 7. 학습 실행
doTrain = true;
if doTrain
    trainingStats = train(agents, env, trainOpts, 'Evaluator', evl);
    save(fullfile(this_dir,'DDPG_HybridFlocking8-4.mat'),       'agents');
    save(fullfile(this_dir,'DDPG_HybridFlocking_stats8-4.mat'), 'trainingStats');
    EpisodeRewards = trainingStats.EpisodeReward;
    AvgRewards = trainingStats.AverageReward;
    save("rewards8-4.mat", "AvgRewards", "EpisodeRewards");
    fprintf('학습 완료. 결과 저장: %s\n', this_dir);
end

%% =======================================================================
%  Reset Function (에피소드마다 호출)
%   - 군집 중점을 목표 궤도 근처(±200m)에 랜덤 배치
%   - 중점 주위에 5대를 충돌 없이 균등 배치
%   - 초기 헤딩은 접선 방향 + 약한 잡음
%% =======================================================================
function in = localResetFcn(in)
    N         = evalin('base','N_agents');
    R_orbit   = evalin('base','R_orbit');
    V_target  = evalin('base','V_target');

    R_form_init   = 60;        % 시작 시 군집 반경(작게 모아서 안정 출발)
    min_dist_init = 25;        % 초기 충돌 방지 최소 이격

    % 군집 중점 초기 배치
    radial0 = R_orbit + (rand()-0.5) * 200;
    th0     = rand() * 2 * pi;
    cx      = radial0 * cos(th0);
    cy      = radial0 * sin(th0);
    psi_tan = th0 + pi/2;       % 접선 방향

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
