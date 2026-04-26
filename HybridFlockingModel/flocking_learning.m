clc; 
d2r = pi/180;
m = 10;
g = 9.806;
%% 1. 기초 설정 및 모델 경로 확인
mdl = 'test_flocking_algorithm';
% load_system(mdl); % 필요 시 주석 해제

% 모델 내 'RL Agent' 블록 경로 자동 추출 (줄바꿈/공백 문제 해결)
% allAgentBlocks = find_system(mdl, 'RegExp', 'on', 'Name', 'RL Agent*');
% blks = string(sort(allAgentBlocks)); 
blks = [...
    "test_flocking_algorithm/2-Dimensional model/RL Agent1",...
    "test_flocking_algorithm/2-Dimensional model/RL Agent2",...
    "test_flocking_algorithm/2-Dimensional model/RL Agent3",...
    "test_flocking_algorithm/2-Dimensional model/RL Agent4",...
    "test_flocking_algorithm/2-Dimensional model/RL Agent5"...
    ];
% if numel(blks) ~= 5
%     error('모델에서 RL Agent 블록 5개를 찾을 수 없습니다.');
% end

Ts = 0.1; % 샘플링 시간
Tf = 300; % 에피소드 종료 시간
%% 2. 에이전트 규격(Spec) 정의
oinfo = rlNumericSpec([6 1]); oinfo.Name = "observations";
ainfo = rlNumericSpec([3 1], 'LowerLimit', 0.1, 'UpperLimit', 1); ainfo.Name = "actors";

%% 3. 네트워크 설계 (Shared Policy)
rng(0, "twister");

% --- Critic 네트워크 ---
obsPath = featureInputLayer(oinfo.Dimension(1), Name="obsInLyr");
actPath = featureInputLayer(ainfo.Dimension(1), Name="actInLyr");
commonPath = [
    concatenationLayer(1, 2, Name="concat")
    fullyConnectedLayer(256); reluLayer()
    fullyConnectedLayer(128); reluLayer()
    fullyConnectedLayer(1, Name="QValue")
    ];
criticNet = dlnetwork();
criticNet = addLayers(criticNet, obsPath);
criticNet = addLayers(criticNet, actPath);
criticNet = addLayers(criticNet, commonPath);
criticNet = connectLayers(criticNet, "obsInLyr", "concat/in1");
criticNet = connectLayers(criticNet, "actInLyr", "concat/in2");
critic = rlQValueFunction(criticNet, oinfo, ainfo, ...
    ObservationInputNames="obsInLyr", ActionInputNames="actInLyr");

% --- Actor 네트워크 ---
actorNet = [
    featureInputLayer(oinfo.Dimension(1))
    fullyConnectedLayer(256); reluLayer()
    fullyConnectedLayer(128); reluLayer()
    fullyConnectedLayer(ainfo.Dimension(1), Name="actOut")
    sigmoidLayer(Name="sigmoid") 
    ];
actorNet = dlnetwork(actorNet);
actor = rlContinuousDeterministicActor(actorNet, oinfo, ainfo);

%% 4. [중요] 에이전트 객체 5개를 먼저 생성 (변수명: agents)
% ★ 에러 메시지 해결의 핵심: 'agents'라는 이름의 배열로 5개를 미리 정의합니다.
agents = []; 
for i = 1:5
    newAgent = rlDDPGAgent(actor, critic);
    newAgent.AgentOptions.SampleTime = Ts;
    newAgent.AgentOptions.DiscountFactor = 0.995;
    newAgent.AgentOptions.MiniBatchSize = 128;
    newAgent.AgentOptions.ExperienceBufferLength = 1e6;
    newAgent.AgentOptions.TargetSmoothFactor = 5e-4;
    newAgent.AgentOptions.ActorOptimizerOptions = rlOptimizerOptions(LearnRate=1e-5, GradientThreshold=1);
    newAgent.AgentOptions.CriticOptimizerOptions = rlOptimizerOptions(LearnRate=1e-4, GradientThreshold=1);
    newAgent.AgentOptions.NoiseOptions.StandardDeviation = 0.5;
    newAgent.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-4;
    
    agents = [agents; newAgent];
end

%% 5. 환경 생성 (이제 메모리에 agents가 있으므로 성공합니다)
% ★ 여기서 두 번째 인자로 agents(에이전트 배열)를 직접 전달하거나, 
% Workspace에 agents가 있는 상태에서 blks만 넘겨줍니다.
env = rlSimulinkEnv(mdl, blks); 

% env.ResetFcn = @localResetFcn;
[env.ResetFcn] = deal(@localResetFcn);
% env.UseFastRestart = 'on';
[env.UseFastRestart] = deal('on');

fprintf('환경(env) 생성에 성공했습니다!\n');

%% 6. 학습 설정 및 시작
trainOpts = rlMultiAgentTrainingOptions(...
    AgentGroups={[1,2,3,4,5]}, ... 
    LearningStrategy="centralized", ...
    MaxEpisodes=1000, ...
    MaxStepsPerEpisode=ceil(Tf/Ts), ...
    Plots="training-progress", ...
    Verbose=false, ...
    StopTrainingCriteria="EvaluationStatistic", ...
    StopTrainingValue=1e12...
    );

evl = rlEvaluator(EvaluationFrequency=20, NumEpisodes=5);

if true
    trainingStates = train(agents, env, trainOpts, Evaluator=evl);
    save("DDPG_Flocking_Final_1.mat", "agents");
    save("DDPG_Flocking_Final_1TrainingStates", "trainingStates");
end
%% 초기화 함수
function in = localResetFcn(in)

    N = 5; 
    min_dist = 80; r_min = 750; r_max = 800; scatter_range = 300;
    
    center_r = r_min + (r_max - r_min) * rand();
    center_theta = rand() * 2 * pi;
    center_x = center_r * cos(center_theta);
    center_y = center_r * sin(center_theta);
    
    state0 = zeros(5, N);
    x0 = zeros(1, N); y0 = zeros(1, N);
    
    for i = 1:N
        valid = false;
        while ~valid
            x_temp = center_x + (rand()-0.5) * 2 * scatter_range;
            y_temp = center_y + (rand()-0.5) * 2 * scatter_range;
            if i == 1
                valid  = true;
            else
                dists = sqrt((x0(1:i-1) - x_temp).^2 + (y0(1:i-1) - y_temp).^2);
                if all(dists >= min_dist), valid = true; end
            end
        end
        x0(i) = x_temp; y0(i) = y_temp;
    end
    
    to_center = atan2(-y0, -x0);
    tangent = atan2(y0, x0) + pi/2;
    
    state0(1,:) = x0; 
    state0(2,:) = y0; 
    state0(3,:) = 20; 
    state0(4,:) = 0.7 * to_center + 0.  * tangent; 
    state0(5,:) = 0;  
    
    in = setVariable(in, 'init_state', state0); 
end