clc;
% open_system("test_flocking_algorithm");
mdl = 'test_flocking_algorithm';
% load_system(mdl); % 환경 생성 전 모델 로드 필수
agentPath = [mdl, '/2-Dimensional model/RL Agent'];
previousRngState = rng(0,"twister");

%% Info & Setting value
obsInfo = rlNumericSpec([80 1]); % 8개의 상태 정보
obsInfo.Name = "observations";
actInfo = rlNumericSpec([3 1]); % 3개의 가중치 (0~1 사이)
actInfo.Name = "actors";

Ts = 0.1; % Sample time
Tf = 300; % Final time
 
%% Creating critic
% Observation path
obsPath = featureInputLayer(obsInfo.Dimension(1), ...
    Name="obsInLyr");

% Action path
actPath = featureInputLayer(actInfo.Dimension(1), ...
    Name="actInLyr");
%% Environment
env = rlSimulinkEnv(mdl, agentPath, obsInfo, actInfo);
env.ResetFcn = @localResetFcn;
%%
% Common path
commonPath = [
    concatenationLayer(1, 2, Name="concat") % 1번째 차원으로 2개의 입력을 합침
    fullyConnectedLayer(256) % 뉴런 수를 50개 정도로 늘리면 학습이 더 잘 됩니다
    reluLayer() % 현실적 판단? 
    fullyConnectedLayer(128)
    reluLayer()
    fullyConnectedLayer(1, Name="QValue") % Q-value 사용
    ];

% Create the network object and add the layers
criticNet = dlnetwork();
criticNet = addLayers(criticNet,obsPath);
criticNet = addLayers(criticNet,actPath);
criticNet = addLayers(criticNet,commonPath);

% Connect the layers
criticNet = connectLayers(criticNet, ...
    "obsInLyr","concat/in1");
criticNet = connectLayers(criticNet, ...
    "actInLyr","concat/in2");
plot(criticNet);


rng(0,"twister");
criticNet = initialize(criticNet);
summary(criticNet);

% Create critic obj
critic = rlQValueFunction(criticNet, ...
    obsInfo,actInfo, ...
    ObservationInputNames="obsInLyr", ...
    ActionInputNames="actInLyr");

getValue(critic, ...
    {rand(obsInfo.Dimension)}, ...
    {rand(actInfo.Dimension)});

% Creating actor
actorNet = [
    featureInputLayer(obsInfo.Dimension(1))
    fullyConnectedLayer(256)
    reluLayer()
    fullyConnectedLayer(128)
    reluLayer()
    fullyConnectedLayer(actInfo.Dimension(1), Name="actOut") % 3 출력
    sigmoidLayer(Name="sigmoid")       
    % scalingLayer(Scale=1, Bias=0.01, Name="scale")

    ];

rng(0,"twister");
actorNet = dlnetwork(actorNet);
summary(actorNet);
plot(actorNet);

% Actor 생성 시 입력/출력 레이어 이름 명시 (안정성)
actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo);
getAction(actor,{rand(obsInfo.Dimension)});

%% Creating DDPG(Deep Deterministic Policy Gradient) agent
agent = rlDDPGAgent(actor,critic);

agent.AgentOptions.SampleTime = Ts;
agent.AgentOptions.DiscountFactor = 0.95;
agent.AgentOptions.MiniBatchSize = 128;
agent.AgentOptions.ExperienceBufferLength = 1e6;
% agent.AgentOptions.TargetSmoothFactor = 1e-3; % Target Network 업데이트 속도 (안정성)

actorOpts = rlOptimizerOptions( ...
    LearnRate=1*1e-4, ...
    GradientThreshold=1);
criticOpts = rlOptimizerOptions( ...
    LearnRate=5*1e-4, ...
    GradientThreshold=1);
agent.AgentOptions.ActorOptimizerOptions = actorOpts;
agent.AgentOptions.CriticOptimizerOptions = criticOpts;

% Noise model
agent.AgentOptions.NoiseOptions.StandardDeviation = 0.5;
agent.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-4;
getAction(agent,{rand(obsInfo.Dimension)});
% 

% v.ResetFcn = @(in) setVariable(in, 'state0', state0, 'Workspace', mdl);

% Training agent
% training options
trainOpts = rlTrainingOptions(...
    MaxEpisodes=200, ...
    MaxStepsPerEpisode=ceil(Tf/Ts), ...
    Plots="training-progress", ...
    Verbose=false, ...
    StopTrainingCriteria="EvaluationStatistic", ...
    SaveAgentDirectory= pwd, ...
    StopTrainingValue= 0 ...
    );

% agent evaluator
evl = rlEvaluator(EvaluationFrequency=10,NumEpisodes=5);

rng(0,"twister");

doTraining = false;

% Start the training process if doTraining is true
if doTraining
    % Train the agent.
    trainingStats = train(agent, env, trainOpts, Evaluator=evl);
    save("RL_test/AgentTest5_2.mat","agent");
else
    load("RL_test/AgentTest5_2.mat", "agent");
    trainingStats = train(agent, env, trainOpts, Evaluator=evl);
    save("RL_test/AgentTest5_3.mat","agent");
end 

%% Validation training agent
rng(0,"twister");

simOpts = rlSimulationOptions( ...
    MaxSteps=ceil(Tf/Ts), ...
    StopOnError="on");
experiences = sim(env,agent,simOpts);

rng(previousRngState);

%% Initialization
function in = localResetFcn(in)
    % 1. 에피소드 카운트를 위한 영구 변수 설정
    disp('--- Reset 함수가 호출되었습니다! ---'); 

    persistent episode_count;
    if isempty(episode_count)
        episode_count = 0;
    end
    episode_count = episode_count + 1;
    N = 10; 
    min_dist = 75;
    state0 = zeros(5, N);
    x0 = zeros(1, N); y0 = zeros(1, N);
    r_min = 600;
    r_max = 900;
    scatter_range = 300;

    center_r = r_min + (r_max - r_min) * rand();
    center_theta = rand() * 2 * pi;
    center_x = center_r * cos(center_theta);
    center_y = center_r * sin(center_theta);

    for i = 1:N
        valid = false;
        while ~valid
            x_temp = center_x + (rand()-0.5) * 2 * scatter_range;
            y_temp = center_y + (rand()-0.5) * 2 * scatter_range;

            if i == 1
                valid = true;
            else
                dists = sqrt((x0(1:i-1) - x_temp).^2 + (y0(1:i-1) - y_temp).^2);
                if all(dists >= min_dist), valid = true; end
            end
        end
        x0(i) = x_temp; y0(i) = y_temp;
    end

    to_center = atan2(-y0, -x0);
    tangent = atan2(y0, x0) + pi/2;
    % 상태 대입 및 방향 설정
    state0(1,:) = x0;
    state0(2,:) = y0;
    state0(3,:) = 20; 
    state0(4,:) = 0.7 * to_center + 0.3 * tangent;     
    state0(5,:) = 0; 
    fprintf('DEBUG: 에피소드 %d 실행 중. r_min=%.1f\n', episode_count, r_min);

    strState = mat2str(state0);
    blkPath = 'test_flocking_algorithm/2-Dimensional model/UAVs/Integrator2';
    try
        in = setBlockParameter(in, blkPath, 'InitialCondition', strState);
    catch
        fprintf('경로 오류! 실제 경로: %s\n', blkPath);
    end
end