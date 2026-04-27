clc;
mdl = 'WaterTank';
agentPath = [mdl, '/RL Agent'];
previousRngState = rng(0,"twister");

%% Info & Setting value
% Observation info
obsInfo = rlNumericSpec([3 1],...
    LowerLimit=[-inf -inf 0]',...
    UpperLimit=[ inf  inf inf]');

% Name and description are optional and not used 
% by the software
obsInfo.Name = "observations";
obsInfo.Description = "integrated error, error," + ... 
    " and measured height";

% Action info
actInfo = rlNumericSpec([1 1]);
actInfo.Name = "flow";

env = rlSimulinkEnv(mdl, agentPath, obsInfo, actInfo);

env.ResetFcn = @localResetFcn;

Ts = 1.0; % Sample time
Tf = 200; % Final time

%% Creating critic
% Observation path
obsPath = featureInputLayer(obsInfo.Dimension(1), ...
    Name="obsInLyr");

% Action path
actPath = featureInputLayer(actInfo.Dimension(1), ...
    Name="actInLyr");

% Common path
commonPath = [
    concatenationLayer(1,2,Name="concat") % 1번째 차원으로 2개의 입력을 합침
    fullyConnectedLayer(50) % 뉴런 수를 50개 정도로 늘리면 학습이 더 잘 됩니다
    reluLayer() % 현실적 판단?
    fullyConnectedLayer(50)
    reluLayer()
    fullyConnectedLayer(1,Name="QValue") % Q-value 사용
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
plot(criticNet)


rng(0,"twister");
criticNet = initialize(criticNet);
summary(criticNet)

% Create critic obj
critic = rlQValueFunction(criticNet, ...
    obsInfo,actInfo, ...
    ObservationInputNames="obsInLyr", ...
    ActionInputNames="actInLyr");

getValue(critic, ...
    {rand(obsInfo.Dimension)}, ...
    {rand(actInfo.Dimension)})

% Creating actor
actorNet = [
    featureInputLayer(obsInfo.Dimension(1))
    fullyConnectedLayer(50)
    reluLayer()
    fullyConnectedLayer(50)
    reluLayer()
    fullyConnectedLayer(actInfo.Dimension(1))
    ];

rng(0,"twister");
actorNet = dlnetwork(actorNet);
summary(actorNet)
plot(actorNet)

actor = rlContinuousDeterministicActor(actorNet,obsInfo,actInfo);
getAction(actor,{rand(obsInfo.Dimension)})

%% Creating DDPG(Deep Deterministic Policy Gradient) agent
agent = rlDDPGAgent(actor,critic);

agent.AgentOptions.SampleTime = Ts;
agent.AgentOptions.DiscountFactor = 1.0;
agent.AgentOptions.MiniBatchSize = 400;
agent.AgentOptions.ExperienceBufferLength = 1e6;

actorOpts = rlOptimizerOptions( ...
    LearnRate=1e-4, ...
    GradientThreshold=1);
criticOpts = rlOptimizerOptions( ...
    LearnRate=1e-3, ...
    GradientThreshold=1);
agent.AgentOptions.ActorOptimizerOptions = actorOpts;
agent.AgentOptions.CriticOptimizerOptions = criticOpts;

% Noise model
agent.AgentOptions.NoiseOptions.StandardDeviation = 0.3;
agent.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-5;
getAction(agent,{rand(obsInfo.Dimension)})

% %% Training agent
% % training options
% trainOpts = rlTrainingOptions(...
%     MaxEpisodes=5000, ...
%     MaxStepsPerEpisode=ceil(Tf/Ts), ...
%     Plots="training-progress", ...
%     Verbose=false, ...
%     StopTrainingCriteria="EvaluationStatistic", ...
%     StopTrainingValue=2000 ...
%     );
% 
% % agent evaluator
% evl = rlEvaluator(EvaluationFrequency=10,NumEpisodes=5);
% 
% rng(0,"twister");
% 
% doTraining = false;
% % Start the training process if doTraining is true
% if doTraining
%     % Train the agent.
%     trainingStats = train(agent, env, trainOpts, Evaluator=evl);
%     % save("initialAgent1.mat","agent");
% else
%     % Load the pretrained agent for the example.
%     load("WaterTankDDPG.mat", "agent");
% end
% 
% %% Validation training agent
% rng(0,"twister");
% 
% simOpts = rlSimulationOptions( ...
%     MaxSteps=ceil(Tf/Ts), ...
%     StopOnError="on");
% experiences = sim(env,agent,simOpts);
% 
% rng(previousRngState);
% 
%% Local reset function
function in = localResetFcn(in)

    % Randomize reference signal
    blk = sprintf("WaterTank/Desired \nWater Level");
    h = 3*randn + 10;
    while h <= 0 || h >= 20
        h = 3*randn + 10;
    end
    in = setBlockParameter(in,blk,Value=num2str(h));

    % Randomize initial height
    h = 3*randn + 10;
    while h <= 0 || h >= 20
        h = 3*randn + 10;
    end
    blk = "WaterTank/Water-Tan k System/H";
    in = setBlockParameter(in,blk,InitialCondition=num2str(h));

end