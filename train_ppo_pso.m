% --- Train_PPO_PSO.m ---
function trainingStats = train_ppo_pso(targetFun,nvars,lb,ub,maxIter)
    nPop = 30;
    env = rl_env_pso(targetFun, nvars, lb, ub, nPop, maxIter);
    obsInfo = getObservationInfo(env);
    actInfo = getActionInfo(env);
    
    % Create Actor and Critic (Simple MLP)
    network = [
    featureInputLayer(obsInfo.Dimension(1),"Name","state")
    fullyConnectedLayer(64,"Name","fc1")
    reluLayer("Name","relu1")
    fullyConnectedLayer(64,"Name","fc2")
    reluLayer("Name","relu2")
];
    
    % --- Actor network (mean + std) ---
    statePath = [
        featureInputLayer(obsInfo.Dimension(1),"Name","state")
        fullyConnectedLayer(64,"Name","fc1")
        reluLayer("Name","relu1")
        fullyConnectedLayer(64,"Name","fc2")
        reluLayer("Name","relu2")
    ];
    
    meanPath = fullyConnectedLayer(actInfo.Dimension(1),"Name","mean");
    stdPath  = [
        fullyConnectedLayer(actInfo.Dimension(1),"Name","stdFC")
        softplusLayer("Name","std")
    ];
    
    lg = layerGraph(statePath);
    lg = addLayers(lg, meanPath);
    lg = addLayers(lg, stdPath);
    lg = connectLayers(lg,"relu2","mean");
    lg = connectLayers(lg,"relu2","stdFC");

    criticNet = [
        network
        fullyConnectedLayer(1,"Name","criticOut")
    ];
    
    actor = rlContinuousGaussianActor( ...
        lg, obsInfo, actInfo, ...
        "ActionMeanOutputNames","mean", ...
        "ActionStandardDeviationOutputNames","std" ...
    );
    critic = rlValueFunction(criticNet, obsInfo);
    
    % PPO Options
    agentOpts = rlPPOAgentOptions('SampleTime', 1, 'ExperienceHorizon', 256, 'ClipFactor', 0.2);
    agent = rlPPOAgent(actor, critic, agentOpts);
    
    % Training Options
    trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 50, ...
    'MaxStepsPerEpisode', maxIter, ...
    'ScoreAveragingWindowLength', 10, ...
    'Plots', 'training-progress', ...
    'Verbose', true);

    trainingStats = train(agent, env, trainOpts);
    save('trainedPPO_PSO.mat', 'agent');
end