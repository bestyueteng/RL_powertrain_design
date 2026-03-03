function env = rl_env_pso(CostFcn, nVar, VarMin, VarMax, nPop, MaxIt)

obsInfo = rlNumericSpec([4 1], ...
    "LowerLimit", [-inf; 0; 0; -inf], ...
    "UpperLimit", [ inf; inf; 1;  inf]);
obsInfo.Name = "PSOStats";

actInfo = rlNumericSpec([3 1], ...
    "LowerLimit", [0.1; 0.0; 0.0], ...
    "UpperLimit", [1.2; 3.0; 3.0]);
actInfo.Name = "PSOParams";

envData = struct();
envData.CostFcn = CostFcn;
envData.nVar   = nVar;
envData.VarMin = VarMin;
envData.VarMax = VarMax;
envData.nPop   = nPop;
envData.MaxIt  = MaxIt;

% Velocity bounds
envData.VMax = 0.2 .* (VarMax - VarMin);
envData.VMin = -envData.VMax;

% ✅ ResetFcn must be zero-input
ResetFcn = @() resetFcn(envData);

% StepFcn signature must be (Action, LoggedSignals)
StepFcn  = @(action, logged) stepFcn(action, logged, envData);

env = rlFunctionEnv(obsInfo, actInfo, StepFcn, ResetFcn);
end


function [obs, logged] = resetFcn(envData)

    rng("shuffle");
    
    nPop = envData.nPop;
    nVar = envData.nVar;
    
    % Expand bounds to 1xnVar if needed
    VarMin = expandBound(envData.VarMin, nVar);
    VarMax = expandBound(envData.VarMax, nVar);
    VMin   = expandBound(envData.VMin,   nVar);
    VMax   = expandBound(envData.VMax,   nVar);
    
    % Initialize particles
    particle(nPop) = struct();
    gbest.Cost = inf;
    gbest.Position = [];
    
    costs = zeros(nPop,1);
    
    for i = 1:nPop
        particle(i).Position = VarMin + rand(1,nVar).*(VarMax-VarMin);
        particle(i).Velocity = zeros(1,nVar);
    
        particle(i).Cost = envData.CostFcn(particle(i).Position);
    
        particle(i).Best.Position = particle(i).Position;
        particle(i).Best.Cost     = particle(i).Cost;
    
        costs(i) = particle(i).Cost;
    
        if particle(i).Best.Cost < gbest.Cost
            gbest = particle(i).Best;
        end
    end
    
    logged = struct();
    logged.particle = particle;
    logged.gbest = gbest;
    
    logged.it = 1;
    logged.prevBestCost = gbest.Cost;
    
    % Track episode start best for progress ratio
    logged.startBestCost = gbest.Cost;
    
    % Create initial observation
    obs = makeObs(costs, logged.gbest.Cost, logged.it, envData.MaxIt, logged.startBestCost);
end

function [nextObs, reward, isDone, logged] = stepFcn(action, logged, envData)
    % action = [w; c1; c2]
    
    % Clamp action to avoid NaNs / crazy values (extra safety)
    w  = clamp(action(1), 0.1, 1.2);
    c1 = clamp(action(2), 0.0, 3.0);
    c2 = clamp(action(3), 0.0, 3.0);
    
    nPop = envData.nPop;
    nVar = envData.nVar;
    
    VarMin = expandBound(envData.VarMin, nVar);
    VarMax = expandBound(envData.VarMax, nVar);
    VMin   = expandBound(envData.VMin,   nVar);
    VMax   = expandBound(envData.VMax,   nVar);
    
    particle = logged.particle;
    gbest    = logged.gbest;
    
    % PSO update (one iteration)
    costs = zeros(nPop,1);
    
    for i = 1:nPop
        r1 = rand(1,nVar);
        r2 = rand(1,nVar);
    
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*r1.*(particle(i).Best.Position - particle(i).Position) ...
            + c2*r2.*(gbest.Position - particle(i).Position);
    
        % Velocity limits
        particle(i).Velocity = max(min(particle(i).Velocity, VMax), VMin);
    
        % Position update
        particle(i).Position = particle(i).Position + particle(i).Velocity;
    
        % Apply bounds (hard clamp)
        particle(i).Position = max(min(particle(i).Position, VarMax), VarMin);
    
        % Evaluate
        particle(i).Cost = envData.CostFcn(particle(i).Position);
        costs(i) = particle(i).Cost;
    
        % Update personal best
        if particle(i).Cost < particle(i).Best.Cost
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost     = particle(i).Cost;
    
            % Update global best
            if particle(i).Best.Cost < gbest.Cost
                gbest = particle(i).Best;
            end
        end
    end
    disp(['The best cost is: ', num2str(gbest.Cost)]);
    % Reward = improvement in global best (positive if improved)
    prevBest = logged.prevBestCost;
    newBest  = gbest.Cost;
    
    reward = prevBest - newBest;          % >0 when best cost decreases
    % Optional shaping (uncomment if you want)
    % reward = 10*(prevBest - newBest) - 0.01*(w^2 + c1^2 + c2^2);
    
    % Update logged signals
    logged.particle = particle;
    logged.gbest = gbest;
    logged.prevBestCost = newBest;
    
    logged.it = logged.it + 1;
    
    % Termination
    isDone = logged.it > envData.MaxIt;
    
    % Observation
    nextObs = makeObs(costs, gbest.Cost, logged.it, envData.MaxIt, logged.startBestCost);
end

function obs = makeObs(costs, bestCost, it, MaxIt, ~)
    meanCost = mean(costs);
    stdCost  = std(costs);
    
    progressRatio = min(max(it / MaxIt, 0), 1);
    
    % If you prefer "progress of improvement", use this instead:
    % progressRatio = min(max((startBestCost - bestCost) / (abs(startBestCost)+eps), 0), 1);
    
    obs = [meanCost; stdCost; progressRatio; bestCost];
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
    end
    
    function b = expandBound(bound, nVar)
    if isscalar(bound)
        b = repmat(bound, 1, nVar);
    else
        b = reshape(bound, 1, []);
        if numel(b) ~= nVar
            error("Bound size must be scalar or 1xnVar.");
        end
    end
end

% Example cost function (Sphere)
% CostFcn = @(x) sum(x.^2);
% 
% nVar = 10;
% VarMin = -5;
% VarMax = 5;
% 
% nPop = 30;
% MaxIt = 50;
% 
% env = makeDPSOTuningEnv(CostFcn, nVar, VarMin, VarMax, nPop, MaxIt);



