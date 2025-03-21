% Transmission: single gear * 1
%               multi gear * 3 (two motor, one generator)
% Motor:        Electric generator * 1
%               Electric motor * 2 (one B, one FC)
%               Fuel cell * 1
% Storage:      Battery * 1
%               Tank * 1

% ComponentNumber = [1, 1, 2, 2, 2, 3, 3]; % 1: Transmission; 2: Motor; 3: Storage
% TypeOfComponent = [1, 2, 1, 2, 3, 1, 2]; % [single gear, multi gear]; [Generator, Motor, Fuel cell], [Battery, Tank] 
% NumberOfInstances = [1, 3, 1, 2, 1, 1, 1];
% model_scale = {1, 3, 2, 3, 0, 2, 1, 100, 70, 0};
% 
% component_library = table(ComponentNumber, TypeOfComponent, NumberOfInstances);
% 
% % Define your DSM here (this is just an example structure)
% % [SG MG(B) MG(FC) MG(G) EG EM(B) EM(FC) FC B T]
% DSM = [
%     0, 1, 1, 0, 0, 0, 0, 0, 0, 0;
%     1, 0, 0, 0, 0, 1, 0, 0, 0, 0;
%     1, 0, 0, 0, 0, 0, 1, 0, 0, 0;
%     0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%     0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%     0, 1, 0, 0, 0, 0, 0, 0, 1, 0;
%     0, 0, 1, 0, 0, 0, 0, 1, 0, 0;
%     0, 0, 0, 0, 0, 0, 1, 0, 0, 1;
%     0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
%     0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
% ];


function total_cost = evaluation(ComponentNumber,TypeOfComponent,NumberOfInstances,DSM,component_class,component_type,performance_req,optimizing_method, opt_tol)
    component_library = table(ComponentNumber, TypeOfComponent, NumberOfInstances); % For Python
    warning('on','all');
    clc;

    % try
    
        % Initialize Simulink model
        modelName = 'VehiclePowertrain';
        open_system(new_system(modelName));
        set_param(modelName, 'Solver', 'FixedStepDiscrete');
        
        % Create blocks based on DSM
        % [numComponents, ~] = size(DSM);
        [comclass, comtype, comconnectionstype, comconnectionsinstance, Component_struct] = analyze_DSM(DSM, component_library,component_class,component_type);
        assignin('base','comconnectionstype',comconnectionstype);
        assignin('base','comconnectionsinstance',comconnectionsinstance);
        assignin('base','Component_struct',Component_struct);
        assignin('base','comclass',comclass);
        assignin('base','comtype',comtype);
        generate_blocks(comclass, comtype, modelName, DSM, comconnectionstype, comconnectionsinstance, component_type,Component_struct);
        % create_blocks(comclass, comtype, comconnectionstype, comconnectionsinstance, modelName);
        create_connections(comclass, comtype, Component_struct, comconnectionsinstance, modelName, DSM, comconnectionstype);
        scaling(modelName, comtype, DSM, Component_struct); % Only for nr of gears （fixed)
        func = @(x)control_optimization(x,comtype,DSM,modelName,Component_struct,performance_req);
        x_init = set_default_value(DSM,comtype,optimizing_method);
        switch optimizing_method
            case 'fminsearch'
                % [V_result,results] = control_optimization(x_init,comtype,DSM,model_scale,modelName);
                disp('fminsearch');
                options = optimset('TolFun',opt_tol);
                % [V_result,results] = evalc('fminsearch(func,x_init,options);');
                x_opt = fminsearch(func,x_init,options);
                assignin('base','x_opt',x_opt);
                total_cost = control_optimization(x_opt,comtype,DSM,modelName,Component_struct,performance_req);
                assignin('base','total_cost',total_cost);
            case 'pso'
                lb = x_init{1};
                ub = x_init{2};
                options = optimoptions('particleswarm','SwarmSize',100,'FunctionTolerance',opt_tol, 'HybridFcn',@fmincon);
                x_opt = particleswarm(func,length(lb),lb,ub,options);
                total_cost = control_optimization(x_opt,comtype,DSM,modelName,Component_struct,performance_req);
            otherwise
                V_result = control_optimization(x_init,comtype,DSM,modelName,Component_struct,performance_req);
                total_cost = V_result;
                disp(performance_req)
        end

        % total_cost = control_optimization(V_result,comtype,DSM,modelName,Component_struct);

    % catch ME
    %     total_cost = 1e6;
    % end
end