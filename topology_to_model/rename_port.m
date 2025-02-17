function rename_port(block,i,class)
    if class == 'GearSystems'
        for j = 1:3
            var_name = get_param([block '/To Workspace' num2str(j)], 'VariableName');
            set_param([block '/To Workspace' num2str(j)], 'VariableName', [var_name '_' num2str(i)]);
        end
end