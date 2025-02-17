function result = summarizeComponents(component_library, component_class, component_type)
    % sumaarize the component library. Know which component belows to which
    % type

    % Initialize result structure
    result = struct();
    count = 0;
    total_num = 1;
    
    % Iterate through grouped data to populate the result
    for i = 1:length(component_type)
        subclass_type = {};
        subclass = component_type{i};
        % class_name = component_class{i};
        % subclass_type = [subclass_type];
        if ischar(subclass)
            subclass_type{end+1} = subclass;
        else
            for j = 1:length(subclass)
                subclass_type{end+1} = subclass(j);
            end
        end
  
        for j = 1:length(subclass_type)
            count = count + 1;
            type_name = strrep(subclass_type{j}, ' ', '');
            type_name = string(type_name);
            num = component_library.NumberOfInstances(count);
            total_instances = total_num : total_num+num-1; 
            result.(type_name) = total_instances;

            total_num = total_num+num;
        end
    end

end
