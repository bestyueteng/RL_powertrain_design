function blockname = findBlock(componentStruct, componentID)
    % Initialize outputs
    type_name = '';
    idx = [];

    % Loop through each field in the struct
    fields = fieldnames(componentStruct);
    for i = 1:length(fields)
        fieldName = fields{i};
        values = componentStruct.(fieldName);
        
        % Check if the componentID exists in the field's values
        if ismember(componentID, values)
            type_name = fieldName; % Assign the field name as the type
            idx = find(values == componentID); % Find the index within the array
            break; % Exit the function as we found the component
        end
    end

    % If no match is found, return an error message
    if isempty(type_name)
        error('Component ID %d not found in the given struct.', componentID);
    end
    
    blockname = [type_name '_' num2str(idx)];
end