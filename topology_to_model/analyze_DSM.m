function [comclass, comtype, comconnectionsclass, comconnectionsinstance, Component_struct] = analyze_DSM(DSM, component_library,component_class,component_type)

    matrix = DSM;
    [numComponents, ~] = size(DSM);
    comclass = string(zeros(numComponents,1));
    comtype = string(zeros(numComponents,1));

    numInstance = length(component_library.NumberOfInstances);
    
    % Get the amount of all components of different types and classes
    Component_struct = summarizeComponents(component_library, component_class, component_type);
    
    sum = 0;
    for i = 1:numInstance
        for j = 1:numComponents
            if j <= component_library.NumberOfInstances(i)
                sum = sum+1;
                if ~all(DSM(sum, :) == 0)
                    comclass(sum) = component_class(component_library.ComponentNumber(i));
                   
                    subclass = component_type{component_library.ComponentNumber(i)};
                    comtype(sum) = subclass(component_library.TypeOfComponent(i));
                else
                    comclass(sum) = 'None';
                    comtype(sum) = 'None';
                end
            end
        end
        if i~= numInstance
            % component_library.NumberOfInstances(i)
            matrix(1:component_library.NumberOfInstances(i),:)=[];
        end
        
    end
    
    % Get the connections
    comconnectionsclass = cell(1, numComponents);
    comconnectionstype = cell(1, numComponents);
    comconnectionsinstance = cell(1, numComponents);

    for i = 1: numComponents
        subclass = string([]);
        subclasstype = string([]);
        subclassinstance = [];
        for j = 1: numComponents
            if DSM (i,j) == 1
                subclass(end+1) = comclass(j);
                subclasstype(end+1) = comtype(j);
                subclassinstance(end+1) = j;
            end 
        end
        comconnectionsclass{i} = subclass;
        comconnectionstype{i} = subclasstype;
        comconnectionsinstance{i} = subclassinstance;
    end

end