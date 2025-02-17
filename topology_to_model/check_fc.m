function res = check_fc(connectionsinstance, comtype)
    res = 0;
    instance = connectionsinstance;
        for s = 1:length(instance)
            if comtype(instance(s)) == 'Fuel cell'
                res = 1;
                break
            end
        end
end