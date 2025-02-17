function res = check_fc(connectionsinstance, comtype)
    res = 0;
    instance = connectionsinstance;
        for s = 1:length(instance)
            if comtype(instance(s)) == 'Battery'
                res = 1;
                break
            end
        end
end