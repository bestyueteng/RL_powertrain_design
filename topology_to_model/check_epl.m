function res = check_epl(connectionsinstance, comtype)
    res = 0;
    instance = connectionsinstance;
        for s = 1:length(instance)
            if comtype(instance(s)) == 'Electric Power Link'
                res = 1;
                break
            end
        end
end