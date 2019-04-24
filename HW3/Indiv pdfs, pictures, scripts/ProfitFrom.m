function profit = ProfitFrom(machineState, weekNumber)
    if weekNumber == 4
        if machineState == 1
            profit = 100;
        else
            profit = 0;
        end
    else
        if machineState == 1
            doMaintenance = -20 + 0.4*ProfitFrom(0, weekNumber+1) +...
                0.6*ProfitFrom(1,weekNumber+1);
            dontDoMaintenance = 0.7*ProfitFrom(0, weekNumber+1) +...
                0.3*ProfitFrom(1,weekNumber+1);
%             if doMaintenance > dontDoMaintenance
%                 fprintf('At end of week %d, do maintenance \n',weekNumber);
%             else
%                 fprintf('At end of week %d, don''t do maintenance \n',weekNumber);
%             end
            profit = (100+max(doMaintenance,dontDoMaintenance));
        else
            repair = -40 + 0.4*ProfitFrom(0, weekNumber+1) +...
                0.6*ProfitFrom(1,weekNumber+1);
            replace = -150 + ProfitFrom(1,weekNumber+1);
%             if repair > replace
%                 fprintf('At end of week %d, repair \n',weekNumber);
%             else
%                 fprintf('At end of week %d, replace \n',weekNumber);
%             end
            profit = (max(repair,replace));
        end
    end
%     fprintf('At end of week %d, for machine state %d, profit is %.2f \n',weekNumber, machineState, profit);

end