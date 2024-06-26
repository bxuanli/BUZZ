A100(1,1) = ego(1,1);
A100(1,3:7) = ego(1,3:7);
if task_anal(1,1) == 5
    if task_anal(1,carnum) == 6
        A100(1,2) = mean(road_line(10:11));
    elseif task_anal(1,carnum) == 4
        A100(1,2) = mean(road_line(14:15));
    end
elseif task_anal(1,1) == 2
    if task_anal(1,carnum) == 3
        A100(1,2) = mean(road_line(2:3));
    elseif task_anal(1,carnum) == 1
        A100(1,2) = mean(road_line(6:7));
    end
end
clear ego 
ego = A100;
clear A100
carnum = carnum+1;
%% 剔除初始位置重叠场景
ego_edge = getEdgePoints([ego(1,1),ego(1,2)],ego(1,7),ego(1,6),ego(1,5));
for i = 1:carnum-1
    cardata = eval(sprintf('A%d',i));
    car_edge = getEdgePoints([cardata(1,1),cardata(1,2)],cardata(1,7),cardata(1,6),cardata(1,5));
    min_dis = calculateMinDistance(ego_edge,car_edge);
    if min_dis < 0.2
        overlap = 1;
        disp('初始位置重叠')
        break
    end
end