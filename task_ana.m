a = sprintf('A%d',carnum);
eval([a '= ego;']);
clear a line_mat task_anal 
%% 场景驾驶方向分析
if ego(1,1) < target(1,1)
    direct = 0;%正向为0，从小至大
else
    direct = 1;%反向为1
end
%% 构造驾驶任务分析矩阵
task_mat =zeros(length(A1),6*carnum);
for i =1:carnum
    data = eval(sprintf('A%d',i));
    for j = 1:length(data)
        if isin(data(j,2),road_line(2),road_line(3))
                task_mat(j,6*i-5) = i;
        elseif isin(data(j,2),road_line(4),road_line(5))
                task_mat(j,6*i-4) = i;
        elseif isin(data(j,2),road_line(6),road_line(7))
                task_mat(j,6*i-3) = i;
        elseif isin(data(j,2),road_line(10),road_line(11))
                task_mat(j,6*i-2) = i;
        elseif isin(data(j,2),road_line(12),road_line(13))
                task_mat(j,6*i-1) = i;
        elseif isin(data(j,2),road_line(14),road_line(15))
                task_mat(j,6*i) = i;        
        end
    end
end
[~, unique_rows] = unique(task_mat, 'rows', 'stable');
task_mat = task_mat(unique_rows, :);
task_mat = [task_mat unique_rows];
for i = 1:size(task_mat, 1)
    row = task_mat(i, 1:end-1);
    for j = 1:carnum
        [~, index] = find(row == j);
        if ~isempty(index)
            task_anal(i,j) = index(1);
        else
            task_anal(i,j) = -1; 
        end
    end
end
task_anal = [task_anal unique_rows];
isNegativeOne = task_anal == -1;
rowsWithNegativeOne = any(isNegativeOne, 2);
task_anal = task_anal(~rowsWithNegativeOne, :);
for i = 1:size(task_anal,1)
    for j = 1:size(task_anal,2)-1
        if mod(task_anal(i,j),6) == 0
            task_anal(i,j) = 6;
        else
            task_anal(i,j) = mod(task_anal(i,j),6);
        end
    end
end
if isempty(task_anal) ~= 1
    task_anal_1 = task_anal(1,1:end-1)';
    task_ana_2 = task_anal;
    for i = 1:size(task_anal,1)-1
        for j = 1:size(task_anal,2)-1
            task_anal_1(j,i+1) = task_anal(i,j) - task_anal(i+1,j);
            if task_anal(i,j) - task_anal(i+1,j) ~= 0
                task_anal_1(carnum+1,i+1) = task_anal(i+1,end);
            end
        end
    end
    task_anal = task_anal_1;
    task_anal = task_anal';
    clear  line_mat row data task_anal_1 index isNegativeOne rowsWithNegativeOne task_mat unique_rows
%% 车辆活跃度与驾驶任务相似度计算
    alpha = zeros(carnum,carnum);
    for i = 1:carnum
        for j = 1:carnum
            if i ~= j
                alpha(i,j) = 100/(carnum-1);
            end
        end
    end
    %活跃度计算
    for i = 1:carnum
        for j = 2:size(task_anal,1)%驾驶任务矩阵的行
            for k = 1:size(task_anal,2)-1
                if abs(task_anal(j,k)) == 1 && i ~= k
                    alpha(i,k) = alpha(i,k) + carnum-2;
                    for l = 1:carnum
                        if l ~= k && l ~= i && i ~= k
                            alpha(i,l) = alpha(i,l) -1;
                        end
                    end
                end
            end
        end
    end
    %驾驶任务相似度计算
    for i =1:carnum
        for j = 1:size(task_ana_2,1)
            for k = 1:carnum
                if j == 1 && task_ana_2(j,k) == task_ana_2(j,i) && i ~= k
                    alpha(i,k) = alpha(i,k) + 0.8*(carnum-2);
                    for l = 1:carnum
                        if l ~= k && l ~= i && i ~= k
                            alpha(i,l) = alpha(i,l) -0.8;
                        end
                    end
                elseif task_ana_2(j,k) == task_ana_2(j,i) && i ~= k && task_ana_2(j,k) ~= task_ana_2(j-1,k)
                    alpha(i,k) = alpha(i,k) + 0.8*(carnum-2);
                    for l = 1:carnum
                        if l ~= k && l ~= i && i ~= k
                            alpha(i,l) = alpha(i,l) -0.8;
                        end
                    end
                end
            end
        end
    end
%% 确认主车与变异车辆候选
    [row, col] = find(alpha == max(max(alpha)));
    row(:,2) = col;
    clear col
    cand_group = row;
    clear row pick col ego_line
end
if isempty(task_anal) == 1
    sc_invalid = 1;
    disp('task_anal is empty')
end