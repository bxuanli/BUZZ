clear
%% 指定场景文件夹****文件名在最后
path = 'F:/ONSITE/hu/';
sce_file_dir = {dir([path 'inputs']).name};
sce_file_dir = sce_file_dir(1,3:end);
sce_file_dir = sce_file_dir';
final_score = {'Scenario','Safety','Efficiency','Comfort','Total'};
invalid_count = 0;
invalid_sc = [];
alltime =0;
%% 导入场景数据
for sc_count = 1:length(sce_file_dir)
    tic
    csv = [path 'outputs/' sce_file_dir{sc_count} '_result.csv'];
    findzifu;
    xodr_name = sce_file_dir{sc_count}(1:end-numCharsAfterUnderscore-1);
    xosc = [path 'inputs/' sce_file_dir{sc_count}  '/' xodr_name '_C.xosc'];
    xodr = [path 'inputs/' sce_file_dir{sc_count} '/' xodr_name '.xodr'];
    scenario_load_2;
    clear csv xosc xodr ans fileList xodr_name
    if ~isempty(ego)
        score_safe = 50;
        score_effi = 0;
        score_conf = 20;
        if ego(1,1) < target(1,1)
            direct = 0;%正向为0，从小至大
        else
            direct = 1;%反向为1
        end
    %% 安全得分    
    % 驶出行车道
        driveout_time = 0;
        if isin(ego(1,2),road_line(2),road_line(7)) %下半区
            for i = 1:size(ego,1)
                if ego(i,2) <= road_line(2) || ego(i,2) >= road_line(7)
                    driveout_time = driveout_time + 1;
                end
            end
        elseif isin(ego(1,2),road_line(10),road_line(15))%上半区
            for i = 1:size(ego,1)
                if ego(i,2) <= road_line(10) || ego(i,2) >= road_line(15)
                    driveout_time = driveout_time + 1;
                end
            end
        end
        driveout_score = driveout_time/size(ego,1)*50;
        score_safe = score_safe - driveout_score;
        clear driveout_score driveout_time
    % 驶入对向车道
        driveoppo_time = 0;
        if isin(ego(1,2),road_line(2),road_line(7)) %下半区
            for i = 1:size(ego,1)
                if ego(i,2) >= road_line(10) && ego(i,2) <= road_line(15)
                    driveoppo_time = driveoppo_time + 1;
                end
            end
        elseif isin(ego(1,2),road_line(10),road_line(15))%上半区
            for i = 1:size(ego,1)
                if ego(i,2) >= road_line(2) && ego(i,2) <= road_line(7)
                    driveoppo_time = driveoppo_time + 1;
                end
            end
        end
        driveoppo_score = driveoppo_time/size(ego,1)*25;
        score_safe = score_safe - driveoppo_score;
        clear driveoppo_score driveoppo_time
    % TTC
        ttc = 2;
        tet = 0;
        for i = 1:carnum-1
            obj_car_data = eval(sprintf('A%d',i));
            for j = 1:size(ego,1)
                ttc = 2;
                if direct == 0 && abs(ego(j,2) - obj_car_data(j,2)) <= 1 && ego(j,1) < obj_car_data(j,1)
                    obj_car_tail = obj_car_data(j,1) - 0.5*obj_car_data(1,7) ;
                    ego_head = ego(j,1) + 0.5*ego(1,7);
                    ttc = abs(obj_car_tail - ego_head)/(abs(ego(j,3)-obj_car_data(j,3)));
                elseif direct == 1 && abs(ego(j,2) - obj_car_data(j,2)) <= 1 && ego(j,1) > obj_car_data(j,1)
                    obj_car_tail = obj_car_data(j,1) + 0.5*obj_car_data(1,7) ;
                    ego_head = ego(j,1) - 0.5*ego(1,7);
                    ttc = abs(obj_car_tail - ego_head)/(abs(ego(j,3)-obj_car_data(j,3)));
                end
                if ttc <= 1
                    tet = tet + 1;
                end
            end
            clear obj_car_data 
        end
        ttc_score = tet/size(ego,1)*50;
        score_safe = score_safe - ttc_score;
        clear ttc tet ttc_list ttc_score score_ttc
    % 碰撞检测
        crash = 0;
        for i = 1:carnum-1
            obj_car_data = eval(sprintf('A%d',i));
            for j = 1:size(ego,1)
                ego_edge = getEdgePoints([ego(j,1),ego(j,2)],ego(1,7),ego(1,6),ego(j,5));
                obj_edge = getEdgePoints([obj_car_data(j,1),obj_car_data(j,2)],obj_car_data(1,7),obj_car_data(1,6),obj_car_data(j,5));
                min_dis = calculateMinDistance(ego_edge,obj_edge);
                if min_dis <= 0.2
                    score_safe = 0;
                    crash = 1;
                    mark = [i,j];
                end
            end
        end
        clear obj_car_data ego_edge obj_edge min_dis
    %% 效率得分
    %任务完成
        for i = 1:size(ego,1)
            if isin(ego(i,1),target(1,1),target(1,2)) && isin(ego(i,2),target(2,1),target(2,2)) 
                score_effi = score_effi + 15;
                break
            end
        end
    %任务耗时
        time_exp = abs(ego(1,1)-ego(end,1))/33.333;
        time_real = size(ego,1) * 0.04;
        score_effi_add = time_exp/time_real*15;
        if score_effi_add >15
            score_effi_add = 15;
        end
        score_effi = score_effi + score_effi_add;
        clear time_exp time_real score_effi_add
    %% 舒适得分
        acc_exc_time = 0;
        longi_jerk_exc_time = 0;
        late_acc_exc_time = 0;
        late_jerk_exc_time = 0;
        longi_jerk = diff(ego(:,4))/0.04;
        late_spd = diff(ego(:,2))/0.04;
        late_acc = diff(late_spd)/0.04;
        late_jerk = diff(late_acc)/0.04;
        for i = 1:size(ego,1)
            if abs(ego(i,4)) > 4
                acc_exc_time = acc_exc_time + 1;
            end
        end
        for i = 1:size(ego,1)-1
            if abs(longi_jerk(i)) > 6
                longi_jerk_exc_time = longi_jerk_exc_time + 1;
            end
        end
        for i = 1:size(ego,1)-2
            if abs(late_acc(i)) > 0.5
                late_acc_exc_time = late_acc_exc_time + 1;
            end
        end
        for i = 1:size(ego,1)-3
            if abs(late_jerk(i)) > 1
                late_jerk_exc_time = late_jerk_exc_time + 1;
            end
        end
        deduct_conf = (acc_exc_time + longi_jerk_exc_time + late_acc_exc_time + late_jerk_exc_time)/size(ego,1)*5;
        score_conf = score_conf - deduct_conf;
        clear acc_exc_time longi_jerk_exc_time late_acc_exc_time late_jerk_exc_time longi_jerk late_spd late_acc late_jerk deduct_conf
    %% 得分写入
        final_score{sc_count+1,1} = sce_file_dir{sc_count};
        final_score{sc_count+1,2} = score_safe;
        final_score{sc_count+1,3} = score_effi;
        final_score{sc_count+1,4} = score_conf;
        total = score_safe + score_effi + score_conf;
        final_score{sc_count+1,5} = total;
    else
        invalid_sc{invalid_count+1,1} = sce_file_dir{sc_count};
        invalid_count = invalid_count + 1;
        disp('无效场景');
    end
%% 终端输出
elapsedTime = toc;
alltime = alltime + elapsedTime;
disp([sce_file_dir{sc_count} ,'第',num2str(sc_count),'个，已评价'])
fprintf('耗时: %f 秒\n', elapsedTime);
clearvars -except sce_file_dir sc_count final_score invalid_count invalid_sc alltime path
end
final_score{end+1,1} = 'Mean';
for i = 2:5
    final_score{end,i} = mean(cell2mat(final_score(2:end,i)));
end
writecell(final_score,'hu67.csv');
% save 1.mat final_score

