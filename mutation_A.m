for cand_count =5% 1:size(cand_group,1)
    %% 指定候选组
    load 1.mat
    other_car = 2;
    for j = 1:carnum
        carname = sprintf('A%d',j);
        cardata = eval(carname);
        if j == cand_group(cand_count,1)
            B0 = cardata;
        elseif j == cand_group(cand_count,2)
            B1 = cardata;
        else
            if other_car <= carnum-1 
                carname_new = sprintf('B%d',other_car);
                eval([carname_new ' = cardata;']);
                other_car = other_car + 1;
            end
        end
    end
    clear carname cardata carname_new other_car
    %% 计算场景整体期望速度
    for j = 1:carnum-1
        cardata = eval(sprintf('B%d',j));
        exp_spd_list(j,1) = mean(cardata(:,3));
    end
    clear cardata
    exp_spd = mean(exp_spd_list);
    %% 初始位置对齐
    simu_time = size(B0,1) * 0.04;
    exp_length = simu_time * exp_spd;
    real_length = abs(B0(1,1)-target(1,1));
    gap = real_length - exp_length;%分正负
    clear exp_length real_length exp_spd_list exp_spd
    if direct == 0
        for j = 1:carnum
            carname = sprintf('B%d',j-1);
            cardata = eval(carname);
            cardata(:,1) = cardata(:,1) + gap;
            eval([carname ' = cardata;']);
        end
    elseif direct == 1
        for j = 1:carnum
            carname = sprintf('B%d',j-1);
            cardata = eval(carname);
            cardata(:,1) = cardata(:,1) - gap;
            eval([carname ' = cardata;']);
        end
    end
    clear simu_time exp_length real_length gap carname cardata
    %% 车型变异
    B1(:,6) = 2.55;
    B1(:,7) = 20;
    %% 候选车辆初始位置变异
    B1gap = B1(1,1)-B0(1,1);
    if direct == 0
        if task_anal(1,cand_group(cand_count,1)) == task_anal(1,cand_group(cand_count,2))
            B1(:,1) = B1(:,1) + B1gap + 20;
        else 
            B1(:,1) = B1(:,1) + B1gap;
        end
    else
        if task_anal(1,cand_group(cand_count,1)) == task_anal(1,cand_group(cand_count,2))
            B1(:,1) = B1(:,1) - B1gap - 20;
        else 
            B1(:,1) = B1(:,1) - B1gap;
        end        
    end
    clear B1gap
    %% 删除初始位置重叠车辆
    B0_edge_ini = getEdgePoints([B0(1,1),B0(1,2)],B0(1,7),B0(1,6),B0(1,5));
    delete_car_count = 0;
    delete_car_list = [];
    overlap = 0;
    for j = 1:carnum-1
        carname = sprintf('B%d',j);
        cardata = eval(carname);
        car_edge = getEdgePoints([cardata(1,1),cardata(1,2)],cardata(1,7),cardata(1,6),cardata(1,5));
        min_dis = calculateMinDistance(B0_edge_ini,car_edge);
        if min_dis <= 0.2
            if j ~= 1
                eval(['clear B' num2str(j)])
                delete_car_list(1,delete_car_count+1) = j;
                delete_car_count = delete_car_count + 1;    
            else
                overlap = 1;
                break
            end
        end
    end
    clear carname cardata B0_edge_ini car_edge min_dis
    %% 车辆重新排序
    if overlap ~= 1
        C0 = B0;
        C1 = B1;
        carname_new_count = 2;
        for j = 2:carnum-1
            if ~ismember(j,delete_car_list)
                carname = sprintf('B%d',j);
                cardata = eval(carname);
                carname_new = sprintf('C%d',carname_new_count);
                eval([carname_new ' = cardata;']);
                carname_new_count = carname_new_count + 1;
            end
        end
        for j = 1:carnum
            eval(['clear B' num2str(j-1)]);
        end
        carnum_ini = carnum;
        carnum = carnum - delete_car_count;
        clear delete_car_count carname_new_count carname cardata delete_car_list carname_new
    %% 剔除NPC碰撞场景
        find_crash = 0;
%     keshihuaB;
        for j = 1:carnum-1
            maincarname = sprintf('C%d',j);
            maincardata = eval(maincarname);
            for k = 1:carnum-1
                if j ~= k 
                    carname = sprintf('C%d',k);
                    cardata = eval(carname);
                    for l = 1:size(ego,1)
                        maincar_edge = getEdgePoints([maincardata(l,1),maincardata(l,2)],maincardata(1,7),maincardata(1,6),maincardata(l,5));
                        car_edge = getEdgePoints([cardata(l,1),cardata(l,2)],cardata(1,7),cardata(1,6),cardata(l,5));
                        min_dis = calculateMinDistance(maincar_edge,car_edge);
                        if min_dis <= 0.2
                            disp(['组合' ,num2str([cand_group(cand_count,1),cand_group(cand_count,2)]),'碰撞'])
                            crash_point = [j,k,l];
                            find_crash = 1;
                            break
                        end
                    end
                    if find_crash == 1
                        break
                    end
                end
                if find_crash == 1
                    break
                end
            end
            if find_crash == 1
                break
            end
        end
        clear maincarname maincardata carname cardata maincar_edge car_edge min_dis 
%         if find_crash == 0
%             disp(['组合' ,num2str([cand_group(cand_count,1),cand_group(cand_count,2)]),'有效'])
%             for j = 1:carnum_ini
%                 eval(['clear A' num2str(j)]);
%             end
%             ego = C0;
%             for j = 1:carnum-1
%                 cardata = eval(sprintf('C%d',j));
%                 carname_new = sprintf('A%d',j);
%                 eval([carname_new ' = cardata;']);
%             end
%             dirname = "F:\ONSITE\sce_3_2\" + sce_file_dir{sc_count} + '_' + num2str(cand_count);
%             mkdir(dirname);
%             xodr = "F:\ONSITE\sce_3_1\"+sce_file_dir{sc_count}+'\'+sce_file_dir{sc_count}+'.xodr';
%             copyfile(xodr,dirname,"f");
%             xodr = dirname + '\' + sce_file_dir{sc_count} + '.xodr';
%             export_xosc;
%         end
    else
        disp(['组合' ,num2str([cand_group(cand_count,1),cand_group(cand_count,2)]),'初始位置重叠']);
    end
%     clearvars -except cand_count sce_file_dir sc_count  alltime
end