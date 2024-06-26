clear
%% 指定场景文件夹
sc_path = 'scenarios';
all_sc = {dir(sc_path).name};
all_sc = all_sc(1,3:end);
all_sc = all_sc';
alltime = 0;
for sc_count = 1:length(all_sc)
    tic
    sc_invalid = 0;
    csv  = [sc_path '\' all_sc{sc_count} '\' all_sc{sc_count} '_initial.csv'];
    xosc = [sc_path '\' all_sc{sc_count} '\' all_sc{sc_count} '_exam.xosc'] ;
    xodr = [sc_path '\' all_sc{sc_count} '\' all_sc{sc_count} '.xodr'];
    scenario_load;%导入场景数据
    clear csv xosc xodr ans fileList
    disp(['当前场景：',all_sc{sc_count},'——第',num2str(sc_count),'个'])
    task_ana;% 驾驶任务分析
    if sc_invalid == 0
        density_ana;% 周边车辆密度与遮挡情况分析
        cand_group = [cand_group;cand_group_1];% 合并候选组
        cand_group = unique(cand_group, 'rows');
        for i = size(cand_group,1):-1:1
            if cand_group(i,1) == cand_group(i,2)
                if i == 1
                    cand_group = cand_group(2:end,:);
                elseif i == size(cand_group,1)
                    cand_group = cand_group(1:end-1,:);
                else
                    cand_group = [cand_group(1:i-1,:);cand_group(i+1:end,:)];
                end
            end
        end
        save 1.mat
        mutation_A;% 执行变异策略A
        delete 1.mat
        testing;
        elevate;
            target_mut;
        if new_sence == 1
            return
        end
        overlap = 0;
        if task_anal(1,1) == task_anal(1,carnum)
            mut_B_2;
        else
            if task_anal(1,1) == 5 || task_anal(1,1) == 2
                mut_B;
            end
        end
        if overlap == 0
            dirname = ['F:\ONSITE\av2buzz65\' all_sc{sc_count}] ;
            mkdir(dirname);
            copyfile(xodr,dirname,"f");
            xodr = [dirname '\' xodrname '.xodr'];
            export_xosc;
        end
    end
    elapsedTime = toc;
    alltime = alltime + elapsedTime;
    fprintf('耗时: %f 秒\n', elapsedTime);
%     clearvars -except sce_file_dir sc_count  alltime
end