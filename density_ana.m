%% 构造车辆密度分析矩阵
density_mat = [];
for i = 1:carnum
    main_cardata = eval(sprintf('A%d',i));
    for j = 1:carnum
        cardata = eval(sprintf('A%d',j));
        roundcar = zeros(5,1);
        if i ~= j
            for k = 1:size(ego,1)
                if norm(main_cardata(k,1:2) - cardata(k,1:2)) <= 10 %定义感知范围
                    roundcar(1) = roundcar(1) + 1;
                end
                if norm(main_cardata(k,1:2) - cardata(k,1:2)) <= 20 
                    roundcar(2) = roundcar(2) + 1;
                end
                if norm(main_cardata(k,1:2) - cardata(k,1:2)) <= 30 
                    roundcar(3) = roundcar(3) + 1;
                end
                if norm(main_cardata(k,1:2) - cardata(k,1:2)) <= 40 
                    roundcar(4) = roundcar(4) + 1;
                end
                if norm(main_cardata(k,1:2) - cardata(k,1:2)) <= 50 
                    roundcar(5) = roundcar(5) + 1;
                end
            end
        density_mat(1,carnum*(i-1)+j) = roundcar(1);
        density_mat(2,carnum*(i-1)+j) = roundcar(2);
        density_mat(3,carnum*(i-1)+j) = roundcar(3);
        density_mat(4,carnum*(i-1)+j) = roundcar(4); 
        density_mat(5,carnum*(i-1)+j) = roundcar(5);
        density_mat(6,carnum*(i-1)+j) = j;
        end
    end
end
clear cardata roundcar main_cardata
%% 确定主车
density_mat(:,end+1) = 0;
density_mat_1 = [];
for i = 1:carnum
    for j = 1:5
        density_mat_1(j,i) = sum(density_mat(j,carnum*(i-1)+1:carnum*(i-1)+carnum));
    end
end
main_cand = [];
for i = 1:5
    max_in_line = max(density_mat_1(i,:));
    a = 1;
    for j = 1:carnum
        if density_mat_1(i,j) == max_in_line
            main_cand(i,a) = j;
            a = a+1;
        end
    end
end
clear max_in_line a
%% 确定变异车辆
mut_cand = [];
for i = 1:5
    max_in_line = max(density_mat(i,carnum*(main_cand(i)-1)+1:carnum*(main_cand(i)-1)+carnum));
    b = 1;
    for j = 1:carnum
        if density_mat(i,carnum*(main_cand(i)-1)+j) == max_in_line
            mut_cand(i,b) = j;
            b = b+1;
        end
    end
end
clear max_in_line b
%% 形成候选组
cand_group_1 = [];
for i = 1:size(main_cand, 1)
    rowA = main_cand(i, :);
    rowB = mut_cand(i, :);
    for j = 1:length(rowA)
        for k = 1:length(rowB)
            if rowA(j) == 0 || rowB(k) == 0
                continue;
            end            
            cand_group_1 = [cand_group_1; rowA(j), rowB(k)];
        end
    end
end
clear main_cand mut_cand rowA rowB 