%% 图框边界锚定
min_x = min(ego(:, 1));
max_x = max(ego(:, 1));
min_y = min(road_line);
max_y = max(road_line);
for i = 1:carnum-1
    car_data = eval(sprintf('A%d', i));
    car_min_x = min(car_data(:, 1));
    car_max_x = max(car_data(:, 1));
    min_x = min(min_x, car_min_x);
    max_x = max(max_x, car_max_x);
end
clear car_max_x car_min_x car_data 
%% 场景绘制
figure; 
for i = 1:size(ego, 1)
    for j = 1:carnum - 1
        car_data = eval(sprintf('A%d', j));
        car_wps = car_data(:, 1:2);
        car_size = car_data(1, 6:7);
        rectangle('Position', [car_wps(i, 1) - 0.5 * car_size(2), car_wps(i, 2) - 0.5 * car_size(1), car_size(2), car_size(1)], 'EdgeColor', 'b', 'LineWidth', 1);
        text(car_wps(i, 1), car_wps(i, 2), sprintf('A%d', j), 'Color', 'b', 'HorizontalAlignment', 'center');
    end
    egopos_now = ego(i,1:2);
    rectangle('Position', [egopos_now(1) - 0.5 * ego(1,7), egopos_now(2) - 0.5 * ego(1,6), ego(1,7), ego(1,6)], 'EdgeColor', 'g', 'LineWidth', 1);
    rectangle('Position', [target(1, 1), target(2, 1), target(1, 2) - target(1, 1), target(2, 2) - target(2, 1)], 'EdgeColor', 'r', 'LineWidth', 1);
%     viscircles([egopos_now(1), egopos_now(2)], 20, 'EdgeColor', 'k', 'LineWidth', 1);
    hold on 
    for k = 1:length(road_line)
        plot(min_x:max_x, repmat(road_line(k), 1, length(min_x:max_x)), '--k');
    end
    xlim([min_x, max_x]);
    ylim([min_y, max_y]);
     pause(0.01);
    if i < size(ego, 1)
        clf; 
    end
end
close all ;
clear car_data car_size car_wps egopos_now max_y max_x min_y min_x ans