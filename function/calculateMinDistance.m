function minDistance = calculateMinDistance(rect1, rect2)
    % rect1 和 rect2 是两个矩形边框坐标点的矩阵，尺寸为 Nx2
    
    % 初始化最小距离为无穷大
    minDistance = inf;
    
    % 获取矩形1和矩形2的点数
    numPoints1 = size(rect1, 1);
    numPoints2 = size(rect2, 1);
    
    % 遍历矩形1中的每个点
    for i = 1:numPoints1
        % 计算点rect1(i,:)到矩形2中所有点的距离
        distances = sqrt(sum((rect2 - rect1(i,:)).^2, 2));
        
        % 更新最小距离
        if min(distances) < minDistance
            minDistance = min(distances);
        end
    end
    
    % 遍历矩形2中的每个点
    for i = 1:numPoints2
        % 计算点rect2(i,:)到矩形1中所有点的距离
        distances = sqrt(sum((rect1 - rect2(i,:)).^2, 2));
        
        % 更新最小距离
        if min(distances) < minDistance
            minDistance = min(distances);
        end
    end
end