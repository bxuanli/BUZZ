function edgePoints = getEdgePoints(center, width, height,angle)
    % center: 中心坐标点的二维数组，例如 [x_center, y_center]
    % width: 矩阵的宽度
    % height: 矩阵的高度
    % 内置参数：边缘数值的间距
    spacing = 0.1;
    
    % 计算矩阵的左上角和右下角坐标
    x1 = center(1) - width / 2;
    y1 = center(2) - height / 2;
    x2 = center(1) + width / 2;
    y2 = center(2) + height / 2;
    
    % 初始化边缘点数组
    edgePoints = [];
    
    % 左边缘
    for x = x1:spacing:x1+width
        edgePoints = [edgePoints; x, y1];
    end
    
    % 上边缘
    for y = y1:spacing:y1+height
        edgePoints = [edgePoints; x1, y];
    end
    
    % 右边缘
    for x = x2:-spacing:x2-width
        edgePoints = [edgePoints; x, y2];
    end
    
    % 下边缘
    for y = y2:-spacing:y2-height
        edgePoints = [edgePoints; x2, y];
    end
    
    edgePoints = unique(edgePoints, 'rows');
    % 创建旋转矩阵
    rotationMatrix = [cos(angle), -sin(angle);
                      sin(angle),  cos(angle)];
    
    % 应用旋转矩阵到边缘点
    edgePoints = (edgePoints-center) * rotationMatrix + center;
end