scenario = drivingScenario;
roadNetwork(scenario,"OpenDRIVE",xodr);
road = scenario.laneMarkingVertices;
road_line = unique(road(:,2));
clear road
%数据预处理
data = readmatrix(csv);%读取数据
data = data(2:end-1,:);%删除无用行
carnum  = (size(data,2)-2)/7;
%为各车辆矩阵赋值
for i = 1:carnum
    carname = sprintf('A%d', i-1);
    col_start = 7*(i-1)+2;
    col_end = 7*i+1;
    eval([carname ' = data(:, col_start:col_end);']);
end
ego = A0;
%% 获取目的地
fileID = fopen(xosc, 'r');
fileContent = textscan(fileID, '%s', 'Delimiter', '\n');
fclose(fileID);
targetString = 'x_target';
matchingRows = strfind(fileContent{1}, targetString);
rowNumber = find(~cellfun(@isempty, matchingRows), 1);
task = fileContent{1}{rowNumber};
pattern = '\(([^,]+), ([^)]+)\)';
matches = regexp(task, pattern, 'tokens');
target = [str2double(matches{1}{1}), str2double(matches{1}{2}); str2double(matches{2}{1}), str2double(matches{2}{2})];
clear data col_end col_start carname A0 fileContent targetString matchingRows rowNumber task pattern matches csvfilepath sourcexosc xodrfilepath scenario fileID


