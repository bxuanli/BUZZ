% 设置路径
path = 'F:\ONSITE\idm_demo';

% 构建运行的命令
command = ['cd /d ' path ' & python planner'];

% 执行命令
status = system(command);

% 检查命令执行状态
if status == 0
    disp('Python程序执行成功。');
else
    disp('Python程序执行失败。');
end