from onsite import scenarioOrganizer, env
from onsite.controller import ReplayParser
from idm import IDM
import time
import os
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
# 版本号:V1.8
# 更新日期:2023.5.20
# 更新内容:
    # 根据赛方要求，遍历场景时添加了try-except结构，以应对规划算法在个别场景失效导致的测试终止问题。
def check_dir(target_dir):
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

if __name__ == "__main__":
    # 指定输入输出文件夹位置
    input_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../inputs'))
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../outputs'))

    # check dir
    check_dir(output_dir)
    # 记录测试时间，用于评估效率，没有特殊用途
    tic = time.time()
    # 实例化场景管理模块（ScenairoOrganizer）和场景测试模块（Env）
    so = scenarioOrganizer.ScenarioOrganizer()
    envi = env.Env()
    # 初始化IDM规控器
    planner = IDM()
    # 根据配置文件config.py装载场景，指定输入文件夹即可，会自动检索配置文件
    so.load(input_dir, output_dir)
    print(f"<测试参数>\n{so.config}")
    
    # 碰撞次数统计参数
    collision_num = 0
    # 碰撞场景列表
    collision_list = []
    # 测试场景数量统计
    scenario_test_num = 0
    while True:
        # 使用场景管理模块给出下一个待测场景
        scenario_to_test = so.next()
        if scenario_to_test is None:
            break  # 如果场景管理模块给出None，意味着所有场景已测试完毕。
        print(f"<scene-{scenario_to_test['data']['scene_name']}>")
        # print(scenario_to_test)
        # 如果场景管理模块不是None，则意味着还有场景需要测试，进行测试流程。
        
        # 使用env.make方法初始化当前测试场景，可通过visilize选择是否可视化，默认关闭
        observation, traj = envi.make(scenario=scenario_to_test)
        try:
            # 添加场景解析器
            parser = ReplayParser()  # 构造场景解析器
            replay_info = parser.parse(scenario_to_test['data'])  # 通过解析器解析场景
            # 建立路网列表
            luwang_list = []
            for discrete_lane in replay_info.road_info.discretelanes:  # 通过解析器解析场景

                line_id = discrete_lane.lane_id #line为每一次获取的数据
                luwang_list.append(line_id) #将每一次获取到的数据存放到总的列表中
                line_left = discrete_lane.left_vertices #line为每一次获取的数据
                luwang_list.append(line_left) #将每一次获取到的数据存放到总的列表中
                line_right = discrete_lane.right_vertices #line为每一次获取的数据
                luwang_list.append(line_right) #将每一次获取到的数据存放到总的列表中
                line_center = discrete_lane.center_vertices #line为每一次获取的数据
                luwang_list.append(line_center) #将每一次获取到的数据存放到总的列表中

            time_test_start = -1
            # 当测试还未进行完毕，即观察值中test_setting['end']还是-1的时候
            while observation['test_setting']['end'] == -1:
                time_test_start += 1
                # action = planner.act(observation, lane_center,lane_center_x_max,lane_center_x_min, traj)  # 规划控制模块做出决策，得到本车加速度和方向盘转角。
                action = planner.act(observation, luwang_list, time_test_start, traj)
                # print("action:", action)
                observation = envi.step(action)  # 根据车辆的action，更新场景，并返回新的观测值。
                # 如果测试完毕，将测试结果传回场景管理模块（ScenarioOrganizer)
            scenario_test_num += 1
            print('测试场景数量：',scenario_test_num)
            if observation['test_setting']['end'] == 2: # V1.6增加批量测试碰撞次数统计,以及碰撞场景列表，便于测试开发
                collision_num += 1
                collision_list.append(f"<scene-{scenario_to_test['data']['scene_name']}>")
            print('碰撞发生次数为',collision_num)
            if len(collision_list) > 0:
                print('碰撞场景为',collision_list)
        except Exception as e:
            print(repr(e))
        finally:
            so.add_result(scenario_to_test, observation['test_setting']['end'])
            # 在每一次测试最后都关闭可视化界面，避免同时存在多个可视化
            plt.close()
    toc = time.time()
    print(toc - tic)
