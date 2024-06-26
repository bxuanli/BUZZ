
%% openscenario-1级
opensc = com.mathworks.xml.XMLUtils.createDocument('OpenSCENARIO');
opensc.setXmlStandalone(true);
%% 头文件-2级
root = opensc.getDocumentElement();
    FileHeader = opensc.createElement('FileHeader');
    currentTime = datestr(now, 'yyyy-mm-ddTHH:MM:SS'); 
    FileHeader.setAttribute('revMajor', '1');
    FileHeader.setAttribute('revMinor', '0');
    FileHeader.setAttribute('date',currentTime);
root.appendChild(FileHeader);
%% 参数声明-2级
ParameterDeclarations = opensc.createElement('ParameterDeclarations');
root.appendChild(ParameterDeclarations);
%% 目录位置-2级
CatalogLocations = opensc.createElement('CatalogLocations');
root.appendChild(CatalogLocations);
%% 创建路网-2级
roadNetwork = opensc.createElement('RoadNetwork');
root.appendChild(roadNetwork);
    logicFile = opensc.createElement('LogicFile');
    [~,Xodrfile,~] = fileparts(xodr);
    Xodrfile = Xodrfile+'.xodr';
    logicFile.setAttribute('filepath',Xodrfile);
roadNetwork.appendChild(logicFile);
%% 创建实体-2级
Entities = opensc.createElement('Entities');
root.appendChild(Entities);
    ScenarioObject = opensc.createElement('ScenarioObject');
Entities.appendChild(ScenarioObject);
%% 定义自车-3级
    ScenarioObject.setAttribute('name','Ego');
        Vehicle = opensc.createElement('Vehicle');
        Vehicle.setAttribute('name','Default_car');
        Vehicle.setAttribute('vehicleCategory','car');
            BoundingBox = opensc.createElement('BoundingBox');
                Center = opensc.createElement('Center');
                Center.setAttribute('x',sprintf('%.16e',1.5));
                Center.setAttribute('y',sprintf('%.16e',0));
                Center.setAttribute('z',sprintf('%.16e',0.9));
            BoundingBox.appendChild(Center);
                Dimensions = opensc.createElement('Dimensions');
                width = sprintf('%.16e',ego(1,6));
                length = sprintf('%.16e',ego(1,7));
                Dimensions.setAttribute('width',width);
                Dimensions.setAttribute('length',length);
                Dimensions.setAttribute('height',sprintf('%.16e',1.8));
            BoundingBox.appendChild(Dimensions);
            performance = opensc.createElement('performance');
            performance.setAttribute('maxSpeed','200');
            performance.setAttribute('maxAcceleration','200');
            performance.setAttribute('maxDeceleration','10');
            Axles = opensc.createElement('Axles');
                FrontAxle = opensc.createElement('FrontAxle');
                FrontAxle.setAttribute('maxSteering','0.5');
                FrontAxle.setAttribute('wheelDiameter','0.5');
                FrontAxle.setAttribute('trackWidth','1.75');
                FrontAxle.setAttribute('positionX','2.8');
                FrontAxle.setAttribute('positionZ','0.25');
            Axles.appendChild(FrontAxle);
                RearAxle = opensc.createElement('FrontAxle');
                RearAxle.setAttribute('maxSteering','0.5');
                RearAxle.setAttribute('wheelDiameter','0.5');
                RearAxle.setAttribute('trackWidth','1.75');
                RearAxle.setAttribute('positionX','0');
                RearAxle.setAttribute('positionZ','0.25');
            Axles.appendChild(RearAxle);  
            properties = opensc.createElement('properties');
        Vehicle.appendChild(BoundingBox);
        Vehicle.appendChild(performance);
        Vehicle.appendChild(Axles);
        Vehicle.appendChild(properties);
    ScenarioObject.appendChild(Vehicle);

%% 定义NPC车辆
for i = 1:carnum-1
    car_name = sprintf('A%d', i); 
    car_data = eval(car_name);
    ScenarioObject = opensc.createElement('ScenarioObject');
    ScenarioObject.setAttribute('name',car_name);
    Entities.appendChild(ScenarioObject);
        Vehicle = opensc.createElement('Vehicle');
        Vehicle.setAttribute('name','Default_car');
        Vehicle.setAttribute('vehicleCategory','car');
            BoundingBox = opensc.createElement('BoundingBox');
                Center = opensc.createElement('Center');
                Center.setAttribute('x',sprintf('%.16e',1.5));
                Center.setAttribute('y',sprintf('%.16e',0));
                Center.setAttribute('z',sprintf('%.16e',0.9));
            BoundingBox.appendChild(Center);
                Dimensions = opensc.createElement('Dimensions');
                width = sprintf('%.16e',car_data(1,6));
                length = sprintf('%.16e',car_data(1,7));
                Dimensions.setAttribute('width',width);
                Dimensions.setAttribute('length',length);
                Dimensions.setAttribute('height',sprintf('%.16e',1.8));
            BoundingBox.appendChild(Dimensions);
            performance = opensc.createElement('performance');
            performance.setAttribute('maxSpeed','200');
            performance.setAttribute('maxAcceleration','200');
            performance.setAttribute('maxDeceleration','10');
            Axles = opensc.createElement('Axles');
                FrontAxle = opensc.createElement('FrontAxle');
                FrontAxle.setAttribute('maxSteering','0.5');
                FrontAxle.setAttribute('wheelDiameter','0.5');
                FrontAxle.setAttribute('trackWidth','1.75');
                FrontAxle.setAttribute('positionX','2.8');
                FrontAxle.setAttribute('positionZ','0.25');
            Axles.appendChild(FrontAxle);
                RearAxle = opensc.createElement('FrontAxle');
                RearAxle.setAttribute('maxSteering','0.5');
                RearAxle.setAttribute('wheelDiameter','0.5');
                RearAxle.setAttribute('trackWidth','1.75');
                RearAxle.setAttribute('positionX','0');
                RearAxle.setAttribute('positionZ','0.25');
            Axles.appendChild(RearAxle);  
            properties = opensc.createElement('properties');
        Vehicle.appendChild(BoundingBox);
        Vehicle.appendChild(performance);
        Vehicle.appendChild(Axles);
        Vehicle.appendChild(properties);
    ScenarioObject.appendChild(Vehicle);
end
%% 定义故事板-2级
    Storyboard = opensc.createElement('Storyboard');
root.appendChild(Storyboard);
        Init = opensc.createElement('Init');
    Storyboard.appendChild(Init);
            Actions = opensc.createElement('Actions');
        Init.appendChild(Actions);
%% 定义自车任务与动作-5级
                Private = opensc.createElement('Private');
                Private.setAttribute('entityRef','Ego');
                    comment1 = opensc.createComment('Information of the ego vehicle will be hidden, and its initial state and driving task will be explained in the comments below');
                    comment2 = opensc.createComment('[Initial State] v_init = '+string(ego(1,3))+', x_init = '+string(ego(1,1))+', y_init = '+string(ego(1,2))+', heading_init = '+string(ego(1,5)));
                    comment3 = opensc.createComment('[Driving Task] x_target = ('+string(target(1,1))+', '+string(target(1,2))+'), y_target = ('+string(target(2,1))+', '+string(target(2,2))+')');
                Private.appendChild(comment1);
                Private.appendChild(comment2);
                Private.appendChild(comment3);
            Actions.appendChild(Private);
%% 定义NPC车辆初始状态-5级
for i = 1:carnum-1
    car_nameA = sprintf('A%d', i); 
    car_data = eval(car_nameA);
                Private = opensc.createElement('Private');
                Private.setAttribute('entityRef',car_nameA);
            Actions.appendChild(Private);
                    PrivateAction = opensc.createElement('PrivateAction');
                Private.appendChild(PrivateAction);
                        LongitudinalAction = opensc.createElement('LongitudinalAction');
                    PrivateAction.appendChild(LongitudinalAction);
                            SpeedAction = opensc.createElement('SpeedAction');
                        LongitudinalAction.appendChild(SpeedAction);
                                SpeedActionDynamics = opensc.createElement('SpeedActionDynamics');
                                SpeedActionDynamics.setAttribute('dynamicsShape',"step");
                                SpeedActionDynamics.setAttribute('value',"0");
                                SpeedActionDynamics.setAttribute('dynamicsDimension',"time");
                            SpeedAction.appendChild(SpeedActionDynamics);
                                SpeedActionTarget = opensc.createElement('SpeedActionTarget');
                            SpeedAction.appendChild(SpeedActionTarget);
                                    AbsoluteTargetSpeed = opensc.createElement('AbsoluteTargetSpeed');
                                    targetspd = sprintf('%.16e',-car_data(1,3));
                                    AbsoluteTargetSpeed.setAttribute('value',targetspd);
                                SpeedActionTarget.appendChild(AbsoluteTargetSpeed);
                    PrivateAction = opensc.createElement('PrivateAction');
                Private.appendChild(PrivateAction);
                        TeleportAction = opensc.createElement('TeleportAction');
                    PrivateAction.appendChild(TeleportAction);
                            Positon = opensc.createElement('Positon');
                        TeleportAction.appendChild(Positon);
                                WorldPosition = opensc.createElement('WorldPosition');
                                WorldPosition.setAttribute('x',sprintf('%.16e',car_data(1,1)));
                                WorldPosition.setAttribute('y',sprintf('%.16e',car_data(1,2)));
                                WorldPosition.setAttribute('z',sprintf('%.16e',0));
                                WorldPosition.setAttribute('h',sprintf('%.16e',car_data(1,5)));
                                WorldPosition.setAttribute('p',sprintf('%.16e',0));
                                WorldPosition.setAttribute('r',sprintf('%.16e',0));
                            Positon.appendChild(WorldPosition);
end
%% 编辑故事-3级
        Story = opensc.createElement('Story');
        Story.setAttribute('name','scenario');
    Storyboard.appendChild(Story);
            ParameterDeclarations = opensc.createElement('ParameterDeclarations');
        Story.appendChild(ParameterDeclarations);
%% 轨迹赋予-4级
for i = 1:carnum-1
    car_nameA = sprintf('A%d', i);
    car_nameAct = sprintf('Act_A%d', i); 
    car_data = eval(car_nameA);
            Act = opensc.createElement('Act');
            Act.setAttribute('name',car_nameAct);
        Story.appendChild(Act);
                ManeuverGroup = opensc.createElement('ManeuverGroup');
                ManeuverGroup.setAttribute('maximumExecutionCount','1');
                ManeuverGroup.setAttribute('name',sprintf('Sqence_A%d', i));
            Act.appendChild(ManeuverGroup);
                    Actors = opensc.createElement('Actors');
                    Actors.setAttribute('selectTriggeringEntities',"false");
                ManeuverGroup.appendChild(Actors);
                        EntityRef = opensc.createElement('EntityRef');
                        EntityRef.setAttribute('entityRef',sprintf('A%d', i));
                    Actors.appendChild(EntityRef);
                    Maneuver = opensc.createElement('Maneuver');
                    Maneuver.setAttribute('name','Maneuver1');
                ManeuverGroup.appendChild(Maneuver);
                        Event = opensc.createElement('Event');
                        Event.setAttribute('name','Event1');
                        Event.setAttribute('priority',"overwrite");
                    Maneuver.appendChild(Event);
                            Action = opensc.createElement('Action');
                            Action.setAttribute('name',"Action1");
                        Event.appendChild(Action);
                                PrivateAction = opensc.createElement('PrivateAction');
                            Action.appendChild(PrivateAction);
                                    RoutingAction = opensc.createElement('RoutingAction');
                                PrivateAction.appendChild(RoutingAction);
                                        FollowTrajectoryAction = opensc.createElement('FollowTrajectoryAction');
                                    RoutingAction.appendChild(FollowTrajectoryAction);
                                            Trajectory = opensc.createElement('Trajectory');
                                            Trajectory.setAttribute('closed',"false");
                                            Trajectory.setAttribute('name',sprintf('Trajectory_A%d',i));
                                        FollowTrajectoryAction.appendChild(Trajectory);
                                                Shape = opensc.createElement('Shape');
                                            Trajectory.appendChild(Shape);
                                                    Polyline = opensc.createElement('Polyline');
                                                Shape.appendChild(Polyline);
for j = 1:size(car_data,1)

        Vertex = opensc.createElement('Vertex');
        Vertex.setAttribute('time',num2str(0.04*(j-1)));
    Polyline.appendChild(Vertex);
            Positon = opensc.createElement('Positon');
        Vertex.appendChild(Positon);
                WorldPosition = opensc.createElement('WorldPosition');
                WorldPosition.setAttribute('x',sprintf('%.16e',car_data(j,1)));
                WorldPosition.setAttribute('y',sprintf('%.16e',car_data(j,2)));
                WorldPosition.setAttribute('z',sprintf('%.16e',0));
                WorldPosition.setAttribute('h',sprintf('%.16e',car_data(j,5)));
                WorldPosition.setAttribute('p',sprintf('%.16e',0));
                WorldPosition.setAttribute('r',sprintf('%.16e',0));
            Positon.appendChild(WorldPosition);
end

TimeReference = opensc.createElement('TimeReference');
FollowTrajectoryAction.appendChild(TimeReference);
Timing = opensc.createElement('Timing');
Timing.setAttribute('domainAbsoluteRelative',"absolute");
Timing.setAttribute('scale',"1.0");
Timing.setAttribute('offset',"0.0");
TimeReference.appendChild(Timing);
TrajectoryFollowingMode = opensc.createElement('TrajectoryFollowingMode');
TrajectoryFollowingMode.setAttribute('followingMode',"follow");
FollowTrajectoryAction.appendChild(TrajectoryFollowingMode);
%% 事件初始诱因-8级
StartTrigger = opensc.createElement('StartTrigger');
Event.appendChild(StartTrigger);
ConditionGroup = opensc.createElement('ConditionGroup');
StartTrigger.appendChild(ConditionGroup);
Condition = opensc.createElement('Condition');
Condition.setAttribute('name','');
Condition.setAttribute('delay','0');
Condition.setAttribute('conditionEdge',"rising");
ConditionGroup.appendChild(Condition);
ByValueCondition = opensc.createElement('ByValueCondition');
Condition.appendChild(ByValueCondition);
SimulationTimeCondition = opensc.createElement('SimulationTimeCondition');
SimulationTimeCondition.setAttribute('value','0.03');
SimulationTimeCondition.setAttribute('rule',"greaterThan");
ByValueCondition.appendChild(SimulationTimeCondition);
%% 场景初始诱因-5级
StartTrigger = opensc.createElement('StartTrigger');
Act.appendChild(StartTrigger);
ConditionGroup = opensc.createElement('ConditionGroup');
StartTrigger.appendChild(ConditionGroup);
Condition = opensc.createElement('Condition');
Condition.setAttribute('name','');
Condition.setAttribute('delay','0');
Condition.setAttribute('conditionEdge',"rising");
ConditionGroup.appendChild(Condition);
ByValueCondition = opensc.createElement('ByValueCondition');
Condition.appendChild(ByValueCondition);
SimulationTimeCondition = opensc.createElement('SimulationTimeCondition');
SimulationTimeCondition.setAttribute('value','0');
SimulationTimeCondition.setAttribute('rule',"greaterThan");
ByValueCondition.appendChild(SimulationTimeCondition);
end
%% 场景结束诱因
StopTrigger = opensc.createElement('StopTrigger');
Storyboard.appendChild(StopTrigger);
ConditionGroup = opensc.createElement('ConditionGroup');
StopTrigger.appendChild(ConditionGroup);
Condition = opensc.createElement('Condition');
Condition.setAttribute('name','');
Condition.setAttribute('delay','0');
Condition.setAttribute('conditionEdge',"rising");
ConditionGroup.appendChild(Condition);
ByValueCondition = opensc.createElement('ByValueCondition');
Condition.appendChild(ByValueCondition);
SimulationTimeCondition.setAttribute('value',num2str(0.04*size(car_data,1)));
SimulationTimeCondition.setAttribute('rule',"greaterThan");
ByValueCondition.appendChild(SimulationTimeCondition);
%% 将文档保存为文件
xoscfilelen = strlength(xodr)-5;
xoscfile = [xodr{1}(1:xoscfilelen) '_0.xosc'];
xmlwrite(xoscfile, opensc);
% 读取1.xosc文件内容
fileID = fopen(xoscfile, 'r');
fileContent = fread(fileID, '*char')';
fclose(fileID);
% 在文件内容中插入换行符并添加三个tab键缩进
newFileContent = strrep(fileContent, '><', sprintf('>\n\t\t\t<'));
% 将修改后的内容写入新文件
newFilename = [xodr{1}(1:xoscfilelen) '_' num2str(cand_count) '.xosc']; % 新文件名
newFileID = fopen(newFilename, 'w');
fwrite(newFileID, newFileContent);
fclose(newFileID);
delete(xoscfile);