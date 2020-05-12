classdef move < handle % Movement of robots or bodies
    properties
    end
    
    methods
        function self = move()
           
        end     
        
        function moveRobot(self, robot, targetPos, qtype)
            if (qtype == 1)
                tempq = robot.model.ikcon(targetPos, robot.model.getpos());
            else
                tempq = robot.model.ikine(targetPos);
            end
            
            trajectory = jtraj(robot.model.getpos(), tempq ,50);
            
            for step = 1:size(trajectory,1)
                q = trajectory(step,:);
                robot.model.animate(q);
                dataLogger(robot);
                pause(0.001);
            end
        end
        
        function moveRobotAndPart(self, robot, targetPos, part)
            tempq = robot.model.ikcon(targetPos, robot.model.getpos());
            trajectory = jtraj(robot.model.getpos(), tempq ,50);
            
            for step = 1:size(trajectory,1)
                q = trajectory(step,:);
                robot.model.animate(q);
                endefect = robot.model.fkine(q);
                part.model.base = endefect * trotx(pi);
                part.model.animate(0);
                dataLogger(robot);
                pause(0.001);
            end
        end
        
        function moveRobotAndAllParts(self, robot, targetPos, part1, part2, part3)
            tempq = robot.model.ikcon(targetPos, robot.model.getpos());
            trajectory = jtraj(robot.model.getpos(), tempq ,25);
            
            for step = 1:size(trajectory,1)
                q = trajectory(step,:);
                robot.model.animate(q);
                endefect = robot.model.fkine(q);
                part1.model.base = endefect * trotx(pi);
                part2.model.base = part1.model.base;
                part3.model.base = part1.model.base;
                part1.model.animate(0);
                part2.model.animate(0);
                part3.model.animate(0);
                dataLogger(robot);
                pause(0.001);
            end
        end
    end
end
