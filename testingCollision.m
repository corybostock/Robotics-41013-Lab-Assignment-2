%% initialising all the objects

close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
hold on;
floorOffset = (-1.0896/2);  
workSpace = [-2.5 2.5 -2.5 2.5 (2*floorOffset) 2];
sawyerBase = transl(0,0,0);
motion = move();

%put models in the workspace
sawyer1 = sawyer(workSpace, 1, sawyerBase);
hold on;
%table = body(workSpace, 'table', transl(0,0,1));
table = body(workSpace, 'table', transl(0,0,floorOffset));
lightCurtain  = body(workSpace, 'fence', transl(1, 1, floorOffset));
r = 0.05;
radii = [r,r,r];

%object faces and vertices
tableFace = cell2mat(table.model.faces(1));
tableVertex = cell2mat(table.model.points(1));
tableVertex(:,3) = tableVertex(:,3)+floorOffset;
tableNormals = cell2mat(table.faceNormals(1));

%centerpnt = [0.5,0,0.2];
%side = 0.25;
%plotOptions.plotFaces = true;
%[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);


%plot3(tableVertex(:,1),tableVertex(:,2), tableVertex(:,3))
alpha(0.5)
axis equal
camlight

%set up positions
targetpos = transl(0.262,-0.337,1.184);
tempq = sawyer1.model.ikcon(targetpos);
trajectory = jtraj(sawyer1.model.getpos(), tempq ,50);
centers = zeros(8,3);
%sawyer1.model.teach
%% 
%move the robot and check collisions
robotToTable = CollisionReact(sawyer1, tableFace, tableVertex, tableNormals, trajectory)

%% avoid collisions inspired by randomly picking points within joint angle

 % 3.3: Randomly select waypoints (primative RRT)
 q1 = sawyer1.model.getpos()
 q2 = tempq
sawyer1.model.animate(q1)
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
robotTableAvoid = CollisionAvoid(sawyer1, qWaypoints, tableFace, tableVertex, tableNormals, isCollision, q2, qMatrix, checkedTillWaypoint)
% while (isCollision)
%     startWaypoint = checkedTillWaypoint;
%     for i = startWaypoint:size(qWaypoints,1)-1
%         qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
%         if ~IsCollision(sawyer1.model,qMatrixJoin,tableFace,tableVertex,tableNormals)
%             qMatrix = [qMatrix; qMatrixJoin]; 
%             sawyer1.model.animate(qMatrixJoin);
%             size(qMatrix)
%             isCollision = false;
%             checkedTillWaypoint = i+1;
%             % Now try and join to the final goal (q2)
%             qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
%             if ~IsCollision(sawyer1.model,qMatrixJoin,tableFace,tableVertex,tableNormals)
%                 qMatrix = [qMatrix;qMatrixJoin];
%                 % Reached goal without collision, so break out
%                 break;
%             end
%         else
%             % Randomly pick a pose that is not in collision
%             qRand = (2 * rand(1,3) - 1) * pi;
%             while IsCollision(sawyer1.model,qMatrixJoin,tableFace,tableVertex,tableNormals)
%                 qRand = (2 * rand(1,3) - 1) * pi;
%             end
%             qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
%             isCollision = true;
%             break;
%         end
%     end
% end




