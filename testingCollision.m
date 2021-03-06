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
%motion = move();

%put models in the workspace
%sawyer1 = sawyer(workSpace, 1, sawyerBase);
hold on;
%table = body(workSpace, 'table', transl(0,0,1));
table = body(workSpace, 'table', transl(0,0,0), floorOffset);
line.X = [-1, -1, 1, 1];
line.Y = [-1.5, 1.5, 1.5, -1.5];
line.Z = [-0.5, -0.5, -0.5, -0.5];

for lineIt = line.Z(1):0.1:2
    plot3([line.X(1),line.X(2)],[line.Y(1),line.Y(2)],[lineIt, lineIt],'--b','LineWidth',0.2);
    plot3([line.X(2),line.X(3)],[line.Y(2),line.Y(3)],[lineIt, lineIt],'--b','LineWidth',0.2);
    plot3([line.X(3),line.X(4)],[line.Y(3),line.Y(4)],[lineIt, lineIt],'--b','LineWidth',0.2);
    plot3([line.X(4),line.X(1)],[line.Y(4),line.Y(1)],[lineIt, lineIt],'--b','LineWidth',0.2);
end

%lightCurtain  = body(workSpace, 'fence', transl(1, 1, floorOffset), 0);
hand = body(workSpace, 'hand', transl(2, 0, 0.5),0);

steps = 50;
initx = 2
yVAL = 0
zVAL = 0.5
finalx = 1
array = lspb(initx,finalx,steps)

for i = 1:steps
	hand.model.base = transl(array(i),yVAL,zVAL);
	hand.model.animate(0);
end
  %IsCollision(hand,50,lightCurtainFace,lightCurtainVertex,lightCurtainNormals,false)
   % CollisionReact(hand, lightCurtainFace, lightCurtainVertex, lightCurtainNormals, array)
%r = 0.05;
%radii = [r,r,r];

%object faces and vertices
% tableFace = cell2mat(table.model.faces(1));
% tableVertex = cell2mat(table.model.points(1));
% tableVertex(:,3) = tableVertex(:,3)+floorOffset;
% tableNormals = cell2mat(table.faceNormals(1));



%centerpnt = [0.5,0,0.2];
%side = 0.25;
%plotOptions.plotFaces = true;
%[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);


%plot3(tableVertex(:,1),tableVertex(:,2), tableVertex(:,3))
%alpha(lightCurtain.model, 0.3)
% axis equal
% camlight
% 
% %set up positions
% targetpos = transl(2,-1,0);
% tempq = 0
% %tempq = sawyer1.model.ikcon(targetpos);
% trajectory = jtraj(hand.model.getpos(), tempq ,targetpos,15);
% 
% hand.model.animate(tempq)
%centers = zeros(8,3);
%sawyer1.model.teach
%% 
%move the robot and check collisions
handToLightCurtain = CollisionReact(hand, lightCurtainFace, lightCurtainVertex, lightCurtainNormals, trajectory)
%robotToTable = CollisionReact(sawyer1, tableFace, tableVertex, tableNormals, trajectory)

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




