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
targetpos = transl(-0.055,-0.385,1.211);
tempq = sawyer1.model.ikcon(targetpos);
trajectory = jtraj(sawyer1.model.getpos(), tempq ,50);
centers = zeros(8,3);
%sawyer1.model.teach
%% 
%move the robot and check collisions
for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    IsCollision(sawyer1.model, q, tableFace, tableVertex, tableNormals)

    if IsCollision(sawyer1.model, q, tableFace, tableVertex, tableNormals) 
        display('Collsion')
        return
    else
        display('all g to keep moving')
    end
    
    sawyer1.model.animate(q);
end
%% avoid collisions inspired by randomly picking points within joint angle

 % 3.3: Randomly select waypoints (primative RRT)
 q1 = sawyer1.model.getpos()
 q2 = tempq
sawyer1.model.animate(q1)
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(sawyer1.model,qMatrixJoin,tableFace,tableVertex,tableNormals)
            qMatrix = [qMatrix; qMatrixJoin]; 
            sawyer1.model.animate(qMatrixJoin);
            size(qMatrix)
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(sawyer1.model,qMatrixJoin,tableFace,tableVertex,tableNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,3) - 1) * pi;
            while IsCollision(sawyer1.model,qMatrixJoin,tableFace,tableVertex,tableNormals)
                qRand = (2 * rand(1,3) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end




%% 
%set up positions
% targetpos = transl(0.4,0.3,-0.5)
% tempq = sawyer1.model.ikcon(targetpos);
% trajectory = jtraj(sawyer1.model.getpos(), tempq ,50);
% 
% for step = 1:size(trajectory,1)
%     q = trajectory(step,:);
%     sawyer1.model.animate(q);
%     
% end
% 
% %check for intersections
% for i = 1 : size(tr,3)-1    
%     for faceIndex = 1:size(faces,1)
%         vertOnPlane = vertex(faces(faceIndex,1)',:);
%         [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
%         if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%             plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%             display('Intersection');
%         end
%     end    
% end
% 
% %% moving and collision checking
% %transform of every joint
% tr = zeros(4,4,sawyer1.model.n+1);   %create a 4x4 matrix of each joint
% tr(:,:,1) = sawyer1.model.base;      %let the first matrix represent the base 
% L = sawyer1.model.links;
% centers = zeros(8,3);
% %sawyer1.model.fkine(sawyer1.model.getpos())
% for i = 1 : sawyer1.model.n          %loop through and create a 4x4 matrix for each joint
%     tr(:,:,i+1) = tr(:,:,i) * trotz(sawyer1.defaultq(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
%     temp = tr(:,:,i);
%     temp = temp(1:3,4)';
%     centers(i, :) = temp;
% end
% 
% %set up positions
% targetpos = transl(0.531441000000000,-0.902481000000000,0.252341000000000)
% tempq = sawyer1.model.ikcon(targetpos)
% trajectory = jtraj(sawyer1.model.getpos(), tempq ,50);
% 
% %ellipsoid distances to the table
% 
% 
% for step = 1:size(trajectory,1)
%     q = trajectory(step,:);
%     sawyer1.model.animate(q);
%     
% end
%        





%% old bits of code

%set up collision checks
%endeffectpos = p560.fkine(p560.getpos())
% [X,Y] = meshgrid(-2:0.5:2,-2:0.5:2);
% sizeMat = size(X);
% Z = repmat(-0.2, sizeMat(1),sizeMat(2));
% intersectionPlane = surf(X,Y,Z);
% planePoints = [X(:),Y(:),Z(:)];

% 
% % Go through each link and also each triangle face
% for i = 1 : size(tr,3)-1    
%     for faceIndex = 1:size(tableFace,1)
%         vertOnPlane = tableVertex(tableFace(faceIndex,1)',:);
%         [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
%         if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%             plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%             display('Intersection');
%         end
%     end    
% end
% 
% algebraicDistx = (tableVertex{1}(:,1)-endeffectpos(1,4)).^2;
% algebraicDisty = (tableVertex{1}(:,2)-endeffectpos(2,4)).^2 ;
% algebraicDistz = (tableVertex{1}(:,3)-endeffectpos(3,4)).^2


%     endeffectpos = p560.fkine(p560.getpos())
%     algebraicDistz = (tableVertex{1}(:,3)-endeffectpos(3,4)).^2
%        if algebraicDistz < 0.05
%         display(['COLLISION OCCURED']) 
%         break

%plot3(tableVertex{1}(:,1),tableVertex{1}(:,2), tableVertex{1}(:,3)+floorOffset)

% for step = 1:size(trajectory,1)
%     for i = 2:7
%         algebraicDist = ((tableVertex{1}(:,1)-centers(i,1))/radii(1)).^2 ...
%             + ((tableVertex{1}(:,2)-centers(i,2))/radii(2)).^2 ...
%             + ((tableVertex{1}(:,3)-centers(i,3))/radii(3)).^2;
%         pointsInside = find(algebraicDist<radii);
%         if pointsInside >0
%             display(['There are ', num2str(size(pointsInside,1)),' points inside for joint', num2str(i)]);
%         end
%     end
%     q = trajectory(step,:);
%     sawyer1.model.animate(q);
%     
% end

   % temp = tr(:,:,i);
  %  temp = temp(1:3,4)';
 %   centers(i, :) = temp;
 
 %     for i = 2: 8
%         [X,Y,Z] = ellipsoid(centers(i,1), centers(i,2), centers(i,3), radii(1), radii(2), radii(3) );
%         hold on;
%         view(3)
%         ellispoidAtOrigin_h = surf(X,Y,Z)
%         alpha(0.1);
%     end
    %q = zeros(1,7)
    
    %         temp = tr(:,:,i);
%         temp = temp(1:3,4)';
%         centers(i, :) = temp;
%         centers(8,:) = endEffect

    centers = zeros(8,3);
    endEffect = sawyer1.model.fkine(sawyer1.model.getpos())
    endEffect = endEffect(1:3,4)'
    
    
        %find each link and store their centers in a matrix
%     tr = zeros(4,4,sawyer1.model.n+1);   %create a 4x4 matrix of each joint
%     tr(:,:,1) = sawyer1.model.base;      %let the first matrix represent the base
%     L = sawyer1.model.links;
%     for i = 1 : sawyer1.model.n          %loop through and create a 4x4 matrix for each joint
%         tr(:,:,i+1) = tr(:,:,i) * trotz(sawyer1.defaultq(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
%         temp = tr(:,:,i);
%         temp = temp(1:3,4)';
%         centers(i, :) = temp;
%         endEffect = sawyer1.model.fkine(sawyer1.model.getpos());
%         endEffect = endEffect(1:3,4)'
%         centers(8,:) = endEffect;
%         
%         for i = 2: 8
%             [X,Y,Z] = ellipsoid(centers(i,1), centers(i,2), centers(i,3), radii(1), radii(2), radii(3) );
%             hold on;
%             view(3);
%             ellispoidAtOrigin_h = surf(X,Y,Z)
%             alpha(0.1);
%         end  
  %  end
    
    