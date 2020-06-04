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
table = body(workSpace, 'table', transl(0,0,floorOffset));
r = 0.05
radii = [r,r,r]

%object faces and vertices
tableFace = table.model.faces
tableVertex = table.model.points
tableNormals = table.faceNormals 

alpha(0.5)
axis equal
camlight

%find each link and store their centers in a matrix
 tr = zeros(4,4,sawyer1.model.n+1);   %create a 4x4 matrix of each joint
 tr(:,:,1) = sawyer1.model.base;      %let the first matrix represent the base 
 L = sawyer1.model.links;
% centers = zeros(7,3);

for i = 1 : sawyer1.model.n          %loop through and create a 4x4 matrix for each joint
    tr(:,:,i+1) = tr(:,:,i) * trotz(sawyer1.defaultq(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

for i = 1 : size(tr,3)-1
    for faceIndex = 1:size(tableFace{1},1)
        vertOnPlane = tableVertex{1}(tableFace{1}(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(tableNormals{1}(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,tableVertex{1}(tableFace{1}(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection')
        end
    end    
end
%% 
%set up positions
targetpos = transl(0.4,0.3,-0.5)
tempq = sawyer1.model.ikcon(targetpos);
trajectory = jtraj(sawyer1.model.getpos(), tempq ,50);



for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    sawyer1.model.animate(q);
    
end

%check for intersections
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
    end    
end

%% moving and collision checking
%transform of every joint
tr = zeros(4,4,sawyer1.model.n+1);   %create a 4x4 matrix of each joint
tr(:,:,1) = sawyer1.model.base;      %let the first matrix represent the base 
L = sawyer1.model.links;
centers = zeros(7,3);

for i = 1 : sawyer1.model.n          %loop through and create a 4x4 matrix for each joint
    tr(:,:,i+1) = tr(:,:,i) * trotz(sawyer1.defaultq(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    temp = tr(:,:,i);
    temp = temp(1:3,4)';
    centers(i, :) = temp;
end

%set up positions
targetpos = transl(0.531441000000000,-0.902481000000000,0.252341000000000)
tempq = sawyer1.model.ikcon(targetpos)
trajectory = jtraj(sawyer1.model.getpos(), tempq ,50);

%ellipsoid distances to the table


for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    sawyer1.model.animate(q);
    
end
       





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