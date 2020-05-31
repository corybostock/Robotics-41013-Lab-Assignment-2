close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
workSpace = [-6 6 -6 0 -2 6];

mdl_puma560
q = [0,0,0,0,0,0];
p560.plot(q)
hold on;
table = body(workSpace, 'table', transl(0,0,-0.65));
radii = [0.1,0.1,0.1]

%object faces and vertices
tableFace = table.model.faces
tableVertex = table.model.points

%transform of every joint
tr = zeros(4,4,p560.n+1);   %create a 4x4 matrix of each joint
tr(:,:,1) = p560.base;      %let the first matrix represent the base 
L = p560.links;
centers = zeros(7,3);

for i = 1 : p560.n          %loop through and create a 4x4 matrix for each joint
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    temp = tr(:,:,i);
    temp = temp(1:3,4)';
    centers(i, :) = temp;
end

%elipses
for i = 1: 7
  [X,Y,Z] = ellipsoid(centers(i,1), centers(i,2), centers(i,3), radii(1), radii(2), radii(3) );
  hold on;
  view(3)
  ellispoidAtOrigin_h = surf(X,Y,Z)
  alpha(0.1);
end

%set up positions
targetpos = transl(0.5, 0.5, -0.5)
tempq = p560.ikcon(targetpos)
trajectory = jtraj(p560.getpos(), tempq ,50);
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

%set up collision checks
endeffectpos = p560.fkine(p560.getpos())
% [X,Y] = meshgrid(-2:0.5:2,-2:0.5:2);
% sizeMat = size(X);
% Z = repmat(-0.2, sizeMat(1),sizeMat(2));
% intersectionPlane = surf(X,Y,Z);
% planePoints = [X(:),Y(:),Z(:)];

algebraicDistx = (tableVertex{1}(:,1)-endeffectpos(1,4)).^2;
algebraicDisty = (tableVertex{1}(:,2)-endeffectpos(2,4)).^2 ;
algebraicDistz = (tableVertex{1}(:,3)-endeffectpos(3,4)).^2

for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    p560.animate(q);
    endeffectpos = p560.fkine(p560.getpos())
    algebraicDistz = (tableVertex{1}(:,3)-endeffectpos(3,4)).^2
       if algebraicDistz < 0.05
        display(['COLLISION OCCURED']) 
        break
       end
       
end







