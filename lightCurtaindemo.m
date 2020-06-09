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

table = bodyCopy(workSpace, 'table', transl(0,0,floorOffset));
hand = bodyCopy(workSpace, 'hand', transl(2, 0, 0.5));
handFace = cell2mat(hand.model.faces(1));
handVertex = cell2mat(hand.model.points(1));
handNormals = cell2mat(hand.faceNormals(1));

line.X = [-1, -1, 1, 1];
line.Y = [-1.5, 1.5, 1.5, -1.5];
line.Z = [-0.5, -0.5, -0.5, -0.5];
 
for lineIt = line.Z(1):0.1:2
    plot3([line.X(1),line.X(2)],[line.Y(1),line.Y(2)],[lineIt, lineIt],'--b','LineWidth',0.2);
    plot3([line.X(2),line.X(3)],[line.Y(2),line.Y(3)],[lineIt, lineIt],'--b','LineWidth',0.2);
    plot3([line.X(3),line.X(4)],[line.Y(3),line.Y(4)],[lineIt, lineIt],'--b','LineWidth',0.2);
    plot3([line.X(4),line.X(1)],[line.Y(4),line.Y(1)],[lineIt, lineIt],'--b','LineWidth',0.2);
end
axis equal
camlight

%set up positions
targetpos1 = transl(0.262,-0.337,-0.5);
tempq1 = sawyer1.model.ikcon(targetpos1);
trajectory1 = jtraj(sawyer1.model.getpos(), tempq1 ,50);
centers = zeros(8,3);
%sawyer1.model.teach
%% iscollision for the lightcurtain
steps = 50;
initx = 2;
yVAL = 0;
zVAL = 0.5;
finalx = 1;
array = lspb(initx,finalx,steps);
 
for i = 1:steps
    hand.model.base = transl(array(i),yVAL,zVAL);
    pose = hand.model.fkine(hand.model.getpos);
    pose = pose(1:3,4)';
    handVertex = handVertex + pose;
    handFace = cell2mat(hand.model.faces(1));
    handNormals = cell2mat(hand.faceNormals(1));
    
    hand.model.base()
    hand.model.animate(0);
  %   for i = 1 : 50
        for faceIndex = 1:size(handFace,1)
            vertOnPlane = handVertex(handFace(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(handNormals(faceIndex,:),vertOnPlane,...
                            [line.X(1),line.Y(1),(line.Z(1)-(i/10))],[line.X(2),line.Y(2),(line.Z(2)-(i/10))]);
            disp(check)
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,handVertex(handFace(faceIndex,:)',:))
                disp('HAND IN WORKSPACE !STOP!')
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*')
                intersect = true;
                return
            end
        end
   % end
    
end


%     if hand.model.base(1,1) == line.X(3)
%         display('hand detected in workspace, emergency stop')
%         pause
%     end

   