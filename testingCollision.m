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
%set up positions
targetpos = transl(0.5, 0.5, -0.5)
tempq = p560.ikcon(targetpos)
trajectory = jtraj(p560.getpos(), tempq ,50);

%set up collision checks
endeffectpos = p560.fkine(p560.getpos())
[X,Y] = meshgrid(-10:0.1:10,-10:0.1:10);
sizeMat = size(X);
Z = repmat(-0.2, sizeMat(1),sizeMat(2));
intersectionPlane = surf(X,Y,Z);
planePoints = [X(:),Y(:),Z(:)];

algebraicDistx = (planePoints(:,1)-endeffectpos(1,4)).^2;
algebraicDisty = (planePoints(:,2)-endeffectpos(2,4)).^2 ;
algebraicDistz = (planePoints(:,3)-endeffectpos(3,4)).^2

for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    p560.animate(q);
    endeffectpos = p560.fkine(p560.getpos())
    algebraicDistz = (planePoints(:,3)-endeffectpos(3,4)).^2
       if algebraicDistz < 0.01
        display(['COLLISION OCCURED']) 
        break
       end
       
    pause(0.001);
end







