close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

% Init Workspace
% startup_rvc;                                                                % Ensuring robotics toolbox is active and functional
floorOffset = (-1.0896/2);                                                  % measured height or table in body0.ply
objectOffset = transl(0, 0 , -0.1);                                         % offset to lower endeffector onto object 
workSpace = [-2.5 2.5 -2.5 2.5 (2*floorOffset) 2];
sawyerBase = transl(0,0,0);

% Init Sawyer
motion      = move();
sawyer1     = sawyer(workSpace, 1, sawyerBase);

% Init bodies
% table       = body(workSpace, 'table', transl(0,0,floorOffset));            % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
% bowl        = body(workSpace, 'Bowl', transl(0,0,0));
% mushroom    = body(workSpace, 'mushroom', transl(0.1,0,0));
% tomato      = body(workSpace, 'tomato', transl(0.2,0,0));
% carrot      = body(workSpace, 'carrot', transl(0.1,0.1,0));
% lettuce     = body(workSpace, 'lettuce', transl(0.2,0.1,0));
% onion       = body(workSpace, 'onion', transl(0.15,0.15,0));
% tomatosauce = body(workSpace, 'tomatosauce', transl(0.2,0.15,0));


