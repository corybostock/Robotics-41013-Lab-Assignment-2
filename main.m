close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

% Init Workspace
% startup_rvc;                                                                % Ensuring robotics toolbox is active and functional
floorOffset = (-1.0896/2);                                                  % measured height or table in body0.ply
objectOffset = transl(0, 0 , -0.1);                                         % offset to lower endeffector onto object 
workspace = [-2.5 2.5 -2.5 2.5 (2*floorOffset) 1];
% workspace = [-1.5 1.5 -1.5 1.5 0 1.5];

% Init bodies
% table       = body(workspace, 'table', transl(0,0,floorOffset));                  % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
% bowl        = body(workspace, 'Bowl', transl(0,0,0));
% mushroom    = body(workspace, 'mushroom', transl(0.1,0,0));
% tomato      = body(workspace, 'tomato', transl(0.2,0,0));

sawyer1     = sawyer(workspace, 1, transl(0,0,0));

sawyer1.model.teach();

motion      = move();