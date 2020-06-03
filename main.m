close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf

clear

% Init Workspace
% startup_rvc;                                                                % Ensuring robotics toolbox is active and functional

mode = 1;
switch (mode)
    case 0
        run('gui.mlapp');        
    case 1
       floorOffset = (-1.0896/2);                                           % measured height or table in table.ply
       objectOffset = transl(0, 0 , -0.1);                                  % offset to lower endeffector onto object 
       workSpace = [-2.5 2.5 -2.5 2.5 (2*floorOffset) 2];
       %workSpace = [-1 1 -1 1 -0.1 1]; 
       axis(workSpace);
       hold on;
       
       sawyerBase              = transl(0,0,0);
       bowlBaseCoord           = transl(-0.4, 0.5, 0);
       mushroomBaseCoord       = transl(0.6, -0.4, 0.0);
       tomatoBaseCoord         = transl(0.6, 0.0, 0.0);
       carrotBaseCoord         = transl(0.6, 0.4, 0.0);
       lettuceBaseCoord        = transl(0.4, 0.5, 0.0);
       onionBaseCoord          = transl(0.0, 0.6, 0.0);
       tomatoSauceBaseCoord    = transl(-0.2, 0.5 ,0.0);

       % Init Sawyer
       motion      = move();
       sawyer1     = sawyer(workSpace, 1, sawyerBase);
       
       motion.rmrcToPointFromCurrent(sawyer1, bowlBaseCoord);
       motion.rmrcToPointFromCurrent(sawyer1, mushroomBaseCoord);

       % Init bodies
%        table       = body(workSpace, 'table',       transl(0,0,floorOffset));     % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
%        bowl        = body(workSpace, 'Bowl',        bowlBaseCoord);
%        mushroom    = body(workSpace, 'mushroom',    mushroomBaseCoord);
%        tomato      = body(workSpace, 'tomato',      tomatoBaseCoord);
%        carrot      = body(workSpace, 'carrot',      carrotBaseCoord);
%        lettuce     = body(workSpace, 'lettuce',     lettuceBaseCoord);
%        onion       = body(workSpace, 'onion',       onionBaseCoord);
%        tomatoSauce = body(workSpace, 'tomatosauce', tomatoSauceBaseCoord);
       
end
