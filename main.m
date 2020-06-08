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
       workSpace = [-1 1 -1 1 -0.1 1.5]; 
       axis(workSpace);
       hold on;
       
       sawyerBase              = transl(0,0,0);
       sawywerInit             = transl(0.2, 0, 1.2);
       bowlBaseCoord           = transl(-0.4, 0.5, 0);
       knifeCoord              = transl(0, -0.5, 0.42) * trotx(deg2rad(0)) * troty(deg2rad(90));
       mushroomBaseCoord       = transl(0.6, -0.4, 0.0);
       tomatoBaseCoord         = transl(0.6, 0.0, 0.0);
       carrotBaseCoord         = transl(0.6, 0.4, 0.0);
       lettuceBaseCoord        = transl(0.4, 0.5, 0.0);
       onionBaseCoord          = transl(0.0, 0.6, 0.0);
       tomatoSauceBaseCoord    = transl(-0.2, 0.5 ,0.0);

       % Init Sawyer
       sawyer1     = sawyer(workSpace, 1, sawyerBase);
       
       
       % Init bodies
       table       = body(workSpace, 'table',       transl(0,0,floorOffset),    floorOffset);     % Dimensions of the table (x, y, z) = (1.4880, 2.3383, 1.0896)
       bowl        = body(workSpace, 'bowlimproved',bowlBaseCoord,              floorOffset);
       knife       = body(workSpace, 'knife',       knifeCoord,                 floorOffset);
       mushroom    = body(workSpace, 'mushroom',    mushroomBaseCoord,          floorOffset);
       tomato      = body(workSpace, 'tomato',      tomatoBaseCoord,            floorOffset);
       carrot      = body(workSpace, 'carrot',      carrotBaseCoord,            floorOffset);
       lettuce     = body(workSpace, 'lettuce',     lettuceBaseCoord,           floorOffset);
       onion       = body(workSpace, 'onion',       onionBaseCoord,             floorOffset);
       tomatoSauce = body(workSpace, 'tomatosauce', tomatoSauceBaseCoord,       floorOffset);
       
       bodies = [table, bowl, knife, mushroom, tomato, carrot, lettuce, onion, tomatoSauce];
       motion = move(sawyer1, bodies);
       
       motion.rmrcToPointFromCurrent(sawyer1, knifeCoord);
       motion.rmrcToPointFromCurrentWBodies(sawyer1, sawywerInit, [knife]);
       
       % Cutting Mushroom
       motion.cutVeg(sawyer1, [knife], mushroomBaseCoord, transl(-0.1, 0.3, 0.27) * trotz(deg2rad(90)), transl(-0.1, 0.3, 0.35) * trotz(deg2rad(90)), transl(-0.1, 0.2, 0.35) * trotz(deg2rad(90)), transl(-0.1, 0.3, 0.27) * trotz(deg2rad(90)));
       
       % Cutting Tomato
       motion.rmrcToPointFromCurrentWBodies(sawyer1, tomatoBaseCoord * transl(0,0,0.4), [knife]);
       motion.cutVeg(sawyer1, [knife], tomatoBaseCoord,   transl(-0.1, 0.3, 0.27) * trotz(deg2rad(90)), transl(-0.1, 0.3, 0.35) * trotz(deg2rad(90)), transl(-0.1, 0.05, 0.35) * trotz(deg2rad(90)), transl(-0.1, 0.3, 0.27) * trotz(deg2rad(90)));
       
       % Cutting Carrot
       
       
       % Cutting Lettuce
       
       
       % Cutting Onion
       
       
       % Dressing
       

end
