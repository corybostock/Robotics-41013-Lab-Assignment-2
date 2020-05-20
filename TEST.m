L1 = Link('d',81,'a',317,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L2 = Link('d',0,'a',192.5,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L3 = Link('d',0,'a',400,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L4 = Link('d',0,'a',168.5,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L5 = Link('d',0,'a',400,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L6 = Link('d',0,'a',136.3,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L7 = Link('d',0,'a',133.75,'alpha',0,'offset',0, 'qlim', [deg2rad(0), deg2rad(535)])



sawyer = SerialLink([L1 L2 L3 L4 L5 L6 L7]);

q = [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161]

sawyer.teach(q)

%         x: 0.450635802326
%         y: 0.161618483757
%         z: 0.215599073475

%%
%% Puma 560 DH Model
% 41013 Robotics
% Jonathan Woolfrey
% August 2016

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

L1 = Link('d',81,'a',317,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L2 = Link('d',0,'a',192.5,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L3 = Link('d',0,'a',400,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L4 = Link('d',0,'a',168.5,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L5 = Link('d',0,'a',400,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L6 = Link('d',0,'a',136.3,'alpha',-pi/2,'offset',0, 'qlim', [deg2rad(0), deg2rad(345)])
L7 = Link('d',0,'a',133.75,'alpha',0,'offset',0, 'qlim', [deg2rad(0), deg2rad(535)])

myRobot = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'sawyer')

q = [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161]

myRobot.plot(q)

myRobot.teach;
q = myRobot.getpos();
j = myRobot.jacob0(q);
y = inv(j);
