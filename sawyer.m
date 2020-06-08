classdef sawyer < handle % setup the UR3 robot
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;
        radius;
        defaultq;
    end
    
    methods
        function self = sawyer(workspace,roboNum, location)
            self.workspace = workspace;
            self.GetRobot(roboNum);
            self.defaultq = [90 270 5 180 5 180 90];
            self.defaultq = [5 180 90 180 90 180 180];
            self.defaultq = deg2rad(self.defaultq);
            self.currentJoints = self.defaultq;
            self.model.base = location;
            self.PlotAndColour(self.currentJoints);
            self.model.delay = 0.001;
        end
        
        function PlotAndColour(self, q)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['L',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(q,'workspace',self.workspace,'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
            end
            
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end    
        end
        
        function GetRobot(self, roboNum) % Setup Robot Parameters
            pause(0.001);
            L0 = Link('d',0.317,    'a',0.081,      'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(-345), deg2rad(345)]);
            L1 = Link('d',0.1925,   'a',0,          'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(90),   deg2rad(270)]);
            L2 = Link('d',0.4,      'a',0,          'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(-345), deg2rad(345)]);
            L3 = Link('d',0.1685,   'a',0,          'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(-345), deg2rad(345)]);
            L4 = Link('d',0.4,      'a',0,          'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(-345), deg2rad(345)]);
            L5 = Link('d',0.1363,   'a',0,          'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(-345), deg2rad(345)]);
            L6 = Link('d',0.13375,  'a',0,          'alpha',0,      'offset',0, 'qlim', [deg2rad(-345), deg2rad(535)]);
            pause(0.0001);
            name = ['Sawyer',num2str(roboNum)];
            self.model = SerialLink([L0 L1 L2 L3 L4 L5 L6], 'name', name);             
        end
               
        function [t] = limitCheck(self, jointAngles)
            joints = jointAngles;
            t = 1;
            [rows, columns] = size(joints);
            currentLink = transl(0,0,0);                                    % assuming robot is at the origin
            for joint = 1:columns
                currentLink = currentLink * self.model.A(joint,joints);
                if(currentLink(3,4) < 0)
                    t = 0;
                    return
                end
            end
        end
    end
end
