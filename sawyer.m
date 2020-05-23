classdef sawyer < handle % setup the UR3 robot
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;
        radius;
    end
    
    methods
        function self = sawyer(workspace,roboNum, location)
            self.workspace = workspace;
            self.GetRobot(roboNum);
            q = [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161];
            self.currentJoints = q;
            self.model.base = location;
            self.PlotAndColour();
            
        end
        function PlotAndColour(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['L',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.currentJoints,'workspace',self.workspace,'floorlevel', 0);
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
            L1 = Link('d',0.081,    'a',0.317,      'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(5), deg2rad(345)]);
            L2 = Link('d',0,        'a',0.1925,     'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(5), deg2rad(345)]);
            L3 = Link('d',0,        'a',0.400,      'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(5), deg2rad(345)]);
            L4 = Link('d',0,        'a',0.1685,     'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(5), deg2rad(345)]);
            L5 = Link('d',0,        'a',0.400,      'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(5), deg2rad(345)]);
            L6 = Link('d',0,        'a',0.1363,     'alpha',-pi/2,  'offset',0, 'qlim', [deg2rad(5), deg2rad(345)]);
            L7 = Link('d',0,        'a',0.13375,    'alpha',0,      'offset',0, 'qlim', [deg2rad(5), deg2rad(535)]);
            pause(0.0001)
            name = ['Sawyer',num2str(roboNum)];
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', name);             
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
