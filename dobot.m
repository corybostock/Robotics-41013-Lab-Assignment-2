classdef dobot < handle % setup the UR3 robot
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;
        radius;
    end
    
    methods
        function self = dobot(workspace,roboNum, location)
            self.workspace = workspace;
            self.GetRobot(roboNum);
            self.currentJoints = zeros(1,6);
            self.model.base = location;
            self.PlotAndColour();
            
        end
        function PlotAndColour(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri');
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
            L1 = Link('d',0.135,'a',0,'alpha',pi/2,'offset',0, 'qlim', [deg2rad(-135), deg2rad(135)])
            L2 = Link('d',0,'a',0.147,'alpha',0,'offset',-pi/2, 'qlim', [deg2rad(5), deg2rad(80)])
            L3 = Link('d',0,'a',0.1,'alpha',0,'offset',0, 'qlim', [deg2rad(-5), deg2rad(90)])
            L4 = Link('d',0,'a',0.05,'alpha',pi/2,'offset',0, 'qlim', [deg2rad(-90), deg2rad(90)])
            L5 = Link('d',0,'a',0,'alpha',0,'offset',0, 'qlim', [deg2rad(-85), deg2rad(85)])
           
                        
            pause(0.0001)
            name = ['Dobot',num2str(roboNum)];
            self.model = SerialLink([L1 L2 L3 L4 L5], 'name', name);             
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
