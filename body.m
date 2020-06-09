classdef body < handle % class to handle setting up of the static body
    properties
        model;
        plyData;
        workspace;
        location;
        faceNormals;
        faces;
        vertex;
        normals;
        floorOffset;
    end
    
    methods
        function self = body(workspace, name, location, floorOffset)
            self.floorOffset = floorOffset;
            self.model.delay = 0;
            self.plotAndColour(workspace, name, location);            
        end
               
        function plotAndColour(self, workspace, name, location)
            for linkIndex = 0:1
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([name,'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            L1 = Link('alpha',0,'a',0.2,'d',0.2,'offset',0);
            self.model = SerialLink(L1, 'name' , name);
            % 1 link robot used to simulate bodys for simplicity
            self.model.faces = {faceData,[]};
            self.model.points = {vertexData,[]};
            
            % Display body
            self.model.base = location;
            self.model.plot3d(0, 'workspace', workspace);
            
            %finding the normals to faces for collision checking
                   
            self.faceNormals{1} = zeros(size(faceData,1),3);
            for faceIndex = 1:size(faceData,1)
                v1 = vertexData(faceData(faceIndex,1)',:);
                v2 = vertexData(faceData(faceIndex,2)',:);
                v3 = vertexData(faceData(faceIndex,3)',:);
                self.faceNormals{1}(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
            self.faces = cell2mat(self.model.faces(1));
            self.vertex = cell2mat(self.model.points(1));
            self.vertex(:,3) = self.vertex(:,3)+self.floorOffset;
            self.normals = cell2mat(self.faceNormals(1));
            
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
    end
end
