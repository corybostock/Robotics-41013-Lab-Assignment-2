function [collisionResponse, noCollision] = CollisionReact(robot,faces, vertex, normals, trajectory)
%implements the isCollision class to come to a halt before a collision
%happens
for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    IsCollision(robot.model, q, faces, vertex, normals)

    if IsCollision(robot.model, q, faces, vertex, normals) 
        collisionResponse = 'Collsion avoided';
        return
    else
        noCollision = 'all g to keep moving'; 
    end
    
    robot.model.animate(q);
end
end

