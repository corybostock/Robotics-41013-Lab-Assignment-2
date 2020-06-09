function CollisionReact(robot,faces, vertex, normals, trajectory)
%implements the isCollision class to come to a halt before a collision
%happens
for step = 1:size(trajectory,1)
    q = trajectory(step,:);
    IsCollision(robot.model, q, faces, vertex, normals)

    if IsCollision(robot.model, q, faces, vertex, normals) 
        display( 'Collsion avoided');
        return
    else
        display( 'all g to keep moving'); 
    end
    
   % robot.model.animate(q);
end
end

