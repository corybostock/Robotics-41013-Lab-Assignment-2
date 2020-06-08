function result = CollisionAvoid(robot,qWaypoints, bodies, iscollision, q2, qMatrix , checkedTillWaypoint)
%finds a way to move to avoid colliding with specified objects
while (iscollision)

	for j  = 1:size(bodies,2)

    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(robot.model,qMatrixJoin,bodies(j).faces,bodies(j).vertex,bodies(j).normals)
            qMatrix = [qMatrix; qMatrixJoin];
            robot.model.animate(qMatrixJoin);
            size(qMatrix)
            iscollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(robot.model,qMatrixJoin,bodies(j).faces,bodies(j).vertex,bodies(j).normals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            %qRand = (2 * rand(1,3) - 1) * pi;
            %while IsCollision(robot.model,qMatrixJoin,bodies(j).faces,bodies(j).vertex,bodies(j).normals)
            %    qRand = (2 * rand(1,3) - 1) * pi;
            %end
            %qWaypoints = [ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            %iscollision = true;
            %break;
        end
        
    end
    %result = 'reached goal position'
	
	end
	
end
end
