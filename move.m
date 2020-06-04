classdef move < handle % Movement of robots or bodies
    properties
    end
    
    methods
        function self = move()
           
        end
        
        function rmrcStartToEnd(self, robot, startPose, endPose)
            t = 20; % time * stepInterval                                                          % Total time (s)
            deltaT = 0.02;                                                  % Control frequency
            steps = t/deltaT;                                               % No. of steps for simulation
            lambdaThreshhold = 0.3;                                       % Threshold value for manipulability/Damped Least Squares
%             dampingValue = (1 - m(i)/lambdaThreshhold)*5E-2;
            W = diag([1 1 1 0 0 0]);                                  % Weighting matrix for the velocity vector

            % Allocate array data
            m               = zeros(steps,1);                               % Array for Measure of Manipulability
            qMatrix         = zeros(steps,robot.model.n);                   % Array for joint anglesR
            qdot            = zeros(steps,robot.model.n);                   % Array for joint velocities
            theta           = zeros(3,steps);                               % Array for roll-pitch-yaw angles
            x               = zeros(3,steps);                               % Array for x-y-z trajectory
            positionError   = zeros(3,steps);                               % For plotting trajectory error
            angleError      = zeros(3,steps);                               % For plotting trajectory error
            
            p1Mask = startPose(1:3,4);
            p2Mask = endPose(1:3,4);

            xs = lspb(p1Mask(1),p2Mask(1),steps);                           % Trapezoidal trajectory scalar
            ys = lspb(p1Mask(2),p2Mask(2),steps);                           % Trapezoidal trajectory scalar
            zs = lspb(p1Mask(3),p2Mask(3),steps);                           % Trapezoidal trajectory scalar
            
            % Set up trajectory, initial pose
            for i=1:steps
                x(1,i) = xs(i);                 % Points in x
                x(2,i) = ys(i);                 % Points in y
                x(3,i) = zs(i);                 % Points in z
                theta(1,i) = 180;               % Roll angle 
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            qMatrix(1,:) = robot.model.getpos();                                        % Solve joint angles to achieve first waypoint

            % Track the trajectory with RMRC
            for i = 1:steps-1
                T = robot.model.fkine(qMatrix(i,:));                                    % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = robot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < lambdaThreshhold                                                       % If manipulability is less than given threshold
                    lambda = (1 - m(i)/lambdaThreshhold)*5E-2;
                else
                    lambda = 0;
                end
                invJ_dls = inv(J'*J + lambda *eye(robot.model.n))*J';                       % DLS Inverse
                qdot(i,:) = (invJ_dls*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:robot.model.n                                                 % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)          % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)      % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            % Plot the results
            animateMatrix(self, robot, qMatrix);
        end
        
        function animateMatrix(self, robot, qMatrix)
            stepInterval = 10;
            for step = 1:stepInterval:size(qMatrix,1)
                q = qMatrix(step,:)
                robot.model.animate(q);
                pause(0.001);
            end
        end
        
        function rmrcToPointFromCurrent(self, robot, endPose)
            startPose = robot.model.fkine(robot.model.getpos());
            rmrcStartToEnd(self, robot, startPose, endPose);
        end
    end
end
