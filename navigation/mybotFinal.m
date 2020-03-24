function mybotFinal()

    % (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017, Mathieu Baijot 2017.
    % Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
   
    %% Initiate the connection to the simulator. 
    
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    % If you get an error like: 
    %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
    % Make sure your code is within a function! You cannot call V-REP from a script. 

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    % The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the 
    % file focused/youbot_arm.m). 
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or 
    % the robot pose with the simx_opmode_buffer option.
    pause(.2);

    %% Youbot constants
    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

    %% Preset values for the demo. 
    disp('Starting robot');
    
    % Define the preset pickup pose for this demo. 
    pickupJoints = [90 * pi / 180, 19.6 * pi / 180, 113 * pi / 180, - 41 * pi / 180, 0];

    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0; % Move straight ahead. 
    rightVel = 0; % Go sideways. 
    rotateRightVel = 0; % Rotate. 
    prevOrientation = 0; % Previous angle to goal (easy way to have a condition on the robot's angular speed). 
    prevPosition = 0; % Previous distance to goal (easy way to have a condition on the robot's speed). 

    % Set the arm to its starting configuration. 
    res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object. 
    vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error.
    
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);
    
    % Map and meshgrid initialization
    meshSize = 0.1; % Resolution of the mesh.
    [X, Y] = meshgrid(-8:meshSize:8,-8:meshSize:8);
    X = reshape(X, 1, []); % Make a vector of the matrices.
    Y = reshape(Y, 1, []);
    
    map = ones(length(X), length(Y)); % 0 : Free path, 1 : Unknown, 2 : Obstacle.
    
    % Path finding algorithm initialization
    dijkstra = Dijkstra(map);
    
    
    % Make sure everything is settled before we start. 
    pause(2);

    % Retrieve the position of the gripper. 
    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise the occupancyGrid
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    m = robotics.OccupancyGrid(30, 30, 4);
    
    % Initialise the state machine. 
    fsm = 'rotate';
    
    % Initial angle and position
    initAngle = youbotEuler(3);
    initX = youbotPos(1);
    initY = youbotPos(2);
    
    offsetX = 15;%-youbotPos(1) + 15; % May be incorrect
    offsetY = 15;%-youbotPos(2) + 15; % To make sure the robot starts at 15,15.

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        % Matrix to robot reference.
        offset = [15 15 0];
        to_robot = transl(youbotPos + offset) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
        
        
        
        %% Plot something if required. 
        if plotData
            % Read data from the depth sensor, more often called the Hokuyo (if you want to be more precise about 
            % the way you control the sensor, see later for the details about this line or the file 
            % focused/youbot_3dpointcloud.m).
            % This function returns the set of points the Hokuyo saw in pts. contacts indicates, for each point, if it
            % corresponds to an obstacle (the ray the Hokuyo sent was interrupted by an obstacle, and was not allowed to
            % go to infinity without being stopped). 
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, to_robot);
            
            % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo (it returns the area that
            % is visible, but the visualisation draws a series of points that are within this visible area). 
            in = inpolygon(X, Y,...
                           [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
                           [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);

            % Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
            % visibility range from the Hokuyo sensor. 
            % The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
            % sensor position. 
            
            
            % Previous angle
            diff = youbotEuler(3) - initAngle;
            trans1 = youbotPos(1) - initX;
            trans2 = youbotPos(2) - initY;
            
            X(in) = X(in) + 15;
            Y(in) = Y(in) + 15;
            
            setOccupancy(m, [transpose(pts(1,:)), transpose(pts(2,:))], contacts);
            setOccupancy(m, [transpose(X(in)), transpose(Y(in))], 0);
            
            X(in) = X(in) - 15;
            Y(in) = Y(in) - 15;
            
            % Iterate all points and map them
            %{
            for point = 1:size(contacts, 2)
                x = pts(1,point) * cos(diff) - pts(2,point) * sin(diff) + trans1 + offsetX;
                y = pts(1,point) * sin(diff) + pts(2,point) * cos(diff) + trans2 + offsetY;
                setOccupancy(m, [x y], contacts(point));
            end

            for point = 1:size(in,2)
                x = X(point) * cos(diff) - Y(point) * sin(diff) + trans1 + offsetX;
                y = X(point) * sin(diff) + Y(point) * cos(diff) + trans2 + offsetY;
                setOccupancy(m, [x y], 0);
            end
            %}
            
            show(m);
            
        end
        angl = -pi/2;
            
        %contour(m);
        show(m);
        drawnow;

        %% Apply the state machine. 
        if strcmp(fsm, 'rotate')
            %% First, rotate the robot to go to one table.             
            % The rotation velocity depends on the difference between the current angle and the target. 
            rotateRightVel = angdiff(angl, youbotEuler(3));
            
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
                rotateRightVel = 0;
                fsm = 'drive';
            end
            
            prevOrientation = youbotEuler(3);
        elseif strcmp(fsm, 'drive')
            %% Then, make it move straight ahead until it reaches the table (x = 3.167 m). 
            % The further the robot, the faster it drives. (Only check for the first dimension.)
            % For the project, you should not use a predefined value, but rather compute it from your map. 
            forwBackVel = - (youbotPos(1) + 3.167);

            % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
            % a specific location before moving on to the next state.
            if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevPosition) < .001)
                forwBackVel = 0;
                
                % Change the orientation of the camera to focus on the table (preparation for next state). 
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, pi/4], vrep.simx_opmode_oneshot);
                
                % Move the arm to the preset pose pickupJoints (only useful for this demo; you should compute it based
                % on the object to grasp). 
                for i = 1:5
                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
                                                          vrep.simx_opmode_oneshot);
                    vrchk(vrep, res, true);
                end

                fsm = 'snapshot';
            end
            prevPosition = youbotPos(1);
        end

end % main function
   
