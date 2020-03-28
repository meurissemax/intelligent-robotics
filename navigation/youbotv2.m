function youbotv2()

    % (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017, Mathieu Baijot 2017.
    % Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
    
    %% Variables
    robot_width = 0.4; % https://www.generationrobots.com/img/Kuka-YouBot-Technical-Specs.pdf
    robot_height = 0.6;
    margin = 0.05; % Margin for the width and the height
   
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
    
    % Define the preset pickup pose for  demo. 
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
    [X, Y] = meshgrid(-5:meshSize:5,-5:meshSize:5);
    X = reshape(X, 1, []); % Make a vector of the matrices.
    Y = reshape(Y, 1, []);

    map = robotics.OccupancyGrid(30,30,10);
    edges = {};
    
    % Make sure everything is settled before we start. 
    pause(2);
    
    % Initialise the state machine. 
    fsm = 'rotate';
    
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    %initAngle = youbotEuler(3);
    %initX = youbotPos(1);
    %initY = youbotPos(2);
    
    initPos = youbotPos - [15 15 0];
    initA1 = youbotEuler(1);
    initA2 = youbotEuler(2);
    initA3 = youbotEuler(3);
    
    %% Start the demo. 
    while true
        tic
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        pos = youbotPos - initPos;
        A1 = youbotEuler(1) - initA1;
        A2 = youbotEuler(2) - initA2;
        A3 = youbotEuler(3) - initA3;
        
        % Transformation to robot absolute position
        trf = transl(pos) * trotx(A1) * troty(A2) * trotz(A3);
        
        % Point obtention
        [pts, cts] = youbot_hokuyo(vrep,h, vrep.simx_opmode_buffer);
        simplifiedPoly = simplify_polygon([pts(1,:); pts(2,:)]);
        
        % Ajouter hokuyo dans les points ?
        %plot(simplifiedPoly(1,:),simplifiedPoly(2,:),'.b','markersize', 10);
        %hold on;
        
        
        in = inpolygon(X, Y, simplifiedPoly(1,:), simplifiedPoly(2,:));
        %plot(X(in),Y(in),'.b','markersize', 10);
        
        % Transforming inside points to absolute reference
        inValue = homtrans(trf, [X(in); Y(in) ; zeros(1,size(X(in),2))]);
        inValue = transpose([inValue(1,:); inValue(2,:)]);
        
        setOccupancy(map, inValue, 0);
        plot(round(inValue(:,1),1),round(inValue(:,2),1),'.b','markersize', 10);
        hold on;

        % Transforming pts to absolute reference
        inPts = homtrans(trf, [pts(1,cts); pts(2,cts) ; zeros(1,size(pts(1,cts),2))]);
        inPts = transpose([inPts(1,:); inPts(2,:)]);
        setOccupancy(map, inPts, 1); 
        %show(map);
        %drawnow;
        plot(round(inPts(:,1),1),round(inPts(:,2),1),'.r','markersize', 10);
        hold on;
        %if strcmp(fsm, "exploration")
            
            %% Detect the edges
            mcopy = copy(map);
            inflate(mcopy,0.2);
            occMat = flipud(occupancyMatrix(mcopy, 'ternary'));
            edges = {};

            for x = 2:size(occMat, 1)-1
                for y = 2:size(occMat,2)-1
                    if occMat(x,y) == 0 || occMat(x,y) == -1
                        continue;
                    end
                    
                    % Faire 8 connectivity ?
                    if occMat(x,y-1) == 1 && occMat(x-1,y) == 1 && occMat(x,y+1) == 0 && occMat(x+1,y) == 0
                        edges(end+1) = {[x+1 y+1]};
                    elseif occMat(x,y-1) == 0 && occMat(x-1,y) == 1 && occMat(x,y+1) == 1 && occMat(x+1,y) == 0
                        edges(end+1) = {[x+1 y-1]};
                    elseif occMat(x,y-1) == 1 && occMat(x-1,y) == 0 && occMat(x,y+1) == 0 && occMat(x+1,y) == 1
                        edges(end+1) = {[x-1 y+1]};
                    elseif occMat(x,y-1) == 0 && occMat(x-1,y) == 0 && occMat(x,y+1) == 1 && occMat(x+1,y) == 1
                        edges(end+1) = {[x-1 y-1]};
                    end
                end
            end
            
            %% Detect closest point to explore
            dist = 100;
            nextPoint = [0 0];
            for x = 2:size(occMat, 1)-1
                for y = 2:size(occMat,2)-1
                    if occMat(x,y) ~= -1
                        continue;
                    end
                    if occMat(x-1,y-1) == 0 || occMat(x,y+1) == 0 || occMat(x, y-1) == 0 || occMat(x+1,y+1) == 0
                        if pdist([y/10 x/10; pos(1) pos(2)],'euclidean') < dist
                            nextPoint = [y/10 x/10]; % Vérifier que c'est le bon sens.
                        end
                    end
                end
            end
            disp(nextPoint);
            pause(100);
            
        %end
        %contour(occMat);
        %drawnow;
        plotdata = 1;
        if plotdata
            for i = 1:size(edges,2)
                plot(edges{i}(2)/10,edges{i}(1)/10,'.g','markersize', 10);
            end
        end
        
        %% Je suis là
        
        angl = -pi/2;
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
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
                
                % Move the arm to the preset pose pickupJoints (only useful for  demo; you should compute it based
                % on the object to grasp). 
                for i = 1:5
                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
                                                          vrep.simx_opmode_oneshot);
                    vrchk(vrep, res, true);
                end

                fsm = 'snapshot';
            end
            prevPosition = youbotPos(1);
        elseif strcmp(fsm, 'snapshot')
            %% Read data from the depth camera (Hokuyo)
            % Reading a 3D image costs a lot to VREP (it has to simulate the image). It also requires a lot of 
            % bandwidth, and processing a 3D point cloud (for instance, to find one of the boxes or cylinders that 
            % the robot has to grasp) will take a long time in MATLAB. In general, you will only want to capture a 3D 
            % image at specific times, for instance when you believe you're facing one of the tables.

            % Reduce the view angle to pi/8 in order to better see the objects. Do it only once. 
            % ^^^^^^     ^^^^^^^^^^    ^^^^                                     ^^^^^^^^^^^^^^^ 
            % simxSetFloatSignal                                                simx_opmode_oneshot_wait
            %            |
            %            rgbd_sensor_scan_angle
            % The depth camera has a limited number of rays that gather information. If  number is concentrated 
            % on a smaller angle, the resolution is better. pi/8 has been determined by experimentation. 
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again. 
            % ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            % simxSetIntegerSignal          1        simx_opmode_oneshot_wait
            %         |
            %         handle_xyz_sensor
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Then retrieve the last point cloud the depth sensor took.
            % If you were to try to capture multiple images in a row, try other values than 
            % vrep.simx_opmode_oneshot_wait. 
            fprintf('Capturing a point cloud...\n');
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
            % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
            % the output data. To get a correct plot, you should invert the y and z dimensions. 
    
            % Here, we only keep points within 1 meter, to focus on the table. 
            pts = pts(1:3, pts(4, :) < 1);

            if plotData
                subplot(223)
                plot3(pts(1, :), pts(3, :), pts(2, :), '*');
                axis equal;
                view([-169 -46]);
            end

            % Save the point cloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
            fileID = fopen('pc.xyz','w');
            fprintf(fileID,'%f %f %f\n', pts);
            fclose(fileID);
            fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));

            %% Read data from the RGB camera
            % This starts the robot's camera to take a 2D picture of what the robot can see. 
            % Reading an image costs a lot to VREP (it has to simulate the image). It also requires a lot of bandwidth, 
            % and processing an image will take a long time in MATLAB. In general, you will only want to capture 
            % an image at specific times, for instance when you believe you're facing one of the tables or a basket.

            % Ask the sensor to turn itself on, take A SINGLE IMAGE, and turn itself off again. 
            % ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            % simxSetIntegerSignal          1        simx_opmode_oneshot_wait
            %         |
            %         handle_rgb_sensor
            res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            % Then retrieve the last picture the camera took. The image must be in RGB (not gray scale). 
            %      ^^^^^^^^^^^^^^^^^^^^^^^^^     ^^^^^^                            ^^^
            %      simxGetVisionSensorImage2     h.rgbSensor                       0
            % If you were to try to capture multiple images in a row, try other values than 
            % vrep.simx_opmode_oneshot_wait. 
            fprintf('Capturing image...\n');
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

            % Finally, show the image. 
            if plotData
                subplot(224)
                imshow(image);
                drawnow;
            end

            % Next state. 
            fsm = 'extend';
        elseif strcmp(fsm, 'extend')
            %% Move the arm to face the object.
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % If the arm has reached the wanted position, move on to the next state. 
            % Once again, your code should compute  based on the object to grasp. 
            if norm(tpos - [0.3259, -0.0010, 0.2951]) < .002
                % Set the inverse kinematics (IK) mode to position AND orientation. 
                res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                fsm = 'reachout';
            end
        elseif strcmp(fsm, 'reachout')
            %% Move the gripper tip along a line so that it faces the object with the right angle.
            % Get the arm tip position. The arm is driven only by the position of the tip, not by the angles of 
            % the joints, except if IK is disabled.
            % Following the line ensures the arm attacks the object with the right angle. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);

            % If the tip is at the right position, go on to the next state. Again,  value should be computed based
            % on the object to grasp and on the robot's position. 
            if tpos(1) > .39
                fsm = 'grasp';
            end

            % Move the tip to the next position along the line. 
            tpos(1) = tpos(1) + .01;
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        elseif strcmp(fsm, 'grasp')
            %% Grasp the object by closing the gripper on it.
            % Close the gripper. Please pay attention that it is not possible to adjust the force to apply:  
            % the object will sometimes slip from the gripper!
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            % Make MATLAB wait for the gripper to be closed. This value was determined by experiments. 
            pause(2);
            
            % Disable IK;  is used at the next state to move the joints manually. 
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fsm = 'backoff';
        elseif strcmp(fsm, 'backoff')
            %% Go back to rest position.
            % Set each joint to their original angle, as given by startingJoints. Please note that  operation is not
            % instantaneous, it takes a few iterations of the code for the arm to reach the requested pose. 
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            
            % Get the gripper position and check whether it is at destination (the original position).
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            if norm(tpos - homeGripperPosition) < .02
                % Open the gripper when the arm is above its base. 
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
            end
            
            if norm(tpos - homeGripperPosition) < .002
                fsm = 'finished';
            end
        elseif strcmp(fsm, 'finished')
            %% Demo done: exit the function. 
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end

        % Update wheel velocities using the global values (whatever the state is). 
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);

        % Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end

end % main function

% Taken from https://www.youtube.com/watch?v=b7t6qkfPKuI
% Returns a simplified polygon from 'polygon'
%
% 'polygon' is a list of coordinates
% 'new_mesh' is a list of coordinates
function new_mesh = simplify_polygon(polygon)
number_it = 0;
while true
    number_change = 0;
    new_mesh = zeros(2,size(polygon,2));
    polygon = horzcat(polygon, polygon(:,2));
    size_p = size(polygon,2);
    i = 1;
    k = 1;
    while i < size_p - 1
        p1 = polygon(:,i);
        p2 = polygon(:,i+1);
        
        if norm(p2-p1) < 0.05
            i = i+2;
            has_small = true;
        else
            p3 = polygon(:,i+2);
            
            v1 = p2-p1;
            v2 = p3-p2;
            v1 = [v1(1) v1(2) 0]/norm(v1);
            v2 = [v2(1) v2(2) 0]/norm(v2);
            
            area = cross(v1,v2);
            area = abs(area(3));
            
            if area < 0.15
                i = i+2;
                number_change = number_change + 1;
            else
                i = i + 1;
            end
        end
        new_mesh(:,k) = p1;
        k = k+1;
    end
    
    new_mesh(:,k) = new_mesh(:,1);
    new_mesh = new_mesh(:,1:k);
    
    polygon = new_mesh;
    
    if number_change < 2 || number_it > 7
        break;
    end
    number_it = number_it+1;
end
end

% As we have positive integer indices in the map, we must translate them by 
% shifting by 8 and multiplying by the resolution. (and adding 1 as there
% are no 0 in matlab indices). robot_coord is a matrix of size 2xN.
function map_coord = robot_to_map_coord(robot_coord, mesh_size)
    mesh_mult = 1/mesh_size;
    length = size(robot_coord,2);
    trans = 8 * ones(2,length);
    map_coord = (robot_coord + trans) * mesh_mult + ones(2, length);
end

% Function used to express points in robot referential to absolute
% referential.
function abs_coords = robot_to_abs(coord, transl, angle)
    n = size(coord, 2); % Number of points.
    rot = [cos(angle) -sin(angle); sin(angle) cos(angle)]; % Rotation matrix
    trans = [transl(1) * ones(1,n); transl(2) * ones(1,n)]; % Translation matrix
    abs_coords = rot * coord + trans;
end

% Provides the index from the coordinate. Same principle as
% robot_to_map_coord.
function idx_abs = index_abs(coord_abs, mesh_size)
    mesh_mult = 1/mesh_size;
    length = size(coord_abs,2);
    trans = 8 * ones(2,length);
    idx_abs = round((coord_abs + trans) * mesh_mult + ones(2, length));
end

function bool = isEdge(map, x, y)
    try 
        count = 0; % Must == 3 for edge.
        bool = 0;
        for i = x-1:x+1
          for j = y-1:y+1
             if x == i && j == y
                 continue
             end
             if map(i,j) == 1
                 count = count + 0;
             end
          end
        end
        disp(count);
        if count == 3
            bool = 1;
        end
    catch
    end
end
   
