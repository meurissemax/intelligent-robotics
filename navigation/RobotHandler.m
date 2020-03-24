classdef RobotHandler < handle
    properties
        vrep
        h
        mesh_size
        id
        
        width
        length
        margin % Margin for the width and the length
        
        robot_abs_pos
        robot_abs_or
        prev_abs_pos
        prev_abs_or
        
        matrix_pos % Position in matrix
        map_pos % Position in the map
        
        sensors_pts % Sensors points and contacts as returned by the Hokuyo
        sensors_cts
        sensors_pts_abs % In absolute referential
    end
    
    methods (Access = public)
        
        % constructor
        function this = RobotHandler(vrep, h, id, mesh_size, robot_width, robot_length, margin)
            this.vrep = vrep;
            this.h = h;
            this.id = id;
            this.mesh_size = mesh_size;
            this.width = robot_width + margin;
            this.length = robot_length + margin;
            this.margin = margin;
            
            % Obtain position and orientation of the robot
            [res, youbotPos] = this.vrep.simxGetObjectPosition(this.id, this.h.ref, -1, this.vrep.simx_opmode_buffer);
            vrchk(this.vrep, res, true);
            [res, youbotEuler] = this.vrep.simxGetObjectOrientation(this.id, this.h.ref, -1, this.vrep.simx_opmode_buffer);
            vrchk(this.vrep, res, true);
            
            this.robot_abs_pos = youbotPos;
            this.robot_abs_or = youbotEuler;
            this.prev_abs_pos = youbotPos;
            this.prev_abs_or = youbotEuler;
            
            this.map_pos = robot_to_map_coord([youbotPos(1) youbotPos(2)]', this.mesh_size);
            this.matrix_pos = round(this.map_pos);
            
            this.get_sensors();
        end
        
        % Function used to read the latest sensors measurements from the
        % Hokuyo.
        function get_sensors(this)
            [this.sensors_pts, this.sensors_cts] = youbot_hokuyo(this.vrep, this.h, this.vrep.simx_opmode_buffer);
            % All points obtained in robot referential. Need to translate
            % them to absolute referential.
            %hokuyo_1 = [this.h.hokuyo1Pos(1); this.h.hokuyo1Pos(2)];
            %hokuyo_2 = [this.h.hokuyo2Pos(1); this.h.hokuyo2Pos(2)];
            
            %this.    NEEDED ???
            this.sensors_pts_abs = robot_to_abs(this.sensors_pts, this.robot_abs_pos, this.robot_abs_or(3));
        end
        
        % Function used to update the state of the robot at every iteration
        function update_state(this)
            [res, youbotPos] = this.vrep.simxGetObjectPosition(this.id, this.h.ref, -1, this.vrep.simx_opmode_buffer);
            vrchk(this.vrep, res, true);
            [res, youbotEuler] = this.vrep.simxGetObjectOrientation(this.id, this.h.ref, -1, this.vrep.simx_opmode_buffer);
            vrchk(this.vrep, res, true);
            
            % Updating position and orientation
            this.prev_abs_pos = this.robot_abs_pos;
            this.prev_abs_or = this.robot_abs_or;
            this.robot_abs_pos = youbotPos;
            this.robot_abs_or = youbotEuler;
            
            this.map_pos = robot_to_map_coord([youbotPos(1) youbotPos(2)]', this.mesh_size);
            this.matrix_pos = round(this.map_pos);
            
            this.get_sensors();
        end
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
    trans = [transl(1) * ones(1,n), transl(2) * ones(1,n)]; % Translation matrix
    
    abs_coords = rot * coord + trans; 
end