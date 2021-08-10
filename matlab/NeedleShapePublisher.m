%% NeedleShapePublisher.m
%
% class to publish the needle shape in ROS 2
%
% TODO
%   - join the needle parameters and the sensing stuff together into 1
%       struct
%   - subscriber to needle insertion length
%   - subscriber to theta0
%   - needle shape with rotation
%
% - written by: Dimitri Lezcano

classdef NeedleShapePublisher < handle
    properties
        % ROS 2 stuff
        node                    ros2node;
        current_pub             ros2publisher;
        predicted_pub           ros2publisher;
        sensor_sub              ros2subscriber;
        needlepose_sub          ros2subscriber;
        entrypoint_sub          ros2subscriber;
        sub_timeout             double {mustBePositive} = 20;
        % FBG Parameters
        num_samples             {mustBeInteger, mustBePositive} = 200; % number of samples to be gathered
        needleLength            double {mustBePositive}; % length of the entire needle
        num_channels            {mustBeInteger, mustBePositive};
        num_activeAreas         {mustBeInteger, mustBePositive};
        sensorLocations         (1,:) {mustBePositive}; % from the tip
        sensorCalMatrices       (2,:,:);
        aaReliabilityWeights    (1,:);
        % shape sensing parameters
        needleMechParams        struct;
        current_L               double = 0;
        kc_i                    double = 0.0025;
        w_init_i                (3,1)  = [0.0025; 0; 0];
        current_entry_point            = [];
    end
    methods
        % constructor
        function obj = NeedleShapePublisher(num_chs, num_aas, slocs, calMats, ...
                            needle_length, needle_mech_params, options)
            arguments
                num_chs {mustBeInteger, mustBePositive};
                num_aas {mustBeInteger, mustBePositive};
                slocs (1,:); % from the tip
                calMats (2,:,:);
                needle_length double {mustBePositive};
                needle_mech_params struct; % mechanical properties of th needle
                % ros options
                options.ns string = '/needle';
                options.node_name string = '/NeedleShape';
                options.num_samples {mustBeInteger, mustBePositive} = 200;
                options.timeout {mustBePositive} = 20;
                % shape sensing options
                options.kc_i double = 0.0025;
                options.w_init_i (3,1) = [0.0025; 0; 0];
            end
            % argument checking
            assert(numel(slocs) == num_aas);
            assert(size(calMats,1) == 2); % kx, ky
            assert(size(calMats,2) == num_chs); % CH1, CH2, ...
            assert(size(calMats,3) == num_aas); % AA1, AA2, ...
            
            % configure needle parameters
            obj.needleMechParams = needle_mech_params;
            obj.needleLength = needle_length;
            obj.num_channels = num_chs;
            obj.num_activeAreas = num_aas;
            obj.sensorLocations = slocs;
            obj.num_samples = options.num_samples;
            obj.sensorCalMatrices = calMats;
            
            % initial kc and w_init
            obj.kc_i = options.kc_i;
            obj.w_init_i = options.w_init_i;            
            
            % configure node, publishers and subscribers
            node_name = strcat(options.ns, options.node_name);
            curr_pub_name = strcat(options.ns, '/state/shape');
            pred_pub_name = strcat(options.ns, '/state/predicted_shape');
            sensor_sub_name = strcat(options.ns, '/sensor/processed');
            needlepose_sub_name = '/stage/state/needle_pose';
            entrypoint_sub_name = '/subject/state/skin_entry';
            
            % configure node, publishers and subscibers
            obj.node = ros2node(node_name);
            obj.current_pub    = ros2publisher(obj.node, curr_pub_name,...
                                                'geometry_msgs/PoseArray');
            obj.predicted_pub  = ros2publisher(obj.node, pred_pub_name,...
                                                'geometry_msgs/PoseArray');
                                            
            obj.sensor_sub     = ros2subscriber(obj.node, sensor_sub_name,...
                                                'std_msgs/Float64MultiArray');
            obj.needlepose_sub = ros2subscriber(obj.node, needlepose_sub_name, @obj.needlepose_cb,...
                                                'geometry_msgs/PoseStamped'); 
            obj.entrypoint_sub = ros2subscriber(obj.node, entrypoint_sub_name, @obj.skin_entry_cb,...
                                                'geometry_msgs/PointStamped');
            obj.sub_timeout = options.timeout;
        end
        
        % generate the air portion of the needle (cubic)
        function [pmat, Rmat] = generate_air_needle(obj, entrypt)
           pmat = [];
           Rmat = [];
           if isempty(entrypt)
               return;
           end
           
           % get the coefficients of the quadratic
           if entrypt(3) > 0
%                a_x = entrypt(1)/entrypt(3)^2; % quadratic
%                a_y = entrypt(2)/entrypt(3)^2; % quadratic
               a_x = entrypt(1)/(-2 * entrypt(3)^2); % cubic
               a_y = entrypt(2)/(-2 * entrypt(3)^2); % cubic
           
               % generate the quadratic needle shape in lots of points
               z = reshape(linspace(0, entrypt(3), 100), 1, []);
%                pts = [a_x * z.^2, a_y * z.^2, z]; % quadratic
               pts = [a_x * z.^3 - 3 * a_x * entrypt(3) * z.^2;
                      a_y * z.^3 - 3 * a_y * entrypt(3) * z.^2;
                      z]'; % cubic
               L_air = arclength(pts);
               ds = 0.5;
               s_interp = 0:ds:L_air;
               
               
               pmat = interp_pts(pts, s_interp)'; % 0.5 mm increments
               
               % generate rotation matrices 
               Rmat = repmat(eye(3), 1, 1, size(pmat,2));
               for i = 2:size(pmat,2)
                   dz = pmat(3,i) - pmat(3,i-1);
                   
                   % compute the tangent vector
                   tv = [pmat(1:2, i) - pmat(1:2,i-1); dz]/dz; 
                   tv = tv/norm(tv); % normalize the vector
                   
                   % compute normal vector (easy if possible)
                   if tv(2) == 0
                       nv = [0;1;0];
                   elseif tv(1) == 0
                       nv = [1;0;0];
                   else
                       nv = [1; 1; -tv(1) - tv(2)];
                       nv = nv/norm(nv); % normalize the vector
                   end
                   
                   % compute binormal vector
                   bv = cross(nv, tv);
                   bv = bv/norm(bv); % normalize the vector
                   
                   % add the new matrix
                   Rmat(:,:,i) = [bv, nv, tv];
               end
               
               
           elseif entrypt(3) == 0 % at the entry location
               pmat = zeros(3,1);
               Rmat = eye(3);
               
           else
               error("z coordinate of entry point must be >= 0");
               
           end
           
        end
        
        % generate a straight needle insertion
        function [pmat, Rmat] = generate_straight_needle(obj, L)
            z = (L - obj.needleLength):0.5:0;
            pmat = [zeros(2,length(z)); z] + obj.current_entry_point;
            
            Rmat = repmat(eye(3), 1,1,length(z));
            
        end
        
        % generate the current needle shape from the measured curvatures
        function [pmat, Rmat] = generate_needleshape(obj, curvatures)
            % check for all paremters to be configured
            pmat = [];
            Rmat = [];
            if all(obj.current_L <= obj.sensorLocations)
                return;
            end
            
            % make sure that the current length is less than the entire
            % length of the needle
            if (obj.current_L > obj.needleLength)
                error("The current length is longer than the needle's total length");
            end
            
            % see if there is a current entry point
            if isempty(obj.current_entry_point)
                return;
            end
            
            % generate straight needle 
            [pmat_s, Rmat_s] = obj.generate_straight_needle(obj.current_L);
            
            % generate the out-of-air needle portion
            [pmat_a, Rmat_a] = obj.generate_air_needle(obj.current_entry_point);
            
            % generate the in-tissue needle portion
            [pmat_t, ~, Rmat_t] = singlebend_Rinit_needleshape(curvatures, ...
                                                               obj.sensorLocations,...
                                                               obj.current_L, ...
                                                               obj.kc_i, obj.w_init_i,...
                                                               Rmat_a(:,:,end),...
                                                               obj.aaReliabilityWeights);
            pmat_t = pmat_t + pmat_a(:,end); % include the offset 


            % combine the needle shapes from the sections
            pmat = [pmat_s(:,1:end-1), pmat_a(:,1:end-1), pmat_t];
            Rmat = cat(3, Rmat_s(:,:,1:end-1), Rmat_a(:,:,1:end-1), Rmat_t);
            
        end
        
        % callback to update needle shape parateters
        function needlepose_cb(obj, pose_msg)
           % message orientation
           % - header
           % - pose
           %    - position
           %        - x: right,left (+, -) displacement of needle guide
           %        - y: insertion depth outside the needle guide
           %        - z: up, down (+, -) displacement of needle guide
           %    - orientation
           %        - y: rotation angle of the needle
          
           obj.current_L = pose_msg.pose.position.y;
           
           current_orientation = pose_msg.pose.orientation.y;
%            disp("New current length:");
%            disp(obj.current_L);
           
        end
        
        % publish the needle shape
        function publish_shape(obj)
            % check for current length viable
            if all(obj.current_L <= obj.sensorLocations)
                return;
            end
            
            % make sure that the current length is less than the entire
            % length of the needle
            if (obj.current_L > obj.needleLength)
                error("The current length is longer than the needle's total length");
            end
            
            % see if there is a current entry point
            if isempty(obj.current_entry_point)
                return;
            end
            
            % gather FBG samples and averagemv 
            fbg_peaks = zeros(obj.num_channels, obj.num_activeAreas);
            for i = 1:obj.num_samples
                fbg_msg = receive(obj.sensor_sub);
                fbg_peaks = fbg_peaks + obj.parseFBGPeaks(fbg_msg);

            end
            fbg_peaks = fbg_peaks/obj.num_samples;
            
            % calibrate sensors and determine needle shape
            theta0 = 0; % TODO: change to current pose
            curvatures = calibrate_fbgsensors(fbg_peaks, obj.sensorCalMatrices);
            [pmat, wv, Rmat, kc, w_init] = singlebend_needleshape(curvatures, ...
                                            obj.sensorLocations, obj.current_L,...
                                            obj.kc_i, obj.w_init_i, theta0);
            
            % generate the straight needle portion
            [pmat_s, Rmat_s] = obj.generate_straight_needle(obj.current_L);                        
            
            pmat_t = [pmat_s(:,1:end-1), pmat + pmat_s(:,end) + obj.current_entry_point];
            Rmat_t = cat(3, Rmat_s(:,:,1:end-1), Rmat);
                                        
            % parse the position and Rmat into a geometry_msgs/PoseArray msg
            poseArr_msg = NeedleShapePublisher.shape2posearray(pmat_t, Rmat_t);
            
            % publish the message
            obj.current_pub.send(poseArr_msg);
            
            % updates
            obj.kc_i = kc;
            obj.w_init_i = w_init;
        end
        
        % parse FBG peak message
        function [peaks, peaks_struct] = parseFBGPeaks(obj, fbg_msg)
           % fbg_msg is of type std_msgs/Float64MultiArray 
            peaks = zeros(obj.num_channels, obj.num_activeAreas);
            peaks_struct = struct();
            idx = 1;
            for i = 1:numel(fbg_msg.layout.dim)
               ch_i = fbg_msg.layout.dim(i).label;
               size_i = fbg_msg.layout.dim(i).size/fbg_msg.layout.dim(i).stride;
               if size_i > 0
                    peaks(i,:) = fbg_msg.data(idx:idx + size_i - 1);
               
                    peaks_struct.(ch_i) = fbg_msg.data(idx:idx + size_i - 1);
               end
               idx = idx + size_i;
            end
        end
        
        % callback to update skin entry point
        function skin_entry_cb(obj, msg)
            obj.current_entry_point = [msg.point.x; msg.point.y; msg.point.z];
%             disp("New entry point:");
%             disp(reshape(obj.current_entry_point,1,[]));
            
        end
    end
    
    methods(Static)
       % turn needle shape pose into PoseArray
       function poseArray = shape2posearray(pmat, Rmat)
          arguments
              pmat (3,:);
              Rmat (3,3,:);
          end
          
          assert(size(pmat,2) == size(Rmat,3)); % they must have the same arclength points
          poseArray = ros2message('geometry_msgs/PoseArray');
          pose_i = ros2message('geometry_msgs/Pose');
          for i = 1:size(pmat,2)
              p_i = pmat(:,i);
              q_i = rotm2quat(Rmat(:,:,i));
              
              pose_i.position.x = p_i(1);
              pose_i.position.y = p_i(2);
              pose_i.position.z = p_i(3);
              
              pose_i.orientation.w = q_i(1);
              pose_i.orientation.x = q_i(2);
              pose_i.orientation.y = q_i(3);
              pose_i.orientation.z = q_i(4);
              
              poseArray.poses(i) = pose_i;
          end
       end
       
       % unpack the needle shape from a pose array
       function [pmat, Rmat] = posearray2shape(msg)
           N = numel(msg.poses);
           pmat = zeros(3,N);
           Rmat = zeros(3,3,N);
           
           for i = 1:N
               pmat(:,i) = [msg.poses(i).position.x; 
                            msg.poses(i).position.y; 
                            msg.poses(i).position.z];
           
               quat = [msg.poses(i).orientation.w;
                       msg.poses(i).orientation.x;
                       msg.poses(i).orientation.y;
                       msg.poses(i).orientation.z]';
                   
               Rmat(:,:,i) = quat2rotm(quat);
           end
       end
       
       % function to parse FBG json into a new object
       function obj = fromFBGneedlefiles(json_filename, needle_mechanics_file, options)
           arguments
               json_filename         string;
               needle_mechanics_file string;
               options.ns string = '/needle';
               options.node_name string = '/NeedleShape';
               options.num_samples {mustBeInteger, mustBePositive} = 200;
               options.timeout {mustBePositive} = 20;
               % shape sensing options
               options.kc_i double = 0.0025;
               options.w_init_i (3,1) = [0.0025; 0; 0];
           end
           % read in the data
           needle_params    = parse_fbgneedle_json(json_filename);
           needle_mechanics = load(needle_mechanics_file);
           
           % check for fields
           assert(isfield(needle_mechanics, 'B'));
           if ~isfield(needle_mechanics, 'Binv')
               needle_mechanics.Binv = inv(needle_mechanics.B);
           end
           obj = NeedleShapePublisher(needle_params.num_channels,...
                                      needle_params.num_activeAreas,...
                                      needle_params.slocs_tip,...
                                      needle_params.aa_calMats,...
                                      needle_params.length,...
                                      needle_mechanics, ...
                                      'ns', options.ns,...
                                      'node_name', options.node_name,...
                                      'num_samples', options.num_samples,...
                                      'timeout', options.timeout,...
                                      'kc_i', options.kc_i,...
                                      'w_init_i', options.w_init_i);
           obj.aaReliabilityWeights = needle_params.aa_weights;
       end
    end
    
end 