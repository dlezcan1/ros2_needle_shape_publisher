%% demo_needle_shape_publisher.m
%
% script to publish the needle shape (testing script)
%
% - written by: Dimitri Lezcano

clear; 
!taskkill /f /im libmwros2server.exe & rem shutdown existing matlab ROS 2 servers for windows
%% Setup
global needle_shape

% setup namespace
namespace = '/needle';
needlepose_pub_name = '/stage/state/needle_pose';
entrypoint_pub_name = '/subject/state/skin_entry';

% setup needle shape publisher
needle_mechfile  = fullfile("../needle_data/shapesensing_needle_properties.mat");
needle_paramfile = fullfile("../needle_data","needle_3CH_3AA",...
                            "needle_params-Jig_Calibration_11-15-20_weighted_weights.json");

%% Configure ROS Nodes
% setup needle pose publisher
helper.node = ros2node('TestNeedleHelper');
helper.pose_pub    = ros2publisher(helper.node, needlepose_pub_name, 'geometry_msgs/PoseStamped');
helper.entrypt_pub = ros2publisher(helper.node, entrypoint_pub_name, 'geometry_msgs/PointStamped');
helper.pose_sub    = ros2subscriber(helper.node, needlepose_pub_name, @needlepose_cb);

needleshape_talker = NeedleShapePublisher.fromFBGneedlefiles(needle_paramfile, needle_mechfile);

helper.shape_sub   = ros2subscriber(helper.node, '/needle/state/shape', @needleshape_cb);


%% Run through the loop
disp("Press [ENTER] to start publishing.")
pause;
L = 50; entrypt = [0;0;5];
disp("Publishing needle shape along with helper node.");
while true
    pose_msg = generate_pose_msg(L, 0);
    skinentry_msg = generate_skinentry_msg(entrypt);
    helper.pose_pub.send(pose_msg);
    helper.entrypt_pub.send(skinentry_msg);
%     L = L + 5;
    [p, R] = needleshape_talker.generate_needleshape(zeros(2,3));
    pause(0.1);
end

%% Helper functions
function msg = generate_pose_msg(L, theta_rot)
    msg = ros2message('geometry_msgs/PoseStamped');
    
    msg.pose.position.x = 0; % lateral (left-right) displacement
    msg.pose.position.z = 0; % vertical (up-down)   displacement
    msg.pose.position.y = L; % insertion depth
    
    q = rotm2quat(rotz(theta_rot));
    msg.pose.orientation.w = q(1);
    msg.pose.orientation.y = q(4); % 
    
end

function needlepose_cb(msg)
%     fprintf("PoseSub: I heard z: %.2f, ang_z: %.2f\n", msg.pose.position.z, msg.pose.orientation.z);
end

function needleshape_cb(msg)
   global needle_shape
    % grab the positions
   [pmat, ~] = NeedleShapePublisher.posearray2shape(msg);
   needle_shape = pmat;
   
   disp("Got needle shape");
   
   figure(1);
   plot3(pmat(3,:), pmat(1,:), pmat(2,:));
   axis equal;

end

function msg = generate_skinentry_msg(pt)
    msg = ros2message('geometry_msgs/PointStamped');
    msg.point.x = pt(1);
    msg.point.y = pt(2);
    msg.point.z = pt(3);

end