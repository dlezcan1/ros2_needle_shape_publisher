%% needle_shape_publisher.m
%
% script to publish the needle shape
%
% - written by: Dimitri Lezcano

clear; 
%% Setup
% setup namespace
namespace = '/needle';
needlepose_pub_name = '/stage/state/needle_pose';
entrypoint_pub_name = '/subject/state/skin_entry';

% timer delay
timer_length = 0.1;

% setup needle shape publisher
needle_mechfile  = fullfile("../needle_data/shapesensing_needle_properties.mat");
needle_paramfile = fullfile("../needle_data","needle_3CH_3AA",...
                            "needle_params-Jig_Calibration_11-15-20_weighted_weights.json");

%% Configure ROS Nodes
% setup needle pose publisher
needleshape_talker = NeedleShapePublisher.fromFBGneedlefiles(needle_paramfile, needle_mechfile);


%% Run through the loop publishing the needle shape
disp("Press [ENTER] to start publishing.")
pause;
disp("Publishing needle shape.");
while true
    needleshape_talker.publish_shape();
    disp("Published needle shape");
    pause(timer_length);
end

