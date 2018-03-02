close all;
clear;


addpath('utils');


%% pre-calculated trajectories
%trajhandle = @traj_line;
% trajhandle = @traj_helix;

%% Trajectory generation with waypoints

trajhandle = @traj_generator;
 waypoints = [0    0   0;
              1    1   1;
              2    0   2;
              3    -1  1;
              4    0   0]';
 trajhandle([],[],waypoints);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller

[t, state] = simulation_3d(trajhandle, controlhandle);
