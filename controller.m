function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
kpT3=30;
kdT3=13;

kpT1=30;
kdT1=13;

kpT2=30;
kdT2=13;

kpM1=45;
kdM1=12;

kpM2=45;
kdM2=12;

kpM3=45;
kdM3=12;

DesiredX=des_state.acc(1)+kpT1*(des_state.pos(1)-state.pos(1))+kdT1*(des_state.vel(1)-state.vel(1));
DesiredY=des_state.acc(2)+kpT2*(des_state.pos(2)-state.pos(2))+kdT2*(des_state.vel(2)-state.vel(2));
PitchDes=(1/params.gravity)*(DesiredX*sin(des_state.yaw)-DesiredY*cos(des_state.yaw));
RollDes=(1/params.gravity)*(DesiredX*cos(des_state.yaw)+DesiredY*sin(des_state.yaw));
PitchDesVel=(1/params.gravity)*(des_state.acc(1)*des_state.yawdot*cos(des_state.yaw)+des_state.acc(2)*des_state.yawdot*sin(des_state.yaw));
RollDesVel=(1/params.gravity)*(des_state.acc(2)*des_state.yawdot*cos(des_state.yaw)-des_state.acc(1)*des_state.yawdot*sin(des_state.yaw));

% Thrust
F = params.mass*params.gravity+params.mass*(des_state.acc(3)+kpT3*(des_state.pos(3)-state.pos(3))+kdT3*(des_state.vel(3)-state.vel(3)));

% Moment
M =[kpM1*(PitchDes-state.rot(1))+kdM1*(PitchDesVel-state.omega(1)); kpM2*(RollDes-state.rot(2))+kdM2*(RollDesVel-state.omega(2)); kpM3*(des_state.yaw-state.rot(3))+kdM3*(des_state.yawdot-state.omega(3))];

% =================== Your code ends here ===================

end
