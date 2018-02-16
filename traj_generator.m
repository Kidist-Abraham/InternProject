function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

%persistent waypoints0 traj_time d0
%if nargin > 2
 %   d = waypoints(:,2:end) - waypoints(:,1:end-1);
  %  d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
   % traj_time = [0, cumsum(d0)];
%    waypoints0 = waypoints;
%else
    %if(t > traj_time(end))
  %      t = traj_time(end);
   % end
 %   t_index = find(traj_time >= t,1);

%    if(t_index > 1)
     %   t = t - traj_time(t_index-1);
    %end
    %if(t == 0)
     %   desired_state.pos = waypoints0(:,1);
    %else
      %  scale = t/d0(t_index-1);
     %   desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    %end
    %desired_state.vel = zeros(3,1);
   % desired_state.acc = zeros(3,1);
  %  desired_state.yaw = 0;
 %   desired_state.yawdot = 0;
%end
%


%% Fill in your code here
    desired_state.pos = zeros(3,1);
   desired_state.vel = zeros(3,1);
   desired_state.acc = zeros(3,1);
   desired_state.yaw = 0;
   desired_state.yawdot = 0;
persistent coffx coffy coffz waypoints0 traj_time d0
if nargin > 2
    desired_state.pos = zeros(3,1);
   desired_state.vel = zeros(3,1);
   desired_state.acc = zeros(3,1);
   desired_state.yaw = 0;
   desired_state.yawdot = 0;
   
   coffx=getCoff(waypoints(:,1),4);
   coffy=getCoff(waypoints(:,2),4);
   coffz=getCoff(waypoints(:,3),4);
   
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end)-0.0001;
        
    end
    t_index = find(traj_time >= t,1)-1;
    t_index=max(t_index,1);
    scale=(t-traj_time(t_index))/d0(t_index);
    poss=[1;scale;scale^2;scale^3;scale^4;scale^5;scale^6;scale^7];
    vell=[0;1;2*scale;3*(scale^2);4*(scale^3);5*(scale^4);6*(scale^5);7*(scale^6)];
    accc=[0;0;2;6*scale;12*(scale^2);20*(scale^3);30*(scale^4);42*(scale^5)];
    index = [(t_index-1)*8+1:t_index*8];

    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    end
     if(t_index<1)
       desired_state.pos=scale*[coffx(1:8)*poss;coffy(1:8)*poss;coffz(1:8)*poss];
        desired_state.vel=scale*[coffx(1:8)*vell;coffy(1:8)*vell;coffz(1:8)*vell];
         desired_state.acc=scale*[coffx(1:8)*accc;coffy(1:8)*accc;coffz(1:8)*accc];
     end
      if(t_index>1&&t_index<2)
      desired_state.pos=scale*[coffx(9:16)*poss;coffy(9:16)*poss;coffz(9:16)*poss];
      desired_state.vel=scale*[coffx(9:16)*vell;coffy(9:16)*vell;coffz(9:16)*vell];
      desired_state.acc=scale*[coffx(9:16)*accc;coffy(9:16)*accc;coffz(9:16)*accc];
      end
       if(t_index>2&&t_index<3)
      desired_state.pos=scale*[coffx(17:24)*poss;coffy(17:24)*poss;coffz(17:24)*poss];
      desired_state.vel=scale*[coffx(17:24)*vell;coffy(17:24)*vell;coffz(17:24)*vell];
      desired_state.acc=scale*[coffx(17:24)*accc;coffy(17:24)*accc;coffz(17:24)*accc];
       end
       if(t_index>3&&t_index<=4)
      desired_state.pos=scale*[coffx(25:32)*poss;coffy(25:32)*poss;coffz(25:32)*poss];
      desired_state.vel=scale*[coffx(25:32)*vell;coffy(25:32)*vell;coffz(25:32)*vell];
      desired_state.acc=scale*[coffx(25:32)*accc;coffy(25:32)*accc;coffz(25:32)*accc];
      end
       % desired_state.pos = (1 - scale) * waypoints0(:,t_index) + scale * waypoints0(:,t_index);
    
  
end
% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

