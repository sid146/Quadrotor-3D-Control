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
k= [0 0; 0 0; 0 0];
persistent waypoints0 traj_time d0 

if nargin > 2 %so that after initialisation the foll is not calculated for every call, only t and state are passed for calls after intitialisation.
    d = waypoints(:,2:end) - waypoints(:,1:end-1); % distance between points [3x4]matrix
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2); % time interval between points
    traj_time = [0, cumsum(d0)]; %cumilative time in between waypoints
    waypoints0 = waypoints; % waypoints
   
else
    if(t > traj_time(end))
%         if(t > traj_time(end)-0.25)
        t = traj_time(end);
        desired_state.pos = waypoints0(:,end);
    desired_state.vel=0;
        desired_state.acc =0; %(v_f - v_i)/t
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    end
%     t_index = find(traj_time >= t,1); %finds one position of traj_time that's greater than t
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
%          desired_state.pos = ppval(pp,t);
   
   
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    
    end
  
  pp=spline(traj_time,[k(:,1) waypoints0 k(:,2)]);
    desired_state.pos = ppval(pp,t);
    desired_state.vel=((ppval(pp,t+.001)-ppval(pp,t))/(.001));
    v1 = (ppval(pp,t+.0005)-ppval(pp,t))/(.0005);
    v2 = (ppval(pp,t+.001)-ppval(pp,t+.0005))/(.0005);
    desired_state.acc = ((v2-v1)/(.0005)); %(v_f - v_i)/t
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%


%% Fill in your code here


% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

