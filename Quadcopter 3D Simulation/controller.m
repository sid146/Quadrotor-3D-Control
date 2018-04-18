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

% % Thrust
% F = 0;
% 
% % Moment
M = zeros(3,1);
Kpx = 25;
Kdx = 1.5;
Kpz = 22;
Kdz = 1.5;
Kpphi = 20;
Kdphi = .1;

% Kpx = 25;
% Kdx = 2;
% Kpz = 22;
% Kdz = 2;
% Kpphi = 20;
% Kdphi = .1;

Kpy = Kpx;
Kdy = Kdx;
Kppsi = Kpphi;
Kdpsi = Kdphi;
Kptheta = Kpphi;
Kdtheta = Kdphi;


r1des_ddot = des_state.acc(1)+Kdx*(des_state.vel(1)-state.vel(1))+Kpx*(des_state.pos(1)-state.pos(1));
r2des_ddot = des_state.acc(2)+Kdy*(des_state.vel(2)-state.vel(2))+Kpy*(des_state.pos(2)-state.pos(2));
r3des_ddot = des_state.acc(3)+Kdz*(des_state.vel(3)-state.vel(3))+Kpz*(des_state.pos(3)-state.pos(3));

phi_des   	= (r1des_ddot*des_state.yaw - r2des_ddot)/params.gravity ;
theta_des 	= (r1des_ddot + r2des_ddot*des_state.yaw)/params.gravity ;

M =[Kpphi*(phi_des-state.rot(1)) + Kdphi*(0-state.omega(1));
	Kptheta*(theta_des-state.rot(2)) + Kdtheta*(0-state.omega(2));
	Kppsi*(des_state.yaw-state.rot(3)) + Kdpsi*(des_state.yawdot-state.omega(3))];

F = params.mass*(params.gravity+r3des_ddot);

% =================== Your code ends here ===================

end
