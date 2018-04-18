clc, clear all, close all
k= [0 0; 0 0; 0 0];

waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0;]';
      

t = 10;         

    d = waypoints(:,2:end) - waypoints(:,1:end-1); % distance between points [3x4]matrix
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2); % time interval between points
    traj_time = [0, cumsum(d0)]; %cumilative time in between waypoints
    waypoints0 = waypoints; % waypoints
    
x1 = waypoints(1,:);
x2 = waypoints(2,:);
x3 = waypoints(3,:);

y = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
pp=spline(traj_time,[k(:,1) k(:,2) waypoints0 k(:,2) k(:,2)]);
xx=0:.001:13.8564;
view(3);
plot3(waypoints0(1,:)',waypoints0(2,:)',waypoints0(3,:)','o')
hold on;
path = ppval(pp,xx);
plot3(path(1,:)',path(2,:)',path(3,:)','-')      
%%
x = -4:4; y = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
       cs = spline(x,[0 y 0]);
       xx = linspace(-4,4,101);
       plot(x,y,'o'); %plot(time_traj,waypoint0,'o')
%          plot(x,y,'o',xx,ppval(cs,xx),'-');

plot(time_traj,waypoint0,'o')
      


% pp3 = spline(x3,[0 traj_time 0]); 
% xx = linspace(0,13.8564,101);
% plot(x1,traj_time,'o',xx,ppval(pp1,xx),'-');
%plot(x1,traj_time,'o',xx,ppval(pp,xx),'-');
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1); %
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
% 
% %     if(t == 0)
% %         desired_state.pos = waypoints0(:,1);
% %     else
% %         scale = t/d0(t_index-1);
% %         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
% %     end
% %     desired_state.vel = zeros(3,1);
% %     desired_state.acc = zeros(3,1);
% %     desired_state.yaw = 0;
% %     desired_state.yawdot = 0;
% % end