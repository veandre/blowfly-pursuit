
%%  Leandre Varennes
%   APRIL 2020
%   this code executes 3 control strategies of pursuit, applied to 2 target trajectories.
%   Control equation: (1)Pure Pursuit, (2)Biased Pursuit or (3)Parallele Navigation
% 1) INITIATION
% 2) TARGET TRAJ
% 3)



%% INITIATION
close all
clear all
F1 = figure;


%% TARGET TRAJECTORY

target_linear_speed = 1;                                % m/sec
target_rotational_speed = deg2rad(1000);                % deg/sec
Ts = 1/190;                                             % time between two positions
t_end = 0.2;                                            % in sec
los_time_scale = 4;
div_time_scale = 4;     % combien de points on veut dessiner? ex : 4 pour dessiner 1 point /4
points = floor(linspace(1,t_end/Ts , t_end/Ts/div_time_scale-1));

% LINEAR TARGET POSITION
x_target_L = linspace(0,t_end * target_linear_speed, t_end/Ts);
y_target_L = zeros(1 , t_end/Ts) + 0.2;

% CIRCULAR TARGET POSITION
theta_target = target_rotational_speed * Ts;            % angular step of the target
all_theta_target = linspace(0,pi,pi/theta_target);
d_circle_target = target_linear_speed*Ts/tan(theta_target);
% call function to create circular traj
[x_target_C , y_target_C] = target_circular(Ts,target_linear_speed,target_rotational_speed);


%% FLY TRAJ ON LINEAR TARGET TRAJ

[x_fly_PP , y_fly_PP] = pure_poursuite(x_target_L , y_target_L, Ts);
[x_fly_DP , y_fly_DP , thetaE_cst] = deviated_poursuite (x_target_L , y_target_L , Ts );
[x_fly_PN , y_fly_PN , N] = proportional_navigation(x_target_L , y_target_L , Ts );

figure(F1)
subplot(3,3,4)
scatter(x_target_L(points),y_target_L(points),'filled')
hold on
scatter(x_fly_PP(points),y_fly_PP(points),'filled')
for los = 1:los_time_scale:length(x_target_L)
    plot([x_fly_PP(los) x_target_L(los)] , [y_fly_PP(los) y_target_L(los)],'k' )
end
title(['P-P thetaE = 0' ])
ylim([-0.02 0.27])
axis equal

subplot(3,3,5)
scatter(x_target_L(points),y_target_L(points),'filled')
hold on
scatter(x_fly_DP(points),y_fly_DP(points),'filled')
for los = 1:los_time_scale:length(x_target_L)
    plot([x_fly_DP(los) x_target_L(los)] , [y_fly_DP(los) y_target_L(los)],'k' )
end
title(['D-P thetaE = ' num2str(rad2deg(thetaE_cst))])
ylim([-0.02 0.27])
axis equal

subplot(3,3,6)
scatter(x_target_L(points),y_target_L(points),'filled')
hold on
scatter(x_fly_PN(points),y_fly_PN(points),'filled')
for los = 1:los_time_scale:length(x_target_L)
    plot([x_fly_PN(los) x_target_L(los)] , [y_fly_PN(los) y_target_L(los)],'k' )
end
title(['P-N (CBA) N = ' num2str(N)])
ylim([-0.02 0.27])
axis equal



%% FLY TRAJ ON CIRCULA TARGET TRAJ

[x_fly_PP , y_fly_PP] = pure_poursuite(x_target_C , y_target_C, Ts);
[x_fly_DP , y_fly_DP ] = deviated_poursuite (x_target_C , y_target_C , Ts );
[x_fly_PN , y_fly_PN] = proportional_navigation(x_target_C , y_target_C , Ts );

points = floor(linspace(1,length(x_fly_PP) , length(x_fly_PP)/div_time_scale-1));

figure(F1)
subplot(3,3,7)
scatter(x_target_C(points),y_target_C(points),'filled')
hold on
scatter(x_fly_PP(points),y_fly_PP(points),'filled')
for los = 1:los_time_scale:length(x_target_C)
    plot([x_fly_PP(los) x_target_C(los)] , [y_fly_PP(los) y_target_C(los)],'k' )
end
title(['P-P thetaE = 0' ])
ylim([-0.02 0.27])
axis equal

subplot(3,3,8)
scatter(x_target_C(points),y_target_C(points),'filled')
hold on
scatter(x_fly_DP(points),y_fly_DP(points),'filled')
for los = 1:los_time_scale:length(x_target_C)
    plot([x_fly_DP(los) x_target_C(los)] , [y_fly_DP(los) y_target_C(los)],'k' )
end
title(['D-P thetaE = ' num2str(rad2deg(thetaE_cst))])
ylim([-0.02 0.27])
axis equal

subplot(3,3,9)
scatter(x_target_C(points),y_target_C(points),'filled')
hold on
scatter(x_fly_PN(points),y_fly_PN(points),'filled')
for los = 1:los_time_scale:length(x_target_C)
    plot([x_fly_PN(los) x_target_C(los)] , [y_fly_PN(los) y_target_C(los)],'k' )
end
title(['P-N (CBA) N = ' num2str(N)])
ylim([-0.02 0.27])
axis equal


%% PLOT CURVES OF CONTROL PARAM
theta = -2*pi:pi/10:2*pi;
omega = -360:360/10:360; omega = deg2rad(omega);
N = 3;

kp = 1;
Biased_angle = -pi/6;
subplot(3,3,1)
WP_PP = kp.*theta;
plot(theta,WP_PP)
title(['PP kp = 1'])
axis equal
xlim([-pi pi])
ylim([-pi pi])
xticks([-pi 0 pi])
xticklabels({'-\pi','0','\pi'})
yticks([-pi 0 pi])
yticklabels({'-\pi','0','\pi'})
% grid on

subplot(3,3,2)
WP_BP = kp.*theta + Biased_angle;
plot(theta,WP_BP)
title('BP kp = 1 , Ba = -pi/6')
axis equal
xlim([-pi pi])
ylim([-pi pi])
xticks([-pi 0 pi])
xticklabels({'-\pi','0','\pi'})
yticks([-pi 0 pi])
yticklabels({'-\pi','0','\pi'})
% grid on

subplot(3,3,3)
WP_PN = N * omega;
plot(omega,WP_PN)
title('PN N = 3 ')
axis equal
xlim([-pi pi])
ylim([-pi pi])
xticks([-pi 0 pi])
xticklabels({'-\pi','0','\pi'})
yticks([-pi 0 pi])
yticklabels({'-\pi','0','\pi'})
% grid on


%% FUNCTIONS

%% Create circular trajectory
function [x_target , y_target] = target_circular(Ts,target_linear_speed,target_rotational_speed)

theta_target = target_rotational_speed * Ts;
all_theta_target = linspace(0,pi,pi/theta_target);

d_circle_target = target_linear_speed*Ts/tan(theta_target);

x_target = cos(pi-all_theta_target)*d_circle_target + d_circle_target ;
y_target = sin(pi-all_theta_target)*d_circle_target + 0.2;

end


%% PURE PURSUIT
function [x_fly , y_fly] = pure_poursuite (x_target , y_target , Ts )
x_fly = 0;
y_fly = 0;
fly_linear_speed = 1.5;        % m/sec
theta_P = pi/2;
omega_P = 0;
kp = 1;

for i =2:length(x_target)
    x_fly(i) = x_fly(i-1) + fly_linear_speed*Ts*cos(theta_P(i-1) + omega_P(i-1));
    y_fly(i) = y_fly(i-1) + fly_linear_speed*Ts*sin(theta_P(i-1) + omega_P(i-1));
    theta_A(i) = atan2(y_target(i)-y_fly(i) , x_target(i)-x_fly(i)) ;
    theta_P(i) = theta_P(i-1) + omega_P(i-1);
    theta_E(i) = theta_A(i)-theta_P(i);
    
    omega_P(i) = kp * theta_E(i);
end

end


%% BIASED PURSUITE
function [x_fly , y_fly , thetaE_cst] = deviated_poursuite (x_target , y_target , Ts )

thetaE_cst = -pi/6 ;

x_fly = 0;
y_fly = 0;
fly_linear_speed = 1.5;        % m/sec
theta_P = pi/2;
omega_P = 0;
kp = 1;

for i =2:length(x_target)
    x_fly(i) = x_fly(i-1) + fly_linear_speed*Ts*cos(theta_P(i-1) + omega_P(i-1));
    y_fly(i) = y_fly(i-1) + fly_linear_speed*Ts*sin(theta_P(i-1) + omega_P(i-1));
    theta_A(i) = atan2(y_target(i)-y_fly(i) , x_target(i)-x_fly(i)) ;
    theta_P(i) = theta_P(i-1) + omega_P(i-1);
    theta_E(i) = unwrap(theta_A(i)-theta_P(i));
    
    omega_P(i) = kp * theta_E(i) + thetaE_cst;
end

end


%% PROPORTIONAL NAVIGATION CBA

function [x_fly , y_fly , N] = proportional_navigation(x_target , y_target , Ts )

x_fly = 0;
y_fly = 0;
fly_linear_speed = 1.5;        % m/sec
theta_P = pi/2;
omega_P = 0;
theta_A = atan2(y_target-y_fly , x_target-x_fly) ;

thetaA_cst = pi/2 ;
N = 3 ;
for i =2:length(x_target)
    x_fly(i) = x_fly(i-1) + fly_linear_speed*Ts*cos(theta_P(i-1) + omega_P(i-1));
    y_fly(i) = y_fly(i-1) + fly_linear_speed*Ts*sin(theta_P(i-1) + omega_P(i-1));
    theta_A(i) = atan2(y_target(i)-y_fly(i) , x_target(i)-x_fly(i)) ;
    
    if i == 3
        thetaA_cst = theta_A(i) ;
    end
    
    theta_P(i) = theta_P(i-1) + omega_P(i-1);
    theta_E(i) = theta_A(i) - thetaA_cst;
    
    omega_P(i) =  N * (theta_E(i)-theta_E(i-1));
end

% figure
% plot(theta_P)
% hold on
% plot(theta_A)
% plot(theta_E)
% legend('thetaP','thetaA','thetaE')
end
