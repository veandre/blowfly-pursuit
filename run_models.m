

%%  Leandre Varennes
%   APRIL 2020
%   this code executes the male fly heading control models.
%   this code gives an error (in mean distance) between the trajectory of the virtual fly and that of the real fly.

%   The user has to choose the different options:
%   Control plane : (1)Horizontal or (2)Vertical,
%   Parameters values: (1)Behavioural data, or (2)Best fit to fly trajectory.
%   Control equation: (1)Biased Pursuit, (2)Parallele Navigation or (3)Mixed Pursuit.


%% INITIATION AND PROMPT VALUES
clear all
close all

dec_plan = 0;
while dec_plan == 0
    prompt = 'Which Plan HORIZONTAL (1) or VERTICAL (2) ';
    user_plan = input(prompt);
    if user_plan ==1
        dec_plan = 1;
    elseif user_plan ==2
        dec_plan = 1;
    end
end

dec_model = 0;
while dec_model == 0
    prompt = 'WhIch Model? BEHAVIORAL DATA (1)   or    BEST FIT (2)  ? ';
    user_model = input(prompt);
    if user_model ==1
        dec_model = 1;
    elseif user_model ==2
        dec_model = 1;
    end
end

dec_control = 0;
while dec_control == 0
    prompt = 'WhIch Control? BIASED PURSUIT (1)   ,    PROPORTIONAL NAV (2)   or    MIXED PURSUIT  (3) ? ';
    user_control = input(prompt);
    if user_control ==1
        dec_control = 1;
    elseif user_control ==2
        dec_control = 1;
    elseif user_control ==3
        dec_control = 1;
    end
end


%%

Ts = 1/190;                 % Cameras acquisition speed
load ('DATA.mat')           % file with 3D positions and precomputed angles
all_err = [];

for captureNum = 1 :17      % Number of captures
    
    switch     user_plan
        case 1 % HORIZONTAL
            % Virtual fly's sstarting position (was in mm, convert in
            % meters)
            Fx0 = target_fly(captureNum).position_fly(1,1)/1000;
            Fy0 = target_fly(captureNum).position_fly(1,2)/1000;
            % create array of X and Y positions of the real protagonists
            target_traj = target_fly(captureNum).position_ball(2:end,1:2) ./ 1000;
            fly_traj = target_fly(captureNum).position_fly(2:end,1:2) ./ 1000;
            
            t_end = length(target_traj)*Ts;
            time_chase = linspace(0,t_end,length(target_traj));
            
            % Create values to be used in the model as basis for continuous extrapolation
            target_TX=[time_chase' , target_traj(:,1)];
            target_TY=[time_chase' , target_traj(:,2)];
            
            % initial fly heading
            thetaP_init = atan2(fly_traj(2,2) - fly_traj(1,2) , fly_traj(2,1)-fly_traj(1,1)) ;
            % Once the fly locked the target, it fixes the absolute bearing
            % angle, after the 4th frame -- arbitrary .
            ThetaA_cst = atan2(target_traj(4,2) - fly_traj(4,2) , target_traj(4,1) - fly_traj(4,1)) ;
            % Horizontal speed of the real fly, used as forward speed of the virtual fly
            Vfly = sqrt((diff(target_fly(captureNum).position_fly(:,1))./1000).^2 + (diff(target_fly(captureNum).position_fly(:,2)./1000)).^2) ./Ts;
            vNt = [time_chase' ,Vfly  ];  % Create values to be used in the model as basis for continuous extrapolation
            
            
        case 2 % VERTICAL
            % Virtual fly's sstarting position (was in mm, convert in meters)
            Fx0 = sqrt((target_fly(captureNum).position_fly(1,1)./1000).^2+(target_fly(captureNum).position_fly(1,2)./1000).^2);
            Fy0 = target_fly(captureNum).position_fly(1,3)/1000;
            % In vertical plane, y axis is elevation and x axis is the deplacement on the
            % horizontal plane, always positive, initiated on the starting
            % position of the fly.
            Tx0 = sqrt((target_fly(captureNum).position_ball(1,1)./1000).^2+(target_fly(captureNum).position_ball(1,2)./1000).^2);
            Ty0 = target_fly(captureNum).position_ball(1,3)/1000;
            DECAL = abs(Tx0-Fx0);
            Fx0 = 0;
            % Z axis
            target_trajZ = target_fly(captureNum).position_ball(2:end,3) ./ 1000;
            fly_trajZ = target_fly(captureNum).position_fly(2:end,3) ./ 1000;
            % horizontal move
            target_trajXY_diff = sqrt((diff(target_fly(captureNum).position_ball(2:end,1))./1000).^2+(diff(target_fly(captureNum).position_ball(2:end,2))./1000).^2);
            target_trajXY_diff = cumsum(abs(target_trajXY_diff));
            target_trajXY= [DECAL ; DECAL+target_trajXY_diff];
            fly_trajXY_diff = sqrt((diff(target_fly(captureNum).position_fly(2:end,1))./1000).^2+(diff(target_fly(captureNum).position_fly(2:end,2))./1000).^2);
            fly_trajXY_diff = cumsum(abs(fly_trajXY_diff));
            fly_trajXY= [0 ; 0+fly_trajXY_diff];
            
            target_traj=[target_trajXY , target_trajZ];
            fly_traj=[fly_trajXY , fly_trajZ];
            
            t_end = length(target_trajZ)*Ts;
            time_chase = linspace(0,t_end,length(target_trajZ));
            target_TX=[time_chase' , target_trajXY];
            target_TY=[time_chase' , target_trajZ];
            
            thetaP_init = atan2(fly_traj(2,2) - fly_traj(1,2) , fly_traj(2,1)-fly_traj(1,1)) ;
            % Fly speed of the real fly IN 3Dimensions, used as speed of the virtual fly
            Vfly = sqrt((diff(target_fly(captureNum).position_fly(:,1))./1000).^2 +(diff(target_fly(captureNum).position_fly(:,2))./1000).^2 +...
                (diff(target_fly(captureNum).position_fly(:,3))./1000).^2)./Ts;
            vNt = [time_chase' ,Vfly  ];  % Create values to be used in the model as basis for continuous extrapolation
            
    end
    
    
    %% This part is to test the relationship between horizontal and
    % vertical turns
    %     omega_p_H = theta_dyn(captureNum).v_theta_p;
    % omega_p_HNt = [time_chase(4:end)' ,0.*omega_p_H]; % ICI on *0 pour faire comme si il n'y avait pas d'existence
    %     kp_3d = -0.25;
    %     C_3d = 3.8;
    %     tau_3d = 0.007; % deltaT = 21ms
    
    
    %% CONTROL PARAMETERS
    
    switch     user_plan
        case 1 % HORIZONTAL
            switch     user_model
                case 1 % BEHAVIOURAL DATA
                    kp = 17.4;
                    kd = 0.17;
                    c = 0;
                    tau_BP = 0.003; % for deltaT= 10ms
                    ThetaE_cst = deg2rad(0);
                    
                    N =  0.43;
                    tau_PN = 0.009; % for deltaT= 26ms
                    
                case 2 % BEST FIT MODEL
                    switch user_control
                        case 1
                    kp = 26;                        
                        case 2
                    N = 5.1;
                        case 3
                    kp = 24;
                    N = 0.15;
                    end

                    kd = 0;%0.01;
                    c = 0;
                    tau_BP = 0.003; % for deltaT= 10ms
                    ThetaE_cst = deg2rad(0);
                    tau_PN = 0.009; % for deltaT= 26ms
            end
        case 2 % VERTICAL
            switch     user_model
                case 1 % BEHAVIOURAL DATA
                    kp = 15.6;
                    kd = 0.05;
                    tau_BP = 0.007; % for deltaT= 21ms
                    ThetaE_cst = deg2rad(32);
                    
                    N =  0.62;
                    tau_PN = 0.010; % for deltaT= 32ms
                    ThetaA_cst = deg2rad(47);
                    
                case 2 % BEST FIT MODEL
                    switch user_control
                        case 1
                    kp = 10;                        
                        case 2
                    N = 1.8;
                        case 3
                    kp = 8;
                    N = 0.05;
%                     kp = 20;
%                     N = 0.9;
                    end
                    kd = 0.05;
                    tau_BP = 0.007; % for deltaT= 21ms
                    ThetaE_cst = deg2rad(23);
                    
                    tau_PN = 0.010; % for deltaT= 32ms
                    ThetaA_cst = deg2rad(46);
            end
    end
    
    %% Run SIMULINK MODELS
    switch  user_control
        case 1  % BIASED PURSUIT
            sim('models\BP',t_end);
        case 2  % Proportional Nav
            sim('models\PN',t_end);
        case 3  % Mixed Pursuit
            if user_plan ==1
                sim('models\MP_H',t_end);
            elseif user_plan ==2
                sim('models\MP_V',t_end);
            end
    end
    
    %% Compute the error of the model
    MOD_fly_X = ans.pos_fly(:,1);
    MOD_fly_Y = ans.pos_fly(:,2);
    err_dist= sqrt((MOD_fly_X(2:end)-fly_traj(:,1)).^2+ (MOD_fly_Y(2:end)-fly_traj(:,2)).^2);
    all_err = [all_err ; err_dist];
    
    %% PLOT EACH PURSUIT
    figure
    hold on
    plot(target_traj(:,1),target_traj(:,2),'b')
    plot(fly_traj(:,1),fly_traj(:,2),'--r')
    plot(MOD_fly_X,MOD_fly_Y,'r')
    axis equal
%--------------- Uncomment to plot the LOS every 20ms
    % for los = 1 : 4 : length(fly_traj)
    %     plot([MOD_fly_X(los) target_traj(los,1)], [MOD_fly_Y(los) target_traj(los,2)] , 'k' )
    % end
    switch user_plan
        case 1  % axis change a bit depending on the plan of interest
            ylim([0.15 0.65])
        case 2
            ylim([0 0.6])
    end
    ylabel('meters')
    xlabel('meters')
    title(['pursuit ' num2str(captureNum)])
    legend('target','fly','model')
end


disp([ ' mean err = ' num2str(mean(all_err)) ' std err = ' num2str(std(all_err))])

