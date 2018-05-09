
clear all;
close all;
clc;

D2R = 3.141592/180;
Cover_range = 3;
SCAN_RANGE = 50;
SCAN_THETA = 90*D2R;
del_theta = 2*D2R;

%% UGV Profile

VELO_UGV = [0.1 0.1 0];
POS_UGV = [0 0 45*D2R];

N_step = 8;        % receding horizon time step.
N_angle = 8;       % angle step.
dt = 0.3;           % time step.
MAX_omega = 30*D2R;    % Maximum angular velocity.
MAX_omega_dot = 40*D2R; % Maximum angular acceleration.

VELO_LAW = 5/3.6;  VELO_HIGH = 35/3.6; % m/sec  

% V(R) = CONST_A X ROOT(R) + CONST_B

meet_pts = zeros(1,3);
wpt_cnt = 1;

iter_prcd = 1;
det = 1;

%% Plotting

aviobj = avifile('UGV Obstacle Avoidance_omega030_wpt.avi','fps',10,'compression','IYUV','Quality',100);

axis equal; hold on; grid on;

CAR_body_vrt = ...
    [-0.778 0.750;
    -0.65 0.8925;
    2 0.8925;
    3.327 0.7;
    3.327 -0.7;
    2 -0.8925;
    -0.65 -0.8925;
    -0.778 -0.750;
    -0.778 0.750];

CAR_tire_vrt_tmp = ...
    [-0.5 0.125; 0.5 0.125; 0.5 -0.125; -0.5 -0.125; -0.5 0.125];

CAR_waypoint = plot(0,0,'b*');
CAR_plan = plot(zeros(1,N_step+1), zeros(1,N_step+1),'c-');

CAR_body = patch(CAR_body_vrt(:,1), CAR_body_vrt(:,2),'blue');
% for iter = 1 : 4
%     CAR_tire_vrt(iter,:) = CAR_tire_vrt_tmp+tire_bias(iter,:)
%     CAR_tire(iter) = patch(CAR_tire_vrt(:,1), CAR_tire_vrt(:,2),'blue');


for iter = 1 : OBS_NUM
    patch(vrt((OBS_VRT+1)*(iter-1)+1:(OBS_VRT+1)*iter,1),vrt((OBS_VRT+1)*(iter-1)+1:(OBS_VRT+1)*iter,2),'green');
end

% VELO_omega = zeros(1,N_step+1);

while (det)
    %% DATA SCANNING.

    [meet_pts,flag_meet_pts] = scan_obstacle(POS_UGV(iter_prcd,:),vrt,OBS_NUM,OBS_VRT,SCAN_RANGE,SCAN_THETA,del_theta);
    plot(meet_pts(:,1),meet_pts(:,2),'*','color',rand(1,3)); hold on;

    %% velocity and path profiles.

    % case 1. law -> high velocity profile.
    [POS_UGV_cnd, VELO_UGV_cnd, VELO_mag(1,:), VELO_omega(1,:)] = estimate_future_condition(POS_UGV(iter_prcd,:), VELO_UGV(iter_prcd,:), VELO_LAW, VELO_HIGH, N_step, N_angle, dt, MAX_omega, MAX_omega_dot, 1);
    % case 2. high -> law velocity profile.
    [POS_UGV_cnd(:,3*(N_angle+1)+1:3*(2*N_angle+2)), VELO_UGV_cnd(:,3*(N_angle+1)+1:3*(2*N_angle+2)),VELO_mag(2,:), VELO_omega(2,:)] = estimate_future_condition(POS_UGV(iter_prcd,:), VELO_UGV(iter_prcd,:), VELO_LAW, VELO_HIGH, N_step, N_angle, dt, MAX_omega, MAX_omega_dot, -1);    
    % case 3. constant velocity profile.    
    [POS_UGV_cnd(:,3*(2*N_angle+2)+1:3*(3*N_angle+3)), VELO_UGV_cnd(:,3*(2*N_angle+2)+1:3*(3*N_angle+3)),VELO_mag(3,:), VELO_omega(3,:)] = estimate_future_condition(POS_UGV(iter_prcd,:), VELO_UGV(iter_prcd,:), VELO_LAW, VELO_HIGH, N_step, N_angle, dt, MAX_omega, MAX_omega_dot, 0); 

    %% Select optimal path.

    for iter_cnd = 1 : length(POS_UGV_cnd(1,:))/3
        cost(iter_cnd) = 0;
        for iter_step = 1 : N_step
            for iter_pt = 1 : length(meet_pts(:,1))
                ref_obs = POS_UGV_cnd(iter_step,3*(iter_cnd-1)+1:3*(iter_cnd-1)+2)-meet_pts(iter_pt,1:2);
                temp_obs = ref_obs*VELO_UGV_cnd(iter_step,3*(iter_cnd-1)+1:3*(iter_cnd-1)+2)'/(norm(ref_obs)*norm(VELO_UGV_cnd(iter_step,3*(iter_cnd-1)+1:3*(iter_cnd-1)+2)));
                
                if temp_obs > 1
                    temp_obs = 1;
                elseif temp_obs < -1
                    temp_obs = -1;
                end
                ref_theta = acos(temp_obs);
                cost(iter_cnd) = cost(iter_cnd) + (1/iter_step)*((0.5*cos(ref_theta-pi)+0.5+BIAS_OBS)*exp(-(norm(ref_obs))));
            end
            
            ref_goal = POS_UGV_cnd(iter_step,3*(iter_cnd-1)+1:3*(iter_cnd-1)+2)-WPT_POS(wpt_cnt,1:2);
            temp_goal = WPT_VELO(wpt_cnt,1:2)*VELO_UGV_cnd(iter_step,3*(iter_cnd-1)+1:3*(iter_cnd-1)+2)'/(norm(WPT_VELO(wpt_cnt,1:2))*norm(VELO_UGV_cnd(iter_step,3*(iter_cnd-1)+1:3*(iter_cnd-1)+2)));

            if temp_goal > 1
                temp_goal = 1;
            elseif temp_goal < -1
                temp_goal = -1;
            end
            ref_theta_goal = acos(temp_goal);
            
            cost(iter_cnd) = cost(iter_cnd) + (1/iter_step)*(0.5*cos(ref_theta_goal-pi)+0.5+BIAS_GOAL)*(1-exp(-norm(ref_goal)));     
        end

        if iter_cnd == 1
            min_cost = cost(iter_cnd);
            min_idx = iter_cnd;
        else
            if cost(iter_cnd) < min_cost
                min_cost = cost(iter_cnd);
                min_idx = iter_cnd;
            end
        end
    end
    
    POS_UGV(iter_prcd+1,1:3) = POS_UGV_cnd(2,3*(min_idx-1)+1:3*min_idx);
    VELO_UGV(iter_prcd+1,1:3) = VELO_UGV_cnd(2,3*(min_idx-1)+1:3*min_idx);
    
    if ((WPT_POS(wpt_cnt,1:2)-POS_UGV(iter_prcd+1,1:2))*(WPT_VELO(wpt_cnt,1:2)/norm(WPT_VELO(wpt_cnt,1:2)))')*(WPT_VELO(wpt_cnt,1:2)/norm(WPT_VELO(wpt_cnt,1:2))) < WPT_ADMISSIBLE
        if (wpt_cnt == length(WPT_POS(:,1)))
            det = 0;
        else
            wpt_cnt = wpt_cnt + 1;
        end
    end
    
    axis([POS_UGV(iter_prcd,1)-15, POS_UGV(iter_prcd,1)+15, POS_UGV(iter_prcd,2)-15,POS_UGV(iter_prcd,2)+15]);

    euler = [cos(-POS_UGV(iter_prcd,3)) -sin(-POS_UGV(iter_prcd,3)); sin(-POS_UGV(iter_prcd,3)) cos(-POS_UGV(iter_prcd,3))];

    CAR_body_trans = CAR_body_vrt*euler;
    set(CAR_body,'Xdata',CAR_body_trans(:,1)+POS_UGV(iter_prcd,1),'YData',CAR_body_trans(:,2)+POS_UGV(iter_prcd,2));
    set(CAR_waypoint,'XData',WPT_POS(wpt_cnt,1),'YData',WPT_POS(wpt_cnt,2));
    set(CAR_plan,'XData',POS_UGV_cnd(:,3*(min_idx-1)+1),'YData',POS_UGV_cnd(:,3*(min_idx-1)+2));

    drawnow;

          frame = getframe(gcf);
          aviobj = addframe(aviobj, frame);
    
    iter_prcd = iter_prcd + 1

    if (iter_prcd > 500)
        break;
    end
end


aviobj = close(aviobj);

% plot(meet_pts(:,1),meet_pts(:,2),'r*');
plot(POS_UGV(:,1),POS_UGV(:,2),'ko');

time = 0 : dt : (length(VELO_UGV(:,1))-1)*dt;
for iter_speed = 1 : length(VELO_UGV(:,1))
    SPEED_UGV(iter_speed) = norm(VELO_UGV(iter_speed,1:2))*3.6;   % km/h
end

figure(2)
subplot(211), plot(time,SPEED_UGV);
xlabel('time (sec)');
ylabel('speed (km/h)');

subplot(212), plot(time,VELO_UGV(:,3)/D2R);
xlabel('time (sec)');
ylabel('turning rate (deg/sec)');

% figure(3)
% hold on;
% for iter_cnd = 1 : length(POS_UGV_cnd(1,:))/3
%     plot(POS_UGV_cnd(:,3*(iter_cnd-1)+1),POS_UGV_cnd(:,3*(iter_cnd-1)+2),'*-','color','green');
% end
% for iter_plot = N_angle*4+1 : N_angle*6
%     plot(POS_UGV_cnd(:,3*(iter_plot-1)+1),POS_UGV_cnd(:,3*(iter_plot-1)+2),'*-','color','red');
% end