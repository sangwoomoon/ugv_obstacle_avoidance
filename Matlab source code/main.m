
clear all;
close all;
clc;

D2R = 3.141592/180;
Cover_range = 3;
SCAN_RANGE = 50;
SCAN_THETA = 90*D2R;
del_theta = 2*D2R;

OBS_MODEL = ...
   [-4 -0.5; -4 0.5; 4 0.5; 4 -0.5; -4 -0.5;             % GATE 1 (2개)
    -3 -0.5; -3 0.5; 3 0.5; 3 -0.5; -3 -0.5;
   -120 -0.1; -120 0.1; 120 0.1; 120 -0.1; -120 -0.1;    % 도로 (noize 세게)                        3 4
    -0.4 -0.4; -0.4 0.4; 0.4 0.4; 0.4 -0.4; -0.4 -0.4;   % GATE 2 (중앙 분리)
    -1 -1; -1 1; 1 1; 1 -1; -1 -1;                       % GATE 3 (건초더미) (noize 약하게)         28 ~ 37
    -2.5 -1; -2.5  1;  2.5  1;  2.5 -1; -2.5 -1];        % 차량


OBS_INFO = ...
   [15 20 45*D2R;
    22.5 12.5 45*D2R;  
    75 85 -45*D2R;
    85 75 -45*D2R;
    35 35 -45*D2R;
    37 37 -45*D2R;
    39 39 -45*D2R;
    41 41 -45*D2R;
    43 43 -45*D2R;
    68 60 -45*D2R;
    68 62 -45*D2R;
    68 64 -45*D2R;
    68 66 -45*D2R;
    68 68 -45*D2R;
    68 70 -45*D2R;
    69 71 -45*D2R;
    70 72 -45*D2R;
    71 73 -45*D2R;
    72 74 -45*D2R;
    73 75 -45*D2R;
    74 76 -45*D2R;
    75 77 -45*D2R;
    76 78 -45*D2R;
    77 79 -45*D2R;
    78 80 -45*D2R;
    79 81 -45*D2R;
    80 82 -45*D2R;
    88.9398   95.9642 -45*D2R;
    90.6000   94.9496 -45*D2R;
    91.9834   96.3331 -45*D2R;
    90.7844   97.1632 -45*D2R;
    93.9203   96.4253 -45*D2R;
    92.6291   94.3962 -45*D2R;
    94.9349   93.9351 -45*D2R;
    96.2261   95.9642 -45*D2R;
    93.9203   98.2700 -45*D2R;
    92.7213   99.3767 -45*D2R; 
    120 125 -45*D2R;
    115 110 -45*D2R];   % X,Y, ANGLE

OBS_NUM = length(OBS_INFO(:,1));
OBS_VRT = 4;

WPT_POS = [10 10; 20 20; 30 30; 40 40; 50 50; 60 60; 70 70; 80 80; 90 90; 100 100; 110 110; 120 120; 130 130; 140 140];  % to be modified.
WPT_VELO = [1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;1 1;];  % only for direction.
WPT_ADMISSIBLE = 3;
BIAS_OBS = 0.1;
BIAS_GOAL = 0.1;

vrt = 0;

for iter_obs = 1 : length(OBS_INFO(:,1))
    if (iter_obs == 1)
        vrt((OBS_VRT+1)*(iter_obs-1)+1:(OBS_VRT+1)*iter_obs,1:2) = OBS_MODEL(1:5,:)*[cos(OBS_INFO(iter_obs,3)) -sin(OBS_INFO(iter_obs,3)); sin(OBS_INFO(iter_obs,3)) cos(OBS_INFO(iter_obs,3))]+([OBS_INFO(iter_obs,1) 0; 0 OBS_INFO(iter_obs,2)]*ones(2,5))';
    elseif (iter_obs == 2)
        vrt((OBS_VRT+1)*(iter_obs-1)+1:(OBS_VRT+1)*iter_obs,1:2) = OBS_MODEL(6:10,:)*[cos(OBS_INFO(iter_obs,3)) -sin(OBS_INFO(iter_obs,3)); sin(OBS_INFO(iter_obs,3)) cos(OBS_INFO(iter_obs,3))]+([OBS_INFO(iter_obs,1) 0; 0 OBS_INFO(iter_obs,2)]*ones(2,5))';
    elseif (iter_obs >= 3) && (iter_obs <= 4)
        vrt((OBS_VRT+1)*(iter_obs-1)+1:(OBS_VRT+1)*iter_obs,1:2) = OBS_MODEL(11:15,:)*[cos(OBS_INFO(iter_obs,3)) -sin(OBS_INFO(iter_obs,3)); sin(OBS_INFO(iter_obs,3)) cos(OBS_INFO(iter_obs,3))]+([OBS_INFO(iter_obs,1) 0; 0 OBS_INFO(iter_obs,2)]*ones(2,5))';        
    elseif (iter_obs >= 5) && (iter_obs <= 27)
        vrt((OBS_VRT+1)*(iter_obs-1)+1:(OBS_VRT+1)*iter_obs,1:2) = OBS_MODEL(16:20,:)*[cos(OBS_INFO(iter_obs,3)) -sin(OBS_INFO(iter_obs,3)); sin(OBS_INFO(iter_obs,3)) cos(OBS_INFO(iter_obs,3))]+([OBS_INFO(iter_obs,1) 0; 0 OBS_INFO(iter_obs,2)]*ones(2,5))';                
    elseif (iter_obs >= 28) && (iter_obs <= 37)
        vrt((OBS_VRT+1)*(iter_obs-1)+1:(OBS_VRT+1)*iter_obs,1:2) = OBS_MODEL(21:25,:)*[cos(OBS_INFO(iter_obs,3)) -sin(OBS_INFO(iter_obs,3)); sin(OBS_INFO(iter_obs,3)) cos(OBS_INFO(iter_obs,3))]+([OBS_INFO(iter_obs,1) 0; 0 OBS_INFO(iter_obs,2)]*ones(2,5))';                
    else
        vrt((OBS_VRT+1)*(iter_obs-1)+1:(OBS_VRT+1)*iter_obs,1:2) = OBS_MODEL(26:30,:)*[cos(OBS_INFO(iter_obs,3)) -sin(OBS_INFO(iter_obs,3)); sin(OBS_INFO(iter_obs,3)) cos(OBS_INFO(iter_obs,3))]+([OBS_INFO(iter_obs,1) 0; 0 OBS_INFO(iter_obs,2)]*ones(2,5))';                        
    end
end

%% UGV Profile

VELO_UGV = [0.1 0.1 0];
POS_UGV = [0 0 45*D2R];

N_step = 20;        % receding horizon time step.
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

figure(3)
hold on;
for iter_cnd = 1 : length(POS_UGV_cnd(1,:))/3
    plot(POS_UGV_cnd(:,3*(iter_cnd-1)+1),POS_UGV_cnd(:,3*(iter_cnd-1)+2),'*-','color','green');
end
for iter_plot = N_angle*4+1 : N_angle*6
    plot(POS_UGV_cnd(:,3*(iter_plot-1)+1),POS_UGV_cnd(:,3*(iter_plot-1)+2),'*-','color','red');
end



for iter_cnd = length(POS_UGV_cnd(1,:))/9+1 : length(POS_UGV_cnd(1,:))*2/9
plot(POS_UGV_cnd(:,3*(iter_cnd-1)+1),POS_UGV_cnd(:,3*(iter_cnd-1)+2),'color','red','linewidth',2);
end
for iter_cnd = 1 : length(POS_UGV_cnd(1,:))/9
plot(POS_UGV_cnd(:,3*(iter_cnd-1)+1),POS_UGV_cnd(:,3*(iter_cnd-1)+2),'color','black','linewidth',2);
end
for iter_cnd = length(POS_UGV_cnd(1,:))*2/9+1 : length(POS_UGV_cnd(1,:))*3/9
plot(POS_UGV_cnd(:,3*(iter_cnd-1)+1),POS_UGV_cnd(:,3*(iter_cnd-1)+2),'color','green','linewidth',2);
end
