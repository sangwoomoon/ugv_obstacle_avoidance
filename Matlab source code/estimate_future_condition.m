function [POS_UGV_cnd, VELO_UGV_cnd, VELO_mag, VELO_omega] = estimate_future_condition(POS_now, VELO_now, VELO_LAW, VELO_HIGH, N_step, N_angle, dt, MAX_omega, MAX_omega_dot, FLAG)

% 1 : acceleration
% 0 : constant velocity
% -1 : decceleration

% only for KINAMATICS.

for iter_step = 1 : N_step
    if (FLAG == 1)
        VELO_mag(1,iter_step) = norm(VELO_now(1,1:2))+(VELO_HIGH-norm(VELO_now(1,1:2)))/N_step*iter_step;
    elseif (FLAG == -1)
        if (norm(VELO_now) > VELO_LAW)
            VELO_mag(1,iter_step) =norm(VELO_now(1,1:2))-(norm(VELO_now(1,1:2))-VELO_LAW)/N_step*iter_step;
        else
            VELO_mag(1,iter_step) =norm(VELO_now(1,1:2))+((VELO_HIGH-VELO_LAW)/2+VELO_LAW-norm(VELO_now(1,1:2)))/N_step*iter_step;
        end
    elseif (FLAG == 0)
        if (norm(VELO_now) > VELO_LAW)
            VELO_mag(1,iter_step) = norm(VELO_now(1,1:2));
        else
            VELO_mag(1,iter_step) =norm(VELO_now(1,1:2))-(norm(VELO_now(1,1:2))-VELO_LAW)/N_step*iter_step;
        end
    end

    omega_dot = MAX_omega_dot*(0.9*exp(-VELO_mag(1,iter_step)^2/20)+0.1);
    sat_omega = MAX_omega*(0.9*exp(-VELO_mag(1,iter_step)^2/20)+0.1);
    
    for iter_angle = 1 : N_angle+1
        if (iter_step == 1)
            POS_UGV_cnd(1,3*(iter_angle-1)+1:3*iter_angle) = POS_now;
            VELO_UGV_cnd(1,3*(iter_angle-1)+1:3*iter_angle) = VELO_now;
        end
        
        VELO_omega(1,iter_step) = VELO_now(3)+omega_dot*(-1+2*(iter_angle-1)/N_angle)*dt;
        heading = POS_UGV_cnd(iter_step,3*iter_angle) + VELO_omega(1,iter_step)*dt;

        VELO_UGV_cnd(iter_step+1,3*(iter_angle-1)+1:3*iter_angle) = [VELO_mag(1,iter_step)*cos(heading) VELO_mag(1,iter_step)*sin(heading) VELO_omega(1,iter_step)];
        POS_UGV_cnd(iter_step+1,3*(iter_angle-1)+1:3*iter_angle) = [POS_UGV_cnd(iter_step,3*(iter_angle-1)+1:3*(iter_angle-1)+2)+VELO_mag(1,iter_step)*dt*[cos(heading) sin(heading)] heading];
    end
end