function [meet_pts,flag_meet_pts] = scan_obstacle(wpt,vrt,OBS_NUM,OBS_VRT,SCAN_RANGE,SCAN_THETA,del_theta)

meet_pts = [0 0];
flag_meet_pts = 0;
index = 1;
det_mpt = 0;

for srh_angle = -SCAN_THETA : del_theta : SCAN_THETA
    srh_bound = [wpt(1,1),wpt(1,2); wpt(1,1)+SCAN_RANGE*cos(wpt(1,3)+srh_angle),wpt(1,2)+SCAN_RANGE*sin(wpt(1,3)+srh_angle)];
    [meet_pts_tmp,meet_idx_tmp,det_mpt] = find_obstacles(OBS_NUM,OBS_VRT,vrt,srh_bound(1,:),srh_bound(2,:));
    if (det_mpt == 1)
        if ((meet_idx_tmp >=3) && (meet_idx_tmp <= 4))
            meet_pts(index,:) = meet_pts_tmp - 1 + 2*rand(1,2);  % big noize;
        elseif ((meet_idx_tmp >=28) && (meet_idx_tmp <= 37))
            meet_pts(index,:) = meet_pts_tmp - 0.5 + rand(1,2);  % small noize;            
        else
            meet_pts(index,:) = meet_pts_tmp;  % small noize;
        end
        index = index + 1;          
        flag_meet_pts = 1;
    end
    det_mpt = 0;
end

% 포인트에 대한 예외처리.
if length(meet_pts(:,1)) == 1
    meet_pts = [0,0];
    flag_meet_pts = 0;
end