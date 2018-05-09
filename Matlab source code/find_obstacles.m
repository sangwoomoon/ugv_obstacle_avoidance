function [meet_pt,meet_idx,det_mpt] = find_obstacles(OBS_NUM,OBS_VRT,v,wpt_1,wpt_2)

det_mpt = 0;
index = 1;
for idx_bldg = 1 : OBS_NUM
    for idx_edge = 1 : OBS_VRT
        %%%singular case를 막기 위함.
        if abs(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)) < 0.0000001
            v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1) = v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1) + 0.0001;
        end
        if abs((wpt_1(1,1)-wpt_2(1,1))) < 0.0000001
            wpt_1(1,1) = wpt_1(1,1) + 0.0001;
        end
        
        a = (v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,2)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2))/(v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,1)-v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1));   
        b = (wpt_1(1,2)-wpt_2(1,2))/(wpt_1(1,1)-wpt_2(1,1));

        mpt_tmp(1,1) = (-a*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+b*wpt_2(1,1)-wpt_2(1,2))/(-a+b);
        mpt_tmp(1,2) = (-a*b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,1)+b*v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,2)+a*b*wpt_2(1,1)-a*wpt_2(1,2))/(-a+b);
        
        if (determine_on_line(mpt_tmp,v((OBS_VRT+1)*(idx_bldg-1)+idx_edge,[1,2]),v((OBS_VRT+1)*(idx_bldg-1)+idx_edge+1,[1,2])))   
            if (determine_on_line(mpt_tmp,wpt_1,wpt_2))
                meet_pt(index,:) = mpt_tmp;
                meet_idx(index,:) = idx_bldg;
                det_mpt = 1;
                index = index + 1;
            end
        end
    end
end

if (det_mpt == 1)
    if (length(meet_pt(:,1)) >= 2)
        min = distance(meet_pt(1,:),wpt_1);
        meet_pt_res = meet_pt(1,:);
        meet_idx_res = meet_idx(1,:);
        for i = 2 : length(meet_pt(:,1))
            if (min > distance(meet_pt(i,:),wpt_1))
                min = distance(meet_pt(i,:),wpt_1);
                meet_pt_res = meet_pt(i,:);
                meet_idx_res = meet_idx(i,:);
            end
        end
        clear meet_pt;
        meet_pt = meet_pt_res;
        meet_idx = meet_idx_res;
    end
end

if (det_mpt == 0)
    meet_pt = 0;
    meet_idx = 0;
end