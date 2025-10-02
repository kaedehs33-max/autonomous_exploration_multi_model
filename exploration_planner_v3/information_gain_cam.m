function ig_cam = information_gain_cam(omap_local_cam, omap_local_lidar, robot_pose_next, min_range, max_range, angles)
% this function predicts information gain of camera for a new robot pose
% v3
% omap_local: local map
% robot_pose: [x, y, theta] in meters and radius

colored_angles_pre = 0;
colored_angles_new = 0;


for i = 1:length(angles)
    angle = robot_pose_next(3) + angles(i);
    
    start_pt = robot_pose_next(1:2);  % [x, y] in meters
    end_pt = robot_pose_next(1:2) + max_range * [cos(angle), sin(angle)];
    % Raycast from current pose
    [~, mid_points_pre] = raycast(omap_local_cam,start_pt,end_pt);
    [~, mid_points_new] = raycast(omap_local_lidar,start_pt,end_pt);
    if isempty(mid_points_pre) 
        continue;
    end
    for j = 1:size(mid_points_pre, 1)-1
        uv = mid_points_pre(j, :);
        xy = grid2world(omap_local_cam, uv);
        d = norm(xy - start_pt);
        occ = getOccupancy(omap_local_cam, xy);
        if occ >= 0.65 && d > min_range % previous colored angle
            colored_angles_pre = colored_angles_pre + 1;
            break;
        end
    end

    if isempty(mid_points_new)
        continue;
    end
    for j = 1:size(mid_points_new, 1)-1
        uv = mid_points_new(j, :);
        xy = grid2world(omap_local_lidar, uv);
        d = norm(xy - start_pt);
        occ = getOccupancy(omap_local_lidar, xy);
        if occ >= 0.65 && d > min_range % new colored angle
            colored_angles_new = colored_angles_new + 1;
            break;
        end
    end

end

ig_cam = max(0, colored_angles_new - colored_angles_pre)/length(angles);

end