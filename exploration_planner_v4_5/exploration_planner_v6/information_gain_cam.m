function information_gain = information_gain_cam(omap_local_cam, omap_local_lidar, robot_pose_next, max_range_cam, angles)
% this function predicts information gain of camera for a new robot pose
% V4
% omap_local: local map
% omap_gt: truth map (scanned map)
% robot_pose: [x, y, theta] in meters and radius
% omap_local_predict = lidar_scan_2d(omap_local_cam, omap_local_lidar, robot_pose_next, max_range_cam, angles);
% 
% % occupancy matrix
% occ_local = occupancyMatrix(omap_local_cam);
% occ_local_predict = occupancyMatrix(omap_local_predict);
% 
% occ_diff = double(occ_local~=occ_local_predict);
% 
% information_gain = sum(occ_diff, "all")/(omap_local_cam.Resolution)^2;

min_range_cam = 0.2;
colored_angles_new = 0;

for i = 1:length(angles)
    angle = robot_pose_next(3) + angles(i);

    start_pt = robot_pose_next(1:2);  % [x, y] in meters
    end_pt = robot_pose_next(1:2) + max_range_cam * [cos(angle), sin(angle)];
    % Raycast from current pose
    [~, mid_points_new] = raycast(omap_local_lidar,start_pt,end_pt);

    for j = 1:size(mid_points_new, 1)
        uv = mid_points_new(j, :);
        xy = grid2world(omap_local_lidar, uv);
        d = norm(xy - start_pt);
        occ = getOccupancy(omap_local_lidar, xy);
        if occ >= 0.65
            if getOccupancy(omap_local_cam, xy) < 0.65 && d > min_range_cam % new colored angle
                colored_angles_new = colored_angles_new + 1;
            end
            break;
        end
    end

end

information_gain = 10 * colored_angles_new/length(angles);

end