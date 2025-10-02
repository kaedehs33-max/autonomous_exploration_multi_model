function omap_local_new = lidar_scan_2d(omap_local, omap_gt, robot_pose, max_range, angles)
% omap_local: local map
% omap_gt: truth map (scanned map)
% robot_pose: [x, y, theta] in meters and radius
% max_range: in meters, no bigger than pad_meters
% angles: colume vector of scanning angles in radius
% set the same as max_range

%% Perform LiDAR scan (raycasting from robot pose)
omap_local_new = copy(omap_local);
% LiDAR scan simulation
for i = 1:length(angles)
    angle = robot_pose(3) + angles(i);
    
    start_pt = robot_pose(1:2);  % [x, y] in meters
    end_pt = start_pt + max_range * [cos(angle), sin(angle)];
    % Raycast from current pose
    [~, mid_points] = raycast(omap_gt,start_pt,end_pt);
    if isempty(mid_points)
        continue;
    end
    for j = 1:size(mid_points, 1)-1
        uv = mid_points(j, :);
        xy = grid2world(omap_gt, uv);
        occ = getOccupancy(omap_gt, xy);
        if occ >= 0.65
            setOccupancy(omap_local_new,xy,1);
            break;
        else
            setOccupancy(omap_local_new,xy,0);
        end
    end
end

end