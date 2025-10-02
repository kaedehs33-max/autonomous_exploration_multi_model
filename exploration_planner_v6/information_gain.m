function information_gain = information_gain(omap_local, robot_pose_next, max_range, angles)
% this function predicts information gain for a new robot pose

% omap_local: local map
% omap_gt: truth map (scanned map)
% robot_pose: [x, y, theta] in meters and radius
omap_local_predict = lidar_scan_2d(omap_local, omap_local, robot_pose_next, max_range, angles);

% occupancy matrix
occ_local = occupancyMatrix(omap_local);
occ_local_predict = occupancyMatrix(omap_local_predict);

occ_diff = double(occ_local~=occ_local_predict);

information_gain = sum(occ_diff, "all")/(omap_local.Resolution)^2;
end