function ig_lidar = information_gain_lidar(omap_local_lidar, robot_pose_next, max_range_lidar, angles)
% this function predicts information gain of lidar for a new robot pose

% omap_local: local map
% omap_gt: truth map (scanned map)
% robot_pose: [x, y, theta] in meters and radius
omap_local_lidar_predict = lidar_scan_2d(omap_local_lidar, omap_local_lidar, robot_pose_next, max_range_lidar, angles);

% occupancy matrix of lidar
occ_local_lidar = occupancyMatrix(omap_local_lidar);
occ_local_lidar_predict = occupancyMatrix(omap_local_lidar_predict);
occ_diff_lidar = double(occ_local_lidar~=occ_local_lidar_predict);

ig_lidar = sum(occ_diff_lidar, "all")/(omap_local_lidar.Resolution)^2;

end