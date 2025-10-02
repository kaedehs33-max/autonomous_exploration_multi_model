function [ig, ig_lidar, ig_cam] = information_gain(omap_local_lidar, omap_local_cam, robot_pose_next, max_range_lidar, min_range_cam, max_range_cam, angles, alpha)
% this function predicts information gain for a new robot pose

% omap_local: local map
% robot_pose: [x, y, theta] in meters and radius
ig_lidar = information_gain_lidar(omap_local_lidar, robot_pose_next, ...
    max_range_lidar, angles);
ig_cam = information_gain_cam(omap_local_cam, omap_local_lidar, ...
    robot_pose_next, min_range_cam, max_range_cam, angles);

ig = ig_lidar + alpha * ig_cam;
end