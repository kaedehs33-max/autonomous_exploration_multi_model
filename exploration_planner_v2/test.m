clear;
clc;
%% generate ground truth map from image
image_file_name = 'map\map_2.jpg';
pad_meters = 5;
resolution = 4;           % 10 cells per meter ⇒ each cell = 0.1m
map_size = [16, 20];        % map size in meter [y, x]
omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size); % ground truth map

%% Define LiDAR parameters
max_range_lidar = 5; 
max_range_cam = 2;
angles = linspace(-pi, pi, 360);  % 360 beams

%% initialize local map as unknown
omap_local_lidar = occupancyMap(map_size(2)+2*pad_meters, map_size(1)+2*pad_meters, resolution);
omap_local_cam = occupancyMap(map_size(2)+2*pad_meters, map_size(1)+2*pad_meters, resolution);
%% initialize robot pose in meters and radius 
robot_pose = [1, 15, 0]+[pad_meters, pad_meters, 0];

%% RHNBV_RRT

% scan
omap_local_lidar = lidar_scan_2d(omap_local_lidar, omap_gt, robot_pose, max_range_lidar, angles);
omap_local_cam = lidar_scan_2d(omap_local_cam, omap_gt, robot_pose, max_range_cam, angles);

aux.start = robot_pose(1:2);
aux.omap = copy(omap_local_lidar);
aux.pad_meters = pad_meters;
aux.max_range = max_range_lidar;
aux.angles = angles;
aux.step_size = 1.5;
aux.reconnect_range = 2 * aux.step_size;
aux.default_iter = 20;
aux.max_iter = 200;
aux.finish_threshold = 0.5;
aux.tree = aux.start; % Tree initialization
aux.lambda = 0.2;

RHNBV_RRT_planner = RHNBV_RRT(aux);

iter_num = 0;
robot_pose_list = robot_pose(1:2);
finish_flag = false;

fig_lidar = figure('Name', 'Lidar_map');
fig_cam = figure('Name', 'Camera_map');

while finish_flag == false
    iter_num = iter_num + 1;

    % scan
    if iter_num ~= 1
        omap_local_lidar = lidar_scan_2d(omap_local_lidar, omap_gt, robot_pose, max_range_lidar, angles);
        omap_local_cam = lidar_scan_2d(omap_local_cam, omap_gt, robot_pose, max_range_cam, angles);
    end

    % plan
    if iter_num ~= 1
        RHNBV_RRT_planner.restart_tree(omap_local_lidar);
    end
    finish_flag = RHNBV_RRT_planner.run();

    % visualize
    figure(fig_lidar);
    show(omap_local_lidar);
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1);RHNBV_RRT_planner.best_branch(:,1)], [robot_pose(2);RHNBV_RRT_planner.best_branch(:,2)], 'b');
        robot_pose(1:2) = RHNBV_RRT_planner.best_branch(1, :);
        robot_pose_list = [robot_pose_list; robot_pose(1:2)];
    end
    hold off;

    figure(fig_cam);
    show(omap_local_cam);
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1);RHNBV_RRT_planner.best_branch(:,1)], [robot_pose(2);RHNBV_RRT_planner.best_branch(:,2)], 'b');
    end
    hold off;
    pause(0.1);
end

path_length = sum(vecnorm(diff(robot_pose_list), 2, 2));