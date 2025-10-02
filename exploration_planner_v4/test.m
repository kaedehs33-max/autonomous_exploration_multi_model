clear;
clc;
% v4
%% generate ground truth map from image
image_file_name = 'map\map_1.jpg';
pad_meters = 5;
resolution = 4;           % 10 cells per meter ⇒ each cell = 0.1m
map_size = [16, 20];        % map size in meter [y, x]
omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size); % ground truth map
load('map_1_gt.mat');
%% Define LiDAR parameters
max_range_lidar = 5; 
max_range_cam = 2;
angles = linspace(-pi, pi, 360);  % 360 beams

%% initialize local map as unknown
omap_local_lidar = occupancyMap(map_size(2)+2*pad_meters, map_size(1)+2*pad_meters, resolution);
omap_local_cam = occupancyMap(map_size(2)+2*pad_meters, map_size(1)+2*pad_meters, resolution);
%% initialize robot pose in meters and radius 
start_pose = [1.5, 15, 0]+[pad_meters, pad_meters, 0];
robot_pose = start_pose;

%% RHNBV_RRT
range_coverage_list = [];
vision_coverage_list = [];
path_length = 0;
path_length_list = 0;

% scan
omap_local_lidar = lidar_scan_2d(omap_local_lidar, omap_gt, robot_pose, max_range_lidar, angles);
omap_local_cam = cam_scan_2d(omap_local_cam, omap_local_lidar, robot_pose, max_range_cam, angles);

% coverage calculate
[range_coverage, vision_coverage] = coverage_calculate(...
    omap_local_lidar, omap_local_cam, ...
    total_scan, total_surf);
range_coverage_list = [range_coverage_list; range_coverage];
vision_coverage_list = [vision_coverage_list; vision_coverage];


aux.start = robot_pose(1:2);
aux.omap_lidar = copy(omap_local_lidar);
aux.omap_cam = copy(omap_local_cam);
aux.pad_meters = pad_meters;
aux.max_range_lidar = max_range_lidar;
aux.max_range_cam = max_range_cam;
aux.angles = angles;
aux.step_size = 1.5;
aux.reconnect_range = 2 * aux.step_size;
aux.default_iter = 30;
aux.max_iter = 210;
aux.finish_threshold = 0.1;
aux.tree = aux.start; % Tree initialization
aux.lambda = 0.3;
aux.ig_cam_threshold = 100;
%
RHNBV_RRT_planner = RHNBV_RRT_1(aux);

iter_num = 0;
robot_pose_list = robot_pose(1:2);
finish_flag = false;

% fig_lidar = figure('Name', 'Lidar_map');
% fig_cam = figure('Name', 'Camera_map');
fig_merged = figure('Name', 'Visualize');

while finish_flag == false
    iter_num = iter_num + 1;
    
    % scan
    if iter_num ~= 1
        vec = RHNBV_RRT_planner.best_branch(1, :) - robot_pose(1:2);
        for i = 1:4
            r_pose = [robot_pose(1:2) + i/4 * vec, 0];
            omap_local_lidar = lidar_scan_2d(omap_local_lidar, omap_gt, r_pose, max_range_lidar, angles);
            omap_local_cam = cam_scan_2d(omap_local_cam, omap_local_lidar, r_pose, max_range_cam, angles);
        end
        path_length = path_length + vecnorm(vec);
        path_length_list = [path_length_list; path_length];

        robot_pose(1:2) = RHNBV_RRT_planner.best_branch(1, :);
        robot_pose_list = [robot_pose_list; robot_pose(1:2)];

        % coverage calculate
        [range_coverage, vision_coverage] = coverage_calculate(...
            omap_local_lidar, omap_local_cam, ...
            total_scan, total_surf);
        range_coverage_list = [range_coverage_list; range_coverage];
        vision_coverage_list = [vision_coverage_list; vision_coverage];
    end

    % plan
    if iter_num ~= 1
        RHNBV_RRT_planner.restart_tree(omap_local_lidar, omap_local_cam);
    end
    finish_flag = RHNBV_RRT_planner.run();

    % visualize
    figure(fig_merged);
    t = tiledlayout(2,2);

    nexttile(1);
    show(omap_local_lidar);
    title('Range Coverage');
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1); RHNBV_RRT_planner.best_branch(:,1)], [robot_pose(2); RHNBV_RRT_planner.best_branch(:,2)], 'b');
    end
    hold off;

    nexttile(2);
    show(omap_local_cam);
    title('Vision Coverage');
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1); RHNBV_RRT_planner.best_branch(:,1)], [robot_pose(2); RHNBV_RRT_planner.best_branch(:,2)], 'b');
    end
    hold off;

    % nexttile(3);
    % imagesc(heat);
    % axis equal;
    % xlim([0 120]);
    % title('Heatmap of Potential Vision Coverage');

    nexttile(3, [1 2]);
    plot(path_length_list, range_coverage_list, 'b', 'LineWidth', 2);
    hold on;
    plot(path_length_list, vision_coverage_list, 'r', 'LineWidth', 2);
    hold off;
    xlim([0 200]);
    ylim([0 100]);
    title('Coverage Curve');
    xlabel('distance [m]');
    ylabel('coverage [%]');
    legend('range', 'vision');
    pause(0.1);
end


save("map_1_result_2", 'vision_coverage_list', 'range_coverage_list', ...
    'path_length_list', 'robot_pose_list', ...
    'omap_local_cam', 'omap_local_lidar');