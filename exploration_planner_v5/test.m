clear;
clc;
%% generate ground truth map from image
image_file_name = 'map\map_1.jpg';
pad_meters = 5;
resolution = 4;           % 10 cells per meter ⇒ each cell = 0.1m
map_size = [16, 20];        % map size in meter [y, x]
omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size); % ground truth map
load("map_1_gt.mat");
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

%% explore
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
aux.omap = copy(omap_local_lidar);
aux.pad_meters = pad_meters;
aux.max_range = max_range_lidar;
aux.angles = angles;
aux.step_size = 1.5;
aux.default_iter = 30;
aux.max_iter = 210;
aux.finish_threshold = 0.1;
aux.tree = aux.start; % Tree initialization
aux.lambda = 0.3;
%

NBV_Searcher = Search_NBV(aux);

iter_num = 0;
robot_pose_list = robot_pose(1:2);
finish_flag = false;

% heat parameters
d_peak = 1.2;     % meters
sigma = 0.8;      % meters

% fig_lidar = figure('Name', 'Lidar_map');
% fig_cam = figure('Name', 'Camera_map');
% fig_heat = figure('Name', 'Heat_map');
% fig_curve_rv = figure('Name', 'Distance_Coverage_Curve');
fig_merged = figure('Name', 'Visualize');

while finish_flag == false
    iter_num = iter_num + 1;
    
    % scan
    if iter_num ~= 1
        for i = 1:step_exe
            r_pose = [path(i, :), 0];
            omap_local_lidar = lidar_scan_2d(omap_local_lidar, omap_gt, r_pose, max_range_lidar, angles);
            omap_local_cam = cam_scan_2d(omap_local_cam, omap_local_lidar, r_pose, max_range_cam, angles);
        end
        path_length = path_length + sum(vecnorm(diff([robot_pose(1:2); path(1:step_exe, :)]), 2, 2));
        path_length_list = [path_length_list; path_length];

        robot_pose(1:2) = path(step_exe, :);
        robot_pose_list = [robot_pose_list; path(1:step_exe, :)];
        % coverage calculate
        [range_coverage, vision_coverage] = coverage_calculate(...
            omap_local_lidar, omap_local_cam, ...
            total_scan, total_surf);
        range_coverage_list = [range_coverage_list; range_coverage];
        vision_coverage_list = [vision_coverage_list; vision_coverage];
    end

    % search nbv
    if iter_num ~= 1
        NBV_Searcher.restart_tree(omap_local_lidar, path, step_exe);
    end
    finish_flag = NBV_Searcher.run();

    if finish_flag == false
        % Dijkstra path planning
        [heat, ~, costmap] = dist_heatmap(omap_local_lidar, omap_local_cam, d_peak, sigma);
        % path planning to nbv
        path = a_star_costmap(costmap, ...
            world2grid(omap_local_lidar, robot_pose(1:2)), ...
            world2grid(omap_local_lidar, NBV_Searcher.nbv)...
            );
        path = grid2world(omap_local_lidar, path);
    end

    figure(fig_merged);
    t = tiledlayout(2,2);

    nexttile(1);
    show(omap_local_lidar);
    title('Range Coverage');
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1); path(:,1)], [robot_pose(2); path(:,2)], 'b');
    end
    hold off;

    nexttile(2);
    show(omap_local_cam);
    title('Vision Coverage');
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1); path(:,1)], [robot_pose(2); path(:,2)], 'b');
        step_exe = min(10, size(path, 1));
    end
    hold off;

    nexttile(3);
    imagesc(heat);
    axis equal;
    xlim([0 120]);
    title('Heatmap of Potential Vision Coverage');

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
%%
% range exploration finishes
% remaining vision exploration
addpath('matlab-tsp-ga-master');

[heat, ~, ~] = dist_heatmap(omap_local_lidar, omap_local_cam, d_peak, sigma);
occMatrix = occupancyMatrix(omap_local_lidar);
freeMask = occMatrix < 0.25;
threshold = 0.8;
[rows, cols] = find(heat > threshold & freeMask);
points = [rows, cols];  % (Y, X)

if isempty(points)
    return;
end


k = round(size(points, 1)/20);  % number of desired clusters
[labels, centroids] = kmeans(points, k);
vis_targets = grid2world(omap_local_lidar, round(centroids));
vis_targets = [robot_pose(1:2); vis_targets];

costmap = ones(size(occMatrix));
costmap(~freeMask) = inf;


n = k+1;

D = zeros(n);

for i = 1:n
    for j = 1:i
        if i == j
            D(i,j) = 0;
            continue;
        end
        path = a_star_costmap(costmap, ...
            world2grid(omap_local_lidar, vis_targets(i, :)), ...
            world2grid(omap_local_lidar, vis_targets(j, :))...
            );
        path = grid2world(omap_local_lidar, path);
        path = [vis_targets(i, :); path];
        dist = sum(vecnorm(diff(path), 2, 2));
        D(i,j) = dist;
        D(j,i) = dist;
    end
end

resultStruct = tspofs_ga('xy', vis_targets);

tourPoints = vis_targets(resultStruct.optRoute, :);

tourPoints = [robot_pose(1:2); tourPoints];

tsp_path = robot_pose(1:2);

for i = 1:k
    path = a_star_costmap(costmap, ...
        world2grid(omap_local_lidar, tourPoints(i, :)), ...
        world2grid(omap_local_lidar, tourPoints(i+1, :))...
        );
    path = grid2world(omap_local_lidar, path);
    tsp_path = [tsp_path; path];
end

% figure('Color',"white")
% imagesc(heat);
% axis equal;
% xlim([0 120]);
% title('remaining vision targets');
% hold on;
% plot(vis_targets(:,1)*4, 104-vis_targets(:,2)*4, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
% plot(tsp_path(:,1)*4, 104-tsp_path(:,2)*4, 'r', 'LineWidth',2);
% hold off;

% visualize
figure('color','white'); hold on; axis equal;
show(omap_local_cam);
title('final path');
plot(tourPoints(:,1), tourPoints(:,2), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
plot(tsp_path(:,1), tsp_path(:,2), 'r');
hold off;
pause(1);

for i = 1:size(tsp_path, 1)
    r_pose = [tsp_path(i, :), 0];
    robot_pose = r_pose;
    omap_local_lidar = lidar_scan_2d(omap_local_lidar, omap_gt, r_pose, max_range_lidar, angles);
    omap_local_cam = cam_scan_2d(omap_local_cam, omap_local_lidar, r_pose, max_range_cam, angles);
    path_length = path_length + vecnorm(tsp_path(i, :) - robot_pose_list(end, :));
    robot_pose_list = [robot_pose_list; tsp_path(i, :)];

    if mod(i, 10) == 0 || i == size(tsp_path, 1)
        path_length_list = [path_length_list; path_length];
        % coverage calculate
        [range_coverage, vision_coverage] = coverage_calculate(...
            omap_local_lidar, omap_local_cam, ...
            total_scan, total_surf);
        range_coverage_list = [range_coverage_list; range_coverage];
        vision_coverage_list = [vision_coverage_list; vision_coverage];

        % visualize
        % figure(fig_merged);
        t = tiledlayout(2,2);

        nexttile(1);
        show(omap_local_lidar);
        title('Range Coverage');
        hold on;
        plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
        plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
        hold off;

        nexttile(2);
        show(omap_local_cam);
        title('Vision Coverage');
        hold on;
        plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
        plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
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
end

% save("map_2_result_5", 'vision_coverage_list', 'range_coverage_list', ...
%     'path_length_list', 'robot_pose_list', ...
%     'omap_local_cam', 'omap_local_lidar');