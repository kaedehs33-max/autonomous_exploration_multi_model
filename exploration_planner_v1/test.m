clear;
clc;
%% generate ground truth map from image
image_file_name = 'map\map_4.jpg';
pad_meters = 5;
resolution = 5;           % 10 cells per meter ⇒ each cell = 0.1m
map_size = [15, 15];        % map size in meter [y, x]
omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size); % ground truth map

%% Define LiDAR parameters
max_range = 5; 
angles = linspace(-pi, pi, 360);  % 360 beams

%% initialize local map as unknown
omap_local = occupancyMap(map_size(2)+2*pad_meters, map_size(1)+2*pad_meters, resolution);

%% initialize robot pose in meters and radius 
robot_pose = [6, 6, 0]+[pad_meters, pad_meters, 0];

%% RHNBV_RRT
omap_local = lidar_scan_2d(omap_local, omap_gt, robot_pose, max_range, angles);

aux.start = robot_pose(1:2);
aux.omap = copy(omap_local);
aux.pad_meters = pad_meters;
aux.max_range = max_range;
aux.angles = angles;
aux.step_size = 1;
aux.default_iter = 10;
aux.max_iter = 100;
aux.finish_threshold = 0.5;
aux.tree = aux.start; % Tree initialization
aux.lambda = 2;

RHNBV_RRT_planner = RHNBV_RRT(aux);

iter_num = 1;
robot_pose_list = robot_pose(1:2);
finish_flag = RHNBV_RRT_planner.run();

figure;

title(['iteration ' num2str(iter_num)]);
show(omap_local);
hold on;
plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
plot([robot_pose(1);RHNBV_RRT_planner.best_branch(:,1)], [robot_pose(2);RHNBV_RRT_planner.best_branch(:,2)], 'b');
hold off;
pause(0.1);

while finish_flag == false
    robot_pose(1:2) = RHNBV_RRT_planner.best_branch(1, :);
    robot_pose_list = [robot_pose_list; robot_pose(1:2)];
    omap_local = lidar_scan_2d(omap_local, omap_gt, robot_pose, max_range, angles);

    iter_num = iter_num + 1;
    RHNBV_RRT_planner.restart_tree(omap_local);
    finish_flag = RHNBV_RRT_planner.run();
    
    title(['iteration ' num2str(iter_num)]);
    show(omap_local);
    hold on;
    plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_pose_list(:,1), robot_pose_list(:,2), 'r');
    if finish_flag == false
        plot([robot_pose(1);RHNBV_RRT_planner.best_branch(:,1)], [robot_pose(2);RHNBV_RRT_planner.best_branch(:,2)], 'b');
    end
    hold off;
    pause(0.1);
end