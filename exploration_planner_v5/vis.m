v5 = load('map_2_result_5.mat');
v4 = load('map_2_result_4.mat');
figure('Color','white');
plot(v4.path_length_list, v4.range_coverage_list, 'b--', 'LineWidth', 1);
hold on;
plot(v4.path_length_list, v4.vision_coverage_list, 'r--', 'LineWidth', 1);
plot(v5.path_length_list, v5.range_coverage_list, 'b', 'LineWidth', 2);
plot(v5.path_length_list, v5.vision_coverage_list, 'r', 'LineWidth', 2);
hold off;
title('Coverage Curve');
xlabel('distance [m]');
ylabel('coverage [%]');
legend('RH-NBVP range', 'RH-NBVP vision', 'our range', 'our vision');


%%
t = tiledlayout(2,2);

nexttile(1);
show(v5.omap_local_lidar);
title('Range Coverage');
hold on;
plot(v5.robot_pose_list(:,1), v5.robot_pose_list(:,2), 'r');
hold off;

nexttile(2);
show(v5.omap_local_cam);
title('Vision Coverage');
hold on;
plot(v5.robot_pose_list(:,1), v5.robot_pose_list(:,2), 'r');
hold off;

% nexttile(3);
% imagesc(heat);
% axis equal;
% xlim([0 120]);
% title('Heatmap of Potential Vision Coverage');

nexttile(3, [1 2]);
plot(v5.path_length_list, v5.range_coverage_list, 'b', 'LineWidth', 2);
hold on;
plot(v5.path_length_list, v5.vision_coverage_list, 'r', 'LineWidth', 2);
hold off;
xlim([0 200]);
ylim([0 100]);
title('Coverage Curve');
xlabel('distance [m]');
ylabel('coverage [%]');
legend('range', 'vision');

%%
image_file_name = 'map\map_2.jpg';
pad_meters = 0;
resolution = 4;           % 10 cells per meter ⇒ each cell = 0.1m
map_size = [16, 20];        % map size in meter [y, x]
omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size); % ground truth map
figure('Color','white');
show(omap_gt);
title('Map2');