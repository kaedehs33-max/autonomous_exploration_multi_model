%% generate ground truth map from image
image_file_name = 'map\map_1.jpg';
pad_meters = 5;
resolution = 5;           % 10 cells per meter ⇒ each cell = 0.1m
map_size = [16, 20];        % map size in meter [y, x]
omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size); % ground truth map

%%
occMatrix = occupancyMatrix(omap_gt);  % 0 = free, 1 = occupied, 0.5 = unknown
occupiedMask = occMatrix > 0.65;
freeMask = occMatrix < 0.25;

distMap = bwdist(occupiedMask) / omap_gt.Resolution;  % in meters

d_peak = 1.5;       % meters
sigma = 0.8;      % meters
heat = exp(-((distMap - d_peak).^2) / (2 * sigma^2));

heat(~freeMask) = 0;  % Remove heat from unknown or occupied regions

figure;
imagesc(heat);
axis equal;
colorbar;
title('Heatmap with peak at 2m from obstacles');

grayRGB = repmat(1-occMatrix, 1, 1, 3);
heat_norm = heat / max(heat(:));
% %%
% colormapUsed = hot(256);
% heatColorIdx = uint8(heat_norm * 255);
% heatRGB = ind2rgb(heatColorIdx, colormapUsed);
% 
% overlayRGB = grayRGB;
% for i = 1:3
%     channel = grayRGB(:,:,i);
%     heatRGB_channel = heatRGB(:,:,i);
%     channel(freeMask) = heatRGB_channel(freeMask);
%     overlayRGB(:,:,i) = channel;
% end
% 
% % Display
% figure;
% imshow(overlayRGB);
% title('Occupancy Map with Heatmap Overlay');
% 
% threshold = 0.8 * max(heat(:));
% [rows, cols] = find(heat > threshold & freeMask);
% points = [rows, cols];  % (Y, X)
% 
% k = 100;  % number of desired clusters
% labels = kmeans(points, k);
% 
% figure; imagesc(heat); axis equal; hold on;
% colors = lines(max(labels)+1);
% for i = 0:max(labels)
%     if i == -1  % noise from DBSCAN
%         idx = labels == -1;
%         plot(points(idx,2), points(idx,1), 'k.');
%     else
%         idx = labels == i;
%         plot(points(idx,2), points(idx,1), '.', 'Color', colors(i+1,:));
%     end
% end
% title('Clusters of Heatmap Points');
% 
% uniqueLabels = unique(labels(labels~=-1));  % exclude noise
% centroids = zeros(length(uniqueLabels), 2);
% for i = 1:length(uniqueLabels)
%     cluster_points = points(labels == uniqueLabels(i), :);
%     centroids(i, :) = mean(cluster_points, 1);
% end
% 
% % Plot centroids
% plot(centroids(:,2), centroids(:,1), 'rx', 'MarkerSize', 12, 'LineWidth', 2);

%%

% Example costmap (10x10)
costmap = ones(size(occMatrix));     % 1 means normal cost

costmap = costmap - 0.9*heat_norm;
costmap(occupiedMask) = inf;

% % Add high-cost zones (e.g., muddy terrain)
% costmap(4:6, 4:6) = 5;      % Higher resistance
% 
% % Add obstacles
% costmap(2, 2:5) = Inf;      % Untraversable

start = world2grid(omap_gt, [6, 18]);
goal  = world2grid(omap_gt, [22, 6]);


path = a_star_costmap(costmap, start, goal);
%%
% Visualize
figure; imagesc(costmap); axis equal; colormap parula; colorbar;
hold on;
if ~isempty(path)
    plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);  % y,x because of image
    plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(2), goal(1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
else
    title('No path found');
end

%%
% Open a new figure for the merged layout
figure;
t = tiledlayout(2,2);  % 2 rows, 2 columns

nexttile(t);  % Go to the next tile
axList  = findall(fig_lidar, 'Type', 'axes');
for ax = flip(axList')  % flip to preserve plotting order
    if isempty(get(ax, 'Children'))
        continue  % skip empty axes (like legends)
    end

    % Copy the contents (lines, images, etc.) into current tile
    newAxes = gca;
    copyobj(allchild(ax), newAxes);

    % Match the axis limits and labels
    set(newAxes, 'XLim', get(ax, 'XLim'));
    set(newAxes, 'YLim', get(ax, 'YLim'));
    xlabel(newAxes, get(get(ax, 'XLabel'), 'String'));
    ylabel(newAxes, get(get(ax, 'YLabel'), 'String'));
    zlabel(newAxes, get(get(ax, 'ZLabel'), 'String'));
    title(newAxes, get(get(ax, 'Title'), 'String'));
end

nexttile(t);  % Go to the next tile
axList  = findall(fig_cam, 'Type', 'axes');
for ax = flip(axList')  % flip to preserve plotting order
    if isempty(get(ax, 'Children'))
        continue  % skip empty axes (like legends)
    end

    % Copy the contents (lines, images, etc.) into current tile
    newAxes = gca;
    copyobj(allchild(ax), newAxes);

    % Match the axis limits and labels
    set(newAxes, 'XLim', get(ax, 'XLim'));
    set(newAxes, 'YLim', get(ax, 'YLim'));
    xlabel(newAxes, get(get(ax, 'XLabel'), 'String'));
    ylabel(newAxes, get(get(ax, 'YLabel'), 'String'));
    zlabel(newAxes, get(get(ax, 'ZLabel'), 'String'));
    title(newAxes, get(get(ax, 'Title'), 'String'));
end

nexttile(t);  % Go to the next tile
axList  = findall(fig_heat, 'Type', 'axes');
for ax = flip(axList')  % flip to preserve plotting order
    if isempty(get(ax, 'Children'))
        continue  % skip empty axes (like legends)
    end

    % Copy the contents (lines, images, etc.) into current tile
    newAxes = gca;
    copyobj(allchild(ax), newAxes);

    % Match the axis limits and labels
    set(newAxes, 'XLim', get(ax, 'XLim'));
    set(newAxes, 'YLim', get(ax, 'YLim'));
    xlabel(newAxes, get(get(ax, 'XLabel'), 'String'));
    ylabel(newAxes, get(get(ax, 'YLabel'), 'String'));
    zlabel(newAxes, get(get(ax, 'ZLabel'), 'String'));
    title(newAxes, get(get(ax, 'Title'), 'String'));
end

nexttile(t);  % Go to the next tile
axList  = findall(fig_curve_rv, 'Type', 'axes');
for ax = flip(axList')  % flip to preserve plotting order
    if isempty(get(ax, 'Children'))
        continue  % skip empty axes (like legends)
    end

    % Copy the contents (lines, images, etc.) into current tile
    newAxes = gca;
    copyobj(allchild(ax), newAxes);

    % Match the axis limits and labels
    set(newAxes, 'XLim', get(ax, 'XLim'));
    set(newAxes, 'YLim', get(ax, 'YLim'));
    xlabel(newAxes, get(get(ax, 'XLabel'), 'String'));
    ylabel(newAxes, get(get(ax, 'YLabel'), 'String'));
    zlabel(newAxes, get(get(ax, 'ZLabel'), 'String'));
    title(newAxes, get(get(ax, 'Title'), 'String'));
end