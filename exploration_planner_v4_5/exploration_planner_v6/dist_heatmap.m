function [heat, heat_norm, costmap] = dist_heatmap(omap_local_lidar, omap_local_cam, d_peak, sigma)
occMatrix = occupancyMatrix(omap_local_lidar);  % 0 = free, 1 = occupied, 0.5 = unknown
visMatrix = occupancyMatrix(omap_local_cam);
occupiedMask = occMatrix > 0.65;
scanned_occupiedMask = visMatrix > 0.65;
freeMask = occMatrix < 0.25;
[distMap, id] = bwdist(occupiedMask);
[~, id_scanned] = bwdist(scanned_occupiedMask);
distMap = distMap / omap_local_lidar.Resolution;
distMap(id == id_scanned) = inf;

heat = exp(-((distMap - d_peak).^2) / (2 * sigma^2));

heat(~freeMask) = 0;  % Remove heat from unknown or occupied regions

heat_norm = heat / max(heat(:));

costmap = ones(size(occMatrix));     % 1 means normal cost

costmap = costmap - 0.8*heat_norm;
costmap(~freeMask) = inf;
end