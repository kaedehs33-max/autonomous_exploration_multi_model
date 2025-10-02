function [range_coverage, vision_coverage] = coverage_calculate(...
    omap_local_lidar, omap_local_cam, ...
    total_scan, total_surf)
% coverage calculate
occ_lidar = occupancyMatrix(omap_local_lidar);
occ_cam = occupancyMatrix(omap_local_cam);
current_surf_cam = sum(occ_cam>0.65, "all");
current_surf_lidar = sum(occ_lidar>0.65, "all");
current_free_lidar = sum(occ_lidar<0.25, "all");
current_scan_lidar = current_surf_lidar + current_free_lidar;
range_coverage = current_scan_lidar / total_scan * 100; % percentage
vision_coverage = current_surf_cam / total_surf * 100; % percentage

end