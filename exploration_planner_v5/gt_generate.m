% map_1
occ = occupancyMatrix(omap_local_lidar);
total_surf = sum(occ>0.65, "all");
total_free = sum(occ<0.25, "all");
total_scan = total_surf + total_free;
save('map_1_gt.mat', "total_scan", "total_surf");