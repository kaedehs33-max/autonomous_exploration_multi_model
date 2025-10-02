function omap_gt = generate_omap(image_file_name, resolution, pad_meters, map_size)
% this function generates a ground truth occupancy map from image

% image_file_name: eg. 'map_1.jpg'
% resolution: cells per meter
% pad_meters: prevent end_point of raycasting get out of the map
% map_size: [y, x] in meters
%%
% load image
img = imread(image_file_name);
% grayscale
img = im2gray(img);
% binarize
bw = imbinarize(img);
% flip 0/1 (in the image, black stands for occupied but 0 stands for vacant in occupancy map)
bw = imcomplement(bw);

map_size_cells = map_size.*resolution;             % size in pixel

bw = imresize(bw, map_size_cells, 'nearest');  % Preserve binary nature

%% pad the map to prevent endpoint of raycasting get outside the map
pad_cells = pad_meters*resolution;
pad_bw = zeros(map_size_cells + 2 * pad_cells);  % free padded
pad_bw(pad_cells+1:end-pad_cells, pad_cells+1:end-pad_cells) = bw;

%%
omap_gt = occupancyMap(pad_bw, resolution);

% show(omap_gt);
% title('True Occupancy Map');

end