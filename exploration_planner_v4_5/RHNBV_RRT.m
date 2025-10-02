classdef RHNBV_RRT < handle
    properties
        start   % current robot position (x, y) in meters
        omap_lidar    % occupancy map
        omap_cam
        pad_meters 
        max_range_lidar     % Lidar parameter
        max_range_cam       % camera parameter
        angles        % Lidar parameter
        step_size
        reconnect_range % radius in meters
        default_iter  % default iteration
        max_iter      % max iteration
        finish_threshold  % determine finish depend on ig
        tree
        parent
        ig_raw  % information gain (not penalized by distance
        ig      % penalized by distance (exp)
        distance  % distance to nodes
        lambda  % distance penalize paramter
        nbv     % next best view
        best_branch   % path to nbv
        best_branch_id
        cache
        best_path
        ig_cam
        ig_cam_threshold
    end
    
    methods
        % コンストラクタ
        function obj = RHNBV_RRT(aux)
            obj.start = aux.start;
            obj.omap_lidar = aux.omap_lidar;
            obj.omap_cam = aux.omap_cam;
            obj.pad_meters = aux.pad_meters;
            obj.max_range_lidar = aux.max_range_lidar;
            obj.max_range_cam = aux.max_range_cam;
            obj.angles = aux.angles;
            obj.step_size = aux.step_size;
            obj.reconnect_range = aux.reconnect_range;
            obj.default_iter = aux.default_iter;
            obj.max_iter = aux.max_iter;
            obj.finish_threshold = aux.finish_threshold;
            obj.tree = aux.tree; % Tree initialization
            obj.parent = 0;   
            obj.ig_raw = 0;   
            obj.ig = 0;   
            obj.distance = 0;
            obj.lambda = aux.lambda;
            obj.nbv = [];
            obj.best_branch = [];
            obj.best_branch_id = [];
            obj.cache = [];
            obj.best_path = [];
            obj.ig_cam = 0;
            obj.ig_cam_threshold = aux.ig_cam_threshold;
        end
        %%
        % RRTアルゴリズムの実行
        function finish_flag = run(obj)
            
            finish_flag = false;
            find_ig = false;
            iter = obj.default_iter;
            while find_ig == false && iter <= obj.max_iter
                grow_tree(obj, obj.default_iter);
                [~, nbv_id] = max(obj.ig);
                [best_ig_raw, ~] = max(obj.ig_raw);
                if best_ig_raw > obj.finish_threshold % high predicted information gain
                    find_ig = true;
                    obj.nbv = obj.tree(nbv_id, :);
                    obj.best_branch = obj.nbv;
                    obj.best_branch_id = nbv_id;
                    parent_to_nbv = obj.parent(nbv_id);
                    while parent_to_nbv ~= 1
                        obj.best_branch = [obj.tree(parent_to_nbv, :); obj.best_branch];
                        obj.best_branch_id = [parent_to_nbv; obj.best_branch_id];
                        parent_to_nbv = obj.parent(parent_to_nbv);
                    end
                end
                iter = iter + obj.default_iter;
            end

            if find_ig == false  % fail to find high ig even with max_iter, finish
                finish_flag = true;
            end

        end % end of function "run"
%%
function restart_tree(obj, new_omap_lidar, new_omap_cam)
            obj.start = obj.best_branch(1, :);
            obj.omap_cam = new_omap_cam;
            obj.omap_lidar = new_omap_lidar;
            obj.tree = obj.best_branch; % Tree initialization
            obj.parent = (0:size(obj.tree, 1)-1)';  
            obj.ig_raw = 0;   
            obj.ig = 0;  
            obj.distance = obj.distance(obj.best_branch_id);
            obj.nbv = [];
            obj.best_branch = [];
            obj.ig_cam = 0;
            
            for i = 2:size(obj.tree, 1)
                new_node = obj.tree(i, :);
                new_ig_raw = information_gain(obj.omap_lidar, [new_node, 0], obj.max_range_lidar, obj.angles);
                obj.ig_raw = [obj.ig_raw; new_ig_raw];
                new_ig = new_ig_raw * exp(-obj.lambda * obj.distance(i));
                obj.ig = [obj.ig; new_ig];
                new_ig_cam = information_gain_cam(obj.omap_cam, obj.omap_lidar, [new_node, 0], obj.max_range_cam, obj.angles);
                obj.ig_cam = [obj.ig_cam; new_ig_cam];
            end

            if isempty(obj.cache) == false
                prune_list = [];
                for j = 1:size(obj.cache, 1)
                    if information_gain_cam(obj.omap_cam, obj.omap_lidar, [obj.cache(j, :), 0], obj.max_range_cam, obj.angles) < obj.ig_cam_threshold
                        prune_list = [prune_list; j];
                    end
                end
                if isempty(prune_list) == false
                    disp('prune_list');
                    disp(prune_list);
                    obj.cache(prune_list, :) = [];
                end
            end
        end
%%
        function grow_tree(obj, iter_num)
            for iter = 1:iter_num
                add_new_node = 0;
                while add_new_node == 0
                    % randam sampling
                    rand_point = [rand*(obj.omap_lidar.XWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters, rand*(obj.omap_lidar.YWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters];

                    % nearest node
                    Mdl = KDTreeSearcher(obj.tree);  % grow kd-tree for nearest neighbor search
                    nearest_idx = knnsearch(Mdl, rand_point);
                    nearest_node = Mdl.X(nearest_idx, :);
                    % steer new node to nearest node
                    edge_length = norm(rand_point - nearest_node);
                    direction = (rand_point - nearest_node) / edge_length;
                    
                    % check if there is obstacle or unknow area along the
                    % edge (occ > 0.2) and stear the new_node to free area
                    for i = 1:3
                        r = 1-i/4;
                        new_node = nearest_node + direction * r * obj.step_size;
                        [end_points, mid_points] = raycast(obj.omap_lidar,nearest_node,new_node);
                        occ = getOccupancy(obj.omap_lidar, grid2world(obj.omap_lidar, [end_points; mid_points]));
                        if occ < 0.2
                            add_new_node = 1;
                            % append new node to tree
                            obj.tree = [obj.tree; new_node];
                            obj.parent = [obj.parent; nearest_idx];
                            new_ig_raw = information_gain(obj.omap_lidar, [new_node, 0], obj.max_range_lidar, obj.angles);
                            obj.ig_raw = [obj.ig_raw; new_ig_raw];
                            new_distance = obj.distance(nearest_idx) + r * obj.step_size;
                            obj.distance = [obj.distance; new_distance];
                            new_ig = new_ig_raw * exp(-obj.lambda * new_distance);
                            obj.ig = [obj.ig; new_ig];
                            % ..
                            new_ig_cam = information_gain_cam(obj.omap_cam, obj.omap_lidar, [new_node, 0], obj.max_range_cam, obj.angles);
                            obj.ig_cam = [obj.ig_cam; new_ig_cam];
                            if new_ig_cam >= obj.ig_cam_threshold
                                obj.cache = [obj.cache; new_node];
                            end
                            break;
                        end
                    end
                end % one iteration of creating new node ends

                % reconnect new node inside radius (obj.reconnect_range)
                new_node_id = size(obj.tree, 1);
                [idx, D] = rangesearch(obj.tree, obj.tree(new_node_id, :), obj.reconnect_range);
                inrange_id = idx{1};
                distance_to_node = D{1};
                for i = 1:length(inrange_id)
                    id = inrange_id(i);
                    d = distance_to_node(i);
                    if id == new_node_id % skip the new added node itself
                        continue;
                    end
                    if obj.distance(id) + d < obj.distance(new_node_id)
                        % check obstacles along new edge
                        [end_points, mid_points] = raycast(obj.omap_lidar, obj.tree(id, :), obj.tree(new_node_id, :));
                        occ = getOccupancy(obj.omap_lidar, grid2world(obj.omap_lidar, [end_points; mid_points]));
                        if occ < 0.2
                            % reconnect new node
                            obj.parent(new_node_id) = id;
                            obj.distance(new_node_id) = obj.distance(id) + d;
                            obj.distance(new_node_id) = obj.distance(new_node_id);
                            obj.ig(new_node_id) = obj.ig_raw(new_node_id) * exp(-obj.lambda * obj.distance(new_node_id));
                        end
                    end
                end

                % reconnect other nodes inside radius
                for i = 1:length(inrange_id)
                    id = inrange_id(i);
                    d = distance_to_node(i);
                    if id == new_node_id || id == 1 % skip the new added node itself and start point
                        continue;
                    end
                    if obj.distance(id) > obj.distance(new_node_id) + d
                        % check obstacles along new edge
                        [end_points, mid_points] = raycast(obj.omap_lidar,obj.tree(id, :),obj.tree(new_node_id, :));
                        occ = getOccupancy(obj.omap_lidar, grid2world(obj.omap_lidar, [end_points; mid_points]));
                        if occ < 0.2
                            % reconnect to new node
                            obj.parent(id) = new_node_id;
                            obj.distance(id) = obj.distance(new_node_id) + d;
                            obj.ig(id) = obj.ig_raw(id) * exp(-obj.lambda * obj.distance(id));
                        end
                    end
                end

            end % all iteration end
        end

    end
end