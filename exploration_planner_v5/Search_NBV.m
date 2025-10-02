classdef Search_NBV < handle
    properties
        start   % current robot position (x, y) in meters
        omap    % occupancy map
        pad_meters 
        max_range     % Lidar parameter
        angles        % Lidar parameter
        step_size
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
    end
    
    methods
        % コンストラクタ
        function obj = Search_NBV(aux)
            obj.start = aux.start;
            obj.omap = aux.omap;
            obj.pad_meters = aux.pad_meters;
            obj.max_range = aux.max_range;
            obj.angles = aux.angles;
            obj.step_size = aux.step_size;
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
                end
                iter = iter + obj.default_iter;
            end

            if find_ig == false  % fail to find high ig even with max_iter, finish
                finish_flag = true;
            end

        end % end of function "run"
%%
function restart_tree(obj, new_omap, path, step_exe)
            obj.start = path(step_exe, :);
            best_branch = obj.start;
            if step_exe < size(path, 1)
                path = path(step_exe + 1:end, :);
                num_nodes = 1;
                while num_nodes*step_exe < size(path, 1)
                    best_branch = [best_branch; path(num_nodes*step_exe, :)];
                    num_nodes = num_nodes + 1;
                end
                best_branch = [best_branch; path(end, :)];
            end

            obj.omap = new_omap;
            
            obj.tree = best_branch; % Tree initialization
            obj.parent = (0:size(obj.tree, 1)-1)';  
            obj.ig_raw = 0;   
            obj.ig = 0;
            obj.distance = 0;
            if size(best_branch, 1) > 1
                obj.distance = [0; cumsum(vecnorm(diff(best_branch), 2, 2))];
            end
            obj.nbv = [];

            for i = 2:size(obj.tree, 1)
                new_node = obj.tree(i, :);
                new_ig_raw = information_gain(obj.omap, [new_node, 0], obj.max_range, obj.angles);
                obj.ig_raw = [obj.ig_raw; new_ig_raw];
                new_ig = new_ig_raw * exp(-obj.lambda * obj.distance(i));
                obj.ig = [obj.ig; new_ig];
            end
        end
%%
        function grow_tree(obj, iter_num)
            for iter = 1:iter_num
                add_new_node = 0;
                while add_new_node == 0
                    % randam sampling
                    rand_point = [rand*(obj.omap.XWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters, ...
                        rand*(obj.omap.YWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters];

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
                        [end_points, mid_points] = raycast(obj.omap,nearest_node,new_node);
                        occ = getOccupancy(obj.omap, grid2world(obj.omap, [end_points; mid_points]));
                        if occ < 0.2
                            add_new_node = 1;
                            % append new node to tree
                            obj.tree = [obj.tree; new_node];
                            obj.parent = [obj.parent; nearest_idx];
                            new_ig_raw = information_gain(obj.omap, [new_node, 0], obj.max_range, obj.angles);
                            obj.ig_raw = [obj.ig_raw; new_ig_raw];
                            new_distance = obj.distance(nearest_idx) + r * obj.step_size;
                            obj.distance = [obj.distance; new_distance];
                            new_ig = new_ig_raw * exp(-obj.lambda * new_distance);
                            obj.ig = [obj.ig; new_ig];
                            break;
                        end
                    end
                end % one iteration of creating new node ends
            end % all iteration end
        end

    end
end