classdef RHNBV_RRT < handle
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
        best_branch   % path to nbv
    end
    
    methods
        % コンストラクタ
        function obj = RHNBV_RRT(aux)
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
            obj.best_branch = [];
        end
%%       
        % RRTアルゴリズムの実行
        function finish_flag = run(obj)
            finish_flag = false;
            grow_tree(obj, 1, obj.default_iter);
            [best_ig, nbv_id] = max(obj.ig);
            [best_ig_raw, nbv_raw_id] = max(obj.ig_raw);
            if best_ig_raw > obj.finish_threshold % high predicted information gain
                obj.nbv = obj.tree(nbv_id, :);
                obj.best_branch = obj.nbv;
                parent_to_nbv = obj.parent(nbv_id);
                while parent_to_nbv ~= 1
                    obj.best_branch = [obj.tree(parent_to_nbv, :); obj.best_branch];
                    parent_to_nbv = obj.parent(parent_to_nbv);
                end
            else % not high enough information gain in default_inter, continue
                grow_tree(obj, obj.default_iter + 1, obj.max_iter);
                [best_ig, nbv_id] = max(obj.ig);
                [best_ig_raw, nbv_raw_id] = max(obj.ig_raw);
                if best_ig_raw > obj.finish_threshold  % high predicted information gain
                    obj.nbv = obj.tree(nbv_id, :);
                    obj.best_branch = obj.nbv;
                    parent_to_nbv = obj.parent(nbv_id);
                    while parent_to_nbv ~= 1
                        obj.best_branch = [obj.tree(parent_to_nbv, :); obj.best_branch];
                        parent_to_nbv = obj.parent(parent_to_nbv);
                    end
                else % assume no unexplored area
                    finish_flag = true;
                end
            end
        end % end of function "run"
%%
        function restart_tree(obj, new_omap)
            obj.start = obj.best_branch(1, :);
            obj.omap = new_omap;
            obj.tree = obj.best_branch; % Tree initialization
            obj.parent = (0:size(obj.tree, 1)-1)';  
            obj.ig_raw = 0;   
            obj.ig = 0;  
            obj.distance = obj.step_size*(obj.parent);
            obj.nbv = [];
            obj.best_branch = [];
            
            for i = 2:size(obj.tree, 1)
                new_node = obj.tree(i, :);
                new_ig_raw = information_gain(obj.omap, [new_node, 0], obj.max_range, obj.angles);
                obj.ig_raw = [obj.ig_raw; new_ig_raw];
                new_ig = new_ig_raw * exp(-obj.lambda * obj.distance(i));
                obj.ig = [obj.ig; new_ig];
            end
        end
%%
        function grow_tree(obj, iter_start, iter_end)
            for iter = iter_start:iter_end
                while 1
                    % randam sampling in free space
                    while 1   % sample random_point only in free area (occ < 0.2)
                        rand_point = [rand*(obj.omap.XWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters, rand*(obj.omap.YWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters];
                        if getOccupancy(obj.omap, rand_point) < 0.2
                            break;
                        end
                    end

                    % nearest node
                    distances = vecnorm(obj.tree - rand_point, 2, 2);
                    [~, nearest_idx] = min(distances);
                    nearest_node = obj.tree(nearest_idx, :);

                    % steer new node to nearest node
                    direction = (rand_point - nearest_node) / norm(rand_point - nearest_node);
                    new_node = nearest_node + obj.step_size * direction;

                    % check if there is obstacle or unknow area along the edge (occ > 0.2)
                    [~, mid_points] = raycast(obj.omap,nearest_node,new_node);
                    occ = getOccupancy(obj.omap, grid2world(obj.omap, mid_points));
                    if occ < 0.2
                        % append new node to tree
                        obj.tree = [obj.tree; new_node];
                        obj.parent = [obj.parent; nearest_idx];
                        new_ig_raw = information_gain(obj.omap, [new_node, 0], obj.max_range, obj.angles);
                        obj.ig_raw = [obj.ig_raw; new_ig_raw];
                        step_num = 1;
                        parent_to_node = nearest_idx;
                        while parent_to_node~=0
                            step_num = step_num + 1;
                            parent_to_node = obj.parent(parent_to_node);
                        end
                        new_distance = step_num * obj.step_size;
                        obj.distance = [obj.distance; new_distance];
                        new_ig = new_ig_raw * exp(-obj.lambda * new_distance);
                        obj.ig = [obj.ig; new_ig];
                        break;
                    end
                end % one iteration of creating new node ends
            end % all iteration end
        end

    end
end