classdef frontiers < handle
    properties
        N_max
        omap_lidar
        omap_cam
        ig_threshold
        frontier_list % current frontiers sampled
        pad_meters
        max_range
        angles
    end
    methods
        % constructor
        function obj = frontiers(aux)
            obj.N_max = aux.N_max;
            obj.omap_lidar = aux.omap_lidar;
            obj.omap_cam = aux.omap_cam;
            obj.ig_threshold = aux.ig_threshold;
            obj.frontier_list = [];
            obj.pad_meters = aux.pad_meters;
            obj.max_range = aux.max_range;
            obj.angles = aux.angles;
        end

        function fsample(obj)
            % frontier sample
            N = size(obj.frontier_list, 2); % current frontier number
            if N < obj.N_max
                for i = N+1:obj.N_max
                    isfrontier = false;
                    while isfrontier == false
                    % randam sampling
                    rand_point = [rand*(obj.omap.XWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters, ...
                        rand*(obj.omap.YWorldLimits(2)-2*obj.pad_meters)+obj.pad_meters];
                    occ = getOccupancy(obj.omap, rand_point);
                    if occ < 0.2
                        ig = information_gain(obj.omap, [rand_point, 0], obj.max_range, obj.angles);
                        if ig >= obj.ig_threshold
                            isfrontier = true;
                            obj.frontier_list = [obj.frontier_list; rand_point];
                        end
                    end
                    end


                end
            end
        end
    end

end