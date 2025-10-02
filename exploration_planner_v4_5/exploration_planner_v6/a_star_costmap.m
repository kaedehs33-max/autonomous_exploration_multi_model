function path = a_star_costmap(costmap, start, goal)

    sz = size(costmap);
    cameFrom = zeros([sz, 2]);  % store previous cell
    gScore = Inf(sz);           % cost from start
    gScore(start(1), start(2)) = 0;
    
    % Heuristic: Euclidean distance
    h = @(p) norm(p - goal);

    % Priority queue (as linear indices)
    openSet = [sub2ind(sz, start(1), start(2))];
    fScore = Inf(sz);
    % fScore(start(1), start(2)) = h(start);
    fScore(start(1), start(2)) = 0;
    
    % Directions (8-connected)
    dirs = [ -1 -1; -1 0; -1 1;
              0 -1;        0 1;
              1 -1;  1 0;  1 1];

    while ~isempty(openSet)
        % Find current with lowest fScore
        [~, idx] = min(fScore(openSet));
        currentIdx = openSet(idx);
        [cx, cy] = ind2sub(sz, currentIdx);
        current = [cx, cy];
        
        % Check if goal
        if isequal(current, goal)
            % Reconstruct path
            path = current;
            while any(cameFrom(path(1,1), path(1,2), :))
                prev = squeeze(cameFrom(path(1,1), path(1,2), :))';
                path = [prev; path];
            end
            return;
        end
        
        % Remove from open set
        openSet(idx) = [];
        fScore(cx, cy) = Inf;  % prevent reselection

        % For all neighbors
        for i = 1:size(dirs,1)
            nx = cx + dirs(i,1);
            ny = cy + dirs(i,2);
            if nx < 1 || ny < 1 || nx > sz(1) || ny > sz(2)
                continue;
            end
            if isinf(costmap(nx, ny))
                continue;
            end

            step_cost = norm(dirs(i,:)) * costmap(nx, ny);
            tentative_gScore = gScore(cx, cy) + step_cost;
            if tentative_gScore < gScore(nx, ny)
                cameFrom(nx, ny, :) = [cx, cy];
                gScore(nx, ny) = tentative_gScore;
                % fScore(nx, ny) = tentative_gScore + h([nx, ny]);
                fScore(nx, ny) = tentative_gScore + 0;
                if ~ismember(sub2ind(sz, nx, ny), openSet)
                    openSet(end+1) = sub2ind(sz, nx, ny);
                end
            end
        end
    end
    
    path = []; % No path found
end
