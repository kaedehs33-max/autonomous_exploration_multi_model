%TSPO_GA_TURBO Open Traveling Salesman Problem (TSP) Genetic Algorithm (GA)
%   Finds a (near) optimal solution to a variation of the TSP by setting up
%   a GA to search for the shortest route (least distance for the salesman
%   to travel to each city exactly once without returning to the starting city)
%
% Summary:
%     1. A single salesman travels to each of the cities but does not close
%        the loop by returning to the city he started from
%     2. Each city is visited by the salesman exactly once
%
% Input:
%     USERCONFIG (structure) with zero or more of the following fields:
%     - XY (float) is an Nx2 matrix of city locations, where N is the number of cities
%     - DMAT (float) is an NxN matrix of point to point distances/costs
%     - POPSIZE (scalar integer) is the size of the population (should be divisible by 4)
%     - NUMITER (scalar integer) is the number of desired iterations for the algorithm to run
%     - SHOWPROG (scalar logical) shows the GA progress if true
%     - SHOWRESULT (scalar logical) shows the GA results if true
%     - SHOWWAITBAR (scalar logical) shows a waitbar if true
%
% Input Notes:
%     1. Rather than passing in a structure containing these fields, any/all of
%        these inputs can be passed in as parameter/value pairs in any order instead.
%     2. Field/parameter names are case insensitive but must match exactly otherwise.
%
% Output:
%     RESULTSTRUCT (structure) with the following fields:
%         (in addition to a record of the algorithm configuration)
%     - OPTROUTE (integer array) is the best route found by the algorithm
%     - MINDIST (scalar float) is the cost of the best route
%
% Usage:
%     tspo_ga_turbo
%       -or-
%     tspo_ga_turbo(userConfig)
%       -or-
%     resultStruct = tspo_ga_turbo;
%       -or-
%     resultStruct = tspo_ga_turbo(userConfig);
%       -or-
%     [...] = tspo_ga_turbo('Param1',Value1,'Param2',Value2, ...);
%
% Example:
%     % Let the function create an example problem to solve
%     tspo_ga_turbo;
%
% Example:
%     % Request the output structure from the solver
%     resultStruct = tspo_ga_turbo;
%
% Example:
%     % Pass a random set of user-defined XY points to the solver
%     userConfig = struct('xy',10*rand(50,2));
%     resultStruct = tspo_ga_turbo(userConfig);
%
% Example:
%     % Pass a more interesting set of XY points to the solver
%     n = 50;
%     phi = (sqrt(5)-1)/2;
%     theta = 2*pi*phi*(0:n-1);
%     rho = (1:n).^phi;
%     [x,y] = pol2cart(theta(:),rho(:));
%     xy = 10*([x y]-min([x;y]))/(max([x;y])-min([x;y]));
%     userConfig = struct('xy',xy);
%     resultStruct = tspo_ga_turbo(userConfig);
%
% Example:
%     % Pass a random set of 3D (XYZ) points to the solver
%     xyz = 10*rand(50,3);
%     userConfig = struct('xy',xyz);
%     resultStruct = tspo_ga_turbo(userConfig);
%
% Example:
%     % Change the defaults for GA population size and number of iterations
%     userConfig = struct('popSize',200,'numIter',1e4);
%     resultStruct = tspo_ga_turbo(userConfig);
%
% Example:
%     % Turn off the plots but show a waitbar
%     userConfig = struct('showProg',false,'showResult',false,'showWaitbar',true);
%     resultStruct = tspo_ga_turbo(userConfig);
%
% See also: tsp_ga, tsp_nn, tspof_ga, tspofs_ga
%
% Author: Joseph Kirk
% Email: jdkirk630@gmail.com
%
function varargout = tspo_ga_turbo(varargin)
    
    
    %
    % Initialize default configuration
    %
    defaultConfig.xy          = 10*rand(200,2);
    defaultConfig.dmat        = [];
    defaultConfig.popSize     = 100;
    defaultConfig.numIter     = 1e4;
    defaultConfig.showProg    = true;
    defaultConfig.showStatus  = true;
    defaultConfig.showResult  = true;
    defaultConfig.showWaitbar = false;
    
    
    %
    % Interpret user configuration inputs
    %
    if ~nargin
        userConfig = struct();
    elseif isstruct(varargin{1})
        userConfig = varargin{1};
    else
        try
            userConfig = struct(varargin{:});
        catch
            error('??? Expected inputs are either a structure or parameter/value pairs');
        end
    end
    
    
    %
    % Override default configuration with user inputs
    %
    configStruct = get_config(defaultConfig,userConfig);
    
    
    %
    % Extract configuration
    %
    xy          = configStruct.xy;
    dmat        = configStruct.dmat;
    popSize     = configStruct.popSize;
    numIter     = configStruct.numIter;
    showProg    = configStruct.showProg;
    showStatus  = configStruct.showStatus;
    showResult  = configStruct.showResult;
    showWaitbar = configStruct.showWaitbar;
    if isempty(dmat)
        nPoints = size(xy,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);
    end
    
    
    %
    % Verify inputs
    %
    [N,dims] = size(xy);
    [nr,nc] = size(dmat);
    if (N ~= nr) || (N ~= nc)
        error('??? Invalid XY or DMAT inputs')
    end
    n = N;
    
    
    %
    % Sanity checks
    %
    popSize     = 1+3*ceil((popSize-1)/3);
    numIter     = max(1,round(real(numIter(1))));
    showProg    = logical(showProg(1));
    showStatus  = logical(showStatus(1));
    showResult  = logical(showResult(1));
    showWaitbar = logical(showWaitbar(1));
    
    
    %
    % Initialize the population
    %
    pop = zeros(popSize,n);
    pop(1,:) = (1:n);
    for k = 2:popSize
        pop(k,:) = randperm(n);
    end
    
    
    %
    % Seed the algorithm with a previous result if available
    %
    if isfield(userConfig,'optRoute')
        optRoute = userConfig.optRoute;
        isValid = isequal(pop(1,:),sort(optRoute));
        if isValid
            pop(1,:) = optRoute;
        end
    end
    
    
    %
    % Run the GA
    %
    globalMin = Inf;
    localMin = Inf;
    distHistory = NaN(1,numIter);
    fullHistory = zeros(popSize,numIter);
    tmpPop = zeros(3,n);
    newPop = zeros(popSize,n);
    [isClosed,isStopped,isCancelled] = deal(false);
    if showProg
        hFig = figure('Name','TSPO_GA_TURBO | Current Best Solution', ...
            'Numbertitle','off','CloseRequestFcn',@close_request);
        hAx = gca;
        if showStatus
            [hStatus,isCancelled] = figstatus(0,numIter,[],hFig);
        end
    end
    if showWaitbar
        hWait = waitbar(0,'Searching for near-optimal solution ...', ...
            'CreateCancelBtn',@cancel_search);
    end
    nSame = 0;
    isRunning = true;
    for iter = 1:numIter
        
        %
        % EVALUATE SOLUTIONS
        %   This section of code computes the total cost of each solution
        %   in the population. The actual code that gets executed uses a
        %   much faster (vectorized) method to calculate the route lengths
        %   compared to the double for-loop below (provided for reference)
        %   but gives the same result.
        %
        %     totalDist = zeros(popSize,1);
        %     for p = 1:popSize
        %         d = 0;
        %         for k = 2:n
        %             d = d + dmat(pop(p,k-1),pop(p,k));
        %         end
        %         totalDist(p) = d;
        %     end
        %
        row = pop(:,1:n-1);
        col = pop(:,2:n);
        ind = N*(col-1) + row;
        totalDist = sum(dmat(ind),2);
        
        
        %
        % SELECT THE BEST
        %   This section of code finds the best solution in the current
        %   population and stores it if it is better than the previous best.
        %
        [minDist,index] = min(totalDist);
        fullHistory(:,iter) = sort(totalDist);
        if (minDist < globalMin)
            globalMin = minDist;
            optRoute = pop(index,:);
            if showProg
                
                %
                % Plot the best route
                %
                if (dims > 2), plot3(hAx,xy(optRoute,1),xy(optRoute,2),xy(optRoute,3),'r.-');
                else, plot(hAx,xy(optRoute,1),xy(optRoute,2),'r.-'); end
                title(hAx,sprintf('Total Distance = %1.4f, Iteration = %d',minDist,iter));
                drawnow;
            end
        end
        distHistory(iter) = globalMin;
        
        
        %
        % Update the status bar and check cancellation status
        %
        if showProg && showStatus && ~mod(iter,ceil(numIter/100))
            [hStatus,isCancelled] = figstatus(iter,numIter,hStatus,hFig);
        end
        if (isStopped || isCancelled)
            break
        end
        
        
        %
        % Determine whether to restart a stalled search
        %
        if (minDist < localMin)
            localMin = minDist;
            nSame = 0;
        else
            nSame = nSame + 1;
        end
        if (nSame >= 500)
            for p = 1:popSize
                pop(p,:) = randperm(n);
            end
            localMin = Inf;
        else
            
            %
            % Genetic Algorithm operators
            %
            bestRoute = pop(index,:);
            for p = 3:3:popSize
                routeInsertionPoints = sort(randperm(n,2));
                I = routeInsertionPoints(1);
                J = routeInsertionPoints(2);
                for k = 1:3 % Mutate the Best to get Three New Routes
                    tmpPop(k,:) = bestRoute;
                    switch k
                        case 1 % Flip
                            tmpPop(k,I:J) = tmpPop(k,J:-1:I);
                        case 2 % Swap
                            tmpPop(k,[I J]) = tmpPop(k,[J I]);
                        case 3 % Slide
                            tmpPop(k,I:J) = tmpPop(k,[I+1:J I]);
                        otherwise % Do Nothing
                    end
                end
                newPop(p-2:p,:) = tmpPop;
            end
            pop = newPop;
            pop(popSize,:) = bestRoute;
        end
        
        
        %
        % Update the waitbar
        %
        if showWaitbar && ~mod(iter,ceil(numIter/325))
            waitbar(iter/numIter,hWait);
        end
        
    end
    if showProg && showStatus
        figstatus(numIter,numIter,hStatus,hFig);
    end
    if showWaitbar
        delete(hWait);
    end
    isRunning = false;
    if isClosed
        delete(hFig);
    end
    
    
    %
    % Append prior distance history if present
    %
    if isfield(userConfig,'distHistory')
        priorHistory = userConfig.distHistory;
        isNan = isnan(priorHistory);
        distHistory = [priorHistory(~isNan) distHistory];
    end
    
    
    %
    % Show the final results
    %
    if showResult
        
        %
        % Plot the GA results
        %
        figure('Name','TSPO_GA_TURBO | Results','Numbertitle','off');
        subplot(2,2,1);
        pclr = ~get(0,'DefaultAxesColor');
        if (dims > 2), plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
        else, plot(xy(:,1),xy(:,2),'.','Color',pclr); end
        title('City Locations');
        subplot(2,2,2);
        imagesc(dmat(optRoute,optRoute));
        title('Distance Matrix');
        subplot(2,2,3);
        if (dims > 2), plot3(xy(optRoute,1),xy(optRoute,2),xy(optRoute,3),'r.-');
        else, plot(xy(optRoute,1),xy(optRoute,2),'r.-'); end
        title(sprintf('Total Distance = %1.4f',globalMin));
        subplot(2,2,4);
        plot(distHistory,'b','LineWidth',2);
        title('Best Solution History');
        set(gca,'XLim',[1 length(distHistory)],'YLim',[0 1.1*max([1 distHistory])]);
    end
    
    
    %
    % Return output
    %
    if nargout
        
        %
        % Create anonymous functions for plot generation
        %
        plotPoints  = @(s)plot(s.xy(:,1),s.xy(:,2),'.','Color',~get(gca,'Color'));
        plotResult  = @(s)plot(s.xy(s.optSolution,1),s.xy(s.optSolution,2),'r.-');
        plotHistory = @(s)plot(s.distHistory,'b-','LineWidth',2);
        plotMatrix  = @(s)imagesc(s.dmat(s.optSolution,s.optSolution));
        
        
        %
        % Save results in output structure
        %
        resultStruct = struct( ...
            'xy',          xy, ...
            'dmat',        dmat, ...
            'popSize',     popSize, ...
            'numIter',     numIter, ...
            'showProg',    showProg, ...
            'showResult',  showResult, ...
            'showWaitbar', showWaitbar, ...
            'optRoute',    optRoute, ...
            'optSolution', optRoute, ...
            'plotPoints',  plotPoints, ...
            'plotResult',  plotResult, ...
            'plotHistory', plotHistory, ...
            'plotMatrix',  plotMatrix, ...
            'distHistory', distHistory, ...
            'minDist',     globalMin);
        
        varargout = {resultStruct};
        
    end
    
    
    %
    % Nested function to cancel search
    %
    function cancel_search(varargin)
        isStopped = true;
    end
    
    
    %
    % Nested function to close the figure window
    %
    function close_request(varargin)
        if isRunning
            [isClosed,isStopped] = deal(true);
            isRunning = false;
        else
            delete(hFig);
        end
    end
    
end

