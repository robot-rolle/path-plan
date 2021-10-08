function     [RRTree_goal,static_param_data,donytic_param_data]=g_rrtb(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data,map),
	% creat param talbe for input data
	%    static_param_data=   stepsize              disTh                     maxFailedAttempts      rrstar_area
	%                        source(1,1)          source(1,2)               point_connect(1,1)      point_connect(1,2)
	%                        center_point(1.1)    center_point(1,2)          prev_ahead             prev-graft-ahead
	%                        goal(1,2)            goal(1,2)                  prev_behing	         prev-graft_behind  
	% 			 point_connectmain(1,1)   point_connectmain(1,2)    	 point_connect2(1,1)    point_connect2(1,2)
	%    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found];
	    faildcount =0;	
	    buffer_local = 0;
	    buffer_global = 0;
    
	while faildcount <= static_param_data(1,3),
		if  rand < 0.5,
			sample = rand(1,2) .* size(map);
			elseif rand < 0.75
				sample = [static_param_data(3,1) static_param_data(3,2)];
			 else 
				sample = [static_param_data(2,1) static_param_data(2,2)];
		end
    
	    [A, I]=min( distanceCost(RRTree_goal(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    
	    closestNode = RRTree_goal(I(1), 1:2);
    
	    theta = atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));
    
	    newPoint = double(int32(closestNode(1:2) + static_param_data(1,1) * [sin(theta)  cos(theta)]));
    
	    if ~checkPath(closestNode(1:2),newPoint,map),
		    faildcount = faildcount+1;
		    continue
	    end
	    
	    [Disatance_Graft,Index_graft] = min(in_distance(GraftTree(:,1:2),newPoint),[],1);
    
	    if distanceCost(GraftTree(Index_graft(1),1:2),newPoint)<static_param_data(1,2) && checkPath(GraftTree(Index_graft(1),1:2),newPoint,map) 
		    donytic_param_data(1,2) = false; 	
		    donytic_param_data(1,4) = true;
		    
		    static_param_data(5,3) = newPoint(1,1);
		    static_param_data(5,4) = newPoint(1,2);
		    
		    static_param_data(4,3) = I(1);
		    static_param_data(4,4) = Index_graft(1);
		    break;
	    end
	    	% check the start tree with the goal tree state;
	     [Disatance_goal,Index_goal] = min(in_distance(RRTree_main(:,1:2),newPoint),[],1);%此处是否优化 看现象 占用内存过高

	    if distanceCost(RRTree_main(Index_goal(1),1:2),newPoint)<static_param_data(1,2) && checkPath(RRTree_main(Index_goal(1),1:2),newPoint,map) 
		donytic_param_data(1,2) = false; 
		donytic_param_data(1,3) = true;
		donytic_param_data(1,4) = true;
		donytic_param_data(1,5) = true;

		static_param_data(5,1) = newPoint(1,1);
		static_param_data(5,2) = newPoint(1,2);
		
		static_param_data(4,3) = I(1);
		static_param_data(3,3) = Index_goal(1);
		break;
            end

    
	    [Distance_goal,DIndex_goal] =  min( in_distance(RRTree_goal(:,1:2),newPoint),[], 1);
    
	    if  in_distance(newPoint, RRTree_goal(DIndex_goal(1),1:2)) < static_param_data(1,2),
		    faildcount = faildcount+1;
		    continue;
	    end

	    RRTree_goal = [RRTree_goal;newPoint I(1)];
	    
	    donytic_param_data(1,2) = false; 
	    
	    break;
    
    
       end
    