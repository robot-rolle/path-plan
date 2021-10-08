%l主函数测路径
% map=im2bw(imread('../RRT_START/map1.bmp'));


map = imread('../RRT_START/model4.bmp');

    source=[251 1]; 

    goal = [251 499];

countt = 10;
timeer  =0;
f=countt;
while  countt>0,

    stepsize=20; 

    disTh=20; 

    maxFailedAttempts = 10000;
    
    rrstar_area = stepsize .* 2;
    display=true; 

    flag_goal = false;

    center_point = rand_center(source,goal,map);
    if ~in_ablePoint(source,map), 
        error('false '); 
    end
    if ~in_ablePoint(goal,map), 
        error('false ');
    end

    static_param_data =[stepsize disTh maxFailedAttempts rrstar_area; source 0 0 ; center_point 0 0 ; goal 0 0];
    % creat param talbe for input data
    %  static_param_data =   stepsize              disTh                     maxFailedAttempts      rrstar_area
    %                        source(1,1)          source(1,2)                point_connect(1,1)      point_connect(1,2)
    %                        center_point(1.1)    center_point(1,2)          prev_ahead             prev_behind
    %                        goal(1,2)            goal(1,2)                  prev_behind_ahead      prev_behing_behind        
 	% 	            		 point_connect2(1,1)   point_connect2(1,2)    	
   
    % check point
    %imshow(map);
    % if display,
    %         imshow(map);
    %         rectangle('position',[1 1 size(map)-1],'edgecolor','k'); 
    % end
 
    global_length = 0;
    local_length = 0;
    RRTree_main=double([source -1 global_length local_length ]);
    RRTree_goal=double([goal -1 global_length local_length]);
    GraftTree=double([center_point  -1]);

    Graft_ahead=false;
    Graft_behind=false;
    ahead_found = false;
    behind_found = false;
    errtree_found = false;
    
    donytic_param_data =[Graft_ahead,Graft_behind,ahead_found,behind_found,errtree_found];
    
    counter= 0;

tic;
    while  ~donytic_param_data(3) || ~donytic_param_data(4) ,

        if ~donytic_param_data(1) && ~donytic_param_data(3),
            [RRTree_main,static_param_data,donytic_param_data]=g_rstara(RRTree_main,GraftTree,...
            RRTree_goal,static_param_data,donytic_param_data,map);
            % if ~donytic_param_data(1) && ~donytic_param_data(3) && display,
            %     line([RRTree_main(end,2);RRTree_main(RRTree_main(end,3),2)],[RRTree_main(end,1);...
            %     RRTree_main(RRTree_main(end,3),1)],'color','black');
            %     counter = counter+1;
            %     M(counter) = getframe;
            % end
        end
        if  ~donytic_param_data(2) && ~donytic_param_data(4),
            [RRTree_goal,static_param_data,donytic_param_data]=g_rstarb(RRTree_main,...
            GraftTree,RRTree_goal,static_param_data,donytic_param_data,map);
            % if ~donytic_param_data(2) && ~donytic_param_data(4) && display,
            %     line([RRTree_goal(end,2);RRTree_goal(RRTree_goal(end,3),2)],[RRTree_goal(end,1);...
            %     RRTree_goal(RRTree_goal(end,3),1)],'color','black');
            %     counter = counter+1;
            %     M(counter) = getframe;
            % end 
        end
        if ~donytic_param_data(3) || ~donytic_param_data(4),
            [GraftTree,static_param_data,donytic_param_data]=graft_extend(RRTree_main,GraftTree,...
            RRTree_goal,static_param_data,donytic_param_data,map);
            % if ~donytic_param_data(3) || ~donytic_param_data(4) && display,
            %     line([GraftTree(end,2);GraftTree(GraftTree(end,3),2)],[GraftTree(end,1);GraftTree(GraftTree(end,3),1)],'color','r');
            %     counter = counter +1;
            %     M(counter) = getframe ;
            % end
        end
        
        if donytic_param_data(3) && donytic_param_data(4) ,
           in_path=  re_path(RRTree_main,GraftTree,RRTree_goal,static_param_data,donytic_param_data);
            break;
        end

     end
     timecount = toc;
timeer = timeer + timecount;
fprintf('compute time = %d\n',timecount);
countt = countt -1;
end  

fprintf('compute alltime = %d\n',timeer ./ f);

% seb=figure;
% figure(seb);
% imshow(map);
    
% imshow(map);
%  rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(in_path(:,2),in_path(:,1));
% pathLength=0;
% l1 = size(in_path);
%  for i=1:l1(1,1)-1, 
%     pathLength=pathLength+distanceCost(in_path(i,1:2),in_path(i+1,1:2)); 
% end
% timeer=toc;
% fprintf('compute time = %d\n',timeer);
% pathLength;
% fprintf(' Length1=%d \n', pathLength); 

% sec=figure;
% figure(sec);
% imshow(map);
% in_path=check_globalvalue(in_path,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(in_path(:,2),in_path(:,1));
% l2 = size(in_path);
% fprintf(' Length2=%d \n',in_path(l2(1,1),3));
% length3=0;
% % sed = figure;
% % figure(sed);
% % imshow(map);
% out_path=check_globavalueII(in_path,20,map);
% % rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% % line(out_path(:,2),out_path(:,1));
% l3= size(out_path);
% for index = 1:l3(1,1)-1,
%     length3 = distanceCost(out_path(index,1:2),out_path(index+1,1:2))+length3;
% end
% fprintf(' Length3=%d \n',length3);
% sec=figure;
% figure(sec);
% imshow(map);
% in_path=check_globalvalue(out_path,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(in_path(:,2),in_path(:,1));
% l2 = size(in_path);
% fprintf(' Length2=%d \n',in_path(l2(1,1),3));
