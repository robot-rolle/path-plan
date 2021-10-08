%l主函数测路径
map=imread('../RRT_START/model.bmp'); 

source=[10 10]; 
goal = [499 499];
stepsize=20; 

disTh=20; 

maxFailedAttempts = 10000;

display=true; 

center_point =double(int32(rand(1,2) .* size(map)));

while ~in_ablePoint(center_point,map)  
    center_point =double(int32(rand(1,2) .* size(map))); 
end





flag_goal = false;

tic;
if ~in_ablePoint(source,map), 
    error('source lies on an obstacle or outside map'); 
end
if ~in_ablePoint(center_point,map), 
    error('goal lies on an obstacle or outside map');
 end
if display,
     imshow(map);
     rectangle('position',[1 1 size(map)-1],'edgecolor','k'); 
    end


RRTree1=double([source -1]);

RRTree2=double([center_point -1]);


counter=0;

tree1ExpansionFail=false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail=false; % sets to true if expansion after set number of attempts fails


while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs
    if ~tree1ExpansionFail % found  the  first Path
        [RRTree1,pathFound,tree1ExpansionFail]=rrtExtend(RRTree1,RRTree2,center_point,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','black');
            counter=counter+1;
            M(counter)=getframe;    
            %%plot([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)]);
        end
    end
    if ~tree2ExpansionFail  %found the Second path 
        [RRTree2,pathFound,tree2ExpansionFail]=rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from goal towards source
        if ~tree2ExpansionFail && isempty(pathFound) && display
            line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
            counter=counter+1;M(counter)=getframe;
        end
        if ~isempty(pathFound), pathFound(3:4)=pathFound(4:-1:3); end % path found

    end
    if ~isempty(pathFound) % path found  
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:2)]; % compute path
        prev=pathFound(1,3); % add nodes from RRT 1 first
        while prev>0
            path=[RRTree1(prev,1:2);path];
            prev=RRTree1(prev,3);
        end
        prev=pathFound(1,4); % then add nodes from RRT 2
        while prev>0
            path=[path;RRTree2(prev,1:2)];
            prev=RRTree2(prev,3);
        end
        break;
    end
    
end

seb=figure;
figure(seb);
imshow(map);

if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end
pathLength=0;

for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
imshow(map);
 rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1));

