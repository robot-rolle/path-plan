
map=imread('model4.bmp');

% map =im2bw(imread('map2.bmp'));
source=[250 1 ];
goal=[255 499]; % goal position in Y, X format
stepsize=15; % size of each step of the RRT
rrtstart_area=30;%2point half with 
disTh=15;
maxFailedAttempts = 10000;
in_path=[];
display=true;

tic;%start count time
if ~in_ablePoint(source,map), 
    error('source lies on an obstacle or outside map'); 
end
if ~in_ablePoint(goal,map), 
    error('goal lies on an obstacle or outside map');
 end
if display,
     imshow(map);
     rectangle('position',[1 1 size(map)-1],'edgecolor','k'); 
    end


global_length = 0;
local_length = 0;
RRTree=double([source -1 global_length global_length]); % RRT rooted at the source, representation node and parent index 【point xy  index length_local 
failedAttempts=0;
counter=0;
pathFound=false;




while failedAttempts<=maxFailedAttempts , % loop to grow RRTs
    if rand < 0.5, 
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goalÍ
    end

    buffer_distance = distanceCost(RRTree(:,1:2),sample);

%    [A1, I]=min(distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
   
    I(1) = check_loacalvalue(buffer_distance,RRTree,rrtstart_area);
    if I(1)==0
        I(1)=1;
    end
    

    closestNode = RRTree(I(1),1:2);

 
    theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
   
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
   
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end

    
    if distanceCost(newPoint,goal)<disTh, 
        pathFound=true;
        break; 
    end % goal reached
    
    
    [A2, I2]=min( distanceCost(RRTree(:,1:2),newPoint),[],1); % check if new node is not already pre-existing in the tree
                                            % a . is the distance of the cloest , i is the number
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh, 
        failedAttempts=failedAttempts+1;
        continue;
     end 
    
    local_length = A2;

    global_length=RRTree(I(1),4)+local_length;
    
    RRTree=[RRTree;newPoint I(1) global_length local_length ]; % add node

    failedAttempts=0;
    
    if display, 
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);

        counter=counter+1;
        M(counter)=getframe;
    end
    
end

if display && pathFound 
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;
    M(counter)=getframe;
end
if ~pathFound,
     error('once again，guy');
end

seb=figure;
figure(seb);
imshow(map);
path=[goal,I(1)+1];
prev=I(1);

while prev>0
    path=[RRTree(prev,1:3);path]; % to add the point to the path
    prev=RRTree(prev,3);   %can not sure the data is the shorest of the path
end

pathLength=0;
l1 = size(path);
 for i=1:l1(1,1)-1, 
    pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); 
end
fprintf('compute time = %d\n',toc)
fprintf(' Length1=%d \n', pathLength); 
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(in_path(:,2),in_path(:,1));
line(path(:,2),path(:,1));


sec=figure;
figure(sec);
imshow(map);
in_path=check_globalvalue(path,map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(in_path(:,2),in_path(:,1));
l2 = size(in_path);
fprintf(' Length2=%d \n',in_path(l2(1,1),3));
% buffer_path = size(in_path);
% t_number = buffer_path(1);
% while t_number >0
%     line([source(2),in_path(t_number,2)],[source(1),in_path(t_number,1)]);
%     t_number=t_number-1;
% end
length3=0;
% sed = figure;
% figure(sed);
% imshow(map);
out_path=check_globavalueII(in_path,20,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(out_path(:,2),out_path(:,1));
l3= size(out_path);
for index = 1:l3(1,1)-1,
    length3 = distanceCost(out_path(index,1:2),out_path(index+1,1:2))+length3;
end
fprintf(' Length3=%d \n',length3);

length4=0;
% sef = figure;
% figure(sef);
% imshow(map);
outest_path=check_globavalueII(out_path,20,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(outest_path(:,2),outest_path(:,1));
l4=size(outest_path);
for index = 1:l4(1,1)-1,
    length4 = distanceCost(outest_path(index,1:2),outest_path(index+1,1:2))+length4;
end
fprintf(' Length4=%d \n',length4);


length5=0;
% seg = figure;
% figure(seg);
% imshow(map);
o_path=check_globavalueII(outest_path,20,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(o_path(:,2),o_path(:,1));
l5=size(o_path);
for index = 1:l5(1,1)-1,
    length5 = distanceCost(o_path(index,1:2),o_path(index+1,1:2))+length5;
end
fprintf(' Length5=%d \n',length5);

length6=0;
% sel = figure;
% figure(sel);
% imshow(map);
oo_path=check_globalvalue(o_path,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(oo_path(:,2),oo_path(:,1));
l6=size(oo_path);
for index = 1:l6(1,1)-1,
    length6 = distanceCost(oo_path(index,1:2),oo_path(index+1,1:2))+length6;
end
fprintf(' Length6=%d \n',length6);


Length6=0;
% sel = figure;
% figure(sel);
% imshow(map);
oo_path=check_globalvalue(oo_path,map);
% rectangle('position',[1 1 size(map)-1],'edgecolor','k');
% line(oo_path(:,2),oo_path(:,1));
l6=size(oo_path);
for index = 1:l6(1,1)-1,
    length6 = distanceCost(oo_path(index,1:2),oo_path(index+1,1:2))+length6;
end
fprintf(' Length8=%d \n',length6);

length7=0;
seluoer=figure;
figure(seluoer);
imshow(map);
luoer_path = check_globavalueII(oo_path,20,map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(luoer_path(:,2),luoer_path(:,1));
l7=size(luoer_path);

for index = 1:l7(1,1)-1,
    length7 = distanceCost(luoer_path(index,1:2),luoer_path(index+1,1:2))+length7;
end
fprintf(' Length7=%d \n,deline=%d\n',length7,length7/pathLength);

