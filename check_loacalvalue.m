% buffer_distance  :the distance of every point in the tree get to the new point(sample) 
% RRTree :the main tree
% area   : the area of the local point 
% exit_data : return  the index of the tree of the shortest  distance point
function exit_data= check_loacalvalue(buffer_distance,RRTree,area)
	local_father_length=0;
	local_value=[0,0];
    	for i=1:length(buffer_distance),
	     if buffer_distance(i,1) <area ,
               local_father_length = buffer_distance(i,1)+RRTree(i,4);
               local_value=[[i,local_father_length];local_value]; 
	     end
        end
		local_value=sortrows(local_value,2);
                exit_data=local_value(2);
              