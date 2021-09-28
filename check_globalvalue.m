function in_path = check_globalvalue(path,map)  
	stack_path = [path(1,1:2),0,0];  %存入 缓存树
	tree_length= 0;  
	stack_length=1;
	queue_path =0; %队列 队头
	step_length=0;
	i_path =1;
	while i_path < length(path),
	   for j_path = i_path:length(path),
		  if checkPath(path(i_path,1:2),path(j_path,1:2),map),
			queue_path =j_path;  %放入队列 直接删除前一个 
		  end 
	   end
	  step_length =distanceCost(path(i_path,1:2),path(queue_path,1:2));
	  tree_length =step_length+stack_path(stack_length,3);%一轮循环下来检测栈和队头长度
	  i_path=queue_path; %放入队列 直接替换队列的扫描
	  stack_path=[stack_path;[path(i_path,1:2),tree_length,step_length]];
	  stack_length=stack_length+1;
	end
	in_path =stack_path;