% input:first point  center point  and the last point , map , scan size;
% output: new point 1 2;

function [new_point1,new_point2]=line_check(front_point,center_point,back_point,map,step_size)
	front_step = 0;
	back_step =0;
	step_size_count =1; 			%count the number of the  step

	new_pointL =center_point;
	new_pointR =center_point;

	test_pointL =center_point;
	test_pointR =center_point;

	line_front= distanceCost(front_point,center_point);
	line_back= distanceCost(center_point,back_point);

	short_len =min(line_front,line_back);
	long_len = max(line_front,line_back);

	k =long_len/short_len;

	short_step = short_len/step_size;
	long_step = short_step.*k;

	thetaI =atan2(front_point(1)-center_point(1),front_point(2)-center_point(2));
	thetaII = atan2(back_point(1)-center_point(1),back_point(2)-center_point(2));
	
	
	if line_front == long_len,
			front_step =  long_step;
			back_step = short_step;
		else if  line_front == short_len,
			front_step = short_step;
			back_step = long_step;	
		end
	end

	while step_size_count < step_size ,
		test_pointL = double(int32(center_point(1:2)+(front_step .* step_size_count) * [sin(thetaI)  cos(thetaI)]));
		test_pointR = double(int32(center_point(1:2)+(back_step .* step_size_count) * [sin(thetaII) cos(thetaII)]));		
		if checkPath(test_pointL,test_pointR,map),
			new_pointL = test_pointL;
			new_pointR = test_pointR;
		end
		step_size_count = step_size_count+1;
	end
	new_point1 = new_pointL;
	new_point2 = new_pointR;
	
