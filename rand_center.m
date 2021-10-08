function index = rand_center(source,goal,map)
	dir =  atan2(goal(1)-source(1), goal(2)-source(2));

	len1 = sqrt(sum((goal-source) .^2));
	failedcount = 1000;
	linedata =[source+ (len1 ./ 4) .* [sin(dir) cos(dir)]];
	soal = linedata;
	len1  = (len1 .* 1 ) ./ 2;
	r =0;
	while r < len1,
		buff = soal + r .* [sin(dir) cos(dir)];
		linedata =  [linedata ; buff];
		buff =[];
		r = r+1;
	end
	linedata(1,:)=[];
	lineopen= double(int32(linedata));
	len1 = size(lineopen);
	soal = lineopen(double(int32(rand .* len1(1,1))),:);
	buff = 0;
	while  ~in_ablePoint(soal,map)
		if buff >failedcount,
			error('cannot find center');
			break;
		end
		soal = lineopen(double(int32(rand .* len1(1,1))),:);
		buff =buff + 1;
	end
	index = soal;
	
	