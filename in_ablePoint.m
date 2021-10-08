function   state = in_ablePoint(point, map);
state=true;
if (point(1)>=1&&point(1)<=size(map,1) ...
   && point(2)>=1&&point(2)<=size(map,2) ...
   && map(point(1),point(2))==1 )
   state = true;
   else
	state =false;
end
