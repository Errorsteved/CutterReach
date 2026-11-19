function goals = targets(c,radii,angles)
%TARGETS Registered spoke at several indexing angles. Independent of arm design.
[r,a] = ndgrid(radii,angles);
r = r(:); a = a(:);
n = numel(r); T = repmat(eye(4),1,1,n);
tilt = 12*max(0,min(1,(r-2.25)/1.25)); % outer holders incline radially
for i = 1:n
    er = [cosd(a(i));sind(a(i));0]; et = [-er(2);er(1);0]; ez = [0;0;1];
    T(1:3,1:3,i) = [sind(tilt(i))*er+cosd(tilt(i))*ez, ...
                     cosd(tilt(i))*er-sind(tilt(i))*ez,et];
    T(1:3,4,i) = [c.env.center.';c.env.headZ-c.standoff]+r(i)*er;
end
goals = struct('pose',T,'radius',r,'azimuth',a,'tilt',tilt);
end
