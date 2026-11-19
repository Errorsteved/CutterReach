function [free,margin,reason] = collision(P,e)
%COLLISION Capsule envelope against chamber/cabin, inclined struts and self.
% margin is a conservative signed clearance after the required gap (metres).
n = size(P,1); wall = inf(n,1); brace = wall; self = wall;
[starts,stops,radii] = cr.braceSegments(e);
for j = 1:5
    a = P(:,:,j); b = P(:,:,j+1); r = e.linkRadius(j)+e.clearance;
    wall = min([wall,min(a(:,3),b(:,3))-e.rearZ-r, ...
        e.headZ-max(a(:,3),b(:,3))-r, ...
        e.radius-r-max(vecnorm(a(:,1:2)-e.center,2,2),vecnorm(b(:,1:2)-e.center,2,2))],[],2);
    % Only the segment below gateZ+r can overlap the cabin's walls.
    low = a; high = b;
    swap = a(:,3)>b(:,3); low(swap,:) = b(swap,:); high(swap,:) = a(swap,:);
    dz = high(:,3)-low(:,3);
    frac = min(1,max(0,(e.gateZ+r-low(:,3))./max(eps,dz)));
    tip = low(:,1:2)+frac.*(high(:,1:2)-low(:,1:2));
    xy = max(abs(low(:,1:2)-e.cabinXY),abs(tip-e.cabinXY));
    cabin = min(e.cabinSize(1:2)/2-xy,[],2)-r;
    cabin(low(:,3)>e.gateZ+r) = inf;
    wall = min(wall,cabin);
    for k = 1:size(starts,1)
        gap = cr.segmentDistance(a,b,starts(k,:),stops(k,:))-r-radii(k);
        brace = min(brace,gap);
    end
    for k = j+2:5
        gap = cr.segmentDistance(a,b,P(:,:,k),P(:,:,k+1))-r-e.linkRadius(k);
        self = min(self,gap);
    end
end
[margin,reason] = min([wall brace self],[],2);
free = margin>0;
reason(free) = 0;  % 1 boundary, 2 support, 3 non-adjacent self contact
end
