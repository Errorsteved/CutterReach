function d = segmentDistance(a,b,c,f)
%SEGMENTDISTANCE Euclidean distance of closed segments, one pair per row.
% A constrained quadratic attains its minimum either on an edge of [0,1]^2
% or at the stationary point of the two infinite non-parallel lines.
u = b-a; v = f-c; w = a-c;
A = sum(u.*u,2); B = sum(u.*v,2); C = sum(v.*v,2);
D = sum(u.*w,2); E = sum(v.*w,2);
clip = @(x) min(1,max(0,x));
d2 = min(sum((w-clip(E./max(C,realmin)).*v).^2,2), ...
         sum((w+u-clip((E+B)./max(C,realmin)).*v).^2,2));
d2 = min(d2,sum((w+clip(-D./max(A,realmin)).*u).^2,2));
d2 = min(d2,sum((w-v+clip((B-D)./max(A,realmin)).*u).^2,2));
den = A.*C-B.^2;
interior = den > 16*eps.*A.*C;
safe = den; safe(~interior) = 1;
s = (B.*E-C.*D)./safe; t = (A.*E-B.*D)./safe;
inside = interior & s>=0 & s<=1 & t>=0 & t<=1;
lineDist = sum((w+s.*u-t.*v).^2,2);
d2(inside) = min(d2(inside),lineDist(inside));
d = sqrt(d2);
end
