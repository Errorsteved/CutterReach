function [T,frames] = fk(b,q)
%FK Frame 0 to frame 7, and all seven intermediate transforms.
q = reshape(q,1,6);
m = cr.mdh(b);
m([1 5],3) = q([1 5]);
m([2 3 4 6],4) = q([2 3 4 6]);
T = eye(4);
frames = zeros(4,4,7);
for i = 1:7
    a = m(i,1); ca = cos(m(i,2)); sa = sin(m(i,2));
    d = m(i,3); ct = cos(m(i,4)); st = sin(m(i,4));
    A = [ct -st 0 a; ca*st ca*ct -sa -sa*d; ...
         sa*st sa*ct ca ca*d; 0 0 0 1];
    T = T*A;
    frames(:,:,i) = T;
end
end
