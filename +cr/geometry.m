function [P,R] = geometry(b,Q)
%GEOMETRY Vectorized MDH expansion. P is N x xyz x six capsule endpoints
% (frames 1,2,3,4,5=6,7). R is 3 x 3 x N and uses frame-7 axes as columns.
n = size(Q,1);
u = [cos(Q(:,2)) sin(Q(:,2))];
t3 = Q(:,3); t34 = t3+Q(:,4); s = t34+Q(:,6);
rad = [zeros(n,1),repmat(b.shoulderOffset,n,1), ...
       b.shoulderOffset+b.arm*cos(t3)];
rad(:,4) = rad(:,3)-Q(:,5).*sin(t34);
rad(:,5) = rad(:,4)-b.tool*cos(s);
z = [Q(:,1),Q(:,1),Q(:,1)-b.arm*sin(t3)];
z(:,4) = z(:,3)-Q(:,5).*cos(t34);
z(:,5) = z(:,4)+b.tool*sin(s);
P = zeros(n,3,6);
P(:,3,1) = Q(:,1);
for j = 1:5
    P(:,:,j+1) = [b.baseOffset+u(:,1).*rad(:,j),u(:,2).*rad(:,j),z(:,j)];
end
if nargout > 1
    R = zeros(3,3,n);
    R(:,1,:) = reshape([-cos(s).*u sin(s)].',3,1,n);
    R(:,2,:) = reshape([sin(s).*u cos(s)].',3,1,n);
    R(:,3,:) = reshape([-u(:,2) u(:,1) zeros(n,1)].',3,1,n);
end
end
