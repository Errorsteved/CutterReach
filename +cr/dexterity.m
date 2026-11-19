function eta = dexterity(b,Q,L)
%DEXTERITY sigma4/sigma1 of normalized spatial Jacobian, on attainable twists.
% In the rotating radial frame the yaw column is orthogonal to all others.
% The other five columns have three active rows [radial/L; axial/L; pitch].
n = size(Q,1);
if n==0, eta = zeros(0,1); return; end
t = Q(:,3); h = t+Q(:,4); s = h+Q(:,6); d = Q(:,5);
rad = b.shoulderOffset+b.arm*cos(t)-d.*sin(h)-b.tool*cos(s);
v4r = -d.*cos(h)+b.tool*sin(s);
v4z = d.*sin(h)+b.tool*cos(s);
r = [zeros(n,1), -b.arm*sin(t)+v4r, v4r, -L*sin(h), b.tool*sin(s)]/L;
z = [L*ones(n,1), -b.arm*cos(t)+v4z, v4z, -L*cos(h), b.tool*cos(s)]/L;
pitch = [0 1 1 0 1];
G = zeros(3,3,n);
G(1,1,:) = sum(r.^2,2); G(2,2,:) = sum(z.^2,2); G(3,3,:) = 3;
G(1,2,:) = sum(r.*z,2); G(1,3,:) = r*pitch.'; G(2,3,:) = z*pitch.';
G(2,1,:) = G(1,2,:); G(3,1,:) = G(1,3,:); G(3,2,:) = G(2,3,:);
eigvals = reshape(pagesvd(G,'econ','vector'),3,n).';
yawSquared = 1+(rad/L).^2;
eta = sqrt(min(eigvals(:,3),yawSquared)./max(eigvals(:,1),yawSquared));
end
