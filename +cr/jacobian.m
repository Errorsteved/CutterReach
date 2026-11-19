function J = jacobian(b,q)
%JACOBIAN Spatial geometric Jacobian [angular; linear] in base coordinates.
[T,F] = cr.fk(b,q);
J = zeros(6,6);
for i = 1:6
    axis = F(1:3,3,i);
    if i==1 || i==5
        J(4:6,i) = axis;
    else
        J(:,i) = [axis;cross(axis,T(1:3,4)-F(1:3,4,i))];
    end
end
end
