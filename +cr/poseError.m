function [pos,rot] = poseError(T,G)
%POSEERROR Euclidean translation and SO(3) geodesic angle, metres and radians.
pos = norm(T(1:3,4)-G(1:3,4));
D = G(1:3,1:3).'*T(1:3,1:3);
rot = atan2(norm([D(3,2)-D(2,3);D(1,3)-D(3,1);D(2,1)-D(1,2)])/2, ...
            (trace(D)-1)/2);
end
