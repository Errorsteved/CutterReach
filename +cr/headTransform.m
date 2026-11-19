function T = headTransform(e,angle)
%HEADTRANSFORM World-to-world motion of a head-mounted part about the TBM axis.
R = [cosd(angle) -sind(angle) 0; sind(angle) cosd(angle) 0; 0 0 1];
center = [e.center(:);e.headZ];
T = [R center-R*center; 0 0 0 1];
end
