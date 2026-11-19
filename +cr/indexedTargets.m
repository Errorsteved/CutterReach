function g = indexedTargets(c,cutters,angles)
%INDEXEDTARGETS Retain cutter identity while applying each candidate head rotation.
assert(height(cutters)>0 && numel(unique(cutters.CutterID))==height(cutters) ...
    && all(isfinite(cutters.CutterID) & cutters.CutterID==fix(cutters.CutterID) & cutters.CutterID>0), ...
    'Each cutter requires a unique positive integer ID.');
assert(all(isfinite(cutters.Radius_m) & cutters.Radius_m>=0 & isfinite(cutters.HomeAzimuth_deg)), ...
    'Cutter radii and mounting angles must be finite, with nonnegative radii.');
angles = unique(mod(angles(:),360),'sorted');
assert(~isempty(angles) && all(isfinite(angles)),'Supply at least one finite stop angle.');
n = height(cutters);
home = zeros(4,4,n); tilt = zeros(n,1);
for k = 1:n
    goal = cr.targets(c,cutters.Radius_m(k),cutters.HomeAzimuth_deg(k));
    home(:,:,k) = goal.pose; tilt(k) = goal.tilt;
end
[i,stop] = ndgrid(1:n,angles); i=i(:); stop=stop(:);
poses = zeros(4,4,numel(i));
for k = 1:numel(angles)
    rows = (k-1)*n+(1:n);
    poses(:,:,rows) = pagemtimes(cr.headTransform(c.env,angles(k)),home);
end
g = struct('catalogue',cutters,'cutterID',cutters.CutterID(i),'radius',cutters.Radius_m(i), ...
    'homeAzimuth',cutters.HomeAzimuth_deg(i),'headAngle',stop, ...
    'azimuth',mod(cutters.HomeAzimuth_deg(i)+stop,360),'tilt',tilt(i),'pose',poses);
end
