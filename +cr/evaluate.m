function res = evaluate(b,goals,c,grid)
%EVALUATE Count a target once if any sampled configuration passes all gates.
n = numel(goals.radius);
for i = n:-1:1
    local = c;
    if isfield(goals,'headAngle'), local.env = cr.indexedEnvironment(c.env,goals.headAngle(i)); end
    sol(i,1) = cr.solvePose(b,goals.pose(:,:,i),local,grid);
end
res = struct2table(sol);
res = addvars(res,(1:n)',goals.radius,goals.azimuth,goals.tilt, ...
    'Before',1,'NewVariableNames',{'Target','Radius_m','Azimuth_deg','Tilt_deg'});
if isfield(goals,'headAngle')
    res = addvars(res,goals.cutterID,goals.homeAzimuth,goals.headAngle,'After','Target', ...
        'NewVariableNames',{'CutterID','HomeAzimuth_deg','Head_deg'});
end
end
