function cutters = summarizeCutters(stops)
%SUMMARIZECUTTERS Count each physical ID once and retain an actual feasible stop.
[ids,first] = unique(stops.CutterID,'stable');
cutters = stops(first,{'CutterID','Radius_m','HomeAzimuth_deg'});
n = numel(ids);
cutters.StopsTested = zeros(n,1); cutters.FeasibleStops = zeros(n,1);
cutters.nPose = zeros(n,1); cutters.nFree = zeros(n,1);
cutters.WitnessTarget = nan(n,1); cutters.BestHead_deg = nan(n,1);
cutters.BestAzimuth_deg = nan(n,1); cutters.q = nan(n,6);
for name = ["eta","gap","posErr","rotErr"], cutters.(name) = nan(n,1); end
cutters.status = strings(n,1);
for k = 1:n
    t = stops(stops.CutterID==ids(k),:);
    cutters.StopsTested(k) = height(t); cutters.FeasibleStops(k) = nnz(t.nFree);
    cutters.nPose(k) = sum(t.nPose); cutters.nFree(k) = sum(t.nFree);
    free = t(t.nFree>0,:);
    if isempty(free)
        cutters.status(k) = "no bounded pose sampled";
        if cutters.nPose(k)>0, cutters.status(k) = "all sampled poses blocked"; end
        continue
    end
    % Within numerical eta ties, prefer the shortest rotation from zero.
    ties = free(free.eta >= max(free.eta)-1e-12,:);
    [~,j] = sortrows([min(ties.Head_deg,360-ties.Head_deg) ties.Head_deg]);
    best = ties(j(1),:);
    cutters.WitnessTarget(k) = best.Target; cutters.BestHead_deg(k) = best.Head_deg;
    cutters.BestAzimuth_deg(k) = best.Azimuth_deg; cutters.q(k,:) = best.q;
    for name = ["eta","gap","posErr","rotErr"], cutters.(name)(k) = best.(name); end
    cutters.status(k) = "reached";
end
end
