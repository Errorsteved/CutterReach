function [best,grid] = searchDesign(c,goals,baseline)
%SEARCHDESIGN Exhaustive, deterministic grid; count first, mean eta second.
b0 = c.baseline; sw0 = diff(b0.limits(5,:));
[L,S] = ndgrid(unique([b0.arm c.armGrid]),unique(round([sw0 c.strokeGrid],12)));
n = numel(L);
grid = table(L(:),S(:),strings(n,1),nan(n,1),nan(n,1),nan(n,1), ...
    'VariableNames',{'Arm_m','Stroke_m','Structure','Reached','Rate','MeanEtaAll'});
for k = 1:n
    b = b0; b.arm = L(k); b.limits(5,2) = b.limits(5,1)+S(k);
    [ok,grid.Structure(k)] = cr.structure(b,c);
    if ~ok, continue; end
    if abs(L(k)-b0.arm)<1e-12 && abs(S(k)-sw0)<1e-12
        t = baseline;
    else
        t = cr.summarizeCutters(cr.evaluate(b,goals,c,c.ikGrid));
    end
    grid.Reached(k) = nnz(t.nFree); grid.Rate(k) = grid.Reached(k)/height(t);
    grid.MeanEtaAll(k) = sum(t.eta,'omitnan')/height(t);
end
admitted = find(grid.Structure=="admitted");
assert(~isempty(admitted),'No design passes the structural constraints.');
% Last tie break: smallest sum of movable lengths.
criteria = [-grid.Reached(admitted),-grid.MeanEtaAll(admitted),L(admitted)+S(admitted)];
[~,order] = sortrows(criteria,[1 2 3]); i = admitted(order(1));
grid.Selected = false(n,1); grid.Selected(i) = true;
best = b0; best.arm = L(i); best.limits(5,2) = best.limits(5,1)+S(i);
end
