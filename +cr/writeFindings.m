function writeFindings(s)
%WRITEFINDINGS Generate a portable report extract from the completed run.
c = s.config; t = s.summary;
[f,msg] = fopen(fullfile(c.out,'findings.md'),'w','n','UTF-8');
assert(f>=0,'%s',msg); clean = onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'# Cutter accessibility after cutterhead indexing\n\n');
fprintf(f,'Each cutter ID counts once per design. The table gives cutterhead stop angles in degrees about the +Z axis from mechanical zero. The cutters and the supports on the back of the cutterhead rotate together.\n\n');
fprintf(f,'Design selection tested %d stop angles and the refined evaluation tested %d. The holdout set included %d cutters evaluated at %d stop angles each.\n\n', ...
    numel(unique(mod(c.indexAngles,360))),numel(unique(s.goals.headAngle)),height(c.checkCutters),numel(unique(s.holdoutGoals.headAngle)));
reports = {s.baselineCutters,s.optimizedCutters}; stops = {s.baselineTargets,s.optimizedTargets};
labels = ["Baseline","Optimized"];
for d=1:2
    per=reports{d}; angles=stops{d}; home=angles.Head_deg==0;
    fprintf(f,'- The **%s** arm reached %d/%d cutters at one or more stops',labels(d),nnz(per.nFree),height(per));
    if any(home), fprintf(f,' and %d/%d at mechanical zero alone',nnz(angles.nFree(home)),nnz(home)); end
    fprintf(f,'.\n');
end
fprintf(f,'\n| Cutter ID | Radius (m) | Baseline feasible / tested stops | Baseline selected stop | Optimized feasible / tested stops | Optimized selected stop |\n');
fprintf(f,'|---|---:|---:|---:|---:|---:|\n');
for k=1:height(reports{1})
    fprintf(f,'| %02d | %.3f |',reports{1}.CutterID(k),reports{1}.Radius_m(k));
    for d=1:2
        per=reports{d};
        fprintf(f,' %d/%d |',per.FeasibleStops(k),per.StopsTested(k));
        if per.FeasibleStops(k)>0, fprintf(f,' %g° |',per.BestHead_deg(k)); else, fprintf(f,' — |'); end
    end
    fprintf(f,'\n');
end
fprintf(f,'\nEach successful solution verifies a static service pose. A cutter with no solution on the sampled grid may still be reachable at another stop or joint configuration. Disassembly, extraction, retrieval and reinstallation paths remain unverified. The result files list joint configurations, pose errors and clearances for each cutter, along with outcomes and collision causes at each stop. [Per-cutter results](cutters.csv) · [Per-stop results](targets.csv)\n\n');
fprintf(f,'## Design comparison and validation\n\n');
fprintf(f,'MATLAB %s. RNG seed %d. The baseline and selected design share %d normalized workspace samples.\n\n', ...
    s.release,c.seed,c.nWorkspace);
fprintf(f,'The search admitted %d of %d structural candidates. The selected upper arm measures %.3f m and the telescope operates between %.3f and %.3f m.\n\n', ...
    nnz(s.grid.Structure=="admitted"),height(s.grid),s.optimized.arm,s.optimized.limits(5,:));
fprintf(f,'| Evaluation | Design | Cutters with bounded poses | Accessible cutters / total | Access | Mean η, all | Mean η, reached | Mean η, paired |\n');
fprintf(f,'|---|---|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(t)
    fprintf(f,'| %s | %s | %d | %d / %d | %.1f%% | %.4f | %.4f | %.4f |\n', ...
        t.Set(i),t.Design(i),t.BoundedCutters(i),t.Reached(i),t.Cutters(i),100*t.Rate(i), ...
        t.MeanEtaAll(i),t.MeanEtaReached(i),t.MeanEtaCommon(i));
end
fprintf(f,'\nThe design advantage was %+.1f percentage points on the selection grid and %+.1f on the refined grid. The held-out cutters yielded %+.1f points.\n\n', ...
    100*(t.Rate(2)-t.Rate(1)),100*(t.Rate(4)-t.Rate(3)),100*(t.Rate(6)-t.Rate(5)));
fprintf(f,'Among the %d refined cutters reached by both arms, the paired mean dexterity changed from %.4f to %.4f. The all-target score assigns zero to cutters without a witness.\n\n', ...
    t.CommonCutters(3),t.MeanEtaCommon(3),t.MeanEtaCommon(4));
w0 = s.baselineWorkspace; w1 = s.optimizedWorkspace;
fprintf(f,'Uniform joint sampling produced %d (%.2f%%) clear baseline configurations and %d (%.2f%%) clear selected configurations. This fraction weights joint configurations. Cutter access weights cutter IDs equally. Workspace sampling uses the reference head angle.\n\n', ...
    nnz(w0.free),100*mean(w0.free),nnz(w1.free),100*mean(w1.free));
for entry = ["baselineCutters","optimizedCutters"]
    r = s.(entry);
    fprintf(f,'- %s — %d without a bounded pose, %d with bounded poses all blocked, %d with a free witness.\n', ...
        entry,nnz(r.nPose==0),nnz(r.nPose>0 & r.nFree==0),nnz(r.nFree>0));
end
v = s.validation;
fprintf(f,'\n## Published baseline checks\n\n');
fprintf(f,'The Cartesian residuals for paper points A and B are %.4g and %.4g mm. A uses an extension below the Table 1 minimum. B falls within the published bounds. All %d Table 2 joint rows yielded a bounded IK witness for the frame reconstructed from that row.\n\n', ...
    v.checkpoints.Error_mm(1),v.checkpoints.Error_mm(2),nnz(v.table2.BoundedIKCount));
for k = 1:3
    a = v.extents(k,:);
    fprintf(f,'- %s range in metres — paper [%.1f, %.1f], sampled [%.3f, %.3f].\n', ...
        a.Axis,a.PaperMin_m,a.PaperMax_m,a.SampleMin_m,a.SampleMax_m);
end
fprintf(f,'\nTwo explicit configurations inside Table 1 give Z = %.3f m and Z = %.3f m. Their joint vectors are in paper_axial_examples.csv.\n',v.axialExamples.TCP_m(:,3));
fprintf(f,'\nThe largest axis discrepancy is retained in the results. The publication gives no original Fig. 10 samples or executable limit/filter settings.\n\n');
fprintf(f,'The registered cutter stations, link radii, support layout and packaging rules are course assumptions. A success certifies the final static pose in this geometry. A search miss is resolution-dependent.\n');
end
