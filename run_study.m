function study = run_study(c)
%RUN_STUDY Reproduce paper checks, assess the baseline, then search and validate.
% In the project directory: study = run_study; demo_study
if nargin==0, c = cr.config; end
assert(~isempty(which('rigidBodyTree')),'Install Robotics System Toolbox before running the study.');
b = c.baseline;
assert(cr.structure(b,c),'Baseline fails the structural constraints in this scenario.');
stream = RandStream('mt19937ar','Seed',c.seed);
U = rand(stream,c.nWorkspace,6);
w0 = cr.workspace(b,c.env,U,c.Lref);
v = cr.validateBaseline(b,c,w0);
disp(v.checkpoints); disp(v.extents);
selection = cr.indexedTargets(c,c.cutters,c.indexAngles);
g = cr.indexedTargets(c,c.cutters,c.fineAngles);
assert(all(ismember(selection.headAngle,g.headAngle)),'Refined stops must include all selection stops.');
gCheck = cr.indexedTargets(c,c.checkCutters,c.checkAngles);
coarse0 = cr.summarizeCutters(cr.evaluate(b,selection,c,c.ikGrid));
[opt,grid] = cr.searchDesign(c,selection,coarse0);
coarse1 = cr.summarizeCutters(cr.evaluate(opt,selection,c,c.ikGrid));
r0 = cr.evaluate(b,g,c,c.fineGrid); r1 = cr.evaluate(opt,g,c,c.fineGrid);
h0 = cr.evaluate(b,gCheck,c,c.fineGrid); h1 = cr.evaluate(opt,gCheck,c,c.fineGrid);
w1 = cr.workspace(opt,c.env,U,c.Lref);
summary = table;
sets = ["Selection";"Refined";"Holdout"];
left = {coarse0,cr.summarizeCutters(r0),cr.summarizeCutters(h0)};
right = {coarse1,cr.summarizeCutters(r1),cr.summarizeCutters(h1)};
for j = 1:3
    summary = [summary;cr.summaryRow(sets(j),"Baseline",b,left{j},right{j}); ...
        cr.summaryRow(sets(j),"Optimized",opt,right{j},left{j})]; %#ok<AGROW>
end
study = struct('config',c,'baseline',b,'optimized',opt,'validation',v,'grid',grid, ...
    'goals',g,'holdoutGoals',gCheck,'baselineTargets',r0,'optimizedTargets',r1, ...
    'holdoutBaseline',h0,'holdoutOptimized',h1,'baselineWorkspace',w0, ...
    'optimizedWorkspace',w1,'baselineCutters',left{2},'optimizedCutters',right{2}, ...
    'holdoutBaselineCutters',left{3},'holdoutOptimizedCutters',right{3}, ...
    'summary',summary,'release',version,'toolbox',ver('robotics'));
cr.verifyWitnesses(b,r0,g,c); cr.verifyWitnesses(opt,r1,g,c);
cr.verifyWitnesses(b,h0,gCheck,c); cr.verifyWitnesses(opt,h1,gCheck,c);
if ~isfolder(c.out), mkdir(c.out); end
save(fullfile(c.out,'study.mat'),'study');
baselineRobot = cr.buildRobot(b,c.env); optimizedRobot = cr.buildRobot(opt,c.env);
save(fullfile(c.out,'models.mat'),'baselineRobot','optimizedRobot');
writetable(v.checkpoints,fullfile(c.out,'paper_checkpoints.csv'));
writetable(v.table2,fullfile(c.out,'paper_table2_replay.csv'));
writetable(v.crosscheck,fullfile(c.out,'implementation_checks.csv'));
writetable(v.axialExamples,fullfile(c.out,'paper_axial_examples.csv'));
writetable(v.extents,fullfile(c.out,'paper_workspace_extents.csv'));
writetable(grid,fullfile(c.out,'design_grid.csv'));
writetable(summary,fullfile(c.out,'performance.csv'));
writePair(left{2},right{2},fullfile(c.out,'cutters.csv'));
writePair(left{3},right{3},fullfile(c.out,'holdout_cutters.csv'));
writePair(r0,r1,fullfile(c.out,'targets.csv'));
writePair(h0,h1,fullfile(c.out,'holdout_targets.csv'));
cr.exportFigures(study);
cr.writeFindings(study);
disp(summary); fprintf('Results written to %s\n',c.out);
end

function writePair(b,o,file)
writetable([addvars(b,repmat("Baseline",height(b),1),'Before',1,'NewVariableNames','Design'); ...
            addvars(o,repmat("Optimized",height(o),1),'Before',1,'NewVariableNames','Design')],file);
end
