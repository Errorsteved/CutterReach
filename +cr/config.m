function c = config()
%CONFIG Reproducible course experiment. Distances in metres, angles in radians.
c.root = fileparts(fileparts(mfilename('fullpath')));
c.out = fullfile(c.root,'results');
c.seed = 30386;
c.nWorkspace = 30000;
b.baseOffset = 0.560;
b.shoulderOffset = 0.100;
b.arm = 0.732;
b.tool = 0.460;
b.limits = [0 3; 0 2*pi; deg2rad([-125 75]); deg2rad([-150 65]); ...
            1.420 2.070; deg2rad([-180 0])];
c.baseline = b;
c.armGrid = [0.650 0.732 0.820 0.900];
c.strokeGrid = [0.500 0.650 0.800 0.875 0.950];
c.maxStrokeRatio = 0.65;   % course telescoping/overlap proxy
c.maxExtended = 2.32;      % installation allowance
c.positionTol = 0.004;
c.rotationTol = deg2rad(2);
c.Lref = 1;                % fixed task length and prismatic input scale (m)
c.ikGrid = [41 17];        % upper-arm angle and absolute telescope length
c.fineGrid = 2*c.ikGrid-1; % nested refinement for sensitivity check
e.center = [0.560 0];      % projection of TBM axis in the base XY plane
e.radius = 8.630/2;
e.headZ = 4.00;            % rear surface of the cutterhead
e.depths = [1.425 0.995];  % air chamber then slurry chamber
e.gateZ = e.headZ-sum(e.depths);
e.cabinSize = [1.180 1.150 3.090]; % X,Y,Z in base coordinates
e.cabinXY = [0.330 0];
e.rearZ = e.gateZ-e.cabinSize(3);
e.linkRadius = [0.030 0.038 0.055 0.080 0.095];
e.clearance = 0.008;
% Four inclined struts behind the plate, parametrized by two polar endpoints.
a = deg2rad((0:3)'*90+2);
e.strutA = [e.center+1.25*[cos(a) sin(a)], repmat(e.headZ-0.80,4,1)];
e.strutB = [e.center+3.40*[cos(a+0.07) sin(a+0.07)], repmat(e.headZ-0.20,4,1)];
e.strutRadius = 0.105;
e.postRadius = 0.075;      % assumed bounding radius of bolted mounting posts
e.spokeZero_deg = 90;      % reference spoke azimuth at zero cutterhead rotation
c.env = e;
c.cutters = catalogue(0.50:0.35:3.65,e.spokeZero_deg);
c.checkCutters = catalogue(0.675:0.35:3.475,e.spokeZero_deg);
c.indexAngles = 0:15:345;         % mechanical stops relative to the reference pose
c.fineAngles = 0:5:355;           % contains all selection stops
c.checkAngles = 2.5:5:357.5;      % interleaved verification on held-out radii
c.standoff = 0.16;
end

function cutters = catalogue(radii,home)
n = numel(radii);
cutters = table((1:n)',radii(:),repmat(home,n,1), ...
    'VariableNames',{'CutterID','Radius_m','HomeAzimuth_deg'});
end
