function robot = buildRobot(b,e)
%BUILDROBOT Robotics System Toolbox model. The seventh frame is fixed.
if nargin<2, defaults = cr.config; e = defaults.env; end
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',7);
rows = cr.mdh(b);
types = ["prismatic","revolute","revolute","revolute","prismatic","revolute","fixed"];
parent = robot.BaseName;
for i = 1:7
    name = "frame"+i;
    body = rigidBody(char(name));
    joint = rigidBodyJoint(char("joint"+i),char(types(i)));
    setFixedTransform(joint,rows(i,:),'mdh');
    if i <= 6
        lim = b.limits(i,:);
        joint.PositionLimits = [min(lim(1),0), max(lim(2),0)];
        joint.HomePosition = mean(lim);
        joint.PositionLimits = lim;
    end
    body.Joint = joint;
    cr.dressBody(body,i,b,e);
    addBody(robot,body,parent);
    parent = char(name);
end
end
