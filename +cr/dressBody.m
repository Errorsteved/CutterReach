function dressBody(body,i,b,e)
%DRESSBODY Local engineering solids attached to the seven paper MDH frames.
% Sections are dimensioned from the course collision envelopes, in metres.
navy=[.16 .27 .32]; gold=[.93 .59 .17]; light=[.75 .81 .82];
steel=[.38 .46 .49]; dark=[.17 .20 .21]; edge=[.69 .43 .24];
r=e.linkRadius; outer=.78*b.limits(5,1);
inner=min(b.limits(5,1),b.limits(5,2)-outer+.03);
switch i
    case 1 % Suspended slide shoe and pendant column, extending along X1.
        h=r(1); L=b.baseOffset;
        box([.045 1.35*h 1.15*h],[.012 0 0],steel);
        box([.077 .98*h .92*h],[.038 0 0],navy);
        box([.90*L .85*h 1.12*h],[.50*L 0 0],navy);
        for sign=[-1 1]
            box([.80*L .18*h 1.24*h],[.51*L sign*.46*h 0],light);
        end
        cyl(.72*h,.76*h,[L 0 0],navy);
    case 2 % Turntable, bearing race and short offset to the shoulder.
        h=r(2); L=b.shoulderOffset;
        cyl(.82*h,.62*h,[0 0 0],dark);
        cyl(.77*h,.20*h,[0 0 .36*h],light);
        box([L .92*h .70*h],[L/2 0 0],gold);
    case 3 % Tapered welded boom between the shoulder and elbow pins.
        h=r(3); L=b.arm;
        profile=[-.42*h -.35*h; .09*L -.72*h; .70*L -.47*h; ...
            .92*L -.62*h; L+.40*h -.27*h; L+.40*h .27*h; ...
            .92*L .62*h; .66*L .42*h; .13*L .71*h; -.42*h .35*h];
        for sign=[-1 1]
            plate(profile,.17*h,[0 0 sign*.42*h],gold);
            box([.48*L .12*h .10*h],[.41*L .34*h sign*.55*h],light);
        end
        box([.85*L .68*h .75*h],[.51*L 0 0],navy);
        pin(0,h); pin(L,h);
    case 4 % Hollow external telescope and a coaxial sliding actuator alongside.
        h=r(4); L=outer;
        % Off-centre actuator leaves the bore open throughout the stroke.
        for sign=[-1 1]
            box([.105*h L .98*h],[(-.14+sign*.46)*h L/2 0],gold);
            box([.92*h L .11*h],[-.14*h L/2 sign*.44*h],gold);
        end
        for y=[.055 .91]*L
            for sign=[-1 1]
                box([.14*h .055*L 1.12*h],[(-.14+sign*.5)*h y 0],navy);
                box([.91*h .055*L .14*h],[-.14*h y sign*.49*h],navy);
            end
        end
        alongY=axang2tform([1 0 0 -pi/2]);
        barrelX=.69*h;
        cylinderOn(.20*h,.82*L,[barrelX .43*L 0],alongY,steel);
        cylinderOn(.23*h,.04*L,[barrelX .84*L 0],alongY,navy);
        for y=[.12 .70]*L
            box([.43*h .035*L .23*h],[.47*h y 0],navy);
        end
    case 5 % Inner telescope moves with frame 5; its local -Z points inward.
        h=r(4); L=inner;
        box([.74*h .73*h L],[.14*h 0 -L/2],light);
        box([.76*h .12*h L],[.14*h .30*h -L/2],steel);
        travel=b.limits(5,2)-.84*outer+.02;
        cylinderOn(.113*h,travel,[-.69*h 0 -travel/2],eye(4),light);
        box([1.42*h .55*h .50*h],[-.18*h 0 -.28*h],navy);
    case 6 % Wrist clevis and two separate jaws grip either end of the cutter axle.
        h=r(5); L=b.tool;
        cyl(.55*h,.88*h,[0 0 0],navy);
        cyl(.32*h,1.16*h,[0 0 0],light);
        for side=[-1 1]
            cyl(.19*h,.08*h,[0 0 side*.60*h],dark);
        end
        box([.38*L .82*h .82*h],[.22*L 0 0],navy);
        box([.12*L 1.03*h 1.18*h],[.43*L 0 0],gold);
        profile=[.43*L -.48*h; .56*L -.45*h; .76*L -.20*h; ...
            L+.16*h -.20*h; L+.22*h 0; L+.16*h .20*h; ...
            .74*L .19*h; .55*L .43*h; .43*L .48*h];
        for sign=[-1 1]
            plate(profile,.13*h,[0 0 sign*.48*h],gold);
            box([.12*L .79*h .08*h],[.46*L 0 sign*.58*h],navy);
            for off=[-.23 .23]*h
                addVisual(body,'RegularExtrusion',[6 .10*h .09*h], ...
                    trvec2tform([.46*L off sign*.65*h]),FaceColor=light);
            end
            % Replaceable axle-gripping pad and keeper at each jaw tip.
            box([.155*L .27*h .075*h],[.91*L 0 sign*.56*h],steel);
            cyl(.14*h,.10*h,[.87*L 0 sign*.616*h],light);
            cyl(.20*h,.15*h,[L 0 sign*.52*h],steel);
        end
    case 7 % Stepped disc body, bevelled wear ring, bearing covers and fasteners.
        h=r(5);
        revolve(h*[.20 -.24;.60 -.24;.76 -.12;.76 .12;.60 .24;.20 .24],steel);
        revolve(h*[.69 -.14;.82 -.14;.94 -.045;.94 .045;.82 .14;.69 .14],edge);
        cyl(.23*h,1.04*h,[0 0 0],dark);
        for sign=[-1 1]
            cyl(.48*h,.09*h,[0 0 sign*.265*h],light);
            cyl(.26*h,.11*h,[0 0 sign*.33*h],navy);
            for k=0:5
                xy=.365*h*[cos(k*pi/3) sin(k*pi/3)];
                addVisual(body,'RegularExtrusion',[6 .047*h .035*h], ...
                    trvec2tform([xy sign*.323*h]),FaceColor=dark);
            end
        end
end

    function box(d,p,color)
        addVisual(body,'Box',d,trvec2tform(p),FaceColor=color);
    end
    function cyl(rad,len,p,color)
        cylinderOn(rad,len,p,eye(4),color);
    end
    function cylinderOn(rad,len,p,rot,color)
        addVisual(body,'Cylinder',[rad len],trvec2tform(p)*rot,FaceColor=color);
    end
    function plate(profile,thick,p,color)
        addVisual(body,'GeneralExtrusion',{profile,thick},trvec2tform(p),FaceColor=color);
    end
    function revolve(profile,color)
        addVisual(body,'RevolvedSolid',{profile},eye(4),FaceColor=color);
    end
    function pin(x,h)
        cyl(.64*h,1.17*h,[x 0 0],navy);
        for side=[-1 1]
            cyl(.44*h,.12*h,[x 0 side*.62*h],light);
            addVisual(body,'RegularExtrusion',[6 .21*h .12*h], ...
                trvec2tform([x 0 side*.72*h]),FaceColor=dark);
        end
    end
end
