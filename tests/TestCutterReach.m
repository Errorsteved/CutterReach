classdef TestCutterReach < matlab.unittest.TestCase
    methods (TestClassSetup)
        function projectOnPath(test)
            root = fileparts(fileparts(mfilename('fullpath')));
            test.applyFixture(matlab.unittest.fixtures.PathFixture(root));
        end
    end

    methods (Test)
        function tableOneModel(test)
            c=cr.config; b=c.baseline; tree=cr.buildRobot(b);
            test.verifyEqual(tree.NumBodies,7);
            types=cellfun(@(x)string(x.Joint.Type),tree.Bodies);
            test.verifyEqual(types,["prismatic" "revolute" "revolute" "revolute" "prismatic" "revolute" "fixed"]);
            test.verifyEqual(b.limits(5,:),[1.42 2.07]);
            q=[1.0677 deg2rad([176.6 -81 -66.9]) 1.924 deg2rad(-103.5)];
            T=cr.fk(b,q);
            test.verifyLessThan(norm(T(1:3,4)-[-0.820;0.082;3.857]),0.002);
            test.verifyEqual(T,getTransform(tree,q,'frame7'),'AbsTol',1e-12);
            q=[.807 pi -pi/2 -pi/2 1.310 -pi/2]; T=cr.fk(b,q);
            test.verifyEqual(T(1:3,4),[.46;0;3.309],'AbsTol',1e-12);
            test.verifyFalse(cr.inLimits(b,q));
        end

        function assembledPartsFitCollisionEnvelopes(test)
            c=cr.config;
            % Extreme stroke, joint-limit and mid-travel examples for both designs.
            for arm=[c.baseline.arm max(c.armGrid)]
                b=c.baseline; b.arm=arm;
                if arm~=c.baseline.arm, b.limits(5,2)=2.295; end
                meshes=cr.bodyMeshes(cr.buildRobot(b,c.env));
                Q=[b.limits(:,1).'; b.limits(:,2).'; mean(b.limits,2).'];
                owner=[1 2 3 4 4 5 5];
                for j=1:size(Q,1)
                    [~,F]=cr.fk(b,Q(j,:)); points=reshape(cr.geometry(b,Q(j,:)),3,6).';
                    for k=1:7
                        T=F(:,:,k); verts=meshes{k}.V*T(1:3,1:3).'+T(1:3,4).';
                        id=owner(k); a=points(id,:); axis=points(id+1,:)-a;
                        f=min(1,max(0,(verts-a)*axis.'/dot(axis,axis)));
                        distance=vecnorm(verts-a-f.*axis,2,2);
                        test.verifyLessThanOrEqual(max(distance),c.env.linkRadius(id)+1e-7, ...
                            sprintf('Frame %d protrudes from its checked capsule.',k));
                    end
                end
            end
        end

        function bracketPedestalContact(test)
            c=cr.config; e=c.env;
            e.strutA=[0.2 0.2 2]; e.strutB=[1.3 0.2 2.3];
            p=[.2*ones(6,1),(-1.1:.45:1.15).',3.25*ones(6,1)];
            P=reshape(p.',1,3,6);
            test.verifyTrue(cr.collision(P,rmfield(e,'postRadius')));
            [free,~,reason]=cr.collision(P,e);
            test.verifyFalse(free); test.verifyEqual(reason,2);
        end

        function fixedBaseDemoKeepsBothArmsVisible(test)
            c=cr.config; b=c.baseline; opt=b; opt.arm=.90; opt.limits(5,2)=2.295;
            catalogue=c.cutters([4 8],:); catalogue.HomeAzimuth_deg(:)=72;
            goals=cr.indexedTargets(c,catalogue,[0 90]);
            t0=cr.evaluate(b,goals,c,c.fineGrid); t1=cr.evaluate(opt,goals,c,c.fineGrid);
            s=struct('config',c,'baseline',b,'optimized',opt,'goals',goals, ...
                'baselineTargets',t0,'optimizedTargets',t1, ...
                'baselineCutters',cr.summarizeCutters(t0),'optimizedCutters',cr.summarizeCutters(t1));
            f=demo_study(s); test.addTeardown(@()close(f));
            ax=findobj(f,'Tag','BaseView'); base=findobj(ax,'Tag','BaseOrigin');
            design=findobj(f,'Tag','Design'); station=findobj(f,'Tag','Station');
            mode=findobj(f,'Tag','DetailMode');
            camera=@()[ax.CameraTarget ax.CameraPosition ax.CameraUpVector ax.CameraViewAngle ...
                       ax.XLim ax.YLim ax.ZLim ax.DataAspectRatio ax.Position];
            initial=camera();
            % Same target, two designs. Failed IK must show the checked stowed arm.
            station.Value=2; design.Value=1; feval(design.Callback,design,[]);
            [~,~,park]=cr.structure(b,c); [~,F]=cr.fk(b,park);
            for k=1:7
                part=findobj(ax,'Tag',sprintf('frame%d',k),'Visible','on');
                test.verifyNumElements(part,1); test.verifyEqual(part.Matrix,F(:,:,k),'AbsTol',1e-12);
            end
            test.verifySubstring(findobj(f,'Tag','PoseState').String,'stowed pose');
            test.verifyFalse(contains(findobj(f,'Tag','StationMetrics').String,'TCP error'));
            test.verifyEqual(camera(),initial,'AbsTol',1e-10);
            test.verifyEqual([base.XData base.YData base.ZData],[0 0 0]);
            design.Value=2; feval(design.Callback,design,[]);
            [~,F]=cr.fk(opt,s.optimizedCutters.q(2,:)); tip=findobj(ax,'Tag','frame7','Visible','on');
            test.verifyEqual(tip.Matrix,F(:,:,7),'AbsTol',1e-12);
            station.Value=1; feval(station.Callback,station,[]);
            for k=1:4
                mode.Value=k; feval(mode.Callback,mode,[]);
                test.verifyEqual(camera(),initial,'AbsTol',1e-10);
                test.verifyNumElements(findobj(ax,'Type','hgtransform','Visible','on'),7);
            end
            stop=findobj(f,'Tag','HeadStop'); stop.Value=3; mode.Value=3; feval(stop.Callback,stop,[]);
            head=findobj(f,'Tag','Cutterhead');
            test.verifyEqual(head.Matrix,cr.headTransform(c.env,90),'AbsTol',1e-12);
            target=findobj(ax,'Tag','SelectedTarget'); xyz=[target.XData;target.YData;target.ZData];
            test.verifyEqual(xyz,goals.pose(1:3,4,3),'AbsTol',1e-12);
            test.verifyEqual(camera(),initial,'AbsTol',1e-10);
        end

        function cutterheadMovesRigidlyInsideFixedChamber(test)
            c=cr.config; e=c.env; catalog=c.cutters([2 2],:);
            catalog.CutterID=[37;12]; catalog.HomeAzimuth_deg=[90;135];
            g=cr.indexedTargets(c,catalog,[360 90 0]);
            test.verifyEqual(g.cutterID,[37;12;37;12]);
            test.verifyEqual(g.azimuth,[90;135;180;225]);
            H=cr.headTransform(e,90);
            test.verifyEqual(g.pose(:,:,3),H*g.pose(:,:,1),'AbsTol',1e-12);
            spun=cr.indexedEnvironment(e,90);
            test.verifyEqual(spun.cabinXY,e.cabinXY);
            test.verifyEqual(spun.gateZ,e.gateZ);
            [a,b]=cr.braceSegments(e); [aa,bb]=cr.braceSegments(spun);
            test.verifyEqual(aa,[e.center(1)-(a(:,2)-e.center(2)) e.center(2)+a(:,1)-e.center(1) a(:,3)],'AbsTol',1e-12);
            test.verifyEqual(bb,b*H(1:3,1:3).'+H(1:3,4).','AbsTol',1e-12);
            test.verifyEqual(g.pose(1:3,4,3)-aa(1,:).',H(1:3,1:3)*(g.pose(1:3,4,1)-a(1,:).'),'AbsTol',1e-12);
            % The same world pose clears a stationary bracket but hits the
            % actual bracket that turns along with this cutter.
            g=cr.indexedTargets(c,c.cutters(4,:),342);
            fixed=cr.solvePose(c.baseline,g.pose,c,c.ikGrid);
            actual=cr.evaluate(c.baseline,g,c,c.ikGrid);
            test.verifyGreaterThan(fixed.nFree,0);
            test.verifyGreaterThan(actual.nPose,0); test.verifyGreaterThan(actual.nBrace,0);
            test.verifyEqual(actual.nFree,0);
        end

        function oneVotePerCutterAndIndexedWitness(test)
            c=cr.config; b=c.baseline; b.limits(2,:)=[0 pi/4];
            cat=c.cutters([1 1 10],:); cat.CutterID=[17;3;20]; cat.HomeAzimuth_deg=[90;180;90];
            g=cr.indexedTargets(c,cat,[0 180 270 360]);
            t=cr.evaluate(b,g,c,c.ikGrid); cuts=cr.summarizeCutters(t);
            test.verifyEqual(cuts.CutterID,cat.CutterID); test.verifyEqual(cuts.StopsTested,[3;3;3]);
            test.verifyEqual(t.nFree(t.Head_deg==0),[0;0;0]);
            test.verifyEqual(cuts.FeasibleStops,[1;1;0]); test.verifyEqual(cuts.BestHead_deg,[270;180;NaN]);
            test.verifyTrue(isnan(cuts.WitnessTarget(3)) && all(isnan(cuts.q(3,:))));
            for k=1:2
                j=cuts.WitnessTarget(k);
                test.verifyEqual(cuts.q(k,:),t.q(j,:));
                test.verifyEqual(cr.fk(b,cuts.q(k,:)),g.pose(:,:,j),'AbsTol',1e-10);
            end
            cr.verifyWitnesses(b,t,g,c);
        end

        function finiteDifferenceJacobian(test)
            c=cr.config; b=c.baseline;
            q=[1.9 1.1 -1.2 -0.8 1.73 -1.5]; T=cr.fk(b,q);
            J=cr.jacobian(b,q); h=1e-6;
            for k=1:6
                dq=zeros(1,6); dq(k)=h;
                delta=(cr.fk(b,q+dq)-cr.fk(b,q-dq))/(2*h);
                W=delta(1:3,1:3)*T(1:3,1:3)';
                test.verifyEqual(J(:,k),[W(3,2);W(1,3);W(2,1);delta(1:3,4)],'AbsTol',1e-8);
            end
            sv=svd(J); test.verifyLessThan(sv(5),1e-12);
            test.verifyGreaterThan(sv(4),0.1);
            for L=[0.7 1 1.4]
                sv=svd(diag([1 1 1 1/L 1/L 1/L])*J*diag([L 1 1 1 L 1]));
                test.verifyEqual(cr.dexterity(b,q,L),sv(4)/sv(1),'AbsTol',1e-12);
            end
        end

        function distancesAndDegeneracies(test)
            a=zeros(4,3); b=[2 0 0;1 0 0;2 0 0;0 0 0];
            p=[1 -1 0;2 1 0;1 0 1;0 1 0]; q=[1 1 0;3 1 0;1 2 1;0 1 0];
            test.verifyEqual(cr.segmentDistance(a,b,p,q),[0;sqrt(2);1;1],'AbsTol',1e-14);
            test.verifyEqual(cr.segmentDistance(q,p,b,a),[0;sqrt(2);1;1],'AbsTol',1e-14);
        end

        function obstacleBetweenEndpoints(test)
            % Segment interiors intersect even though all endpoints are clear.
            c=cr.config; e=c.env; e.strutA=[1.5 -1 2.5]; e.strutB=[1.5 1 2.5];
            pts=[(0:5)'*0.6,zeros(6,1),2.5*ones(6,1)];
            P=reshape(pts.',1,3,6);
            [free,gap,id]=cr.collision(P,e);
            test.verifyFalse(free); test.verifyLessThan(gap,0); test.verifyEqual(id,2);
            e.strutA=e.strutA+[0 0 0.5]; e.strutB=e.strutB+[0 0 0.5];
            test.verifyTrue(cr.collision(P,e));
            e.headZ=2.55; [free,~,id]=cr.collision(P,e);
            test.verifyFalse(free); test.verifyEqual(id,1);
        end

        function selfContactAndCabinGate(test)
            c=cr.config; e=c.env; e.strutA=zeros(0,3); e.strutB=zeros(0,3);
            pts=[0 0;.6 0;.6 .5;.25 .5;.25 -.25;.5 -.25];
            P=reshape([pts 2.8*ones(6,1)].',1,3,6);
            [ok,~,id]=cr.collision(P,e); test.verifyFalse(ok); test.verifyEqual(id,3);
            % Below the gate, a straight link outside the storage aperture is blocked.
            pts=[(0:5)'*.3,zeros(6,1),ones(6,1)];
            [ok,~,id]=cr.collision(reshape(pts.',1,3,6),e);
            test.verifyFalse(ok); test.verifyEqual(id,1);
            [ok,~,q]=cr.structure(c.baseline,c); test.verifyTrue(ok);
            test.verifyTrue(cr.inLimits(c.baseline,q));
        end

        function inversePoseAndWrongOrientation(test)
            c=cr.config; b=c.baseline;
            g=cr.targets(c,1.55,72); G=g.pose;
            a=cr.solvePose(b,G,c,c.ikGrid);
            test.verifyEqual(a.status,"reached"); test.verifyTrue(cr.inLimits(b,a.q));
            [p,o]=cr.poseError(cr.fk(b,a.q),G);
            test.verifyLessThan(p,c.positionTol); test.verifyLessThan(o,c.rotationTol);
            test.verifyTrue(cr.collision(cr.geometry(b,a.q),c.env));
            % Same TCP point, incompatible roll about tool X, exceeding 2 deg.
            U=G; U(1:3,1:3)=G(1:3,1:3)*axang2rotm([1 0 0 pi/6]);
            a=cr.solvePose(b,U,c,c.ikGrid); test.verifyEqual(a.nFree,0);
            G(1,4)=G(1,4)+10; a=cr.solvePose(b,G,c,c.ikGrid);
            test.verifyEqual(a.nFree,0);
        end

        function adjustableLimitsAndRepeatability(test)
            c=cr.config; g=cr.targets(c,1.55,72);
            b=c.baseline; b.limits(2,:)=[0 pi/4];
            a=cr.solvePose(b,g.pose,c,c.ikGrid); test.verifyEqual(a.nPose,0);
            r=cr.evaluate(c.baseline,g,c,c.ikGrid);
            test.verifyEqual(r,cr.evaluate(c.baseline,g,c,c.ikGrid));
            b=c.baseline; b.limits(5,2)=2.38;
            [ok,why]=cr.structure(b,c); test.verifyFalse(ok); test.verifyEqual(why,"extended length");
            % Nested refinement retains each coarse feasible witness.
            fine=cr.evaluate(c.baseline,g,c,c.fineGrid);
            test.verifyGreaterThanOrEqual(fine.eta+1e-12,r.eta);
        end
    end
end
