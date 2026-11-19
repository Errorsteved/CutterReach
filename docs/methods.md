# Model, assumptions and reproduction methods

## 1. Published parameters and baseline checks

The source is Mei Yang et al., *Accessibility and Trajectory Planning of Cutter Changing Robot Arm for Large-Diameter Slurry Shield*, Mechanika 29(3), 214–224 (2023). The kinematic model follows Table 1 and Figure 9. Section 2 describes indexing the cutterhead to service cutters on one spoke. The study replays the joint records from Table 2. [DOI](https://doi.org/10.5755/j02.mech.30386) · [Publisher's PDF](https://mechanika.ktu.lt/index.php/Mech/article/download/30386/15886)

| Row | Joint or function | θᵢ | dᵢ (mm) | aᵢ₋₁ (mm) | αᵢ₋₁ | Joint limits |
|---:|---|---|---|---:|---:|---|
| 1 | Prismatic, base rail | 0 | d₁ | 0 | 0° | 0 to 3000 mm |
| 2 | Revolute, base yaw | θ₂ | 0 | 560 | 0° | 0° to 360° |
| 3 | Revolute, upper-arm pitch | θ₃ | 0 | 100 | −90° | −125° to 75° |
| 4 | Revolute, forearm pitch | θ₄ | 0 | 732 | 0° | −150° to 65° |
| 5 | Prismatic, forearm extension | 180° | d₅ | 0 | −90° | 1420 to 2070 mm |
| 6 | Revolute, wrist pitch | θ₆ | 0 | 0 | −90° | −180° to 0° |
| 7 | Fixed tool center point (TCP) at the cutter's center of mass | 0 | 0 | 460 | 0° | Fixed |

The modified DH transform for each row is $T_x(a_{i-1})R_x(\alpha_{i-1})R_z(\theta_i)T_z(d_i)$. The product gives $T_{07}$, the transform from the base to the cutter's center of mass. Its rotation columns are the local tool axes, and its last column gives the tool position in homogeneous coordinates. Joint 5 stores the **absolute** telescoping length and retains the constant 180° rotation. The toolbox joint vector is $[d_1,\theta_2,\theta_3,\theta_4,d_5,\theta_6]$. The toolbox implementation uses `setFixedTransform(...,'mdh')`. An independent implementation multiplies the transforms explicitly. [Table 1 and Eqs. (9)–(11), p. 218](https://mechanika.ktu.lt/index.php/Mech/article/download/30386/15886#page=5)

### Discrepancies in the source

- The structural design section gives a telescoping range of 1310–1960 mm and a tool dimension of 435 mm. The kinematic table gives 1420–2070 mm and 460 mm. Both sets give a stroke of 650 mm, but their absolute extension endpoints differ by 110 mm. The paper does not fully specify the relationship between the reference points. The model uses Table 1. The structural section's outer-arm length of 1100 mm and inner-arm length of 860 mm are recorded for reference.
- The mechanism sketch in Figure 5 includes R7. The last row of Table 1 is fixed, and the kinematic description lists six joint variables. The model therefore uses six independently actuated joints. The angular limits come from Table 1. The model does not assume that the design angles in Section 3.1 define the MDH zero positions.
- The printed matrix in Eq. (9) gives its (3,2) entry as $\sin\alpha$. Multiplying the rotations gives $\cos\theta\sin\alpha$. Equation (12) also includes a variable in row 7 with no motion range in Table 1. The code multiplies the rigid transforms and checks the results against the toolbox, finite differences and two published Cartesian checkpoints.
- Point A on p. 221 uses a $d_5$ outside the Table 1 limits. The study uses that extension only to reproduce A. Sampling and accessibility evaluation enforce the Table 1 limits. The check for point B allows a 2 mm residual because the published joint angles are rounded to 0.1°. For each of the seven Table 2 records, the study reconstructs the TCP pose and finds a joint-limit-compliant inverse solution. Those seven checks establish internal kinematic consistency. A and B provide the independent Cartesian comparisons.
- The paper does not provide the original Figure 10 samples or scene transforms. Following Eq. (13), the study draws 30,000 independent uniform joint samples and compares the extents along all three axes in the same base frame. The toolbox also verifies two configurations within the Table 1 limits whose axial positions lie outside the published range. The available information does not uniquely explain the discrepancy.

## 2. Chamber geometry and installation assumptions

| Item | Model values and sources |
|---|---|
| Chambers | Page 215 gives a cutterhead diameter of 8.8 m, a working chamber inner diameter of 8.630 m, an air cushion chamber depth of 1.425 m, a slurry chamber depth of 0.995 m and a storage cabin measuring 1.150 × 1.180 × 3.090 m. The course model uses the working chamber inner radius of 4.315 m to bound the serviceable cutterhead face. |
| Installation coordinates | The model assumes that $Z_0$ follows the rails toward the cutterhead. The cutterhead rear face is at 4.000 m, the chamber axis projects to $(0.560,0)$ m and the storage cabin opening is at 1.580 m. The cabin center projects to $(0.330,0)$ m. Clear widths in X and Y are 1.180 and 1.150 m. The twin rails show how the carriage is guided. Collision checks exclude the designed contact between the rails and the mounting base. Potential rail interference with other components is also outside the collision model. |
| Internal geometry | The two working chambers form one traversable cylinder bounded by the cylindrical wall, rear wall, back of the cutterhead and cabin opening. A ring marks the chamber partition in the rendering. Two posts attach each of four inclined I-section supports to the cutterhead. The collision model uses four support capsules of radius 105 mm and eight post capsules of radius 75 mm. Their positions and sections are course assumptions specified in `cr.config`. The eight rendered spokes and backing rings are on the material side of the cutterhead boundary. Collision checks treat that boundary as a closed plane. |
| Arm geometry | Five capsules join the origins of frames 1–2, 2–3, 3–4, 4–5 and 6–7. Frames 5 and 6 share an origin. In order, the capsule radii are 30, 38, 55, 80 and 95 mm. The forearm collision envelope uses the outer sleeve radius along the entire telescoping segment. The rendering shows the narrower inner rod. The tool envelope approximates the gripper and cutter because their actual cross sections are unavailable. |
| Cutter catalog and mechanical zero | Ten distinct cutter IDs have mounting radii of 0.50 to 3.65 m in 0.35 m steps. Without per-cutter CAD data, the model places all ten on one spoke with a zero-position azimuth of 90°, the middle azimuth in the original scenario. At this reference position the inner support ends lie at 2°, 92°, 182° and 272°. Each row of `c.cutters` specifies a cutter ID, mounting radius and mounting azimuth. Multiple IDs can share one radius. The target TCP lies 160 mm from the back of the cutterhead. |
| Cutterhead indexing | The model permits a full revolution, with positive rotation about the world +Z axis. Design selection tests stops every 15° and final verification every 5°. Rotations of 0° and 360° correspond to one stop. The holdout set uses nine other radii from 0.675 to 3.475 m and tests stops at 2.5°, 7.5° and so on through 357.5°. |
| Holder orientation | The axial, radial and tangential unit vectors $e_z$, $e_r$ and $e_t$ define all three target axes. $X_7=\sin\delta\,e_r+\cos\delta\,e_z$, $Y_7=\cos\delta\,e_r-\sin\delta\,e_z$ and $Z_7=e_t$. Tilt is zero through a radius of 2.25 m, increases linearly to 12° at 3.50 m and stays at 12° farther out. |
| Feasibility | Position error must be ≤ 4 mm, full SO(3) orientation error must be ≤ 2°, and joints must lie within their closed limit intervals. Conservative clearance between each capsule and the boundaries, supports and nonadjacent links must be strictly greater than 8 mm. |
| Storage and extension | A fixed folded configuration must satisfy joint limits, avoid collisions and fit entirely inside the storage cabin. The design filters require $d_{5,\max}\le 2.32$ m and a stroke no greater than 0.65 times the retracted length to allow for installation space and sleeve overlap. Loads, stiffness and actual hydraulic cylinder dimensions require separate engineering data. |

### Assembly dimensions and motion

Frame 1 carries the carriage and cantilever. Frame 2 carries the rotating base. Frames 3 through 7 carry, respectively, the upper-arm side plates and pins, the outer forearm sleeve, the sliding inner arm, the wrist jaws and the cutter ring. The four-sided square sleeve leaves an opening for the inner arm. The offset hydraulic barrel and piston rod follow the two telescoping components. The jaws straddle the cutter and grip opposite ends of its axle. Revolving a cross section generates the cutter ring. Its axle follows local $Z_7$, and the ring center coincides with the TCP. The arrangement follows Figure 6. The course collision envelopes determine the detailed external dimensions. [Mechanism drawing](https://mechanika.ktu.lt/index.php/Mech/article/download/30386/15886#page=3)

The default cutter ring has an outside diameter of $2 \times 0.94 \times 95 = 178.6$ mm. The cross-section proportions for the sleeves, jaws, pins and bearing caps are defined in `+cr/dressBody.m`. This ring fits the 190 mm tool envelope. The paper does not specify the corresponding full-scale ring diameter. Collision checks use line-segment capsules. Tests transform the triangle meshes of all rigid bodies to their actual joint configurations and check that every vertex lies inside its assigned convex envelope. Rendering and collision checks use the same support endpoints for all eight mounting posts. The rendered holder marks the spoke mounting surface at the selected index. The target TCP lies in front of it on the service reference plane.

## 3. Per-cutter accessibility after indexing

For each cutter, first compute the installation target $T_i(0)$ at mechanical zero. Let $p_c$ be a point on the cutterhead axis and let $R=R_z(\phi)$. The world-frame rigid motion and indexed target are

$$
H(\phi)=\begin{bmatrix}R & p_c-Rp_c\\0 & 1\end{bmatrix},
\qquad T_i(\phi)=H(\phi)T_i(0).
$$

This transform moves the target position and all three orientation axes, along with the four cutterhead supports and eight mounting posts. The chamber, cabin opening, base and rails keep their installed coordinates. The rendered cutterhead assembly uses the same $H$. At each stop, the solver checks the entire arm against the corresponding environment. A head-mounted obstacle keeps its position relative to a neighboring cutter holder as the head turns.

A cutter counts once if the search finds any combination of stop angle and joint configuration that passes all checks. Different cutters may use different stops. Design selection, refined evaluation, holdout evaluation and dexterity statistics all use this per-cutter definition.

The code stores a full $4\times4$ transform for each target. To obtain reproducible inverse solutions, it samples the redundant coordinates and enumerates the analytical branches. The following equations are derived for this project from the product of the Table 1 transforms.

Let $L$ be the upper-arm length, $l$ the tool offset, $h=\theta_3+\theta_4$ and $s=h+\theta_6$. With the radial origin at the projection of joint 2 onto the base plane, the signed radial distance and axial coordinate of the TCP satisfy

$$
r=a_2+L\cos\theta_3-d_5\sin h-l\cos s,\qquad
z=d_1-L\sin\theta_3-d_5\cos h+l\sin s .
$$

1. Obtain the base yaw from the target $Z_7$ axis. The radial and axial components of $X_7$ give $s$. Set $r_w=r+l\cos s$.
2. Sample $\theta_3$ and $d_5$ on a regular grid within their limits. Enumerate both branches of $\sin h=(a_2+L\cos\theta_3-r_w)/d_5$. Recover $\theta_4=h-\theta_3$, $\theta_6=s-h$ and $d_1=z-l\sin s+L\sin\theta_3+d_5\cos h$. Wrap angles into their respective periodic intervals.
3. Discard configurations outside the joint limits. Recheck Euclidean position error and the orientation error $\cos^{-1}[(\operatorname{tr}(R_g^TR)-1)/2]$ with forward kinematics. A point inside the positional workspace must also satisfy the orientation requirement to count.
4. Check the remaining configurations for chamber-wall contact, support collisions and nonadjacent self-collisions. For support-to-link and link-to-link checks, compute the continuous minimum distance between two finite line segments. Adjacent links share their joint and are exempt from the pairwise check. At the cabin opening, clip the segment against the radius-expanded gate boundary and apply a conservative rectangular clearance test. Contact and clearance exactly equal to the allowance both fail.
5. For each stop, save the six joint values, errors and remaining clearance of the feasible solution with the highest dexterity. For each cutter, select the best such solution across stops. When dexterity values differ by at most $10^{-12}$, prefer the shortest rotation from mechanical zero. If the rotation distances also tie, choose the smaller angle expressed in the positive rotation convention. The main program independently recomputes the full-chain poses with the toolbox and repeats the acceptance checks.

Every target has the same search budget. The $41\times17$ grid used for design selection is nested in the $81\times33$ verification grid. The solver targets the specified pose and applies tolerances during acceptance. If the search finds no solution, a feasible configuration may still lie between grid points or at another orientation within tolerance. The measured accessibility rate is therefore a lower bound for the chosen targets and search budget. A successful static pose leaves the approach, withdrawal and complete replacement sequence unverified.

## 4. Dexterity and design selection

Let $J$ be the geometric Jacobian in toolbox order, with angular velocity first and linear velocity second. All designs share a characteristic length $L_c=1$ m to normalize translational velocity and prismatic joint inputs. Revolute inputs use a reference of 1 rad. The normalized Jacobian is

$$
J_n=\operatorname{diag}(1,1,1,1/L_c,1/L_c,1/L_c)\,J\,
\operatorname{diag}(L_c,1,1,1,L_c,1).
$$

For the Table 1 mechanism, base yaw changes both tangential position and orientation. The remaining joints together control radial position, axial position and in-plane pitch. The mechanism thus has at most four independent instantaneous motion directions. At a typical configuration, $\sigma_5$ and $\sigma_6$ are numerically zero and the six-dimensional manipulability volume vanishes. The ratio $\eta=\sigma_4/\sigma_1$ measures conditioning in the four attainable directions. It approaches zero when the fourth direction also degenerates. Every target still undergoes the full three-dimensional position and orientation checks. Batch evaluation uses an equivalent radial, axial and pitch block. Tests compare its result with a full singular value decomposition of the toolbox Jacobian.

The search grid contains four upper-arm lengths and five telescoping strokes, including the baseline. Designs must first pass the structural constraints. The search then ranks them by the number of accessible cutters, followed by mean dexterity across all cutters, with zero assigned to failures. An exact tie favors the smaller sum of upper-arm length and stroke. Each cutter contributes the dexterity at its selected stop. The reported statistics also include means over each design's reachable cutters and over the cutters reached by both designs. The random workspace comparison uses the same normalized joint samples at mechanical zero. Its collision-free fraction depends on the joint sampling distribution. The selected design is optimal within the tested grid.

## 5. Results and verification

| Field or file | Meaning |
|---|---|
| `cutters.csv` and `holdout_cutters.csv` | One row per design and cutter. Reports the numbers of tested and feasible stops, the total candidate counts, the selected mechanical angle and world azimuth, and the joint solution. The selected angle and joint solution are NaN when no solution is found. |
| `targets.csv` and `holdout_targets.csv` | One record per stop and cutter for each design. `CutterID` identifies the cutter and `Target` identifies the evaluation row. `Head_deg` is rotation from mechanical zero. `Azimuth_deg` is holder azimuth in the base frame. The per-cutter `WitnessTarget` points to the evaluation row in the same dataset and design. |
| `q_1` through `q_6`, `posErr` and `rotErr` | Joint values of the selected solution in meters and radians, recomputed position error in meters and orientation error in radians. These values are NaN when no solution passes. |
| `nPose` and `nFree` | Number of candidates within joint limits that pass pose checks, and the subset that also passes collision checks. These are sample counts. Coincident analytical branches can produce duplicates. |
| `nWall`, `nBrace`, `nSelf` and `gap` | Each rejected collision candidate is assigned to one category according to its smallest clearance. For a feasible solution, `gap` is the conservative remaining clearance in meters after subtracting the 8 mm allowance. |
| `paper_checkpoints.csv`, `paper_table2_replay.csv` and `implementation_checks.csv` | Independent Cartesian checkpoints, reconstructed Table 2 configurations and discrepancies between the toolbox and independent implementations. |
| `performance.csv` | Under the defaults, `Selection` uses 15° stops and the coarse inverse-kinematics grid. `Refined` uses 5° stops and the finer grid for the same cutter IDs. `Holdout` uses new radii and offset stops. `MeanEtaAll` assigns zero to failures. `MeanEtaCommon` averages over cutters reached by both designs. |
| `study.mat` | `goals.pose(:,:,i)` and `holdoutGoals.pose(:,:,i)` store the target pose for the corresponding cutter and stop. `baselineWorkspace` and `optimizedWorkspace` retain the raw workspace samples and their individual feasibility results. |
