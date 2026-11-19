# CutterReach

**受限隧道掘进机工作舱内换刀机械臂的工作空间可达性分析与结构改进**

[English README](../README.md)

CutterReach 复现了 Yang 等人提出的六自由度换刀机械臂，包括四个转动关节和两个移动关节（4R2P）。项目研究的问题是：**转动刀盘后，机械臂能否到达每把刀的换刀位置，并以规定姿态对准刀座？** 对每个刀号，只要在一个允许的停靠角能找到同时满足工具中心点（TCP）位置、工具完整姿态、关节限位和碰撞净空要求的构型，就记为可达。可达率按刀具数计算，同一把刀只计一次。

默认模型在同一条刀盘辐条上布置了 10 把刀。沿整圈每隔 5° 检查一个停靠角，基准方案可达 **3/10 把**，优化方案可达 **5/10 把**。在这组布置下，两种方案各自能服务的刀具恰好都能在机械零位到达。换一个停靠角，结果可能改变。例如，优化方案在 90° 时受阻，转到 180° 就能到达 08 号刀。刀具布置和支架相对位置均为课程建模假设。[逐刀结果与选定停靠角](../results/findings.md)

## 在 MATLAB 中运行

项目需要 MATLAB 和 Robotics System Toolbox，已在 R2026b 上验证。将 MATLAB 的 **Current Folder** 设为包含 `run_study.m` 的项目根目录，再执行以下命令。

```matlab
study = run_study;                % 复核论文基准、比较设计并导出结果
demo_study(study);                % 在三维界面中查看机械臂、刀具与刀盘停靠角
assertSuccess(runtests('tests')); % 运行自动化测试，若有测试未通过则报错
```

项目附带已计算的结果。直接运行 `demo_study` 即可打开演示。选定设计和**刀号**后，保留 **Automatic stop selection**（自动选择停靠角），可查看该刀是否可达、程序选取的角度以及可行停靠角的数量。手动选择停靠角可以比较同一把刀在不同角度的情况。界面中的 0° 表示刀盘相对机械零位的转角为零，此时装刀辐条在机械臂基坐标系中的方位角为 90°。

左图的蓝色菱形标记固定基座 `base_link`，紫色圆环标记所选刀具的位置。程序未找到可行解时，机械臂显示为收纳姿态。右图可以独立旋转，并可切换 **Cutter close-up**（刀头特写）、**Installed arm detail**（安装局部）、**Chamber overview**（工作舱全景）和 **Arm assembly**（整臂装配）。在全景中，刀盘、刀座、支架及其安装立座一起旋转，工作舱、收纳舱和导轨保持固定。收纳舱沿 X、Y、Z 方向的尺寸分别为 1.18、1.15、3.09 m。[演示截图](../results/demo_fixed_base.png)

在终端可使用 `matlab -batch "run_study;"` 批量运行。每次运行都会更新所配置输出目录中的结果文件。

以下示例从优化方案中取出一组已验证的关节构型，在 Robotics System Toolbox 中显示可编辑的三维模型，并计算对应的末端变换和几何雅可比矩阵。

```matlab
load('results/models.mat','optimizedRobot')
load('results/study.mat','study')
i = find(study.optimizedCutters.FeasibleStops > 0, 1);
headRotation_deg = study.optimizedCutters.BestHead_deg(i)
q = study.optimizedCutters.q(i,:);
show(optimizedRobot,q,'Frames','on')
T07 = getTransform(optimizedRobot,q,'frame7');
J = geometricJacobian(optimizedRobot,q,'frame7');
```

![换刀机械臂装配与刀头特写](../results/assembly.png)

金色部件包括承载侧板、套筒和夹爪，银灰色部件是可伸缩的内臂，铜色外环是盘形滚刀的刀圈。工作舱视图还显示了刀盘背面的四根工字形支架及其安装立座。每种设计在工具箱模型和演示窗口中使用同一套三维几何。[部件尺寸与碰撞建模假设](methods.md)

## 论文基准复现

基准模型采用论文表 1 的修正 Denavit–Hartenberg（MDH）参数。六个关节依次为 P–R–R–R–P–R，第七坐标系固定在工具上。模型算出的终点与论文独立报告的终点位置相差 **1.30 mm**。代入论文给出的 1.310 m 伸缩长度可以复现起点，但该长度低于表 1 的 1.420 m 下限。表 2 中的七组关节构型均通过限位检查，按各组关节量重建的末端位姿也均能求得满足限位的逆解。[基准核对、DH 参数表与假设](methods.md)

在表 1 的关节范围内均匀采样 30,000 次，得到的轴向 Z 范围为 **−2.904 至 6.039 m**。论文图 10 的配套文字报告为 **−0.2 至 4.6 m**。项目还给出了两组满足表 1 限位、但轴向位置超出论文所报范围的具体构型。后续结构对比统一采用表 1 的模型。[采样范围](../results/paper_workspace_extents.csv) · [两组轴向反例](../results/paper_axial_examples.csv) · [论文原文，第 218–221 页](https://mechanika.ktu.lt/index.php/Mech/article/download/30386/15886)

## 仿真结果

| 默认加密网格下的指标 | 基准方案（Baseline） | 优化方案（Optimized） |
|---|---:|---:|
| 大臂长度 / 伸缩行程 | 732 / 650 mm | 900 / 875 mm |
| 至少一个停靠角满足目标位姿和关节限位的刀具数 | 7/10 | 9/10 |
| 至少一个停靠角还同时通过碰撞检查的刀具数 | 3/10 | 5/10 |
| 两种方案均可达的 3 把刀上的平均灵巧度 | 0.2132 | 0.2168 |
| 各方案自身可达刀具上的平均灵巧度 | 0.2132 | 0.2026 |
| 留出集可达刀具数（另选 9 个半径，停靠角网格偏移 2.5°） | 4/9 | 5/9 |
| 刀盘处于机械零位时，随机关节构型中的无碰撞比例 | 11.38% | 9.92% |

基准方案可以到达 01、02、07 号刀。优化后新增 08 号刀（半径 2.95 m）和 09 号刀（半径 3.30 m）。对于 03–06 号刀，两种方案都找到了满足位姿和关节限位的候选构型，但所有候选均在碰撞检查中被淘汰。对于 10 号刀（半径 3.65 m），两种方案在所有测试角度均未找到满足关节限位的目标位姿解。

刀盘分度会改变支架相对工作舱的位置，而支架与同一刀盘上刀座的相对位置保持固定。本研究验证刀盘停稳后的静态换刀姿态。机械臂的接近、退出和完整拆装过程仍需单独验证。[逐刀解](../results/cutters.csv) · [逐停靠角结果](../results/targets.csv) · [性能指标](../results/performance.csv)

![两种设计在各刀号和刀盘停靠角下的可达性](../results/accessibility.png)

## 项目文件与参数

| 文件 | 用途 |
|---|---|
| [run_study.m](../run_study.m) | 运行基准复核、仿真、设计比较和结果导出 |
| [+cr/config.m](../+cr/config.m) | 配置 DH 尺寸、关节限位、舱体与支架、目标、容差、采样和候选设计 |
| [+cr/buildRobot.m](../+cr/buildRobot.m) 和 [+cr/fk.m](../+cr/fk.m) | 前者构建含滑台、销轴、伸缩臂和滚刀装配的 `rigidBodyTree`，后者独立计算 MDH 正运动学 |
| [+cr/solvePose.m](../+cr/solvePose.m) 和 [+cr/collision.m](../+cr/collision.m) | 在有限网格上搜索冗余关节变量，复核完整目标位姿，并对连杆的线段胶囊包络做碰撞检查 |
| [+cr/dexterity.m](../+cr/dexterity.m) 和 [+cr/searchDesign.m](../+cr/searchDesign.m) | 计算可实现运动方向上的雅可比条件指标，并搜索离散结构参数组合 |
| [tests/TestCutterReach.m](../tests/TestCutterReach.m) | 测试论文控制点、雅可比矩阵与数值微分的一致性、伸缩与角度限位、工具位姿、舱体碰撞和自碰撞等 |
| [data/paper_table2.csv](../data/paper_table2.csv) | 保存论文七个刀号的关节记录，列名标明毫米或度 |

每把刀具有唯一编号 `CutterID`、安装半径 `Radius_m` 和机械零位时的方位角 `HomeAzimuth_deg`。同一半径上可以登记多个刀号。获得真实刀盘布置后，可替换 `c.cutters`。如需将修改设计网格或舱体参数后的实验结果写入单独目录，在运行前设置 `c.out`。

```matlab
c = cr.config;
c.armGrid = [0.70 0.732 0.80 0.85];
c.strokeGrid = [0.60 0.65 0.75 0.85];
c.out = fullfile(c.root,'results_variant');
variant = run_study(c);
```

`run_study` 使用 `c.baseline` 复现论文模型并核验论文给出的坐标。单独探索其他机械臂设计时，可用 `b = c.baseline` 复制基准参数，修改 `b.arm`、`b.limits` 等字段，再直接调用 `cr.buildRobot`、`cr.fk`、`cr.solvePose` 或 `cr.evaluate`。运动学计算使用米和弧度，表格与图中的展示单位以各自标注为准。

## 课程报告、汇报与仿真演示

- **期末报告**可依据方法文档说明建模假设和可达性判据，并用计算摘要、候选设计网格和留出集逐刀结果讨论性能。[方法与数据来源](methods.md) · [计算摘要](../results/findings.md) · [全部候选设计](../results/design_grid.csv) · [留出集逐刀结果](../results/holdout_cutters.csv)
- **课堂汇报**可先展示舱体和机械臂，再依次展示工作空间、逐刀覆盖情况、结构设计对比和灵巧度。[工作舱模型](../results/model.png) · [工作空间](../results/workspace.png) · [逐刀可达性](../results/accessibility.png) · [设计对比](../results/design_comparison.png) · [灵巧度](../results/dexterity.png)
- **仿真演示**可运行 `demo_study`。默认选中一把优化后新增的可达刀具。选择 08 号刀并保留 **Automatic stop selection**，在 **Baseline** 和 **Optimized** 之间切换可比较机械臂结构的影响。随后保持 **Optimized**，在 90° 与 180° 之间切换可观察刀盘分度的影响。每个画面对应一个静态装配位姿。
- **MATLAB 数据文件**中，`study.mat` 保存配置、主实验与留出集的目标变换、两种方案的关节采样、末端位置、净空、失败原因、逐目标解和汇总指标。`models.mat` 保存两棵可用 `show` 显示的机器人模型。[研究数据](../results/study.mat) · [机器人模型](../results/models.mat)
