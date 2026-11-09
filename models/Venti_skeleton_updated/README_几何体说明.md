# 温迪简化机器人：身体结构与五指模型

2026-11-08 修订。用途是可互动温迪的第一阶段：几何、关节与接触建模。
参考 `../Venti_MJCF/venti.xml` 的身体骨架，以及原 PMX 的手指骨节位置。原 Venti_MJCF 未修改。

## 模型入口

| 文件 | 用途 | 显示组 |
|---|---|---|
| venti_skeleton_plain.xml | 推荐建模入口，直接显示灰色物理包络 | Geom 1 |
| venti_skeleton.xml | 同一物理模型，加少量温迪风格彩色装饰 | Geom 2 外观；Geom 3 碰撞包络 |
| venti_skeleton_untextured.xml | 同一装饰布局，统一灰色 | 同上 |

三个 XML 都自包含，不依赖 OBJ、贴图、SKN 或 JSON，物理结构一致。地面为 Geom 0。
掌心、指尖和合并前参考原点是 Site 4。隐藏显示组不会关闭接触。

项目根目录运行：

```powershell
& 'D:\program files\mujoco-3.17.0-windows-x86_64\bin\simulate.exe' '.\models\Venti_skeleton_updated\venti_skeleton_plain.xml'
```

也可以拖入 MuJoCo 窗口。暂停后调整 Joint 面板。
在 simulate 的 Simulation 面板选 Key，再点击 Load key：

| 索引 | 名称 | 用途 |
|---|---|---|
| 0 | open | 原 A 姿势，双手展开 |
| 1 | relaxed | 手指轻微弯曲 |
| 2 | fist | 握拳的运动学参考姿势 |
| 3 | point | 食指伸直，其余手指弯曲 |

这些是静态参考姿势，不是执行器控制或抓握技能。

## body 与 geom 调整

原来 19 个 body 中，4 个没有独立关节的中转节点进行了合并：

- chest 的几何和子节点转入 waist，保留 chest_reference site。
- right_clavicle、left_clavicle 合并至躯干；肩部关节锚点不变。
- 旧 head 与旧 neck 合为一个头颈刚性连杆，最终命名 head；枢轴仍在原颈关节，旧头部原点保留为 head_reference。

固定 body 并不一定错误；这里简化的是不需要独立运动的参考框架。原 29 个身体关节名和锚点保留。
身体从 19 个 body 减为 15 个。waist 包含腰和胸的固定连接，没有新增胸部自由度。
原大椭球手形及固定装饰拇指改为薄盒手掌和独立五指；手掌尺寸按原指根位置计算。
足底盒体和踝部椭球保留，两者共同连接小腿与足底；简单删除踝部会留下明显几何空隙。
彩色/灰色装饰版本移除了多层披风、描边、纽扣和花饰等；保留少量帽子、羽毛、辫子、眼睛。
披风为两块无碰撞刚性面板，没有布料模拟。装饰不参与质量和惯量计算。

## 五指结构

原 PMX 有手指骨骼，只是第一次转换时合并进手掌。本次读取原骨骼并存入 hand_reference.json；重建不再依赖原 PMX。
尺度、指根、关节和指尖位置来自源数据；转轴、范围和碰撞半径属于本次建模选择。

每手 15 个 body、20 个转动自由度：

- 食指、中指、无名指、小指各三段：MCP 张合、MCP 弯曲、PIP 弯曲、DIP 弯曲，各 4 自由度。
- 拇指三段：CMC 对掌、CMC 弯曲、MCP 弯曲、IP 弯曲，共 4 自由度。
- 拇指段统一使用 proximal/middle/distal 后缀，解剖上对应掌骨、近节、远节。
- 每段为胶囊体，圆端按原骨节和指尖位置衔接；末端有 tip site。

最终为 **45 个机器人 body、69 个关节自由度**，另有一个 world body。身体 29 + 双手 40。
例如 left_index_mcp_spread、left_index_mcp_flex、left_index_pip_flex、left_index_dip_flex；
拇指为 left_thumb_cmc_opposition、left_thumb_cmc_flex、left_thumb_mcp_flex、left_thumb_ip_flex。
右手替换前缀为 right。掌心 site 如 left_palm_center，指尖如 left_index_tip。
四指正弯曲朝向选择的掌侧法向；拇指有第二条轴用于对掌。
手腕按实际掌面修正轴向：wrist_pitch 横掌，wrist_roll 沿掌长方向。
这是虚拟机器人腕部设计，没有完整复现人类前臂旋前/旋后机构。

新增手指后旧 qpos 索引发生变化。后续按关节名称查 jnt_qposadr/jnt_dofadr，不要沿用旧 29 个下标。
最新顺序见 model_revision.json。XML angle=degree 影响关节范围等定义；Python qpos 与 keyframe qpos 使用弧度。

## 质量、惯量与接触

- 删除每个 body 固定 1 kg、惯量 0.01 的占位惯性。
- 身体包络分配分段名义质量；手指按胶囊体体积及假定等效密度 700 kg/m³ 估算。
- inertiafromgeom=true，从物理包络推导质心和惯量；装饰不参与。
- 总名义质量约 24.42 kg，是虚拟身体的初始设计值，没有现实硬件标定依据。
- 所有关节有初始活动范围。手指单独使用较小阻尼和 armature。
- 机器人接触位 1/2、环境 2/1；与环境接触开启，机器人自碰撞仍关闭。
- 掌面和指根有少量有意重叠；四个预设中，中/远指节不穿入手掌，不同手指之间不穿插。

允许范围内的任意关节组合仍可能穿插。抓握阶段需设计手掌/手指自碰撞、相邻连杆排除，再配置执行器与反馈控制。
当前仍固定骨盆、零重力、无执行器。通过检查不代表会站立、行走或物理抓握。
Venti_MJCF 的原 OBJ/SKN 未重绑手指；要使原角色皮肤跟随新手指，需要重新导出蒙皮权重。

## 验证与重建

主模型目录不保留旧预览 PNG 和验证报告 JSON；桌面旧版完整快照位于 backups/desktop_before_project_setup_20261109。
保留 hand_reference.json（原 PMX 指骨参考位置，重建与验证脚本读取）及 model_revision.json（关节顺序与掌面参考，验证脚本读取）。
这两个 JSON 不是 MuJoCo 加载或训练所需资源，但删除会使现有工具失效。三个 XML 均可独立使用。
如再次运行 code/skeleton/verify_model.py，会重新生成 validation.json、validation_untextured.json 和 previews_revision 内的预览图。

验证包括三个版本物理数组一致、原关节锚点与 PMX 指骨位置一致、69 个关节逐一运动、
四个姿态各运行 1000 步无警告、非零初速度运行、预设穿插检查、足底接触和指节外物接触测试。
这些不是平衡或抓握成功率测试。

建议在工程根目录创建独立 .venv，依赖为 MuJoCo 3.17.0、numpy、Pillow：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r code\requirements.txt
.\.venv\Scripts\python.exe code\skeleton\verify_model.py
```

code/skeleton/build_model.py 从修订前备份和 hand_reference.json 重建三个 XML，会覆盖当前 XML。
手工修改 XML 后请勿直接重建，除非同时更新生成脚本。原文件备份在 backups/before_hand_revision_20261108。
下一阶段建议固定底座下先给腕部和一根手指加执行器，验证平滑动作，再做全手协同与物体接触。
