# 温迪基础 MJCF 模型

已直接读取你的 `Venti.pmx`，生成可由 MuJoCo 加载的模型，无需先经过 Blender。

## 先打开哪个文件

- **`venti.xml`**：常规机器人建模版本。采用 OBJ 外观网格、19 个机器人 body、29 个 hinge 关节自由度。建议以此为后续机器人建模起点。
- **`venti_skin.xml`**：相同骨架的蒙皮外观版本。保留合并后的顶点权重，弯曲关节时外观更连续。蒙皮只负责显示，不参与碰撞。
- `assets/`：两个 XML 引用的网格、贴图和蒙皮数据，必须与 XML 一起保留。XML 不是单文件自包含模型。
- `venti_preview.png`、`venti_skin_posed.png`：实际由 MuJoCo 渲染的初始姿态和关节调整示例。

双击 **`open_in_mujoco.cmd`** 可以用当前电脑已安装的 MuJoCo 打开基础版本；双击 `open_skin_in_mujoco.cmd` 打开蒙皮版本。脚本使用本机路径：

```text
D:\program files\mujoco-3.17.0-windows-x86_64\bin\simulate.exe
```

也可以启动 MuJoCo，将 `venti.xml` 拖入窗口。在关节面板中修改关节位置即可查看运动。

使用 Python 查看时，在此文件夹运行（先安装 `mujoco`）：

```powershell
python -m mujoco.viewer --mjcf=venti.xml
```

## 已做的转换

- 源模型：16,211 个顶点、20,703 个三角面、355 根骨骼。
- 保留躯干、颈部、头部、双臂、双手、双腿和双脚；辅助骨骼、手指、头发、披风骨骼合并到相应主要连杆。
- 29 个自由度：腰部 3、颈部 2、双肩 6、双肘 2、双腕 4、双髋 6、双膝 2、双踝 4。
- 38 个 OBJ 外观网格，保留 20,261 个三角面和基础颜色贴图。省略藏在头内的 442 个表情辅助面。
- 统一为米和右手坐标系：X 向前、Y 向左、Z 向上。
- 换算关系为 PMX `(x,y,z)` → `(-z,x,y) × 0.08`，并使最低顶点位于地面。
- 包含帽子/羽毛的总外观高度约 **1.5408 米**；这是换算假设，不是测得的真实角色身高。
- `qpos=0` 对应 PMX 原始 A 姿态，绝对关节位置来自源骨骼。关节类型、自由度和轴向是基础机器人模板的设计选择，不能从 PMX 推导出真实硬件结构。

## 当前物理设置

这是外观和运动学建模起点，尚未标定动力学参数：

- 根部 `pelvis` 固定在世界中；重力为零。
- 所有碰撞均关闭；地面也只用于显示。
- 每个 body 的占位质量为 1 kg、对角惯量为 `(0.01, 0.01, 0.01) kg·m²`。这些值只为满足 MuJoCo 加载/关节要求，不代表真实机器人。
- 没有电机、传感器、关节限位或机器人控制器。阻尼和 armature 也只是占位值。
- 常规版本按面片的主要蒙皮权重将三角形分到刚性连杆，大角度运动可能出现接缝。蒙皮版本改善外观连续性，但不会实现衣服或头发物理。
- 保留普通颜色贴图；未还原 MMD 的 toon、sphere 特效、表情 morph、IK 控制器和 MMD 刚体系统。

之后用于行走、接触或强化学习时，需要补充真实关节范围、质量/质心/惯量、简化碰撞体、执行器和接触参数，并根据需要添加浮动底座。`pelvis` 内已标注添加 `<freejoint name="root"/>` 的位置。

## 调整尺寸和重新生成

包内 `../../code/mjcf/convert_venti.py` 与 `../../code/mjcf/pmx_reader.py` 是本次转换使用的脚本，针对当前这份温迪骨骼结构编写。先安装 `numpy`、`Pillow`，再执行：

```powershell
python -m pip install numpy Pillow
python code/mjcf/convert_venti.py "models/Venti_origin/温迪.pmx" "models/Venti_rescaled" --scale 0.08
```

如果希望帽子/羽毛在内的总外观高度为 H 米，将比例改为 `H / 19.2599981`。比例会同时作用于骨骼位置和网格尺寸。

`bone_map.csv` 记录每根原骨骼归属的机器人 body；`conversion_info.json` 记录坐标换算、body 位置和关节顺序。

## 验证

已在 **MuJoCo 3.17.0** 中完成：

- 两个 XML 编译及加载；确认 `nq=29`、`nv=29`。
- 检查所有主要 body 在初始姿态下的位置与转换后的源骨骼一致。
- 逐个改变 29 个关节，检查连杆旋转有效。
- 在当前无重力、无接触设置下运行 1,000 步，未产生数值异常或 MuJoCo warning。这不代表真实机器人动力学已验证。
- 检查蒙皮初始姿态误差，并实际渲染、检查贴图和非零关节姿态。

完整结果见 `validation.json`。若安装了 `mujoco`、`numpy`、`Pillow`，可在本目录运行 `python code/mjcf/verify_venti.py` 复现检查和预览图。

源文件的作者及使用规则保存在 `source_model_rules.txt`。源 PMX 和原始贴图文件均未修改。


## 工程目录更新（2026-11-09）

Python 工具已移到 `../../code/mjcf/`。本文涉及 Python 工具的命令请从工程根目录执行。
验证命令为 `python code/mjcf/verify_venti.py`；转换示例为 `python code/mjcf/convert_venti.py "models/Venti_origin/温迪.pmx" "models/Venti_MJCF_rebuilt" --scale 0.08`。
XML 与 assets 的相对路径保持不变，CMD 启动脚本仍可直接使用。
