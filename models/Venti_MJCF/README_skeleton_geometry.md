# 温迪简化几何模型

## 新增无贴图版本（2026-11-08）

- `venti_skeleton_untextured.xml`：保留完整简化造型，统一灰色；无 texture、material 或外部文件依赖。
- `venti_skeleton_plain.xml`：去掉装饰，直接以不透明灰色显示 19 个碰撞包络；适合观察碰撞结构。
- 两个版本保留原来的 29 个关节、质量/惯量、碰撞尺寸和接触掩码。均通过 MuJoCo 3.17.0 加载及 1,000 步运行检查，无 warning。结果见 `validation_untextured.json`。
- 原彩色文件 `venti_skeleton.xml` 仍保留。原角色使用的是几何体纯色材质，没有角色图片贴图；此次额外移除了材质和天空纹理，并生成灰色版本。

本次修改 `venti_skeleton.xml`，保留原有的 19 个 body、29 个关节自由度、全部关节位置/轴向和占位惯性参数。原来的细杆和节点球被替换成符合角色体型的简化几何体。此 XML 自包含，无需外部网格或贴图。

## 外观与碰撞

- `class="visual"`、geom group **2**：躯干、服装、头部、手脚及温迪风格的绿帽、羽毛、辫子、披风、配饰。仅负责显示，不产生碰撞。
- `class="collision"`、geom group **3**：19 个碰撞包络，涵盖骨盆、腰、胸、颈、头、双上臂、前臂、手掌、大腿、小腿、足底和脚踝。使用 capsule、ellipsoid 和 box。通常在查看器中默认隐藏，开启 geom group 3 可见。
- `class="environment"`、geom group **0**：环境碰撞对象。地面属于此类。

碰撞包络按当前体型近似设计，用于初步几何接触实验，不是精确复现角色皮肤、衣服或真实机器人外壳。披风、帽子、辫子和面部细节不参与接触；披风当前刚性跟随胸部，没有布料模拟。

足部碰撞盒长度 0.18 m、宽度 0.072 m、高度 0.044 m，初始姿态下底面位于 Z=0；脚踝另有椭球包络。几何体 `size` 对 box 和 ellipsoid 表示半尺寸。

## 接触设置

机器人碰撞体使用 `contype="1" conaffinity="2"`，环境使用 `contype="2" conaffinity="1"`。机器人和环境之间的接触已启用；机器人自碰撞关闭，以免关节处相邻包络的合理重叠产生不必要的接触。

若在 `<worldbody>` 中添加静态障碍物，可使用：

```xml
<geom name="test_obstacle" class="environment" type="box"
      pos="0.4 0 0.7" size="0.05 0.10 0.10" rgba="0.5 0.5 0.5 1" />
```

若以后启用自碰撞，需要修改碰撞掩码，并为相邻及固定连接的部位设置合适的接触排除规则。不能仅把所有外观部件设置成可碰撞。

## 当前仍采用的设置

固定底座、零重力；各 body 的质量、质心和惯量仍为原来的占位值。没有加入电机、关节范围或控制器。此次变更适合先查看体型和进行环境接触调试；行走、平衡和动力学控制仍需配置实际机器人参数。

## 验证

在 MuJoCo 3.17.0 中检查了加载、初始姿态、四个方向的渲染及关节弯曲效果。验证原有骨架和惯量数据不变；运行 1,000 步无 warning、初始关节无漂移。

另在临时测试模型中验证了：环境球与头部包络产生接触；将脚底压入地面 1 mm 时，两脚足底共生成 8 个地面接触点，没有装饰物或自碰撞接触。临时测试对象未加入交付模型。

`validation.json` 是机器验证结果；`preview_front.png`、`preview_threequarter.png`、`preview_back.png`、`preview_side.png`、`preview_collision.png`、`preview_pose.png` 为实际 MuJoCo 渲染结果。


## Project directory update (2026-11-09)

Python tools are now under `../../code/mjcf/`. Run `python code/mjcf/verify_venti.py` from the project root. Conversion example: `python code/mjcf/convert_venti.py "models/Venti_origin/温迪.pmx" "models/Venti_MJCF_rebuilt" --scale 0.08`. XML asset paths remain unchanged.
