# 温迪具身智能工程

目的：本项目尝试以温迪为原型，建立一个能够运动并与环境交互的真实具身智能。(复活吧我的爱人！！！)

项目目前处于第一阶段：角色建模


## 初期项目结构

项目代码集中在 `code/`，模型和所需资源集中在 `models/`。本地 Python 环境及 MuJoCo 原生程序放在独立目录中，不提交到 GitHub。

```text
Venti-Embodied_AI/
├── README.md
├── code/
│   ├── requirements.txt        # Python 依赖
│   ├── view_model.py           # MuJoCo Studio 启动入口，也支持无界面加载检查
│   ├── mjcf/                   # PMX 读取、MJCF 转换及参考模型验证
│   └── skeleton/               # 当前简化模型的重建与验证
├── models/
│   ├── Venti_origin/           # 原始 PMX 模型及使用规则
│   ├── Venti_MJCF/             # 原始转换结果：完整外观、初始骨架及 assets
│   └── Venti_skeleton_updated/ # 当前维护的简化机器人模型
├── .tools/                     # 本地安装的 MuJoCo 原生发行包（可选），安装后生成
└── .venv/                      # 本地 Python 虚拟环境，创建后生成
```

当前主模型是 `models/Venti_skeleton_updated/venti_skeleton_plain.xml`，包含 **45 个机器人 body、69 个关节自由度**，其中身体占 29 个自由度，双手占 40 个自由度。每只手都具有独立的五指结构。


## 开始之前

以下步骤以 **Windows x86_64、Python 3.12、MuJoCo 3.17.0** 为例，命令均在项目根目录的 PowerShell 中执行。

当前使用 MuJoCo 官方安装包中的 **MuJoCo Studio（`mujoco_studio.exe`）** 查看模型，通过独立界面浏览模型结构、调整关节和检查姿态。

可以通过 `code/view_model.py` 启动 Studio，也可以直接运行安装包中的 EXE。项目模型已保存在 `models/` 中。

选择 Studio 的理由是界面更加好看（bushi）。

### 创建 Python 环境


```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r code\requirements.txt
```

已有 `.venv` 时可以跳过创建步骤。依赖安装完成后，可确认 MuJoCo 是否能够导入：

```powershell
.\.venv\Scripts\python.exe -c "import mujoco; print(mujoco.__version__)"
```

预期输出为 `3.17.0`。


## 安装 MuJoCo 并使用 Studio 查看模型

### 下载并放置原生发行包

打开 [MuJoCo 3.17.0 官方发行页](https://github.com/google-deepmind/mujoco/releases/tag/3.17.0)，在 Assets 中下载 Windows x86_64 版本 `mujoco-3.17.0-windows-x86_64.zip`。并将其存到`.tools`文件夹。
```

目录结构：

```text
.tools/
└── mujoco-3.17.0-windows-x86_64/
    ├── bin/
    │   ├── mujoco_studio.exe
    │   ├── mujoco.dll
    │   └── assets/
    └── ...
```

### 查看方式1：通过项目脚本启动

运行：

```powershell
.\.venv\Scripts\python.exe code\view_model.py
```

默认加载 `models/Venti_skeleton_updated/venti_skeleton_plain.xml`。

需要更换模型时，添加 `--model` 参数:

```powershell
.\.venv\Scripts\python.exe code\view_model.py --model models/Venti_skeleton_updated/venti_skeleton.xml
.\.venv\Scripts\python.exe code\view_model.py --model models/Venti_MJCF/venti_skin.xml
```

### 查看方式2：直接运行 Studio

命令行启用：

```powershell
& '.\.tools\mujoco-3.17.0-windows-x86_64\bin\mujoco_studio.exe' '--model_file=models/Venti_skeleton_updated/venti_skeleton_plain.xml'
```

或者先运行exe程序，再将xml文件拖入即可。