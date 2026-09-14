# sw2robot — Inventor 支持分支

这是 `inventor-support` 分支的 Windows 使用说明。

本分支在原版 sw2robot 的基础上增加了 **Autodesk Inventor `.iam/.ipt` → graph.json + STL → Web Editor → URDF / MuJoCo MJCF** 的工作流。

> 当前 Inventor 支持只在 `inventor-support` 分支中。请不要安装上游 Release 的普通 `sw2robot-web.exe` 来测试 Inventor，因为正式 Release 目前还不包含这个后端。
>
> 原项目的完整英文说明保存在 [`README_UPSTREAM.md`](README_UPSTREAM.md)，Inventor 后端的更详细说明见 [`examples/INVENTOR.md`](examples/INVENTOR.md)。

## 1. Windows 电脑需要什么

建议使用 Windows 10/11 x64，并安装：

- **Autodesk Inventor**（用于读取 `.iam/.ipt`）
- **Python 3.12 x64**
- **Git**

如果没有 Python 3.12 或 Git，可以在 PowerShell 中安装：

```powershell
winget install -e --id Python.Python.3.12
winget install -e --id Git.Git
```

安装完成后重新打开 PowerShell，确认：

```powershell
py -3.12 --version
git --version
```

## 2. 下载 Inventor 分支

在 PowerShell 中执行：

```powershell
cd $HOME
git clone -b inventor-support --single-branch https://github.com/ssybh2/solidworks_urdf_exporter2.git
cd solidworks_urdf_exporter2
```

如果你以前已经 clone 过这个仓库：

```powershell
cd 你的仓库路径
git fetch origin
git switch inventor-support
git pull origin inventor-support
```

## 3. 安装 sw2robot

下面的命令**不需要执行 `Activate.ps1`**，因此即使 PowerShell Execution Policy 比较严格也可以使用。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e .
```

如果你还想使用完整的自碰撞检测、自动关节极限和 CoACD 功能，可以再安装可选组件：

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[ui,coacd]"
```

可选组件安装失败不会影响最基本的 Inventor → URDF/MJCF 转换，可以先只使用 `pip install -e .`。

## 4. 推荐：启动网页编辑器

执行：

```powershell
.\.venv\Scripts\sw2robot-web.exe
```

然后浏览器会打开：

```text
http://localhost:8090
```

如果没有自动打开，就手动在浏览器输入上面的地址。

### 在网页中打开 Inventor 文件

1. 点击页面里的文件浏览器。
2. 找到你的 Inventor 文件：
   - `.iam`：Inventor Assembly，推荐用于机器人。
   - `.ipt`：Inventor Part，会作为单 link 模型处理。
3. 点击 `.iam/.ipt`。
4. sw2robot 会调用 Inventor COM API，读取零件、装配变换、质量/质心/惯量、UCS 和关节信息，并导出 STL。
5. 抽取完成后会自动进入原来的 sw2robot Web Editor。
6. 在网页中检查/修改 root、joint type、axis、limit、mass、collision 等。
7. 最后从网页导出 URDF / ROS package / MuJoCo MJCF。

如果 Inventor 已经打开，可以保持它运行；本分支会优先尝试复用正在运行的 Inventor。没有运行时会尝试启动 Inventor COM 实例。

## 5. 命令行直接转换

如果不想先开网页，也可以直接转换 `.iam`。

### 生成 URDF

```powershell
.\.venv\Scripts\inv2robot.exe "D:\CAD\my_robot.iam" -o "D:\robot_export"
```

输出目录会类似：

```text
D:\robot_export\my_robot\
    graph.json
    meshes\
    urdf\my_robot.urdf
```

### 同时生成 MuJoCo MJCF

```powershell
.\.venv\Scripts\inv2robot.exe "D:\CAD\my_robot.iam" -o "D:\robot_export" --mujoco
```

固定在世界上的机械臂/平台可以使用：

```powershell
.\.venv\Scripts\inv2robot.exe "D:\CAD\my_robot.iam" -o "D:\robot_export" --mujoco --mujoco-fixed-base
```

### 复用已经打开的 Inventor

```powershell
.\.venv\Scripts\inv2robot.exe "D:\CAD\my_robot.iam" --attach --mujoco
```

### 只抽取 CAD，之后再进网页编辑

```powershell
.\.venv\Scripts\inv2robot.exe "D:\CAD\my_robot.iam" -o "D:\robot_export" --extract-only
.\.venv\Scripts\sw2robot-web.exe
```

## 6. Inventor 里面怎样建模最容易识别

为了让机器人关节自动识别得更可靠，优先使用 Inventor 的 **Assembly Joint**：

| Inventor | sw2robot / URDF |
| --- | --- |
| Rigid | fixed |
| Rotational | revolute |
| Slider | prismatic |
| Locked | fixed |

建议给 IMU、相机、末端执行器、脚底等重要位置创建并命名 **UCS (User Coordinate System)**，例如：

```text
UCS_IMU
UCS_CAMERA
UCS_TCP
UCS_FOOT_FL
```

这些 UCS 会被读取到 sw2robot 的坐标系数据中。

## 7. 当前 Inventor 版本的限制

这是实验性第一版，建议先用一个简单机器人验证。

- `Rigid / Rotational / Slider` Assembly Joint 支持最好。
- `Cylindrical / Planar / Ball` 属于多自由度关节，目前保守处理为 fixed，并给出警告。
- 传统 `Mate / Flush / Angle / Tangent` 约束的自动推断暂时没有 SolidWorks 后端那么完整；`Insert` 已有基础识别。
- 可运动子装配的内部结构目前还没有完整递归展开。
- Inventor Model States / iAssembly 暂时没有映射成 sw2robot configuration。
- 目前还需要在真实 Inventor `.iam` 模型上继续做端到端验证。

## 8. 常见问题

### `py -3.12` 找不到

安装 Python 3.12 x64，然后**关闭并重新打开 PowerShell**：

```powershell
winget install -e --id Python.Python.3.12
```

### PowerShell 不允许执行脚本

本 README 的命令不要求执行：

```powershell
.\.venv\Scripts\Activate.ps1
```

直接使用：

```powershell
.\.venv\Scripts\python.exe
.\.venv\Scripts\sw2robot-web.exe
.\.venv\Scripts\inv2robot.exe
```

即可。

### Inventor 连接失败

先手动启动 Autodesk Inventor，并打开目标 `.iam`，然后测试：

```powershell
.\.venv\Scripts\inv2robot.exe "D:\CAD\my_robot.iam" --attach --extract-only
```

### 网页文件浏览器里看不到 `.iam/.ipt`

先确认当前确实在这个分支：

```powershell
git branch --show-current
git pull origin inventor-support
```

应该显示：

```text
inventor-support
```

然后重新安装一次 editable package：

```powershell
.\.venv\Scripts\python.exe -m pip install -e .
```

再重新启动：

```powershell
.\.venv\Scripts\sw2robot-web.exe
```

## 最短使用流程

已经安装 Python 3.12、Git 和 Autodesk Inventor 的情况下，只需要：

```powershell
git clone -b inventor-support --single-branch https://github.com/ssybh2/solidworks_urdf_exporter2.git
cd solidworks_urdf_exporter2
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e .
.\.venv\Scripts\sw2robot-web.exe
```

然后打开浏览器中的 `http://localhost:8090`，选择你的 `.iam` 文件即可。
