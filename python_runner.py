import mujoco
import mujoco.viewer
import ctypes
import os
import sys

# --- 加载 C++ 动态库 ---
lib_filename = "openarm_core.so"
current_dir = os.path.dirname(os.path.abspath(__file__))
lib_path = os.path.join(current_dir, lib_filename)

if not os.path.exists(lib_path):
    print(f"[错误] 找不到库文件: {lib_path}")
    sys.exit(1)

core_lib = ctypes.CDLL(lib_path)
core_lib.sim_init.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
core_lib.sim_init.restype = None
core_lib.sim_step.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
core_lib.sim_step.restype = None

# --- 加载 MuJoCo 模型 ---
model_path = "model/openarm_bimanual.xml"
if not os.path.exists(model_path):
     print(f"[错误] 找不到模型文件: {model_path}")
     sys.exit(1)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# --- 初始化 C++ 控制器 ---
print("正在调用 C++ Init...")
core_lib.sim_init(ctypes.c_void_p(model._address), ctypes.c_void_p(data._address))

# 设置控制回调函数
def controller_callback(m, d):
    core_lib.sim_step(ctypes.c_void_p(m._address), ctypes.c_void_p(d._address))

mujoco.set_mjcb_control(controller_callback)

# --- 启动可视化窗口 ---
print("启动 Python Viewer...")
mujoco.viewer.launch(model, data)