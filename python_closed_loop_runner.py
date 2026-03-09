import ctypes
import os
import sys

import mujoco
import mujoco.viewer


LIB_FILENAME = "openarm_closed_loop.so"
MODEL_PATH = "model/openarm_head.xml"


def main() -> int:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_path = os.path.join(current_dir, LIB_FILENAME)
    model_path = os.path.join(current_dir, MODEL_PATH)

    if not os.path.exists(lib_path):
        print(f"[错误] 找不到闭环库文件: {lib_path}")
        return 1

    if not os.path.exists(model_path):
        print(f"[错误] 找不到模型文件: {model_path}")
        return 1

    core_lib = ctypes.CDLL(lib_path)
    core_lib.sim_init.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    core_lib.sim_init.restype = None
    core_lib.sim_step.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    core_lib.sim_step.restype = None
    core_lib.sim_close.argtypes = []
    core_lib.sim_close.restype = None

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print("正在调用闭环控制 Init...")
    core_lib.sim_init(ctypes.c_void_p(model._address), ctypes.c_void_p(data._address))

    def controller_callback(m, d):
        core_lib.sim_step(ctypes.c_void_p(m._address), ctypes.c_void_p(d._address))

    mujoco.set_mjcb_control(controller_callback)

    try:
        print("启动闭环抓取 Viewer...")
        mujoco.viewer.launch(model, data)
    finally:
        mujoco.set_mjcb_control(None)
        core_lib.sim_close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
