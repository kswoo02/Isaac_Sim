"""[R2 예제] D455 손목 카메라의 /rgb, /rgb_left, /depth, /camera_info 발행

실행: isaac_python r2_camera.py [장면 USD 경로]
      Play(▶) 후 rqt의 Image View 두 개에서 /rgb와 /rgb_left를 나란히 비교한다.
"""

import argparse
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SCENES = (
    SCRIPT_DIR / "scenes" / "pick_place_cube.usd",
    SCRIPT_DIR.parent / "scenes" / "pick_place_cube.usd",
)


def parse_scene_path():
    parser = argparse.ArgumentParser(description="Publish the D455 wrist-camera streams.")
    parser.add_argument("scene", nargs="?", type=Path, help="path to the pick-and-place USD scene")
    args = parser.parse_args()
    if args.scene is not None:
        scene = args.scene.expanduser().resolve()
        if not scene.is_file():
            parser.error(f"장면 USD 파일이 없습니다: {scene}")
        return scene
    for scene in DEFAULT_SCENES:
        if scene.is_file():
            return scene
    parser.error("pick_place_cube.usd를 찾지 못했습니다. 실행 인자로 장면 USD 경로를 지정하세요.")


SCENE_USD = parse_scene_path()

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
import omni.graph.core as og
import omni.usd
from pxr import Sdf, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

GRAPH_PATH = "/World/Graph/camera_graph"  # GUI 강의와 같은 위치
ROBOT_PRIM = "/World/m0609_rg2_d455"
CAMERA_ROOT = f"{ROBOT_PRIM}/angle_bracket/realsense_d455/RSD455"
CAMERA_PRIM = f"{CAMERA_ROOT}/Camera_OmniVision_OV9782_Color"
LEFT_CAMERA_PRIM = f"{CAMERA_ROOT}/Camera_OmniVision_OV9782_Left"
WIDTH, HEIGHT = 640, 640  # 발행할 영상 크기 (픽셀)
# WIDTH, HEIGHT = 320, 320  # 실습: 영상 크기 바꾸기
FRAME_ID = "Camera"       # 기존 컬러 영상, depth, camera_info의 frame_id
LEFT_FRAME_ID = "Camera_Left"
CAMERA_LOOK_DEG = [0, 0, 90, 0, 90, 0]  # 카메라가 바닥의 큐브를 내려다보는 팔 자세 (joint_3, joint_5 = 90 deg)

# (가) ROS2 Bridge 확장 켜기: ROS2 노드를 쓰기 전에 켠다
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# (나) 작업 씬 열기: 로봇, 바닥, 큐브가 들어 있는 기본 씬
if not open_stage(str(SCENE_USD)):
    simulation_app.close()
    raise RuntimeError(f"장면을 열 수 없습니다: {SCENE_USD}")

stage = omni.usd.get_context().get_stage()
for camera_path in (CAMERA_PRIM, LEFT_CAMERA_PRIM):
    prim = stage.GetPrimAtPath(camera_path)
    if not prim or not prim.IsA(UsdGeom.Camera):
        simulation_app.close()
        raise RuntimeError(f"장면에서 Camera prim을 찾지 못했습니다: {camera_path}")

world = World(stage_units_in_meters=1.0)
robot = world.scene.add(SingleArticulation(prim_path=ROBOT_PRIM, name="m0609"))

# (다) Camera Graph 만들기: GUI 의 Tools > Robotics > ROS 2 OmniGraphs > Camera Graph에 왼쪽 카메라용 Render Product와 Helper를 추가
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},  # execution = Action Graph
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),                       # Play 중 매 프레임 실행 신호
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),             # 첫 프레임에만 다음 노드 실행
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),            # 카메라의 렌더 결과 만들기
            ("LeftRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("LeftRGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),                              # ROS2 통신 설정 (Domain ID)
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),           # /camera_info 발행
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),                      # /rgb 발행
            ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),                    # /depth 발행
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),                     # 실행 신호 -> 첫 프레임만
            ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),                      # 첫 프레임에 렌더 결과 만들기
            ("RunOnce.outputs:step", "LeftRenderProduct.inputs:execIn"),
            ("LeftRenderProduct.outputs:execOut", "LeftRGBPublish.inputs:execIn"),
            ("LeftRenderProduct.outputs:renderProductPath", "LeftRGBPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "LeftRGBPublish.inputs:context"),
            ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),         # 렌더 결과를 만든 뒤 발행 노드 실행
            ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),  # 어느 렌더 결과를
            ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),               # 통신 설정
            ("Context.outputs:context", "RGBPublish.inputs:context"),
            ("Context.outputs:context", "DepthPublish.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("Context.inputs:useDomainIDEnvVar", True),
            ("RenderProduct.inputs:cameraPrim", [Sdf.Path(CAMERA_PRIM)]),
            ("LeftRenderProduct.inputs:cameraPrim", [Sdf.Path(LEFT_CAMERA_PRIM)]),
            ("LeftRenderProduct.inputs:width", WIDTH),
            ("LeftRenderProduct.inputs:height", HEIGHT),
            ("LeftRGBPublish.inputs:type", "rgb"),
            ("LeftRGBPublish.inputs:topicName", "rgb_left"),
            ("LeftRGBPublish.inputs:frameId", LEFT_FRAME_ID),
            ("RenderProduct.inputs:width", WIDTH),                                        # 영상 크기
            ("RenderProduct.inputs:height", HEIGHT),
            ("CameraInfoPublish.inputs:topicName", "camera_info"),                        # 토픽 이름
            ("CameraInfoPublish.inputs:frameId", FRAME_ID),
            ("RGBPublish.inputs:type", "rgb"),                                            # 발행할 데이터 종류
            ("RGBPublish.inputs:topicName", "rgb"),
            ("RGBPublish.inputs:frameId", FRAME_ID),
            ("DepthPublish.inputs:type", "depth"),
            ("DepthPublish.inputs:topicName", "depth"),
            ("DepthPublish.inputs:frameId", FRAME_ID),
            # ("DepthPublish.inputs:frameSkipCount", 1),  # 실습: /depth 를 2 프레임에 한 번 발행
        ],
    },
)
print("Camera Graph 생성:", GRAPH_PATH)

# (라) 카메라가 큐브를 내려다보는 자세로 순간이동하고, 멈춘 상태로 기다린다
world.reset()
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]  # 팔 관절 6개가 관절 배열의 몇 번째인지
pose = robot.get_joint_positions()
pose[arm] = np.radians(CAMERA_LOOK_DEG)
robot.set_joint_positions(pose)
world.step(render=True)  # 한 프레임 진행해서 순간이동한 자세를 화면에 반영
world.pause()            # 화면 왼쪽의 Play(▶) 를 누르면 영상 발행 시작
print("Play 를 누르면 /rgb, /rgb_left, /depth, /camera_info 를 발행한다")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
while simulation_app.is_running():
    world.step(render=True)

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
