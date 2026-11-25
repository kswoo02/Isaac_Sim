# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCone, VisualCone
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# [1번] 원뿔 두 개 추가 (뾰족한 끝이 처음엔 +z)
red = world.scene.add(
    DynamicCone(
        prim_path="/World/Red", name="red", position=np.array([0.4, 0.0, 0.5]),
        orientation=euler_angles_to_quat(np.array([60.0, 0.0, 0.0]), degrees=True),  # x 축 60도
        radius=0.05, height=0.15, color=np.array([1.0, 0, 0]),
    )
)
yellow = world.scene.add(
    VisualCone(
        prim_path="/World/Yellow", name="yellow", position=np.array([0.4, 0.0, 0.8]),
        radius=0.05, height=0.15, color=np.array([1.0, 1.0, 0]),
    )
)
world.reset()

OFFSET = np.array([0.0, 0.0, 0.3])

# ============================================================
# 3. 메인 루프
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1

    # [2번] 매 프레임 yellow 를 red 의 0.3 m 위, 같은 자세로
    red_position, red_orientation = red.get_world_pose()
    yellow.set_world_pose(position=red_position + OFFSET, orientation=red_orientation)

    # [3번] 60 프레임마다 red 의 위치와 각도(deg)
    if frame % 60 == 0:
        red_angles = quat_to_euler_angles(red_orientation, degrees=True)
        print(f"{frame} 프레임 | red 위치 {np.round(red_position, 3)} | 각도(deg) {np.round(red_angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()