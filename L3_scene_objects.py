"""[L3 예제] 장면 객체: 추가, 조회, 이동 + 연습 문제

실행: isaac_python L3_scene_objects.py
"""

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

# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# 연습 1: 원뿔 두 개 추가
# x 축으로 60 deg 기울인 자세
tilt = euler_angles_to_quat(np.array([60.0, 0.0, 0.0]), degrees=True)

# red: 물리 적용 (떨어지고 넘어짐)
red = world.scene.add(
    DynamicCone(
        prim_path="/World/RedCone", name="red",
        position=np.array([0.4, 0.8, 0.5]), orientation=tilt,
        radius=0.05, height=0.15, color=np.array([1.0, 0.0, 0.0]),
    )
)
# yellow: 보이기만 함 (공중에 그대로)
yellow = world.scene.add(
    VisualCone(
        prim_path="/World/YellowCone", name="yellow",
        position=np.array([0.4, 0.0, 0.8]), orientation=tilt,
        radius=0.05, height=0.15, color=np.array([1.0, 1.0, 0.0]),
    )
)
world.reset()

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1

    # 연습 2: yellow 가 red 보다 0.3 m 위에서 같은 자세로 따라감
    red_position, red_orientation = red.get_world_pose()
    yellow.set_world_pose(
        position=red_position + np.array([0.0, 0.0, 0.3]),
        orientation=red_orientation,
    )

    # 연습 3: 60 프레임마다 red 의 위치와 각도(deg) 출력
    if frame % 60 == 0:
        red_angles = quat_to_euler_angles(red_orientation, degrees=True)
        print(f"{frame} 프레임 | red 위치 {np.round(red_position, 3)} | 각도(deg) {np.round(red_angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
