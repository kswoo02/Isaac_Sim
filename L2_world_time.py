"""[L2 예제] World 와 프레임 시간 + 연습 문제

실행: isaac_python L2_world_time.py   (창을 닫으면 종료)
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
from isaacsim.core.api.objects import DynamicCuboid

RENDERING_DT = 1.0 / 60.0  # 한 프레임의 시간 (초)

# 연습 1: 2초가 몇 프레임인지 RENDERING_DT 로 계산
FRAME_2SEC = int(round(2.0 / RENDERING_DT))  # 120 프레임

# 장면 구성: World 생성 -> 물체 추가 -> reset
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDERING_DT)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube", position=np.array([0.0, 0.0, 1.0]), size=0.1)
)
world.reset()  # 물체를 다 넣은 뒤 한 번: 물리를 시작할 준비

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)  # 한 프레임(1/60 초) 진행
    frame += 1

    # 연습 1: 2초 뒤 큐브 높이
    if frame == FRAME_2SEC:
        height = cube.get_world_pose()[0][2]
        print(f"{frame} 프레임 ({frame * RENDERING_DT:.1f} 초): 큐브 높이 {height:.3f} m")

    # 연습 2: 180 프레임에서 reset
    if frame == 180:
        height = cube.get_world_pose()[0][2]
        print(f"리셋 전 큐브 높이 {height:.3f} m")
        world.reset()
        height = cube.get_world_pose()[0][2]
        print(f"리셋 후 큐브 높이 {height:.3f} m")

    # 연습 3: 300 프레임에서 큐브만 옮기기
    if frame == 300:
        cube.set_world_pose(position=np.array([0.3, 0.0, 1.0]))
        print("300 프레임: 큐브를 (0.3, 0.0, 1.0) 으로 옮김")

    if frame == 360:
        position = cube.get_world_pose()[0]
        print(f"360 프레임: 큐브 위치 {np.round(position, 3)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
