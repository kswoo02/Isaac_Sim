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

np.set_printoptions(precision=3, suppress=True)

RENDERING_DT = 1.0 / 60.0  # 한 프레임의 시간 (초)

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDERING_DT)
world.scene.add_default_ground_plane()
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube", position=np.array([0.0, 0.0, 1.0]), size=0.1)
)
world.reset()

# [1번] 2초가 몇 프레임인지 계산
TWO_SEC_FRAME = round(2.0 / RENDERING_DT)  # 120
print(f"2초는 {TWO_SEC_FRAME} 프레임")

RESET_FRAME = 180
MOVE_FRAME = 300
CHECK_FRAME = 360

# ============================================================
# 3. 메인 루프
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1

    # [1번] 2초 뒤 큐브 높이
    if frame == TWO_SEC_FRAME:
        height = cube.get_world_pose()[0][2]
        print(f"{frame} 프레임 ({frame * RENDERING_DT:.1f} 초): 큐브 높이 {height:.3f} m")

    # [2번] 루프 도중 reset
    if frame == RESET_FRAME:
        print(f"reset 전 큐브 높이 {cube.get_world_pose()[0][2]:.3f} m")
        world.reset()
        print(f"reset 후 큐브 높이 {cube.get_world_pose()[0][2]:.3f} m")

    # [3번] 큐브만 옮기기
    if frame == MOVE_FRAME:
        cube.set_world_pose(position=np.array([0.3, 0.0, 1.0]))
        print("큐브를 (0.3, 0, 1.0) 으로 옮김")

    if frame == CHECK_FRAME:
        print(f"옮긴 뒤 60 프레임: 큐브 위치 {cube.get_world_pose()[0]}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()