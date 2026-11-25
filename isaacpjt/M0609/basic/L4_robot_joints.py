"""[L4 예제] M0609 로봇 불러오기와 관절 제어

실행: isaac_python L4_robot_joints.py
"""

from pathlib import Path

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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"  # isaacpjt/M0609/robot

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# (가) 로봇 USD 참조: 파일의 기본 prim(로봇)을 /World/m0609_rg2_d455 에 넣는다
add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
world.reset()  # reset 후에 관절 정보를 읽을 수 있다

# (나) 관절 정보
print("관절 개수:", robot.num_dof)
for index, name in enumerate(robot.dof_names):
    print(f"  {index:2d} {name}")
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]
finger = robot.get_dof_index("finger_joint")
print("arm (팔 관절 6개가 관절 배열의 몇 번째인지):", arm)
print("finger (그리퍼가 관절 배열의 몇 번째인지):", finger)

# (다) 순간이동: joint_1 45 deg, 그리퍼 열기
start = np.zeros(robot.num_dof)
start[arm] = np.radians([45, 0, 90, 0, 90, 0])  # 팔
start[finger] = 0.0                             # 그리퍼 열기 (0.0 rad)
print("순간이동 명령 start (관절 12개, rad):", np.round(start, 3).tolist())
robot.set_joint_positions(start)
q = robot.get_joint_positions()
print(f"순간이동 후 | joint_1 {np.degrees(q[arm[0]]):6.1f} deg | 그리퍼 {abs(q[finger]):.2f} rad")


# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
MOVE_FRAME = 45   # 이 프레임부터 왕복 시작
MOVE_FRAMES = 60  # 한 방향 이동에 걸리는 프레임 수 (늘리면 느려짐)

frame = 0
go_left = True
to_pose = start.copy()
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    if frame >= MOVE_FRAME:
        t = (frame - MOVE_FRAME) % MOVE_FRAMES + 1   # 구간 안에서 1 → MOVE_FRAMES
        if t == 1:                                   # 구간 시작: 새 목표 정하기
            from_pose = to_pose
            to_pose = start.copy()
            if go_left:   # 왼쪽으로 갈 때: 그리퍼 닫기
                to_pose[arm] = np.radians([-45, 0, 90, 0, 90, 0])
                to_pose[finger] = 0.9
            else:         # 오른쪽으로 갈 때: 그리퍼 열기
                to_pose[arm] = np.radians([45, 0, 90, 0, 90, 0])
                to_pose[finger] = 0.0
            go_left = not go_left
        target = from_pose + (to_pose - from_pose) * (t / MOVE_FRAMES)  # 조금씩 이동
        robot.apply_action(ArticulationAction(joint_positions=target))

        q = robot.get_joint_positions()
        print(f"{frame} 프레임 | joint_1 {np.degrees(q[arm[0]]):6.1f} deg | 그리퍼 {abs(q[finger]):.2f} rad")
# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
