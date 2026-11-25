"""[L4 연습] 고개 젓는 로봇: joint_1 을 좌우 45 deg 로 계속 왕복

실행: isaac_python L4_robot_joints.py
"""

from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
#   - SimulationApp : Isaac Sim 을 켠다. 다른 Isaac 모듈은 이 뒤에 import 해야 한다
#   - headless=False : 창을 띄운다 (True 이면 창 없이 실행)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
#   - numpy                  : 배열 계산, 각도 변환
#   - omni.kit.commands      : 메뉴 동작을 코드로 실행
#   - World                  : 시뮬레이션 세계 (시간, 물리, 물체 관리)
#   - SingleArticulation     : 관절 로봇 1대를 다루는 클래스
#   - add_reference_to_stage : USD 파일을 장면에 불러오기
#   - ArticulationAction     : 모터에 보내는 명령 형식
#   - set_camera_view        : 화면 카메라 위치 설정
# ============================================================
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view

# ============================================================
# (가) 장면과 로봇 준비
#   - ROBOT_USD : 이 파일 기준 parents[1] = M0609 폴더 -> M0609/robot/m0609_rg2_d455.usd
#   - World : 길이 단위 1.0 = 1 m, 이어서 기본 바닥을 깐다
#   - 조명 : 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
#   - 로봇 : USD 를 /World/m0609_rg2_d455 에 넣고, 관절 로봇으로 등록한다
#   - 카메라 : eye = 카메라 위치, target = 바라보는 점 (x, y, z, 단위 m)
#   - world.reset() : 물리를 초기화한다. reset 후에 관절 정보를 읽을 수 있다
# ============================================================
ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
set_camera_view(eye=[1.6, 1.6, 1.2], target=[0.0, 0.0, 0.4])
world.reset()

# ============================================================
# (나) 관절 정보
#   - num_dof : 움직이는 관절 수. M0609 팔 6 + RG2 그리퍼 6 = 12
#   - dof_names : 관절 12개의 이름. 번호와 함께 출력한다
#   - arm : joint_1 ~ joint_6 이 12칸 배열의 몇 번째 칸인지 -> [0, 1, 2, 3, 4, 5]
#   - finger : 그리퍼 관절의 칸 번호 -> 6
#     finger_joint 만 움직이면 나머지 손가락 관절은 따라온다 (mimic 관절)
# ============================================================
print("관절 개수:", robot.num_dof)
for index, name in enumerate(robot.dof_names):
    print(f"  {index:2d} {name}")
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]
finger = robot.get_dof_index("finger_joint")
print("arm (팔 관절 6개가 관절 배열의 몇 번째인지):", arm)
print("finger (그리퍼가 관절 배열의 몇 번째인지):", finger)

# ============================================================
# (다) 순간이동: 홈 자세, 그리퍼 열기
#   - start : 0 으로 채운 12칸 배열에 팔 자세와 그리퍼 값을 넣는다
#   - 팔 순서 : joint_1 ~ joint_6, 홈 자세는 joint_3 과 joint_5 만 90 deg
#   - 단위 : 코드는 rad 를 쓴다. np.radians 는 deg -> rad, np.degrees 는 rad -> deg
#   - set_joint_positions : 모터를 거치지 않고 관절값을 바로 바꾼다 (순간이동)
#     그래서 프레임이 지나지 않아도 바로 읽으면 이미 바뀌어 있다
# ============================================================
start = np.zeros(robot.num_dof)
start[arm] = np.radians([0, 0, 90, 0, 90, 0])
start[finger] = 0.0
print("순간이동 명령 start (관절 12개, rad):", np.round(start, 3).tolist())
robot.set_joint_positions(start)
q = robot.get_joint_positions()
print(f"순간이동 후 | joint_1 {np.degrees(q[arm[0]]):6.1f} deg | 그리퍼 {abs(q[finger]):.2f} rad")

# ============================================================
# 3-1. 설정값
#   - MOVE_FRAME : 첫 모터 목표를 보내는 프레임
#   - ARRIVE_DEG : joint_1 오차가 이 값 이하이면 도착 (deg)
#     한 프레임에 약 2.5 deg 움직이므로 절반인 1.25 보다 커야 건너뛰지 않는다
#   - GRIP_CLOSE : 왼쪽(-45 deg)으로 갈 때 그리퍼 닫기 (rad)
#   - GRIP_OPEN  : 오른쪽(45 deg)으로 갈 때 그리퍼 열기 (rad)
# ============================================================
MOVE_FRAME = 100
ARRIVE_DEG = 1.5
GRIP_CLOSE = 0.9
GRIP_OPEN = 0.0


# ============================================================
# 3-2. 모터 목표를 보내는 함수
#   - start 를 복사해 joint_1 만 goal_deg 로 바꾼다 (copy 를 해야 start 가 안 바뀐다)
#   - 목표가 음수(왼쪽)이면 그리퍼를 닫고, 아니면(오른쪽) 연다
#   - apply_action : 한 번만 보내도 모터가 여러 프레임에 걸쳐 목표까지 간다
# ============================================================
def move_to(goal_deg):
    """joint_1 을 goal_deg 로 보내는 모터 목표를 한 번 보낸다. 그리퍼는 방향에 따라 정한다."""
    target = start.copy()
    target[arm] = np.radians([goal_deg, 0, 90, 0, 90, 0])
    target[finger] = GRIP_CLOSE if goal_deg < 0 else GRIP_OPEN
    robot.apply_action(ArticulationAction(joint_positions=target))


# ============================================================
# 3-3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
#   - frame : 지금까지 진행한 프레임 수 / goal : 현재 목표 각도, 처음은 왼쪽(-45)
#   - world.step : 물리를 한 단계 계산하고 화면을 그린다. 이래야 로봇이 움직인다
#   - frame == MOVE_FRAME : 첫 출발 (그 전까지는 홈 자세 유지)
#   - 그 뒤로는 매 프레임 now(joint_1 현재 각도)와 goal 의 차이를 확인한다
#     차이가 ARRIVE_DEG 이하이면 도착 -> 출력하고 goal 부호를 뒤집어 반대쪽으로 출발
#   - 도착 판정은 joint_1 만 본다 (그리퍼가 다 움직였는지는 확인하지 않는다)
# ============================================================
frame = 0
goal = -45.0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    if frame == MOVE_FRAME:
        move_to(goal)
    elif frame > MOVE_FRAME:
        now = np.degrees(robot.get_joint_positions()[arm[0]])
        if abs(now - goal) <= ARRIVE_DEG:
            print(f"{frame} 프레임 | joint_1 {now:6.1f} deg, {goal:.0f} deg 도착")
            goal = -goal
            move_to(goal)

# ============================================================
# 4. 종료
#   - 창을 닫으면 루프가 끝나고, 앱을 정리한다
# ============================================================
simulation_app.close()
