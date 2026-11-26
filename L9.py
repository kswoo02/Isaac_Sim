"""[L8 응용] Task와 Controller 구조로 빨간 타겟 실시간 IK 추종하기

실행: isaac_python L8_ik_follow.py
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
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# ============================================================
# 3. 파일 경로 및 로봇 상수 설정
# ============================================================
M0609_DIR = Path(__file__).resolve().parents[1]
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
TCP_FRAME = "gripper_center"

# 타겟 시작 자세: 그리퍼가 아래를 향하는 자세 (roll=180 deg, pitch=0 deg, yaw=0 deg)
DOWNWARD_ORIENT = euler_angles_to_quat(np.radians([180.0, 0.0, 0.0]))
TARGET_SIZE = 0.04  # 한 변 4 cm (0.04 m)


# ============================================================
# 4. Task: 장면(바닥, 로봇+열린 그리퍼, 빨간 타겟) 구성 및 관측
# ============================================================
class FollowTargetTask(BaseTask):
    """바닥, M0609+RG2(그리퍼 열림), 4cm 빨간 VisualCuboid 타겟을 구성하고 상태를 관측한다"""

    def __init__(self, name, initial_tcp_pos):
        super().__init__(name)
        self._initial_tcp_pos = initial_tcp_pos  # 홈 자세의 TCP 시작 위치

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        # 1) 로봇 USD 로드
        add_reference_to_stage(
            usd_path=ROBOT_USD,
            prim_path="/World/m0609_rg2_d455",
        )

        # 2) RG2 그리퍼 설정
        gripper = ParallelGripper(
            end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0.0]),
            joint_closed_positions=np.array([1.0]),
            use_mimic_joints=True,
        )

        # 3) 팔 + 그리퍼 통합 매니퓰레이터 등록
        self._robot = scene.add(
            SingleManipulator(
                prim_path="/World/m0609_rg2_d455",
                name="m0609",
                end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",
                gripper=gripper,
            )
        )

        # 4) 빨간 타겟 (VisualCuboid, 한 변 4 cm)
        #    시작 위치: 홈 자세의 TCP 위치, 시작 자세: 그리퍼가 아래를 향하는 자세
        self._target = scene.add(
            VisualCuboid(
                prim_path="/World/Target",
                name="target",
                position=self._initial_tcp_pos,
                orientation=DOWNWARD_ORIENT,
                scale=np.array([TARGET_SIZE] * 3),
                size=1.0,
                color=np.array([1.0, 0.0, 0.0]),
            )
        )

        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._target.name] = self._target

    def post_reset(self):
        # 팔은 홈 자세(HOME_DEG)로 초기화
        home = np.zeros(self._robot.num_dof)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        self._robot.set_joint_positions(home)

        # 그리퍼는 열린 상태(0.0 rad)로 초기화
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )

        # 타겟도 시작 위치(홈 TCP 위치)와 아래를 향하는 자세로 초기화
        self._target.set_world_pose(
            position=self._initial_tcp_pos,
            orientation=DOWNWARD_ORIENT,
        )

    def get_observations(self):
        target_pos, target_orient = self._target.get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            self._target.name: {
                "position": target_pos,
                "orientation": target_orient,
            },
        }

    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},
            "target_name": {"value": self._target.name},
        }


# ============================================================
# 5. Controller: 매 프레임 IK 계산 + 팔 형태 유지 + 실패 시 대기
# ============================================================
class IkFollowController(BaseController):
    """매 프레임 현재 자세 근처에서 IK를 풀고, 팔 형태가 뒤집히지 않도록 유지하며 추종한다"""

    def __init__(self, name, ik_solver, arm_indices):
        super().__init__(name)
        self._ik_solver = ik_solver
        self._arm_indices = arm_indices
        self.reset()

    def reset(self):
        # 기억하는 상태를 처음 값(홈 자세 명령, 추종 가능 상태)으로 되돌림
        self._last_action = ArticulationAction(
            joint_positions=np.radians(HOME_DEG),
            joint_indices=self._arm_indices,
        )
        self.is_following = True  # 메인 루프가 읽을 수 있는 상태 플래그 (쓸 답이 있었는지)

    def _is_valid_elbow_wrist_posture(self, joints_rad):
        """
        [팔 형태 유지 검사]
        홈 자세(joint_3 = +90 deg, joint_5 = +90 deg)와 같은 굽힘 방향을 유지해
        팔꿈치나 손목이 반대로 뒤집히지 않았는지 확인한다.
        """
        joint_3_rad = joints_rad[2]  # 팔꿈치 관절 (양수 방향 굽힘 유지)
        joint_5_rad = joints_rad[4]  # 손목 관절 (양수 방향 굽힘 유지)
        return (joint_3_rad > np.radians(5.0)) and (joint_5_rad > np.radians(5.0))

    def forward(self, current_arm, target_pos, target_orient):
        # 1) 매 프레임 지금 자세(current_arm) 근처에서 IK를 푼다 (warm_start=current_arm)
        joints, success = self._ik_solver.compute_inverse_kinematics(
            TCP_FRAME,
            target_pos,
            target_orient,
            warm_start=current_arm,
        )

        # 2) IK 해가 존재하고 + 팔 형태가 뒤집히지 않은 유효한 답인지 판정
        if success and self._is_valid_elbow_wrist_posture(joints):
            self.is_following = True
            self._last_action = ArticulationAction(
                joint_positions=joints,
                joint_indices=self._arm_indices,
            )
        else:
            # 3) 쓸 답이 없으면 마지막 명령을 그대로 보낸다 (그 자리에서 기다림)
            self.is_following = False

        return self._last_action


# ============================================================
# 6. IK 솔버 생성 및 홈 자세 TCP 시작 위치 계산
# ============================================================
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,
    urdf_path=URDF_PATH,
)

# 홈 자세(HOME_DEG)일 때의 TCP 위치 [x, y, z]를 정기구학(FK)으로 정확히 구함
home_tcp_pos, _ = ik_solver.compute_forward_kinematics(
    TCP_FRAME,
    np.radians(HOME_DEG),
)

# ============================================================
# 7. World, Task, Controller 등록 및 초기화
# ============================================================
world = World(
    stage_units_in_meters=1.0,
    physics_dt=1.0 / 120.0,
    rendering_dt=1.0 / 60.0,
)
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

task = FollowTargetTask(name="follow_target_task", initial_tcp_pos=home_tcp_pos)
world.add_task(task)
world.reset()

params = task.get_params()
robot_name = params["robot_name"]["value"]
target_name = params["target_name"]["value"]

robot = world.scene.get_object(robot_name)
arm_indices = [robot.get_dof_index(name) for name in ARM_JOINTS]

controller = IkFollowController(
    name="ik_follow_controller",
    ik_solver=ik_solver,
    arm_indices=arm_indices,
)

# ============================================================
# 8. 메인 루프 (실시간 타겟 추종)
# ============================================================
reset_needed = False
prev_following = None  # 추종 상태가 바뀔 때만 터미널에 알림 출력

while simulation_app.is_running():
    world.step(render=True)

    if world.is_stopped() and not reset_needed:
        reset_needed = True

    if world.is_playing():
        if reset_needed:
            world.reset()
            controller.reset()
            prev_following = None
            reset_needed = False
            print("리셋 완료: 홈 자세 및 초기 타겟 위치로 복귀")

        # 1) Task에서 현재 팔 각도와 빨간 타겟 위치·자세 관측
        obs = world.get_observations()
        current_arm = obs[robot_name]["joint_positions"][arm_indices]
        target_pos = obs[target_name]["position"]
        target_orient = obs[target_name]["orientation"]

        # 2) Controller가 매 프레임 IK 계산 후 명령 반환
        action = controller.forward(current_arm, target_pos, target_orient)

        # 3) 로봇 팔에 명령 적용
        if action is not None:
            robot.apply_action(action)

        # 4) 메인 루프에서 controller.is_following 상태를 읽어 상태 변화 출력
        if controller.is_following != prev_following:
            if controller.is_following:
                print("[상태] 타겟 추종 중 (유효한 IK 해로 이동)")
            else:
                print("[상태] 도달 불가 또는 팔 형태 뒤집힘 감지 -> 마지막 위치에서 대기")
            prev_following = controller.is_following

# ============================================================
# 9. 종료
# ============================================================
simulation_app.close()