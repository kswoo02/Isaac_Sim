"""[L1 예제] Isaac Sim standalone 의 가장 기본 골격 (4단계) + 연습 문제

실행: isaac_python L1_app.py   (300 프레임 후 자동 종료)
"""

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
from isaacsim.core.version import get_version
from isaacsim.core.api import World

print("Isaac Sim 버전:", get_version()[0])

world = World()
print("World 생성 성공")

# ============================================================
# 3. 메인 루프 (프레임 세기, 300 프레임 후 종료)
# ============================================================
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1

    if frame % 60 == 0:
        print(f"{frame} 프레임")

    if frame >= 300:
        print("300 프레임이 지나 종료합니다")
        break

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
