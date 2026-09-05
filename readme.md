# KAIST Mobility Challenge

## Project Overview

TEAM AIM의 KAIST Mobility Challenge 자율주행 프로젝트입니다. ROS2(Foxy) 기반으로 CAV(Connected
Autonomous Vehicle)의 경로 추종, 다중 차량 협력 주행, 차선 변경, 원형 교차로 통과를 순차적으로
구현하며 Mission 1-1부터 Mission 3까지 4단계로 발전했습니다.

### youtube 실제 주행 영상
- https://youtu.be/1H9E8ckReM0?si=kfsPTFC-xzXFp3OD

---

## Mission Evolution

각 Mission은 대회 진행 과정에서 로직이 발전한 별도의 단계이며, Mission마다 독립된 ROS2 패키지로
구현되어 있습니다(공통 라이브러리로 통합하지 않고 각 단계의 구현을 그대로 보존).

```
Mission 1-1
단일 CAV의 기본 경로 추종 (Pure Pursuit, 5랩 완주)

  ↓

Mission 1-2
CAV 2대 + Control Tower 협력 주행 (ROI 기반 충돌 회피, Red Flag)

  ↓

Mission 2
3차선 도로에서 CAV 1대 + HV 2대 협력 주행 (차선 변경, Zone 기반 충돌 회피)

  ↓

Mission 3
CAV 최대 4대 + Control Tower + Control Rotary (원형 교차로 통과, Yellow/Red Flag)
```

| Mission | CAV 수 | HV 수 | 협력 노드 | 핵심 로직 |
|---|---|---|---|---|
| 1-1 | 1 | 0 | 없음 | Pure Pursuit 경로 추종 |
| 1-2 | 2 | 0 | Control Tower | ROI 기반 교차로 충돌 회피 |
| 2 | 1 | 2 | 없음 (CAV 노드가 직접 판단) | 3차선 차선 변경, Zone 충돌 회피 |
| 3 | 최대 4 | 2 | Control Tower, Control Rotary | 다중 교차로 + 원형 교차로 통과 |

각 Mission의 상세한 목표, ROS2 토픽, Planning/Control 로직은 아래 Mission별 README를 참고하세요.

- [`src/mission_1_1/README.md`](src/mission_1_1/README.md)
- [`src/mission_1_2/README.md`](src/mission_1_2/README.md)
- [`src/mission_2/README.md`](src/mission_2/README.md)
- [`src/mission_3/README.md`](src/mission_3/README.md)

---

## Repository Structure

```
KAIST_Mobility_Challenge/
├── src/
│   ├── global_path/          # 전역 경로 CSV (모든 Mission이 공유)
│   ├── mission_1_1/          # Mission 1-1 패키지
│   ├── mission_1_2/          # Mission 1-2 패키지
│   ├── mission_2/            # Mission 2 패키지
│   └── mission_3/            # Mission 3 패키지
├── docker_kaist_aim/         # 개발용 Docker 이미지 (ROS2 Foxy)
├── Mobility_Challenge_Simulator/  # 시뮬레이터 (서브모듈)
├── entrypoint.sh             # PROBLEM_ID/ROLE 기반 노드 실행 진입점
├── mission_1_1.sh ~ mission_3.sh  # Mission별 실행 스크립트
└── mission_domain_*.sh       # ROS_DOMAIN_ID 설정 스크립트
```

각 Mission 패키지는 Study_ITS 스타일의 기능별 모듈 구조를 따릅니다(모든 executable이 이 구조를
갖는 것은 아니며, Mission이 발전하며 실제로 필요했던 모듈만 존재합니다):

```
mission_x/src/
├── <executable>.cpp   # ROS2 초기화, Pub/Sub 설정, Process 함수 호출 흐름만 담당
├── Global/             # 공유 자료구조(struct), 전역 상태
├── Utils/              # 범용 헬퍼 (거리/각도 계산, CSV 로드 등)
├── Planning/           # 경로 탐색, 목표 Waypoint, 차선 선택
├── Control/            # 목표 속도 계획, Pure Pursuit, 명령 생성
├── Mission/            # Zone/ROI 판단, Lap/완주 판정 (Mission-specific)
├── Tower/              # Control Tower 노드의 충돌 회피 판단 (Mission 1-2, 3)
├── Rotary/             # Control Rotary 노드의 ROI 통행 허가 판단 (Mission 3)
└── Visualizer/         # RViz 시각화 (Tower/Rotary 노드)
```

Mission 1-2와 Mission 3는 CAV/Tower/Rotary/check_motor가 각각 독립된 실행 파일이므로,
`src/cav/`, `src/tower/`, `src/rotary/`, `src/check_motor/` 하위에 위 모듈 트리를 각자 따로 둡니다.

---

## ROS2 Architecture

- 모든 Mission은 `ROS_DOMAIN_ID=100` 도메인을 사용합니다(entrypoint.sh 기준).
- CAV 노드는 `CAV_ID`(자신의 ID), Mission 3부터는 추가로 `CAV_IDS`(참여 중인 CAV ID 목록,
  콤마 구분) 환경변수로 자신의 경로/역할을 결정합니다.
- 경로 CSV는 `TEAM_AIM_HOME` 환경변수 기준 `${TEAM_AIM_HOME}/src/global_path/`에서 로드합니다
  (기본값은 노드마다 `/root/TEAM_AIM` 또는 `/home/aim/TEAM_AIM`).
- Mission 1-2, 3의 Control Tower/Rotary 노드는 `RED_FLAG`(정지), `YELLOW_FLAG`(감속, Mission 3),
  `target_vel`(목표 속도 지정) 토픽으로 CAV 노드를 제어합니다.

## Algorithm

- **Pure Pursuit**: 모든 Mission의 CAV 경로 추종 알고리즘. 목표 Waypoint와의 곡률(kappa)을 계산해
  각속도로 변환합니다.
- **Zone/ROI 기반 충돌 회피**: Control Tower가 CAV들의 위치를 모니터링하여 Precollision/Imminent
  Zone 진입 여부를 판단하고 RED_FLAG를 발행합니다.
- **차선 변경 (Mission 2)**: 3개 차선의 경로를 미리 로드해두고, Zone 충돌 상태에 따라
  `choose_lane()`으로 다음 차선을 결정합니다.
- **원형 교차로 통행 허가 (Mission 3)**: Control Rotary가 CAV ROI와 HV ROI를 짝지어 HV 도착
  여부에 따라 CAV의 통행을 허가(GO)하거나 정지(STOP)시킵니다.

---

## How to Run

모든 미션은 도메인 ID 100으로 설정되어 있습니다.
각 미션별 shell script를 실행하면 시뮬레이터부터 도메인까지 모든 것이 자동으로 설정됩니다.

### 시뮬레이터 재설치
```bash
./start_TEAM_AIM.sh
```

### Mission 실행 방법

```bash
./mission_1_1.sh
./mission_1_2.sh
./mission_2.sh
./mission_3.sh
```

### 호스트에서 ROS2 토픽 확인 방법

#### 1단계: 환경 변수 설정

호스트 터미널에서 다음 환경 변수를 설정합니다:

```bash
# ROS Domain ID 설정 (도메인 ID 100 사용)
export ROS_DOMAIN_ID=100

# Localhost 통신 비활성화 (다른 호스트와 통신하기 위함)
export ROS_LOCALHOST_ONLY=0

# RMW (ROS Middleware) 구현 설정
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

#### 2단계: 토픽 확인

```bash
ros2 topic list
```

### 각 미션별 토픽 확인 체크리스트

#### Mission 1-1
1. 터미널 1 (in container)에서 실행:
   ```bash
   ./mission_1_1.sh
   ```

2. 새 터미널(터미널 2_in host)을 열어 환경 변수 설정 및 토픽 확인:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ros2 topic list
   ```

#### Mission 1-2
1. 터미널 1 (in container)에서 실행:
   ```bash
   ./mission_1_2.sh
   ```

2. 새 터미널(터미널 2_in host)을 열어 환경 변수 설정 및 토픽 확인:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ros2 topic list
   ```

#### Mission 2
1. 터미널 1 (in container)에서 실행:
   ```bash
   ./mission_2.sh
   ```
2. 새 터미널(터미널 2_in host)을 열어 환경 변수 설정 및 토픽 확인:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ros2 topic list
   ```

#### Mission 3
1. 터미널 1 (in container)에서 실행:
   ```bash
   ./mission_3.sh
   ```

2. 새 터미널(터미널 2_in host)을 열어 환경 변수 설정 및 토픽 확인:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ros2 topic list
   ```

### 주의사항

- **Domain ID**: 모든 미션은 도메인 ID 100을 사용합니다
- **호스트에서, 환경 변수는 새 터미널을 열 때마다 다시 설정해야 합니다**
- 호스트에서 토픽을 확인하려면 반드시 위의 환경 변수 3가지를 설정해야 합니다. 그래야만 컨테이너와 호스트간 토픽 확인이 가능해집니다.

---

## Development History

- 대회 진행 중 Mission 1-1 → 1-2 → 2 → 3 순서로 기능이 확장되며 각 단계의 코드가 그대로
  누적되어 있습니다(Mission별 독립 패키지 구조).
- `refactor/project-structure` 브랜치에서 각 Mission 패키지 내부를 Study_ITS 스타일의
  Global/Utils/Planning/Control/Mission(+Tower/Rotary/Visualizer) 모듈 구조로 재구성했습니다.
  기존 대회 최종 코드의 알고리즘, ROS 토픽, 파라미터, 환경변수, executable 이름은 모두
  그대로 보존했습니다.
