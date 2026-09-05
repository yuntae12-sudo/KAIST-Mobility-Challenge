# Mission 1-1: 기본 주행 제어 (Basic Vehicle Trajectory Tracking)

## 📋 개요

**Mission 1-1**은 단일 자동주행 차량(CAV, Connected Autonomous Vehicle)이 사전 정의된 경로를 따라 안정적으로 주행하는 기본 미션입니다. Pure Pursuit 알고리즘을 기반으로 경로 추적 제어를 수행하며, 5바퀴 주행을 완료하는 것을 목표로 합니다.

---

## 🎯 미션 목표

- **경로 추적**: 전역 경로(CSV 파일)를 따라 정확하게 주행
- **속도 제어**: 직선 구간은 2.0 m/s, 커브 구간은 1.7 m/s로 자동 조정
- **완주 감지**: 5바퀴 주행 완료 인식 및 정지
- **Red Flag 대응**: 타워로부터의 정지 신호(Red Flag)에 즉시 반응

---

## 🏗️ 주요 구조

### 📁 파일 구성
```
mission_1_1/
├── CMakeLists.txt           # CMake 빌드 설정
├── package.xml              # ROS2 패키지 메타데이터
└── src/
    ├── control_cav_mission_1_1.cpp  # ROS2 초기화 및 Pub/Sub, Process 함수 호출 흐름
    ├── Global/       # 공유 자료구조(ControllerState, CavState) 및 위치 전역 상태
    ├── Utils/        # 범용 헬퍼 (yaw 계산, CSV 로드, ID 변환 등)
    ├── Planning/     # Closest Point 탐색, Corner 판단, Lookahead/목표 Waypoint 계산
    ├── Control/      # 목표 속도 계획, Pure Pursuit, 제어 명령 생성
    └── Mission/       # Lap 카운트 및 5바퀴 완주 판정
```

### 📦 ROS2 노드

#### `control_cav_mission_1_1` 노드
자동주행 차량의 주행 제어를 담당하는 메인 노드입니다. main은 ROS2 초기화와 Pub/Sub만 담당하며,
Pose 콜백에서 `MissionProcess` → `PlanningProcess` → `DecideTargetSpeed` → `FindTargetWaypoint`
→ `ControlProcess` 순서로 각 모듈의 Process 함수를 호출합니다.

---

## 🔧 핵심 기능

### 1️⃣ **경로 로딩 및 관리**
- **경로 파일**: `/root/TEAM_AIM/src/global_path/path_mission1_01.csv`
- **포맷**: `x, y` 좌표 쌍의 CSV 파일
- **기능**: 차량이 따라야 할 웨이포인트 시퀀스

```cpp
struct integrate_path_struct {
  double x;
  double y;
};
static std::vector<integrate_path_struct> integrate_path_vector;
```

### 2️⃣ **Pure Pursuit 경로 추적 제어**

Pure Pursuit은 현재 위치에서 일정한 거리(Look-Ahead Distance, `Ld`) 앞의 웨이포인트를 목표로 하여 조향 명령을 계산합니다.

**주요 파라미터:**
- **`speed_mps`**: 기본 속도 (기본값: 0.5 m/s)
- **`lookahead_m`**: 선행거리 (기본값: 0.3 m)
- **`max_yaw_rate`**: 최대 회전 속도 (기본값: 5.5 rad/s)

**동적 선행거리 계산 (`GetLd` 함수):**
```cpp
double gain_ld = 0.4;          // 속도 게인
double max_ld  = 0.355;        // 최대 선행거리
double min_ld  = 0.15;         // 최소 선행거리
double ld = gain_ld * velocity;
```

### 3️⃣ **속도 계획 (Velocity Planning)**

코너 감지를 통해 속도를 동적으로 조정합니다:
- **직선 구간**: 2.0 m/s (빠른 주행)
- **커브 구간**: 1.7 m/s (안정적 회전)

```cpp
bool isCorner(const vector<integrate_path_struct>& path, double L_d, int closest_idx) {
  // 미래 20개 웨이포인트 앞의 각도 변화를 감지
  // 10도 이상의 각도 변화가 감지되면 커브로 판단
  double threshold_deg = 10.0;
}
```

### 4️⃣ **주행 완주 감지 (Lap/Finish Logic)**

차량이 시작점 주변에 돌아올 때마다 LAP 카운트를 증가시킵니다:

```cpp
struct CavState {
  int id;                 // CAV ID
  double start_x, start_y;  // 시작점
  int current_lap;        // 현재 랩 (0~5)
  bool is_initialized;    // 초기화 여부
  bool is_in_zone;        // 시작점 영역 내 여부
  bool finished;          // 완주 여부 (5랩 달성)
};
```

- 차량이 시작점으로부터 0.1m 이내로 진입하면 **LAP 증가**
- 5바퀴 완료 후 **자동 정지**

### 5️⃣ **Red Flag 대응**

타워(제어 센터)로부터의 정지 신호에 즉시 반응:

```
Red Flag = 1: 모든 명령 정지 (선속도 = 0, 각속도 = 0)
Red Flag = 0: 정상 제어 로직 실행
```

---

## 📡 ROS2 토픽

### 구독(Subscription)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01` | `geometry_msgs/msg/PoseStamped` | 차량의 자신의 위치/자세 정보 |
| `/CAV_01_RED_FLAG` | `std_msgs/msg/Int32` | 타워의 정지 신호 |
| `/CAV_01_target_vel` | `std_msgs/msg/Float64` | 타워의 목표 속도 명령 (음수: 타워 모드 해제) |

### 발행(Publication)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01_accel` | `geometry_msgs/msg/Accel` | 계산된 선속도 및 각속도 명령 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 차량 속도 명령 (선속도 + 각속도) |

---

## 🚀 파라미터

노드 실행 시 아래 파라미터로 제어 성능을 튜닝할 수 있습니다:

```bash
ros2 run mission_1_1 control_cav_mission_1_1 \
  --ros-args \
  -p speed_mps:=2.0 \
  -p lookahead_m:=0.4 \
  -p max_yaw_rate:=5.5
```

| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `speed_mps` | 0.5 | 0.1~5.0 | 기본 주행 속도 (m/s) |
| `lookahead_m` | 0.4 | 0.1~1.0 | Pure Pursuit 선행거리 (m) |
| `max_yaw_rate` | 5.5 | 1.0~10.0 | 최대 회전 속도 제한 (rad/s) |
| `cav_id` | 1 | 1~99 | CAV의 차량 ID |

---

## 💻 빌드 및 실행

### 빌드
```bash
cd /root/TEAM_AIM
colcon build --packages-select mission_1_1
source install/setup.bash
```

### 실행 (시뮬레이터와 함께)
```bash
# 터미널 1: 시뮬레이터 실행
ros2 launch simulator_launch simulator.launch.py

# 터미널 2: Mission 1-1 노드 실행
ros2 run mission_1_1 control_cav_mission_1_1
```

### 환경 변수로 CAV ID 설정
```bash
export CAV_ID=1
ros2 run mission_1_1 control_cav_mission_1_1
```

---

## 📊 제어 흐름도

```
1. 초기화
   ├─ 경로 CSV 로딩
   ├─ 시작점 설정
   └─ ROS2 Pub/Sub 설정

2. 메인 루프 (100 Hz)
   ├─ 현재 위치/자세 수신
   ├─ LAP 상태 업데이트
   ├─ 현재 위치에서 가장 가까운 웨이포인트 찾기
   ├─ 커널 검지 및 속도 계획
   ├─ Pure Pursuit 계산
   │  ├─ 선행거리(Ld) 동적 계산
   │  ├─ 목표 웨이포인트 탐색
   │  └─ 곡률(kappa) 계산 → 각속도(wz) 변환
   ├─ Red Flag 확인 후 명령 생성
   └─ 속도 및 회전 명령 발행

3. 종료
   ├─ 5LAP 완주 감지
   └─ 정지 명령 유지
```

---

## 🔍 주요 함수 설명

### `findClosestPoint()` (Planning)
현재 위치에서 경로 상의 가장 가까운 웨이포인트 인덱스를 반환합니다.

### `findWaypoint()` (Planning)
현재 위치로부터 선행거리(Ld) 이상 떨어진 첫 번째 웨이포인트를 찾습니다.

### `isCorner()` (Planning)
향후 20개 웨이포인트의 각도 변화를 분석하여 커널 여부를 판단합니다.

### `GetLd()` (Planning)
현재 속도를 기반으로 Pure Pursuit의 선행거리를 동적으로 계산합니다.

### `MissionProcess()` (Mission)
LAP 카운팅 로직을 관리합니다 (리팩토링 이전 이름: `PoseCallbackForLap()`).

---

## ⚙️ 튜닝 가이드

### 경로 추적이 진동하는 경우
- **문제**: `max_yaw_rate` 값이 너무 큼
- **해결**: `max_yaw_rate`를 3.0 ~ 4.0으로 낮춤

### 경로를 못 따라가는 경우
- **문제**: `lookahead_m`이 너무 크거나 작음
- **해결**: `lookahead_m`을 0.3 ~ 0.5 범위로 조정

### 커브에서 속도가 부족한 경우
- **문제**: 속도 계획의 커브 속도(1.7 m/s)가 과도하게 낮음
- **해결**: `planVelocity()` 함수에서 속도 값 조정

---

## 📝 주석 및 코드 분석

### Pure Pursuit 핵심 수식
```
kappa = (2 * sin(heading_error)) / lookahead_distance
angular_velocity = linear_velocity * kappa
```

여기서:
- `heading_error`: 차량과 목표 웨이포인트 간의 각도 오차
- `lookahead_distance`: Pure Pursuit의 선행거리
- `angular_velocity`: 계산된 회전 속도 (rad/s)

---

## 📚 관련 ROS2 의존성

```xml
<build_depend>rclcpp</build_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>tf2</build_depend>
```

---

## 🐛 문제 해결

| 문제 | 원인 | 해결책 |
|------|------|--------|
| 경로 파일 로드 실패 | 경로 CSV 파일 없음 | `/root/TEAM_AIM/src/global_path/` 확인 |
| 차량이 움직이지 않음 | 시뮬레이터 미실행 또는 토픽 오류 | `ros2 topic list` 확인 |
| 과도한 회전 | `max_yaw_rate` 과대 | 값 감소 |
| 경로 추적 오차 큼 | `lookahead_m` 부적절 | 0.3~0.5 범위로 조정 |

---

## 📄 라이선스 및 저작권

TEAM AIM 프로젝트의 일부입니다.

---

## 👥 기여자

- KAIST 자율주행 팀

---

*Last Updated: 2026-02-02*
