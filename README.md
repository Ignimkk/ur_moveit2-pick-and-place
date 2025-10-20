# UR Pick and Place - 모듈화된 시스템

이 패키지는 UR 로봇을 위한 모듈화된 Pick & Place 시스템입니다. 기존의 하드코딩된 단일 파일 구조에서 ROS2의 모듈화된 아키텍처로 리팩토링되었습니다.

## 최신 업데이트 (v3.0)

### 🎯 주요 개선사항
- **Gazebo 제거**: 물리 시뮬레이션 없이 Fake Hardware로 경량 실행
- **통합 런치 파일**: `fake_hardware_pick_and_place.launch.py`로 모든 것을 한 번에 실행
- **RViz 시각화**: Gazebo 대신 RViz만으로 로봇 동작 확인
- **빠른 개발/테스트**: 시뮬레이션 부하 없이 빠른 응답 속도
- **표준 UR 패키지 활용**: `ur_bringup`과 `ur_moveit_config` 사용 가능

### 📜 이전 버전 주요 기능
- **v2.1**: Place 위치 하드코딩, Pick only 입력
- **v2.0**: 실물 로봇 지원, Ready Position 노드, 안전 데모 스크립트
- **v1.1**: 단일 파일 빌드 구조, 자동 시퀀스 관리
- **v1.0**: 초기 모듈화 구조, 토픽 기반 좌표

## 시스템 구조

### 노드 구성

| 기능 | 노드명 | 통신 방식 | 역할 |
|------|--------|-----------|------|
| 목표 pose 수신 | `goal_receiver_node` | 토픽 (`/pick_goal`, `/place_goal`) | 외부에서 위치 명령을 수신하고 시퀀스 관리 |
| Ready 위치 이동 | `ready_executor_node` | 서비스 (`/ready_service`) | 로봇을 안전한 준비 위치로 이동 |
| Pick 동작 수행 | `pick_executor_node` | 액션 (`pick_action`) | Pick 작업 전문 수행 |
| Place 동작 수행 | `place_executor_node` | 액션 (`place_action`) | Place 작업 전문 수행 |
| Gripper 제어 | `gripper_controller_node` | 서비스 (`/gripper/control`) | 서보 또는 IO 제어 |
| 전체 통합 | `pick_place_manager_node` | 액션 클라이언트 | Pick 완료 후 자동으로 Place 실행하는 시퀀스 관리 |

### 통신 구조 (v3.0)

```
외부 시스템 → Pick 위치만 전송
    ↓ (/pick_goal)
goal_receiver_node
    ↓ (내부 토픽: /internal/pick_goal)
pick_place_manager_node (Place 위치 하드코딩)
    ↓ (Pick 액션 → Pick 완료 → Place 액션)
pick_executor_node ←─────→ place_executor_node
    ↓ (서비스: /gripper/control)
gripper_controller_node

별도 서비스:
ready_executor_node (/ready_service) → 안전한 준비 위치로 이동

하드웨어 인터페이스:
Fake Hardware (ur_bringup) → ros2_control → MoveIt
```

**주요 변경점**: 
- Gazebo 제거, Fake Hardware 사용
- Place goal 토픽 제거 (하드코딩)

### 🔄 자동 시퀀스 플로우 (v3.0)

1. **목표 설정**: Pick 좌표만 토픽으로 전송 (Place는 자동 설정)
2. **즉시 실행**: Pick goal 수신 즉시 manager가 시퀀스 시작
3. **Ready 위치**: 안전한 준비 위치로 먼저 이동
4. **Pick 실행**: Pick executor가 지정된 위치에서 물체 집기
5. **자동 Place**: Pick 완료 시 하드코딩된 위치로 자동 이동하여 놓기
6. **완료**: Place 완료 후 Ready 위치로 복귀

**⚠️ 중요**: 
- Place 위치는 하드코딩 (x=-0.340, y=0.310, z=0.264)
- **Gazebo 불필요**: Fake Hardware로 가볍게 실행

## 실행 방법

### 🎯 실행 환경 선택 가이드 (v3.0)

| 환경 | 사용 시나리오 | 시뮬레이션 | 실행 방법 |
|------|-------------|-----------|----------|
| **Fake Hardware** ⭐ | 개발/테스트, 빠른 검증 | ❌ Gazebo 불필요 | 통합 또는 분리 실행 |
| **Gazebo** | 물리 시뮬레이션 필요 | ✅ 물리 엔진 | 분리 실행 (무거움) |
| **실물 로봇** | 실제 하드웨어 배포 | ❌ 시뮬레이션 없음 | 분리 실행 |

### 1. Fake Hardware 환경 (권장 ⭐ - Gazebo 없이 빠르고 가벼움)

#### 방법 A: 통합 런치 파일 (가장 간단)
```bash
# 모든 것을 한 번에 실행 (UR Control + MoveIt + Pick and Place)
ros2 launch ur_pick_and_place fake_hardware_pick_and_place.launch.py

# RViz 없이 실행
ros2 launch ur_pick_and_place fake_hardware_pick_and_place.launch.py launch_rviz:=false

# 자동 테스트 실행
ros2 run ur_pick_and_place test_modular_system.py
```

#### 방법 B: 표준 UR 패키지 사용 (3개 터미널 분리 실행)
```bash
# 터미널 1: UR Control with Fake Hardware
ros2 launch ur_bringup ur5e.launch.py \
    robot_ip:=xxx.yyy.zzz.www \
    use_fake_hardware:=true \
    fake_sensor_commands:=true

# 터미널 2: MoveIt with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=false

# 터미널 3: Pick and Place 노드들
ros2 launch ur_pick_and_place modular_pick_and_place.launch.py use_sim_time:=false
```

**장점**:
- ✅ **Gazebo 불필요** - 시스템 부하 최소화
- ✅ RViz로 로봇 동작 시각화
- ✅ 모든 pick and place 기능 동일하게 작동
- ✅ 빠른 시작 및 응답 속도 (Gazebo 대비 10배 이상 빠름)
- ✅ 표준 UR 패키지와 호환

### 2. Gazebo 시뮬레이션 환경 (선택사항 - 물리 시뮬레이션 필요시만)

**⚠️ 주의**: 대부분의 경우 Fake Hardware(방법 1)로 충분합니다. Gazebo는 물리 엔진이 필요한 경우에만 사용하세요.

```bash
# 터미널 1: UR Gazebo 시뮬레이션 환경 시작
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py

# 터미널 2: Pick and Place 노드들 실행 (시뮬레이션 모드)
ros2 launch ur_pick_and_place modular_pick_and_place.launch.py use_sim_time:=true

# 터미널 3: 자동 테스트 실행
ros2 run ur_pick_and_place test_modular_system.py
```

**단점**:
- ❌ 무거운 실행 (Gazebo 물리 엔진)
- ❌ 느린 시작 시간 (30초 이상)
- ❌ 높은 시스템 리소스 사용

### 3. 실물 로봇 환경 실행

#### 🤖 실물 로봇 연결 요구사항
- UR5e 로봇 (또는 다른 UR 시리즈)
- 네트워크 연결 (예: 로봇 IP `192.168.1.102`)
- URCap External Control 설치 (headless_mode=false인 경우)

#### 실행 순서

```bash
# 터미널 1: 실물 로봇 드라이버 실행
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.1.102 \
    headless_mode:=false

# 터미널 2: MoveIt 계획 실행 (실물 모드)
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    use_sim_time:=false

# 터미널 3: Pick and Place 노드들 실행 (실물 모드)
ros2 launch ur_pick_and_place modular_pick_and_place.launch.py use_sim_time:=false

# 터미널 4: 안전한 실물 로봇 데모 실행 ⚠️ 안전 주의!
ros2 run ur_pick_and_place real_robot_demo.py
```

#### ⚠️ 실물 로봇 안전 사항
- **작업 공간 확인**: 로봇 주변에 장애물이 없는지 확인
- **비상정지 준비**: 언제든 Emergency Stop 버튼을 누를 수 있도록 준비  
- **보수적 좌표**: 처음에는 더 안전한 거리의 좌표 사용
- **단계별 테스트**: 한 번에 모든 동작이 아닌 단계별로 확인

## 프로젝트 디렉토리 구조 (v3.0)

```
ur_pick_and_place/
├── action/                           # ROS2 액션 정의
│   ├── PickAndPlace.action          # 기존 Pick&Place 액션 (호환성)
│   ├── Pick.action                  # Pick 전용 액션
│   └── Place.action                 # Place 전용 액션
├── srv/                             # ROS2 서비스 정의
│   └── GripperControl.srv           # Gripper 제어 서비스
├── include/ur_pick_and_place/       # C++ 헤더 파일
│   ├── goal_receiver_node.hpp       # 목표 수신 노드 클래스
│   ├── ready_executor_node.hpp      # Ready 위치 이동 노드 클래스
│   ├── gripper_controller_node.hpp  # Gripper 제어 노드 클래스
│   ├── pick_executor_node.hpp       # Pick 실행 노드 클래스
│   ├── place_executor_node.hpp      # Place 실행 노드 클래스
│   └── pick_place_manager_node.hpp  # 매니저 노드 클래스
├── src/                             # C++ 구현 파일 (단일 파일 구조)
│   ├── ur_pick_and_place_moveit.cpp # 기존 단일 파일 코드 (호환성)
│   ├── goal_receiver_node.cpp       # 목표 수신 구현 + main
│   ├── ready_executor_node.cpp      # Ready 위치 이동 구현 + main
│   ├── gripper_controller_node.cpp  # Gripper 제어 구현 + main
│   ├── pick_executor_node.cpp       # Pick 실행 구현 + main
│   ├── place_executor_node.cpp      # Place 실행 구현 + main
│   └── pick_place_manager_node.cpp  # 매니저 구현 + main
├── launch/                          # ROS2 런치 파일
│   ├── fake_hardware_pick_and_place.launch.py  # Fake HW 통합 런치 (권장)
│   ├── modular_pick_and_place.launch.py        # Pick&Place 노드만 실행
│   └── ur_pick_and_place.launch.py             # 기존 호환성 런치
├── scripts/                         # 테스트 및 데모 스크립트
│   ├── test_modular_system.py       # 자동화된 테스트 (시뮬레이션)
│   └── real_robot_demo.py           # 실물 로봇 안전 데모
├── CMakeLists.txt                   # CMake 빌드 설정
├── package.xml                      # ROS2 패키지 메타데이터
└── README.md                        # 이 파일
```


## 빌드 및 설치

### 1. 의존성 설치

```bash
# ROS2 및 MoveIt 의존성 확인
sudo apt install ros-humble-moveit ros-humble-joint-state-publisher-gui

# UR 로봇 관련 패키지 (Fake Hardware 포함)
sudo apt install ros-humble-ur-*

# Ignition Gazebo (선택사항 - 물리 시뮬레이션 필요시만)
# sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ign-ros2-control

# 워크스페이스에서 의존성 자동 설치
cd ~/dev_ws/UR/ur_simulation
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 빌드

```bash
cd ~/dev_ws/UR/ur_simulation
colcon build --packages-select ur_pick_and_place
source install/setup.bash
```

## 사용법 - 수동 토픽 전송

### CLI 명령어로 Pick & Place 실행

#### 기본 사용법 (모든 환경 공통)
```bash
# Pick 목표만 설정 (Place는 자동으로 하드코딩된 위치: x=-0.340, y=0.310, z=0.264)
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "base_link"}, pose: {position: {x: 0.010, y: 0.410, z: 0.264}, orientation: {w: 1.0}}}'
```

**📝 참고**: Place 위치는 하드코딩되어 있으므로 Pick 위치만 지정하면 됩니다.

#### 다양한 Pick 위치 예시
```bash
# 테이블 중앙
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "base_link"}, pose: {position: {x: 0.0, y: 0.35, z: 0.26}, orientation: {w: 1.0}}}'

# 테이블 왼쪽
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "base_link"}, pose: {position: {x: 0.2, y: 0.3, z: 0.25}, orientation: {w: 1.0}}}'

# 테이블 오른쪽
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
'{header: {frame_id: "base_link"}, pose: {position: {x: -0.2, y: 0.3, z: 0.25}, orientation: {w: 1.0}}}'
```

### 실행되는 노드들
- `goal_receiver_node`: 외부 목표 수신 및 시퀀스 관리
- `ready_executor_node`: 안전한 Ready 위치로 이동
- `pick_executor_node`: Pick 동작 전문 실행
- `place_executor_node`: Place 동작 전문 실행  
- `gripper_controller_node`: Gripper 제어
- `pick_place_manager_node`: 시퀀스 흐름 관리

**📝 참고**: 두 목표 중 하나만 설정하면 시스템이 대기 상태에 머물며, 두 목표가 모두 설정되어야 시퀀스가 시작됩니다.

### Ready Position 서비스 사용

로봇을 안전한 준비 위치로 이동:
```bash
# Ready 위치로 이동
ros2 service call /ready_service std_srvs/srv/Trigger
```

## 동작 시퀀스 상세

### Pick 시퀀스
1. **Home 위치**: 안전한 시작 위치로 이동
2. **Pick 위치**: 목표 위치 + 0.08m 높이로 이동
3. **하강**: 0.08m 단계적 하강하여 목표 위치 도달
4. **Grasp**: 그리퍼로 물체 집기
5. **상승**: 0.08m 단계적 상승

### Place 시퀀스  
1. **Carry**: Pick 위치에서 Place 위치 + 0.08m로 이동
2. **Approach**: Place 위치 + 0.04m → Place 위치로 하강
3. **Drop**: 그리퍼 열어서 물체 놓기
4. **Retreat**: Place 위치 → Place 위치 + 0.08m로 상승
5. **Home**: 안전한 홈 위치로 복귀

### 좌표계 및 좌표 정보

모든 좌표는 `base_link` 프레임을 기준으로 합니다:

```yaml
# 하드코딩된 Place 위치 (고정)
Place Position: [-0.340, 0.310, 0.264]  # 테이블 뒤쪽 (변경 불가)

# Pick 위치 예시 (사용자 지정 가능)
Default Pick:   [0.010, 0.410, 0.264]   # 테이블 앞쪽
Table Center:   [0.000, 0.350, 0.260]   # 테이블 중앙
Table Left:     [0.200, 0.300, 0.250]   # 테이블 왼쪽
Table Right:    [-0.200, 0.300, 0.250]  # 테이블 오른쪽
```

**중요**: Place 위치는 코드에 하드코딩되어 있으며, Pick 위치만 토픽으로 지정할 수 있습니다.

## 트러블슈팅

### 일반적인 문제들

1. **시뮬레이션 환경이 실행되지 않는 경우**:
   ```bash
   # UR 관련 패키지 재설치
   sudo apt install ros-humble-ur-*
   
   # Ignition Gazebo (gz) 관련 패키지 설치
   sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
   sudo apt install ros-humble-ign-ros2-control
   
   # 전체 gz 관련 패키지 설치
   sudo apt install ros-humble-ros-gz*
   ```

2. **MoveIt 계획 실패**:
   - 목표 위치가 로봇 작업공간 내에 있는지 확인
   - 장애물과의 충돌 가능성 확인
   - Planning time 증가: `setPlanningTime(30.0)`

3. **액션 서버 연결 실패**:
   ```bash
   # 액션 서버 상태 확인
   ros2 action list
   ros2 node list
   ```

4. **Gripper 제어 실패**:
   - Gripper 서비스가 실행 중인지 확인
   - 시뮬레이션에서는 placeholder로 동작

### 로그 확인

```bash
# 특정 노드 로그 확인
ros2 node info /pick_executor_node
ros2 topic echo /pick_place_status

# 상세 로그 출력
ros2 launch ur_pick_and_place modular_pick_and_place.launch.py --ros-args --log-level DEBUG
```

## 개발자 정보

### 버전 히스토리
- **v3.0** (현재): 
  - **Gazebo 제거**: Fake Hardware 기반으로 전환
  - 통합 런치 파일 (`fake_hardware_pick_and_place.launch.py`)
  - 표준 UR 패키지 활용 (ur_bringup, ur_moveit_config)
  - RViz 전용 시각화로 10배 이상 빠른 실행
- **v2.1**: Place 위치 하드코딩, Pick only 입력
- **v2.0**: 실물 로봇 지원, Ready Position 노드, 안전 데모 스크립트
- **v1.1**: 단일 파일 구조, 시퀀스 관리 최적화
- **v1.0**: 초기 모듈화 구조

### 기여 방법
1. 이슈 리포트: 문제 발생 시 상세한 로그와 함께 이슈 등록
2. 기능 요청: 새로운 기능이나 개선사항 제안
3. 코드 기여: Fork → 개발 → Pull Request

### 라이센스
이 프로젝트는 MIT 라이센스 하에 배포됩니다. 