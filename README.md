# UR Pick and Place - 모듈화된 시스템

이 패키지는 UR 로봇을 위한 모듈화된 Pick & Place 시스템입니다. 기존의 하드코딩된 단일 파일 구조에서 ROS2의 모듈화된 아키텍처로 리팩토링되었습니다.

## 시스템 구조

### 노드 구성

| 기능 | 노드명 | 통신 방식 | 역할 |
|------|--------|-----------|------|
| 목표 pose 수신 | `goal_receiver_node` | 토픽 (`/pick_goal`, `/place_goal`) | 외부에서 위치 명령을 수신 |
| Pick 동작 수행 | `pick_executor_node` | 액션 (`pick_action`) | 수신된 pose를 기반으로 pick 수행 |
| Place 동작 수행 | `place_executor_node` | 액션 (`place_action`) | 수신된 pose를 기반으로 place 수행 |
| Gripper 제어 | `gripper_controller_node` | 서비스 (`/gripper/control`) | 서보 또는 IO 제어 |
| 전체 통합 | `pick_place_manager_node` | 내부 호출 및 상태 관리 | pick → carry → place 일련의 흐름 담당 |

### 통신 구조

```
외부 시스템 → 토픽 목표 설정 → 토픽 트리거 전송
    ↓ (/pick_goal, /place_goal)        ↓ (/pick_place_trigger)
goal_receiver_node ←──────────────────────
    ↓ (내부 토픽: /internal/pick_goal, /internal/place_goal)
pick_place_manager_node
    ↓ (액션: /pick_action, /place_action)
pick_executor_node / place_executor_node
    ↓ (서비스: /gripper/control)
gripper_controller_node
```

### ⚠️ 중요한 동작 흐름

**두 단계 프로세스**가 필요합니다:

1. **목표 설정 단계**: Pick/Place 위치를 `/pick_goal`, `/place_goal` 토픽으로 전송
2. **실행 트리거 단계**: `/pick_place_trigger` 토픽으로 실행 명령 전송

**주의**: 목표만 설정하고 트리거를 보내지 않으면 아무 동작도 하지 않습니다!

## 프로젝트 디렉토리 구조

```
ur_pick_and_place/
├── action/                           # ROS2 액션 정의
│   ├── PickAndPlace.action          # 기존 Pick&Place 액션
│   ├── Pick.action                  # 새로운 Pick 전용 액션
│   └── Place.action                 # 새로운 Place 전용 액션
├── srv/                             # ROS2 서비스 정의
│   └── GripperControl.srv           # Gripper 제어 서비스
├── include/ur_pick_and_place/       # C++ 헤더 파일
│   ├── goal_receiver_node.hpp       # 목표 수신 노드 클래스
│   ├── gripper_controller_node.hpp  # Gripper 제어 노드 클래스
│   ├── pick_executor_node.hpp       # Pick 실행 노드 클래스
│   ├── place_executor_node.hpp      # Place 실행 노드 클래스
│   └── pick_place_manager_node.hpp  # 매니저 노드 클래스
├── src/                             # C++ 구현 파일
│   ├── ur_pick_and_place_moveit.cpp # 기존 단일 파일 코드
│   ├── goal_receiver_node.cpp       # 목표 수신 구현
│   ├── goal_receiver_main.cpp       # 목표 수신 main 함수
│   ├── gripper_controller_node.cpp  # Gripper 제어 구현
│   ├── gripper_controller_main.cpp  # Gripper 제어 main 함수
│   ├── pick_executor_node.cpp       # Pick 실행 구현
│   ├── pick_executor_main.cpp       # Pick 실행 main 함수
│   ├── place_executor_node.cpp      # Place 실행 구현
│   ├── place_executor_main.cpp      # Place 실행 main 함수
│   ├── pick_place_manager_node.cpp  # 매니저 구현
│   └── pick_place_manager_main.cpp  # 매니저 main 함수
├── launch/                          # ROS2 런치 파일
│   └── modular_pick_and_place.launch.py  # 모든 노드 실행
├── scripts/                         # 테스트 스크립트
│   └── test_modular_system.py       # 자동화된 테스트 스크립트
├── CMakeLists.txt                   # CMake 빌드 설정
├── package.xml                      # ROS2 패키지 메타데이터
└── README.md                        # 이 파일
```

## 빌드 및 실행

### 1. 의존성 설치

```bash
# ROS2 및 MoveIt 의존성 확인
sudo apt install ros-humble-moveit ros-humble-joint-state-publisher-gui

# 워크스페이스에서 의존성 자동 설치
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 빌드

```bash
cd ~/dev_ws
colcon build --packages-select ur_pick_and_place
source install/setup.bash
```

### 3. 실행

#### 모듈화된 시스템 실행
```bash
# 메인 터미널
ros2 launch ur_pick_and_place modular_pick_and_place.launch.py
```

실행되는 노드들:
- `goal_receiver_node`: 외부 목표 수신
- `pick_executor_node`: Pick 동작 실행
- `place_executor_node`: Place 동작 실행  
- `gripper_controller_node`: Gripper 제어
- `pick_place_manager_node`: 전체 흐름 관리

#### 기존 시스템 실행 (비교용)
```bash
ros2 run ur_pick_and_place ur_pick_and_place_moveit
```

### 4. 자동화된 테스트

```bash
# 새 터미널에서
ros2 run ur_pick_and_place test_modular_system.py
```

테스트 시나리오 선택:
- **1**: Pick and Place 시퀀스 (기본) - Pick 완료 후 자동으로 Place 실행
- **2**: Pick만 실행 - 물체 집기만 수행
- **3**: Place만 실행 - 물체 놓기만 수행  
- **4**: 개별 테스트 - Pick 완료 후 사용자가 직접 Place 실행

## 사용법 - 수동 토픽 전송

### 기본 사용법 (Pick and Place)

```bash
# 터미널 1: 시스템 실행
ros2 launch ur_pick_and_place modular_pick_and_place.launch.py

# 터미널 2: Pick 목표 설정
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link', stamp: {sec: 0, nanosec: 0}}, 
  pose: {position: {x: 0.010, y: 0.410, z: 0.264}, 
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 터미널 3: Place 목표 설정 (1-2초 후)
ros2 topic pub --once /place_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link', stamp: {sec: 0, nanosec: 0}}, 
  pose: {position: {x: -0.340, y: 0.310, z: 0.264}, 
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 터미널 4: 실행 트리거 (두 목표 모두 설정 후)
ros2 topic pub --once /pick_place_trigger std_msgs/msg/String "data: 'start_pick_place'"
```

### Pick만 실행

```bash
# Pick 목표 설정
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, 
  pose: {position: {x: 0.010, y: 0.410, z: 0.264}, 
         orientation: {w: 1.0}}}"

# Pick 실행 트리거
ros2 topic pub --once /pick_place_trigger std_msgs/msg/String "data: 'start_pick'"
```

### Place만 실행

```bash
# Place 목표 설정
ros2 topic pub --once /place_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, 
  pose: {position: {x: -0.340, y: 0.310, z: 0.264}, 
         orientation: {w: 1.0}}}"

# Place 실행 트리거
ros2 topic pub --once /pick_place_trigger std_msgs/msg/String "data: 'start_place'"
```

### 다양한 목표 위치 예시

```bash
# 테이블 왼쪽 Pick
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, 
  pose: {position: {x: 0.2, y: 0.3, z: 0.25}, 
         orientation: {w: 1.0}}}"

# 테이블 오른쪽 Place
ros2 topic pub --once /place_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, 
  pose: {position: {x: -0.2, y: -0.3, z: 0.25}, 
         orientation: {w: 1.0}}}"

# 45도 회전된 자세
ros2 topic pub --once /pick_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, 
  pose: {position: {x: 0.1, y: 0.4, z: 0.26}, 
         orientation: {x: 0.0, y: 0.0, z: 0.383, w: 0.924}}}"
```

## 상태 모니터링 및 디버깅

### 실시간 상태 확인

```bash
# 전체 시스템 상태
ros2 topic echo /pick_place_status

# 목표 설정 확인
ros2 topic echo /pick_goal      # Pick 목표
ros2 topic echo /place_goal     # Place 목표

# 내부 통신 확인
ros2 topic echo /internal/pick_goal  # 내부 전달 상태 확인
```

### 액션 모니터링

```bash
# 액션 서버 목록 확인
ros2 action list

# Pick 액션 상태 확인
ros2 action info /pick_action
ros2 action send_goal /pick_action ur_pick_and_place_interfaces/action/Pick "{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.1, y: 0.4, z: 0.26}, orientation: {w: 1.0}}}}"

# Place 액션 상태 확인
ros2 action info /place_action
```

### 서비스 테스트

```bash
# Gripper 서비스 테스트
ros2 service call /gripper/control ur_pick_and_place_interfaces/srv/GripperControl "{command: 'open'}"
ros2 service call /gripper/control ur_pick_and_place_interfaces/srv/GripperControl "{command: 'close'}"
```

### 노드 및 토픽 확인

```bash
# 실행 중인 노드 확인
ros2 node list

# 토픽 목록 확인
ros2 topic list

# 특정 노드의 토픽 확인
ros2 node info /goal_receiver_node
ros2 node info /pick_place_manager_node

# 토픽 정보 확인
ros2 topic info /pick_goal
ros2 topic info /pick_place_trigger
```

## Pick 동작 상세 시퀀스

새로운 모듈화된 시스템에서의 Pick 동작:

1. **Home 위치로 이동** - 안전한 시작 위치
2. **목표지점 + 0.08m 높이로 이동** (`moveToPickPosition`) - 물체 위 접근
3. **그리퍼 열기** - 물체 집기 준비
4. **0.08m 하강** (`descendToTarget`) - 4단계로 분할하여 안전하게 하강
5. **그리퍼 닫기** - 물체 집기
6. **0.08m 상승** (`ascendFromTarget`) - 4단계로 분할하여 안전하게 상승

### Cartesian Path Planning

시스템은 다음과 같은 fallback 메커니즘을 사용합니다:

1. **Cartesian path 우선 시도** (95% 성공률 임계값)
2. **실패 시 pose target planning으로 대체** - 기존 검증된 방식 사용

## 주요 개선사항

### 1. 모듈화
- **기존**: 단일 파일에서 모든 기능 처리 (830줄)
- **개선**: 기능별로 독립적인 노드로 분리 (각 200-300줄)

### 2. 재사용성
- **기존**: 하드코딩된 위치값으로 제한적 사용
- **개선**: 동적으로 목표 위치 설정 가능, 다양한 시나리오 지원

### 3. 확장성
- **기존**: 새로운 기능 추가 시 전체 코드 수정 필요
- **개선**: 독립적인 모듈로 쉽게 확장 가능 (새 노드 추가)

### 4. 디버깅
- **기존**: 전체 시스템 중단 시 디버깅 어려움
- **개선**: 개별 노드별 상태 모니터링 및 디버깅 가능

### 5. 테스트
- **기존**: 전체 시스템 통합 테스트만 가능
- **개선**: 개별 모듈 단위 테스트 가능

### 6. 안전성 ⭐
- **기존**: 동시 실행으로 인한 충돌 위험
- **개선**: 순차적 실행으로 안전성 확보

### 7. 다중 인터페이스
- **기존**: 하드코딩된 실행만 가능
- **개선**: 토픽, 액션, 서비스 다양한 인터페이스 지원

## 기존 코드와의 호환성

모듈화된 시스템은 기존 코드와 동일한 동작을 수행하도록 설계되었습니다:

- 동일한 Home position (0, -1.57, 0, -1.57, 0, 0)
- 동일한 Pick/Place 위치 (기본값)
- 동일한 Cartesian path 계산 방식
- 동일한 MoveIt 설정 및 planning group 사용

## 문제 해결

### 빌드 오류

```bash
# 의존성 재설치
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
rm -rf build/ install/ log/
colcon build --packages-select ur_pick_and_place

# 특정 의존성 문제
sudo apt install ros-humble-moveit-core ros-humble-moveit-ros-planning-interface
```

### 실행 오류

```bash
# MoveIt 설정 확인
ros2 launch ur_moveit_config ur_moveit.launch.py

# 개별 노드 테스트
ros2 run ur_pick_and_place goal_receiver_node
ros2 run ur_pick_and_place pick_place_manager_node

# 로그 확인
ros2 log set logger_name DEBUG
```

### 토픽 통신 문제

```bash
# 목표가 설정되지 않은 경우
ros2 topic echo /pick_goal --once    # 현재 Pick 목표 확인
ros2 topic echo /place_goal --once   # 현재 Place 목표 확인

# 트리거 누락 확인
ros2 topic echo /pick_place_trigger  # 트리거 메시지 확인

# 내부 통신 확인
ros2 topic echo /internal/pick_goal  # 내부 전달 상태 확인
```

### ⚠️ 일반적인 실수와 해결법

1. **트리거 메시지 누락**
   ```bash
   # 잘못된 방법: 목표만 설정
   ros2 topic pub /pick_goal geometry_msgs/msg/PoseStamped "..."
   # → 아무 동작 안 함
   
   # 올바른 방법: 목표 설정 + 트리거 전송
   ros2 topic pub /pick_goal geometry_msgs/msg/PoseStamped "..."
   ros2 topic pub /pick_place_trigger std_msgs/msg/String "data: 'start_pick'"
   ```

2. **목표 설정 순서**
   ```bash
   # Pick & Place 실행 시 두 목표 모두 설정 후 트리거
   ros2 topic pub /pick_goal ...    # 먼저 Pick 목표
   ros2 topic pub /place_goal ...   # 그 다음 Place 목표
   ros2 topic pub /pick_place_trigger std_msgs/msg/String "data: 'start_pick_place'"
   ```

3. **메시지 형식 오류**
   ```bash
   # header와 orientation 필드 확인
   # header.frame_id는 'base_link' 사용
   # orientation은 최소한 w: 1.0 설정
   ```

### MoveIt 연결 문제

```bash
# MoveIt 실행 상태 확인
ros2 node list | grep move_group

# move_group 연결 확인
ros2 service list | grep move_group

# Planning scene 확인
ros2 topic echo /monitored_planning_scene
```

### Gripper 제어 문제

```bash
# Gripper 서비스 가용성 확인
ros2 service list | grep gripper

# 수동 Gripper 테스트
ros2 service call /gripper/control ur_pick_and_place_interfaces/srv/GripperControl "{command: 'open'}"
ros2 service call /gripper/control ur_pick_and_place_interfaces/srv/GripperControl "{command: 'close'}"
```

## 성능 및 제한사항

### 시스템 성능
- **Pick 동작 시간**: 약 15-20초 (0.08m 4단계 분할 하강/상승 포함)
- **Place 동작 시간**: 약 10-15초
- **Cartesian planning 성공률**: 95% 이상 (fallback 포함)

### 현재 제한사항
1. **Gripper 하드웨어**: 실제 gripper 하드웨어 연결 시 `gripper_controller_node.cpp` 수정 필요
2. **동시 실행**: Pick과 Place 동시 실행 불가 (안전상 의도된 제한)
3. **경로 계획**: 복잡한 장애물 회피 미지원 (단순 Cartesian path만)

### 추천 사용 환경
- **시뮬레이션**: Gazebo + MoveIt
- **실제 로봇**: UR5e, UR10e with MoveIt 설정
- **Gripper**: 2-finger parallel gripper 권장

## 기여 및 확장

새로운 기능 추가 시:

1. **새 노드 추가**: `include/` 및 `src/`에 헤더와 구현 파일 추가
2. **CMakeLists.txt 업데이트**: 새 실행파일 추가
3. **Launch 파일 수정**: 새 노드를 launch 파일에 포함
4. **테스트 추가**: `scripts/`에 테스트 스크립트 추가

### 확장 아이디어
- Vision-based object detection 노드
- Force/torque sensor 기반 gripper 제어
- Multi-robot coordination
- Path optimization algorithms 