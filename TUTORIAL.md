# OpenArm ROS 2 튜토리얼 (오른팔 전용)

양팔 본체에서 왼팔이 없는 구성을 가정. 모든 명령은 `~/openarm_ws` 기준.

---

## A. 오른팔 bringup

### Fake hardware (CAN 없이 테스트)
```bash
ros2 launch openarm_bringup openarm.right_arm.launch.py use_fake_hardware:=true
```

### 실 하드웨어
```bash
ros2 launch openarm_bringup openarm.right_arm.launch.py use_fake_hardware:=false right_can_interface:=can0
```

### 상태 확인 (다른 터미널)
```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 topic echo /joint_states --once
```
기대: `joint_state_broadcaster`, `right_joint_trajectory_controller`,
`right_gripper_controller` 모두 `active`. `/joint_states` 의 position 값이
실제 자세와 일치 (실 하드웨어 시).

---

## B. 단일 trajectory 명령

### 이름표
| 종류 | 이름 |
|---|---|
| 컨트롤러 | `right_joint_trajectory_controller`, `right_gripper_controller` |
| 액션 | `/right_joint_trajectory_controller/follow_joint_trajectory` |
| 팔 조인트 | `openarm_right_joint1` ~ `openarm_right_joint7` |
| 그리퍼 조인트 | `openarm_right_finger_joint1` |

### 예제 액션 호출 (5초후 0.15 rad 로 이동)
급작스러운 움직임이 있을 수 있으니 팔 주위에서 떨어질 것. 
```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["openarm_right_joint1","openarm_right_joint2","openarm_right_joint3","openarm_right_joint4","openarm_right_joint5","openarm_right_joint6","openarm_right_joint7"],
  points: [{positions: [0.15,0.15,0.15,0.15,0.15,0.15,0.15], time_from_start: {sec: 5}}]}}'
```

---

## C. MoveIt demo

### Fake hardware
```bash
ros2 launch openarm_bimanual_moveit_config demo.right_arm.launch.py use_fake_hardware:=true
```

### 실 하드웨어
```bash
ros2 launch openarm_bimanual_moveit_config demo.right_arm.launch.py use_fake_hardware:=false right_can_interface:=can0
```

### RViz MotionPlanning 사용
- `Planning Group` = `right_arm`
- 그룹 상태: `home`, `hands_up`
- 그리퍼 그룹 상태: `closed`, `half_closed`, `open`
- `Goal State` 선택 → `Plan` → `Execute`


## D. URDF 시각화 / 조인트 슬라이더 GUI

`openarm_description` 패키지 사용. 자세한 건
[openarm_description/TUTORIAL.md](../openarm_description/TUTORIAL.md) 참고.

```bash
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true include_left:=false include_right:=true
```
