# openarm_bringup

OpenArm 오른팔 1대를 띄우기 위한 launch + 컨트롤러 설정.

## 제공 launch

| 파일 | 용도 |
|---|---|
| `openarm.right_arm.launch.py` | body + 오른팔 robot_state_publisher + ros2_control_node + 컨트롤러 + RViz |
| `openarm.bimanual.launch.py` | (참고용) 양팔 launch. 본 환경에서 사용하지 않음 |

## 주요 인자 (`openarm.right_arm.launch.py`)

| 인자 | 기본값 | 설명 |
|---|---|---|
| `use_fake_hardware` | `true` | `mock_components/GenericSystem` 사용 여부 |
| `right_can_interface` | `can0` | 실 하드웨어용 CAN 인터페이스 |
| `robot_controller` | `joint_trajectory_controller` | 또는 `forward_position_controller` |
| `arm_prefix` | `""` | ROS 네임스페이스 (joint prefix 가 아님) |
| `ee_type` | `openarm_hand` | end-effector 타입 |
| `controllers_file` | `openarm_right_arm_controllers.yaml` | `config/controllers/` 내부 |

## 빠른 실행

```bash
ros2 launch openarm_bringup openarm.right_arm.launch.py use_fake_hardware:=true
```

상세 워크플로 (실 하드웨어, trajectory 명령 등) →
[../TUTORIAL.md](../TUTORIAL.md) 의 B / C 섹션.

## 제공 컨트롤러 설정

`config/controllers/` 내부:
- `openarm_right_arm_controllers.yaml` — 750Hz, `interpolation_method: "none"` (직접 컨트롤용)
- `openarm_right_arm_moveit_controllers.yaml` — 100Hz, `interpolation_method: "splines"` (MoveIt 용)
