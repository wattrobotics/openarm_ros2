# OpenArm ROS 2

양팔 OpenArm v1.0 본체에서 **오른팔만 부착된 구성**을 위한 ROS 2 패키지 모음.

## 포함 패키지

| 패키지 | 역할 |
|---|---|
| `openarm_bringup` | robot_state_publisher + ros2_control + 컨트롤러 + RViz |
| `openarm_hardware` | DM 모터용 ros2_control hardware interface |
| `openarm_bimanual_moveit_config` | MoveIt 설정 (오른팔 demo 포함) |
| `openarm` | 메타 패키지 |

## 선행 조건

- ROS 2 Jazzy
- `openarm_description` 패키지 (URDF/Xacro/Mesh) — 같은 워크스페이스에 클론
- `openarm_can` 패키지

[CAN설정방법](https://docs.openarm.dev/software/setup/can-setup)

[영점설정방법](https://docs.openarm.dev/software/setup/motor-config#step-2-zero-position-calibration)

## 실행 방법

상세한 실행 명령은 [TUTORIAL.md](TUTORIAL.md) 참고.

## Related links

- 📚 [docs.openarm.dev](https://docs.openarm.dev/software/ros2/install) (업스트림 문서)
- 💬 [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE) — Copyright 2025 Enactic, Inc.

## Code of Conduct

[Code of Conduct](CODE_OF_CONDUCT.md)
