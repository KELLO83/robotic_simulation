# TurtleBot3 DRL Stage 4 on Isaac Sim

이 폴더는 기존 ROS 2 Humble DRL 노드와 학습 코드를 그대로 두고 시뮬레이터만
Gazebo Classic에서 Isaac Sim 6으로 교체한다. Docker는 사용하지 않는다.

## 구성

- Isaac Sim: Stage 4 벽, 동적 장애물 2개, TurtleBot3 Waffle Pi, 물리와 LiDAR
- 기존 노드: `environment`, `gazebo_goals`, `train_agent`
- 기존 ROS 계약: `/scan` 40개, `/odom`, `/obstacle/odom`, `/clock`, `/cmd_vel`
- SLAM 계약: `/slam_scan` 360개, `map -> odom` TF, `/map`, `/map_updates`
- 호환 서비스: `/pause_physics`, `/unpause_physics`, `/reset_simulation`,
  `/spawn_entity`, `/delete_entity`

Isaac Sim 6은 Python 3.12를 내장하지만 ROS 2 Humble은 이 시스템에서 Python
3.10.12를 사용한다. 따라서 Humble 노드는 시스템 Python에서 실행하고, 같은 호스트의
`/tmp/turtlebot3_isaac_control.sock`을 통해 Isaac에 제어 명령만 전달한다. DRL 노드의
토픽, 서비스, state/reward, DDPG/TD3/DQN 코드는 변경하지 않는다.

## 최초 1회 빌드

저장소 루트에서 실행한다.

```bash
./Isaac/build_adapter.sh
```

Waffle Pi는 참조 구현
`/home/rr/turtlebot3_drlnav_humble/isaac/assets/turtlebot3_waffle_pi`의 USD와
native PhysX 구성을 사용한다. 다른 Isaac 설치 위치를 쓸 때는 다음과 같이 지정한다.

```bash
export ISAAC_SIM_PATH=/path/to/isaac-sim
```

## 실행 순서

터미널 1 — Isaac Stage 4:

```bash
cd /home/rr/turtlebot3_drlnav
./Isaac/run_stage4.sh
```

화면 없이 실행하려면 `./Isaac/run_stage4.sh --headless`를 사용한다. Isaac 시작에는
이 장비에서 약 30~40초가 걸린다.

터미널 2 — 호환 어댑터와 기존 환경/목표 노드:

```bash
cd /home/rr/turtlebot3_drlnav
./Isaac/run_training_nodes.sh
```

터미널 3 — 기존 학습 노드:

```bash
cd /home/rr/turtlebot3_drlnav
./Isaac/run_agent.sh ddpg
```

마지막 인자는 기존과 동일하게 `ddpg`, `td3`, `dqn` 중 사용할 알고리즘이다. 직접
실행하려면 아래 명령도 동일하다.

```bash
source /home/rr/turtlebot3_drlnav/setup_humble.bash
source /home/rr/turtlebot3_drlnav/Isaac/ros2_ws/install/setup.bash
ros2 run turtlebot3_drl train_agent ddpg
```

종료는 학습 노드, 환경 노드, Isaac 순서로 각 터미널에서 `Ctrl-C`를 누른다.

## SLAM과 RViz

강화학습과 별개로 지도만 자동 작성하려면 Isaac 실행 후 다음 명령을 실행한다.

```bash
cd /home/rr/turtlebot3_drlnav
./Isaac/run_mapping.sh start_drive:=true
```

이 명령은 Isaac 어댑터, Cartographer, 벽 회피 주행 노드와 RViz를 실행한다. RViz의
Fixed Frame은 `map`이고 `/map`, `/slam_scan`, TF가 표시된다.

기존 강화학습이 로봇을 움직이는 동안 지도도 함께 작성하려면 어댑터와 `/cmd_vel`
publisher를 중복 실행하지 않는다.

```bash
./Isaac/run_training_nodes.sh
./Isaac/run_agent.sh ddpg
./Isaac/run_mapping.sh start_adapter:=false start_drive:=false
```

작성된 지도를 저장한다.

```bash
./Isaac/save_map.sh
```

기본 저장 위치는 `Isaac/maps/stage4_waffle_pi.yaml`과 `.pgm`이다.

## 빠른 확인

환경 노드까지 실행한 상태에서 다음 서비스가 보여야 한다.

```bash
ros2 service list | grep -E 'step_comm|goal_comm|pause_physics|reset_simulation'
ros2 topic echo --once /scan
ros2 topic echo --once /slam_scan
ros2 topic echo --once /odom
ros2 topic echo --once /map
```

`/scan.ranges` 길이는 40, `/slam_scan.ranges` 길이는 360이어야 한다. 실행 중 ROS 그래프가 이전 프로세스를 보여 주면
`ros2 daemon stop` 후 다시 확인한다.

## 구현 경계

- `world/`: 원본 Gazebo Stage 4 치수와 장애물 keyframe을 USD로 구성
- `robot/`: 참조 Waffle Pi USD, native PhysX articulation과 차동 구동
- `ros_bridge/`: Isaac 발행 토픽과 같은 호스트 제어 채널
- `ros2_ws/`: Humble 호환 어댑터와 Cartographer/RViz SLAM 패키지
- `apps/`: Isaac standalone 실행 루프

`--no-ros --test-frames N`은 ROS 없이 장면과 물리만 점검할 때 사용할 수 있다.
