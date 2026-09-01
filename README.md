# Go2 순찰 로봇

Isaac Sim 창고에서 **Unitree Go2가 순찰하며 물체(의자·사람)를 찾는** ROS 2 앱과,
코드를 몰라도 로봇을 몰 수 있는 **웹 컨트롤러**. 전부 한 데스크탑에서 컨테이너 3개로 돈다.

```
┌─────────────┐  ROS 2 (DDS, host network)  ┌──────────────┐
│  sim        │  /camera /scan /odom /clock │  robot       │
│  Isaac Sim  │ ───────────────────────────▶│  nav2 + YOLO │
│  창고+Go2 몸 │ ◀─────────────────────────── │  + 순찰 매니저 │
│  +보행 정책  │          /cmd_vel           └──────┬───────┘
└─────────────┘                                    │ /patrol 액션 · /cmd_vel_teleop
                                            ┌──────┴───────┐      ┌──────────┐
                                            │  web         │◀────▶│ 브라우저  │
                                            │  rosbridge 등 │      │ (사용자)  │
                                            └──────────────┘      └──────────┘
```

## 준비물 (1회)

- NVIDIA GPU(테스트: RTX 4080 16GB) + **R580대 드라이버** + docker + nvidia container toolkit.
- Isaac Sim 동의 파일 작성 — 본인이 직접:
  ```bash
  cp .env.example .env   # 열어서 ACCEPT_EULA / PRIVACY_CONSENT 채우기
  ```

## 실행

```bash
docker compose up -d sim      # ① 세상: 창고 + 장애물 + Go2 몸 + 보행  (첫 부팅은 에셋 다운로드로 수 분)
docker compose up -d robot    # ② 순찰 지능: nav2 + YOLO + 매니저
docker compose up -d web      # ③ 컨트롤러 서버
```
④ 브라우저에서 **http://localhost:8000** → 왼쪽 패널로 조작(수동 주행·순찰 시작), 오른쪽에서 로봇의 눈(카메라/지도).

끄기: `docker compose down` (에셋·셰이더 캐시는 볼륨에 남아 다음 부팅이 ~25초).

## 세계 바꾸기

`sim/world.yaml` 하나가 세계다 — 로봇 시작 위치(spawn)와 소품(props: chair/desk/forklift/person/box) 좌표.
수정 후 `docker compose restart sim`.

> ⚠ **spawn을 바꾸면** 로봇의 자기위치 초기값도 같이 바꿔야 한다:
> `robot_sw/src/go2_bringup/params/nav2_params.yaml`의 AMCL `initial_pose`.
> (로봇은 켜질 때 "내가 어디 있는지"를 이 값으로 믿고 시작한다.)

## 로봇 인터페이스 (개발자용)

| 이름 | 타입 | 방향 | 의미 |
|---|---|---|---|
| `/patrol` | `go2_msgs/action/Patrol` | →robot | 순찰 미션: goal `{target_class: "chair"\|"person"\|""}` — 빈 값은 아무 표적. feedback `state`, result `{found, target_pose, message}` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | →robot | nav2 본연의 지점 이동(그대로 노출 — 단순 주행은 이걸 직접) |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | →robot | 수동 조작 입력 (twist_mux에서 자율주행보다 우선) |
| `/cmd_vel` | `geometry_msgs/Twist` | robot→sim | 최종 속도 명령 (보행 정책의 입력) |
| `/camera/image_raw` `/camera/depth/image_raw` `/camera/camera_info` `/scan` `/odom` `/clock` | 표준 센서 | sim→robot | 640×480 RGB/깊이 10Hz · 360° 라이다 10Hz · 오도메트리 30Hz |
| `/detections` | `vision_msgs/Detection2DArray` | robot 내부/web | YOLO 원시 탐지 (화면 좌표) |
| `/targets` | `vision_msgs/Detection3DArray` | robot 내부 | 확증된 표적 (map 좌표 + 클래스 라벨) |

터미널에서 순찰 걸기:
```bash
docker exec go2-robot bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/go2_ws/install/setup.bash && \
  ros2 action send_goal /patrol go2_msgs/action/Patrol "{target_class: chair}" --feedback'
```

## 저장소 구조 — 분리 원칙

```
sim/        시뮬레이션 세계만: Isaac 부팅 스크립트 + world.yaml (환경 명세)
robot_sw/   로봇의 모든 것: ROS 노드(src/) + AI 산출물(models/)
web/        사용자 클라이언트 (rosbridge + 카메라 스트림 + 정적 UI)
```

**파일의 소속과 실행 위치는 별개다.** 보행 정책(`robot_sw/models/locomotion/policy.pt`)은
로봇의 학습 산출물이라 로봇 폴더에 살지만, 50Hz 균형 제어는 물리 루프와 동기여야 하므로
**실행은 sim이 한다**(compose가 읽기 전용으로 마운트). 재학습하면 `policy.pt`와
`policy_meta.yaml`(게인·스케일·스탠스 등 학습과 함께 바뀌는 상수)을 **함께** 교체하면 끝 —
sim 코드는 손대지 않는다.

## 알아두면 좋은 것

- **첫 실행 비용**: Isaac Sim 이미지 ~17GB + 클라우드 에셋(창고·Go2·소품) 다운로드. 이후는 캐시.
- **보행 정책의 성격**(측정된 사실, 버그 아님): 0.2 m/s 미만 저속 명령은 5~23%만 이행(데드존 —
  웹 UI에 속도 슬라이더가 없는 이유), 제자리 회전은 느리고(명령의 ~6%) 걷는 중 회전은 잘 됨(91%),
  정책이 붙는 순간 진행 방향으로 ~1 m 런지가 있다.
- **카메라를 켜면** 시뮬 속도(RTF)가 ~0.75로 내려간다 — 놀이엔 충분.
- person 소품은 팔 벌린 정적 마네킹(폭 1.76 m) — 좁은 통로에 두면 로봇이 지나갈 자리를 계산해서.
- 진단: `docker logs go2-sim` / `go2-robot` / `go2-web`.
