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

> ⚠ **spawn을 바꾸거나 보행 정책을 갈아끼우면** 로봇의 자기위치 초기값도 같이 바꿔야 한다
> (정책마다 활성화 런지가 달라 정착 위치가 다르다):
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
sim 코드는 손대지 않는다. 단, 정책이 바뀌면 활성화 런지가 달라지므로 AMCL 초기값은 다시 재야
한다(위 ⚠).

## 알아두면 좋은 것

- **탐색 전략**: 기본은 **직진-바운스**(스캔에 장애물이 1.2m 안으로 들어오면, 온 방향을 제외한
  임의의 트인 방향으로 아크 턴 후 다시 직진; `search_timeout_s` 180초가 예산). 고정 루트 순찰로
  돌리려면 매니저 파라미터 `search_mode:=route`(+`search_waypoints`).
- **첫 실행 비용**: Isaac Sim 이미지 ~17GB + 클라우드 에셋(창고·Go2·소품) 다운로드. 이후는 캐시.
- **보행 정책의 성격**(2026-09-02 `robust_creep` 정책, 2026-09-03 실측 — 버그 아님): 0.4 m/s 정속은
  97% 이행하지만 저속은 데드존이고 이전 flat 정책보다 **더 깊다**(0.10→7% · 0.15→11% · 0.20→13% ·
  0.25→42% · 0.30→73%; flat은 앞 셋이 23/17/49%였다 — 웹 UI에 속도 슬라이더가 없는 이유).
  제자리 회전은 이제 된다(0.3~0.8 rad/s 명령의 70~93%; flat은 ~6%), 걷는 중 회전은 93%.
  기립 높이는 0.38 m로 flat(0.23 m)보다 15 cm 높다 — 카메라·라이다가 그만큼 올라간다.
  명령이 0이어도 **왼쪽으로 천천히 돈다**(30초에 +0.21 rad, 150초에 +0.41 rad에서 멈춤). 위치는
  안 움직인다. 정책 출처·학습 조건은 `robot_sw/models/locomotion/policy_meta.yaml` 머리말.
- **정책 교체에 맞춰 고친 것 3가지**(2026-09-03): ① 하네스가 로봇을 **학습된 기본 스탠스로** 세운다 —
  USD 자세로 떨어뜨리던 이전 방식의 활성화 런지(0.66 m, flat은 0.97 m)가 0.05 m로 줄었다.
  ② 그에 맞춰 AMCL 초기값을 다시 쟀다. ③ **MPPI `ax_max` 3.0→8.0** — MPPI는 실측 속도에서 한 스텝
  (0.05 s)만큼만 명령을 올리는데 3.0이면 그 첫 스텝이 0.15 m/s로 데드존 안이라, 로봇이 0.002 m/s로
  서 있고 명령도 영원히 안 올라갔다(실측: 60초 정지 후 접근 실패). 근거는 `nav2_params.yaml` DELTA 13.
  카메라가 높아졌지만 의자 검출은 오히려 좋아졌다(2 m에서 신뢰도 0.90, 박스 높이 0.39H).
- **카메라를 켜면** 시뮬 속도(RTF)가 ~0.75로 내려간다 — 놀이엔 충분.
- person 소품은 팔 벌린 정적 마네킹(폭 1.76 m) — 좁은 통로에 두면 로봇이 지나갈 자리를 계산해서.
- 진단: `docker logs go2-sim` / `go2-robot` / `go2-web`.

## cv-infra 검증

`verify/space.pict`는 이 앱의 환경·시작 자세·표적 조합을 선언한다. GitHub Actions는
PICT가 만든 각 케이스마다 cv-infra의 runtime image에서 `verify/run`을 호출한다. 인프라는
이 명령이 Docker Compose와 ROS 2 action을 사용하는지 알지 못하고, `CASE`·`OUT`·`SEED`를
전달하고 결과만 수거한다.

`verify/run`은 기존 로컬 앱과 같은 `sim` + `robot_sw` 경로를 케이스별로 기동한 뒤 `/patrol`
action을 호출한다. 웹 컨테이너와 브라우저는 검증하지 않는다. `verify/judge`는 `$OUT`의
mission evidence를 읽어 flat JSON verdict를 출력한다.

로컬 단일 케이스는 다음처럼 실행할 수 있다.

```bash
CASE=/path/to/case.json OUT=/path/to/out CV_CHECKOUT_HOST="$PWD" CV_OUT_HOST=/path/to/out \
  docker run --rm -v /var/run/docker.sock:/var/run/docker.sock -v "$PWD:/cv/checkout:ro" \
  -v "/path/to/out:/cv/checkout/verify/out:rw" go2-verify-runtime:local verify/run
```
