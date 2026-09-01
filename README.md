# cv-infra-user-go2

CV-Infra의 **두 번째 소비자 / E2E 픽스처** (public). 첫 번째(`cv-infra-user`, Nova Carter 휠 로봇)와 달리 여기서는
**Unitree Go2 사족보행 순찰 앱**을 SUT로 올려, 같은 플랫폼 계약이 *다른 로봇 · 다른 미션 · 다른 산출물 구성*에서도
러너 수정 0으로 통하는지를 소비자 입장에서 행사한다.

> **현 상태: 순찰 미션 완성 단계(U0 부트스트랩 · U1 맵·시나리오·튜닝 · U2 detector · U3 tracker/manager).**
> SUT 이미지는 **Nav2 + AMCL/map_server + 반입된 창고 점유맵**으로 항법하고, **`go2_detector`(YOLO11n CPU)**
> → **`go2_target_tracker`(C++)** → **`go2_patrol_manager`(C++)** 로 이어지는 인지·미션 층을 갖는다.
> **`go2_patrol_manager`가 플랫폼-facing `/navigate_to_pose`를 직접 노출**하고 nav2의 같은 액션은
> `/nav2/navigate_to_pose`로 안쪽에 리맵된다(결정 D4 — **어댑터 계약 변경 0**). `scenarios/`에 T0 스모크 ·
> TA 랜덤 · **TB 순찰 2종(chair/person)** · **의도적 실패 픽스처** · 커스텀 오라클 2종(`hold_near_goal`,
> `upright`) · 요청에 실려 가는 보행 정책 파일이 있다. PR을 열면 SUT 이미지를 빌드해 GHCR에 올리고
> **T0·TA·TB chair + 오라클 2종을 GPU 검증 잡에 배송**한다(실패 픽스처는 의도적으로 배송하지 않고,
> **TB person 은 배치 경로 실패가 미해결이라 격리(quarantine)** 되어 있다 — 아래 §CI 게이트).

## 저장소 레이아웃

```
robot_sw/
├── Dockerfile                    # SUT 이미지 (ros:jazzy @digest + Nav2 핀 + 인지 레이어 4장)
├── constraints-detector.txt      # pip 전량 핀 (torch CPU 고정 = AR-11 함정 차단)
├── ros_entrypoint.sh             # ROS + 오버레이 source 후 exec
├── src/go2_bringup/              # 설치 전용 ament 패키지 (컴파일 0)
│   ├── launch/go2_nav.launch.py    # nav2_bringup 재사용 + 맵 기본값 + use_localization 스위치
│   ├── launch/go2_patrol.launch.py # ★ 기본 CMD — nav 스택(액션 안쪽 리맵) + detector + tracker + manager
│   ├── params/nav2_params.yaml   # 상류 stock 파라미터 + 측정 근거를 단 go2 델타 11개
│   ├── maps/                     # 반입된 창고 점유맵 + 출처·digest (maps/README.md)
│   └── behavior_trees/           # stock 트리에서 Spin 회복만 뺀 사본 (근거: 제자리 선회 6%)
├── src/go2_detector/             # ament_python — YOLO11n CPU 검출 노드 (미션 무지)
│   ├── go2_detector/detection_logic.py   # 순수 로직(스로틀·whitelist·bbox·디코드) — ROS/torch 무의존
│   ├── go2_detector/detector_node.py     # /camera/image_raw -> /detections
│   └── test/                             # 유닛테스트 (python3 -m pytest test)
├── src/go2_target_tracker/       # ament_cmake C++ — 픽셀을 장소로 바꾸는 유일한 지점
│   ├── include/…/projection.hpp  # 순수 수학(역투영·depth median·연관·LPF) — ROS 무의존
│   ├── src/target_tracker_node.cpp       # /detections + depth + camera_info + TF -> /targets
│   └── test/test_projection.cpp          # gtest 15
└── src/go2_patrol_manager/       # ament_cmake C++ — 미션 상태기계
    ├── include/…/patrol_logic.hpp        # 순수 결정(스탠드오프·화면조건·경로 파싱)
    ├── src/patrol_manager_node.cpp       # /navigate_to_pose 서버 + /nav2/navigate_to_pose 클라이언트
    └── test/test_patrol_logic.cpp        # gtest 14
scenarios/
├── go2_t0_smoke.yaml             # T0 — 정적 1샘플 스모크 (CI 게이트)
├── go2_ta_nav_random.yaml        # TA — 출발/목표/장애물 랜덤, 5샘플·pass_ratio 0.8
├── go2_tb_patrol_chair.yaml      # TB — 순찰(표적 chair), 3샘플·pass_ratio 0.67
├── go2_tb_patrol_person.yaml     # TB — 순찰(표적 person) ⚠ 배치 경로 1/3 실패 미해결 → CI 격리(U3c)
├── go2_tb_fail_fixture.yaml      # ⚠ 의도적 실패(예산 4 s, U3c 재실측) — CI 배송 목록에 넣지 않는다
├── hold_near_goal.py             # 커스텀 오라클 — 마지막 K초 동안 goal 반경 안에 서 있었나
├── upright.py                    # 커스텀 오라클 — 전 구간 전도 없음(roll/pitch 한계)
├── test/                         # 오라클 픽스처 테스트 (python3 -m pytest scenarios/test)
└── policy.pt                     # 보행 정책 = SUT의 두 번째 아티팩트 (요청에 실려 감)
.github/workflows/verify.yml      # 소비자 검증 워크플로 (carter 픽스처의 실전 검증본 계승)
```

### `go2_detector` — 미션을 모르는 얇은 노드

`/camera/image_raw`(`sensor_msgs/Image`) → `/detections`(`vision_msgs/Detection2DArray`). **본 것을 전부
발행한다** — 표적이 무엇인지는 tracker/manager(U3)가 정한다. 파라미터(전부 `ros2 run … --ros-args -p`로
덮어쓸 수 있다): `image_topic` · `detections_topic` · `model_path`(기본 `/opt/models/yolo11n.pt`) ·
`process_rate_hz`(5.0) · `conf_threshold`(0.25) · `class_whitelist`(**빈 값 = 전부**) · `imgsz`(640) ·
`torch_threads`(2) · `log_period_s`(10.0).

- **스로틀은 이미지 헤더 스탬프(= sim time) 기준**이고 구독 depth는 1이다. 벽시계가 느려도 오래된 프레임이
  쌓이지 않고, RTF와 무관하게 sim 시간 duty cycle이 같다.
- **CPU 전용**. SUT 컨테이너에는 GPU가 없다(D1-P2). 실측 지연(이미지 안, 640 px, i9-13900K):
  1스레드 44.9 ms · **2스레드 25.4 ms(기본값)** · 4스레드 15.4 ms · 8스레드 10.5 ms — 5 Hz 예산 200 ms 대비
  여유 큼(가정 A10 재확인, 이번엔 **CPU 휠**로).
- **런타임 다운로드 0.** 가중치는 빌드 때 봉인되고, 검증도 `--network none`에서 돌렸다.

### 순찰 미션 — 층 세 개, 각자 자기 것만 안다

```
/camera/image_raw ─► go2_detector ─/detections─► go2_target_tracker ─/targets─► go2_patrol_manager
   (미션 무지)                        (픽셀→장소, 시간 필터)              (미션 상태기계)
                                                                              │
cv-infra ──NavigateToPose @ /navigate_to_pose──────────────────────────────────┘
                                        go2_patrol_manager ──@ /nav2/navigate_to_pose──► nav2
```

- **`go2_target_tracker`**: bbox 중심 픽셀 + depth(32FC1) + `camera_info` + TF → **map 프레임 표적**.
  같은 자리에서 `min_hits`(3, 5 Hz = 0.6 s)회 이상 본 것만 **확정**해 `/targets`(PoseArray)로 낸다 —
  한 프레임 conf는 이 SUT에서 증거가 못 된다(같은 의자가 구도에 따라 0.22~0.80, AR-24). depth는
  bbox 중심 **5×5 median**(좌석-등받이 틈으로 뒤 벽을 읽는 것을 무시), 이미지 스탬프에 **가장 가까운
  depth 프레임**을 고른다(검출은 25~500 ms 늦게 도착한다 — 실측).
- **`go2_patrol_manager`**: 플랫폼-facing `/navigate_to_pose` 액션 서버. `RECEIVED → PROBE →
  SEARCHING(주행 스윕) → APPROACHING(라인업 3.2 m → 표적 전면 **2.0 m 스탠드오프 링**) →
  HOLDING(화면 조건 5 s) → 성공`. **제자리 회전·크립에 의존하지 않는다**(이 정책은 제자리 yaw를 6 %,
  0.2 m/s 미만 명령을 5~23 %만 실행한다 — AR-16/18): 도착 헤딩은 *주행 방향*에서 나오고, 마지막
  구간은 표적을 향해 직진하다가 링에서 **취소**해 세운다. `/cmd_vel` 생산자는 **끝까지 nav2 하나**다.
  - **2.0 m는 측정값이다.** 기하학은 1.2 m를 권했지만(프레임에 온전·크게) 라이브에서 HOLD가 성립하지
    않았다 — 실측 결과 이 detector는 의자를 **1.7~3.2 m에서 0.79~0.87**로, **1.3 m 이하에서는
    `bench`로** 읽는다(AR-24의 정량화). TB 시나리오의 goal도 같은 2.0 m 지점이다.
- **카메라가 없는 시나리오(T0/TA)** 에서는 manager가 5 s 안에 인지 부재를 확인하고 **NAV-ONLY로
  강등**되어 받은 goal로 그냥 주행한다(로그로 크게 알린다). 이미지는 하나·CMD도 하나이므로
  (`sut`에 command 필드가 없다) 미션 모양은 **시나리오가 무엇을 선언했는지**로 갈린다.

## SUT 계약 — 이 블랙박스가 무엇을 노출하나

cv-infra는 이 컨테이너 내부를 **수정하지 않는다**(REQ-EXEC-004/005). 검증이 성립하려면 SUT가 다음을 만족해야 하고,
그 책임은 전적으로 이 저장소에 있다.

| # | 계약 | stage-0에서 누가 제공하나 |
|---|---|---|
| 1 | `use_sim_time:=true` 수용 · 외부 `/clock`으로 동작 (REQ-EXEC-003) | 이미지 기본 CMD가 `use_sim_time:=true`로 기동 |
| 2 | goal 인터페이스 `nav2_msgs/action/NavigateToPose @ /navigate_to_pose` (REQ-EXEC-007) | **`go2_patrol_manager`** 가 이 액션을 직접 노출하고 nav2 것은 `/nav2/navigate_to_pose` 로 리맵(D4) — 라이브 확인(`ros2 node info`) |
| 3 | `/cmd_vel` 발행 + 센서 토픽 구독 (REQ-EXEC-006) | nav2 controller/collision monitor → `/cmd_vel`, `/scan`·`/odom` 구독 |
| 4 | headless (rviz/GUI 없음) | 런치가 구조적으로 rviz를 띄우지 않음 (M8 §3.9 D-O) |

### SUT = 이미지 하나가 아니다

go2의 **SUT = {앱 이미지 `sut.image_ref`, 보행 정책 `sut.locomotion_policy: {file, sha256}`}** (결정
`2026-08-31-go2-model-contract-and-autonomous-run` D2). 둘 다 검증 요청에 실려 올라가고 둘 다 sha로 핀·기록된다.

- **보행 정책은 이 이미지에 들어가지 않는다.** 실물에서 로봇 몸 온보드에서 도는 것이므로 시뮬에서도 로봇 몸(=러너
  in-process)에서 돈다는 *거울 규칙*(D3) 때문이다. 파일은 시나리오 옆에 두고 요청에 실어 보낸다.
- **인지 모델(yolo11n)은 반대로 이미지에 봉인한다**(U2, D1-A). 모델이 이미지 밖에 있으면 "같은 요청 · 같은 digest인데
  행동이 달라지는" 회귀가 생기기 때문 — 봉인하면 모델 변화가 이미지 digest 변화로 드러난다. 추론은 **CPU**이고
  SUT 컨테이너에는 **GPU를 주지 않는다**(D1-P2).

## 로컬 개발 워크플로 (1급 요구)

이 앱은 cv-infra 안에서만 도는 물건이면 안 된다. 개발자는 **로컬 ROS + Isaac Sim에서 먼저 확인하고** push/PR 한다
(마스터 플랜 §1-8). 그래서 인터페이스는 전부 평범한 ROS 2 계약이고, 런치는 cv-infra 없이도 그대로 뜬다.

```bash
# 1) 도커 없이 로컬 빌드 (호스트에 ROS 2 Jazzy + vision_msgs + navigation2 필요)
#    ⚠ U3부터 C++ 패키지 2개가 `vision_msgs`(tracker)와 `nav2_msgs`(manager)를 요구한다.
#      `ros-base`만 깔린 호스트에는 둘 다 없다 — 이미지는 Dockerfile이 핀으로 설치한다.
#      의존이 없으면 `go2_detector`만 빌드되고 나머지는 CMake find_package에서 멈춘다.
source /opt/ros/jazzy/setup.bash
cd robot_sw && colcon build --symlink-install && source install/setup.bash

# 1b) 순수 로직 유닛테스트 — ROS도 모델도 GPU도 필요 없다 (가장 빠른 루프)
cd robot_sw/src/go2_detector && python3 -m pytest test -q          # detector 18
python3 -m pytest scenarios/test -q                                # 오라클 31 (cv_infra 없어도 돈다)
g++ -std=c++17 -I robot_sw/src/go2_target_tracker/include \
    robot_sw/src/go2_target_tracker/test/test_projection.cpp -lgtest -lgtest_main -pthread -o /tmp/t && /tmp/t
#   (ROS 채널에서는) colcon test --packages-select go2_target_tracker go2_patrol_manager  # gtest 29

# 2) 플랫폼의 dev-world 모드로 같은 월드·센서·보행 정책을 띄운다 (미션 구동·판정 없이 유지).
#    검증 잡과 같은 씬·같은 정책·같은 토픽 표면이고, 같은 admit 게이트를 통과한다.
#    (러너 이미지 안에서) ./python.sh -m cv_infra.runner.devworld scenarios/go2_t0_smoke.yaml
#    앱은 같은 docker 네트워크 + 같은 ROS_DOMAIN_ID 로 띄우면 끝이다.
#    ⚠ 랜덤 축이 있는 문서(TA/TB)를 dev-world에 그대로 주지 마라 — dev-world는 표본을 파생하지
#      않는다. 한 표본을 보고 싶으면 `contract.derive.materialize_request(req, i)` 로 뽑아 쓰되
#      `derivation:` 블록은 지워야 한다(플랫폼은 그 스탬프가 박힌 제출 문서를 거부한다 — 실측).

# 3) 앱을 붙여 확인 (맵은 패키지에 들어 있으므로 인자 없이 localization 포함으로 뜬다)
ros2 launch go2_bringup go2_patrol.launch.py use_sim_time:=true    # 순찰 전체 (기본 CMD와 동일)
ros2 launch go2_bringup go2_nav.launch.py    use_sim_time:=true    # 항법만 (U1 단계)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -6.0, y: 4.4}, orientation: {z: 0.7071, w: 0.7071}}}}"
#   순찰 런치에서는 이 액션의 서버가 patrol_manager 다. 확인:
#   ros2 node info /go2_patrol_manager   ->  Action Servers: /navigate_to_pose
#   ros2 node info /bt_navigator         ->  Action Servers: /nav2/navigate_to_pose

# 맵 없는 시뮬 상대로 나머지 스택만 띄우고 싶으면:
ros2 launch go2_bringup go2_nav.launch.py use_localization:=False
#   ⚠ `map:=''` 는 쓸 수 없다 — ros2 launch CLI 가 빈 값을 거부한다(2026-09-01 실측).

# 4) 이상 없으면 브랜치 push → PR → CI가 이미지를 빌드해 GHCR에 올리고 cv-infra가 검증 결과를 PR에 되돌려준다
```

컨테이너로 확인할 때:

```bash
docker build -t go2-sut:local robot_sw/       # 5개 시나리오가 선언하는 그 태그 (U3c에서 통일)
docker run --rm go2-sut:local                 # 기본 CMD = go2_patrol.launch.py use_sim_time:=true
docker run --rm go2-sut:local ros2 launch go2_bringup go2_patrol.launch.py use_detector:=False
#   use_detector:=False = 인지가 죽은 리허설. manager가 NAV-ONLY로 강등되는 경로를 그대로 탄다.
```

## 시나리오를 CI 게이트에 올리는 법 (⚠ 함정)

`scenarios/`에 파일을 두는 것만으로는 **조용히 안 돌아간다.** GPU 잡은 PR 소스를 체크아웃하지 않으므로(R10),
`.github/workflows/verify.yml`의 `Stage verification inputs` 배송 목록에 `cp` 줄을 추가한 파일만 검증에 존재한다(G-94).
지금 배송되는 것은 **`go2_t0_smoke.yaml` · `go2_ta_nav_random.yaml` · `go2_tb_patrol_chair.yaml` ·
`hold_near_goal.py` · `upright.py` · `policy.pt`** 여섯이다. 특히 `policy.pt`는
이미지 안에 없고 요청에 실려 가는 **SUT의 두 번째 아티팩트**이고, 오라클 `.py` 2개는 TB 시나리오가
`module:Class` 로 참조하는 파일이라 — 셋 중 하나라도 빠지면 그 시나리오는 admit에서 거부된다(exit 2).
**배송하지 않는 문서 2종**:
- `go2_tb_fail_fixture.yaml` — **설계상** 배송하지 않는다(의도적 실패 문서라 매 PR을 빨갛게 만든다 —
  carter 픽스처가 자기 obstacle_fail 문서에 대해 내린 것과 같은 판단).
- `go2_tb_patrol_person.yaml` — **설계가 아니라 미해결 결함 때문에 격리**했다(U3c). 첫 제품 경로 배치
  런에서 3표본 중 1표본만 통과했고, 실패한 두 표본은 **restage 된 표본**에서 `/cmd_vel` 이 ~1.2 s에
  **0.04 m/s로 붕괴**(보행 정책 데드존 AR-16/18 안쪽)해 180 s 예산을 서 있는 채로 소진했다. 같은 spawn ·
  같은 기하를 단일 잡으로 3회 재현했으나 **전부 통과** — 원인은 배치/restage 경로에 있고 이 저장소가
  닫을 수 없다(B-11 + M2). **`min_pass_ratio` 는 손대지 않았다**(0.67). 측정·근거·복구 방법은 그 문서
  머리말의 KNOWN LIMITATION 블록과 워크플로 주석에 있다.

목록이 비면 검증 잡 자체가 skip되고 워크플로가 warning으로 크게 알린다.

## 재현성 (환경 핀 — CLAUDE.md §2-7)

| 자산 | 핀 | 확인 |
|---|---|---|
| 베이스 이미지 | `ros:jazzy-ros-base-noble@sha256:31daab66…` (carter 픽스처와 **동일 digest**) | 2026-09-01 `docker buildx imagetools inspect`로 실존 확인 |
| Nav2 런타임 | apt `ros-jazzy-navigation2=1.3.12-1noble.20260615.181551` | 2026-09-01 핀 설치 성공 |
| nav2_bringup(런치 전용) | apt `ros-jazzy-nav2-bringup=1.3.12-1noble.20260616.082701` | 2026-09-01 핀 설치 성공 |
| Nav2 파라미터 | 위 deb의 stock `nav2_params.yaml`(sha256 `299f9002…`) + go2 델타 10개 | 파일 헤더의 델타 표에 stock값·채택값·근거 |
| Nav2 BT XML | stock `navigate_to_pose_w_replanning_and_recovery.xml`(sha256 `5895b638…`) − `<Spin>` 1줄 | 파일 헤더에 출처·delta·stock digest |
| 정적 점유맵 | `carter_navigation` @ `50de0035…`(태그 `IsaacSim-5.1.0`) 원본 바이트 그대로 | `robot_sw/src/go2_bringup/maps/README.md`(URL·커밋·digest·실측 대조) |
| 보행 정책 | `policy.pt` sha256 `73338e49…` (174,184 B) — Isaac Lab `go2_flat` 사전학습 export | 시나리오 `sut.locomotion_policy`가 digest를 핀; 플랫폼이 admit·load 두 번 재검증 |
| vision_msgs / pip | apt `ros-jazzy-vision-msgs=4.1.1-3noble.20260615.113052` · `python3-pip=24.0+dfsg-1ubuntu1.3` | 2026-09-01 핀 설치 성공 (ros-base엔 둘 다 없다) |
| torch / torchvision | `2.8.0+cpu` / `0.23.0+cpu` (CPU 전용 인덱스) | 빌드가 CUDA 빌드를 **거부**한다(어서션) |
| ultralytics + 전체 pip 트리 | `8.4.136` + `constraints-detector.txt`의 32개 전량 핀 | 파일 헤더에 생성 절차·AR-11 근거 |
| yolo11n 가중치 | URL + sha256 `0ebbc80d…` (5,613,764 B) → 이미지에 봉인 | 빌드 중 `sha256sum -c` 실패 시 **빌드 중단** |
| GitHub Action | 전부 commit SHA 핀 (플랫폼만 릴리즈 태그 `@v1`) | carter 픽스처에서 계승 |
| **빌드 컨텍스트** | `robot_sw/.dockerignore` — 워킹트리 잡동사니(`**/__pycache__`·`**/.pytest_cache`) 제외 | U3c 실측: 없을 때 워킹트리 빌드가 `/opt/go2_ws/src` **35파일**, 클린 export 빌드가 **27파일**이었다(호스트 python 3.10 `.pyc`가 이미지에 들어갔다). 추가 후 27 == 27 sha256 일치 |

## 경계 규칙

- 플랫폼(`cv-infra-workspace`)은 **계약 + 릴리즈 이미지 + `verify.yml@v1`** 로만 소비한다. 상대경로 소스 참조는 없다.
- Scenario / `adapter_config` **스키마는 플랫폼 소유**(M1). 이 저장소는 인스턴스만 만든다. go2의 `adapter_config`는
  플랫폼이 실측한 go2 토픽 인벤토리로 작성했다(**carter 것 복붙 아님** — 토픽 수·이름·프레임이 다르다).
- 이 저장소는 새 플랫폼 REQ ID를 만들지 않는다 — 확정된 86개 요구사항을 **행사**할 뿐이다.

## 로드맵

| 사이클 | 내용 | 선행 |
|---|---|---|
| **U0** (완료) | 저장소 부트스트랩 + stage-0 이미지 + 브링업 골격 + 워크플로 골격 | — |
| **U1** (완료) | 맵 반입 · T0/TA 시나리오 · Nav2/AMCL 튜닝 · 정책 파일 반입 · dev-world 상대 풀스택 수동 검증 | 플랫폼 C1(맵)·C2b(정책)·C3(토픽/dev-world) |
| **U2** (완료) | `go2_detector`(yolo11n CPU, 이미지 봉인) + pip 전량 핀 + 오프라인/라이브 검증 | C3 프레임 |
| **U3** (완료) | `go2_target_tracker`·`go2_patrol_manager`(C++) + `go2_patrol.launch.py` + TB 시나리오 3종 + 커스텀 오라클 2종 + dev-world 행동 실증 | U2 |
