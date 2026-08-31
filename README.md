# cv-infra-user-go2

CV-Infra의 **두 번째 소비자 / E2E 픽스처** (public). 첫 번째(`cv-infra-user`, Nova Carter 휠 로봇)와 달리 여기서는
**Unitree Go2 사족보행 순찰 앱**을 SUT로 올려, 같은 플랫폼 계약이 *다른 로봇 · 다른 미션 · 다른 산출물 구성*에서도
러너 수정 0으로 통하는지를 소비자 입장에서 행사한다.

> **현 상태: stage-0 부트스트랩(U0).** 지금 있는 것은 **내비게이션 전용 SUT 이미지**(Nav2 + AMCL/map_server)와
> 그 브링업 패키지, 그리고 소비자 워크플로 골격뿐이다. **시나리오·맵·`adapter_config`는 아직 없다** — 맵은 플랫폼
> 사이클 C1의 산출물, 토픽 인벤토리는 C3의 산출물이라 값을 지어내지 않고 기다린다(§로드맵). 그 전까지 PR을 열면
> SUT 이미지 빌드/GHCR push까지만 돌고 **GPU 검증 잡은 건너뛴다**(빈 배송 목록 가드).

## 저장소 레이아웃

```
robot_sw/
├── Dockerfile                    # stage-0 SUT 이미지 (ros:jazzy @digest + Nav2 핀)
├── ros_entrypoint.sh             # ROS + go2_bringup 오버레이 source 후 exec
└── src/go2_bringup/              # 설치 전용 ament 패키지 (컴파일 0)
    ├── launch/go2_nav.launch.py  # nav2_bringup 재사용 + 맵 없으면 localization off
    └── params/nav2_params.yaml   # 상류 stock 파라미터 + go2 치수 1곳만 수정
.github/workflows/verify.yml      # 소비자 검증 워크플로 (carter 픽스처의 실전 검증본 계승)
```

U2에서 `src/go2_detector`(Python·ultralytics CPU), U3에서 `src/go2_target_tracker`·`src/go2_patrol_manager`(C++)와
`scenarios/`가 추가된다.

## SUT 계약 — 이 블랙박스가 무엇을 노출하나

cv-infra는 이 컨테이너 내부를 **수정하지 않는다**(REQ-EXEC-004/005). 검증이 성립하려면 SUT가 다음을 만족해야 하고,
그 책임은 전적으로 이 저장소에 있다.

| # | 계약 | stage-0에서 누가 제공하나 |
|---|---|---|
| 1 | `use_sim_time:=true` 수용 · 외부 `/clock`으로 동작 (REQ-EXEC-003) | 이미지 기본 CMD가 `use_sim_time:=true`로 기동 |
| 2 | goal 인터페이스 `nav2_msgs/action/NavigateToPose @ /navigate_to_pose` (REQ-EXEC-007) | nav2 `bt_navigator` (U3부터는 `patrol_manager`가 이 액션을 직접 노출하고 nav2 것은 내부로 리맵 — D4) |
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
# 1) 도커 없이 로컬 빌드 (호스트에 ROS 2 Jazzy 필요)
source /opt/ros/jazzy/setup.bash
cd robot_sw && colcon build --symlink-install && source install/setup.bash

# 2) 플랫폼의 dev-world 모드로 같은 월드·센서·보행 정책을 띄운다 (미션 구동·판정 없이 유지)
#    ⚠ dev-world 엔트리포인트는 플랫폼 사이클 C3 산출물 — 아직 없다. 나오면 여기에 명령을 적는다.

# 3) 앱을 붙여 확인 (맵이 없으면 localization은 자동으로 꺼진 채 나머지 스택만 기동한다)
ros2 launch go2_bringup go2_nav.launch.py use_sim_time:=true
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"   # 평범한 액션이라 CLI로도 구동

# 4) 이상 없으면 브랜치 push → PR → CI가 이미지를 빌드해 GHCR에 올리고 cv-infra가 검증 결과를 PR에 되돌려준다
```

컨테이너로 확인할 때:

```bash
docker build -t go2-sut robot_sw/
docker run --rm go2-sut ros2 launch go2_bringup go2_nav.launch.py use_sim_time:=true
```

## 시나리오를 CI 게이트에 올리는 법 (⚠ 함정)

`scenarios/`에 파일을 두는 것만으로는 **조용히 안 돌아간다.** GPU 잡은 PR 소스를 체크아웃하지 않으므로(R10),
`.github/workflows/verify.yml`의 `Stage verification inputs` 배송 목록에 `cp` 줄을 추가한 파일만 검증에 존재한다(G-94).
지금 그 목록은 **비어 있고**, 비어 있는 동안에는 검증 잡 자체가 skip되며 그 사실을 워크플로가 warning으로 크게 알린다.

## 재현성 (환경 핀 — CLAUDE.md §2-7)

| 자산 | 핀 | 확인 |
|---|---|---|
| 베이스 이미지 | `ros:jazzy-ros-base-noble@sha256:31daab66…` (carter 픽스처와 **동일 digest**) | 2026-09-01 `docker buildx imagetools inspect`로 실존 확인 |
| Nav2 런타임 | apt `ros-jazzy-navigation2=1.3.12-1noble.20260615.181551` | 2026-09-01 핀 설치 성공 |
| nav2_bringup(런치 전용) | apt `ros-jazzy-nav2-bringup=1.3.12-1noble.20260616.082701` | 2026-09-01 핀 설치 성공 |
| Nav2 파라미터 | 위 deb의 stock `nav2_params.yaml`(sha256 `299f9002…`) + go2 치수 1곳 | 파일 헤더에 출처·델타 기록 |
| GitHub Action | 전부 commit SHA 핀 (플랫폼만 릴리즈 태그 `@v1`) | carter 픽스처에서 계승 |

## 경계 규칙

- 플랫폼(`cv-infra-workspace`)은 **계약 + 릴리즈 이미지 + `verify.yml@v1`** 로만 소비한다. 상대경로 소스 참조는 없다.
- Scenario / `adapter_config` **스키마는 플랫폼 소유**(M1). 이 저장소는 인스턴스만 만든다. go2의 `adapter_config`는
  C3의 토픽 인벤토리가 나온 뒤에 작성한다(**carter 것 복붙 금지**).
- 이 저장소는 새 플랫폼 REQ ID를 만들지 않는다 — 확정된 86개 요구사항을 **행사**할 뿐이다.

## 로드맵

| 사이클 | 내용 | 선행 |
|---|---|---|
| **U0** (완료) | 저장소 부트스트랩 + stage-0 이미지 + 브링업 골격 + 워크플로 골격 | — |
| U1 | T0 스모크 시나리오 · 맵 배선 · 파라미터 튜닝 · dev-world 상대 수동 주행 | 플랫폼 C1(맵)·C3(토픽/dev-world) |
| U2 | `go2_detector` (yolo11n CPU, 이미지 봉인) | C3 프레임 |
| U3 | `go2_target_tracker` + `go2_patrol_manager` + TB 시나리오 + 커스텀 오라클 2종 | U2 |
