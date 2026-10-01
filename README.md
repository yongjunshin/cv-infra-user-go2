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
verify/     CI 검증만: 케이스 실행 스크립트 + 입력 공간 + 판정자 + 검증 이미지 (아래 §cv-infra 검증)
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

PR을 열면 GitHub Actions가 [cv-infra](https://github.com/yongjunshin/cv-infra-workspace)의
재사용 워크플로를 호출한다. 플랫폼은 `verify/space.pict`를 페어와이즈 커버링 배열로 펼쳐
**케이스마다 컨테이너 하나**를 띄우고 그 안에서 `verify/sim.py --<축>=<값> …`을 돌린 뒤,
`verify/oracle.py`의 판정을 PR에 Check · sticky 코멘트 · 아티팩트(케이스별 `verify/out` zip +
컨테이너 로그)로 돌려준다. 플랫폼은 여기서 무엇이 순찰이고 무엇이 표적인지 모른다 — 축 이름을
argv로 넘기고, `verify/out`을 거두고, 오라클이 찍은 평평한 JSON 한 줄의 **타입**만 읽는다.

### 검증하는 질문: "찾으라는 것을 — 그리고 그것만 — 찾는가?"

케이스마다 앱에 `/patrol` 골을 **딱 한 번**, 클래스 이름만 담아 보낸다(`{target_class: chair}`).
어디에 있는지는 말하지 않는다 — 앱 인터페이스가 원래 그렇다. 세계에는 그 클래스의 물체가 네 곳
중 한 곳에 숨어 있거나 **아예 없고**, 다른 클래스의 미끼(decoy)가 서 있을 수도 있다.

```
 H3 (-5, 10)                     H4 (3, 13)
                decoy (0, 8.5)
                                       H2 (4, 6)
       H1 (-3, 4)
 desk a (-6, 3)        desk b (2, 3)               (있으면 2.8 m 폭으로 가로놓임)
 S1 (-6, -1)           S2 (0, 1.5)          S3 (6, 0)        모두 +y를 향함
```

| 경우 | pass 조건 |
|---|---|
| 표적이 **있다** (`hide`=H1–H4) | 찾았다고 답했고, 답한 위치가 실제 위치에서 **1.0 m 안**이며, 답이 **거리에 비례한 허용 시간** 안에 나왔다: `45 s + 8 s × (출발점→표적 직선거리 m)` |
| 표적이 **없다** (`hide`=none) | **찾았다고 답하지 않았고**, 최소 **150 s** 동안 찾다가 포기했다(5초 만에 그만둔 미션은 찾아본 게 아니다) |

허용 시간과 관찰 시간은 **이 저장소가 앱에 요구하는 것**(테스트의 요구사항)이지 앱을 잰 값이 아니다.
시간은 전부 시뮬레이터 시계(sim-s)로, 골 수락부터 답까지다.

### 파일 4개 + 워크플로 1개

| 파일 | 무엇 |
|---|---|
| `verify/Dockerfile` | **검증 이미지.** 핀된 Isaac Sim 5.1.0 위에 `robot_sw/`를 그대로 구워 올린다(`robot_sw/Dockerfile`의 레이어·핀을 그대로 재생). 케이스 컨테이너는 하나뿐이라 시뮬과 앱이 같은 이미지에 있어야 한다. **`verify/sim.py`·`sim/patrol_world.py`·보행 정책은 굽지 않는다** — 런타임에 체크아웃에서 읽는다. |
| `verify/sim.py` | **표준 Isaac standalone 스크립트 = 테스트 하네스.** ① 앱의 세계 `sim/patrol_world.py`를 **고치지 않고 모듈로 import**해 창고·Go2·보행 정책·센서 리그를 그대로 세우고 ② 케이스의 물체(표적·미끼·책상)를 놓고 ③ 탑뷰 사진을 찍고 ④ 앱을 이미지의 기본 명령 그대로 띄우고(깨끗한 환경으로 — 아래 ⚠) ⑤ 로봇의 현재 위치를 `/initialpose`로 알려 준다(운영자가 RViz에서 "2D Pose Estimate"를 찍는 것과 같다) ⑥ 앱이 **완전히 뜬 뒤**(nav2 lifecycle 두 개 모두 active + 2 s) `/patrol` 골을 보내고 ⑦ 답이 올 때까지(최대 240 sim-s) **실제 걸은 궤적만 기록**한다. 끝에 `verify/out/`에 `run.json`·`trajectory.csv`·`mission.txt`·`app.log`와 사진 두 장을 쓴다. 표적 위치는 앱에 절대 주지 않는다. |
| `verify/space.pict` | **입력 공간**(PICT 문법). 축 6개(`start`·`target`·`hide`·`decoy`·`slot_a`·`slot_b`) = `verify/sim.py`의 플래그. k=2에서 **15 케이스**(그중 표적이 없는 케이스 3개). |
| `verify/oracle.py` | **판정.** 같은 이미지·같은 argv로(GPU 없이) 돌며 `run.json`·`trajectory.csv`를 읽고 평평한 JSON 한 줄을 낸다. stdlib만 쓴다. |
| `.github/workflows/verify.yml` | 잡 하나(`uses: …@main`)와 `with:` 입력 9개. 이 저장소가 유지하는 통합 표면 전부. |

`verify/out/.gitkeep`은 플랫폼이 체크아웃을 **읽기 전용**으로 마운트하고 이 경로에만 케이스별
디렉토리를 읽기·쓰기로 덮기 때문에 커밋돼 있다. 내용물은 `.gitignore`가 막는다.

### 판정은 타입으로 말한다

| 키 | 타입 | 뜻 |
|---|---|---|
| `report_correct` | bool(체크) | 있으면: 찾았고 위치 오차 ≤ 1.0 m. 없으면: 찾았다고 하지 않았다 |
| `in_time` | bool(체크) | 있으면: 허용 시간 안에 찾았다. 없으면: 150 s 이상 찾다가 포기했다 |
| `mission_s` | 숫자(지표) | 골 수락부터 답까지 sim 초 |
| `allowed_s` · `target_dist_m` | 숫자/`null` | 이 케이스의 허용 시간과 출발점→표적 거리. 표적이 없으면 `null` |
| `report_error_m` | 숫자/`null` | 답한 위치와 실제 표적의 거리. 찾지 못했으면 `null` |
| `path_len_m` | 숫자 | 실제로 걸은 거리 |
| `note` | 문자열 | 무엇을 어디서 찾으라 했고, 무엇이라 답했고, 앱이 남긴 이유 |

`run.json`이 없으면 하네스가 미션까지 가지 못한 것이므로 ERROR 레인(판정 없음)이다 — 로봇이
실패한 것과는 다르다.

### 케이스마다 사진 두 장 — 하네스가 찍고 덧그린다

`verify/out/topview_initial.png`는 표적·미끼·책상을 놓고 로봇을 세운 직후 창고 전체를 60 m 위에서
찍은 **실제 렌더**다(지붕·조명은 렌더에서만 숨김). `verify/out/topview_result.png`는 그 위에 출발점
(청록 원), **진짜 표적**(초록 원), 미끼(주황 원), 책상(노란 상자), **실제 걸은 경로**(청록 선),
**앱이 답한 표적 위치**(분홍 ×), 상단에 질문과 답을 그린 것이다. 오라클은 출력 디렉토리를 읽기
전용으로 받으므로 그림은 하네스가 그린다. 사진은 판정에 쓰이지 않는다. carter 예시와 같은 건물이라
카메라 보정도 같다(이 세계에서 다시 확인).

> ⚠ **exit code는 판정이 아니다.** 하네스는 `sim/patrol_world.py`처럼 `os._exit`로 끝난다(`close()`는
> 무슨 일이 있었든 status 0으로 끝내고 종료 정리에서 멈출 수 있다). 0 = 증거를 썼다, 1 = 세계나 앱이
> 안 떴다(ERROR), 2 = argv가 틀렸다. pass/fail은 전부 `verify/oracle.py`가 정한다.

> ⚠ **한 컨테이너, 두 ROS 2 환경.** 하네스는 시뮬레이터에 번들된 Jazzy로 DDS를 말하고, 앱은 이미지의
> `/opt/ros/jazzy`로 돈다. 번들 쪽은 라이브러리 경로가 **프로세스 시작 시점에** 있어야 하므로 하네스가
> 맨 처음 한 번 그 환경(`setup_ros_env.sh`가 내보내는 세 변수 + 공유 `ROS_DOMAIN_ID`)으로 자기 자신을
> 다시 실행한다. 앱은 하네스의 환경(Kit의 `LD_PRELOAD`·`PYTHONPATH`가 섞여 있다)이 아니라
> **화이트리스트 환경**으로 띄운다. (도메인이 갈리면 앱은 `/odom` TF를 영영 못 받는다 — 실측.)

> **탐색은 매번 다르다.** 앱의 기본 탐색은 직진-바운스 랜덤 워크이고 `bounce_seed` 기본값 0은
> 하드웨어 엔트로피로 시드한다(런치 파일이 이 파라미터를 열어 두지 않았다). 그래서 같은 케이스도 실행마다
> 경로가 다르고 결과가 갈릴 수 있다 — `repeats`를 올리면 그 흔들림이 비율로 보인다.

> 관찰: 이 세계의 센서 리그(`sim/patrol_world.py`의 `SensorRig`)가 월드 원점 (0, 0)에 작은 흰색
> 렌더 전용 도형을 남긴다(콜라이더 없음, 리그 없이 렌더하면 사라짐 — 2026-10-01 실측). 앱의 세계이므로
> 손대지 않고, 출발점을 그 위에 두지 않았다.


### 검증 이미지 빌드 · 푸시 · 다이제스트 핀

`sim_image`는 **다이제스트 핀 필수**(플랫폼이 태그를 거부한다). 빌드 컨텍스트는 **저장소 루트**다
(`verify/Dockerfile`이 `robot_sw/`를 COPY한다).

```bash
TAG=$(git rev-parse --short HEAD)
IMAGE=ghcr.io/yongjunshin/cv-infra-user-go2/go2-verify

docker build -f verify/Dockerfile -t "$IMAGE:$TAG" .
docker push "$IMAGE:$TAG"
docker inspect --format '{{index .RepoDigests 0}}' "$IMAGE:$TAG"   # -> ghcr.io/...@sha256:…
```

마지막 줄이 찍은 `name@sha256:…`을 `.github/workflows/verify.yml`의 `sim_image`에 그대로
붙여 넣는다.

> 이미지를 **처음** 바꾸면 워크스테이션의 Omniverse 캐시 트리도 새 다이제스트용으로 하나 더
> 있어야 한다(cv-infra는 Kit 셰이더·CUDA 캐시를 이미지별로 분리한다). 플랫폼은 조용히 콜드로
> 돌지 않고 필요한 `warm_cache.sh …/<digest12> provision` 명령을 그대로 찍으며 멈춘다 —
> 그 줄을 러너 호스트에서 한 번 실행하면 된다. ghcr pull 권한(러너의 `docker login ghcr.io`)도
> 같은 1회 준비물이다.

> ⚠ **`robot_sw/`를 고쳤으면 같은 PR에서 재빌드·푸시·재핀한다.** 앱이 곧 이미지이기 때문에,
> 노드를 고치고 다이제스트를 그대로 둔 PR은 **옛 앱**을 검증하고 그 diff에 대해서는 아무 말도
> 하지 않는다. `sim/`과 `robot_sw/models/`는 런타임에 체크아웃에서 읽으므로 재빌드가 필요 없다.
>
> `verify/Dockerfile`의 ARG는 전부 2026-09-23에 이 베이스 위에서 `apt-cache madison`으로 잰
> 정확한 버전이다. ⚠ `robot_sw/Dockerfile`과 **같은 문자열이 아니다**: packages.ros.org는
> datestamp 접미사를 교체하므로 robot_sw의 2026-09-01 핀은 더 이상 해석되지 않고(첫 빌드가
> 정확히 거기서 죽었다), 그 사이 nav2 자체도 1.3.12→1.3.13으로 올라갔다. 즉 검증 이미지는
> nav2 1.3.13, 2026-09-01에 빌드한 compose 리그는 1.3.12다 — robot_sw를 다음에 재빌드할 때
> 두 파일의 핀을 함께 맞춘다. 핀이 해석되지 않으면 `apt-cache madison <pkg>`로 다시 재서
> ARG를 바꾼다(`=`를 지우지 않는다).

### 로컬에서 케이스 하나 돌리기

CI가 케이스마다 하는 것과 **같은 한 줄**이다(엔트리포인트 래퍼까지 동일). 아래 축은
`verify/space.pict`가 펼쳐지는 15행 중 **한 행 그대로**다(S3에서 사람을 찾으라 — H2에 서 있다:
2026-10-01 실측으로 16.1 sim-s에 찾았고 0.32 m 오차로 답했다).

```bash
mkdir -p verify/out
docker run --rm --gpus all \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e NVIDIA_DRIVER_CAPABILITIES=all -e CV_SEED=7 \
  -v "$PWD:/cv/checkout:ro" \
  -v "$PWD/verify/out:/cv/checkout/verify/out:rw" \
  -w /cv/checkout --shm-size=8g \
  --entrypoint /bin/sh \
  ghcr.io/yongjunshin/cv-infra-user-go2/go2-verify@sha256:0abea884891a712f4aecdf6719edcb972e890c5ab8acd30b52f6017e91ea6d6e \
  -lc 'exec "$0" "$@"' verify/sim.py \
    --start=S3 --target=person --hide=H2 --decoy=none --slot_a=empty --slot_b=empty

python3 verify/oracle.py --start=S3 --target=person --hide=H2 --decoy=none --slot_a=empty --slot_b=empty
```

체크아웃이 `:ro`이고 `verify/out`만 `:rw`인 것이 플랫폼이 하는 일 그대로다 — 그래서 하네스가
쓰는 경로는 전부 체크아웃 루트 기준 상대경로다. 오라클은 stdlib만 쓰므로 노트북의 맨 `python3`로도
돈다(증거 파일만 있으면 된다).

### 검증하지 않는 것

`web/`(rosbridge + 브라우저 UI)은 검증 대상이 아니다. 케이스 컨테이너에는 호스트 네트워크도
docker 소켓도 없고, 사람이 브라우저로 하는 일을 CI가 대신 주장하지 않는다.
