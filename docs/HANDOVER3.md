# AMR 프로젝트 인수인계 (4단계 완료 — 3-B 아두이노 연결 시작 시점)

> **이 문서 하나로 맥락 복원**: 새 대화에서 이 파일을 올리면 지금까지의 진행·설계판단·다음 할 일이 모두 복원됨.
> 최종 갱신: 2026-09-13 (4단계 첫 SLAM 지도 완료 직후)

---

## 나에 대해

- 반도체·제조 장비 SW 취업 목표 (테크윙 "반도체 장비 제어 및 AGV SW개발" 공고 타겟)
- 아두이노 기본 문법, C# 중급, C++ 기초, ROS2 입문 단계
- 선호: 구조 위주 설명, 단계적 검증(한 번에 하나씩), 빠른 진행, 트레이드오프 먼저 짚어주기
- 결정 시 "엔지니어/기업 관점"으로 판단하는 걸 선호
- 리눅스 CLI는 초보. 명령어는 복사·붙여넣기 가능하게 통째로 주는 게 좋음
- `nano` 등 대화형 편집기는 Windows 터미널에서 Ctrl 조합이 안 먹힘 → **heredoc(`tee << 'EOF'`) 방식 선호**
- SSH 접속은 Windows cmd/PowerShell에서 `ssh daehan@192.168.45.100` 사용

---

## 프로젝트 1 (완료) — AMR 관제 + 예지보전

**GitHub**: github.com/jungdaehan11/amr-control (로컬: `C:\adsmartcar`)

- 아두이노 4WD 스마트카(우노 + L298N 2채널 + DC기어모터, **엔코더 없음**), 초음파(HC-SR04), 서보, 전류센서 ACS712, HC-06 블루투스
- 커스텀 패킷 프로토콜: `[STX=0x02][LEN][CMD][DATA][CHK=XOR][ETX=0x03]`
- CMD: `0x10~14` 구동, `0x20` 거리, `0x21` 전류, `0x30` 하트비트, `0x40/41` 경고
- C# WinForms 관제(GDI+ 그래프, 멀티스레드), 하트비트 워치독
- 예지보전: 전류 기반 감지 → 진단(정상/부하/이물질, 평균+표준편차) → 자동대응(서보 경고)
- TCP/IP socket 원격 관제 (브릿지 구조, 양방향, 멀티스레드)
- C++/MFC 포팅 완료 (PacketTest → EchoServer/Client → MfcControl)

**프로젝트 1은 완전히 마무리됨.**
**핵심 자산: `Packet.h`/`Packet.cpp` (C++ 포팅본) — 3-B에서 재사용 예정 (아래 C++ 증명 전략 참고)**

---

## 프로젝트 2 (진행 중) — ROS2 + 라즈베리파이 + LiDAR + SLAM

**GitHub**: github.com/jungdaehan11/amr-ros2-slam (로컬: `C:\ros2slam`)

### 목표

기존 아두이노 스마트카를 구동부로 재활용, 라즈베리파이5 + LiDAR를 얹어
ROS2 기반 2D LiDAR SLAM 자율주행 구현.
프로젝트 1의 "계층 분리 설계" 계승 — 상위(라파/ROS2)가 판단, 하위(아두이노)가 구동.

### 구조

```
[RPLIDAR A1M8] --USB--> [라즈베리파이5 / Ubuntu Server 24.04 + ROS2 Jazzy]
                                    |
                                    | USB Serial (기존 패킷 프로토콜 재사용) ← 3-B에서 연결 예정
                                    v
                            [아두이노 우노] --> [L298N] --> 모터

[개발 PC] <--WiFi--> 라즈베리파이 (SSH / RViz2 원격 시각화) ← 3-A 완료
```

### 부품

- 라즈베리파이5 4GB / 쿨링팬 케이스(온도 기반 자동제어, 정상) / 공식 5V 5A 어댑터
- RPLIDAR A1M8-R6 (2D, 360도, 12m) — 정상 작동
- SD카드 64GB (현재 6.4% 사용, 여유 충분)
- micro HDMI to HDMI (집 TV 출력 실패 — 헤드리스로 우회)
- CAT.5e 랜케이블 2m
- **micro USB 데이터 케이블 (벤선 3A)** — 라이다 전원용 (2A는 모터 안 돎, 아래 2단계 트러블슈팅)

---

## ✅ 1단계 완료 — 라즈베리파이 세팅 (2026-09-06)

| 항목 | 값 |
|---|---|
| OS | Ubuntu **Server** 24.04.4 LTS (arm64) |
| ROS2 | **Jazzy Jalisco** (`ros-jazzy-ros-base`) |
| 호스트명 | `amr` |
| 계정 | `daehan` / `jungdaehan22` |
| 고정 IP | **192.168.45.100** (WiFi `SK_F764_5G`, GW `192.168.45.1`) |
| 접속 | `ssh daehan@192.168.45.100` |

`~/.bashrc`에 `source /opt/ros/jazzy/setup.bash` 등록됨.

**설계 판단 (면접 어필)**
- **Ubuntu Server 선택**: 부팅 RAM Desktop~1.2GB vs Server~300MB → SLAM/Nav2에 배분. RViz2는 개발 PC 원격. 표현/연산 계층 분리.
- **ROS2 Jazzy 선택**: Ubuntu 24.04 페어링 LTS(EOL 2029-05), Nav2·slam_toolbox 레퍼런스 두터움.

**1단계 트러블슈팅**: MAC 벤더 코드 오판(Pi5는 `88:a2:9e`) / apt 의존성 충돌(aptitude 해결) / HDMI 실패(헤드리스 우회)

---

## ✅ 2단계 완료 — LiDAR 연결 + /scan (2026-09-12)

- RPLIDAR A1M8 → `/scan` 약 7.47Hz 정상 발행
- **udev 규칙으로 포트 고정**: `/dev/rplidar` (CP2102, idVendor=`10c4`/idProduct=`ea60`/serial=`0001`)
- **colcon workspace 첫 생성** + `sllidar_ros2` 빌드 (`~/ros2_ws`)
- 라이다 frame_id = `laser` (확인됨)

**2단계 트러블슈팅 (핵심)**: "데이터는 되는데 모터만 안 돎" → 소프트웨어 100% 배제(SDK·DTR·직접명령 다 시도) → **micro USB 케이블 전류 부족**(2A→3A 교체로 해결). Pi 전압 정상이라 `throttled=0x0`엔 안 잡힘.
- 부수 확보물: `~/rplidar_sdk` (ROS2 없이 라이다 최저레벨 테스트: `ultra_simple`)

**실행 명령 (라이다)**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/rplidar
```

---

## ✅ 3-A 완료 — 개발 PC 원격 RViz2 시각화 (2026-09-13)

개발 PC(노트북, Windows 11 25H2)에서 Pi의 `/scan`을 원격 시각화 성공.

### 환경
- **WSL2 + Ubuntu 24.04 + ROS2 Jazzy Desktop** (Pi와 동일 Jazzy — DDS 호환 필수)
- WSL 계정도 `daehan` / `jungdaehan22`
- **미러 네트워크 모드** (`.wslconfig`에 `networkingMode=mirrored`) → WSL이 노트북과 같은 대역(192.168.45.219)에 노출 → DDS 자동 발견
- WSL `/scan` 약 7.5Hz 수신 확인, RViz2 점군 시각화 성공

### 3-A 트러블슈팅
1. **WSL 배포판 설치 중단**(`WSL_E_DISTRO_NOT_FOUND`) → 재부팅 후 `wsl --install -d Ubuntu-24.04` 재실행
2. **DDS 단방향**(talker는 가는데 listener가 못 받음) → Windows 방화벽 인바운드 차단.
   해결(관리자 PowerShell): `New-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`

**문서화됨**: 저장소 `docs/SETUP_WSL2_RVIZ2.md` (재현 가이드, git push 완료)

### WSL RViz2 실행 (매번)
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
rviz2      # Fixed Frame → 스캔만 볼 땐 laser / 지도 볼 땐 map
```

---

## ✅ 4단계 완료 — 첫 SLAM 지도 (2026-09-13)

**손으로 라이다 들고 매핑**해서 방 하나 지도 작성 성공. 저장·git push 완료.
(아두이노 미연결 상태라 로봇 자율주행 아님. "파이프라인이 돈다" 검증 + 눈에 보이는 성과가 목적)

### 최종 파이프라인 (동작 확인됨)
```
라이다(/scan)
  → static TF (base_link→laser)
  → rf2o_laser_odometry (odom→base_link, publish_tf:=true)
  → slam_toolbox (map→odom, /map 발행)
  → WSL RViz2 (map 표시)
```
TF 트리: `map → odom → base_link → laser`

### 설치한 것 (Pi)
- `sudo apt install ros-jazzy-slam-toolbox` (의존성 충돌 → aptitude로 해결, 아래 참고)
- `rf2o_laser_odometry` 소스 빌드 (`~/ros2_ws/src/`에 clone 후 colcon)
  - repo: https://github.com/MAPIRlab/rf2o_laser_odometry.git
  - launch가 내는 odom 토픽은 `/odom_rf2o`, base=base_link, odom_frame=odom, publish_tf=True (freq 20)
- `ros2-jazzy-nav2-map-server` (지도 저장 map_saver_cli 용, 이미 있었음)

### 4단계 실행 순서 (창 4개 + WSL) — 재현용
```bash
# 창 1 — 라이다
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/rplidar

# 창 2 — static TF (base_link→laser, 손매핑이라 값 0)
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser

# 창 3 — rf2o
source ~/ros2_ws/install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py

# 창 4 — slam_toolbox (★ 반드시 수정한 파라미터 파일로, base_link)
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/daehan/ros2_ws/config/my_mapper.yaml

# WSL — RViz2 (Fixed Frame=map, Add: Map(/map) + LaserScan(/scan))
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
rviz2
```

**수정한 파라미터 파일 (`~/ros2_ws/config/my_mapper.yaml`) — 만든 방법:**
```bash
mkdir -p ~/ros2_ws/config
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml ~/ros2_ws/config/my_mapper.yaml
sed -i 's/base_footprint/base_link/g' ~/ros2_ws/config/my_mapper.yaml
```

### 지도 저장 & git
```bash
# Pi에서 저장
mkdir -p ~/ros2_ws/maps && cd ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f my_first_map   # .pgm + .yaml (192x155 @0.05m/pix)

# 개발 PC PowerShell에서 scp로 가져와 커밋 (지도는 Pi, git은 C:\ros2slam)
scp daehan@192.168.45.100:~/ros2_ws/maps/my_first_map.* C:\ros2slam\maps\
cd C:\ros2slam
git add maps/ ; git commit -m "..." ; git pull --no-edit ; git push
```
→ 저장소 `maps/my_first_map.pgm`, `maps/my_first_map.yaml` (push 완료)

### 4단계 트러블슈팅 (중요 — 면접 소재) ⭐
1. **의존성 충돌** — `ros-jazzy-slam-toolbox`가 `libboost-all-dev` 요구 → `libicu-dev`/`libnuma-dev`/`libibverbs-dev`가 이미 깔린 **보안 버전(`.1`)과 안 맞음**.
   → `sudo aptitude install ros-jazzy-slam-toolbox`, 첫 제안(설치 안 함) `n` 거부 → 다음 제안(안전 라이브러리 7개 다운그레이드, remove 0개) 수락.
   → 교훈: "보안 자동업데이트(unattended-upgrades) vs 개발 의존성 버전 고정"의 전형적 충돌. 1단계와 동일 패턴.
2. **`Failed to compute odom pose` 폭포 (진짜 원인)** — TF 트리는 `view_frames`로 정상 확인(`odom→base_link→laser`)됐는데도 slam이 계속 실패.
   → 원인: **slam_toolbox 기본 `base_frame`이 `base_footprint`인데 우리 TF는 `base_link`**. 이름 불일치.
   → 해결: 파라미터 파일에서 `base_footprint`→`base_link` 치환.
   → 교훈: "TF는 멀쩡한데 안 됨" = 노드가 기대하는 **프레임 이름**을 의심. 프로젝트1 예지보전의 체계적 원인배제와 같은 사고.
3. **`ros2 run`으로 slam 띄우면 지도 안 나옴** — 노드는 살아있는데 `/map` 미발행.
   → 원인: async_slam_toolbox_node는 **라이프사이클 노드**라 activate 필요. `ros2 run` 단독은 활성화 안 함. `ros2 launch`가 `Configuring→Activating` 해줌.
   → 교훈: 라이프사이클 노드는 launch로.
4. **노드 중복 표시** — `ros2 node list`에 `/rf2o_laser_odometry` 2개.
   → launch 파일은 노드 1개만 정의. 미러모드 WSL + 다중 인터페이스로 인한 **DDS discovery 착시**(실제 프로세스는 1개). 무시 가능.
5. **RViz2 경고들** (무시 가능):
   - GLSL sampler 에러 = WSLg 소프트웨어 렌더링 특성
   - "extrapolation into the future" / "message dropping" = 네트워크 지연으로 표현계층이 TF를 늦게 받음(원격 시각화 구조상 정상, Pi 로컬이면 안 뜸)
   - "Trying to create a map of size ..." = 에러 아님, 지도 확장 로그

---

## 🔜 다음 단계

### 3-B. 아두이노 연결 (구동부) ← **바로 다음 할 일**

- 아두이노를 Pi에 USB 연결, 기존 패킷 프로토콜로 모터 제어 노드 작성
- **C++ 증명 전략 1번 적용** — 이 노드를 C++로, `Packet.h`/`Packet.cpp`(프로젝트1 C++ 포팅본) 재사용
- `cmd_vel`(geometry_msgs/Twist) 구독 → 패킷 변환 → 시리얼 송신, 센서 패킷 → 토픽 발행
- 서사: "전송수단을 시리얼→블루투스→TCP/IP→ROS2로 네 번 바꾸는 동안 패킷 코드는 불변" (계층분리 4번째 증명)
- 선결: 아두이노 USB도 udev 규칙으로 포트 고정 권장(`/dev/arduino` 등, 라이다 `/dev/rplidar`와 구분)
- 개루프 구동: Nav2는 cmd_vel(m/s), L298N은 PWM → **실측 캘리브레이션 필요**. 움직이기 시작하는 최소 PWM(actuation onset)부터 측정.

### 이후 로드맵
- **5단계** — 자율주행 (Nav2 경로계획). 배터리 필수.
- **최종 목표: C (다지점 순회 자율주행).** 단 A(지도)→B(단일 목표점 주행)→C 순서 엄수. (A는 4단계에서 파이프라인 확인됨)

---

## 미리 인지한 기술적 제약 + 설계 방향 (확정)

**엔코더 없음 → 오도메트리 대책 (난이도순)**
1. **rf2o_laser_odometry** — 4단계에서 채택·동작 확인. 작은 방·저속이면 충분. **회전에 약함**(급회전 시 스캔매칭 깨짐 → 천천히 회전).
2. **IMU 추가(MPU-6050) + robot_localization EKF 융합** — rf2o가 회전에서 깨지면. C(순회)의 보험.
3. **엔코더(모터 교체/외장, 2채널)** — 근본 해결, 최후의 카드.
- slam_toolbox는 `odom→base_link` TF 필수 → rf2o를 **따로 띄워야** 함(안 띄우면 TF 에러). 4단계에서 확인.

**초음파(HC-SR04) 재활용**: 오도메트리 대체 아님. 라이다 20cm 사각지대의 낮은 장애물 회피 보완용(5단계). 프로젝트1 자산 계승.

**라이다 마운팅(4~5단계 실주행 시)**: 최상단, 스캔평면 안 가리게. 바닥 기준 약 20cm. 무게중심 낮게 단단히. (현재는 손매핑이라 static TF 값 0)

**전원 계획 (확정)**
- 개발(현재): 공식 5V 5A 어댑터, 콘센트
- 실주행(4~5단계): **USB-C PD 3A 보조배터리 → USB-C 직결**. 아두이노/모터는 별도 전원계통. Pi 계통은 라이다 1개뿐이라 3A 충분. X1200 UPS HAT 등 불필요(방열케이스 충돌).
- **라이다 전원: micro USB 3A 이상 케이블 필수** (2단계 트러블슈팅).

---

## C++ 증명 전략 (취업 핵심) ⭐

1. **[3-B, 바로 다음] 하드웨어 인터페이스 노드를 C++로** — `Packet.h`/`Packet.cpp` 재사용. cmd_vel→패킷→시리얼, 센서패킷→토픽. (계층분리 4번째 증명)
2. **[5단계 이후] ros2_control 하드웨어 인터페이스 플러그인** — C++ 전용, AGV 업계 표준. 여유되면.
3. **[5단계 이후] Nav2 커스텀 플러그인** — nav2_core 상속→순수가상함수 구현→pluginlib 등록. 상속·다형성·플러그인 로딩 실증.
- 순서 엄수: 3-B는 부담 적음(원래 할 일을 C++로). 2·3번은 후반 캡스톤.

---

## 작업 환경 메모

- 데스크탑: 유선랜 `192.168.75.x` / 노트북: WiFi `192.168.45.x` (**서로 다른 네트워크**)
- **Pi는 `192.168.45.x` → 노트북에서 SSH·개발**
- **개발 PC = 노트북(Windows 11 25H2) + WSL2(Ubuntu 24.04 + ROS2 Jazzy Desktop, 미러모드)**
- 모니터는 TV겸용 1대(구형), 헤드리스 유지
- SSH: `ssh daehan@192.168.45.100` (비번 `jungdaehan22`)
- **ROS_DOMAIN_ID=0** 로 Pi·WSL 통일 (자주 쓰면 양쪽 ~/.bashrc에 넣기)
- git: 지도·파일은 Pi에서 만들고 scp로 `C:\ros2slam` 가져와 커밋. push 전 `git pull --no-edit` 습관(원격 병합 이력 있음).

---

## 저장소 현재 상태 (github.com/jungdaehan11/amr-ros2-slam)
- `docs/SETUP_WSL2_RVIZ2.md` — 3-A 재현 가이드
- `maps/my_first_map.pgm`, `maps/my_first_map.yaml` — 4단계 첫 지도
- (아직 없음: 4단계 SLAM 세팅 가이드 문서, 아두이노 노드 코드 — 3-B에서 생성 예정)
