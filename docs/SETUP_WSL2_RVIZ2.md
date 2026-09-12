# 개발 PC 원격 시각화 세팅 — WSL2 + ROS2 Jazzy + RViz2

> **목적**: 개발 PC(Windows 노트북)에서 라즈베리파이가 발행하는 `/scan`(LiDAR) 토픽을
> 네트워크 너머로 받아 RViz2로 실시간 시각화한다.
> Pi에는 모니터를 달지 않고(헤드리스), 표현(RViz2)은 개발 PC가, 연산·센서는 Pi가 담당한다.
> → 프로젝트 전반의 **표현/연산 계층 분리** 원칙을 개발 환경에도 동일 적용.

- 작성일: 2026-09-12 (프로젝트 3단계 3-A)
- 대상 환경: Windows 11 (25H2, 빌드 26200) / Pi5 Ubuntu Server 24.04 + ROS2 Jazzy

---

## 0. 전체 그림

```
[Windows 노트북]
  └ WSL2 (Ubuntu 24.04 + ROS2 Jazzy Desktop)  ── 미러 네트워크 모드 ──┐
                                                                      │ 같은 대역
                                                                      │ 192.168.45.x
[라즈베리파이5 / Ubuntu Server 24.04 + ROS2 Jazzy] ────────────────────┘
  └ RPLIDAR A1M8 → /scan 발행

* 같은 Wi-Fi(SK_F764_5G), 같은 ROS_DOMAIN_ID → DDS가 서로 자동 발견(discovery)
* Pi IP: 192.168.45.100 / 노트북·WSL IP: 192.168.45.219 (미러 모드로 동일 대역)
```

**핵심 설계 판단**
- WSL의 기본 네트워크는 NAT라 Pi가 WSL을 못 본다 → **미러 네트워크 모드**로 노트북과 같은 IP 대역에 노출시켜야 DDS 자동 발견이 성립.
- 양쪽 ROS2 배포는 **반드시 동일하게 Jazzy** (Ubuntu 24.04 페어링 LTS). 버전이 다르면 DDS 메시지 정의가 어긋난다.
- Pi는 `ros-base`(최소), 개발 PC는 `desktop`(RViz2 포함). 역할이 다르므로 설치 패키지도 다르다.

---

## 1. 사전 조건

| 항목 | 값 / 조건 |
|---|---|
| Windows | 11, 22H2 이상 (미러 모드 요구). 확인: `winver` |
| WSL 버전 | 2.x (미러 모드는 최신 WSL 필요). 확인: `wsl --version` |
| Pi 상태 | 2단계 완료 — LiDAR 연결, `/scan` 정상 발행 |
| 네트워크 | 노트북 Wi-Fi가 Pi와 같은 대역(192.168.45.x) |

---

## 2. WSL2 + Ubuntu 24.04 설치 (Windows PowerShell, 관리자)

```powershell
wsl --install -d Ubuntu-24.04
```

- 설치 후 재부팅 요구 시 재부팅.
- 재부팅 후 배포판이 없다고 나오면(`WSL_E_DISTRO_NOT_FOUND`) 다시 한 번:
  ```powershell
  wsl --install -d Ubuntu-24.04
  ```
- 첫 실행 시 Unix 계정 생성 (Pi와 동일하게 맞추면 혼선 적음):
  - username: `daehan`
  - password: `jungdaehan22` (입력해도 화면에 안 보이는 게 정상)
- 버전 확인:
  ```bash
  lsb_release -a          # Ubuntu 24.04.x LTS (noble) 확인
  ```

> ⚠️ 목록에 Ubuntu 26.04도 보이지만 **반드시 24.04**. Pi의 Jazzy와 맞춰야 함.

---

## 3. WSL 안에 ROS2 Jazzy Desktop 설치 (WSL 셸)

```bash
cd ~

# (1) 로케일 UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# (2) ROS2 apt 저장소 + 키
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# (3) 설치 (Desktop = RViz2 포함, 용량 큼)
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-dev-tools

# (4) 자동 source 등록
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# (5) GUI 확인 (WSLg — Win11이면 기본 내장)
rviz2      # 빈 RViz2 창이 Windows 데스크톱에 뜨면 GUI OK → 닫기
```

---

## 4. 미러 네트워크 모드 설정 (Windows PowerShell) ⭐

WSL 안이 아니라 **Windows** 쪽에서 한다.

```powershell
# .wslconfig 생성
@"
[wsl2]
networkingMode=mirrored
"@ | Set-Content -Path "$env:USERPROFILE\.wslconfig" -Encoding UTF8

# WSL 완전 종료 (설정 적용 위해 필수 — 열려있던 WSL 창은 자동 종료됨)
wsl --shutdown

# 5초 후 다시 시작
wsl -d Ubuntu-24.04
```

WSL 재진입 후 IP 확인:
```bash
ip addr | grep 192.168.45
# 예: inet 192.168.45.219/24 ... eth1   ← 노트북과 같은 대역이면 성공
```

> 미러 모드 전에는 172.x 같은 격리 IP가 뜬다. 192.168.45.x가 떠야 Pi와 같은 네트워크.

---

## 5. DDS 통신 검증 (talker / listener 크로스)

`/scan` 시각화 전에 가장 단순한 통신부터 확인한다.
**양쪽 `ROS_DOMAIN_ID`를 0으로 통일**한다.

**창 A — WSL (listener):**
```bash
cd ~
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

**창 B — Pi (SSH, talker):**
```bash
# (Windows에서) ssh daehan@192.168.45.100
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

- 성공: 창 A에 `I heard: [Hello World: N]` 출력 → DDS가 네트워크를 넘었다.
- 확인 후 양쪽 Ctrl+C.

### ⚠️ 트러블슈팅 — talker는 도는데 listener가 조용함
**원인: Windows Defender 방화벽이 미러 WSL로 들어오는 DDS 인바운드를 차단.**
(패킷은 나가는데 돌아오는 걸 막음 → 단방향처럼 보임)

**해결 (Windows PowerShell, 관리자):**
```powershell
New-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```
- 즉시 적용(WSL 재시작 불필요). 실행 직후 listener가 받기 시작하면 해결.
- 위 명령이 안 먹히면(구버전 등) 임시 대안:
  ```powershell
  Set-NetFirewallProfile -Profile Public,Private -DefaultInboundAction Allow
  ```
  (범위가 넓으니 1차 명령이 되면 쓰지 않는다.)

---

## 6. /scan 원격 시각화

**창 B — Pi (SSH): LiDAR 실행**
```bash
export ROS_DOMAIN_ID=0
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/rplidar
# 이 창을 살려둬야 라이다가 계속 돈다 (멈추려면 이 창에서 Ctrl+C)
```

**창 A — WSL: 데이터 도착 확인 → RViz2**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash

ros2 topic hz /scan     # 약 7.5Hz 뜨면 원격 도착 성공 → Ctrl+C
rviz2
```

**RViz2 설정 3가지:**
1. Global Options → **Fixed Frame** 을 `map` → **`laser`** 로 변경
   (안 바꾸면 "Fixed Frame does not exist" 에러, 점 안 보임)
2. 좌하단 **Add** → **By topic** 탭 → `/scan` 아래 **LaserScan** → OK
3. 점이 안 보이면 LaserScan 항목의 **Size (m)** 를 `0.03` 으로

→ 라이다 중심 주변 360도 점군이 방 윤곽을 그리면 완료.
   손으로 물건을 움직이면 점군이 실시간으로 반응한다.

---

## 7. 결과 / 검증

| 항목 | 값 |
|---|---|
| WSL `/scan` average rate | 약 7.5Hz (Pi 로컬 7.47Hz와 사실상 동일) |
| std dev | ~0.007s (안정적, 네트워크 지연·손실 미미) |
| 시각화 | RViz2에서 실시간 점군 확인 |

이로써 2단계의 미완 항목(개발 PC 원격 시각화)까지 마무리되고, 3-A 완료.

---

## 8. 매번 켤 때 요약 (치트시트)

```bash
# --- Pi (SSH: ssh daehan@192.168.45.100) ---
export ROS_DOMAIN_ID=0
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/rplidar

# --- WSL (wsl -d Ubuntu-24.04) ---
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
rviz2      # Fixed Frame=laser, Add→LaserScan(/scan)
```

> 반복이 잦으면 `export ROS_DOMAIN_ID=0` 를 양쪽 `~/.bashrc`에 넣어두면 매번 안 쳐도 된다.

---

## 트러블슈팅 로그 (면접 소재)

1. **WSL 배포판 설치 중단** — WSL 엔진만 깔리고 재부팅 대기로 배포판 다운로드가 안 끝나 `WSL_E_DISTRO_NOT_FOUND`.
   재부팅으로 기반이 잡힌 뒤 `wsl --install -d Ubuntu-24.04` 재실행으로 해결.
2. **DDS 단방향 현상** — talker는 나가는데 listener가 못 받음.
   전형적인 방화벽 인바운드 차단. 미러 모드 WSL의 인바운드를 열어 해결.
   → 교훈: "한쪽만 되는" 통신은 대개 방향성 있는 차단(방화벽)을 의심.
3. **네트워크 계층 분리 사고의 일관성** — 개발 PC(표현) / Pi(연산·센서) 분리를
   프로젝트 아키텍처뿐 아니라 개발 환경에도 그대로 적용.
