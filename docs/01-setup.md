# 1단계 — 라즈베리파이 5 헤드리스 세팅 + ROS 2 Jazzy

**목표** 모니터 없이 SSH로 접속 가능한 ROS 2 개발 환경 구축

**결과** `ssh daehan@192.xxx.xx.xxx` 한 줄로 접속, ROS 2 노드 간 토픽 통신 검증 완료

| 항목 | 값 |
|---|---|
| OS | Ubuntu Server 24.04.4 LTS (arm64) |
| ROS 2 | Jazzy Jalisco |
| 호스트명 | `amr` |
| 접속 | SSH over WiFi, 고정 IP |

---

## 1. SD카드 이미지 준비

Raspberry Pi Imager 사용.

| 항목 | 선택 |
|---|---|
| 기기 | Raspberry Pi 5 |
| OS | 범용 운영체제 → Ubuntu → **Ubuntu Server 24.04.x LTS (64-bit)** |
| 저장소 | microSD 64GB |

**Imager 사용자 지정에서 미리 설정** (이게 핵심)

- 호스트명: `amr`
- 계정 / 비밀번호
- WiFi SSID / 비밀번호, 국가 `KR`
- 서비스 탭 → SSH 사용 (비밀번호 인증)

> Imager가 생성한 `user-data`는 cloud-init 표준 형식이며 비밀번호가 해시로 저장된다.
> 여기서 설정하면 부팅 파티션의 YAML을 수동 편집할 필요가 없다.

---

## 2. 부팅 및 SSH 접속

1. SD카드 삽입, 랜선 또는 WiFi 환경 준비
2. 전원 연결 후 약 3분 대기 (첫 부팅 시 파일시스템 확장 + cloud-init 실행)
3. IP 확인

```cmd
arp -a
```

4. 접속

```cmd
ssh daehan@<>
```

### 트러블슈팅 — 기기 식별

`arp -a` 결과에서 라즈베리파이를 MAC 벤더 코드(`d8:3a:dd`, `dc:a6:32` 등)로
필터링하면 놓칠 수 있다. Pi 5는 다른 OUI 대역을 사용하는 개체가 있다.

**교훈** 후보를 좁히는 기준 자체가 틀릴 수 있다.
목록에 낯선 IP가 있으면 필터링으로 배제하기 전에 그냥 SSH를 시도해보는 편이 빠르다.

### 대안 — PC 직결 (공유기 우회)

공유기 문제와 Pi 문제를 분리하고 싶을 때 유용하다.
Pi를 PC 랜포트에 직접 연결하고 Windows 인터넷 연결 공유(ICS)를 켜면
PC가 DHCP 서버 역할을 하며 `192.xxx.xxx.x` 대역을 배정한다.

```
ncpa.cpl → Wi-Fi 속성 → 공유 탭
→ "다른 네트워크 사용자가 ... 연결할 수 있도록 허용" 체크
→ 홈 네트워킹 연결: 이더넷
```

PC의 이더넷 IPv4가 `192.xxx.xxx.x`로 바뀌면 정상 동작 중이다.

---

## 3. 시스템 업데이트

```bash
sudo apt update
sudo apt upgrade -y
sudo reboot
```

> 첫 부팅 후 `unattended-upgrades`가 백그라운드에서 자동 실행된다.
> apt 락이 잡혀 있으면 `Waiting for cache lock`이 반복 출력되는데 정상이며,
> **중간에 Ctrl+C로 끊으면 dpkg 상태가 손상**되므로 완료까지 대기해야 한다.

---

## 4. ROS 2 Jazzy 설치

### 저장소 등록

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

`packages.ros.org ... arm64 Packages` 항목이 받아지면 성공.

### 본 설치

```bash
sudo apt install -y ros-jazzy-ros-base
```

`ros-base`는 통신 코어와 CLI 도구만 포함한다.
RViz 등 시각화 도구가 포함된 `ros-jazzy-desktop`은 GUI 라이브러리를 다수 끌어오므로
Pi에는 설치하지 않고 개발 PC 쪽에 둔다.

### 트러블슈팅 — 의존성 충돌

```
liblz4-dev : Depends: liblz4-1 (= 1.9.4-1build1)
             but 1.9.4-1build1.1 is to be installed
libzstd-dev : Depends: libzstd1 (= 1.5.5+dfsg2-2build1)
              but 1.5.5+dfsg2-2build1.1 is to be installed
E: Unable to correct problems, you have held broken packages.
```

**원인** `unattended-upgrades`가 시스템 라이브러리를 보안 업데이트 버전(`.1` 접미사)으로
올려둔 상태인데, ROS 2 저장소의 패키지는 그 이전 버전에 링크되어 빌드되어 있다.

**해결** `aptitude`는 `apt`와 달리 충돌 해결책을 단계적으로 제시한다.

```bash
sudo apt install -y aptitude
sudo aptitude install ros-jazzy-ros-base
```

제안이 순서대로 나온다.

1. `Keep ros-jazzy-ros-base at current version [Not Installed]` → **`n`** (설치 포기이므로 거부)
2. `Downgrade liblz4-1, libzstd1` + ROS 패키지 설치 → **`y`** (채택)

빌드 번호 수준의 미세한 다운그레이드이며 기능 차이는 없다.

### 환경 설정

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo $ROS_DISTRO      # jazzy
```

---

## 5. 통신 검증

```bash
sudo apt install -y ros-jazzy-demo-nodes-cpp
```

**터미널 1**

```bash
ros2 run demo_nodes_cpp talker
```

**터미널 2** (SSH 세션을 하나 더 연다)

```bash
ros2 run demo_nodes_cpp listener
```

```
[talker]:   Publishing: 'Hello World: 77'
[listener]: I heard: [Hello World: 77]
```

### 프로젝트 1과의 대응

| amr-control (커스텀 프로토콜) | ROS 2 |
|---|---|
| C# 관제 앱, 아두이노 펌웨어 | **노드** |
| `CMD 0x20` 거리, `0x21` 전류 | **토픽** (`/scan`, `/odom`) |
| `[STX][LEN][CMD][DATA][CHK][ETX]` | **메시지 타입** (`sensor_msgs/LaserScan`) |
| XOR 체크섬, 하트비트 워치독 | **DDS** (자동 처리) |

프로젝트 1에서는 시리얼 포트를 열고, 패킷 규격을 양쪽에 동일하게 구현하고,
체크섬으로 무결성을 검증하고, 하트비트로 연결을 감시했다.
위 검증에서 두 노드는 IP도 포트도 지정하지 않았는데 서로를 발견하고 통신했다.
DDS가 그 계층을 대신한다.

단, 시리얼은 한 채널에 CMD로 다중화하는 구조인 반면
ROS 2는 토픽 이름으로 채널이 분리되고 발신자가 수신자를 모른다는 차이가 있다.

---

## 6. 고정 IP 설정

DHCP는 재부팅마다 주소가 바뀌어 SSH 접속 시 매번 IP를 찾아야 한다.

### 설정 확인

```bash
ip a show wlan0
sudo cat /etc/netplan/50-cloud-init.yaml
```

### 파일 작성

`nano`의 Ctrl 조합이 Windows 터미널에서 가로채여 저장이 안 되는 경우가 있어
heredoc으로 파일 전체를 작성한다.

```bash
sudo cp /etc/netplan/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml.bak

sudo tee /etc/netplan/50-cloud-init.yaml > /dev/null << 'EOF'
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
      dhcp6: true
  wifis:
    wlan0:
      optional: true
      dhcp4: false
      addresses: [192.xxx.xx.xx]
      routes:
        - to: default
          via: 192.xxx.xx.x
      nameservers:
        addresses: [x.x.x.x, x.x.x.x]
      regulatory-domain: "KR"
      access-points:
        "YOUR_SSID":
          auth:
            key-management: "psk"
            password: "YOUR_PSK_HASH"
EOF

sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan apply
```

> `SSID`와 `password`는 실제 값으로 교체한다.
> netplan은 평문 대신 PSK 해시를 받으며, **해시라도 저장소에 커밋하지 않는다.**

적용과 동시에 IP가 바뀌므로 기존 SSH 세션은 끊긴다. 새 주소로 재접속한다.

```cmd
ssh daehan@192.xxx.xx.x
```

> 설정에 자신이 없으면 `netplan apply` 대신 `sudo netplan try`를 쓴다.
> 120초 안에 확인 입력이 없으면 이전 설정으로 자동 롤백된다.

---

## 체크리스트

- [x] Ubuntu Server 24.04 부팅
- [x] SSH 접속 (유선 → 무선)
- [x] ROS 2 Jazzy 설치, `$ROS_DISTRO` 확인
- [x] talker / listener 토픽 통신
- [x] 고정 IP `192.xxx.xx.xxx`

---

## 다음

[2단계 — LiDAR 연결](02-lidar.md)

1. RPLIDAR A1M8 USB 연결, udev 규칙으로 포트 고정
2. `sllidar_ros2` 드라이버 빌드
3. `/scan` 토픽 데이터 확인 (`ros2 topic echo`, `ros2 topic hz`)
4. 개발 PC에 ROS 2 설치 후 RViz2로 원격 시각화
