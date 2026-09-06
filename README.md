# amr-ros2-slam

ROS 2 기반 2D LiDAR SLAM 자율주행 AMR

기존 아두이노 스마트카를 구동부로 재활용하고 라즈베리파이 5 + LiDAR를 상위 제어기로 올려,
ROS 2 환경에서 SLAM 지도 작성과 자율주행을 구현하는 프로젝트입니다.

이전 프로젝트([amr-control](https://github.com/jungdaehan11/amr-control))에서 정립한
**상위(판단) / 하위(구동) 계층 분리** 설계를 그대로 계승합니다.

---

## 시스템 구성

```
[RPLIDAR A1M8]
      │ USB
      ▼
[Raspberry Pi 5 / Ubuntu 24.04 + ROS 2 Jazzy]   ← 판단 계층
      │ USB Serial (기존 커스텀 패킷 프로토콜 재사용)
      ▼
[Arduino UNO]                                    ← 구동 계층
      │
      ▼
[L298N] → DC 기어모터 ×4
```

| 계층 | 하드웨어 | 역할 |
|---|---|---|
| 상위 | Raspberry Pi 5 (4GB) | SLAM, 경로계획, 센서 처리 |
| 하위 | Arduino UNO | 모터 구동, 센서 읽기, 안전 정지 |
| 시각화 | 개발용 PC | RViz2 원격 실행 |

---

## 하드웨어

| 항목 | 모델 | 비고 |
|---|---|---|
| SBC | Raspberry Pi 5 4GB | 쿨링팬 케이스 필수 |
| LiDAR | RPLIDAR A1M8-R6 | 2D 360°, 12m |
| MCU | Arduino UNO | 프로젝트 1에서 재사용 |
| 모터 드라이버 | L298N 2채널 | |
| 구동 | DC 기어모터 ×4 (4WD) | **엔코더 없음** |
| 전원(개발) | 공식 5V 5A 어댑터 | |
| 전원(주행) | 5V 3A PD 보조배터리 (예정) | |

---

## 설계 판단 기록

### Ubuntu Server 선택 (Desktop 대신)

| 항목 | Desktop | Server |
|---|---|---|
| 부팅 직후 RAM | ~1.2 GB | ~300 MB |
| GUI | 있음 | 없음 |

- **리소스 배분** — 4GB 보드에서 확보한 약 900MB를 SLAM/Nav2 연산에 배분
- **표현/연산 계층 분리** — RViz2는 개발 PC에서 원격 실행, Pi는 센서 처리와 SLAM에 집중.
  실주행 시 로봇에 모니터를 부착할 수 없으므로 구조적으로도 필연적인 선택
- **프로젝트 1 설계 원칙 계승** — 상위/하위 분리와 동일한 사고를 시각화/연산 축에 적용

### ROS 2 Jazzy 선택 (Kilted / Lyrical 대신)

- Ubuntu 24.04 LTS와 페어링되는 LTS 배포판 (EOL 2029-05)
- Nav2, slam_toolbox 등 주요 패키지의 레퍼런스·사례가 가장 두터움
- 국내 로봇·장비 업계 실사용 비중 고려

### LiDAR-only SLAM

엔코더가 없어 휠 오도메트리를 사용할 수 없습니다.
scan matching 기반 SLAM(slam_toolbox)으로 진행하며,
`odom → base_link` TF는 라이다 오도메트리(rf2o 등)로 대체할 예정입니다.

**알려진 한계**

- 빠른 회전 시 스캔 매칭 실패 가능
- 특징이 적은 균일한 복도에서 위치 상실 가능
- 개루프 구동이므로 `cmd_vel`(m/s) → PWM 매핑을 실측 캘리브레이션 필요

→ 개선안: 엔코더 추가 또는 초음파 센서 보완 (추후 단계에서 검토)

---

## 진행 상황

- [x] **1단계** — 라즈베리파이 세팅 (OS, ROS 2 설치) → [문서](docs/01-setup.md)
- [ ] **2단계** — LiDAR 연결 (드라이버, `/scan` 토픽, RViz 확인)
- [ ] **3단계** — 아두이노 연결 (USB 시리얼, 기존 패킷으로 모터 제어)
- [ ] **4단계** — SLAM 지도 작성 (slam_toolbox)
- [ ] **5단계** — 자율주행 (Nav2 경로계획)

---

## 저장소 구조

```
amr-ros2-slam/
├── README.md
├── docs/               # 단계별 구축 문서
│   └── 01-setup.md
├── config/             # SLAM / Nav2 파라미터 (예정)
└── src/                # ROS 2 패키지 (예정)
```

---

## 관련 프로젝트

- [amr-control](https://github.com/jungdaehan11/amr-control) — AMR 관제 시스템 + 전류 기반 예지보전 (C# WinForms / C++ MFC)
