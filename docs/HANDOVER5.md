# AMR 프로젝트 인수인계 (4단계 우선순위1 완료 — 개루프 캘리브레이션 끝, 실주행 SLAM 시작 시점)

> **이 문서 하나로 맥락 복원**: 새 대화에서 이 파일을 올리면 지금까지의 진행·설계판단·다음 할 일이 모두 복원됨.
> 최종 갱신: 2026-09-19 (4단계 개루프 캘리브레이션 완료 — 연속속도 주행 + 데드존/부스트/트림 확정)
> 이전 문서: HANDOVER4.md (3-B C++ 브릿지 완료 시점)

---

## 나에 대해

- 반도체·제조 장비 SW 취업 목표 (테크윙 "반도체 장비 제어 및 AGV SW개발" 공고 타겟)
- 아두이노 기본 문법, C# 중급, C++ 기초→실무 진입 중, ROS2 입문 단계
- 선호: 구조 위주 설명, 단계적 검증(한 번에 하나씩), 빠른 진행, 트레이드오프 먼저 짚어주기
- 결정 시 "엔지니어/기업 관점"으로 판단하는 걸 선호
- 리눅스 CLI 초보. 명령어는 복붙 가능하게 통째로. heredoc(`tee << 'EOF'`) 선호(대화형 편집기 Ctrl 안 먹힘)
- SSH: Windows cmd/PowerShell에서 `ssh daehan@192.168.45.100` (비번 `jungdaehan22`)

---

## 환경 요약 (변동 없음)

| 항목 | 값 |
|---|---|
| Pi OS/ROS2 | Ubuntu Server 24.04.4 + ROS2 **Jazzy** (ros-base) |
| Pi 접속 | `ssh daehan@192.168.45.100` / daehan / jungdaehan22 |
| Pi 고정IP | 192.168.45.100 (WiFi SK_F764_5G) |
| 개발PC | 노트북 Win11 + WSL2 Ubuntu 24.04 + ROS2 Jazzy Desktop(미러모드) |
| ROS_DOMAIN_ID | 0 (Pi·WSL 통일) |
| 라이다 포트 | `/dev/rplidar` (udev 고정, CP2102 10c4:ea60), frame_id=`laser` |
| 아두이노 포트 | `/dev/arduino` (udev 고정, 2341:0043, serial 1344B43523435150E8D5, MODE 0666) |
| GitHub | github.com/jungdaehan11/amr-ros2-slam (로컬 C:\ros2slam) |

- 아두이노 IDE는 노트북 Windows(포트 COM3). 업로드는 아두이노를 노트북에 꽂아서.
- git: Pi에서 만들고 scp로 C:\ros2slam 가져와 커밋. push 전 `git pull --no-edit` 습관.

---

## ✅ 이번 세션 완료 — 4단계 우선순위1: 개루프 캘리브레이션

**목표 달성**: teleop `cmd_vel`(m/s) → 실제 연속속도 주행. 방향 5종 고정속도(1차)에서 → 연속 PWM 제어(2차)로 진화.

### 한 일 요약
1. **0x15 연속 PWM 명령 신설** (프로토콜 확장 — 아래 상세)
2. **아두이노 펌웨어 확장**: 가변길이 파서(LEN으로 5/7바이트 분기) + `setMotorPWM`(int8 x2 스케일업)
3. **속도↔int8 캘리브레이션 실측** (선형회귀 R²=0.985)
4. **브릿지 노드 대개편**: 이산 방향판정 → 차동구동 연속속도 변환 (데드존 보상 + 우측 트림 + 기동 부스트 + turn_gain)
5. **전진/후진/제자리회전/(완만)선회 검증 완료**

---

## ★ 패킷 프로토콜 (0x15 추가 — 3중 비대칭 주의)

형식: `[STX=0x02][LEN][CMD][DATA...][CHK][ETX=0x03]`

| 방향 | 바이트 | LEN | DATA | CHK 계산 | 예시 |
|---|---|---|---|---|---|
| **명령**(5바이트, 기존) | 5 | 0x01 | 없음 | **LEN^CMD** | 전진 `02 01 10 11 03` |
| **연속PWM 0x15**(신규) | **7** | **0x03** | L,R(int8 2개) | **LEN^CMD^L^R** | 양쪽100 `02 03 15 64 64 09 03` |
| **센서**(6바이트) | 6 | 0x01 | 1바이트 | **LEN^CMD^DATA** | 거리29 `02 01 20 1d 3c 03` |

**★★ 체크섬 3규칙이 다 다름 (혼용 절대 금지)**:
- 5바이트 명령: `chk=LEN^CMD` (DATA 없음)
- 0x15 연속PWM: `chk=LEN^CMD^L^R` (DATA 포함) ← 오조작 시 바퀴 폭주 위험이라 DATA 포함시킴
- 센서: `chk=LEN^CMD^DATA`
- 프로젝트1 범용 buildPacket: `chk=CMD^DATA` (LEN 제외) — 또 다름

CMD: `0x10`전진 `0x11`후진 `0x12`좌 `0x13`우 `0x14`정지 / **`0x15`연속PWM(L,R int8)** / `0x20`거리 `0x21`전류 / `0x30`하트비트 / `0x40`경고ON `0x41`경고OFF

**0x15 인코딩**: L,R은 부호 있는 int8(-127~127). 양수=전진, 음수=후진, 0=정지. 2의 보수 바이트로 실음. **0x15 수신도 워치독 리셋 겸함**(연속송신이 하트비트 대체).

---

## ★ 캘리브레이션 결과 (2026-09-19 실측, 바닥·부하 상태)

**펌웨어 스케일업**: int8(-127~127)을 펌웨어 `setMotorPWM`에서 **x2** 하여 실제 PWM(-254~254) 사용. 이유: int8 최대 127로는 기존 구동 PWM 153을 못 냄.

**최소 구동값(onset)**: **int8 80** (실제 PWM 160), static 기준. 60~78은 데드존(소리만 나고 안 굴러감). ← 브릿지 데드존 보상 기준값.

**속도 대응표** (int8 x2=PWM, 2초 직진 실측):
| int8 | PWM | 속도(m/s) |
|---|---|---|
| 80 | 160 | 0.225 |
| 90 | 180 | 0.250 |
| 100 | 200 | 0.280 |
| 110 | 220 | 0.305 |
| 120 | 240 | 0.315 |
| 127 | 254 | 0.345 |

**선형회귀 (R²=0.985)**:
- 정방향: `v(m/s) = 0.002442×int8 + 0.031511`
- **역방향(브릿지에서 씀): `int8 = 409.55×v − 12.91`**

**바퀴간격(wheel separation)**: 0.138 m (좌우 바퀴 중심 간)
**좌우 편향**: 오른쪽 모터가 더 빠름 → 우측 트림 int8 -4 (기본값)

---

## ★ 개루프 저해상도의 근본 한계 (중요 — 면접 소재)

**유효 구동범위가 int8 80~127 = 겨우 47칸.** 여기서 파생되는 제약들:

1. **최저속도 0.225 m/s 고정**: 데드존 때문에 그보다 느린 전진 불가. Nav2 정밀접근 시 오버슈트 가능 → 5단계에서 goal_tolerance 넉넉히 + 제자리회전 위주 접근으로 대응.
2. **부드러운 곡선 불가**: 곡선은 "한 바퀴 느린 전진 + 다른 바퀴 빠른 전진"인데, 느린 쪽이 데드존(80) 밑이면 표현 불가. gain 7.1까지 "치우친 직진", 7.2부터 갑자기 제자리회전 → **중간(곡선) 구간이 없음.** → **곡선 포기 결정.** 직진+제자리회전 조합으로 모든 경로 커버 가능. 5단계 Nav2 **Rotation Shim Controller**(회전먼저→직진)로 대체 예정. 실제 저가 AGV 표준 방식.
3. **기동 정지마찰(scrub)**: 완전정지→제자리회전 기동 시 순간적으로 onset(80)보다 큰 힘 필요(4WD scrub 저항). → **기동 부스트**로 해결.

---

## ★ 브릿지 노드 최종 로직 (bridge_node.cpp)

`onCmdVel`(cmd_vel 수신) → 목표 int8 계산해 저장 → `onTxTimer`(50ms=20Hz)가 0x15로 연속 송신:

```
1. 차동구동: v_left = lin - ang*(wheel_sep/2)*turn_gain
             v_right = lin + ang*(wheel_sep/2)*turn_gain
2. velToInt8(v): |v|<stop_eps → 0
                 아니면 mag = 409.55*|v| - 12.91
                 mag < onset(80) → 80 (데드존 보상)
                 mag > 127 → 127 (클램프)
                 부호 = v방향
3. 우측 트림: r 크기를 right_trim(4)만큼 감소
4. 기동 부스트: 정지→움직임 전환 시 초기 boost_ticks(4=200ms) 동안
   applyBoost() = 각 바퀴 |int8| + boost_add(40), max127 클램프.
   ★가산식(차동 보존). 고정값 대입은 좌우를 같게 만들어 곡선 파괴하므로 금지.
5. cmd_vel이 cmd_timeout(0.5s) 끊기면 자동 정지 (안전)
```

**파라미터 (전부 declare_parameter, 실행시 -p로 조정 가능)**:
| 파라미터 | 기본값 | 의미 |
|---|---|---|
| inv_a / inv_b | 409.55 / -12.91 | 역변환식 계수 |
| onset_int8 | 80 | 데드존 최소값 |
| max_int8 | 127 | 상한 |
| wheel_separation | 0.138 | 바퀴간격(m) |
| right_trim | 4 | 우측 편향 보정 |
| stop_epsilon | 0.01 | 정지 판정 |
| cmd_timeout | 0.5 | 명령끊김 정지(s) |
| turn_gain | **3.0** | 회전 증폭(teleop 조작감용) |
| boost_add | 40 | 기동부스트 가산량 |
| boost_ticks | 4 | 부스트 지속(틱, 50ms당) |

**★ turn_gain 용도 주의**: teleop 수동조작감용. **5단계 Nav2에선 1.0으로 되돌려야**(컨트롤러가 회전반경 계산하므로 증폭하면 경로추종 틀어짐).

---

## 🔧 이번 세션 트러블슈팅 (면접 소재 ⭐⭐)

원인격리를 **소프트웨어→펌웨어→하드웨어** 순으로 체계적으로 수행:

1. **0x15 "소리만 나고 안 움직임"**: 파싱 성공, PWM 미달(actuation onset 미달). int8 127로도 부족 → 펌웨어 x2 스케일업으로 해결.
2. **"1초만 동작하고 멈춤"**: cmd_timeout(0.5s) 안전로직 정상작동. teleop이 키당 cmd_vel 1발만 쏴서. Nav2 연속명령엔 무관.
3. **"제자리회전 2번째 안 됨"**: 완전정지→회전 기동 시 scrub 정지마찰. onset 80은 직진기준이라 회전기동엔 부족 → 기동 부스트로 해결.
4. **"u/o 곡선이 직진→갑자기 좌회전"**: 부스트가 양바퀴를 같은값(고정 120)으로 만들어 차동 파괴 → 부스트를 가산식으로 변경.
5. **"곡선이 아예 안 됨"**: 파이썬 극단테스트(한쪽만 구동/제자리회전)로 하드웨어 정상 확인 → 데드존 47칸 한계로 곡선 물리적 불가 판명 → 곡선 포기 결정.

진단 도구: 브릿지 RCLCPP_INFO 로그로 실제 L/R값 확인, 파이썬 원라이너로 브릿지 배제한 순수 펌웨어 테스트, `topic pub -r 10`으로 teleop 단발 vs 연속 격리.

---

## 🔜 다음 할 일

### 우선순위 2 — 로봇이 움직이며 SLAM 재작성 (4단계 완성) ★다음 세션
- 3-B+캘리브레이션으로 로봇이 스스로 연속속도 주행 가능 → **손매핑 아닌 실주행 매핑**으로 제대로 된 지도.
- 라이다 실제 마운팅(바닥 20cm) + **static TF(base_link→laser) 값 실측으로 교체**.
- 파이프라인(HANDOVER4 4단계): /scan → static TF → rf2o(odom→base_link) → slam_toolbox(map)
- slam base_frame이 base_footprint 기본이라 base_link로 치환 필요(`~/ros2_ws/config/my_mapper.yaml`).
- 주행: teleop 직진(i,)+제자리회전(j,l)로 로봇 몰면서 매핑. 곡선 없이 충분.

### 우선순위 3 — 5단계 자율주행 (Nav2)
- A(지도)→B(단일목표)→C(다지점순회). 최종목표 C.
- **turn_gain 1.0 복귀**, Nav2 Rotation Shim Controller로 회전 처리.
- 데드존 대응: goal_tolerance 넉넉히.
- 배터리 필수 (QCY PB10C 10000mAh 5V/3A, get_throttled로 검증).

---

## 미리 인지한 기술적 제약 (변동 없음)

**엔코더 없음 → 오도메트리 (난이도순)**
1. rf2o_laser_odometry — 4단계 채택·동작확인. 회전에 약함(천천히).
2. IMU(MPU-6050)+robot_localization EKF — rf2o 회전 보완. C의 보험.
3. 엔코더 — 근본해결, 최후 카드. (이번에 데드존/곡선 한계로 후보 거론됐으나 목표엔 불필요로 보류)
- slam_toolbox는 odom→base_link TF 필수 → rf2o 따로 띄워야.

**초음파(HC-SR04)**: 라이다 20cm 사각 저장애물 회피 보완(5단계). `/ultrasonic` 토픽 이미 발행중.
**전원**: 개발=5V5A 어댑터. 실주행=QCY PB10C. 아두이노/모터 별도계통. 라이다 micro USB 3A 케이블 필수.

---

## C++ 증명 전략 (취업 핵심)

1. **[3-B+4단계] 하드웨어 인터페이스 노드 C++** ✅ 완료 — Packet 재사용, termios 시리얼, cmd_vel→연속PWM. "전송수단 교체해도 패킷 계층 불변" + "물리 캘리브레이션을 상위 계층에서 보상" 증명.
2. **[5단계 이후] ros2_control 하드웨어 인터페이스 플러그인** — C++ 전용, AGV 업계표준.
3. **[5단계 이후] Nav2 커스텀 플러그인** — nav2_core 상속→순수가상→pluginlib.

---

## 저장소 상태 / 커밋 예정 (github.com/jungdaehan11/amr-ros2-slam)
- `docs/SETUP_WSL2_RVIZ2.md`, `docs/HANDOVER3.md`, `docs/HANDOVER4.md` — 기존
- `maps/my_first_map.pgm/.yaml` — 4단계 첫 지도(손매핑, 곧 실주행본으로 교체 예정)
- `amr_bridge/` — src/Packet.cpp, SerialPort.cpp, bridge_node.cpp + include + CMakeLists + package.xml
- **커밋 예정(이번 세션 산출물)**:
  - `arduino/amr_usb.ino` — USB Serial 115200, 0x15 가변길이 파서 + setMotorPWM(x2)
  - `amr_bridge/` 갱신 (Packet.h/.cpp에 0x15, bridge_node.cpp 연속속도+부스트+트림+gain)
  - `calibration.xlsx` — 실측 데이터
  - `docs/HANDOVER5.md` — 이 문서

## 아두이노 펌웨어 현재 버전 (USB Serial, 115200)
- 5바이트 명령 파서 + 6바이트 센서 송신 유지(프로젝트1 그대로).
- **추가: 가변길이 파서**(rxBuf[7], LEN으로 5/7바이트 분기), **CMD_SETPWM=0x15 핸들러**, **setMotorPWM(int8 x2 스케일업, 방향+PWM 분리)**.
- 워치독 1.2초, 서보 경고 유지. 0x15 수신도 워치독 리셋.
- 파일: outputs/amr_usb.ino (노트북 IDE로 업로드 완료). 저장소 커밋 권장.

## 실행법 (현재)
```bash
# 창1 — 브릿지 (turn_gain 기본3.0 = Nav2용 물리정확)
source ~/ros2_ws/install/setup.bash
ros2 run amr_bridge bridge_node
# teleop 수동주행 시엔 회전감 위해:
#   ros2 run amr_bridge bridge_node --ros-args -p turn_gain:=5.0 -p cmd_timeout:=2.0
# 창2 — teleop (i전진 ,후진 j좌회전 l우회전 k정지 — 곡선 u/o는 데드존한계로 미지원)
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 센서 확인: ros2 topic echo /ultrasonic
```
