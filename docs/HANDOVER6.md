# AMR 프로젝트 인수인계 (★4단계 완료 — IMU+EKF 센서퓨전 + 실주행 SLAM 지도 생성 완료, 5단계 Nav2 진입 시점)

> **이 문서 하나로 맥락 복원**: 새 대화에서 이 파일을 올리면 지금까지의 진행·설계판단·다음 할 일이 모두 복원됨.
> 최종 갱신: 2026-10-03 (MPU-6050 IMU 통합 + EKF 융합 + **실주행 지도 my_real_map_v2 저장 완료**)
> 이전 문서: HANDOVER5.md (4단계 개루프 캘리브레이션 완료 시점)

---

## 나에 대해

- 반도체·제조 장비 SW 취업 목표 (테크윙 "반도체 장비 제어 및 AGV SW개발" 공고 타겟)
- 아두이노 기본 문법, C# 중급, C++ 기초→실무 진입 중, ROS2 입문 단계
- 선호: 구조 위주 설명, 단계적 검증(한 번에 하나씩), 빠른 진행, 트레이드오프 먼저 짚어주기
- 결정 시 "엔지니어/기업 관점"으로 판단하는 걸 선호
- 리눅스 CLI 초보. 명령어는 복붙 가능하게 통째로. heredoc(`tee << 'EOF'`) 선호
- SSH: Windows cmd/PowerShell에서 `ssh daehan@192.168.45.100` (비번 `jungdaehan22`)

---

## 환경 요약

| 항목 | 값 |
|---|---|
| Pi OS/ROS2 | Ubuntu Server 24.04.4 + ROS2 **Jazzy** (ros-base) |
| Pi 접속 | `ssh daehan@192.168.45.100` / daehan / jungdaehan22 |
| Pi 고정IP | 192.168.45.100 (WiFi SK_F764_5G) |
| 개발PC | 노트북 Win11 + WSL2 Ubuntu 24.04 + ROS2 Jazzy Desktop(미러모드) |
| ROS_DOMAIN_ID | 0 (Pi·WSL 통일) |
| 라이다 포트 | `/dev/rplidar` (udev 고정), frame_id=`laser`, **드라이버는 `sllidar_ros2`** |
| 아두이노 포트 | `/dev/arduino` (udev 고정, 2341:0043) |
| **IMU** | **MPU-6050 (GY-521), I2C: SDA=A4, SCL=A5** |
| GitHub | github.com/jungdaehan11/amr-ros2-slam (로컬 C:\ros2slam) |

**★ 라이다 드라이버 주의**: 패키지명이 `rplidar_ros`가 아니라 **`sllidar_ros2`**. 런치는 `sllidar_a1_launch.py`.

---

## ✅ 이번 세션 완료 — IMU 통합 + EKF 센서 퓨전 + **4단계 매핑 완료**

### 배경: 왜 IMU를 도입했나 (★면접 핵심 소재)

지난 세션 실주행 매핑 시도에서 **지도가 계속 틀어지는 문제** 발생. 원인을 체계적으로 격리:

1. **라이다 진동 의심** → 모터 구동 vs 손으로 밀기 대조 테스트 → 둘 다 동일 → **기각**
2. **RViz QoS/네트워크 의심** → `/map`은 5초 주기(map_update_interval)라 hz 측정이 느릴 뿐 정상 → **기각**
3. **rf2o 회전 추정 실패** → Fixed Frame=odom에서 로봇을 왼쪽 90도 회전시켜도 **벽이 조금 돌다가 원위치로 리셋되는 현상 반복** → **확정**

**결론**: rf2o_laser_odometry는 좁고 특징이 적은 공간에서 **회전을 스캔 변화로 구분하지 못함**. 라이다만으로 ego-motion을 추정하는 방식의 근본 한계.

**해결 선택**: MPU-6050 자이로로 회전을 물리적으로 직접 측정 → robot_localization EKF로 rf2o와 융합.
(엔코더도 후보였으나, "센서 퓨전으로 저가 센서 한계 극복"이 포트폴리오 스토리로 더 가치 있다고 판단)

### 한 일 요약

1. **MPU-6050 구매·배선** (ㄱ자 납땜 완료품, 9,980원) — 라인트레이서 제거 후 A4/A5 전용
2. **I2C 인식 확인** (0x68) → 자이로 Z축 동작 확인
3. **펌웨어 확장**: 0x22 패킷(int16 2바이트) 신설 + 바이어스 자동보정 + 20Hz 송신
4. **브릿지 확장**: 가변길이 RX 파서 + `/imu` 토픽 발행
5. **robot_localization EKF 설정**: rf2o(x,y,yaw) + IMU(vyaw) 융합
6. **공분산 버그 발견·수정** (아래 상세) — 정지 드리프트 200배 개선

---

## ★ 패킷 프로토콜 (0x22 추가 — 체크섬 4종 비대칭)

형식: `[STX=0x02][LEN][CMD][DATA...][CHK][ETX=0x03]`

| 방향 | 바이트 | LEN | DATA | CHK 계산 | 예시 |
|---|---|---|---|---|---|
| **명령**(기존) | 5 | 0x01 | 없음 | **LEN^CMD** | 전진 `02 01 10 11 03` |
| **연속PWM 0x15** | 7 | 0x03 | L,R(int8) | **LEN^CMD^L^R** | `02 03 15 64 64 09 03` |
| **센서**(기존) | 6 | 0x01 | 1바이트 | **LEN^CMD^DATA** | 거리29 `02 01 20 1d 3c 03` |
| **★자이로 0x22**(신규) | **7** | **0x02** | HI,LO(int16) | **LEN^CMD^HI^LO** | — |

**★ LEN 값으로 길이 분기**: 명령 수신(아두이노)은 0x01→5바이트, 0x03→7바이트.
센서 수신(브릿지)은 **0x01→6바이트, 0x02→7바이트**. 총길이 = LEN+5.

**0x22 인코딩**: 자이로 Z 원시값(raw LSB), 바이어스 제거 + **부호 반전** 완료 상태로 송신.
브릿지에서 `/131.0` → °/s, `*π/180` → rad/s 변환.

CMD: `0x10`전진 `0x11`후진 `0x12`좌 `0x13`우 `0x14`정지 / `0x15`연속PWM / `0x20`거리 `0x21`전류 **`0x22`자이로Z** / `0x30`하트비트 / `0x40`경고ON `0x41`경고OFF

---

## ★ IMU 실측 데이터 (2026-10-03)

- **I2C 주소**: 0x68 (기본)
- **자이로 풀스케일**: **±500 °/s (65.5 LSB per °/s)** — 급회전 saturation 대응으로 ±250에서 변경
- **정지 시 바이어스**: 매우 작음 (raw 기준 ±0.3 dps) — 저가품치곤 양호
- **부호**: 실측상 **왼쪽(반시계) 회전이 음수** → 펌웨어에서 `-1` 곱해 ROS REP-103(반시계=양수)에 맞춤
- **송신 주기**: **40Hz (25ms)**, 실측 37.6Hz. 초음파 `pulseIn` 블로킹 때문에 처음 11.6Hz → SEND_INTERVAL 250ms, pulseIn 타임아웃 12000us로 조정
- **90도 회전 검증**: EKF yaw가 38° → 81° → 90.022° → 91°로 단조 증가, 멈추면 유지 ✅

---

## 🔧 이번 세션 트러블슈팅 (면접 소재 ⭐⭐⭐)

### 1. 공분산 `-1` 버그 (★최대 수확)

**증상**: 로봇 정지 상태인데 EKF yaw가 **초당 -4.5도씩 일정하게 흐름**. RViz 지도가 계속 빙빙 돎.

**진단 과정**:
- `/imu` 값은 0 근처로 정상 → 자이로 바이어스 문제 아님
- `tf2_echo odom base_link`에서 yaw만 일정 속도로 흐름 → EKF가 "계속 회전 중"이라 믿는 상태

**원인**: `sensor_msgs/Imu` 규약상 **공분산 배열의 [0]번 원소가 -1이면 해당 데이터 전체가 무효**.
`angular_velocity_covariance[0] = -1`을 "x축 미사용" 의미로 넣었는데, 실제로는 **각속도 전체를 무효 처리** → EKF가 자이로를 아예 못 받고 과거 상태를 무보정 적분.

**해결**: 안 쓰는 축은 -1이 아니라 **큰 분산값(1e6)** 으로 표현.
```cpp
m.angular_velocity_covariance[0] = 1e6;      // x 사실상 무시
m.angular_velocity_covariance[4] = 1e6;      // y 사실상 무시
m.angular_velocity_covariance[8] = imu_var_; // z만 유효
```
**결과**: 정지 드리프트 **-4.5°/s → 0.02°/s (200배 개선)**

### 2. rf2o yaw를 완전히 끈 것이 과했던 문제 (★설계 교훈)

**초기 설계**: rf2o 회전이 실패하는 걸 봤으니 `odom0_config`에서 yaw를 false로 제외, 회전은 IMU 전담.

**증상**: `map→odom` 보정값이 **28.8도**까지 벌어짐 = slam이 EKF 자세를 크게 불신 → 지도가 겹쳐 쌓이며 뭉개짐.

**원인**: 자이로는 각속도를 적분하므로 **각도 오차가 계속 누적**됨. 절대 각도를 잡아줄 기준이 없으면 드리프트.
rf2o yaw는 제자리 회전에선 실패하지만, **느린 변화(직진 중 휨)는 오히려 잘 잡음**.

**해결**: `odom0_config`의 yaw를 다시 **true**로. 둘을 모두 EKF에 넣어 상호 보완.
**결과**: `map→odom` 보정이 **28.8도 → 0.000도**, translation도 [0.368, -0.218] → [-0.010, -0.010]

> **교훈**: "한 센서가 특정 축에서 실패한다"고 그 축을 통째로 빼는 건 과한 대응.
> 센서 퓨전의 본래 취지는 **서로의 약점을 보완**하는 것. EKF에 둘 다 주고 가중치로 판단하게 하는 게 맞음.

### 3. slam이 스캔을 등록하지 않던 문제

**증상**: 회전시키면 **빨간 스캔만 움직이고 검은 지도는 그대로**.

**원인**: `minimum_travel_heading: 0.5` (= 약 28도). "살짝 톡" 방식 회전으로는 이 문턱을 못 넘어 slam이 스캔을 아예 등록 안 함. `map→odom`이 계속 0.000으로 고정된 것이 증거.

**해결**: `minimum_travel_heading: 0.17`(≈10도), `minimum_travel_distance: 0.2`(20cm)로 낮춤.

### 4. EKF "Failed to meet update rate" 오진

**증상**: EKF가 주기를 못 맞춘다는 에러 반복. → 성능 문제로 판단해 frequency를 30 → 15 → 10Hz로 낮춤.

**실제**: `top` 확인 결과 **CPU 89% 유휴, 로드 0.37**. 성능 문제 아니었음. 진짜 원인은 **브릿지가 죽어 `/imu`가 안 들어와 EKF가 센서를 기다리던 것**.

**교훈**: 에러 메시지의 표면적 제안("rate를 낮춰라")을 따르기 전에 **실제 리소스를 측정**할 것. 주파수를 낮춘 것이 오히려 회전 중 자세 추정 품질을 떨어뜨렸고, 30Hz로 되돌리니 지도 품질이 눈에 띄게 개선됨.

---

## ★ 현재 시스템 구조

```
map (slam_toolbox)
 └→ odom (robot_localization EKF)          ← TF 발행 주체
     └→ base_link
         ├→ laser   (static TF: 0.02, 0, 0.20, yaw 0)
         └→ imu_link (static TF: 0,0,0,0,0,0)

EKF 입력:
  /odom_rf2o (rf2o, 7.6Hz)  → x, y, yaw 사용
  /imu       (브릿지, 20Hz) → vyaw(각속도)만 사용
```

**★ rf2o의 `publish_tf`는 반드시 False** (EKF와 TF 충돌). 전용 런치 `~/ros2_ws/launch/rf2o_no_tf.launch.py` 사용.

---

## ★ 설정 파일 현황

### `~/ros2_ws/config/ekf.yaml` (신규)
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0            # ★낮추지 말 것 (품질 저하)
    sensor_timeout: 0.2
    two_d_mode: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    publish_tf: true
    print_diagnostics: false

    odom0: /odom_rf2o
    odom0_config: [true,  true,  false,
                   false, false, true,     # ★yaw 포함 (자이로 적분오차 보정)
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom0_differential: false

    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,      # ★vyaw만
                  false, false, false]
    imu0_differential: false
    # process_noise_covariance: 전부 실수(0.0)로 — 정수 0 섞이면 YAML 파싱 에러
```

**★ YAML 주의**: ROS2 파라미터 배열은 타입 통일 필수. `0`과 `0.05`를 섞으면
`Sequence should be of same type` 에러 → 모두 `0.0` 형식으로.

### `~/ros2_ws/config/my_mapper.yaml` (수정)
- `base_frame: base_link` (원래부터 맞게 돼 있었음)
- `minimum_travel_heading: 0.17` (0.5 → 수정)
- `minimum_travel_distance: 0.2` (0.5 → 수정)
- `max_laser_range: 20.0` → 12.0 권장 (A1 실제 12m, WARN 발생)

### `~/ros2_ws/launch/rf2o_no_tf.launch.py` (신규)
rf2o를 `publish_tf: False`로 띄우는 전용 런치.

---

## ★ 브릿지 노드 변경사항 (bridge_node.cpp)

**추가된 것**:
- `#include <sensor_msgs/msg/imu.hpp>`
- `imu_pub_` (토픽 `/imu`, 큐 50)
- `publishImu(int16_t raw)` — raw → rad/s 변환 + 공분산 설정
- `onSerialRx()` **가변길이 파서로 전면 교체** (LEN으로 6/7바이트 분기)
- `rx_timer_` 20ms → **5ms** (타임스탬프 지연 감소)

**추가 파라미터**:
| 파라미터 | 기본값 | 의미 |
|---|---|---|
| gyro_scale | 131.0 | LSB per °/s. **★실행 시 65.5로 줘야 함 (±500dps)** |
| gyro_sign | 1.0 | 부호 반전용 (현재 1.0이 정상) |
| imu_frame_id | imu_link | IMU 프레임 |
| imu_angular_variance | 0.0004 | z축 각속도 분산 |

**teleop 주행 파라미터 (확정)**: `turn_gain:=5.0 cmd_timeout:=0.8`
- gain 3 → int8 ~72~80 (데드존 경계, 소리만) ❌ / gain 5 → int8 ~120 ✅
- **★Nav2에선 turn_gain 1.0으로 되돌릴 것**

---

## ✅ 매핑 품질 문제 — 해결 완료

**원인 확정**: 자이로 샘플링 부족 + 측정범위 초과.
4WD scrub 특성상 완전정지→회전 시 "버티다 툭 터지듯" 급회전하는데,
- 20Hz(50ms) 샘플링으로는 그 순간의 회전 구간을 놓침 → 적분값이 실제보다 작게 나옴
- ±250°/s 범위를 순간적으로 초과 → 값이 잘림(saturation)

**적용한 수정 (펌웨어 3곳)**:
```cpp
const unsigned long IMU_INTERVAL = 25;    // 50 → 25 (20Hz → 40Hz)
Wire.write(0x08);                          // GYRO_CONFIG: 0x00(±250) → 0x08(±500dps)
const unsigned long SEND_INTERVAL = 250;   // 200 → 250 (초음파 주기 늘려 40Hz 확보)
```
**★브릿지 실행 시 `gyro_scale:=65.5` 필수** (±500dps는 65.5 LSB/°/s. 안 주면 회전량 2배로 계산됨)

**검증 결과**:
| 지표 | 수정 전 | 수정 후 |
|---|---|---|
| IMU 수신율 | 20Hz | **37.6Hz** |
| map→odom yaw 보정 (360도 회전 시) | 10~28도 | **0.4도** |

→ slam이 EKF 자세를 거의 그대로 수용 = 자세 추정이 실제와 일치.

---

## ★ 매핑 주행 노하우 (v1 실패 → v2 성공에서 얻은 것)

같은 설정으로도 **주행 방식에 따라 지도 품질이 크게 갈림**.

**v1 (실패)**: 평소 속도로 주행 → 2D pgm에서 오른쪽 영역 벽이 여러 겹으로 갈라짐
**v2 (성공)**: 아래 원칙 적용 → 벽이 한 겹 직선으로 깔끔

**원칙**:
1. **회전 최소화** — 직진으로 갈 수 있는 데까지 가고, 꼭 필요할 때만 회전. 회전이 오차의 주원인.
2. **회전 후 3초 완전정지** — 예외 없이. slam이 스캔매칭으로 자세를 재정렬할 시간.
3. **전체적으로 천천히** — 급하게 가면 반드시 깨짐.
4. **RViz 실시간 감시** — 벽이 두 겹으로 보이면 즉시 멈추고 5~10초 대기.
5. **한 바퀴 돌아 출발점 복귀** — 루프클로저로 지도 정렬.

**★품질 판정은 반드시 2D pgm으로.** RViz 3D 뷰에서는 괜찮아 보여도 2D로 보면 벽 중복이 드러남.
```bash
# WSL에서
scp daehan@192.168.45.100:~/ros2_ws/maps/my_real_map_v2.* /mnt/c/ros2slam/maps/
cd /mnt/c/ros2slam/maps
python3 -c "from PIL import Image; Image.open('my_real_map_v2.pgm').save('my_real_map_v2.png')"
```

---

## 🔜 다음 할 일

### 우선순위 1 — 5단계 자율주행 (Nav2) ★다음 세션
저장된 `my_real_map_v2`를 기반으로 A(지도)→B(단일목표)→C(다지점순회). 최종목표 C.

**준비사항**:
- **★turn_gain 1.0으로 복귀** — Nav2 컨트롤러가 회전반경을 직접 계산하므로 증폭하면 경로추종이 틀어짐.
  (teleop 수동주행용 5.0은 Nav2에서 쓰면 안 됨)
- Nav2 설치: `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
- **AMCL**로 저장된 지도 위 위치추정 (slam_toolbox 대신)
- **Rotation Shim Controller** — 회전먼저→직진 방식. 데드존(int8 80~127, 47칸)으로 곡선이 불가능한 본 로봇에 적합
- **goal_tolerance 넉넉히** — 최저속도 0.225m/s 고정이라 정밀접근 시 오버슈트 가능
- 배터리 필수 (QCY PB10C 10000mAh)

### 우선순위 2 — 통합 런치파일
현재 매번 8개 창을 수동으로 띄워야 함. 하나로 묶을 것:
static TF ×2 → 라이다 → 브릿지 → rf2o(no_tf) → EKF → slam(또는 AMCL)

### 우선순위 3 — 남은 개선 여지 (급하지 않음)
- **직진 시 미세하게 휨** — `right_trim` 재조정. 바닥 직선 따라 2m 주행으로 값 찾기
- **IMU 타임스탬프** — 브릿지가 `now()` 사용(측정 시각 아님). 현재 품질로 충분하나 더 올리려면 손볼 것
- **라이다 받침 강성** — ㄷ자 종이박스. ㅁ자로 막거나 기둥 보강하면 진동 감소
- **엔코더** — 근본 해결책이지만 현재 불필요. 면접에서 "개선한다면?" 질문엔 "엔코더 추가가 1순위"로 답하는 게 정석

---

## 실행법 (현재 — 수동 7창)

```bash
# 창1 — static TF (laser)
source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0.02 0 0.20 0 0 0 base_link laser

# 창2 — static TF (imu)
source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link

# 창3 — 라이다 (★sllidar_ros2)
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/rplidar frame_id:=laser

# 창4 — 브릿지 (IMU 발행 포함) ★gyro_scale 65.5 필수 (±500dps)
source ~/ros2_ws/install/setup.bash
ros2 run amr_bridge bridge_node --ros-args -p turn_gain:=5.0 -p cmd_timeout:=0.8 -p gyro_scale:=65.5

# 창5 — rf2o (★TF 끈 버전)
source ~/ros2_ws/install/setup.bash
ros2 launch /home/daehan/ros2_ws/launch/rf2o_no_tf.launch.py

# 창6 — EKF
source /opt/ros/jazzy/setup.bash
ros2 run robot_localization ekf_node --ros-args --params-file /home/daehan/ros2_ws/config/ekf.yaml

# 창7 — slam
source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/ros2_ws/config/my_mapper.yaml

# 창8 — teleop (i전진 ,후진 j좌 l우 k정지)
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**검증 명령**:
```bash
ros2 run tf2_ros tf2_echo map base_link    # 전체 체인 확인
ros2 run tf2_ros tf2_echo map odom         # ★slam 보정량 = EKF 정확도 지표
ros2 topic hz /imu /odom_rf2o /scan        # 입력 확인
```

**WSL RViz**: `rviz2` → Fixed Frame `map`, Add → Map(/map) + LaserScan(/scan)

---

## 하드웨어 현황

**추가할 부품 없음.** 현재 구성으로 5단계(Nav2)까지 가능.

- 라이다: 박스 위 **높이 20cm** (이전 21.3cm에서 낮춤). Pi/보조배터리는 라이다 다리 높이까지라 시야 간섭 없음
- **IMU 장착 주의**: 로봇 본체(단단한 곳)에 **수평**으로 고정. 흔들리는 박스에 붙이면 박스 진동까지 회전으로 읽힘
- 라이다 받침이 ㄷ자(앞뒤 뚫림) 종이박스 → 비틀림 강성 약함. 매핑 품질 안 나오면 ㅁ자로 막거나 기둥 보강 고려
- 라인트레이서 3개(A3/A4/A5) 제거 — SLAM 프로젝트에 불필요, A4/A5는 I2C 전용

---

## C++ 증명 전략 (취업 핵심)

1. **[3-B+4단계] 하드웨어 인터페이스 노드 C++** ✅ 완료
   - Packet 재사용, termios 시리얼, cmd_vel→연속PWM
   - **★IMU 추가로 강화**: 가변길이 패킷 파서(LEN 분기), int16 센서 패킷 설계, sensor_msgs/Imu 발행
   - "전송수단 교체해도 패킷 계층 불변" + "물리 캘리브레이션을 상위 계층에서 보상" + **"센서 추가 시 기존 경로 무수정"**
2. **[★신규] 센서 퓨전 설계·디버깅 경험**
   - rf2o 회전 실패를 실측으로 규명 → IMU 도입 결정 → EKF 축별 선택 융합
   - 공분산 규약 버그(−1 = 데이터 무효) 발견·수정
   - "한 센서의 약점을 다른 센서로 보완하되, 완전히 배제하지 않는다"는 설계 교훈
3. **[5단계 이후] ros2_control 하드웨어 인터페이스 플러그인** — C++ 전용, AGV 업계표준
4. **[5단계 이후] Nav2 커스텀 플러그인** — nav2_core 상속→순수가상→pluginlib

---

## 저장소 커밋 예정 (github.com/jungdaehan11/amr-ros2-slam)

- `arduino/amr_usb_imu.ino` — ★0x22 자이로 패킷 + MPU-6050 I2C + 바이어스 보정
- `amr_bridge/` 갱신 — Packet.h/.cpp에 `parseArduinoSensor16`/`dataToInt16`, bridge_node.cpp 가변길이 RX + /imu 발행
- `config/ekf.yaml` — ★robot_localization EKF 설정
- `config/my_mapper.yaml` — minimum_travel 조정본
- `launch/rf2o_no_tf.launch.py` — rf2o TF 비활성 런치
- `docs/HANDOVER6.md` — 이 문서
- `maps/my_real_map_v2.pgm/.yaml` — ★실주행 지도 (4단계 결과물). v1은 벽 중복으로 폐기

---

## 📌 다음 세션 시작 체크리스트

1. HANDOVER6.md 업로드 → 맥락 복원
2. 8창 띄우기 (아래 "실행법") — **브릿지에 `gyro_scale:=65.5` 빠뜨리지 말 것**
3. `tf2_echo map base_link`로 TF 체인 확인
4. Nav2 설치 후 `my_real_map_v2`로 AMCL 기동
5. **turn_gain 1.0으로 복귀** 후 Nav2 테스트
