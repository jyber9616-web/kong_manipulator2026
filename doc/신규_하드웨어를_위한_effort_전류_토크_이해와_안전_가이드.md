# 신규 하드웨어를 위한 effort·전류·토크 이해와 안전 가이드

## 1. 문서 목적

이 문서는 새로운 로봇 하드웨어를 제작하는 교육생이
ROS 2의 `effort`를 정확히 이해하고 안전하게 설계하도록 돕는다.

특히 다음과 같은 오해를 방지하는 것이 목적이다.

- `effort`는 항상 모터 전류 raw 값이다.
- `effort`는 항상 정확한 토크 N·m다.
- `/joint_states.effort` 값을 바꾸면 모터가 더 강하게 움직인다.
- 전류가 높으면 무조건 이상이고 낮으면 무조건 정상이다.
- 위치 제어와 effort 제어는 같은 방식으로 동작한다.

학습 목표는 다음과 같다.

1. state effort와 command effort를 구분한다.
2. 전류, 모터 토크, 관절 토크의 관계를 설명한다.
3. 회전 joint와 직선 joint의 effort 단위를 구분한다.
4. 위치 제어, 전류 제어, current-based position control의 차이를 설명한다.
5. 새로운 hardware plugin에서 effort 단위와 부호를 올바르게 설계한다.
6. 전류 제한, 발열, 충돌, stall 위험을 포함한 안전 검증을 수행한다.

---

## 2. ROS에서 effort의 표준 의미

`sensor_msgs/msg/JointState`에서 effort의 표준 의미는 다음과 같다.

| joint 종류 | effort의 물리적 의미 | 표준 단위 |
|---|---|---|
| revolute | joint에 작용하는 토크 | N·m |
| continuous | joint에 작용하는 토크 | N·m |
| prismatic | joint 축 방향 힘 | N |

예:

```yaml
name:
  - shoulder_joint
  - linear_joint
effort:
  - 1.2
  - 35.0
```

표준 의미대로 구현되었다면:

```text
shoulder_joint effort = 1.2 N·m
linear_joint effort   = 35.0 N
```

그러나 실제 hardware plugin이 raw current나 ADC count를 그대로 넣으면
메시지 필드 이름은 effort여도 표준 단위를 만족하지 않을 수 있다.

따라서 값만 보고 단위를 추측하지 말고 hardware 구현과 모델 파일을 확인해야 한다.

---

## 3. state effort와 command effort

가장 먼저 두 개념을 구분해야 한다.

### 3.1 state effort

현재 관절에 발생하거나 추정된 effort 피드백이다.

```xml
<state_interface name="effort"/>
```

예:

- 토크 센서 측정값
- 모터 전류 기반 토크 추정값
- strain gauge 측정값
- 모델 기반 외력 추정값
- 하드웨어가 제공하는 load 값

`joint_state_broadcaster`는 이 값을 `/joint_states.effort`로 발행할 수 있다.

### 3.2 command effort

controller가 hardware에 요구하는 effort 명령이다.

```xml
<command_interface name="effort"/>
```

예:

- 목표 관절 토크
- 목표 모터 전류
- 목표 힘

### 3.3 핵심 차이

```text
state effort
  = 지금 얼마나 힘이 발생하고 있는가?

command effort
  = 얼마의 힘을 발생시키라고 명령하는가?
```

`/joint_states.effort`는 상태 메시지다.
이 토픽의 값을 수정하거나 새 메시지를 발행한다고 모터 명령이 바뀌는 것은 아니다.

---

## 4. 전류와 토크의 관계

이상적인 모터에서는 다음 관계를 사용할 수 있다.

```text
motor torque = torque constant × motor current

τ_motor = Kt × I
```

여기서:

- `τ_motor`: 모터 축 토크
- `Kt`: 모터 torque constant
- `I`: 모터 전류

감속기가 있다면 관절 출력 토크는 개념적으로 다음과 같다.

```text
τ_joint ≈ τ_motor × gear ratio × efficiency
```

즉:

```text
τ_joint ≈ Kt × I × gear ratio × efficiency
```

그러나 실제 장치에서는 다음 요인이 영향을 준다.

- 감속기 효율
- 마찰
- 백래시
- 온도
- 공급 전압
- 모터 편차
- 전류 측정 오차
- 링크와 transmission 구조
- 케이블과 씰의 저항
- 정적 마찰과 동적 마찰 차이

따라서 전류가 토크와 관련 있다는 사실과
전류 raw 값을 정확한 N·m로 간주하는 것은 서로 다른 문제다.

---

## 5. OpenManipulator-X의 effort 구현

OpenManipulator-X의 ros2_control Xacro에는 다음 state interface가 있다.

```xml
<joint name="${prefix}joint1">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

Dynamixel hardware interface의 매핑은 다음 구조다.

```cpp
ros2_to_dxl_state_map = {
  {position, {"Present Position"}},
  {velocity, {"Present Velocity"}},
  {effort,   {"Present Current", "Present Load"}}
};
```

즉 OpenManipulator-X의 joint effort는 Dynamixel의 `Present Current`에 연결된다.

```text
Dynamixel Present Current
  → ros2_control effort state interface
  → joint_state_broadcaster
  → /joint_states.effort
```

### 5.1 중요한 단위 주의

현재 저장소의 XM430-W350 모델 파일은 다음과 같이 정의되어 있다.

```text
Present Current  1.0  raw  signed  0.0
```

파일:

```text
open_manipulator_ws/src/dynamixel_hardware_interface/
└─ param/dxl_model/xm430_w350.model
```

따라서 이 구성의 `/joint_states.effort`는
정확히 보정된 관절 토크 N·m가 아니라
Dynamixel Present Current raw 값일 수 있다.

OpenManipulator-X의 effort를 사용할 때는 다음처럼 표현해야 한다.

```text
현재 값:
  전류 기반 상대 부하 지표

바로 단정하면 안 되는 값:
  정확한 관절 토크 N·m
```

---

## 6. XM430 전류값 해석 예

XM430-W350의 Present Current는 일반적으로 raw 1당 약 2.69 mA로 환산한다.

```text
current [A] ≈ raw × 2.69 / 1000
```

예:

```text
raw = 100
current ≈ 0.269 A

raw = 200
current ≈ 0.538 A

raw = 500
current ≈ 1.345 A
```

부호는 전류 또는 토크 방향을 나타낼 수 있다.

```text
+200과 -200
  → 크기는 유사
  → 방향은 반대
```

단, 최종 joint 부호는 다음 설정의 영향을 받는다.

- Dynamixel Drive Mode
- motor 방향
- joint axis
- transmission matrix
- offset
- gear 구조

### 주의

2.69 mA 환산은 전류값을 얻는 과정이다.
전류를 관절 토크 N·m로 바꾸려면 별도의 모델 또는 보정이 필요하다.

---

## 7. effort 값이 변하면 현재 로봇 동작이 바뀌는가

OpenManipulator-X의 arm controller 설정은 다음과 같다.

```yaml
arm_controller:
  ros__parameters:
    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

즉 arm controller는:

- position을 명령하고
- position과 velocity를 피드백으로 사용하며
- effort를 제어 피드백으로 사용하지 않는다.

따라서 현재 arm에서는 일반적으로 다음 순서다.

```text
목표 위치와 실제 위치의 오차 증가
  │
  ▼
Dynamixel 내부 위치 제어기가 더 큰 출력 요구
  │
  ▼
모터 전류와 토크 증가
  │
  ▼
Present Current 증가
  │
  ▼
/joint_states.effort 증가
```

현재 구조에서 effort 증가는 대부분 동작 변경의 명령이 아니라
모터가 더 큰 부하를 견디고 있다는 결과다.

---

## 8. 상황별 effort 해석

| 상황 | effort 절댓값의 일반적인 경향 | 해석 |
|---|---:|---|
| 공중에서 저속 이동 | 낮음 | 외부 저항이 작음 |
| 팔을 수평으로 유지 | 증가 | 중력 토크를 버팀 |
| 무거운 물체를 듦 | 증가 | 추가 하중 발생 |
| 급가속·급감속 | 순간 증가 | 관성 극복 |
| 외부에서 관절을 밀음 | 증가 또는 부호 변화 | 위치 유지 토크 발생 |
| 장애물에 접촉 | 증가 가능 | 목표 위치로 진행하지 못함 |
| 기구가 뻑뻑함 | 평소보다 증가 | 마찰 또는 정렬 문제 |
| 목표 위치 도착 | 보통 감소 | 위치 오차 감소 |

### effort가 높다고 항상 충돌은 아니다

다음 정상 상황에서도 effort는 높을 수 있다.

- 중력을 버티는 자세
- 빠른 가속
- 무거운 payload
- gripper가 물체를 잡는 중

### effort가 낮다고 항상 안전한 것도 아니다

다음 상황에서는 센서 또는 통신 오류로 0이 나올 수 있다.

- current 센서 미갱신
- 장치 통신 끊김
- raw 값 매핑 실패
- 잘못된 motor ID
- hardware `read()` 오류

따라서 effort 하나만으로 충돌이나 정상 상태를 판정하면 안 된다.

---

## 9. 위치 제어와 effort 제어의 차이

### 9.1 position control

```text
입력: 목표 위치
출력: 모터 내부 제어기가 필요한 전류를 결정
```

특징:

- 원하는 각도로 이동
- 하중이 증가하면 내부 전류가 자동 증가할 수 있음
- effort는 주로 상태 관찰값

### 9.2 current 또는 effort control

```text
입력: 목표 전류 또는 목표 토크
출력: 지정한 방향과 크기의 힘
```

특징:

- 위치를 직접 보장하지 않음
- 더 큰 명령은 일반적으로 더 큰 토크
- 부호는 토크 방향
- 외부 위치 제어 또는 안전 제약이 별도로 필요할 수 있음

### 9.3 current-based position control

```text
입력 1: 목표 위치
입력 2: 허용 전류 또는 토크 한도
```

특징:

- 목표 위치로 이동
- 출력 전류를 제한
- gripper처럼 위치와 잡는 힘을 함께 고려할 때 유용

---

## 10. OpenManipulator-X 그리퍼 주의사항

OpenManipulator-X의 arm motor는 Operating Mode 3을 사용한다.

```xml
<param name="Operating Mode">3</param>
```

이는 위치 제어 모드다.

그리퍼 motor는 Operating Mode 5를 사용한다.

```xml
<param name="Operating Mode">5</param>
<param name="Goal Current">200</param>
```

이는 current-based position control이다.

```text
Goal Position
  → 그리퍼가 닫히려는 위치

Goal Current
  → 위치 이동 중 사용할 수 있는 전류·토크 한도

Present Current
  → 실제로 측정된 현재 전류
```

`Goal Current: 200`은 `/joint_states.effort`를 200으로 고정한다는 뜻이 아니다.

### Goal Current를 높이면

- 더 강한 파지력이 가능
- 단단한 물체를 더 강하게 누를 수 있음
- 충돌력과 손상 가능성 증가
- 모터 발열 증가

### Goal Current를 낮추면

- 더 부드럽고 compliant한 파지
- 물체 손상 가능성 감소
- 무거운 물체를 놓칠 수 있음
- 마찰이 크면 목표 위치에 도달하지 못할 수 있음

교육 실습에서는 전류 한도를 올리는 것을
단순한 성능 향상으로 설명하면 안 된다.
힘, 발열, 수명, 충돌 위험이 함께 증가한다.

---

## 11. 신규 hardware plugin의 effort 설계

새 하드웨어에서 effort를 제공하기 전에 다음 질문에 답해야 한다.

1. effort의 원본 센서는 무엇인가?
2. 직접 측정인가, 전류 기반 추정인가?
3. 단위는 raw, A, N·m, N 중 무엇인가?
4. motor 축 값인가, 감속기 출력 joint 값인가?
5. 부호 기준은 URDF joint axis와 일치하는가?
6. offset과 bias를 어떻게 보정하는가?
7. 온도 변화에 따른 오차는 어느 정도인가?
8. 센서 포화 범위는 얼마인가?
9. 통신 오류 시 어떤 값을 반환하는가?
10. 정확도와 반복성을 어떻게 검증했는가?

### 11.1 권장 interface 설계

정확한 joint torque를 제공할 수 있다면:

```xml
<state_interface name="effort"/>
```

여기에 N·m 또는 N 단위 값을 제공한다.

raw current도 보존해야 한다면 custom interface를 추가한다.

```xml
<state_interface name="effort"/>
<state_interface name="motor_current_raw"/>
<state_interface name="motor_current"/>
```

예:

```text
joint1/effort            = 1.25 N·m
joint1/motor_current     = 0.42 A
joint1/motor_current_raw = 156
```

raw current만 있고 신뢰할 수 있는 토크 변환이 없다면:

- raw 값을 `effort`에 넣고 N·m라고 주장하지 않는다.
- 가능하면 `motor_current_raw` custom interface로 분리한다.
- 데이터 단위와 제한을 명확히 문서화한다.

---

## 12. joint torque 환산 시 고려사항

모터 전류에서 joint torque를 추정하는 기본식은 다음과 같다.

```text
τ_joint = Kt × I × gear_ratio × efficiency
```

하지만 실제 calibration에서는 다음을 고려해야 한다.

### 12.1 무부하 전류

모터가 움직이기만 해도 마찰 때문에 전류가 필요하다.

```text
측정 전류 = 부하 전류 + 마찰/무부하 전류
```

### 12.2 정방향과 역방향 차이

기어와 마찰 때문에 같은 토크에서도
회전 방향에 따라 전류가 다를 수 있다.

### 12.3 정지 마찰

움직이기 시작할 때와 이미 움직이는 동안의 전류가 다를 수 있다.

### 12.4 중력 영향

로봇팔 자세에 따라 같은 payload에서도 필요한 joint torque가 달라진다.

```text
팔을 아래로 늘어뜨린 자세
  → 중력 토크가 작을 수 있음

팔을 수평으로 뻗은 자세
  → 중력 토크가 커짐
```

### 12.5 transmission

벨트, 링크, 차동 기구 또는 다중 모터 구조에서는
motor current 하나를 joint torque 하나로 단순 매핑할 수 없다.

transmission matrix 또는 기구학적 변환이 필요하다.

---

## 13. effort calibration 권장 절차

### 단계 1. 센서 zero 확인

모터 torque를 끄거나 정의된 무부하 상태에서:

- raw current 평균
- 표준편차
- 온도
- 전원 전압

을 기록한다.

### 단계 2. 알려진 하중 적용

길이 `L`인 링크 끝에 알려진 질량 `m`을 설치한다.

수평 자세에서 단순화한 이론 토크는:

```text
τ = m × g × L
```

예:

```text
m = 0.5 kg
L = 0.2 m
g = 9.81 m/s²

τ = 0.5 × 9.81 × 0.2
  = 0.981 N·m
```

### 단계 3. 여러 하중 측정

한 점만 측정하지 않는다.

```text
0.0 kg
0.1 kg
0.2 kg
0.3 kg
0.4 kg
```

각 조건에서:

- 정방향
- 역방향
- 여러 자세
- 여러 온도

를 측정한다.

### 단계 4. 환산식과 오차 평가

예:

```text
joint torque = a × current + b
```

검증 데이터로 다음을 평가한다.

- 최대 오차
- 평균 오차
- 반복성
- hysteresis
- 포화 구간
- 온도 영향

### 단계 5. 문서화

다음을 반드시 남긴다.

- 센서와 motor 모델
- gear ratio
- 사용한 단위
- 환산식
- calibration 조건
- 유효 범위
- 오차
- 필터 설정
- 안전 제한

---

## 14. effort를 충돌 감지에 사용할 때

effort 또는 current 기반 충돌 감지는 가능하지만
단순 임계값 하나만 사용하면 오검출이 많다.

잘못된 예:

```python
if abs(effort) > 100:
    collision = True
```

이 방식은 다음 정상 상황도 충돌로 판단할 수 있다.

- 팔을 수평으로 유지
- payload 증가
- 급가속
- gripper 파지

개선 시 고려할 정보:

- 예상 중력 토크
- 목표 가속도
- joint 위치와 자세
- payload
- 속도
- 여러 sample 동안의 지속 시간
- effort 변화율
- 모터 온도
- position tracking error

개념적으로:

```text
측정 effort
  - 예상 중력 effort
  - 예상 관성 effort
  - 마찰 보정
  = 외력 후보
```

안전 기능은 effort 하나에만 의존하지 않고
position error, velocity, hardware fault, watchdog와 함께 사용한다.

---

## 15. 필터링 주의사항

전류와 effort 신호에는 노이즈가 있을 수 있다.

필터를 사용할 수 있지만 지연이 생긴다.

```text
강한 필터
  → 노이즈 감소
  → 충돌 감지 지연 증가

약한 필터
  → 빠른 반응
  → 오검출 증가
```

다음 값을 기록해 필터를 선택한다.

- controller update rate
- sensor update rate
- 필터 cutoff
- group delay
- 충돌 후 허용 정지 시간

안전 정지를 위한 신호와 화면 표시용 신호에
동일한 강한 필터를 무조건 적용하지 않는다.

---

## 16. command effort를 사용하는 경우

effort command를 사용하려면 다음 요소가 모두 필요하다.

1. URDF `<ros2_control>`의 effort command interface
2. hardware plugin의 effort command storage
3. `write()`에서 N·m 또는 N을 장치 명령으로 변환
4. effort를 지원하는 controller
5. 장치 operating mode 설정
6. current/torque limit
7. watchdog와 emergency stop

예:

```xml
<joint name="joint1">
  <command_interface name="effort"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

개념적인 `write()`:

```cpp
double requested_joint_torque = joint_effort_command_[0];
double requested_motor_current =
  joint_torque_to_motor_current(requested_joint_torque);

requested_motor_current =
  clamp(requested_motor_current, -current_limit, current_limit);

write_goal_current(requested_motor_current);
```

### 주의

effort controller를 추가했다고 장치 operating mode가 자동으로 바뀌는 것은 아니다.
하드웨어가 실제 current/torque mode에 있어야 하고,
단위 변환과 제한을 hardware plugin이 책임져야 한다.

---

## 17. 안전 요구사항

새로운 effort 제어 하드웨어에서는 다음 기능을 필수로 검토한다.

### 17.1 소프트웨어 제한

- 최대 command effort
- 최대 current
- 최대 effort 변화율
- 최대 velocity
- 최대 position 범위
- 통신 timeout
- command watchdog

### 17.2 하드웨어 제한

- 전원 전류 제한
- motor driver current limit
- fuse
- mechanical stop
- emergency stop
- torque-off 회로
- 온도 보호

### 17.3 열 관리

높은 current를 오래 유지하면 모터가 정지해 있어도 발열할 수 있다.

```text
움직임 없음 ≠ 출력 없음
```

관절이 목표 위치를 유지하며 큰 하중을 버티는 동안
전류와 열은 계속 발생할 수 있다.

### 17.4 stall 상태

목표 위치로 이동하지 못하고 큰 current가 지속되는 상황이다.

감지에 사용할 수 있는 조건:

```text
큰 position error
+ 낮은 velocity
+ 높은 current
+ 일정 시간 지속
```

stall에서는 즉시:

- command 감소
- hold 정책 전환
- torque disable
- fault 상태 보고

중 적절한 안전 동작을 수행해야 한다.

---

## 18. 실습 검증 절차

### 18.1 현재 메시지 확인

```bash
ros2 topic echo /joint_states --once
```

`name`과 `effort` 배열의 인덱스를 함께 확인한다.

### 18.2 정지 상태 baseline

로봇을 안전한 자세로 고정하고 10초 이상 기록한다.

확인:

- 평균
- 최소/최대
- 노이즈
- sign
- 온도 변화

### 18.3 자세 변화 시험

같은 payload에서:

- 팔을 아래로 둔 자세
- 팔을 수평으로 뻗은 자세

를 비교한다.

수평 자세에서 중력 토크와 effort 절댓값이 증가하는지 확인한다.

### 18.4 작은 외력 시험

비상 정지가 가능한 조건에서 한 관절에 작은 외력을 가한다.

확인:

- effort가 예상 방향으로 변하는가?
- 외력을 제거하면 baseline으로 돌아오는가?
- 다른 joint에 비정상적인 값이 나타나지 않는가?

### 18.5 stall 시험 주의

의도적인 stall 시험은 모터와 기구를 손상시킬 수 있다.

반드시:

- 낮은 current limit
- 짧은 시간
- 낮은 속도
- 온도 모니터링
- 즉시 torque-off 가능

조건에서 지도자 감독 아래 수행한다.

---

## 19. 자주 발생하는 오류

### 19.1 raw 값을 N·m로 표시

문제:

```text
effort = 200
→ 200 N·m라고 해석
```

실제로는 current raw 200일 수 있다.

해결:

- 모델 파일의 unit 확인
- hardware 변환 코드 확인
- custom current interface 사용
- calibration 수행

### 19.2 motor torque와 joint torque 혼동

감속기가 있으면 두 값은 다르다.

```text
motor shaft torque ≠ output joint torque
```

gear ratio, efficiency, transmission 구조를 포함해야 한다.

### 19.3 effort state를 command로 오해

`/joint_states.effort`는 상태다.
모터를 제어하려면 command interface와 controller가 별도로 필요하다.

### 19.4 sign 반전

가능한 원인:

- motor 장착 방향
- Drive Mode
- URDF axis
- transmission matrix

한 관절씩 양의 방향 시험으로 확인한다.

### 19.5 높은 effort를 무조건 충돌로 판단

중력, payload, 가속도와 마찰을 함께 고려해야 한다.

### 19.6 통신 실패값을 0으로 사용

0 effort는 정상적인 무부하 값과 구분되지 않는다.
fault 상태를 별도로 보고한다.

---

## 20. 교육생 제출 체크리스트

### 의미와 단위

- [ ] state effort와 command effort를 설명할 수 있다.
- [ ] revolute joint의 effort 단위가 N·m임을 알고 있다.
- [ ] prismatic joint의 effort 단위가 N임을 알고 있다.
- [ ] current raw, A, motor torque, joint torque를 구분한다.

### 구현

- [ ] effort의 센서 원본을 문서화했다.
- [ ] 변환식과 단위를 문서화했다.
- [ ] gear ratio와 transmission을 고려했다.
- [ ] joint axis와 effort 부호가 일치한다.
- [ ] raw 데이터가 필요하면 custom interface로 분리했다.

### 검증

- [ ] 정지 baseline을 측정했다.
- [ ] 알려진 하중으로 calibration했다.
- [ ] 정방향과 역방향을 모두 시험했다.
- [ ] 자세와 온도 변화 영향을 확인했다.
- [ ] sensor 포화와 통신 실패를 확인했다.

### 안전

- [ ] current limit이 설정되어 있다.
- [ ] command effort limit이 있다.
- [ ] watchdog가 있다.
- [ ] stall 감지 정책이 있다.
- [ ] temperature 보호가 있다.
- [ ] emergency stop 또는 torque-off 수단이 있다.
- [ ] 필터 지연을 측정했다.

---

## 21. 핵심 정리

1. ROS 표준 effort는 회전 joint에서는 N·m, 직선 joint에서는 N이다.
2. state effort는 측정 또는 추정된 현재 힘이고,
   command effort는 hardware에 요구하는 힘이다.
3. `/joint_states.effort`를 바꾼다고 모터가 제어되는 것은 아니다.
4. OpenManipulator-X에서는 effort가 Dynamixel `Present Current`에 연결된다.
5. 현재 XM430 모델 설정의 effort는 N·m가 아니라 raw current일 수 있다.
6. 전류가 증가하면 일반적으로 motor torque도 증가하지만,
   정확한 joint torque에는 gear, 효율, 마찰, 자세와 calibration이 필요하다.
7. arm의 position controller에서는 effort가 주로 부하 결과로 나타난다.
8. gripper의 current-based position mode에서는 Goal Current가 파지력 한도에 영향을 준다.
9. effort를 안전 기능에 사용할 때 단일 임계값에 의존하면 안 된다.
10. 새로운 하드웨어는 단위, 부호, 제한, timeout, 발열, stall을 반드시 검증해야 한다.

## 22. 참고 자료

- ROS JointState 메시지 정의:
  https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg
- ROS 2 Jazzy Joint State Broadcaster:
  https://control.ros.org/jazzy/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html
- ROBOTIS Current-Based Control Modes:
  https://www.robotis.us/robotis-ir-pr-blog/current-based-control-modes/
- ROBOTIS XM430-W350:
  https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
- OpenManipulator-X ros2_control 설정:
  `open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`
- Dynamixel hardware interface 매핑:
  `dynamixel_hardware_interface/include/dynamixel_hardware_interface/dynamixel_hardware_interface.hpp`

