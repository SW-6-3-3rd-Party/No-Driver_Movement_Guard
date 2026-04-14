# MAIN ECU

VAPS 시스템의 중앙 제어 ECU입니다. 센서 데이터를 수집하고 운전자 탑승 여부를 판정하며, 위험 상태를 판단하여 ACT/CLU ECU에 제어 명령을 전달합니다.

## 개발 환경

| 항목 | 내용 |
|---|---|
| MCU | Infineon TC375 Lite Kit V2 (TriCore, 300MHz, 3-core) |
| IDE | AURIX Development Studio (ADS) 1.10.32 |
| 컴파일러 | TASKING VX-toolset |
| RTOS | FreeRTOS 10.x (싱글코어, Core0) |
| 드라이버 | iLLD (TC37A) |
| ToF 라이브러리 | ST VL53L0X API v1.0.4 |

## 프로젝트 구조

```
iLLD_TC375_ADS_FreeRTOS_Basic/
├── App/
│   ├── sensor_data.h        # 공유 데이터 구조체 (SensorData, ControlCommand)
│   ├── sensor_driver.c      # Task_Sensor: 센서 취득 + 운전자 판정
│   ├── state_machine.c      # Task_Judge: 상태 머신 + 제어 명령 생성
│   ├── ultrasonic_isr.h     # ERU 인터럽트 헤더
│   ├── ultrasonic_isr.c     # ERU 인터럽트 구현
│   ├── tof_sensor.h         # ToF 센서 래퍼 헤더
│   ├── tof_sensor.c         # Task_ToF: VL53L0X 블로킹 측정
│   ├── can_handler.h        # CAN 통신 헤더
│   └── can_handler.c        # Task_CAN: CAN 송수신
├── VL53L0X_1.0.4/           # ST 공식 VL53L0X API
│   └── Api/
│       ├── core/            # 센서 제어 로직
│       └── platform/        # TC375 I2C 플랫폼 레이어
├── Cpu0_Main.c              # 엔트리포인트: GPIO/센서/CAN 초기화 + 태스크 생성
├── OS/                      # FreeRTOS 커널
├── Libraries/               # iLLD 드라이버
└── Configurations/          # 링커/SSW 설정
```

## 태스크 구조

```
┌─ ERU ISR (Priority 40, 하드웨어) ────────────┐
│  Echo Rising/Falling Edge → STM 타임스탬프     │
│  CPU 점유: 수 us                               │
└───────────────────────┬──────────────────────┘
                        │ g_ultra (비블로킹)
                        ▼
┌─ Task_Sensor (10ms, Priority 3) ─────────────┐
│  기어/도어 GPIO 디바운싱 + press-release 래치  │
│  초음파 비블로킹 상태머신 (Trigger/Wait/Cool)  │
│  EVADC 3채널 압력센서 취득                     │
│  ToF 최신값 읽기                               │
│  단위 변환 + 3샘플 중앙값 필터                 │
│  신뢰도 점수 기반 운전자 판정                  │
│  g_sensor 갱신 (Mutex)                         │
└───────────────────────┬──────────────────────┘
                        │ g_sensor (Mutex)
                        ▼
┌─ Task_Judge (50ms, Priority 3) ──────────────┐
│  상태 머신 (NORMAL→WARN→BRAKE→HOLD→RELEASE)   │
│  제어 명령 생성 (brake_cmd, risk_level)        │
│  g_command 갱신 (Mutex)                        │
└───────────────────────┬──────────────────────┘
                        │ g_command (Mutex)
                        ▼
┌─ Task_CAN (50ms, Priority 4) ────────────────┐
│  0x100: MAIN→ACT 제어 명령 (50ms 주기)        │
│  0x300: MAIN→CLU 상태 표시 (100ms 주기)       │
│  0x200: ACT→ALL 피드백 수신                    │
└──────────────────────────────────────────────┘

┌─ Task_ToF (50ms, Priority 2) ────────────────┐
│  VL53L0X 단일 측정 (블로킹, ~30ms)            │
│  I2C 에러 시 자동 복구 (Soft Reset + 재초기화) │
│  g_tofDistanceMm 갱신                          │
└──────────────────────────────────────────────┘
```

## 핀 할당

| 센서 | Arduino 핀 | AURIX 포트 | 인터페이스 |
|---|---|---|---|
| 압력 FSR #1 | A3 | P40.6 (AN36) | ADC Group8 Ch4 |
| 압력 FSR #2 | A2 | P40.7 (AN37) | ADC Group8 Ch5 |
| 압력 FSR #3 | A1 | P40.8 (AN38) | ADC Group8 Ch6 |
| ToF SCL/SDA | SCL/SDA | P13.1/P13.2 | I2C0 |
| 초음파 Trig | D2 | P02.0 | GPIO 출력 |
| 초음파 Echo | D3 | P02.1 | ERU 인터럽트 |
| 기어 P/R/N/D | D5/D6/D7/D8 | P02.3/5/4/6 | GPIO (active-high) |
| 도어 스위치 | D9 | P02.7 | GPIO |
| 외부 LED | D4 | P10.4 | GPIO 출력 |
| CAN TX | - | P20.8 | CAN0 |
| CAN RX | - | P20.7 | CAN0 |

## 주요 구현 사항

### 초음파 센서 (ERU 인터럽트)

폴링 방식(160ms CPU 점유)에서 ERU 인터럽트 방식(12us Trigger + 수us ISR)으로 변경하여 10ms 태스크 주기를 보장합니다. ERU 설정은 iLLD API 대신 레지스터 직접 접근으로 구현하였습니다.

```
EICR[1]: EXIS0=1(REQ2B=P02.1), FEN0=1, REN0=1, EIEN0=1, INP0=2
IGCR[1]: IPEN02=1, IGP0=01
SRC_SCUERU2 (0xF0038888): SRPN=40, TOS=CPU0, SRE=1
```

### 압력센서 (EVADC)

3개 FSR 센서를 EVADC Group8의 Channel 4/5/6에 할당하고, Queue REFILL 모드로 연속 변환합니다. 비블로킹 읽기(VF 체크 후 즉시 리턴)로 태스크 주기에 영향을 주지 않습니다.

### 운전자 판정 (신뢰도 점수)

단순 임계값 비교 대신, 센서별 7구간 가우시안 점수(0/1/3/5/3/1/0)를 부여하고 가중 평균(초음파 0.3, ToF 0.3, 압력 0.4)으로 종합합니다. 연속 3회 동일 판정이 유지되어야 상태가 확정됩니다. 물건 적재(압력만 높고 거리 미감지)와 상체만 진입(거리만 감지되고 압력 없음) 시나리오에 대한 오판 방지 로직이 포함되어 있습니다.

### 상태 머신

```
NORMAL → WARN_LV1 → WARN_LV2 → D_BRAKE/R_BRAKE → BRAKE_HOLD → RELEASE_WAIT → NORMAL
                                ROLLAWAY_WARN → ROLLAWAY_BRAKE → BRAKE_HOLD
```

P단 전환 시 즉시 NORMAL 복귀(SAF-01), 착석+도어닫힘 시 NORMAL 복귀(SAF-02)가 적용됩니다.

### CAN 통신

| CAN ID | 방향 | 주기 | 내용 |
|---|---|---|---|
| 0x100 | MAIN→ACT | 50ms | brake_cmd, gear_state |
| 0x200 | MAIN→CLU | 100ms | risk_level, driver_present, door_state, gear_state |
| 0x300 | ACT→ALL | 100ms | speed, brake_state, accel_x, accel_y, accel_z |

### 공유 데이터 보호

FreeRTOS Mutex(xSemaphoreCreateMutex)로 g_sensor와 g_command를 보호합니다. 우선순위 상속이 자동 적용되어 Task_CAN(Priority 4)과 Task_Sensor(Priority 3) 간 우선순위 역전이 방지됩니다. 모든 Mutex 접근은 5ms 타임아웃으로 데드락을 방지합니다.

## 디버깅 이력

개발 과정에서 해결한 주요 하드웨어/소프트웨어 이슈입니다.

| 문제 | 원인 | 해결 |
|---|---|---|
| ERU ISR 미호출 | EIEN0/INP0 비트 누락 | IfxScu_regdef.h에서 비트 위치 확인 후 추가 |
| SRC 주소 오류 | SRC_SBCU(0xF0038028) 사용 | IfxSrc_reg.h 확인 → 0xF0038888로 수정 |
| Bus Error 트랩 | SCU 레지스터 ENDINIT 미해제 | Safety/CPU ENDINIT 해제 추가 |
| I2C 레지스터 읽기 실패 | iLLD I2C가 Repeated Start 미지원 | VL53L0X 공식 API + 플랫폼 레이어로 전환 |
| 기어 4핀 동시 LOW | 택트 스위치 방향 오류 (1-2쌍 단락) | 스위치 방향 수정, active-high로 확정 |
| ADC 값 갱신 지연 | do-while 폴링 블로킹 | VF 체크 후 즉시 리턴 (비블로킹) |
| CAN Bus-Off | 트랜시버 STB 핀 미초기화 | STB 핀 LOW 설정 추가 |

## 빌드

AURIX Development Studio에서 프로젝트를 Import한 후 Build Active Project를 실행합니다. FreeRTOS 데모 프로젝트(iLLD_TC375_ADS_FreeRTOS_Basic)를 기반으로 작업하였으므로 별도의 링커/Include 설정 없이 빌드됩니다.
