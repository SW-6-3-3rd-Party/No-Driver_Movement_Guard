# MAIN partMAIN ECU
VAPS(Vehicle Anti-rollaway Protection System) MAIN ECU 소스코드입니다.

개발 환경
항목	내용
보드	TC375 Lite Kit V2
IDE	AURIX Development Studio (ADS) 1.10.32
컴파일러	TASKING VX-toolset
RTOS	FreeRTOS (싱글코어, Core0)
드라이버	iLLD (TC37A)
프로젝트 구조
iLLD_TC375_ADS_FreeRTOS_Basic/
├── App/
│   ├── sensor_data.h        # 공유 데이터 구조체
│   ├── sensor_driver.c      # Task_Sensor (센서 읽기 + 판정)
│   ├── state_machine.c      # Task_Judge (상태 머신 + 명령)
│   ├── ultrasonic_isr.h     # ERU 인터럽트 헤더
│   ├── ultrasonic_isr.c     # ERU 인터럽트 구현
│   └── can_handler.c        # CAN 통신 (미구현)
├── Cpu0_Main.c              # 엔트리포인트
├── OS/                      # FreeRTOS 커널
├── Libraries/               # iLLD 드라이버
└── Configurations/          # 링커/SSW 설정
태스크 구조
ERU ISR (Priority 40)
  └─ 초음파 Echo Rising/Falling Edge → STM 타임스탬프 캡처

Task_Sensor (10ms, Priority 3)
  ├─ 기어/도어 GPIO + 디바운싱
  ├─ 초음파 비블로킹 상태머신 + 5샘플 필터
  ├─ 압력센서 EVADC 읽기
  ├─ 운전자 퓨전 판정
  └─ LED 제어 + g_sensor 갱신

Task_Judge (50ms, Priority 3)
  ├─ 상태 머신 (NORMAL → WARN → BRAKE → HOLD → RELEASE)
  └─ g_command 갱신
핀 할당
센서	Arduino 핀	AURIX 포트	인터페이스
압력 FSR	A1	P40.8 (AN38)	ADC
ToF	SCL/SDA	P13.1/P13.2	I2C0
초음파 Trig	D2	P02.0	GPIO
초음파 Echo	D3	P02.1	ERU 인터럽트
기어 P/R/N/D	D5/D6/D7/D8	P02.3/5/4/6	GPIO
도어	D9	P02.7	GPIO
외부 LED	D4	P10.4	GPIO
빌드
ADS에서 프로젝트 Import 후 Build Active Project 실행.