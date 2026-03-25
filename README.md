# No-Driver_Movement_Guard
This project is the first project of the Hyundai AutoEver Mobility SW School, focused on developing an on-board safety system to prevent unintended vehicle movement in driver-absent situations.

## 배경  
본 프로젝트는 운전자 부재 상태에서 부적절한 기어 상태로 인해 차량이 의도치 않게 움직이는 사고를 예방하기 위한 차량 안전 제어 시스템이다.
운전자가 차량을 완전히 정차시키지 않거나, 기어를 N단 또는 D단 상태로 둔 채 하차하는 경우 차량이 이동하여 보행자 충돌, 차량 손상, 운전자 끼임 사고 등이 발생할 수 있다.
본 시스템은 이러한 상황을 실시간으로 감지하고, 차량의 상태를 안전하게 제어함으로써 사고를 사전에 방지하는 것을 목표로 한다.
이러한 유형의 사고는 국내외에서 지속적으로 발생하고 있으며, 운전자의 단순 부주의로 인해 발생하는 대표적인 차량 안전 문제 중 하나로 지적되고 있다.

### 사례  
후진 기어 놓고 내려 20대 운전자 사망 - https://www.joongang.co.kr/article/25402548 
후진 기어 놓고 내려 40대 운전자 사망 - https://www.yna.co.kr/view/AKR20260313174500063 
후진 기어 놓고 내려 50대 운전자 사망 - https://news.jtbc.co.kr/article/NB12277706 
D단에 놓고 내려 보행자와 추돌 - https://www.kyeonggi.com/article/20260122580550 

### 시나리오  
차량을 N단이나 D단에 두고 운전자가 차 문을 열었을 때 스피커로 경고 메시지를 알려준다.(parking으로 안되어 있다는 알림)
바퀴에 인코더 장치가 있어서 만약 차량이 움직이면 적외선 센서로 운전자 좌석에 사람의 존재 여부 확인
초음파 센서를 이용하여 주변에 충돌
만약 장애물
