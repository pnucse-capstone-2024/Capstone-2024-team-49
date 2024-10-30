# Joy Security
## 1. 연구 배경

현대의 드론 기술이 발전하며 군사, 건설, 농업, 영상 촬영 등 다양한 분야에서 활용되고 있으며, 시장 또한 빠르게 성장하고 있다. 또한, 향후 물류 수송, 교통 관제, 통신 등 새로운 서비스/산업 분야로 그 영역을 확대하는 등 새로운 성장동력으로 주목받고 있다.

미래 항공 산업의 핵심 기술로 드론이 부각되면서 미국, EU, 중국, 일본 등 세계 각국에서 무인 항공기의 단계별 발전 로드맵을 발표하여, 드론 산업 육성과 시장 활성화에 노력을 기울이고 있다. 국내에서도 드론 산업 발전 기본 계획으로 네거티브 방식의 규제 최소화를 통한 드론 산업 실용화를 위해 법제도적 지원을 하고 있다.

하지만 이러한 급격한 UAV 시장의 성장은 UAV의 취약점을 악용한 다양한 공격의 급증을 유발하고 있다. UAV는 원거리에서 무선으로 원격 조정하거나 입력된 프로그램에 따라 비행하고, 센서를 통해 데이터를 전송한다. 이처럼 드론과 지상 제어 장치가 네트워크로 연결되어 있기 때문에, 드론이 탈취당하거나 서비스 장애가 발생할 수 있다.

오픈소스 드론 시스템인 PX4는 통신 프로토콜로 MAVLink를 사용한다. MAVLink는 2.0버전에서 signature 필드의 도입으로 무결성을 보호하고자 하였다. 그러나 PX4에서는 해당 기능을 사용하지 않아 중간자 공격이나 메시지 변조 공격 등의 위협으로부터 취약하다.

---
## 2. 연구 목표

<img src="https://github.com/user-attachments/assets/0c61775e-79e7-43d3-9e10-134632817be4" width="350" height="200"/>

- MAVLink 메시지의 무결성을 보완하고자 서명 기반의 메시지 인증 기능을 추가한다.
- 이에 사용될 키는 MAVLink 통신에서의 ECDH 알고리즘 적용을 통해 안전한 방식으로 대칭키를 가질 수 있도록 한다.
- 같은 키를 이용해 서명을 생성하고, 검증하였을 때 만일 일치하지 않는다면 해당 메시지를 무시할 수 있도록 한다.

---
## 3. 연구 설계
### 3.1 연구 환경
| Tool              | Version  |
|-------------------|----------|
| **Ubuntu**        | 20.04.06 |
| **PX4-Autopilot** | 1.14.3   |
| **ROS**           | Noetic   |
| **QGC**           | 4.4.1    |
| **Qt**            | 6.6.3    |
| **Gazebo**        | 11.0     |
### 3.2 Flow Chart
<img src="https://github.com/user-attachments/assets/73dc88d0-1e14-4be7-a23b-746c25cfeef4" width="60%" height="60%"/>

### 3.3 ECDH
- PX4와 QGC의 통신 채널에는 공개키만 노출됨
- 난수로 생성된 비밀키를 기반으로 공개키 생성
- 서로의 공개키를 교환한 뒤 자신의 비밀키와 상대방의 비밀키를 기반으로 공유 비밀키를 연산
### 3.4 Signature
- Signature의 생성은 다음 식과 같이 이루어진다. <br>
𝑠𝑖𝑔𝑛𝑎𝑡𝑢𝑟𝑒 = 𝑠ℎ𝑎256_48(𝑠𝑒𝑐𝑟𝑒𝑡_𝑘𝑒𝑦 + ℎ𝑒𝑎𝑑𝑒𝑟 + 𝑝𝑎𝑦𝑙𝑜𝑎𝑑 + 𝐶𝑅𝐶 + 𝑙𝑖𝑛𝑘_𝐼𝐷 + 𝑡𝑖𝑚𝑒𝑠𝑡𝑎𝑚𝑝)
- _secret key_ 는 ECDH 알고리즘을 통해 생성된 공유 비밀키이다.

---
## 4. 연구 내용
### 4.1 Enable Signature
<img src="https://github.com/user-attachments/assets/52f09a98-54ed-44d6-8c3f-865eba48f6a1" width="60%" height="60%"/>

- QGC 실행 후 Application Settings의 Telemetry 메뉴에서 "Enable Signature" 버튼 클릭
 &rarr; sendSetupSigning 함수 호출하며 동시에 signing stream 설정됨 

<img src="https://github.com/user-attachments/assets/b0f33a6d-a0a1-4cb0-b8ee-29b76b3710f9" width="70%" height="70%"/>

### 4.2 Signature Verification
- 전송 받은 메시지의 데이터에 SHA256 해시함수를 적용하여 도출된 결과값을, 메시지에 포함된 signature와 비교하여 검증
  

|<img src="https://github.com/user-attachments/assets/8a126c0d-c010-4638-9d66-6deecf1ebf99"/> | <img src="https://github.com/user-attachments/assets/ee562d12-3fde-4f67-9577-c89bad3076cf"/>|
|:--:|:--:|
| **서명 검증에 성공**  | **서명 검증에 실패** |  

### 4.3 Virtual attack simulation
- 공격자가 PX4의 UDP port로 접속을 하여 px4와 연결에 성공하여도, 서명이 활성화 되어 있는 상태라면, 공격자의 제어 명령이 무시된다.
- 키 값이 노출되어 공격자가 서명을 생성할 수 있게 되어도, 서명 스트림이 공격자의 system id와 component id를 가리키지 않기 때문에 서명 검증에 실패하게 된다.
  
  <img src="https://github.com/user-attachments/assets/fdba3b69-1364-4a30-9e58-6ac5d6bf4ead" width="60%" height="60%"/>




---
## 5. 설치 및 사용법
- 아래의 설치 방법은 간략한 설명으로, 설치 과정에서 문제 발생 시 참고 링크 또는 공식 문서를 참고하면 도움이 될 수 있음.

<br />

### 5.1. Ubuntu Desktop 설치 (드론 소유자, 공격자)

- ROS Noetic과의 호환을 위해 20.04 버전 다운로드
    - 다운로드 링크: https://ubuntu.com/download/alternative-downloads
    - Ubuntu 홈페이지 - Alternative downloads - Ubuntu 20.04 LTS (Focal Fossa) - Desktop image
- 이미지 적용 후 VMWare, VirtualBox 등에서 가상 머신 생성

<br />

### 5.2. PX4-Autopilot 설치 (드론 소유자)

- 참고 링크: https://docs.px4.io/v1.14/ko/dev_setup/dev_env_linux_ubuntu.html

**5.2.1. PX4 펌웨어 설치**

- (Ubuntu) 터미널에 아래의 명령어를 입력

```
$ git clone https://github.com/PX4/PX4-Autopilot.git --recursive
$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
```

- PX4-Autopilot 디렉토리에 대해, Github에 업로드된 디렉토리 및 파일을 덮어쓰기

**5.2.2. Gazebo 시뮬레이터 설치**

- 참고 링크: https://docs.px4.io/v1.14/ko/sim_gazebo_gz/
- 터미널에 아래의 명령어를 입력

```
$ sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install gz-garden
```

**5.2.3. PX4 시뮬레이터 빌드**

- PX4-Autopilot 디렉토리로 이동 후, 터미널에 아래의 명령어를 입력하여 Gazebo 빌드 및 실행

```
$ make px4_sitl gazebo
```

<br />

### 5.3. QGroundControl 설치

**5.3.1. QGroundControl (드론 소유자)**

- 아래의 링크에서 압축 파일을 받고 압축 해제, 디렉토리 내 “QGroundControl” 파일을 실행
    - https://www.dropbox.com/scl/fi/k9l7mtvhrin79zed75nku/Release.tar.xz?rlkey=5havfnomwqjyogiv1tkw6q1sk&st=0ii45tvw&dl=0
- 다른 방법으로 qgroundcontrol 디렉토리에 대해, Github에 업로드된 디렉토리 및 파일을 덮어쓴 후 Qt를 이용하여 직접 빌드

**5.3.2. QGroundControl (공격자)**

- 아래 참고 링크의 “Ubuntu Linux” 항목에서, AppImage 파일을 다운로드 받고, 실행 권한 변경
    - 참고 링크: https://docs.qgroundcontrol.com/Stable_V4.3/en/qgc-user-guide/getting_started/download_and_install.html

<br />

### 5.4. ROS, MAVROS 설치 (드론 소유자)

**5.4.1. ROS Noetic 설치**

- 아래 참고 링크의 내용을 따라 설치 진행
    - 참고 링크: https://wiki.ros.org/noetic/Installation/Ubuntu#Installation
- 터미널에서 아래의 명령어 입력 시, ROS가 실행됨

```
$ roscore
```

**5.4.2. MAVROS 빌드**

- catkin_ws 디렉토리에 대해, Github에 업로드된 디렉토리 및 파일을 덮어쓰기
- catkin_ws 디렉토리로 이동 후, 터미널에 아래의 명령어 입력하여 MAVROS 빌드

```
$ catkin build
```

- 빌드가 끝나면, catkin_ws 디렉토리의 터미널에 아래의 명령어 입력하여 MAVROS 실행

```
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ roslaunch mavros px4.launch
```

<br />

### 5.5. key 디렉토리 생성 (드론 소유자)

- home 디렉토리로 이동 후, 터미널에 아래의 명령어를 입력하여 key 디렉토리를 생성

```
$ mkdir key
```

<br />

### 5.6. 사용법

- 위 과정을 모두 마치면 Gazebo 시뮬레이터의 PX4 드론과 QGC가 연결이 됨
- 
**5.6.1. 사용**

- 가상 머신에서 ROS, MAVROS를 실행
- PX4-Autopilot을 실행
- QGC 실행 후 QGC의 Application settings -> Telemtry 메뉴에서 "Enable Signing" 버튼을 클릭하면 서명이 활성화 됨

---
## 6. 소개 영상
[![2024 전기 졸업과제 Joy Security](https://img.youtube.com/vi/Ctr1C__ml1Q/0.jpg)](https://www.youtube.com/watch?v=Ctr1C__ml1Q&t=6s)


---
## 7. 팀 소개
|팀원   |학번   | 역할 | 
|---|---|---|
| **이경민** |201924523   | - 시나리오 설계 <br> - ECDH 알고리즘 적용 <br> - signature 필드 생성 및 적용 <br> - 비행 로그 분석|
| **조수현** |201624587   | - 환경 구축 <br> - custom message를 이용한 통신 메시지 설계 <br> - 서명 검증 과정 구현 <br> - 서명 검증에서의 오버헤드 측정|
