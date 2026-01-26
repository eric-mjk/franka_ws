# Franka FR3 ROS2 Project (MoveIt2 + ros2_control)

---

## 🐳 Development Environment (Docker + ROS2)

이 프로젝트는 **Docker 기반 ROS2 Humble 개발 환경**을 사용합니다.
다른 컴퓨터에서도 동일한 환경을 빠르게 재현할 수 있도록 설계되어 있습니다.

---

### 1️⃣ 사전 요구사항 (Prerequisites)

다음이 설치되어 있어야 합니다.

* **Ubuntu 22.04**
* **Docker**
* **Docker Compose (v2)**

---

### 2️⃣ 레포지토리 클론

```bash
git clone https://github.com/eric-mjk/franka_ws.git
cd franka_ws
```

> ⚠️ `build/`, `install/`, `log/` 디렉토리는 Git에 포함되지 않습니다.

---

### 3️⃣ (선택) 외부 의존 패키지 다운로드

`franka_ros2`, MoveIt2 등 외부 패키지를 `vcs`로 관리하는 경우:

```bash
vcs import src < franka.repos
```

---

### 4️⃣ Docker 컨테이너 빌드 & 실행

```bash
docker compose -f docker/docker-compose.yml up -d --build
```

* ROS2 Humble 개발 환경 이미지가 빌드됩니다.
* 컨테이너는 백그라운드에서 실행됩니다.

컨테이너 상태 확인:

```bash
docker compose -f docker/docker-compose.yml ps
```

---

### 5️⃣ 컨테이너 접속

```bash
docker compose -f docker/docker-compose.yml exec franka bash
```

접속 후:

* ROS2 환경은 자동으로 source 되어 있습니다.
* 작업 디렉토리는 `/workspaces/franka_ws` 입니다.

---

### 6️⃣ 빌드 (컨테이너 내부)

```bash
colcon build
source install/setup.bash
```

> ⚠️ 반드시 **컨테이너 내부에서** `colcon build`를 실행하세요.

---

### 7️⃣ 실행 예시

```bash
ros2 launch <package_name> <launch_file>.py
```

GUI(RViz, MoveIt)를 사용하는 경우:

* 호스트 X11 환경을 그대로 사용합니다.
* 별도의 설정 없이 화면이 표시됩니다.

---

### 8️⃣ 컨테이너 종료

```bash
docker compose -f docker/docker-compose.yml down
```

* 컨테이너만 종료됩니다.
* 소스 코드 및 빌드 결과는 삭제되지 않습니다.

---

### 9️⃣ 디렉토리 구조 요약

```text
franka_ws/
├── src/                 # ROS2 패키지 (Git으로 관리)
├── docker/
│   ├── Dockerfile       # ROS2 Humble 개발 환경 정의
│   ├── docker-compose.yml
│   └── entrypoint.sh
├── build/               # colcon build 결과 (Git 제외)
├── install/             # colcon install 결과 (Git 제외)
├── log/                 # colcon 로그 (Git 제외)
└── franka.repos         # 외부 의존성 정의 (선택)
```

## Dev
⚠️ 실행 시 반드시 bash로 실행하세요. (`sh` 사용 금지)

`chmod +x scripts/*.sh`

`bash scripts/run_fake.sh` 또는 `./scripts/run_fake.sh`

# 프로젝트 설명

## 0. 프로젝트 개요

이 레포지토리는 **Franka Research 3 (FR3)** 로봇을 대상으로
**ROS 2 Humble + MoveIt2 + ros2_control** 기반의 표준 제어 파이프라인을 구축하고,
**Fake hardware → Real robot** 전환을 **동일한 구조**에서 수행하기 위한 연구/개발 프로젝트이다.

본 프로젝트의 핵심 목표는 다음과 같다:

* MoveIt2에서 **Plan → Execute** 파이프라인을 안정적으로 완주
* Fake hardware 환경에서 먼저 검증 후, 실로봇으로 동일하게 전환
* Docker + Git 기반으로 **개발 환경과 현장(로봇 연결 PC) 환경을 완전히 동일하게 재현**
* 연구/실험 코드를 MoveIt 내부 구현과 분리하여 **확장성과 재현성 확보**

> ⚠️ 본 레포는 **Isaac Sim은 포함하지 않는다**.
> (Isaac Sim은 추후 별도 통합 단계로 분리)

---

## 1. 시스템 전체 구조 (개념)

```text
[ Experiments / Research Code ]
            |
            v
        [ motion ]
   (MoveIt wrapper layer)
            |
            v
        [ MoveIt2 ]
   - Planning
   - Collision checking
            |
            v
[ MoveIt Controller Manager ]
            |
            v
       [ ros2_control ]
   - JointTrajectoryController
   - State Broadcasters
            |
            v
[ Franka Hardware Interface ]
   - Fake hardware (development)
   - Real FR3 (deployment)
```

### 핵심 원칙

* **MoveIt2는 계획(Planning)만 담당**한다.
* 실제 제어는 **항상 ros2_control을 통해서만** 수행된다.
* Fake / Real 전환은 **하드웨어 교체**이지 구조 변경이 아니다.

---

## 2. ros2_control과 franka_ros2의 역할

### ros2_control

* ROS 2의 **표준 로봇 제어 프레임워크**
* Hardware Interface + Controller 구조 제공
* MoveIt, teleop, custom control 모두 동일한 인터페이스로 연결 가능

### franka_ros2

* Franka FR3/Panda를 ros2_control에 연결하기 위한 **공식 구현 패키지**
* 포함 내용:

  * Franka Hardware Interface
  * URDF / robot description
  * bringup launch
  * MoveIt config (FR3 기준)

> ros2_control = 제어 프레임워크
> franka_ros2 = FR3 전용 어댑터 + 설정 세트

---

## 3. 워크스페이스 / 레포 구조

```text
franka_ws/
├── src/
│   ├── bringup/              # 로봇 bringup (launch, controller config)
│   ├── motion/               # MoveIt wrapper / 연구용 제어 API
│   ├── experiments/          # 실험/시나리오 코드 (motion만 호출)
│   └── (external via vcs)    # franka_ros2, MoveIt2 등
│
├── docker/                   # Dockerfile, entrypoint
├── .devcontainer/            # VSCode devcontainer
├── scripts/                  # build/run helper scripts
├── franka.repos              # vcs import manifest
├── .github/workflows/        # CI (optional)
└── README.md
```

### Git 관리 원칙

* `src/` 이하의 **직접 개발 코드만 Git으로 관리**
* `build/ install/ log/`는 `.gitignore`
* 외부 의존성(`franka_ros2` 등)은 **vcs import로 고정**

---

## 4. bringup 패키지의 역할

**bringup = “로봇 시동 패키지”**

### 책임 범위

* 로봇 실행에 필요한 노드들을 **한 번에 올리는 launch**
* Fake / Real hardware 전환
* ros2_control 설정 및 controller 로딩

### 포함 내용

* `launch/`

  * bringup launch
  * fake/real 선택 인자 (`use_fake_hardware`, `robot_ip`)
* `config/`

  * `controllers.yaml`
  * hardware 관련 파라미터

> ⚠️ bringup에는 **연구 로직을 넣지 않는다**
> (오직 실행/설정만 담당)

---

## 5. motion 패키지의 역할 (중요)

**motion = MoveIt을 감싸는 “공식 제어 API 레이어”**

### 왜 필요한가?

* MoveIt API를 실험 코드에서 직접 호출하면:

  * 구조가 깨지기 쉽고
  * Fake/Real 전환이 어려워지고
  * 재현성이 급격히 나빠진다.

### motion의 목표

* MoveIt 사용을 **하나의 창구로 통일**
* 실험 코드는 motion의 서비스/액션만 호출

### 예시 기능

* `go_home`
* `plan_to_pose`
* `execute_last_plan`
* `set_speed_scale`

> motion은 MoveIt을 대체하지 않는다.
> **MoveIt을 안전하고 일관되게 사용하기 위한 래퍼**다.

---

## 6. Phase 기반 개발 로드맵

### Phase 1 — 환경/워크스페이스 구축

* franka_ros2 + MoveIt2 + ros2_control 빌드
* 기본 launch 기동 확인

### Phase 2 — Fake hardware로 Plan → Execute 완주 (MVP)

* MoveIt RViz에서 계획/실행 성공
* ros2_control controller active 확인
* trajectory가 controller로 전달되는 것 확인

### Phase 3 — Real robot bringup

* 구조 변경 없이 hardware만 real로 교체
* 짧고 느린 trajectory 실행

### Phase 4 — motion 계층 도입 (연구 구조 고정)

* 실험 코드는 motion만 호출
* MoveIt/ros2_control 의존성을 하위로 캡슐화
* 재현성과 확장성 확보

### Phase 5 — (선택) 모델/툴 커스터마이징

* URDF / SRDF / collision / controller mapping 일괄 수정

---

## 7. Docker + Git 기반 개발/배포 전략

### 목표

* 로컬 개발 환경과 현장(로봇 연결 PC) 환경을 **완전히 동일하게 재현**
* “로컬에서는 되는데 현장에서는 안 됨” 방지

### 전략

* Docker로 ROS2 + 의존성 고정
* Devcontainer로 로컬 개발도 컨테이너 안에서 수행
* GitHub Actions로 빌드 검증(선택)

### 실행 모드

* **개발 모드**: 소스 볼륨 마운트, 빠른 반복
* **현장 모드**: 커밋/이미지 태그 고정, 재현성 우선

---

## 8. Fake ↔ Real 전환 원칙

* 코드 변경 ❌
* controller 이름 변경 ❌
* MoveIt config 변경 ❌

오직 아래만 변경:

* launch argument

  * `use_fake_hardware:=true/false`
  * `robot_ip:=<ip>`

---

## 9. 디버깅 기본 순서 (중요)

문제 발생 시 **반드시 아래 순서로 확인**:

1. ros2_control hardware interface
2. controller 상태 (`ros2 control list_controllers`)
3. MoveIt controller 설정 매칭
4. URDF / TF

---

## 10. 최소 성공 기준 (MVP)

* Fake hardware에서:

  * RViz Plan → Execute 성공
* 동일 launch 구조로:

  * Real FR3에서 짧은 trajectory 실행
* 실험 코드는 motion API만 사용

---

## 11. 향후 확장 방향 (요약)

* Isaac Sim 연동 (별도 단계)

---

## 12. 참고 자료

* Franka Documentation
  [https://frankarobotics.github.io/docs/overview.html](https://frankarobotics.github.io/docs/overview.html)
* franka_ros2
  [https://github.com/frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2)
* MoveIt2
  [https://moveit.picknik.ai](https://moveit.picknik.ai)


