# GTSP Trajectory
## 주요 기능

- 검사 대상 표면 자동 추출 및 뷰포인트 생성
- cuRobo 기반 충돌 없는 로봇 궤적 계획
- Isaac Sim 시뮬레이션 환경
- ROS2 연동

## 시스템 요구사항

- **OS**: Ubuntu 22.04
- **GPU**: NVIDIA GPU (CUDA 지원 필수)
- **Software**:
  - Docker / NVIDIA Container Toolkit
  - Isaac Sim 5.0 / cuRobo

## 설치 방법

### 1. 사전 준비

Docker, NVIDIA Container Toolkit을 설치합니다:

```bash
# 공식 가이드 참고
https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html
```

### 2. Docker 이미지 빌드

자동 설치 스크립트를 실행합니다:

```bash
./install.sh
```

**이 스크립트가 하는 일:**
- Isaac Sim + cuRobo Docker 이미지 빌드 (isaac_curobo:image)
- 최초 1회 실행

**소요 시간:** 첫 빌드 시 30분~1시간 이상 소요될 수 있습니다.

## Docker 컨테이너 실행

### 기본 실행

```bash
./execution.sh
```

### 컨테이너 내부에서 할 수 있는 것

컨테이너가 실행되면 다음과 같은 작업을 수행할 수 있습니다:

```bash
# 프로젝트 디렉토리로 이동
cd /curobo/gtsp_trajectory

# 스크립트 실행 (자세한 내용은 scripts/README.md 참고)
/isaac-sim/python.sh scripts/1_create_viewpoint.py
```

### 코드 수정 반영

프로젝트 폴더가 volume mount로 연결되어 있어, 호스트에서 코드를 수정하면 컨테이너에 즉시 반영됩니다.

**이미지 재빌드가 필요한 경우:**
- Dockerfile 자체 변경 (패키지 추가, 환경 설정 등)
- `ur20_description/` 로봇 설정 변경

**재빌드 없이 즉시 반영:**
- `scripts/`, `common/`, `data/` 코드 변경

## 사용 방법

비전 검사 파이프라인 사용 방법은 [scripts/README.md](scripts/README.md)를 참고하세요.