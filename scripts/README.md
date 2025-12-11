# 비전 검사 파이프라인

로봇 비전 검사를 위한 4단계 파이프라인

## 파이프라인 개요

```
1단계: 뷰포인트 생성 → 2단계: 궤적 계획 → 3단계: 시뮬레이션 → 4단계: ROS 퍼블리시
```

## 전체 실행 예시

```bash
# 1단계: 뷰포인트 생성
/isaac-sim/python.sh scripts/1_create_viewpoint.py \
    --object sample --material-rgb "170,163,158"

# 2단계: 궤적 생성
/isaac-sim/python.sh scripts/2_generate_trajectory.py \
    --object sample --num_viewpoints 163

# 3단계: 시뮬레이션
/isaac-sim/python.sh scripts/3_simulation.py \
    --object sample --num_viewpoints 163 --visualize_spheres

# 4단계: ROS 퍼블리시
python3 scripts/4_publish_trajectory.py \
    --object sample --num_viewpoints 163
```

## 스크립트 설명

### 1_create_viewpoint.py
멀티 머티리얼 메시에서 검사 대상 표면 추출 및 뷰포인트 생성

```bash
--object         # 객체 이름 (필수)
--material-rgb   # 대상 머티리얼 RGB "R,G,B" (필수)
--visualize      # Open3D 시각화 (옵션)
```

### 2_generate_trajectory.py
모든 뷰포인트를 최적 순서로 방문하는 충돌 없는 로봇 궤적 계산

```bash
--object          # 객체 이름 (필수)
--num_viewpoints  # 뷰포인트 개수 (필수)
--knn             # k-NN 이웃 수 (기본값: 5)
--lambda-rot      # 회전 비용 가중치 (기본값: 1.0)
--visualize       # 궤적 시각화 (옵션)
```

### 3_simulation.py
Isaac Sim에서 궤적 실행

```bash
--object            # 객체 이름 (필수)
--num_viewpoints    # 뷰포인트 개수 (필수)
--visualize_spheres # 로봇 충돌 구체 표시 (옵션)
--tilt              # tilt 궤적 사용 (옵션)
--headless          # 헤드리스 모드: native 또는 websocket (옵션)
```

### 4_publish_trajectory.py
ROS2 토픽으로 궤적 퍼블리시

```bash
--object          # 객체 이름 (필수)
--num_viewpoints  # 뷰포인트 개수 (필수)
--dt              # 궤적 포인트 간 시간 간격 (기본값: 0.01초)
```

## 디렉토리 구조

```
data/{object}/
├── mesh/
│   ├── source.obj        # 입력 메시
│   └── source.mtl        # 머티리얼 정의
├── viewpoint/{num}/
│   └── viewpoints.h5     # 1단계 출력
└── trajectory/{num}/
    └── trajectory.csv    # 2단계 출력
```

## 설정

`common/config.py`에서 파라미터 설정
