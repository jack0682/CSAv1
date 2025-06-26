#!/bin/bash

VENV_DIR=~/ros2_venv

# 1. 가상환경 폴더 존재 확인
if [ ! -d "$VENV_DIR" ]; then
    echo "[❌ ERROR] $VENV_DIR 폴더가 존재하지 않습니다."
    echo "→ python3 -m venv ~/ros2_venv 로 먼저 가상환경을 생성하세요."
    exit 1
fi

# 2. 가상환경 진입 (activate)
source $VENV_DIR/bin/activate

if [ "$VIRTUAL_ENV" != "" ]; then
    echo "[✅ CSA] Python venv 환경 활성화됨: $VIRTUAL_ENV"
else
    echo "[❌ ERROR] 가상환경 활성화 실패"
    exit 2
fi

# 3. pip, setuptools, wheel 최신화
echo "[🔁] pip, setuptools, wheel 업그레이드 중..."
pip install --upgrade pip setuptools wheel

# 4. 연구 필수 패키지 설치
echo "[📦] CSA 필수 패키지 설치 중..."
pip install --upgrade \
    numpy \
    torch \
    opencv-python \
    matplotlib \
    scipy \
    pandas \
    catkin_pkg \
    rclpy \
    pyyaml \
    empy==3.3.4 \
    lark-parser

# 5. 설치 확인 및 버전 출력
echo ""
echo "🐍 Python  버전: $(python --version)"
echo "📦 pip     버전: $(pip --version)"
echo "🔍 주요 패키지 설치 상태:"
echo ""

for pkg in numpy torch cv2 matplotlib scipy pandas catkin_pkg rclpy yaml lark em; do
    python -c "import $pkg; print('$pkg:', getattr($pkg, '__version__', 'OK'))" 2>/dev/null || echo "$pkg: ❌ 미설치 또는 import 실패"
done

echo ""
echo "[✅ CSA] 가상환경 준비 완료! 필요한 작업을 진행하세요."
echo "→ 종료 시에는 'deactivate' 입력으로 가상환경에서 나갈 수 있습니다."
