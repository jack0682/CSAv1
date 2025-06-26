#!/bin/bash

echo "========== [CSA 시스템 정밀 환경 진단 도구] =========="
echo ""

# 1. 시스템 정보
echo "🔹 [OS 및 커널 정보]"
lsb_release -d; uname -r; arch
echo ""

# 2. Python 가상환경/Conda
echo "🔹 [Python 환경]"
which python3
python3 --version
echo "VIRTUAL_ENV: $VIRTUAL_ENV"
echo "conda info:"; conda info 2>/dev/null || echo "(conda 미사용)"
echo ""

# 3. 필수 Python 패키지 점검
echo "🔹 [필수 Python 패키지]"
declare -A pkg_map=(
  [rclpy]="rclpy"
  [numpy]="numpy"
  [torch]="torch"
  [cv2]="cv2"
  [pyyaml]="yaml"
  [PyYAML]="yaml"
  [lark_parser]="lark"
  [empy]="em"
  [catkin_pkg]="catkin_pkg"
  [matplotlib]="matplotlib"
  [scipy]="scipy"
  [pandas]="pandas"
)

for name in "${!pkg_map[@]}"; do
    module=${pkg_map[$name]}
    python3 -c "import $module" 2>/dev/null && \
    echo "  ✔ $name (ver: $(python3 -c "import $module; print(getattr($module, '__version__', 'n/a'))" 2>/dev/null))" || \
    echo "  ✖ $name: 미설치"
done
echo ""


# 4. pip/setuptools/colcon/rosdep
echo "🔹 [빌드/패키징 툴]"
pip --version
pip show setuptools wheel | grep Version
colcon --version 2>/dev/null || echo "colcon 미설치"
rosdep --version 2>/dev/null || echo "rosdep 미설치"
echo ""

# 5. ROS2 설정
echo "🔹 [ROS2 환경]"
echo "ROS2 경로: $(which ros2)"
ros2 --version
env | grep ROS_VERSION
env | grep ROS_DISTRO
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
ros2 pkg list | wc -l
ros2 node list || echo "node 실행 없음"
echo ""

# 6. GPU 및 CUDA
echo "🔹 [GPU 및 CUDA]"
nvidia-smi 2>/dev/null || echo "nvidia-smi 없음"
nvcc --version 2>/dev/null || echo "nvcc 없음"
echo ""

# 7. 수동 설치 OpenCV/Pangolin/Eigen 경로
echo "🔹 [수동 설치 라이브러리 경로 체크]"
for dir in /home/jack/opencv-3.4.17 /home/jack/eigen-3.3.7 /home/jack/Pangolin; do
  [ -d "$dir" ] && echo "  ✔ $dir: 존재" || echo "  ✖ $dir: 없음"
done
echo ""

# 8. ORB-SLAM2 설치 상태
echo "🔹 [ORB-SLAM2 설치 체크]"
[ -d /home/jack/ORB_SLAM2 ] && {
  echo "  ✔ ORB-SLAM2 디렉토리 존재"
  for sub in build Vocabulary Examples; do
    [ -d "/home/jack/ORB_SLAM2/$sub" ] && echo "    └ $sub: 있음" || echo "    └ $sub: 없음"
  done
} || echo "  ✖ ORB-SLAM2 디렉토리 없음"
echo ""

# 9. CSA 관련 워크스페이스 경로 점검
echo "🔹 [CSA 디렉토리 체크]"
for d in ~/ros2_ws/models ~/ros2_ws/dataset ~/ros2_ws/output ~/ros2_ws/log; do
  [ -d "$d" ] && echo "  ✔ $d (권한: $(stat -c "%a" $d))" || echo "  ✖ $d: 없음"
done
echo ""

# 10. 패키지 및 설정 파일 상태
echo "🔹 [ROS2 패키지 점검]"
cd ~/ros2_ws || exit 1
echo "src 디렉토리:"
ls -l src | grep ^d
echo ""

echo "setup.py / package.xml 파일 리스트:"
find src -name setup.py
find src -name package.xml
echo ""

# 11. config 파일 미리보기
echo "🔹 [설정 파일 내용 미리보기]"
find src -name '*.yaml' -o -name '*.json' | while read f; do
  echo "  ▸ $f"; head -n 5 "$f"; echo ""
done

# 12. git 상태
echo "🔹 [Git 상태]"
git status
echo ""

echo "🔍 ORB-SLAM2 환경 점검 스크립트"
echo "──────────────────────────────"

# Eigen3
echo -n "📦 Eigen3 설치 확인: "
if [ -d "/usr/include/eigen3/Eigen" ]; then
    eigen_version=$(grep "VERSION" /usr/include/eigen3/Eigen/src/Core/util/Macros.h | grep "#define EIGEN_WORLD_VERSION" | awk '{print $3}')
    echo "✔️ 설치됨 (대략 버전: $eigen_version.x)"
else
    echo "❌ /usr/include/eigen3/Eigen 경로 없음 (설치 필요)"
fi

# Pangolin
echo -n "📦 Pangolin 설치 확인: "
if [ -d "/usr/include/pangolin" ] || pkg-config --exists pangolin; then
    pangolin_ver=$(pkg-config --modversion pangolin 2>/dev/null)
    echo "✔️ 설치됨 (버전: ${pangolin_ver:-unknown})"
else
    echo "❌ Pangolin 설치 안됨 또는 pkg-config 등록 안됨"
fi

# OpenCV
echo -n "📦 OpenCV (C++) 설치 확인: "
opencv_ver=$(pkg-config --modversion opencv4 2>/dev/null || pkg-config --modversion opencv 2>/dev/null)
if [ -n "$opencv_ver" ]; then
    echo "✔️ 설치됨 (버전: $opencv_ver)"
else
    echo "❌ OpenCV 개발 패키지 미설치"
fi

echo "──────────────────────────────"

echo "========== [CSA 환경 진단 완료 — 이상 없을 시 실행 시작 가능] =========="
