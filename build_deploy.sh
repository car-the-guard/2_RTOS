#!/bin/bash

# 에러 발생 시 즉시 스크립트 중단 (안전장치)
set -e

# ==========================================
# 경로 설정 (필요시 수정)
# ==========================================
PROJECT_ROOT="/home/amoled/topst-rtos"
BUILD_DIR="${PROJECT_ROOT}/build/tcc70xx/gcc"
OUTPUT_ROM="tcc70xx_pflash_boot_2M_ECC.rom"
TOOLS_DIR="${PROJECT_ROOT}/tools/fwdn_vcp"
WIN_DEST="/mnt/c/"

# 1. 빌드 시작
echo "========================================"
echo ">> 1. Start Building..."
echo "========================================"

# 빌드 디렉토리로 이동
cd "${BUILD_DIR}"

# Make 실행 (-j 옵션으로 CPU 코어를 최대한 써서 빠르게 빌드, -s는 조용히)
# make -j$(nproc)
make

# 2. 결과물 복사 (ROM 파일 -> tools 폴더)
echo ""
echo "========================================"
echo ">> 2. Copying ROM to tools directory..."
echo "========================================"

if [ -f "output/${OUTPUT_ROM}" ]; then
    cp "output/${OUTPUT_ROM}" "${TOOLS_DIR}/"
    echo "Done: ROM copied to ${TOOLS_DIR}"
else
    echo "Error: Output file not found!"
    exit 1
fi

# 3. 윈도우로 복사 (tools 폴더 -> C드라이브)
echo ""
echo "========================================"
echo ">> 3. Deploying to Windows (/mnt/c/)..."
echo "========================================"

# 윈도우 경로로 복사 (-r: 폴더 통째로, -f: 강제 덮어쓰기)
cp -rf "${TOOLS_DIR}" "${WIN_DEST}"

echo ">> All Process Finished Successfully!"
