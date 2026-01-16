# 1. 현재 프로젝트의 기본 경로 설정
MCU_BSP_APP_PROJECT_PATH := $(MCU_BSP_SOURCES_PATH)/app.project

# --------------------------------------------------------------------------
# [핵심] 하위 디렉토리 자동 포함 로직
# --------------------------------------------------------------------------

# (1) 'find' 명령어로 현재 프로젝트 내의 모든 하위 폴더(-type d)를 찾아서 변수에 저장
APP_PROJECT_ALL_DIRS := $(shell find $(MCU_BSP_APP_PROJECT_PATH) -type d)

# (2) 찾은 모든 폴더 경로 앞에 '-I'를 붙여서 컴파일러 헤더 경로(INCLUDES)에 추가
# 예: /path/to/dir -> -I/path/to/dir
INCLUDES += -I$(MCU_BSP_APP_PROJECT_PATH)/tcc70xx
INCLUDES += $(addprefix -I,$(APP_PROJECT_ALL_DIRS))

# (3) VPATH에도 추가 (소스 파일(.c)을 하위 폴더에 뒀을 때 make가 찾을 수 있게 함)
VPATH += $(APP_PROJECT_ALL_DIRS)

# --------------------------------------------------------------------------
# 소스 파일 및 기타 설정
# --------------------------------------------------------------------------

# 모든 .c 파일 찾아서 빌드 대상에 추가
ALL_C_SRCS := $(shell find $(MCU_BSP_APP_PROJECT_PATH) -name '*.c')
SRCS += $(notdir $(ALL_C_SRCS))

# 전처리기 플래그 (필요시 사용)
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_PROJECT=1
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_BASE=1

# (선택사항) 외부 드라이버(GPIO 등) 헤더 경로 추가
INCLUDES += -I$(MCU_BSP_SOURCES_DEV_DRIVERS_PATH)/gpio