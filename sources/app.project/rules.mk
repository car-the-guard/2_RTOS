# 1. 현재 프로젝트의 기본 경로 설정
MCU_BSP_APP_PROJECT_PATH := $(MCU_BSP_SOURCES_PATH)/app.project

# --------------------------------------------------------------------------
# [설정] 헤더 파일 경로 (INCLUDES)
# --------------------------------------------------------------------------

# 기본 경로 포함
INCLUDES += -I$(MCU_BSP_APP_PROJECT_PATH)/tcc70xx
INCLUDES += -I$(MCU_BSP_APP_PROJECT_PATH)/utils

# [추가됨] Sonar 센서 경로 명시적 추가
# sonar.h 파일을 찾기 위해 필요합니다.
INCLUDES += -I$(MCU_BSP_APP_PROJECT_PATH)/sensors/sonar

# (옵션) 만약 '자동 찾기' 로직을 유지하고 싶다면 아래 코드를 그대로 두세요.
# 하지만 명시적 지정이 더 안전하므로, 특정 폴더만 필요하다면 자동 찾기는 주석 처리해도 됩니다.
APP_PROJECT_ALL_DIRS := $(shell find $(MCU_BSP_APP_PROJECT_PATH) -type d)
INCLUDES += $(addprefix -I,$(APP_PROJECT_ALL_DIRS))
VPATH += $(APP_PROJECT_ALL_DIRS)

# --------------------------------------------------------------------------
# [설정] 소스 파일 경로 (VPATH)
# --------------------------------------------------------------------------

# make가 sonar.c 파일을 찾을 수 있도록 검색 경로(VPATH)에 추가
VPATH += $(MCU_BSP_APP_PROJECT_PATH)/sensors/sonar
VPATH += $(MCU_BSP_APP_PROJECT_PATH)/sensors/accel

# --------------------------------------------------------------------------
# 소스 파일 및 기타 설정
# --------------------------------------------------------------------------

# 방법 1: 모든 .c 파일 자동 검색 (기존 방식 유지)
ALL_C_SRCS := $(shell find $(MCU_BSP_APP_PROJECT_PATH) -name '*.c')
SRCS += $(notdir $(ALL_C_SRCS))

# 방법 2: (추천) 만약 위 자동 검색이 sonar.c를 못 찾으면 아래처럼 직접 추가
# SRCS += sonar.c

# 전처리기 플래그
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_PROJECT=1
COMMON_FLAGS += -DMCU_BSP_SUPPORT_APP_BASE=1

# 외부 드라이버(GPIO 등) 헤더 경로 추가
INCLUDES += -I$(MCU_BSP_SOURCES_DEV_DRIVERS_PATH)/gpio