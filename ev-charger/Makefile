#
# Makefile
#
CC = arm-linux-gnueabihf-gcc
LVGL_DIR_NAME ?= lvgl
LVGL_DIR ?= ${shell pwd}
SQLITE_DIR ?= /home/linux/nfs/sqlite3/sqlite-3.7.3
SQLITE_INC ?= $(SQLITE_DIR)
SQLITE_LIB ?= $(SQLITE_DIR)/.libs
CFLAGS ?= -O3 -g0 -I$(LVGL_DIR)/ -Wall -Wshadow -Wundef -Wmissing-prototypes -std=c99 -Wall -Wextra -Wno-unused-function -Wno-error=strict-prototypes -Wpointer-arith -fno-strict-aliasing -Wno-error=cpp -Wuninitialized -Wmaybe-uninitialized -Wno-unused-parameter -Wno-missing-field-initializers -Wtype-limits -Wsizeof-pointer-memaccess -Wno-format-nonliteral -Wno-cast-qual -Wunreachable-code -Wno-switch-default -Wreturn-type -Wmultichar -Wformat-security -Wno-ignored-qualifiers -Wno-error=pedantic -Wno-sign-compare -Wno-error=missing-prototypes -Wdouble-promotion -Wclobbered -Wdeprecated -Wempty-body -Wtype-limits -Wstack-usage=2048 -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers -Wuninitialized -Wmaybe-uninitialized -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wtype-limits -Wsizeof-pointer-memaccess -Wno-format-nonliteral -Wpointer-arith -Wno-cast-qual -Wmissing-prototypes -Wunreachable-code -Wno-switch-default -Wreturn-type -Wmultichar -Wformat-security -Wno-ignored-qualifiers -Wno-sign-compare
CFLAGS += -I/home/linux/mqtt/paho.mqtt.c-master/src
LDFLAGS += -lpaho-mqtt3c -L/home/linux/mqtt/paho.mqtt.c-master/build/output
CFLAGS += -I$(SQLITE_INC)
#-Wno-discarded-qualifiers 
#-Wshift-negative-value 
#-Wno-discarded-qualifiers 
LDFLAGS ?= -lm -lpthread 
LDFLAGS += -L$(SQLITE_LIB) -lsqlite3
LDFLAGS += -lpaho-mqtt3c -L/home/linux/mqtt/paho.mqtt.c-master/build/output
LDFLAGS += -lpthread -lm



BIN = demo


#Collect the files to compile
MAINSRC = ./main.c

include $(LVGL_DIR)/lvgl/lvgl.mk
include $(LVGL_DIR)/lv_drivers/lv_drivers.mk
include $(LVGL_DIR)/components/components.mk 
include $(LVGL_DIR)/images/images.mk 
include $(LVGL_DIR)/fonts/fonts.mk 
include $(LVGL_DIR)/screens/screens.mk

# --- 智能网关应用模块 ---
CSRCS +=$(LVGL_DIR)/app/app_theme.c
CSRCS +=$(LVGL_DIR)/app/app_sensors.c
CSRCS +=$(LVGL_DIR)/app/app_ui.c
CSRCS +=$(LVGL_DIR)/app/app_pages.c
CSRCS +=$(LVGL_DIR)/app/app_db.c
CSRCS +=$(LVGL_DIR)/app/app_camera.c
CSRCS +=$(LVGL_DIR)/app/app_icons.c

CSRCS +=$(LVGL_DIR)/mouse_cursor_icon.c 
CSRCS +=$(LVGL_DIR)/ui.c
CSRCS +=$(LVGL_DIR)/ui_helpers.c
CSRCS +=$(LVGL_DIR)/ui_events.c

OBJEXT ?= .o

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

MAINOBJ = $(MAINSRC:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(MAINSRC)
OBJS = $(AOBJS) $(COBJS)

.DEFAULT_GOAL := all

## MAINOBJ -> OBJFILES

all: default

%.o: %.c
	@$(CC)  $(CFLAGS) -c $< -o $@
	@echo "CC $<"
    
default: $(AOBJS) $(COBJS) $(MAINOBJ)
	$(CC) -o $(BIN) $(MAINOBJ) $(AOBJS) $(COBJS) $(LDFLAGS)
	cp $(BIN) ~/nfs/rootfs
clean:
	rm -f $(BIN) $(AOBJS) $(COBJS) $(MAINOBJ)

# 头文件依赖: 配置改动后所有目标文件自动重编
# (放在文件末尾, 避免抢占 make 的默认目标)
$(OBJS) $(MAINOBJ): lv_conf.h lv_drv_conf.h

