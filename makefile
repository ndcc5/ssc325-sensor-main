PROJ_ROOT ?=../../../project
include $(PROJ_ROOT)/configs/current.configs


SENSOR_HEADER = drv_ms_cus_

SENSOR_SRC_DIR=drv/src
SENSORSRC = $(notdir  $(shell find ./drv -name *.c ))
SENSOR_SRC = $(filter-out %.mod.c,$(SENSORSRC))
SNESORS:=$(patsubst $(SENSOR_HEADER)%,%,$(patsubst %.c,%, $(SENSOR_SRC)))
SNESORS_CLEAN:=$(patsubst %, %_clean, $(SNESORS))
SNESORS_INSTALL := $(patsubst %, %_install, $(SNESORS))

TARGET_MODULES := $(patsubst %, %_module, $(SNESORS))
TARGET_LIBS:= $(patsubst %, %_lib, $(SNESORS))

MOD_PREFIX:=

include $(PROJ_ROOT)/release/$(PRODUCT)/$(CHIP)/$(BOARD)/$(TOOLCHAIN)/toolchain.mk

export PROJ_ROOT CHIP PRODUCT BOARD TOOLCHAIN TOOLCHAIN_VERSION KERNEL_VERSION CUSTOMER_OPTIONS MOD_PREFIX INTERFACE_ENABLED INTERFACE_DISABLED MHAL_ENABLED MHAL_DISABLED


.PHONY: $(SNESORS) $(PROJ_ROOT)/kbuild/$(KERNEL_VERSION)/.config

all:mkfile $(SNESORS)
	@echo done
clean:mkfile $(SNESORS_CLEAN) mkfile_clean
	@echo done
install:$(SNESORS_INSTALL)
	@echo done

mkfile:
	@ln -sf $(CURDIR)/sensor.mk $(SENSOR_SRC_DIR)/Makefile
mkfile_clean:
	@rm -f $(SENSOR_SRC_DIR)/Makefile

$(SNESORS):%:$(PROJ_ROOT)/kbuild/$(KERNEL_VERSION)/.config
	@$(MAKE) $@_module 
	@$(MAKE) $@_lib

$(SNESORS_INSTALL):%:$(PROJ_ROOT)/kbuild/$(KERNEL_VERSION)/.config
	$(MAKE) -C $(SENSOR_SRC_DIR) module_install SENSOR_NAME=$(patsubst %_install,%,$@) SENSOR_FILE=$(patsubst %_install,%,$(patsubst %,$(SENSOR_HEADER)%,$@))	
	@if [[ "`ls -A $(CURDIR)/drv/pub`" != "" ]]; then \
		cp -rf $(CURDIR)/drv/pub/drv_ms_cus_sensor.h $(PROJ_ROOT)/release/$(PRODUCT)/include/drivers/sensorif; \
	fi;

$(TARGET_MODULES): %_module: 
	$(MAKE) -C $(SENSOR_SRC_DIR) module SENSOR_NAME=$(patsubst %_module,%,$@) SENSOR_FILE=$(patsubst %_module,$(SENSOR_HEADER)%,$@)
	@mv $(patsubst %,$(SENSOR_SRC_DIR)/%.ko,$(patsubst %_module,$(SENSOR_HEADER)%,$@))  $(patsubst %,$(SENSOR_SRC_DIR)/%.ko,$(patsubst %_module,%,$@)) 
$(TARGET_LIBS): %_lib: 
	$(MAKE) -C $(SENSOR_SRC_DIR) lib SENSOR_NAME=$(patsubst %_lib,%,$@) SENSOR_FILE=$(patsubst %,$(SENSOR_HEADER)%,$@)
#@mv $(patsubst %,$(SENSOR_SRC_DIR)/%.a,$(patsubst %_module,$(SENSOR_HEADER)%,$@))  $(patsubst %,$(SENSOR_SRC_DIR)/%.a,$(patsubst %_module,%,$@)) 

$(SNESORS_CLEAN):%:$(PROJ_ROOT)/kbuild/$(KERNEL_VERSION)/.config
	$(MAKE) -C $(SENSOR_SRC_DIR) module_clean SENSOR_NAME=$(patsubst %_clean,%,$@) SENSOR_FILE=$(patsubst %_clean,%,$(patsubst %,$(SENSOR_HEADER)%,$@))  

$(PROJ_ROOT)/kbuild/$(KERNEL_VERSION)/.config:
	$(MAKE) -C $(PROJ_ROOT) kbuild/$(KERNEL_VERSION)/.config
