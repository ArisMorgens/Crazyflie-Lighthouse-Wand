# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ./crazyflie-firmware

#
# To include header files from the directory
#
EXTRA_CFLAGS += -I$(PWD)/src/common_files

# APP_TYPE=wand or APP_TYPE=receiver can be used by flash_all.py.
# Falls back to WAND=1 (default) / RECEIVER=0 for manual builds.
ifdef APP_TYPE
  ifeq ($(APP_TYPE),wand)
    WAND := 1
  else ifeq ($(APP_TYPE),receiver)
    RECEIVER := 1
  else ifeq ($(APP_TYPE),receiver_hlc)
    RECEIVER_HLC := 1
  endif
else
  WAND ?= 1
  RECEIVER ?= 0
  RECEIVER_HLC ?= 0
endif

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
ifeq ($(WAND),1)
	OOT_CONFIG := $(PWD)/config/app-config-wand
	EXTRA_CFLAGS += -DCONFIG_WAND_APP=1
endif

ifeq ($(RECEIVER),1)
	OOT_CONFIG := $(PWD)/config/app-config-receiver
	EXTRA_CFLAGS += -DCONFIG_RECEIVER_APP=1
endif

ifeq ($(RECEIVER_HLC),1)
	OOT_CONFIG := $(PWD)/config/app-config-receiver-hlc
	EXTRA_CFLAGS += -DCONFIG_RECEIVER_HLC_APP=1
endif


include $(CRAZYFLIE_BASE)/tools/make/oot.mk
