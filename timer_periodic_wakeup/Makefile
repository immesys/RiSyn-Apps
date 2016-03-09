APPLICATION = timer_periodic_wakeup
RIOTBASE ?= $(CURDIR)/../..
BOARD ?= hamilton
USEMODULE += xtimer
USEMODULE += rtt_stdio
CFLAGS += -DRTT_STDIO_ENABLE_BLOCKING_STDOUT
USEMODULE += at30ts74
QUIET ?= 1

FEATURES_REQUIRED += periph_timer

include $(RIOTBASE)/Makefile.include
