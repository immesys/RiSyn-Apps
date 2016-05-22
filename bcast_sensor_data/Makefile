APPLICATION = bcast_sensor_data
RIOTBASE ?= $(CURDIR)/../..
BOARD ?= hamilton
USEMODULE += xtimer
USEMODULE += rtt_stdio
#CFLAGS += -DRTT_STDIO_ENABLE_BLOCKING_STDOUT

# Sensors
USEMODULE += at30ts74 # temperature sensor
USEMODULE += mma7660 # accelerometer

# networking
USEMODULE += gnrc_netif_default
USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_udp
# This application dumps received packets to STDIO using the pktdump module
USEMODULE += gnrc_pktdump
# Additional networking modules that can be dropped if not needed
USEMODULE += gnrc_icmpv6_echo
# Add also the shell, some shell commands
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

QUIET ?= 1

FEATURES_REQUIRED += periph_timer

include $(RIOTBASE)/Makefile.include
