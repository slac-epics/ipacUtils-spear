# Makefile.Host
TOP = ..
include $(TOP)/configure/CONFIG

DBD += drvAvme470.dbd
DBD += devAvme470.dbd

INC += drvAvme470.h

# Source files (for depends target):
LIBSRCS += drvAvme470.c
LIBSRCS += devAvme470.c

# Link everything into a library:
LIBRARY_IOC_vxWorks = Avme470
LIBRARY_IOC_RTEMS   = Avme470
Avme470_LIBS += Ipac

include $(TOP)/configure/RULES
