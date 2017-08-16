# Makefile.Host
TOP = ..
include $(TOP)/configure/CONFIG

DBD += drvAvme445.dbd
DBD += devAvme445.dbd

# Source files (for depends target):
LIBSRCS += drvXy2445.c
LIBSRCS += devXy2445.c

# Link everything into a library:
LIBRARY_IOC_vxWorks = Xy2445
LIBRARY_IOC_RTEMS   = Xy2445
Xy2445_LIBS += Ipac

include $(TOP)/configure/RULES
