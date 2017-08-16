# Makefile.Host
TOP = ..
include $(TOP)/configure/CONFIG

DBD += drvXy9660.dbd

INC += xipIo.h

# Source files (for depends target):
LIBSRCS += drvXy9660.c

# Link everything into a library:
LIBRARY_IOC_vxWorks = Xy9660
LIBRARY_IOC_RTEMS   = Xy9660
#Xy9660_LIBS += Ipac

include $(TOP)/configure/RULES
