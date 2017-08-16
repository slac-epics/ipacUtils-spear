TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += avme9660
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard ip*))

include $(TOP)/configure/RULES_TOP
