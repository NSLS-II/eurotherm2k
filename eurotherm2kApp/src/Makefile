TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += eurothermModbus

DBD += eurothermModbusSupport.dbd

DBD += eurotherm2k_vdct.dbd
eurotherm2k_vdct_DBD += base.dbd
eurotherm2k_vdct_DBD += calc.dbd
eurotherm2k_vdct_DBD += stream.dbd

# The following are compiled and added to the support library
eurothermModbus_SRCS += eurothermModbus.c

# We need to link against the EPICS Base libraries
eurothermModbus_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
