TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += xeryon

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install xeryon.dbd into <top>/dbd
DBD += xeryonSupport.dbd

xeryonSupport_DBD += xeryon.dbd

# specify all source files to be compiled and added to the library
xeryon_SRCS += xeryon_driver.cpp

xeryon_LIBS += motor asyn
xeryon_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
