# ../../bin/${EPICS_HOST_ARCH}/xeryonExample st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocxeryonExampleLinux.dbd")
iocxeryonExampleLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "xeryonExample:")

< xeryon.iocsh

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
