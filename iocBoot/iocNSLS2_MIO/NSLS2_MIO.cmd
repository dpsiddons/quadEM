epicsEnvSet("PREFIX",    "quadEMTest:")
epicsEnvSet("RECORD",    "NSLS2_MIO:")
epicsEnvSet("PORT",      "NSLS2_MIO")
epicsEnvSet("TEMPLATE",  "NSLS2_MIO")
epicsEnvSet("QSIZE",     "20")
epicsEnvSet("RING_SIZE", "10000")
epicsEnvSet("TSPOINTS",  "1000")
epicsEnvSet("MODULE_ID", "0")

drvNSLS2_MIOConfigure("$(PORT)", $(RING_SIZE))

asynSetTraceIOMask("$(PORT)", 0, 2)
# Uncomment this line to enable asynPrint statements with ASYN_TRACEIO_DRIVER
#asynSetTraceMask("$(PORT)", 0, 9)

dbLoadRecords("$(QUADEM)/db/$(TEMPLATE).template", "P=$(PREFIX), R=$(RECORD), PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE)_Ranges.template","P=$(PREFIX), R=$(RECORD),PORT=$(PORT),N=0")
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE)_Ranges.template","P=$(PREFIX), R=$(RECORD),PORT=$(PORT),N=1")
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE)_Ranges.template","P=$(PREFIX), R=$(RECORD),PORT=$(PORT),N=2")
dbLoadRecords("$(QUADEM)/db/$(TEMPLATE)_Ranges.template","P=$(PREFIX), R=$(RECORD),PORT=$(PORT),N=3")
dbLoadTemplate("DACs.substitutions")

# Comment out this line to suppress loading the plugins
< $(QUADEM)/iocBoot/commonPlugins.cmd

< $(QUADEM)/iocBoot/saveRestore.cmd

iocInit()

# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(PREFIX), R=$(RECORD)")
dbl > pvlist.txt

