/*
 * drvNSLS2I_EM.h
 * 
 * Asyn driver that inherits from the drvQuadEM class to control the NSLS2 electrometer / XBPM
 *
 * Author: Mark Rivers, Pete Siddons
 *
 * Created January 18th, 2016
 */

#include "drvQuadEM.h"

#define MAX_FIRMWARE_LEN 64
#define MAX_CUR_RANGES 6
#define MAX_VOLT_RANGES 4
#define P_DACString               "QE_DAC"                 /* asynInt32,    r/w */
#define P_CalibrationModeString   "QE_CALIBRATION_MODE"    /* asynInt32,    r/w */
#define P_ADCOffsetString         "QE_ADC_OFFSET"          /* asynInt32,    r/w */
#define P_VoltageInString         "QE_VOLTAGE_IN"          /* asynInt32,    r/w */
#define P_DigitalInString         "QE_DIGITAL_IN"          /* asynInt32,r/w */  
#define P_DigitalOutString        "QE_DIGITAL_OUT"         /* asynInt32,r/w */
#define P_CurRangeString          "QE_CUR_RANGE"   /* asynInt32,    r/w */
#define P_VoltRangeString         "QE_VOLT_RANGE"  /* asynInt32,    r/w */

/** Class to control the NSLS Precision Integrator */
class drvNSLS2_MIO : public drvQuadEM {
public:
    drvNSLS2_MIO(const char *portName, int ringBufferSize);
    ~drvNSLS2_MIO();
    
    /* These are the methods we implement from asynPortDriver */
    void report(FILE *fp, int details);
                 
    /* These are the methods that are new to this class */
    virtual void exitHandler();
    /* This should be private but we call it from C so it needs to be public */
    void callbackFunc();
    bool isAcquiring();
//    unsigned int fpgabase_;
//    int Peekfunc(int reg);
//    int Pokefunc(int reg, int value);

protected:
    /* These are the methods we implement from quadEM */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus getBounds(asynUser *pasynUser, epicsInt32 *low, epicsInt32 *high);
    virtual asynStatus setAcquire(epicsInt32 value);
//    virtual asynStatus setRange(epicsInt32 channel, epicsInt32 value);
    virtual asynStatus setValuesPerRead(epicsInt32 value);
    virtual asynStatus setAveragingTime(epicsFloat64 value);
    virtual asynStatus initDAC();  
    virtual asynStatus setBiasVoltage(epicsFloat64 value);
    virtual asynStatus readStatus();
    virtual asynStatus reset();
    
    int P_DAC;
    #define FIRST_NSLS2_COMMAND P_DAC
    int P_CalibrationMode;
    int P_ADCOffset;
    int P_VoltageIn;
    int P_DigitalIn;
    int P_DigitalOut;
    int P_Cur_Range;
    int P_Volt_Range;
 
private:
    /* Our data */
    double iranges_[MAX_CUR_RANGES];
    double vranges_[MAX_VOLT_RANGES];
    epicsFloat64 rawCurData_[QE_MAX_INPUTS];
    epicsFloat64 rawVoltData_[QE_MAX_INPUTS];
    int readingsAveraged_;
    int readingActive_;
    bool calibrationMode_;
    int ADCOffset_[QE_MAX_INPUTS];
    int Cur_Range_[QE_MAX_INPUTS];
    int Volt_Range_[QE_MAX_INPUTS];
    char firmwareVersion_[MAX_FIRMWARE_LEN];
    volatile unsigned int *fpgabase_;  //mmap'd fpga registers
    epicsFloat64 iscaleFactor_[QE_MAX_INPUTS][MAX_CUR_RANGES];
    epicsFloat64 vscaleFactor_[QE_MAX_INPUTS][MAX_VOLT_RANGES];
    int memfd_;
    int intfd_;
    int channel;
    int value;

    /* our functions */
    asynStatus getFirmwareVersion();
    asynStatus readMeter(int *iadcbuf, int *vadcbuf, int *bbuf);
    asynStatus readBit(int channel, int *value);
    asynStatus writeBit(int channel, int value);
    asynStatus setDAC(int channel, int value);
    asynStatus setCurRange(int channel,int value);
    asynStatus setVoltRange(int channel, int value);
//    void mmap_fpga();
    asynStatus pl_open(int *fd);
    asynStatus setAcquireParams();
};

