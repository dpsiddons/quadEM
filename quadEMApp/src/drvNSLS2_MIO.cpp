// Define SIMULATION_MODE to run on a system without the FPGA
//#define SIMULATION_MODE 1

// Define POLLING_MODE to poll the ADCs rather than using interrupts
//#define POLLING_MODE 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#ifndef _WIN32
  #include <unistd.h>
  #include <sys/mman.h>
#endif
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsEvent.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <drvAsynIPPort.h>
#include <iocsh.h>

#include <epicsExport.h>
#include "drvNSLS2_MIO.h"
#include "pl_lib.h"

#define INBITS 0
#define OUTBITS 1
#define LEDS 5
#define FPGAVER 7
#define SA_RATE 8
#define IRQ_ENABLE 9
#define ADC_CLK 11
#define RAW_CUR 36
#define RAW_CUR_A 36
#define RAW_CUR_B 37
#define RAW_CUR_C 38
#define RAW_CUR_D 39
#define FRAME_NO 16
#define SA_RATE_DIV 19
#define CURGAINREG 28
#define VOLTGAINREG 29
#define GTX_RST 31
#define HV_BIAS 32
#define RAW_VOLT 40
#define RAW_VOLT_A 40
#define RAW_VOLT_B 41
#define RAW_VOLT_C 42
#define RAW_VOLT_D 43
#define CURAVG 44
#define CURAVG_A 44 
#define CURAVG_B 45 
#define CURAVG_C 46 
#define CURAVG_D 47
#define VOLTAVG 48
#define VOLTAVG_A 48 
#define VOLTAVG_B 49 
#define VOLTAVG_C 50 
#define VOLTAVG_D 51
#define EVR_TS_S 52
#define EVR_TS_NS 53
#define EVR_TS_S_LAT 54
#define EVR_TS_NS_LAT 55
#define EVR_TRIGDELAY 56
#define EVR_USRTRIG 57
#define DACS 72 
#define DACS_LDAC 73
#define DAC_OPMODE 74
#define THERM_DAC 76
#define THERM_DATA 80
#define FA_DATA 128
#define FA_FIFOCNT 129
#define SOFT_TRIG 130
#define FA_TRIGLEN 131
#define FA_PERCCOMP 132
#define FA_DATA_RATE_SEL 133
#define FA_CONTROL 134

#define FREQ 500000.0

#define DEVNAME "/dev/vipic"

#define POLL_TIME 0.001 
#define NOISE 1000.


static const char *driverName = "drvNSLS2_MIO";

// Global variable containing pointer to your driver object
class drvNSLS2_MIO *pdrvNSLS2_MIO;


/*******************************************
* Read ADCs and input bits
*
*
*********************************************/
asynStatus drvNSLS2_MIO::readMeter(int *iadcbuf, int *vadcbuf, int *bbuf)
{

    int i, ival, vval, bval;
    static const char *functionName = "readMeter";

    for (i=0;i<=3;i++) {
        ival = pl_register_read(intfd_, CURAVG+i);
        vval = pl_register_read(intfd_, VOLTAVG+i);
        readBit(i, &bval);
        asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
            "%s::%s i=%d ival=%d vval=%d bval=%0x\n",
            driverName, functionName, i, ival, vval, bval);
        *iadcbuf++ = ival;
	*vadcbuf++ = vval;
        *bbuf++ =bval;
	setIntegerParam(i, P_VoltageIn, vval);
	setIntegerParam(i, P_DigitalIn, bval);
    } 
    return(asynSuccess);
}

asynStatus drvNSLS2_MIO::readBit(int channel, int *value)
{
 unsigned int bits,bit; 
   bits = (unsigned int)pl_register_read(intfd_, INBITS);
   bit=(bits >> channel) & 0x01;
//   printf("channel %i bit=%i\n",channel, bit);
    *value=bit;
    return asynSuccess ;
}

bool drvNSLS2_MIO::isAcquiring()
{
  return acquiring_;
}

#ifdef POLLING_MODE
static void pollerThread(void *pPvt)
{
    while(1) { /* Do forever */
        if (pdrvNSLS2_MIO->isAcquiring()) pdrvNSLS2_MIO->callbackFunc();
        epicsThreadSleep(POLL_TIME);
    }
}
#else
/*******************************************
* open interrupt driver 
*
********************************************/
asynStatus drvNSLS2_MIO::pl_open(int *fd) {
    if ( (*fd = open(DEVNAME, O_RDWR)) <= 0 ) {
        perror(__func__);
        return(asynError);
    }

    return(asynSuccess);
}

// C callback function called by Linux when an interrupt occurs.  
// It calls the callbackFunc in your C++ driver.
static void frame_done(int signum)
{
    pdrvNSLS2_MIO->callbackFunc();
}
#endif


// The constructor for your driver
drvNSLS2_MIO::drvNSLS2_MIO(const char *portName, int ringBufferSize) : drvQuadEM(portName, ringBufferSize)
{
    int i;
    float fsd;

// Set the global pointer
    pdrvNSLS2_MIO = this;      
 
    //const char *functionName = "drvNSLS2_MIO";

    // Current ranges in microamps
    iranges_[0]=0.1;
    iranges_[1]=1;
    iranges_[2]=10;
    iranges_[3]=100;
    iranges_[4]=1000;
    iranges_[5]=10000;
    // Voltage ranges in Volts
    vranges_[0]=1.25;
    vranges_[1]=2.5;
    vranges_[2]=5.0;
    vranges_[3]=10.0;
    
    calibrationMode_ = false;
    for (i=0; i<QE_MAX_INPUTS; i++) ADCOffset_[i] = 0;
  
    // Initialize Linux driver, set callback function
#ifdef POLLING_MODE
    epicsThreadCreate("NSLS2I_EMPoller",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)pollerThread,
                      this);
#else
    pl_open(&intfd_);
    signal(SIGIO, &frame_done);
    fcntl(intfd_, F_SETOWN, getpid());
    int oflags = fcntl(intfd_, F_GETFL);
    fcntl(intfd_, F_SETFL, oflags | FASYNC);
#endif   
    // Create new parameter for DACs
    createParam(P_DACString,             asynParamInt32, &P_DAC);
    createParam(P_CalibrationModeString, asynParamInt32, &P_CalibrationMode);
    createParam(P_ADCOffsetString,       asynParamInt32, &P_ADCOffset);
    createParam(P_VoltageInString, asynParamInt32, &P_VoltageIn);
    createParam(P_DigitalInString, asynParamInt32, &P_DigitalIn);
    createParam(P_DigitalOutString, asynParamInt32, &P_DigitalOut);
    createParam(P_CurRangeString,           asynParamInt32, &P_Cur_Range);
    createParam(P_VoltRangeString,           asynParamInt32, &P_Volt_Range);

    pl_register_write(intfd_, SA_RATE_DIV, (int)(50e6/FREQ + 0.5));
    fsd=(pow(2.0,17)-1.0);
    for (i=0;i<QE_MAX_INPUTS;i++){
        iscaleFactor_[i][0] = 0.1/fsd;
        iscaleFactor_[i][1] = 1.0/fsd;
        iscaleFactor_[i][2] = 10.0/fsd;
        iscaleFactor_[i][3] = 100.0/fsd;
        iscaleFactor_[i][4] = 1000.0/fsd;
	iscaleFactor_[i][5] = 10000.0/fsd;

        vscaleFactor_[i][0] = 1.25/fsd;
        vscaleFactor_[i][1] = 2.5/fsd;
        vscaleFactor_[i][2] = 5.0/fsd;
        vscaleFactor_[i][3] = 10.0/fsd;
	
    }
//    printf("scaleFactors: %g  %g  %g  %g\n", scaleFactor_[0][0], scaleFactor_[0][1], scaleFactor_[0][2],scaleFactor_[0][3] );

    initDAC();
    for(i=0;i<4;i++){
    setCurRange(i,0);
    setVoltRange(i,0);
    setDAC(i,0);
    }
    getFirmwareVersion();
    setIntegerParam(P_Model, QE_ModelNSLS2_MIO);
    callParamCallbacks();
}

drvNSLS2_MIO::~drvNSLS2_MIO()
{
    // This is your object's destructor
    // Free any resources that your object has allocated
//    munmap(0,255);
}


/** Called when asyn clients call pasynInt32->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus drvNSLS2_MIO::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int channel;
    const char *paramName;
    const char* functionName = "writeInt32";

    
    getAddress(pasynUser, &channel);
    
   /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Set the parameter in the parameter library. */
    setIntegerParam(channel, function, value);
    if (function == P_DAC) {
        status = setDAC(channel, value);
    }
    else if (function == P_CalibrationMode) {
        calibrationMode_ = (value != 0);
    }
    else if (function == P_ADCOffset) {
        ADCOffset_[channel] = value;
    }
    else if (function == P_DigitalOut) {
        writeBit(channel, value);
    }
    else if (function == P_Cur_Range){
    	status = setCurRange(channel,value);
	}
    else if (function == P_Volt_Range){
    	status = setVoltRange(channel,value);
	}

    //  If this is a base class parameter then call the base class
    else if (function < FIRST_NSLS2_COMMAND) {
        return drvQuadEM::writeInt32(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%d", 
                  driverName, functionName, status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName, functionName, function, paramName, value);
    return (asynStatus)status;
}

asynStatus drvNSLS2_MIO::writeBit(int bit, epicsInt32 value)
{
 int bits, result;
    bits = pl_register_read(intfd_, OUTBITS);
    if(value==0){
       result=(bits & (~(0x0001<<bit)));
       }
    if(value==1){
       result=(bits | (0x0001<<bit));
       }
    pl_register_write(intfd_, OUTBITS, result);
    return asynSuccess ;
}

asynStatus drvNSLS2_MIO::getBounds(asynUser *pasynUser, epicsInt32 *low, epicsInt32 *high)
{
    int function = pasynUser->reason;

    if (function == P_DAC) {
        *low = -32768;
        *high = 32767;
    }
    else {
        return drvQuadEM::getBounds(pasynUser, low, high);
    }
    return asynSuccess;
}

//  Callback function in driver
void drvNSLS2_MIO::callbackFunc()
{
    int iinput[QE_MAX_INPUTS];
    int vinput[QE_MAX_INPUTS];
    int binput[QE_MAX_INPUTS];
    int i, irange, vrange, nvalues;
    //static const char *functionName="callbackFunc";
   pl_register_write(intfd_, LEDS, 0x00000010);
    
    lock();

    /* Read the new data as integers */
    readMeter(iinput,vinput,binput);

    getIntegerParam(P_Cur_Range, &irange);
    getIntegerParam(P_Volt_Range, &vrange); 
    getIntegerParam(P_ValuesPerRead, &nvalues); 

    /* Convert to double) */
    for (i=0; i<QE_MAX_INPUTS; i++) {
        rawCurData_[i] = (double)iinput[i] * 16.0 / (double)nvalues;
        rawVoltData_[i] = (double)vinput[i] / (double)nvalues;
        if (!calibrationMode_) {
            rawCurData_[i] = (rawCurData_[i] - ADCOffset_[i]) * iscaleFactor_[i][irange];
	    rawVoltData_[i] = rawVoltData_[i] * vscaleFactor_[i][vrange];
        }
    }

    computePositions(rawCurData_);
    unlock();
    pl_register_write(intfd_, LEDS, 0xffffffef);
}


// Other functions

asynStatus drvNSLS2_MIO::setAcquire(epicsInt32 value)
{
    // 1=start acquire, 0=stop.
    acquiring_ = value;
    pl_register_write(intfd_, IRQ_ENABLE, value);
    return asynSuccess;
}

asynStatus drvNSLS2_MIO::setAcquireParams()
{
    int numAverage;
    int valuesPerRead;
    double sampleTime;
    double averagingTime;
    //static const char *functionName = "setAcquireParams";

    getIntegerParam(P_ValuesPerRead,    &valuesPerRead);
    getDoubleParam (P_AveragingTime,    &averagingTime);

    // Program valuesPerRead in the FPGA
    pl_register_write(intfd_, SA_RATE, valuesPerRead);

#ifdef POLLING_MODE
    sampleTime = POLL_TIME;
#else
    // Compute the sample time.  This is valuesPerRead / FREQ. 
    sampleTime = valuesPerRead / FREQ;
#endif
    setDoubleParam(P_SampleTime, sampleTime);

    numAverage = (int)((averagingTime / sampleTime) + 0.5);
    setIntegerParam(P_NumAverage, numAverage);
    if(numAverage > 1){ 
        readingsAveraged_=1;
    }
    else readingsAveraged_=0;
    printf("ReadingsAveraged = %i\n", readingsAveraged_);
    return asynSuccess;
}

asynStatus drvNSLS2_MIO::setAveragingTime(epicsFloat64 value)
{
    return setAcquireParams();
}

/** Sets the values per read.
  * \param[in] value Values per read. Minimum depends on number of channels.
  */
asynStatus drvNSLS2_MIO::setValuesPerRead(epicsInt32 value) 
{
   setIntegerParam(P_ValuesPerRead, value); 
   return setAcquireParams();
}

asynStatus drvNSLS2_MIO::initDAC()
{
//     static const char *functionName = "initDAC";
   // First must write the Power Control Register to turn on DAC outputs
    pl_register_write(intfd_, DACS, 0x10001F);
   //see data sheet for bit def.
    epicsThreadSleep(0.001); 
    // Set Output Select Range to -10v to +10v
    pl_register_write(intfd_, DACS, 0x0C0004);
    epicsThreadSleep(0.001);
    return(asynSuccess);
} 

asynStatus drvNSLS2_MIO::setDAC(int channel, int value)
{
    int dacVolts;
    static const char *functionName = "setDAC";
    
    printf("DAC %i; Value %i\n",channel, value); 
    //append which of the 4 dacs to bits 18-16 (see data sheet)
    dacVolts = (channel << 16) | (value & 0xffff); 
    // Write the DAC voltage
    printf("dacVolts=%0x\n",dacVolts);
    pl_register_write(intfd_, DACS, dacVolts);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s::%s channel=%d val=%d\n",
        driverName, functionName, channel, value);
    return(asynSuccess);
}


asynStatus drvNSLS2_MIO::setBiasVoltage(epicsFloat64 value)
{
    pl_register_write(intfd_, HV_BIAS, (int) (value *65535.0/10.0));
    printf("Setting bias voltage: %g\n",value);
    return asynSuccess ;
}

asynStatus drvNSLS2_MIO::setCurRange(int channel, epicsInt32 value)
{
    printf("Channel %i  Gain: %i\n",channel,value);

unsigned int code, reg;

    switch(value){
       case 0:
         code = 0x000041;
        break;
       case 1:
         code = 0x000082;
         break;
       case 2:
         code = 0x000104;
         break;
       case 3:
         code = 0x000208;
         break;
       case 4:
         code = 0x000410;
         break;
       case 5:
         code = 0x000820;
         break;
	 }
   reg = code | (0x1 <<(24+channel));
   printf("reg=%i\n",reg);
   pl_register_write(intfd_, CURGAINREG, reg);
   callParamCallbacks(channel);
   return asynSuccess;
}
asynStatus drvNSLS2_MIO::setVoltRange(int channel, epicsInt32 value)
{
int reg;

    printf("Channel %i  Gain: %i\n",channel,value);

 reg=pl_register_read(intfd_,VOLTGAINREG);
 switch(channel){
   case 0:
    switch(value){
       case 0:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffffc) | 0x00000000));
     /*    fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffffc)|0x00000000;*/
        break;
       case 1:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffffc) | 0x00000001));
     /*    fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffffc)|0x00000001;*/
         break;
       case 2:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffffc) | 0x00000002));
    /*     fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffffc)|0x00000002;*/
         break;
       case 3:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffffc) | 0x00000003));
    /*     fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffffc)|0x00000003;*/
         break;
	 }
    break;
  case 1:
    switch(value){
       case 0:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffff3) | 0x00000000));
    /*     fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffff3)|0x0000000;*/
        break;
       case 1:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffff3) | 0x00000004));
   /*      fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffff3)|0x0000004;*/
         break;
       case 2:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffff3) | 0x00000008));
   /*      fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffff3)|0x0000008;*/
         break;
       case 3:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xfffffff3) | 0x0000000c));
   /*      fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xfffffff3)|0x000000c;*/
         break;
	 }
    break;
  case 2:	 
    switch(value){
       case 0:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffffcf) | 0x00000000));
  /*       fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffffcf)|0x00000000;*/
        break;
       case 1:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffffcf) | 0x00000010));
  /*       fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffffcf)|0x00000010;*/
         break;
       case 2:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffffcf) | 0x00000020));
  /*       fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffffcf)|0x00000020;*/
         break;
       case 3:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffffcf) | 0x00000030));
  /*       fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffffcf)|0x00000030;*/
         break;
	 }
    break;
  case 3:
    switch(value){
       case 0:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffff3f) | 0x00000000));
 /*        fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffff3f)|0x00000000;*/
        break;
       case 1:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffff3f) | 0x00000040));
 /*        fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffff3f)|0x00000040;*/
         break;
       case 2:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffff3f) | 0x00000080));
 /*        fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffff3f)|0x00000080;*/
         break;
       case 3:
         pl_register_write(intfd_, VOLTGAINREG, ((reg & 0xffffff3f) | 0x000000c0));
 /*        fpgabase_[VOLTGAINREG] = (fpgabase_[VOLTGAINREG]&0xffffff3f)|0x000000c0;*/
         break;
	 }
    break;
   }
   callParamCallbacks(channel);
   return asynSuccess;
}


asynStatus drvNSLS2_MIO::readStatus()
{
    return asynSuccess;
}

asynStatus drvNSLS2_MIO::getFirmwareVersion()
{
 int fver, i;
 char tmpstr[256];
     for(i=0;i<256;i++){
        tmpstr[i]=0;
	}
     
     fver = pl_register_read(intfd_, FPGAVER);
  /*   fver = fpgabase_[FPGAVER]; */
     sprintf(tmpstr,"FPGA Version %i",fver);
     printf("FPGA version=%s\n",tmpstr);
     strncpy(firmwareVersion_,tmpstr, strlen(tmpstr)+1);
     setStringParam(P_Firmware, firmwareVersion_);
    return asynSuccess;
}

asynStatus drvNSLS2_MIO::reset()
{
    return asynSuccess;
}

void drvNSLS2_MIO::report(FILE *fp, int details)
{
    // Print any information you want about your driver
    
    // Call the base class report method
    drvQuadEM::report(fp, details);
}

/*
int Peekfunc(int reg){
  int value;
  if((reg>0)&(reg<256)){
    val = pl_register_read(intfd_, reg);
    }
  else{
   printf("Bad register number: %i\n",reg);
   value=0;
   }
  return(value);
}

int Pokefunc(int reg, int value){
  if((reg>0)&(reg<256)){
    pl_register_write(intfd_, reg, value);
    }
  else{
   printf("Bad register number: %i\n",reg);
   value=0;
   }
   return(Peekfunc(reg));
}
*/


void drvNSLS2_MIO::exitHandler()
{
    // Do anything that needs to be done when the EPICS is shutting down
    fcloseall();
}

/* That's all you need to send the data to the quadEM base class.  
You need to write a readMeter() function (or some other name) 
to actually read the ADC registers.  You also need to implement 
any of these quadEM base class functions that apply to your device: */

//    virtual asynStatus setAcquireMode(epicsInt32 value);
//    virtual asynStatus setAveragingTime(epicsFloat64 value);
//    virtual asynStatus setBiasState(epicsInt32 value);
//    virtual asynStatus setBiasVoltage(epicsFloat64 value);
//    virtual asynStatus setBiasInterlock(epicsInt32 value);
//    virtual asynStatus setPingPong(epicsInt32 value);
//    virtual asynStatus setIntegrationTime(epicsFloat64 value);
//    virtual asynStatus setNumChannels(epicsInt32 value);
//    virtual asynStatus setNumAcquire(epicsInt32 value);
//    virtual asynStatus setReadFormat(epicsInt32 value);
//    virtual asynStatus setResolution(epicsInt32 value);
//    virtual asynStatus setTriggerMode(epicsInt32 value);
//    virtual asynStatus setTriggerPolarity(epicsInt32 value);
//    virtual asynStatus setValuesPerRead(epicsInt32 value);
//    virtual asynStatus triggerCallbacks();

/* If your device requires parameters that are not included in the 
base class then you need to implement the writeInt32 and/or writeFloat64 
methods to handle the driver-specific parameters, and then call 
quadEM::writeInt32 or quadEM::writeFloat64 so it can handle the 
standard parameters. */
/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

/** EPICS iocsh callable function to call constructor for the drvNSLS_EM class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] ringBufferSize The number of samples to hold in the input ring buffer.
  *            This should be large enough to hold all the samples between reads of the
  *            device, e.g. 1 ms SampleTime and 1 second read rate = 1000 samples.
  *            If 0 then default of 2048 is used.
  */
int drvNSLS2_MIOConfigure(const char *portName, int ringBufferSize)
{
    new drvNSLS2_MIO(portName, ringBufferSize);
    return(asynSuccess);
}


/* EPICS iocsh shell commands */

//extern int Peekfunc(int);
//extern int Pokefunc(int,int);
static const iocshArg initArg0 = { "portName", iocshArgString};
static const iocshArg initArg1 = { "moduleID",iocshArgInt};
static const iocshArg initArg2 = { "ring buffer size",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"drvNSLS2_MIOConfigure",2,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    drvNSLS2_MIOConfigure(args[0].sval, args[1].ival);
}

void drvNSLS2_MIORegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(drvNSLS2_MIORegister);

//
//extern unsigned int fpgabase_;

//int peek(int reg){
//	printf("Register %i = %i\n", fpgabase_[reg]);
//	return(0);
//	}

//static const iocshArg peekArg0 = {"Register #", iocshArgInt};
//static const iocshArg * const peekArgs[1] = {&peekArg0};

//static const iocshFuncDef peekFuncDef={"peek",1,peekArgs};
//static void peekCallFunc(const iocshArgBuf *args)
//{
// peek((int) args[0].ival);
//}

//void registerpeek(void){
//        iocshRegister(&peekFuncDef,&peekCallFunc);
//        }

//epicsExportRegistrar(registerpeek);


//int poke(int reg, int val){
//	fpgabase_[reg]=val;
//       peek(reg);
//	return(0);
//	}



//static const iocshArg pokeArg0 = {"Register #", iocshArgInt};
//static const iocshArg pokeArg1 = {"Val", iocshArgInt};
//static const iocshArg * const pokeArgs[2] = {&pokeArg0,&pokeArg1,};


//static const iocshFuncDef pokeFuncDef={"poke",2,pokeArgs};
//static void pokeCallFunc(const iocshArgBuf *args)
//{
// poke((int) args[0].ival, (int) args[1].ival);
//}

//void registerpoke(void){
//        iocshRegister(&pokeFuncDef,&pokeCallFunc);
//        }

//epicsExportRegistrar(registerpoke);

}
