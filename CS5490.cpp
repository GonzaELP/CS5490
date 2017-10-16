#include "CS5490.h"

CS5490::CS5490(void):SerialCS(DEFAULT_RX_PIN,DEFAULT_TX_PIN)
{
	//SerialCS.setRX(DEFAULT_RX_PIN);
	//SerialCS.setTX(DEFAULT_TX_PIN);
	SerialCS.begin(CS5490_DEFAULT_BAUDRATE);
	delay(UART_BEGIN_DELAY);
}

CS5490::CS5490(byte rxPin, byte txPin):SerialCS(rxPin,txPin)
{
	//SerialCS.setRX(rxPin);
	//SerialCS.setTX(txPin);
	SerialCS.begin(CS5490_DEFAULT_BAUDRATE);
	delay(UART_BEGIN_DELAY);
}

uint32_t CS5490::readReg(uint16_t dir)
{
	byte buff[CS5490_REG_SIZE];
	uint32_t val=0;
	
	SerialCS.write((byte)((dir >> 6) | 0x80)); //primero escribo la página (comienza en por eso el 0x80), es decir los 6 bits MS
	SerialCS.write((byte)(dir & 0x3F)); // luego escribo el registro dentro de la pagina, los 6 bits LS
	
	SerialCS.readBytes(buff,CS5490_REG_SIZE); //Leo los bytes con los valores del registro
	
	for(int i=0; i<CS5490_REG_SIZE;i++)
	{val|=(buff[i] << (8*i));}
	
	return val;
}

uint32_t CS5490::writeReg(uint16_t dir, uint32_t val)
{
	SerialCS.write((byte)((dir >>6) | 0x80));
  SerialCS.write((byte)((dir & 0x3F) | 0x40));
    
	for(int i=0; i< CS5490_REG_SIZE;i++)
	{SerialCS.write((byte)(val >> 8*i));}
  
  return val;
}

void CS5490::cmd(uint8_t cmd)
{SerialCS.write(cmd | 0xC0);}

float CS5490::regToPm1(uint32_t regVal)
{
   if(regVal & (1<<23)) //si el bit 23 es 1, entonces el número es negativo!
   {return ((((float)regVal)-((float)(1<<24)))/((float)(1<<23)));}

   else
   {return (((float)(regVal))/((float)(1<<23)));}
}

uint32_t CS5490::pm1ToReg(float val)
{
   if(val < 0) //caso negativo!
   {
    if(val < -1.0)
      {return ((uint32_t)(-1.0*(1<<23)+(1<<24)));}
    else
      {return ((uint32_t)(val*(1<<23)+(1<<24)));}
    }
   else //caso positivo
   {  
    if(val > 1.0)
    {return ((uint32_t)(1.0*(1<<23))); }
    else
    {return ((uint32_t)(val*(1<<23))); }
   }
}

/*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 0*/
void CS5490::setBaudRate(uint32_t baudRate)
{  
  uint32_t aux= (uint32_t) (float(baudRate)*(524288.0/CS5490_DEFAULT_MCLK)); //calculo el valor de BaudRate de 16 bits (ver datasheet).
  aux&=0x00FFFF; //borro cualquier cosa por encima de los 16 LSB
  aux|= ((readReg(CS5490_SERIALCTRL)) & (0x00FF0000)); //Los demás bits los mantengo en el valor original del registro

  writeReg(CS5490_SERIALCTRL,aux); //Finalmente escribo el registro con el nuevo valor.

  SerialCS.flush(); //limpio el buffer del canal serie.
  SerialCS.begin(baudRate); //vuelvo a iniciar el puerto serie con el nuevo baudRate.
  delay(UART_BEGIN_DELAY);
 
} 

void CS5490::updateStatus0()
{regStatus0=readReg(CS5490_STATUS0);}

bool CS5490::DRDY()
{return(regStatus0 & (1<<23));}
bool CS5490::VSWELL()
{return(regStatus0 & (1<<16));}
bool CS5490::IOC()
{return(regStatus0 & (1<<8));}
bool CS5490::VSAG()
{return(regStatus0 & (1<<6));}
bool CS5490::TUP()
{return(regStatus0 & (1<<5));}

void CS5490::clrDRDY()
{ regStatus0 &= (~(1<<23));
  writeReg(CS5490_STATUS0,(1<<23));}
void CS5490::clrVSWELL()
{ regStatus0 &= (~(1<<16));
  writeReg(CS5490_STATUS0,(1<<16));}
void CS5490::clrIOC()
{ regStatus0 &= (~(1<<8));
  writeReg(CS5490_STATUS0,(1<<8));}
void CS5490::clrVSAG()
{ regStatus0 &= (~(1<<6));
  writeReg(CS5490_STATUS0,(1<<6));}
void CS5490::clrTUP()
{ regStatus0 &= (~(1<<5));
  writeReg(CS5490_STATUS0,(1<<5));}


float CS5490::getVPeak(void)
{return readReg(regToPm1(CS5490_VPEAK));}
float CS5490::getIPeak(void)
{return readReg(regToPm1(CS5490_IPEAK));}

uint32_t CS5490::setZxNum(uint32_t num)
{return writeReg(CS5490_ZXNUM,num);}


/*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 16*/
uint32_t CS5490::getRegChk()
{return readReg(CS5490_REGCHK);}

float CS5490::getPAvg()
{return regToPm1(readReg(CS5490_PAVG));}
float CS5490::getIRMS()
{return (readReg(CS5490_IRMS)) /((float)(1<<24));}
float CS5490::getVRMS()
{return (readReg(CS5490_VRMS)) /((float)(1<<24));}
float CS5490::getQAvg()
{return regToPm1(readReg(CS5490_QAVG));}
float CS5490::getS()
{return (readReg(CS5490_S)) /((float)(1<<24));}
float CS5490::getPF()
{return regToPm1(readReg(CS5490_PF));}
float CS5490::getT()
{return 128*regToPm1(readReg(CS5490_T));}

uint32_t CS5490::setIDCOff(float val)
{return writeReg(CS5490_IDCOFF,pm1ToReg(val));}
uint32_t CS5490::setIGain(float val)
{return writeReg(CS5490_IGAIN,(float)(1<<22));}
uint32_t CS5490::setVDCOff(float val)
{return writeReg(CS5490_VDCOFF,pm1ToReg(val));}
uint32_t CS5490::setVGain(float val)
{return writeReg(CS5490_VGAIN,(float)(1<<22));}
uint32_t CS5490::setPOff(float val)
{return writeReg(CS5490_POFF,pm1ToReg(val));}
uint32_t CS5490::setQOff(float val)
{return writeReg(CS5490_QOFF,pm1ToReg(val));}
uint32_t CS5490::setIACOff(float val)
{return writeReg(CS5490_IACOFF,(uint32_t)(val*(1<<24)));}

uint32_t CS5490::setEpsilon(float val)
{return writeReg(CS5490_EPSILON,pm1ToReg(val));}

uint32_t CS5490::setSampleCount(uint32_t val)
{return writeReg(CS5490_SAMPLECOUNT,val);}

uint32_t CS5490::setTGain(float val)
{return writeReg(CS5490_TGAIN, (uint32_t)(val*(1<<16)));}
uint32_t CS5490::setTOff(float val)
{return writeReg(CS5490_TOFF,pm1ToReg(val/128.0));}

uint32_t CS5490::setTSettle(uint32_t time_ms)
{return writeReg(CS5490_TSETTLE,(CS5490_OWR/1000)*time_ms);}
    
uint32_t CS5490::setLoadMin(float val)
{return writeReg(CS5490_LOADMIN,pm1ToReg(val));}
    
uint32_t CS5490::setSysGain(float val)
{return writeReg(CS5490_SYSGAIN,pm1ToReg(val/2.0));}

/*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 17*/
uint32_t CS5490::setVSagDur(uint32_t time_ms)
{return writeReg(CS5490_VSAGDUR,(CS5490_OWR/1000)*time_ms);}
uint32_t CS5490::setVSagLevel(float val)
{return writeReg(CS5490_VSAGLEVEL,pm1ToReg(val));}
uint32_t CS5490::setIOverDur(uint32_t time_ms)
{return writeReg(CS5490_IOVERDUR,(CS5490_OWR/1000)*time_ms);}
uint32_t CS5490::setIOverLevel(float val)
{return writeReg(CS5490_IOVERLEVEL,pm1ToReg(val));}


/*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 18*/
uint32_t CS5490::setIZxLevel(float val)
{return writeReg(CS5490_IZXLEVEL,pm1ToReg(val));}

uint32_t CS5490::setVSwellDur(uint32_t time_ms)
{return writeReg(CS5490_VSWELLDUR,(CS5490_OWR/1000)*time_ms);}
uint32_t CS5490::setVSwellLevel(float val)
{return writeReg(CS5490_VSWELLLEVEL,pm1ToReg(val));}

uint32_t CS5490::setVZxLevel(float val)
{return writeReg(CS5490_IZXLEVEL,pm1ToReg(val));}

uint32_t CS5490::setScale(float val)
{return writeReg(CS5490_SCALE,pm1ToReg(val));}


/*METODOS ASOCIADOS A COMANDOS*/
void CS5490::singleConv()
{cmd(CS5490_CMD_SC);}

/*METODOS PARA CALIBRAR*/
void CS5490::calDCOffIV(void)
{cmd(CS5490_CMD_DCOFF_IV);}

void CS5490::calACOffI(void)
{cmd(CS5490_CMD_ACOFF_I);}

void CS5490::calGainIV(void)
{cmd(CS5490_CMD_GAIN_IV);}


