#ifndef CS5490_h
#define CS5490_h

#include "Arduino.h"
#include "SoftwareSerial.h"

#define DEFAULT_RX_PIN 13
#define DEFAULT_TX_PIN 15
#define UART_BEGIN_DELAY 100


#define BITS_ADDR 6

/*DEFINES GENERALES DEL CS5490*/

#define  CS5490_DEFAULT_BAUDRATE 600
#define  CS5490_MAX_BAUDRATE 512000
#define  CS5490_REG_SIZE 3 //TAMAÑO DE LOS REGISTROS EN BYTES
#define  CS5490_DEFAULT_MCLK 4096000.0 //Frecuencia del cristal en Hz
#define  CS5490_OWR 4000 //output word rate en hz
#define  CS5490_DEFAULT_SAMPLE_COUNT 4000


//REGISTROS DE LA PAGINA 0
#define CS5490_CONFIG0 00
#define CS5490_CONFIG1 01
#define CS5490_MASK 03
#define CS5490_PC 05
#define CS5490_SERIALCTRL 07
#define CS5490_PULSEWIDTH 08
#define CS5490_PULSECTRL 09
#define CS5490_STATUS0 23
#define CS5490_STATUS1 24
#define CS5490_STATUS2 25 
#define CS5490_REGLOCK 34
#define CS5490_VPEAK 36
#define CS5490_IPEAK 37
#define CS5490_PSDC 48
#define CS5490_ZXNUM 55

//REGISTROS DE LA PAGINA 16
#define CS5490_CONFIG2 (00 | (16 << BITS_ADDR))
#define CS5490_REGCHK  (01 | (16 << BITS_ADDR))
#define CS5490_I (02 | (16 << BITS_ADDR))
#define CS5490_V (03 | (16 << BITS_ADDR))
#define CS5490_P (04 | (16 << BITS_ADDR))
#define CS5490_PAVG (05 | (16 << BITS_ADDR))
#define CS5490_IRMS (06 | (16 << BITS_ADDR))
#define CS5490_VRMS (07 | (16 << BITS_ADDR))
#define CS5490_QAVG (14 | (16 << BITS_ADDR))
#define CS5490_Q (15 | (16 << BITS_ADDR))
#define CS5490_S (20 | (16 << BITS_ADDR))
#define CS5490_PF (21 | (16 << BITS_ADDR))
#define CS5490_T (27 | (16 << BITS_ADDR))
#define CS5490_PSUM (29 | (16 << BITS_ADDR))
#define CS5490_SSUM (30 | (16 << BITS_ADDR))
#define CS5490_QSUM (31 | (16 << BITS_ADDR))
#define CS5490_IDCOFF (32 | (16 << BITS_ADDR))
#define CS5490_IGAIN (33 | (16 << BITS_ADDR))
#define CS5490_VDCOFF (34 | (16 << BITS_ADDR))
#define CS5490_VGAIN (35 | (16 << BITS_ADDR))
#define CS5490_POFF (36 | (16 << BITS_ADDR))
#define CS5490_IACOFF (37 | (16 << BITS_ADDR))
#define CS5490_QOFF (38 | (16 << BITS_ADDR))
#define CS5490_EPSILON (49 | (16 << BITS_ADDR))
#define CS5490_SAMPLECOUNT (51 | (16 << BITS_ADDR))
#define CS5490_TGAIN (54 | (16 << BITS_ADDR))
#define CS5490_TOFF (55 | (16 << BITS_ADDR))
#define CS5490_TSETTLE (57 | (16 << BITS_ADDR))
#define CS5490_LOADMIN (58 | (16 << BITS_ADDR))
#define CS5490_SYSGAIN (60 | (16 << BITS_ADDR))
#define CS5490_TIME (61 | (16 << BITS_ADDR))


//REGISTROS DE LA PAGINA 17
#define CS5490_VSAGDUR (00 | (17 << BITS_ADDR))
#define CS5490_VSAGLEVEL (01 | (17 << BITS_ADDR))
#define CS5490_IOVERDUR (04 | (17 << BITS_ADDR))
#define CS5490_IOVERLEVEL (05 | (17 << BITS_ADDR))


//REGISTROS DE LA PAGINA 18
#define CS5490_IZXLEVEL (24 | (18 << BITS_ADDR))
#define CS5490_PULSERATE (28 | (18 << BITS_ADDR))
#define CS5490_INTGAIN (43 | (18 << BITS_ADDR))
#define CS5490_VSWELLDUR (46 | (18 << BITS_ADDR))
#define CS5490_VSWELLLEVEL (47 | (18 << BITS_ADDR))
#define CS5490_VZXLEVEL (58 | (18 << BITS_ADDR))
#define CS5490_CYCLECOUNT (62 | (18 << BITS_ADDR))
#define CS5490_SCALE (63 | (18 << BITS_ADDR))

//INSTRUCCIONES CS5490
#define CS5490_CMD_SRST 0x01
#define CS5490_CMD_SB 0x02
#define CS5490_CMD_WU 0x03
#define CS5490_CMD_SC 0x14
#define CS5490_CMD_CC 0x15
#define CS5490_CMD_HC 0x18

#define CS5490_CMD_DCOFF_I 0x21
#define CS5490_CMD_DCOFF_V 0x22
#define CS5490_CMD_DCOFF_IV 0x26

#define CS5490_CMD_ACOFF_I 0x31 //La calibracion del offset de AC solo es valida para la corriente, ver datasheet.

#define CS5490_CMD_GAIN_I 0x39
#define CS5490_CMD_GAIN_V 0x3A
#define CS5490_CMD_GAIN_IV 0x3E

class CS5490
{
	public:
          
	
	
	public:
		
		/*Constructores*/
		CS5490();
		CS5490(byte,byte);
    
	  //Metodos básicos de lectura/escritura de registros y comandos.
	  uint32_t readReg(uint16_t);
    uint32_t writeReg(uint16_t,uint32_t);
	  void cmd(uint8_t);

    /*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 0*/
    void setBaudRate(uint32_t);
    void updateStatus0();
    bool DRDY();
    bool VSWELL();
    bool IOC();
    bool VSAG();
    bool TUP();
    void clrDRDY();
    void clrVSWELL();
    void clrIOC();
    void clrVSAG();
    void clrTUP();
    float getVPeak();
    float getIPeak();
    uint32_t setZxNum(uint32_t);

    /*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 16*/
    uint32_t getRegChk();
    
    float getPAvg();
    float getIRMS();
    float getVRMS();
    float getQAvg();
    float getS();
    float getPF();
    float getT();
    
    uint32_t setIDCOff(float);
    uint32_t setIGain(float);
    uint32_t setVDCOff(float);
    uint32_t setVGain(float);
    uint32_t setPOff(float);
    uint32_t setQOff(float);
    uint32_t setIACOff(float);

    uint32_t setSampleCount(uint32_t);
        
    uint32_t setEpsilon(float);
    
    uint32_t setTGain(float);
    uint32_t setTOff(float);

    uint32_t setTSettle(uint32_t);
    
    uint32_t setLoadMin(float);
    
    uint32_t setSysGain(float);

    
    /*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 17*/
    uint32_t setVSagDur(uint32_t);
    uint32_t setVSagLevel(float);

    uint32_t setIOverDur(uint32_t);
    uint32_t setIOverLevel(float);

    /*METODOS ASOCIADOS A REGISTROS DE LA PAGINA 18*/
    uint32_t setIZxLevel(float);
    
    uint32_t setVSwellDur(uint32_t);
    uint32_t setVSwellLevel(float);
    
    uint32_t setVZxLevel(float);
    
    uint32_t setScale(float);

    /*Metodos vinculados a comandos*/
    void singleConv();

    /*METODOS PARA CALIBRAR*/
    void calDCOffIV();    
    void calACOffI();    
    void calGainIV();
    uint32_t regStatus0=0;
 
 private:
    
		SoftwareSerial SerialCS;
    
    
    float m_Vmin=100.0;
    float m_Imin=0.001;
    float m_Imax=10.0;
    float m_VLine=220.0;
    uint32_t m_IntReg;
    
    uint32_t m_xtalF=4096000; //frecuencia del cristal en hz 
    uint32_t m_sampleRate;
    
    uint8_t m_DOpin;
    uint8_t m_RSTpin;

    //Funciones auxiliares
    float regToPm1(uint32_t);
    uint32_t pm1ToReg(float);
    
};

#endif
