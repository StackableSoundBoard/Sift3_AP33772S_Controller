#pragma once
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_i2c.h"
#include <stdint.h>

namespace AP33772S {
  namespace REG {
  constexpr uint16_t STATUS   = 0x01<<8 | 0x01;
  constexpr uint16_t MASK     = 0x02<<8 | 0x01;
  constexpr uint16_t OPMODE   = 0x03<<8 | 0x01;
  constexpr uint16_t CONFIG   = 0x04<<8 | 0x01;
  constexpr uint16_t PDCONFIG = 0x05<<8 | 0x01;
  constexpr uint16_t SYSTEM   = 0x06<<8 | 0x01;

  constexpr uint16_t TR25     = 0x0c<<8 | 0x02;
  constexpr uint16_t TR50     = 0x0d<<8 | 0x02;
  constexpr uint16_t TR75     = 0x0e<<8 | 0x02;
  constexpr uint16_t TR100    = 0x0f<<8 | 0x02;

  constexpr uint16_t VOLTAGE  = 0x11<<8 | 0x02;
  constexpr uint16_t CURRENT  = 0x12<<8 | 0x01;
  constexpr uint16_t TEMP     = 0x13<<8 | 0x01;
  constexpr uint16_t VREQ     = 0x14<<8 | 0x02;
  constexpr uint16_t IREQ     = 0x15<<8 | 0x02;
  constexpr uint16_t VSELMIN  = 0x16<<8 | 0x01;
  constexpr uint16_t UVPTHR   = 0x17<<8 | 0x01;
  constexpr uint16_t OVPTHR   = 0x18<<8 | 0x01;
  constexpr uint16_t OCPTHR   = 0x19<<8 | 0x01;
  constexpr uint16_t OTPTHR   = 0x19<<8 | 0x01;
  constexpr uint16_t DRTHR    = 0x1a<<8 | 0x01;
  
  constexpr uint16_t SRCPDO         = 0x20<<8 | 0x1a;
  constexpr uint16_t SPC_SPR_PDO1   = 0x21<<8 | 0x02;
  constexpr uint16_t SPC_SPR_PDO2   = 0x22<<8 | 0x02;
  constexpr uint16_t SPC_SPR_PDO3   = 0x23<<8 | 0x02;
  constexpr uint16_t SPC_SPR_PDO4   = 0x24<<8 | 0x02;
  constexpr uint16_t SPC_SPR_PDO5   = 0x25<<8 | 0x02;
  constexpr uint16_t SPC_SPR_PDO6   = 0x26<<8 | 0x02;
  constexpr uint16_t SPC_SPR_PDO7   = 0x27<<8 | 0x02;
  constexpr uint16_t SPC_EPR_PDO8   = 0x28<<8 | 0x02;
  constexpr uint16_t SPC_EPR_PDO9   = 0x29<<8 | 0x02;
  constexpr uint16_t SPC_EPR_PDO10  = 0x2a<<8 | 0x02;
  constexpr uint16_t SPC_EPR_PDO11  = 0x2b<<8 | 0x02;
  constexpr uint16_t SPC_EPR_PDO12  = 0x2c<<8 | 0x02;
  constexpr uint16_t SPC_EPR_PDO13  = 0x2d<<8 | 0x02;

  constexpr uint16_t PD_REQMSG  = 0x31<<8 | 0x02;
  constexpr uint16_t PD_CMDMSG  = 0x32<<8 | 0x01;
  constexpr uint16_t PD_MSGRLT  = 0x33<<8 | 0x01;  
  }
  
  typedef enum: uint8_t{
    FixedPDO = 0, ADPO = 1
  }PDOType_E;

  typedef enum{
    MaxWatt, MinWatt, MaxCurrent 
  }PDOFind_Mode ;

  typedef struct{
    uint8_t VoltageMax:8;
    uint8_t PeakCurrent_VoltageMin:2;
    uint8_t CurrentMax:4;
    uint8_t Type:1;
    uint8_t Detect:1;
    
    void Clear(){
      VoltageMax =0;
      PeakCurrent_VoltageMin = 0;
      CurrentMax = 0;
      Type = 0;
      Detect = 0;
    };
    uint32_t MaxCurrent(uint16_t currentidx){return currentidx*250+1000;};

    bool Eval_FixedPDO(uint16_t Voltage100mV_, uint32_t CurrentmA_){
      return (Detect)&(Type==FixedPDO)&(VoltageMax == Voltage100mV_)&(CurrentmA_<=MaxCurrent(CurrentMax));
    }

    bool Eval_ADPO(uint16_t Voltage100mV_, uint32_t CurrentmA_){
      uint16_t VoltageMin = (PeakCurrent_VoltageMin==1)? 33 : 50;
      return (Detect)&(Type==ADPO)&(VoltageMin<=Voltage100mV_)&&(Voltage100mV_<=VoltageMax)&(CurrentmA_<=MaxCurrent(CurrentMax));
    }
  }SRCPDO_T;

  typedef struct{
    uint8_t VOLTAGE_SEL:8;
    uint8_t CURRENT_SEL:4;
    uint8_t POD_Index:4;
  } ReqMsg_T;

  class AP33772S_C{
    private:  
      uint8_t AP33772S_Base_Addr = 0x52<<1;
      int SrcPDO_Num;
      ReqMsg_T ReqPDO_MSG;
      I2C_HandleTypeDef* hi2c;
      bool LastCMD_Sucess = false;

    public:
      AP33772S_C(I2C_HandleTypeDef *hi2c_){
        hi2c = hi2c_;
        for(int ii=0;ii<14;ii++) SrcPDO_List[ii].Clear();
        SrcPDO_Num = 0;
      }

      SRCPDO_T SrcPDO_List[14];

      // AP33772S Function
      void Read_SrcPDO();
      void SetVout(bool IsEnable);

      // Util function
      uint16_t Current2Idx(uint16_t ReqCurrentmA_);
      uint8_t  WaitResponse(int Maxloop=30);

      // FInd PDO Function
      uint8_t FindPDO_Fixed(uint16_t Req_MinVoltage100mV_, uint16_t Req_MaxVoltage100mV_, uint16_t ReqCurrentmA_, PDOFind_Mode Mode);
      uint8_t FindPDO_ADPO(uint16_t Req_MinVoltage100mV_, uint16_t Req_MaxVoltage100mV_, uint16_t ReqCurrentmA_, PDOFind_Mode Mode, uint16_t &ReqVoltage100mV_);
      int     FindPDO_Nums();
      uint8_t ReqFixed(uint8_t SRCPDOIdx_,  uint16_t ReqCurrentmA_);
      uint8_t ReqAPDO(uint8_t SRCPDOIdx_, uint16_t ReqVoltage100mV_, uint16_t ReqCurrentmA_);      

    };
}
