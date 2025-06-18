#pragma once
#include "stm32c0xx_hal.h"
#include "cstdint"
#include "stm32c0xx_hal_i2c.h"
#include <cstdint>

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
    uint32_t MaxCurrent(uint16_t currentidx){return currentidx*1250-10;};

    bool Eval_FixedPDO(uint16_t Voltage100mV_, uint32_t CurrentmA_){
      return (Detect)&(Type==FixedPDO)&(VoltageMax == Voltage100mV_)&(CurrentmA_<=MaxCurrent(CurrentMax));
    }

    bool Eval_ADPO(uint16_t Voltage100mV_, uint32_t CurrentmA_){
      uint16_t MinVoltage = (PeakCurrent_VoltageMin==1)? 33 : 50;
      return (Detect)&(Type==ADPO)&(MinVoltage<=Voltage100mV_)&&(Voltage100mV_<=MinVoltage)&(CurrentmA_<=MaxCurrent(CurrentMax));
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
      SRCPDO_T SrcPDO_List[14];
      ReqMsg_T ReqPDO_MSG;
      I2C_HandleTypeDef* hi2c;

    public:
      AP33772S_C(I2C_HandleTypeDef *hi2c_){
        hi2c = hi2c_;
        for(int ii=0;ii<14;ii++) SrcPDO_List[ii].Clear();
      }

      void Read_SrcPDO(){
        HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::SRCPDO>>8, 1, (uint8_t*)SrcPDO_List, REG::SRCPDO&0xff, 10);
      }

      uint16_t Current2Idx(uint16_t ReqCurrentmA_){return ReqCurrentmA_/250-4;};

      uint8_t FindPDO_Fixed(uint16_t ReqVoltage100mV_, uint16_t ReqCurrentmA_, PDOFind_Mode Mode){
        uint8_t WattIdx = 0xff;
        for(int ii=0;ii<14;ii++){
          if(SrcPDO_List[ii].Eval_FixedPDO(ReqVoltage100mV_, ReqCurrentmA_)){
            uint32_t CurrentWatt = SrcPDO_List[ii].MaxCurrent(SrcPDO_List[ii].CurrentMax)*SrcPDO_List[ii].VoltageMax;
            uint32_t OldWatt     = SrcPDO_List[WattIdx].MaxCurrent(SrcPDO_List[WattIdx].CurrentMax)*SrcPDO_List[WattIdx].VoltageMax;
            if(WattIdx==0xff) WattIdx = ii;
            else if ((Mode==MaxWatt)&(CurrentWatt>OldWatt)) WattIdx = ii;
            else if ((Mode==MinWatt)&(CurrentWatt<OldWatt)) WattIdx = ii;
          }
        }
        return (WattIdx==0xff)?0xff : WattIdx+1;
      }

      uint8_t ReqFixedPDO(uint8_t SRCPDOIdx_, uint16_t ReqVoltage100mV_, uint16_t ReqCurrentmA_){
        uint8_t res = 0x00;
        ReqPDO_MSG.POD_Index = SRCPDOIdx_;
        ReqPDO_MSG.CURRENT_SEL = Current2Idx(ReqCurrentmA_);
        ReqPDO_MSG.VOLTAGE_SEL = 0;
        //HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::VSELMIN>>8, 1, (uint8_t*), uint16_t Size, uint32_t Timeout)

        HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::PD_REQMSG>>8, 1, (uint8_t*)&ReqPDO_MSG, REG::PD_REQMSG&0xff, 10);
        HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::PD_MSGRLT>>8, 1, &res, REG::PD_MSGRLT&0xff, 100);
        return res;
      }

      void SetVout(bool IsEnable=true){
        uint8_t msg = (IsEnable)? 0x00:0x01;
        HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::SYSTEM>>8, 1, &msg, REG::SYSTEM&0xff, 10);
      }
      
      uint8_t WaitResponse(int Maxloop=50){
        uint8_t res = 0x00;
        for(int ii=0;ii<Maxloop;Maxloop++){
          HAL_Delay(5);
          HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::PD_MSGRLT>>8, 1, &res, REG::PD_MSGRLT&0xff, 100);
          if(res&0x01) break;
        }
        return res;
      }
    };
}
