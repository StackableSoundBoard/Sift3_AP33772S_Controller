#include "AP33772S.hpp"
#include <cstdint>

namespace AP33772S {

  void AP33772S_C::Read_SrcPDO(){
    HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::SRCPDO>>8, 1, (uint8_t*)SrcPDO_List, REG::SRCPDO&0xff, 10);
    for(uint32_t i=0;i<sizeof(SrcPDO_List)/sizeof((SrcPDO_List[0]));i++){
      if(SrcPDO_List[i].Detect) SrcPDO_Num++;
      else return;
    }
  }

  void AP33772S_C::SetVout(bool IsEnable=true){
    uint8_t msg = (IsEnable)? 0x00:0x01;
    HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::SYSTEM>>8, 1, &msg, REG::SYSTEM&0xff, 10);
  }
  
  uint16_t AP33772S_C::Current2Idx(uint16_t ReqCurrentmA_){return ReqCurrentmA_/250-4;};

  uint8_t AP33772S_C::WaitResponse(int Maxloop){
    uint8_t res = 0x00;
    LastCMD_Sucess = true;
    for(int ii=0;ii<Maxloop;ii++){
      HAL_Delay(5);
      HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::PD_MSGRLT>>8, 1, &res, REG::PD_MSGRLT&0xff, 10);
      if(res&0x01) {
        LastCMD_Sucess = true;  
        break;
      }
    }
    return res;
  }

  uint8_t AP33772S_C::FindPDO_Fixed(uint16_t Req_MinVoltage100mV_, uint16_t Req_MaxVoltage100mV_, uint16_t ReqCurrentmA_, PDOFind_Mode Mode){
    uint8_t WattIdx = 0xff;
    for(uint16_t ReqVoltage100mV_=Req_MinVoltage100mV_; ReqVoltage100mV_<=Req_MaxVoltage100mV_; ReqVoltage100mV_++){
      for(int ii=0;ii<8;ii++){
        if(SrcPDO_List[ii].Eval_FixedPDO(ReqVoltage100mV_, ReqCurrentmA_)){
          uint32_t CurrentWatt = SrcPDO_List[ii].MaxCurrent(SrcPDO_List[ii].CurrentMax)*SrcPDO_List[ii].VoltageMax;
          uint32_t OldWatt     = SrcPDO_List[WattIdx].MaxCurrent(SrcPDO_List[WattIdx].CurrentMax)*SrcPDO_List[WattIdx].VoltageMax;
          if(WattIdx==0xff) WattIdx = ii;
          else if ((Mode==MaxWatt)&(CurrentWatt>OldWatt)) WattIdx = ii;
          else if ((Mode==MinWatt)&(CurrentWatt<OldWatt)) WattIdx = ii;
        }
      }
    }
    return (WattIdx==0xff)?0xff : WattIdx+1;
  }
  
  uint8_t AP33772S_C::FindPDO_ADPO(uint16_t Req_MinVoltage100mV_, uint16_t Req_MaxVoltage100mV_, uint16_t ReqCurrentmA_, PDOFind_Mode Mode, uint16_t &ReqVoltage100mV_){
    uint8_t WattIdx = 0xff;
    uint32_t OldWatt = 0;
    ReqVoltage100mV_ = 0;
    for(uint16_t ReqVoltage100mV=Req_MinVoltage100mV_; ReqVoltage100mV<=Req_MaxVoltage100mV_; ReqVoltage100mV++){
      for(int ii=0;ii<14;ii++){
        if(SrcPDO_List[ii].Eval_ADPO(ReqVoltage100mV, ReqCurrentmA_)){
          uint32_t      CurrentWatt = SrcPDO_List[ii].MaxCurrent(SrcPDO_List[ii].CurrentMax)*ReqVoltage100mV;
          if(WattIdx!=0xff) OldWatt = SrcPDO_List[WattIdx].MaxCurrent(SrcPDO_List[WattIdx].CurrentMax)*ReqVoltage100mV;
          if(WattIdx==0xff)                               {WattIdx = ii; ReqVoltage100mV_ = ReqVoltage100mV;}
          else if ((Mode==MaxWatt)&(CurrentWatt>=OldWatt)) {WattIdx = ii; ReqVoltage100mV_ = ReqVoltage100mV;}
          else if ((Mode==MinWatt)&(CurrentWatt<OldWatt)) {WattIdx = ii; ReqVoltage100mV_ = ReqVoltage100mV;}
        }
      }
    }
    return (WattIdx==0xff)?0xff : WattIdx+1;
  }

  
  uint8_t AP33772S_C::ReqFixed(uint8_t SRCPDOIdx_, uint16_t ReqCurrentmA_){
    uint8_t res = 0x00;
    ReqPDO_MSG.POD_Index = SRCPDOIdx_;
    ReqPDO_MSG.CURRENT_SEL = Current2Idx(ReqCurrentmA_);
    ReqPDO_MSG.VOLTAGE_SEL = SrcPDO_List[SRCPDOIdx_-1].VoltageMax;
    //HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::VSELMIN>>8, 1, (uint8_t*), uint16_t Size, uint32_t Timeout)
    
    HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::PD_REQMSG>>8, 1, (uint8_t*)&ReqPDO_MSG, REG::PD_REQMSG&0xff, 10);
    HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::PD_MSGRLT>>8, 1, &res, REG::PD_MSGRLT&0xff, 100);
    return res;
  }
  
  uint8_t AP33772S_C::ReqAPDO(uint8_t SRCPDOIdx_, uint16_t ReqVoltage100mV_, uint16_t ReqCurrentmA_){
    uint8_t res = 0x00;
    ReqPDO_MSG.POD_Index = SRCPDOIdx_;
    ReqPDO_MSG.CURRENT_SEL = Current2Idx(ReqCurrentmA_);
    ReqPDO_MSG.VOLTAGE_SEL = ReqVoltage100mV_;
    //HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::VSELMIN>>8, 1, (uint8_t*), uint16_t Size, uint32_t Timeout)
    
    HAL_I2C_Mem_Write(hi2c, AP33772S_Base_Addr, REG::PD_REQMSG>>8, 1, (uint8_t*)&ReqPDO_MSG, REG::PD_REQMSG&0xff, 10);
    HAL_I2C_Mem_Read(hi2c, AP33772S_Base_Addr, REG::PD_MSGRLT>>8, 1, &res, REG::PD_MSGRLT&0xff, 100);
    return res;
  }
  
  
  int AP33772S_C::FindPDO_Nums(){return SrcPDO_Num;};

};