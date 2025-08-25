/**
* @file AP33772S.cpp
* @author StackableSoundBoard project. (https://github.com/StackableSoundBoard)
* @brief Library source for AP33772 USB-PD controller
* @version v1.0
* @date 
* 
* @copyright Copyright (c) Aug, 2025, Released under the MIT license 
* 
*/
#include "AP33772S.hpp"

namespace AP33772S {

  void AP33772S_C::Read_SrcPDO(bool EPR_Flag_){
    EPR_Flag = EPR_Flag_;
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
      for(int ii=0;ii<14;ii++){
        uint16_t VoltCoef = ((ii>=7)&EPR_Flag)? 2:1; // if EPR Voltage step is 200mV
        if(SrcPDO_List[ii].Eval_FixedPDO(ReqVoltage100mV_, ReqCurrentmA_, VoltCoef)){
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
        uint16_t VoltCoef = ((ii>=8)&EPR_Flag)? 2:1; // if EPR Voltage step is 200mV
        if(SrcPDO_List[ii].Eval_ADPO(ReqVoltage100mV, ReqCurrentmA_, VoltCoef)){
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
    RequestedPDOIdx = SRCPDOIdx_;
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
    RequestedPDOIdx = SRCPDOIdx_;
    return res;
  }
  
  
  int AP33772S_C::FindPDO_Nums(){return SrcPDO_Num;};
  uint8_t AP33772S_C::RequestedPDO_Idx(){
    if(LastCMD_Sucess & (RequestedPDOIdx!=0xff)) return RequestedPDOIdx-1;
    else{
      RequestedPDOIdx =0xff;
      return 0xff;
    }
  }
};