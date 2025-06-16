/**
* @file MB85RCxx.hpp
* @author rooty19 (https://github.com/rooty19)
* @brief Library for MB85RCxxV FeRAM
* @version v1.0
* @date 
* 
* @copyright Copyright (c) Jun, 2025
* 
*/

#pragma once
#include "stm32c0xx_hal.h"
#include <cstdint>

namespace MB85RC {
class MB85RCxxV_C{
  public:
    uint8_t  FeRAM_Base_Addr = 0b1010'0000;
    uint8_t  FeRAM_Dev_Addr  = 0;
    uint16_t FeRAM_Mem_Addr  = 0;
    uint16_t FeRAM_Addr_Size = 1;

    uint8_t  A210 = 0b000;

/**
* @fn 
* @brief Calc I2C and memory Address 
* @param addr_ accsess address
* @param IsWrite Write or Read
*/
    virtual void CalcAddress(uint16_t addr_, bool IsWrite = true)=0;
    template <typename T> void write(uint16_t addr_, T* src_, uint32_t size);
    template <typename T> void read(uint16_t addr_, T* dst_, uint32_t size);    
    
    MB85RCxxV_C(I2C_HandleTypeDef* hi2c_, uint8_t A210_ = 0){
      hi2c = hi2c_;
      A210 = A210_;
    }
  
  private:
  I2C_HandleTypeDef* hi2c;
};

template <typename T> void MB85RCxxV_C::read(uint16_t addr_, T* dst_, uint32_t size){    
  CalcAddress(addr_, false);
  HAL_I2C_Mem_Read(hi2c, FeRAM_Dev_Addr, FeRAM_Mem_Addr, FeRAM_Addr_Size, (uint8_t*)dst_, size, HAL_MAX_DELAY);
}

template <typename T> void MB85RCxxV_C::write(uint16_t addr_, T* src_, uint32_t size){
  // memo: __attribute__((__packed__)) = sizeof(struct)はパディング領域が増えたりするのでパディングなしの指定
  CalcAddress(addr_, false);
  HAL_I2C_Mem_Write(hi2c, FeRAM_Dev_Addr, FeRAM_Mem_Addr, FeRAM_Addr_Size, (uint8_t*)src_, size, HAL_MAX_DELAY);
}

class MB85RC04V_C: public MB85RCxxV_C{
  public:
  using MB85RCxxV_C::MB85RCxxV_C;
  void CalcAddress(uint16_t addr_, bool IsWrite = true){
    FeRAM_Dev_Addr = FeRAM_Base_Addr | ((A210<<1)& 0b0000'1110) | ((addr_>>8)&0x01);
    FeRAM_Mem_Addr = addr_&0x00ff;
  }
};
}; // End of Namespace