#ifndef DW1000_H_
#define DW1000_H_

#include "stm32f10x.h"

uint8_t dw1000_init(void);
uint8_t dw1000_distn_poll(void);    //请求端,不能计算数据
uint8_t dw1000_distn_resp(void);    //相应端,能计算数据

#endif