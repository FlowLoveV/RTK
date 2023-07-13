//
// Created by 0-0 mashuo on 2023/3/25.
//

#ifndef RTK_BITOPERATION_H
#define RTK_BITOPERATION_H

// 此文件为bit操作头文件
#define SET_BIT1(x,bit) ( x |= (1<<bit) )
#define SET_BIT0(x,bit) ( x &= (0<<bit) )
#define GET_BIT(x,bit)  ( 1 & (x>>bit)  )



#endif //RTK_BITOPERATION_H
