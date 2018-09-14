#include "Delay.h"
#include <stdint.h>
#include "hal_types.h"
 /*------------------------------------------------------------------------------- 
Function: delayms(int n);
 
Purpose: delay ms. 
 
Parameters: n.
 
Return: none.
 
Comments: delay nx1 ms.
 
Modified: 
  <Modified by> 
  <Date> 
  <Change> 
--------------------------------------------------------------------------------*/ 
void delayms(uint16 n)
{
    volatile i,j;
    for(i = 0; i < n; i++)
    for(j = 0; j<300; j++);
}

void delayus(uint16 n)
{
    int i,j;  
    for(i = 0; i < n; i++)
    for(j = 0; j<1; j++);
}
