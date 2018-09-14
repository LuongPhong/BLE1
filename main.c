/**
  @file  main.c
  @brief main entry of the BLE stack sample application.

  <!--
  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ``AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
  -->
*/
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/UART.h>
#include "Delay.h"
#include <xdc/runtime/Error.h>

#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/BIOS.h>

#include "ICall.h"
#include "bcomdef.h"
#include "peripheral.h"
#include "simpleBLEPeripheral.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "bleUserConfig.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

/**
 * Exception handler
 */
void exceptionHandler()
{
    volatile uint8_t i = 1;
    while(i){}
}

#ifdef FEATURE_OAD
#if defined(__IAR_SYSTEMS_ICC__)
extern uint32_t __vector_table;
#elif defined (__TI_COMPILER_VERSION__)
extern uint32_t ti_sysbios_family_arm_m3_Hwi_resetVectors;
#endif //Compiler
#endif //FEATURE_OAD

/*
 *  ======== main ========
 */
//Task_Struct myTask;
//Char myTaskStack[512];
///* Global memory storage for a PIN_Config table */
// PIN_State buttonPinState;
// PIN_State ledPinState;
//extern uint8 blink;
UART_Handle uart;
UART_Params uartParams;
//
//
//
//#define Board_LED0 IOID_6
//#define Board_LED1 IOID_7
//#define Board_BUTTON0 IOID_13
//#define Board_BUTTON1 IOID_14
//PIN_Config ledPinTable[] = {
//    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//
//    PIN_TERMINATE
//};
//PIN_Config LedPinTable_BLINK[] =
//{
//    Board_LED1    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off */
//    PIN_TERMINATE                                                                      /* Terminate list */
//};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
//PIN_Config buttonPinTable[] = {
//    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
//    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
//    PIN_TERMINATE
//};
//static void taskFxn(UArg a0, UArg a1)
//{
//    /* Locals */
//    PIN_State pinState;
//    PIN_Handle hPin;
//    uint_t currentOutputVal;
//    uint32_t standbyDurationUs = 500000;
//    uint8 dem = 0;
//    /* Allocate LED pins */
//    hPin = PIN_open(&pinState, LedPinTable_BLINK);
//
//    /* Loop forever */
//    while(TRUE && dem <10 )
//    {
//        /* Sleep, to let the power policy transition the device to standby */
//        Task_sleep(standbyDurationUs / Clock_tickPeriod);
//        /* Read current output value for all pins */
//        currentOutputVal =  PIN_getPortOutputValue(hPin);
//        /* Toggle the LEDs, configuring all LEDs at once */
//        PIN_setPortOutputValue(hPin, ~currentOutputVal);
//    }
//}
PIN_Handle buttonPinHandle;
 PIN_Handle ledPinHandle;
 uint_t currentOutputVal;
 uint_t currentOutputVal_LINK_LOSS;

/* Global memory storage for a PIN_Config table */
PIN_State buttonPinState;
PIN_State ledPinState;
uint8 gia_tri = 0;
uint8 check;
uint8 count_blink = 0;
uint8 count_LINK_LOSS = 0;
#define Board_LED0 IOID_6
#define Board_LED1 IOID_7
#define Board_BUTTON0 IOID_13
#define Board_BUTTON1 IOID_14


PIN_Config ledPinTable_KEY_PRESS[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_Config ledPinTable_LINK_LOSS[] = {
//    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_Config buttonPinTable_KEY_PRESS[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    uint32_t currVal = 0;
    uint8 check=0;


//    delayms(5000);
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);
    if (!PIN_getInputValue(pinId)) {
        /* Toggle LED based on the button pressed */
        switch (pinId) {
            case Board_BUTTON0:
                delayms(500);
                check = PIN_getInputValue(Board_BUTTON0);
                if( check==0)//long key
                {
                gia_tri = 0x02;
                currVal =  PIN_getOutputValue(Board_LED0);
                PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
                uint8 advertEnabled = TRUE; // Turn on Advertising
                              GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled);
                }
                if( check==1)//short key
                {
                  currVal =  PIN_getOutputValue(Board_LED0);
                  PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
                  gia_tri = 0x08;
                  uint8 advertEnabled = TRUE; // Turn on Advertising
                                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled);

                }
//                print_number(currVal);
//                }

                break;
            default:
                /* Do nothing */
                break;
        }
    }
}
extern uint8 disconnect;

//void disconnection(uint8 a)
//{
//    a = disconnect;
//    if(a == 0x02)
//    {
//    uint8 advertEnabled = FALSE; // Turn on Advertising
//// Disable connectable advertising.
////    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled);
//    GAPRole_TerminateConnection();
//     }
//}
extern uint8 advertEnabled;
int main()
{
    advertEnabled = TRUE;
//  Task_Params taskParams;
  PIN_init(BoardGpioInitTable);
//  disconnection(disconnect);
//  uint8 en;
//  while(1){
//
//  Task_sleep(10000);
//  en=1;
//  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, 1, &en ); // 1=on
//  }
  ledPinHandle = PIN_open(&ledPinState, ledPinTable_KEY_PRESS);
  buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable_KEY_PRESS);
  PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn);
  UART_Params_init(&uartParams);
  uartParams.writeDataMode = UART_DATA_BINARY;
  uartParams.readDataMode = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readEcho = UART_ECHO_OFF;
  uartParams.baudRate = 9600;
  uart = UART_open(Board_UART, &uartParams);


#ifndef POWER_SAVING
    /* Set constraints for Standby, powerdown and idle mode */
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
#endif // POWER_SAVING
    
    /* Initialize ICall module */
    ICall_init();

    /* Start tasks of external images - Priority 5 */
    ICall_createRemoteTasks();
    
    /* Kick off profile - Priority 3 */
    GAPRole_createTask();
    
    SimpleBLEPeripheral_createTask();

#ifdef FEATURE_OAD
    {
      uint8_t counter;
      uint32_t *vectorTable =  (uint32_t*) 0x20000000;
#if defined(__IAR_SYSTEMS_ICC__)
      uint32_t *flashVectors = &__vector_table;
#elif defined(__TI_COMPILER_VERSION__)
      uint32_t *flashVectors = &ti_sysbios_family_arm_m3_Hwi_resetVectors;
#endif //Compiler.
      
      // Write image specific interrupt vectors into RAM vector table.
      for(counter = 0; counter < 15; ++counter)
      {
        *vectorTable++ = *flashVectors++;
      }
    }
#endif //FEATURE_OAD
    
    /* enable interrupts and start SYS/BIOS */
    BIOS_start();
    
    return 0;
}

/**
 * Error handled to be hooked into TI-RTOS
 */
Void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

/**
 * HAL assert handler required by OSAL memory module.
 */
void halAssertHandler(void)
{
  for (;;);
}
