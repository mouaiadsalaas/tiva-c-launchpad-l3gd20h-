#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/watchdog.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/eeprom.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "inc/hw_hibernate.h"
#include "inc/hw_gpio.h"
#include "driverlib/hibernate.h"
#include "inc/hw_uart.h"



#define AVERAGE_WINDOW_SIZE                  ((uint32_t) 10u)
#define CALIBRATION_BUFFER_LENGTH            ((uint32_t) 2000u)
#define L3GD20_SENSITIVITY                   ((float)0.07)

#define X_LOW     0x28
#define X_HIGH    0x29
#define Y_LOW     0x2A
#define Y_HIGH    0x2B
#define Z_LOW     0x2C
#define Z_HIGH    0x2D

#define Write_Data 0x00
#define Read_Data  0x80
#define NUM_SSI_DATA    6

uint8_t ui32buf_Tx[NUM_SSI_DATA]={0x28,0x29,0x2A,0x2B,0x2C,0x2D};
uint8_t ui32Index;

uint16_t ui32buf_Rx[6];
uint32_t count_buffer;
unsigned char TX_BUFFER[] ;



void delayMS ( int ms ) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}


//void SPI_GYRO_READ( ){
//
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);
//
//
//    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
//    {
//
//    SSIDataPut(SSI2_BASE,(Read_Data|ui32buf_Tx[ui32Index])); delayMS(20);
//    while(SSIBusy(SSI2_BASE)){}
//
//    SSIDataGet(SSI2_BASE,(uint32_t*)&ui32buf_Rx[ui32Index]); delayMS(20);
//    while(SSIBusy(SSI2_BASE)){}
//
//    SSIDataPut(SSI2_BASE,Write_Data); delayMS(20);
//    while(SSIBusy(SSI2_BASE)){}
//
//
//    }
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
//
//
//
//}


//void Send_Data(unsigned char data) {
//
//    UARTCharPutNonBlocking(UART1_BASE, data);
//    //UARTCharPutNonBlocking(UART1_BASE, TX_BUFFER[1]);
//    UARTCharPutNonBlocking(UART1_BASE, '\n');
//
//}
void sendUartString(uint32_t base, const char * data)
{
    while (*data != '\0')
    {
        if (UARTSpaceAvail(base))
        {
            while (!UARTCharPutNonBlocking(base, *data));

            data++;
        }
    }
}

void GYRO_READ_X_LOW( ){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE,(Read_Data|0x28)); delayMS(20);


    SSIDataGet(SSI2_BASE,(uint32_t*)&ui32buf_Rx[0]);
    while(SSIBusy(SSI2_BASE)){}

    SSIDataPut(SSI2_BASE,Write_Data);
    while(SSIBusy(SSI2_BASE)){}

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}

void GYRO_READ_X_HIGH( ){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE,(Read_Data|0x29)); delayMS(20);

    SSIDataGet(SSI2_BASE,(uint32_t*)&ui32buf_Rx[1]);
    while(SSIBusy(SSI2_BASE)){}

    SSIDataPut(SSI2_BASE,Write_Data);
    while(SSIBusy(SSI2_BASE)){}

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}


void GYRO_READ_Y_LOW( ){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE,(Read_Data|0x2a)); delayMS(20);

    SSIDataGet(SSI2_BASE,(uint32_t*)&ui32buf_Rx[2]);
    while(SSIBusy(SSI2_BASE)){}

    SSIDataPut(SSI2_BASE,Write_Data);
    while(SSIBusy(SSI2_BASE)){}

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}

void GYRO_READ_Y_HIGH( ){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE,(Read_Data|0x2b)); delayMS(20);

    SSIDataGet(SSI2_BASE,(uint32_t*)&ui32buf_Rx[3]);
    while(SSIBusy(SSI2_BASE)){}

    SSIDataPut(SSI2_BASE,Write_Data);
    while(SSIBusy(SSI2_BASE)){}

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}
void SPI_REGISTER_READ( int8_t address){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE,(Read_Data|address));

    while(SSIBusy(SSI2_BASE)){}

    SSIDataGet(SSI2_BASE,(uint32_t*)&ui32buf_Rx);
    while(SSIBusy(SSI2_BASE)){}

    SSIDataPut(SSI2_BASE,Write_Data);
    while(SSIBusy(SSI2_BASE)){}

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);



}

void SPI_REGISTER_WRITE( int8_t address, int8_t Value){

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, !GPIO_PIN_2);

    SSIDataPut(SSI2_BASE,(Write_Data|address)); delayMS(100);
    while(SSIBusy(SSI2_BASE)){}


    SSIDataPut(SSI2_BASE,Value);
    while(SSIBusy(SSI2_BASE)){}

    while(SSIBusy(SSI2_BASE)){}

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}

void IntUart ( void ){

    uint32_t ui32Status;
    int32_t rcv_ch;

    ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);

    rcv_ch = UARTCharGetNonBlocking( UART1_BASE );
}

void L3GD20_Init(void){

    SPI_REGISTER_WRITE(0x20, 0x0F);
    delayMS(100);
    SPI_REGISTER_WRITE(0x23, 0x30);
    delayMS(100);
}
//mouaiad
//ali
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);


    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure( GPIO_PC5_U1TX);
    GPIOPinConfigure( GPIO_PC4_U1RX);
    GPIOPinTypeUART( GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART1);
    UARTDisable( UART1_BASE);
    UARTClockSourceSet( UART1_BASE, UART_CLOCK_PIOSC );
    UARTConfigSetExpClk( UART1_BASE, 16000000, 115200 , UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE );
    UARTIntRegister( UART1_BASE, IntUart );
    UARTIntEnable( UART1_BASE, UART_INT_RX | UART_INT_RT );
    UARTEnable( UART1_BASE);
    IntEnable( INT_UART1);



    /* SPI */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //CS
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 , !GPIO_PIN_2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    //GPIOPinConfigure(GPIO_PH5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PH7 - SSI2Tx
    //      PH6 - SSI2Rx
    //      PH5 - SSI2Fss
    //      PH4 - SSI2CLK

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7 |GPIO_PIN_6 | GPIO_PIN_4);

    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI2_BASE);

    L3GD20_Init();

    char send[100];
    while(1){

  // SPI_GYRO_READ();
        GYRO_READ_X_LOW();
        GYRO_READ_X_HIGH();
        GYRO_READ_Y_LOW();
        GYRO_READ_Y_HIGH();

        sprintf(send, "X=%d, Y=%d, Z=%d \r\n",ui32buf_Rx[0],ui32buf_Rx[2],ui32buf_Rx[4]);
        sendUartString(UART1_BASE,send);




    }
}


