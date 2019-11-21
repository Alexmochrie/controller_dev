#include "driverlib.h"
#include "Board.h"

//Defines
#define COMPARE_VALUE 50000

#define LCD_PORT GPIO_PORT_P7
#define LCD_EN GPIO_PIN4
#define LCD_RS GPIO_PIN5
#define LCD_DATA 0xf

//Global Variables
uint8_t RXData1, RXData2 = 0;       //UART Data
uint8_t uart_ready = 0;             //Condition variable for data processing
uint16_t adc_x, adc_y = 0;           //X and Y axis readings from joystick
uint8_t sw_a, sw_b, sw_c = 0;       //Pushbuttons
uint8_t control_o = 0;

//INIT Functions
void init_GPIO(void);
void init_TIMER(void);
void init_UART(void);
void init_ADC(void);
void init_LCD(void);

//Math/operation Functions
void lcd_char(char data);
void lcd_command(char cmd);
void lcd_string(char *str);

uint8_t ADC_stick(char axis);
void delay_ms(int ms);

void main (void)
{
    //Stop Watchdog Timer
    WDT_A_hold(WDT_A_BASE);

    //Disables high impedence
    PMM_unlockLPM5();

    //Change clock

    //Initialize peripherals
    init_GPIO();
    init_TIMER();
    init_UART();
    init_ADC();
    //init_LCD();

    //]Turn off buzzer
    GPIO_setAsInputPin(
           GPIO_PORT_P1,
           GPIO_PIN7
       );

    //Enable interrupts before main loop
    __enable_interrupt();

    while (1)
    {
        adc_x = ADC_stick('x');
        adc_y = ADC_stick('y');
        if(uart_ready == 2)
        {
            P5OUT |= GPIO_PIN4; //Set LED to show we're getting data
            //Read both joystick axes
            adc_x = ADC_stick('x');
            adc_y = ADC_stick('y');

            //TODO: Update LCD from RX data + sticks
            //TODO: Update LEDs from RX data
            P5OUT = (P5OUT & 0xcf)  | ((RXData2 & 0xc0) >> 2);    //Set LEDs

//            if( (RXData & 0x1f) < 20)
//            {
//                //Set the pin as input to shut the speaker off
//                GPIO_setAsInputPin(
//                       GPIO_PORT_P1,
//                       GPIO_PIN7
//                   );
//            }
//            else    //Set new duty cycle & re-enable output
//            {
//                GPIO_setAsOutputPin(
//                       GPIO_PORT_P1,
//                       GPIO_PIN7
//                   );
//                TA0CCR1 = RXData & 0x1f;
//                P1SEL0 |= BIT7;
//            }

            control_o = (P6IN & 0x03);  //Read switches

            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, adc_x);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, adc_y);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, control_o);
            uart_ready = 0;
            P5OUT &= !GPIO_PIN4;    //Clear data RX led
        }
    }

}


#pragma vector=USCI_A0_VECTOR
__interrupt void EUSCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            //Store the incoming data and flag that we're ready to pack new data
            if(uart_ready == 0)
            {
                RXData1 = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
                uart_ready++;
            }
            else if(uart_ready == 1)
            {
                RXData2 = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
                uart_ready++;
            }
            break;
       case USCI_UART_UCTXIFG: break;
       case USCI_UART_UCSTTIFG: break;
       case USCI_UART_UCTXCPTIFG: break;
    }
}

void delay_ms(int ms)
{
    uint8_t i;
    for(i = ms; i>0; i--)
        __delay_cycles(1000);
}


void lcd_char(char data)
{
    char temp = data;
    P7OUT = 0x1;
    delay_ms(5);
    data = ( (data & 0xF0) >> 4) | 0x30;  //Write Upper Nibble (RS=1) E --> 1
    P7OUT = data;
    delay_ms(5);
    data ^= 0x30;//  E --> 0
    P7OUT = data;
    delay_ms(5);
    data = temp;
    data = ( (data & 0x0F) ) | 0x30;//Write Lower Nibble (RS=1) E --> 1
    P7OUT = data;
    delay_ms(5);
    data ^= 0x30;//E --> 0
    P7OUT = data;
    delay_ms(7);
}

void lcd_command(char cmd)
{
    char temp = cmd;
    P7OUT=0;
    delay_ms(5);
    cmd = ( (cmd & 0xF0) >> 4) | 0x20;//Write Upper Nibble (RS=0) E --> 1
    P7OUT = cmd;
    delay_ms(5);
    cmd ^= 0x20;//E --> 0
    P7OUT = cmd;
    delay_ms(5);
    cmd = temp;
    cmd = ( (cmd & 0x0F) ) | 0x20;//Write Lower Nibble (RS=0) E --> 1
    P7OUT = cmd;
    delay_ms(5);
    cmd ^= 0x20;//E --> 0
    P7OUT = cmd;
    delay_ms(7);
}

void lcd_string(char *str)
{
    while(*str != '\0')
    {
        lcd_char(str++);
    }
}

void init_LCD(void)
{
    P7DIR = 0x3f;       //P5:0 = outputs
    lcd_command(0x33);  //Initialize LCD Driver
    lcd_command(0x32);  //Four bit mode
    lcd_command(0x2C);  //2 Line Mode
    lcd_command(0x0C);  //Display On, Cursor Off, Blink Off  Change to 0x0F if cursor is desired
    lcd_command(0x01);  //Clear Screen, Cursor Home
}

void init_GPIO(void)
{
    //Set LED1 as an output pin.
//    GPIO_setAsOutputPin(
//        GPIO_PORT_LED2,
//        GPIO_PIN_LED2
//    );

//TODO: Enable for PCB

    //Set LED1 as an output pin.
//    GPIO_setAsOutputPin(
//        GPIO_PORT_P1,
//        GPIO_PIN0
//    );

    //Prove 1Mhz clock
//    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);
//    GPIO_setAsPeripheralModuleFunctionOutputPin(
//            GPIO_PORT_P8,
//            GPIO_PIN0,
//            GPIO_FUNCTION_SMCLK
//        );

//    //Set LED1 as an output pin.
//    GPIO_setAsOutputPin(
//        GPIO_PORT_P5,
//        GPIO_PIN5
//    );
//
//    //Set LED2 as an output pin.
//    GPIO_setAsOutputPin(
//        GPIO_PORT_P5,
//        GPIO_PIN4
//    );
//    //Set SW0 as an input pin.
//    GPIO_setAsInputPin(
//        GPIO_PORT_P6,
//        GPIO_PIN0
//    );

//    //Set SW1 as an input pin.
//    GPIO_setAsInputPin(
//        GPIO_PORT_P6,
//        GPIO_PIN1
//    );

//    //Set SW2 as an input pin.
//    GPIO_setAsInputPin(
//        GPIO_PORT_P6,
//        GPIO_PIN2
//    );

    //Set LCD pins to outputs
    P7DIR = 0x3f;


}

void init_TIMER(void)
{

    //Set P1.7 as an output pin for pwm
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN7
    );
    //Sets P1.7 to Timer Output
    P1SEL0 |= BIT7;
    TA0CCR0 = 128;                             // PWM Period/2 - Change to increase resolution
    TA0CCTL1 = OUTMOD_6;                       // TACCR1 toggle/set
    TA0CCR1 = 64;                              // TACCR1 PWM duty cycle
    TA0CTL = TASSEL_1 | MC_3;                  // ACLK, up-down mode
}

void init_UART(void)
{
    //Configure UART pins

    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_UCA0TXD,
        GPIO_PIN_UCA0TXD,
        GPIO_FUNCTION_UCA0TXD
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCA0RXD,
        GPIO_PIN_UCA0RXD,
        GPIO_FUNCTION_UCA0RXD
    );

    //Configure UART
    //SMCLK = 1.048MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 13, UCBRSx = 34, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 6;
    param.firstModReg = 13;
    param.secondModReg = 34;
    param.parity = EUSCI_A_UART_ODD_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param)) {
            return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
        EUSCI_A_UART_RECEIVE_INTERRUPT);

   //Enable USCI_A0 RX interrupt
   EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
       EUSCI_A_UART_RECEIVE_INTERRUPT);
}

void init_ADC(void)
{
    //Sets pins to ADC mode
    SYSCFG2 |= ADCPCTL6 | ADCPCTL5;

    ADC_init(ADC_BASE,
        ADC_SAMPLEHOLDSOURCE_SC,
        ADC_CLOCKSOURCE_ADCOSC,
        ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    ADC_setupSamplingTimer(ADC_BASE,
            ADC_CYCLEHOLD_16_CYCLES,
            ADC_MULTIPLESAMPLESDISABLE);

    ADC_configureMemory(ADC_BASE,
            ADC_INPUT_A5,
            ADC_VREFPOS_AVCC,
            ADC_VREFNEG_AVSS);

    ADCCTL2 &= 0xffc7; //Set resolution to 8-bit and unsigned

    ADC_clearInterrupt(ADC_BASE,
            ADC_COMPLETED_INTERRUPT);
}

/*Reads either X or Y axis potentiometer
 * Returns 8-bit value of selected axis
 * Toggles active channel each call and polls busy flag before reading
 */
uint8_t ADC_stick(char axis)
{
    uint16_t adc_temp = 0;
    ADC_disableConversions(ADC_BASE, false);
    if(axis == 'x')
    {
        ADC_configureMemory(ADC_BASE,
                      ADC_INPUT_A6,
                      ADC_VREFPOS_AVCC,
                      ADC_VREFNEG_AVSS);
    }
    else if (axis == 'y')
    {
        ADC_configureMemory(ADC_BASE,
                     ADC_INPUT_A5,
                     ADC_VREFPOS_AVCC,
                     ADC_VREFNEG_AVSS);
    }

    //Gets an averaged ADC result
    for(int i = 0; i<=3; i++)
    {
        ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        while(ADC_isBusy(ADC_BASE));
        adc_temp += (ADC_getResults(ADC_BASE) >> 2);
    }
    return(adc_temp);
}



