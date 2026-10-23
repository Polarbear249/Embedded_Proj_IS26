
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define IR_AOUT_PIN      26     //Using GP26 as AOUT
#define ADC_CHANNEL      0      //Define constant for the ADC channel. GP26 correspondes to ADC Channel 0 (Known thru the documentations)
#define SENSOR_DELAY_MS  500    //Delay of 500ms per sensor reading

int
main(void)
{
    uint16_t analog_reading = 0;    //Declare 16bit uint to store the ADC reading ()

    stdio_init_all();               //initialise all standard I/O operations so printf able to be sent thru USB Serial connections
    
    adc_init();                     //initalise ADC hardware inside Raspberry PI Pico
    adc_gpio_init(IR_AOUT_PIN);     //Configure specific GP26, to prepare for analog input rather than standard digital I/O (This is being used to see how "black" the line is)
    adc_select_input(ADC_CHANNEL);  //Actively listen to channel 0

    while (1)
    {
        analog_reading = adc_read(); //read the raw analog voltage from IR Sensor and stores in a var "analog_reading"
        
        printf("IR Sensor Analog Value: %d\n", analog_reading); //send text and int as decimal, and send to the computer serial monitor
        
        sleep_ms(SENSOR_DELAY_MS);  //Pause for 500ms to avoid sending too much data (While this is just a test, best to not overload an potentially cause any destruction to the Rasberry Pico)
    }

    return 0;                       //Indicating successful program execution
}

