/** @file main.c
 * @brief Contains the the implementation of the Assignment4
 * 
 * @author Mattia Longo and Giacomo Bego
 * @date 31 May 2022
 * @bug No known bugs
 */


#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>

/* PWM pin initialization */
#define PWM0_NID DT_NODELABEL(pwm0) 
#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)
#define STACK_SIZE 1024

/*ADC definitions and includes*/
#include <hal/nrf_saadc.h>
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1 
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define BUFFER_SIZE 1
#define SIZE 10 /* Size of the vector to filter */


/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};
struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];
static uint16_t sample;
static uint16_t output;

/* Vector to filter declaration */
uint16_t samples[SIZE]={0,0,0,0,0,0,0,0,0,0};
uint16_t filteredSamples[SIZE]={0,0,0,0,0,0,0,0,0,0};
int8_t index=-1;
uint16_t average=0;
uint16_t upperLevel=0;
uint16_t lowerLevel=0;


/* ISR of the interrupt*/
void but1press_cbfunction(){
    //printk("UP product\n"); // button 1 hit !    
    sw=1;
}


static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}



void main(void) {
    
    int err=0;

    while(1){

      /* ADC setup: bind and initialize */
      adc_dev = device_get_binding(DT_LABEL(ADC_NID));
  	if (!adc_dev) {
          printk("ADC device_get_binding() failed\n");
      } 
      err = adc_channel_setup(adc_dev, &my_channel_cfg);
      if (err) {
          printk("adc_channel_setup() failed with error code %d\n", err);
      }

      err=0;

      /* Get one sample, checks for errors and prints the values */
      err=adc_sample();
      if(err) {
          printk("adc_sample() failed with error code %d\n\r",err);
      }
      else {
          if(adc_sample_buffer[0] > 1023) {
              printk("adc reading out of range\n\r");
              sample=0;  /* Safety value */
          }
          else {
              sample=adc_sample_buffer[0];
              printk(" sample is : %4u\n",sample);
           }
      }


      /* Filter implementation */

      if(index<SIZE-1){
          index++;
      }
      else{
        index=0;
      }

      samples[index]=sample;
	
      /* Average calculation */
       int32_t sum=0;
	
      for(int8_t i=0;i<10;i++){
  	 sum+=samples[i];
      }
      average=sum/10;
	
      /* Samples value limits */
      upperLevel=average*1.1;
      lowerLevel=average*0.9;

      uint16_t j=0;
	
      for(uint8_t i=0;i<10;i++){
          if(samples[i]>=lowerLevel && samples[i]<=upperLevel){
              filteredSamples[j]=samples[i];
              j++;
          }
      }
         
      /* Print the vector samples filtered */
      printk("The filtered samples are: ");
      for(uint8_t i=0;i<10;i++){
          printk("\t %d",filteredSamples[i]);
      }
      printk("\n\n");
          
      


      uint16_t sum2=0;
	
      for(uint16_t a=0;a<j;a++){
          sum2+=filteredSamples[a];
      }
         
      //condition to avoid a zero denominator
      if(j>0){
          output=sum2/j;
      }


      /* PWM control in function of environment light */

      const struct device *pwm0_dev;          /* Pointer to PWM device structure */
      unsigned int pwmPeriod_us = 1000;       /* PWM period in us */
      int ret=0;                              /* Generic return value variable */

      /* Return pointer to device structure with the given name */
      pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
      if (pwm0_dev == NULL) {
          printk("\nError: Failed to bind to PWM0 r");
          return;
      }

      ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)((pwmPeriod_us*output)/1023), PWM_POLARITY_NORMAL);
      if (ret) {
       printk("Error %d: failed to set pulse width\n", ret);
          return;
      }
       

    // k_msleep(1000);


    }
    return;
} 

        









