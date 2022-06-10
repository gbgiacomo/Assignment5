/** Test to Button, ADC and PWM of the Assigment 5
 * 
 *  Bego Giacomo, Longo Mattia - 10/06/2022
 *  
 *  The test is useful to check the peripheral used
 *  in the Assignment 3. When the button 3 is pressed
 *  the system go in the manual mode and then is possible
 *  increase and decrease the light intesity of the LED.
 *  When the button 3 is pressed again the system switch
 *  in the automatic mode where is used the ADC and the 
 *  samples filter. The light intesity of the LED is 
 *  inversely proportional to the enviromental light intensity.
 *
 *  BUTTON 1: Increase the light intensity of the LED
 *  BUTTON 2: Decrease the light intensity of the LED
 *  BUTTON 3: Change the system from automatic to manual mode an viceversa
 *
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

/* Refer to dts file */
#define GPIO0_NID DT_NODELABEL(gpio0) 
#define BOARDBUT1 0xb /* Pin at which BUT1 is connected. Addressing is direct (i.e., pin number) */
#define BOARDBUT2 0xc /* Pin at which BUT2 is connected. Addressing is direct (i.e., pin number) */
#define BOARDBUT3 0x18 /* Pin at which BUT3 is connected. Addressing is direct (i.e., pin number) */

/* Int related declarations */
static struct gpio_callback but1_cb_data; /* Callback structure */
static struct gpio_callback but2_cb_data; /* Callback structure */
static struct gpio_callback but3_cb_data; /* Callback structure */

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
uint16_t highLevel_us=0;

/* Callback function and variables */
volatile bool mode=0;
volatile bool up=0;
volatile bool down=0;

bool state=0;

/* Routine related to the interrupt for mode on/off */
void but1press_cbfunction(){
    printk("Button 1 pressed\n");
    up=1;
}

/* Routine related to the interrupt for increase intensity */
void but2press_cbfunction(){
    printk("Button 2 pressed\n");
    down=1;
}

/* Routine related to the interrupt for decrease intensity */
void but3press_cbfunction(){
    printk("Button 3 pressed\n");
    mode=1;
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

    /* Local vars */
    const struct device *gpio0_dev;         /* Pointer to GPIO device structure */

    int ret=0; 

    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    
    /* check the buttons binding */
    if (gpio0_dev == NULL) {
        printk("Error: Failed to bind to GPIO0\n\r");        
	return;
    }
    else {
        printk("Bind to GPIO0 successfull \n\r");        
    }
    
    /* configuring buttons */
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT1, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 1 \n\r", ret);
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT2, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 2 \n\r", ret);
	return;
    }
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT3, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 3 \n\r", ret);
	return;
    }
    
    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT1, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&but1_cb_data, but1press_cbfunction, BIT(BOARDBUT1));
    gpio_add_callback(gpio0_dev, &but1_cb_data);

    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT2, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&but2_cb_data, but2press_cbfunction, BIT(BOARDBUT2));
    gpio_add_callback(gpio0_dev, &but2_cb_data);

    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT3, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&but3_cb_data, but3press_cbfunction, BIT(BOARDBUT3));
    gpio_add_callback(gpio0_dev, &but3_cb_data);
    

    /* PWM control in function of environment light */
      const struct device *pwm0_dev;          /* Pointer to PWM device structure */
      unsigned int pwmPeriod_us = 1000;       /* PWM period in us */
      ret=0;                              /* Generic return value variable */

      /* Return pointer to device structure with the given name */
      pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
      if (pwm0_dev == NULL) {
          printk("\nError: Failed to bind to PWM0 r");
          return;
      }
    
    while(1){


    /* Manual/Automatic mode */
      if(mode==1 && state==0){
          printk("Switch to MANUAL MODE");
          highLevel_us=0;
          state=1;
          mode=0;
      }
      else if(mode==1 && state==1){
          printk("Switch to AUTOMATIC MODE");
          state=0;
          mode=0;
      }
      k_msleep(5);

    /* Automatic mode */
    if(state==0){

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

          ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
                        pwmPeriod_us,(unsigned int)((pwmPeriod_us*output)/1023), PWM_POLARITY_NORMAL);
          if (ret) {
              printk("Error %d: failed to set pulse width\n", ret);
          return;
          }
      }
      else{
          if(up==1){
              if(highLevel_us<1000){  
                  highLevel_us=highLevel_us+50;
              }
              up=0;
              k_msleep(5);
          }
          else if(down==1){
              if(highLevel_us>0){  
                  highLevel_us=highLevel_us-50;
              }
              down=0;
              k_msleep(5);
          }

          printk("High level period is: %d us \n",highLevel_us);

          ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
                        pwmPeriod_us,(unsigned int)(highLevel_us), PWM_POLARITY_NORMAL);
      }


    // k_msleep(1000);


    }
    return;
} 

        









