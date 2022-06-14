/** Implementation of the manual mode of the Assigment 5
 * 
 *  Bego Giacomo, Longo Mattia - 14/06/2022
 *  
 *  Description...
 *
 *  Manual mode:
 *    - BUTTON 1: Increase the light intensity of the LED
 *    - BUTTON 2: Decrease the light intensity of the LED
 *    - BUTTON 3: Change the system from automatic to manual mode an viceversa
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
#include <hal/nrf_saadc.h> /* ADC includes */


/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_MANUAL_prio 1
#define thread_AUTOMATIC_prio 1
#define thread_FILTER_prio 1
#define thread_PWM_prio 1

/* First therad periodicity (in ms)*/
#define thread_A_period 30

#define STACK_SIZE 1024

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_MANUAL_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_AUTOMATIC_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_FILTER_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_PWM_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_MANUAL_data;
struct k_thread thread_AUTOMATIC_data;
struct k_thread thread_FILTER_data;
struct k_thread thread_PWM_data;

/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_MANUAL_tid;
k_tid_t thread_AUTOMATIC_tid;
k_tid_t thread_FILTER_tid;
k_tid_t thread_PWM_tid;

/* Semaphores for task synch */
struct k_sem sem_a2manual;
struct k_sem sem_a2automatic;
struct k_sem sem_auto2filter;
struct k_sem sem_manFilter2pwm;

/* Thread code prototypes */
void thread_A_code(void *,void *,void *);
void thread_MANUAL_code(void *,void *,void *);
void thread_AUTOMATIC_code(void *,void *,void *);
void thread_FILTER_code(void *,void *,void *);
void thread_PWM_code(void *,void *,void *);

/* ADC definitions */
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1 
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define BUFFER_SIZE 1
#define SIZE 10 /* Size of the vector to filter */

/* PWM pin initialization */
#define PWM0_NID DT_NODELABEL(pwm0)
#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)
//#define BOARDLED_PIN 0x0d


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
static uint16_t highLevel_us;


/* Vector to filter declaration */
uint16_t samples[SIZE]={0,0,0,0,0,0,0,0,0,0};
uint16_t filteredSamples[SIZE]={0,0,0,0,0,0,0,0,0,0};
int8_t index=-1;
uint16_t average=0;
uint16_t upperLevel=0;
uint16_t lowerLevel=0;
uint16_t pwmPeriod_us = 1000;       /* PWM period in us */

/* Time variable declaration */
uint8_t hours;
uint8_t minutes;
uint8_t seconds;
uint8_t days;

uint16_t threshold=500; /* Luce che te orba!!! */

/* Callback function and variables */
volatile bool mode=0;
volatile bool up=0;
volatile bool down=0;

bool state=0;

/* Routine related to the interrupt for mode on/off */
void but1press_cbfunction(){
    //printk("Button 1 pressed\n");
    up=1;
}

/* Routine related to the interrupt for increase intensity */
void but2press_cbfunction(){
    //printk("Button 2 pressed\n");
    down=1;
}

/* Routine related to the interrupt for decrease intensity */
void but3press_cbfunction(){
    //printk("Button 3 pressed\n");
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
    
    int ret=0;

    /* Local vars */
    const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
    
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

    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT3, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&but3_cb_data, but3press_cbfunction, BIT(BOARDBUT3));
    gpio_add_callback(gpio0_dev, &but3_cb_data);


    /* Create and init semaphores */
    k_sem_init(&sem_a2manual, 0, 1);
    k_sem_init(&sem_a2automatic, 0, 1);
    k_sem_init(&sem_manFilter2pwm, 0, 1);
    k_sem_init(&sem_auto2filter, 0, 1);
    
    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
        K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
        NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_MANUAL_tid = k_thread_create(&thread_MANUAL_data, thread_MANUAL_stack,
        K_THREAD_STACK_SIZEOF(thread_MANUAL_stack), thread_MANUAL_code,
        NULL, NULL, NULL, thread_MANUAL_prio, 0, K_NO_WAIT);

    thread_AUTOMATIC_tid = k_thread_create(&thread_AUTOMATIC_data, thread_AUTOMATIC_stack,
        K_THREAD_STACK_SIZEOF(thread_AUTOMATIC_stack), thread_AUTOMATIC_code,
        NULL, NULL, NULL, thread_AUTOMATIC_prio, 0, K_NO_WAIT);

    thread_FILTER_tid = k_thread_create(&thread_FILTER_data, thread_FILTER_stack,
        K_THREAD_STACK_SIZEOF(thread_FILTER_stack), thread_FILTER_code,
        NULL, NULL, NULL, thread_FILTER_prio, 0, K_NO_WAIT);

    thread_PWM_tid = k_thread_create(&thread_PWM_data, thread_PWM_stack,
        K_THREAD_STACK_SIZEOF(thread_PWM_stack), thread_PWM_code,
        NULL, NULL, NULL, thread_PWM_prio, 0, K_NO_WAIT);
    
    return;
} 

        
void thread_A_code(void *argA,void *argB,void *argC)
{

    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    printk("Thread A init (periodic)\n");

    while(1){
        
        /* 
         * 1- Set the date by terminal
         * 2- Increase the time and the date periodicaly
         * 3- Switch from one mode to other one (automatic ---> manual / manual ---> automatic)
         * 
         */

        printk("Task A at time: %lld ms\t\t",k_uptime_get());        
        
        /* Timer implementation */
        seconds++;
        if(seconds==60){
            seconds=0;
            minutes+=1;
        }

        if(minutes==60){
            minutes=0;
            hours+=1;
        }

        if(hours==24){
            hours=0;
            days+=1;
        }
        /* Print the time */
        printk("DAY: %d\t",days);
        printk("TIME: %d:%d:%d\n",hours,minutes,seconds);


        /* Manual/Automatic mode */
        if(mode==1 && state==0){
            printk("Switch to MANUAL MODE\n");
            /* Reset the variables */
            highLevel_us=0;
            up=0;
            down=0;

            state=1;
            mode=0;
        }
        else if(mode==1 && state==1){
            printk("Switch to AUTOMATIC MODE\n");
            highLevel_us=0;
            state=0;
            mode=0;
        }
        k_msleep(5);

        /*semaphore*/
        if(state==0){
            k_sem_give(&sem_a2automatic);
        }
        else if(state==1){
            k_sem_give(&sem_a2manual);
        }

        
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    }
}


void thread_MANUAL_code(void *argA,void *argB,void *argC)
{
    int ret=0;

    printk("Thread MANUAL init\n");

    while(1){
        k_sem_take(&sem_a2manual,  K_FOREVER);

        printk("Task MANUAL at time: %lld ms\t\t",k_uptime_get());
        //printk("Value of variable up: %d\n ",up);

        if(up==1){
              if(highLevel_us<=950){  /* DA sistemare i limiti e il passo !!!! */
                  highLevel_us+=50;
              }
              up=0;
              k_msleep(5);
          }
          else if(down==1){
              if(highLevel_us>0){  
                  highLevel_us-=50;
              }
              down=0;
              k_msleep(5);
          }

          printk("High level period is: %d us \n",highLevel_us);

         /*semaphore*/
         k_sem_give(&sem_manFilter2pwm);


    }
}


void thread_AUTOMATIC_code(void *argA,void *argB,void *argC)
{
    int err=0;
    printk("Thread AUTOMATIC init\n");

    while(1){
        k_sem_take(&sem_a2automatic,  K_FOREVER);

        printk("Task AUTOMATIC at time: %lld ms\t",k_uptime_get());

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
                printk("Sample: %4u\n",sample);
             }
        }

        /*semaphore*/
        k_sem_give(&sem_auto2filter);


    }
}


void thread_FILTER_code(void *argA,void *argB,void *argC)
{
    uint16_t output=0;
    uint16_t diff=0;

    printk("Thread AUTOMATIC init\n");

    while(1){
        k_sem_take(&sem_auto2filter,  K_FOREVER);

        printk("Task FILTER at time: %lld ms\t",k_uptime_get());

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
        printk("Vector filtered: ");
        for(uint8_t i=0;i<10;i++){
            printk("\t %d",filteredSamples[i]);
        }
        printk("\t");
          

        uint16_t sum2=0;
	
        for(uint16_t a=0;a<j;a++){
            sum2+=filteredSamples[a];
        }
         
        //condition to avoid a zero denominator
        if(j>0){
            output=sum2/j;
        }
        
        /* Set the intensity value of the LED */
        if(output-threshold>=5 && diff<1024){
            diff++;
        }
        else if(output-threshold<5 && diff>0){
            diff--;
        }
        

        if(output>0){
            highLevel_us=pwmPeriod_us*(diff)/1023;
        }

        /* Safety value of PWM */
        if(highLevel_us>1000){
            highLevel_us=1000;
        }
        else if(highLevel_us<0){
            highLevel_us=0;
        }
        
        printk("Output=    diff= %d+%d\n",output,diff);
    
        /*semaphore*/
        k_sem_give(&sem_manFilter2pwm);

    }
}


void thread_PWM_code(void *argA,void *argB,void *argC)
{

    /* PWM control in function of environment light */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    
    int ret=0;                              /* Generic return value variable */

    /* Return pointer to device structure with the given name */
    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
        printk("Error: Failed to bind to PWM0 r\n");
        return;
    }

    printk("Thread PWM init\n\n");

    while(1){
        k_sem_take(&sem_manFilter2pwm,  K_FOREVER);

        printk("Task PWM at time: %lld ms\n\n",k_uptime_get());
        //printk("highLevel_us value into PWM thread: %d \n\n",highLevel_us);

        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)(pwmPeriod_us-highLevel_us), PWM_POLARITY_NORMAL);

        if (ret) {
            printk("Error %d: failed to set pulse width\n", ret);
            return;
        }

    }
}

