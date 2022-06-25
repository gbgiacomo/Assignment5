/** @file main.c
 * @brief Contains the implementation of the Assignment5
 *
 * The goal of this code is set the light intensity
 * of the enviromental at a certain threshold,
 * preaviusly defined by the user. In the manual 
 * mode the user can sets directly the light intensity 
 * of the LED, through the two switches.
 * Otherwise, in the automatic mode, LED is power by
 * the control algorithm.
 * The system work in real-time.
 * 
 * @author Mattia Longo and Giacomo Bego
 * @date 25 Jun 2022
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
#include <hal/nrf_saadc.h> /* ADC includes */
#include <console/console.h>  /* Console includs */

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_MANUAL_prio 1
#define thread_AUTOMATIC_prio 1
#define thread_FILTER_prio 1
#define thread_CONTROL_prio 1
#define thread_PWM_prio 1

/* First therad periodicity (in ms)*/
#define thread_A_period 100

#define STACK_SIZE 1024

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_MANUAL_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_AUTOMATIC_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_FILTER_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_CONTROL_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_PWM_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_MANUAL_data;
struct k_thread thread_AUTOMATIC_data;
struct k_thread thread_FILTER_data;
struct k_thread thread_CONTROL_data;
struct k_thread thread_PWM_data;

/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_MANUAL_tid;
k_tid_t thread_AUTOMATIC_tid;
k_tid_t thread_FILTER_tid;
k_tid_t thread_CONTROL_tid;
k_tid_t thread_PWM_tid;

/* Semaphores for task synch */
struct k_sem sem_a2manual;
struct k_sem sem_a2automatic;
struct k_sem sem_auto2filter;
struct k_sem sem_filter2control;
struct k_sem sem_manControl2pwm;

/* Thread code prototypes */
void thread_A_code(void *,void *,void *);
void thread_MANUAL_code(void *,void *,void *);
void thread_AUTOMATIC_code(void *,void *,void *);
void thread_FILTER_code(void *,void *,void *);
void thread_CONTROL_code(void *,void *,void *);
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
#define BOARDBUT4 0x19 /* Pin at which BUT4 is connected. Addressing is direct (i.e., pin number) */

/* Int related declarations */
static struct gpio_callback but1_cb_data; /* Callback structure */
static struct gpio_callback but2_cb_data; /* Callback structure */
static struct gpio_callback but3_cb_data; /* Callback structure */
static struct gpio_callback but4_cb_data; /* Callback structure */

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
static uint16_t output;

/* Vector to filter declaration */
uint16_t samples[SIZE]={0,0,0,0,0,0,0,0,0,0};
uint16_t filteredSamples[SIZE]={0,0,0,0,0,0,0,0,0,0};
int8_t index=-1;
uint16_t average=0;
uint16_t upperLevel=0;
uint16_t lowerLevel=0;
uint16_t pwmPeriod_us = 500;       /* PWM period in us */

/* Variable initialization to console */
static const char prompt[10];

/* Time variable declaration */
static uint8_t hours;
static uint8_t minutes;
static uint8_t seconds;
static uint8_t days;

/* Console vector variables */
static uint16_t start_time[100]={0};
static uint16_t stop_time[100]={0};
static uint16_t intensity[100]={0};
static uint16_t period_n=0;

/* Callback function and variables */
volatile bool mode=0;
volatile bool up=0;
volatile bool down=0;
volatile bool cons=0;
volatile bool dark=0;
bool state=0;

/**
 * @brief but1press_cbfunction function run ISR for browse up
 *
 * but1press_cbfunction is the service
 * routine related to the interrupt
 * for browsing up. It just change
 * the state of the volatile "up" variable
 * 
 */

void but1press_cbfunction(){
    //printk("Button 1 pressed\n");
    up=1;
}

/**
 * @brief but2press_cbfunction function run ISR for browse down
 *
 * but2press_cbfunction is the service
 * routine related to the interrupt
 * for browsing up. It just change
 * the state of the volatile "down" variable
 * 
 */

void but2press_cbfunction(){
    //printk("Button 2 pressed\n");
    down=1;
}

/**
 * @brief but3press_cbfunction function run ISR for changing mode
 *
 * but3press_cbfunction is the service
 * routine related to the interrupt
 * for changing mode. It just modify
 * the state of the volatile "mode" variable
 * 
 */

void but3press_cbfunction(){
    //printk("Button 3 pressed\n");
    mode=1;
}

/**
 * @brief but4press_cbfunction function run ISR for activating the console
 *
 * but4press_cbfunction is the service
 * routine related to the interrupt
 * for activating console. It just modify
 * the state of the volatile "dark" and "cons" variables
 * 
 */

void but4press_cbfunction(){
    dark=1;
    cons=1;
}

/**
 * @brief adc_sample function read the input voltage
 *
 * ADC setting and acquisition of the input voltage.
 * In this case the resolution has been setted to 10 bit (0-1023).
 * 
 * \author Mattia Longo and Giacomo Bego
 * \return integer value, representing the acquired sample
 */

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

/**
 * @brief char2int function convert numerical char to int
 *
 * It convert the numerical char variables to integer values 
 * 
 * \author Mattia Longo and Giacomo Bego
 * \param[ch] char variable to converter (only value from 0 to 9)
 * \return integer value converted
 */

int char2int(char ch){
	switch(ch){
        case '0':
		return 0;
		break;
	case '1':
		return 1;
		break;
	case '2':
		return 2;
		break;
	case '3':
		return 3;
		break;
	case '4':
		return 4;
		break;
	case '5':
		return 5;
		break;
	case '6':
		return 6;
		break;
	case '7':
		return 7;
		break;
	case '8':
		return 8;
		break;
	case '9':
		return 9;
		break;
	}
}

/**
 * @brief printDays function print the days
 *
 * This function print the days of the week to console 
 * 
 * \author Mattia Longo and Giacomo Bego
 * \param[day] integer value that indicates the day from 0 to 6
 */

void printDays(uint8_t day){
    switch(day){
        case 0:
		printk("DAY: SUNDAY\t");
		break;
	case 1:
		printk("DAY: MONDAY\t");
		break;
	case 2:
		printk("DAY: TUESDAY\t");
		break;
	case 3:
		printk("DAY: WEDNESDAY\t");
		break;
	case 4:
		printk("DAY: THURSDAY\t");
		break;
	case 5:
		printk("DAY: FRIDAY\t");
		break;
	case 6:
		printk("DAY: SATURDAY\t");
		break;
	}
    return;
}

/**
 * @brief userSetup function activate the console
 *
 * Activate the user interface console to setup the ON/OFF period and the current time.
 * 
 * \author Mattia Longo and Giacomo Bego
 */

void userSetup(){

    uint16_t i=0;
    uint64_t currentTime=0;

    /* Console initialization */
    char c='0';
    char command[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        
    printk("Task CONSOLE at time: %lld ms\t\t",k_uptime_get());
        
    for(int l=0;l<sizeof(command);l++){
        command[l]=0;
    }
            
    printk("\n\n\n\nDigit a string to set calendar and clock time: \n");
    printk("TDHHMM!\n");
    printk("T= setting the clock time\nD= day of the week (from 0= SUN to 6= SAT)\nHH= hour of the day (0-24)\nMM= minute\n != end of the command\n");
    printk("Example: T51035! ---> Set the timer on friday at 10:35\n\n");

    printk("\n\nDigit a string to set the ON/OFF period: \n");
    printk("SDHHMMDHHMMIII!\n");
    printk("S= setting the ON/OFF period\nD= day when the period starts (from 0= SUN to 6= SAT)\nHH= hour when the period starts (0-24)\nMM= minute when the period starts\n");
    printk("D= day when the period ends (from 0= SUN to 6= SAT)\nHH= hour when the period ends (0-24)\nMM= minute when the period ends\n!= end of the command\n");
    printk("Example: S5220060500080 ---> Set the period fron friday at 22:00 to saturday at 05:00 with 80 % of light intensity\n\n");

    printk("Digit the string:\t");

    currentTime=k_uptime_get(); /* Save the current time */

    while(c!='!'){
        c = console_getchar();
        console_putchar(c);
        command[i]=c;
        i++;
    }

    if(command[0]=='S'){
        start_time[period_n]= char2int(command[1])*10000 + char2int(command[2])*1000 + char2int(command[3])*100 + char2int(command[4])*10+ char2int(command[5]);
        stop_time[period_n]= char2int(command[6])*10000 + char2int(command[7])*1000 + char2int(command[8])*100 + char2int(command[9])*10+ char2int(command[10]);
        intensity[period_n]= char2int(command[11])*100 + char2int(command[12])*10 + char2int(command[13]);
        period_n++;

        /* Update the lost clock time during the console */
        currentTime=k_uptime_get()-currentTime;
        seconds=(seconds+currentTime/1000)%60;
        minutes=(minutes+currentTime/(1000*60))%60;
        hours=(hours+currentTime/(1000*60*60))%60;
                
    }
    else if(command[0]=='T'){
        days=char2int(command[1]);
        hours=char2int(command[2])*10+char2int(command[3]);
        minutes=char2int(command[4])*10+char2int(command[5]);
    }
    else{
        printk("\n Command don't exists. Write the command 'T' or 'S' in the string \n");
    }
        return;
}

/**
 * @brief main function run project
 *
 * main function sets the ADC, 
 * creates the tasks and
 * initilizes the semaphores.
 * 
 */

void main(void) {
    
    int ret=0;
    int err=0;

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
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT4, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 4 \n\r", ret);
	return;
    }

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
    if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }

    /* Init console service */
    console_init();

    /* Output a string */ 
    console_write(NULL, prompt, sizeof(prompt) - 1);

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

    /* Set interrupt HW - which pin and event generate interrupt */
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT4, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&but4_cb_data, but4press_cbfunction, BIT(BOARDBUT4));
    gpio_add_callback(gpio0_dev, &but4_cb_data);

    /* Create and init semaphores */
    k_sem_init(&sem_a2manual, 0, 1);
    k_sem_init(&sem_a2automatic, 0, 1);
    k_sem_init(&sem_auto2filter, 0, 1);
    k_sem_init(&sem_filter2control, 0, 1);
    k_sem_init(&sem_manControl2pwm, 0, 1);
    
    
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

    thread_CONTROL_tid = k_thread_create(&thread_CONTROL_data, thread_CONTROL_stack,
        K_THREAD_STACK_SIZEOF(thread_CONTROL_stack), thread_CONTROL_code,
        NULL, NULL, NULL, thread_CONTROL_prio, 0, K_NO_WAIT);

    thread_PWM_tid = k_thread_create(&thread_PWM_data, thread_PWM_stack,
        K_THREAD_STACK_SIZEOF(thread_PWM_stack), thread_PWM_code,
        NULL, NULL, NULL, thread_PWM_prio, 0, K_NO_WAIT);
    
    return;
} 


/* THREAD IMPLEMENTATION */

/**
 * @brief thread_A_code function implement the clock time 
 *
 * 
 * This function occurs every 100 ms.
 * The aims are increment the current hour, date and 
 * verify the function mode selected.
 * The user interface is activate cecking the switch state.
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything
 * 
 */

void thread_A_code(void *argA,void *argB,void *argC)
{

    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0, init_time=0;

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    while(1){
        init_time=k_uptime_get();

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

        days=days%7;

        /* Accours when console switch is pressed */
        if(cons && dark){
            highLevel_us=0;
            k_sem_give(&sem_manControl2pwm);
        }
        else if(cons && !dark){
            cons=0;
            userSetup();
            release_time = k_uptime_get() + thread_A_period;         
        }


        /* Print the time */
        printDays(days);
        printk("TIME: %d:%d:%d\t",hours,minutes,seconds);


        /* Manual/Automatic mode */
        if(mode==1 && state==0){
            printk("Switch to MANUAL MODE\t");
            /* Reset the variables */
            highLevel_us=pwmPeriod_us/4;
            up=0;
            down=0;

            state=1;
            mode=0;
        }
        else if(mode==1 && state==1){
            printk("Switch to AUTOMATIC MODE\t");
            highLevel_us=0;
            state=0;
            mode=0;
        }

        /*semaphore*/
        if(state==0){
            printk("Execution time task A: %lld ms\n",k_uptime_get()-init_time);
            k_sem_give(&sem_a2automatic);
        }
        else if(state==1){
            printk("Execution time task A: %lld ms\n",k_uptime_get()-init_time);
            k_sem_give(&sem_a2manual);
        }

        
        
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if(fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    }
}

/**
 * @brief thread_MANUAL_code function implement the manual mode 
 *
 * When the user press the button 3 this thread occurs.
 * The aim is increase and deacrease the light intensity 
 * of the LED, through two switches ("up" and "down").
 * Pressing them, the high level period of the PWM change.
 * This initial value is taken from the shared memory.
 * The output is saved in a shared memory (global variable)
 * and passed to the LED power task by another semaphore.
 * 
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything
 * 
 */

void thread_MANUAL_code(void *argA,void *argB,void *argC)
{
    uint16_t step=pwmPeriod_us/20;

    uint64_t init_time=0;

    printk("Thread MANUAL init\n");

    while(1){
        k_sem_take(&sem_a2manual,  K_FOREVER);

        init_time=k_uptime_get();

        printk("Task MANUAL at time: %lld ms\t\t",k_uptime_get());

        if(up==1){
              if(highLevel_us<=pwmPeriod_us-step){
                  highLevel_us+=step;
              }
              up=0;
              k_msleep(5);
          }
          else if(down==1){
              if(highLevel_us>0){  
                  highLevel_us-=step;
              }
              down=0;
              k_msleep(5);
          }

          printk("High level period is: %d us \t",highLevel_us);

          printk("Execution time task MANUAL: %lld ms\n",k_uptime_get()-init_time);

          /*semaphore*/
          k_sem_give(&sem_manControl2pwm);

    }
}

/**
 * @brief thread_AUTOMATIC_code function implement the automatic mode 
 *
 * If the acquisition by the ADC gets a correct value, 
 * it pass the sample by a semaphore to filter task, otherwise
 * it set it to a "safety value" equal to zero. 
 * 
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything
 * 
 */

void thread_AUTOMATIC_code(void *argA,void *argB,void *argC)
{
    int err=0;

    uint64_t init_time=0;

    printk("Thread AUTOMATIC init\n");

    while(1){
        k_sem_take(&sem_a2automatic, K_FOREVER);

        init_time=k_uptime_get();

        printk("Task AUTOMATIC at time: %lld ms\t",k_uptime_get());

        /* Get one sample, checks for errors and prints the values */
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\t\r");
                sample=0;  /* Safety value */
            }
            else {
                sample=1023-adc_sample_buffer[0];
                printk("Sample: %4u\t",sample);
             }
        }

         printk("Execution time task AUTOMATIC: %lld ms\n",k_uptime_get()-init_time);

        /*semaphore*/
        k_sem_give(&sem_auto2filter);


    }
}

/**
 * @brief thread_FILTER_code function implement the filtering task
 *
 * Every new sample is taken from the shared memory and put into a vector
 * from which we extract the average value of the last 10 samples.
 * Then the goal of the task is to copy the starting vector into another 
 * vector except for the samples "more than 5% far" form the average.
 * At the end, a new average on the final vector has been done.
 * The result is saved in another shared memory (global variable)
 * and passed to the output task by another semaphore.
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything
 * 
 */

void thread_FILTER_code(void *argA,void *argB,void *argC)
{

    printk("Thread FILTER init\n");

    uint64_t init_time=0;

    while(1){
        k_sem_take(&sem_auto2filter,  K_FOREVER);

        init_time=k_uptime_get();

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
        upperLevel=average*1.05;
        lowerLevel=average*0.95;

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

        printk("Execution time task FILTER: %lld ms\n",k_uptime_get()-init_time);
    
        /*semaphore*/
        k_sem_give(&sem_filter2control);

    }
}

/**
 * @brief thread_CONTROL_code function implement the control algorithm
 *
 * This function occurs in the automatic mode.
 * The control algorithm is implemented here.
 * The filtered output value is passed from the 
 * FILTER task. The current hour and date are 
 * compare with the periods setted by the user.
 * When the ON periods occurs, the control algorithm
 * increase or decrese the PWM high period value in order
 * to reach the environmental intensity setted by
 * the user.
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything 
 *
 */

void thread_CONTROL_code(void *argA,void *argB,void *argC)
{
    uint16_t diff=0;
    uint32_t actualTime=0;
    uint64_t init_time=0;
    int currentPeriod=0;

    printk("Thread CONTROL init\n");

    while(1){
        k_sem_take(&sem_filter2control, K_FOREVER);

        init_time=k_uptime_get();

        printk("Task CONTROL at time: %lld ms\t",k_uptime_get());

        actualTime=days*10000+hours*100+minutes;
        
        int currentPeriod=-1;

        for(int j=0;j<period_n;j++){
            if(actualTime>=start_time[j] && actualTime<stop_time[j]){
                currentPeriod=j;
                j=period_n;
            }
        }

        printk("CurrentPeriod: %d\t",currentPeriod);

        if(currentPeriod!=-1){
            printk("Intensity: %d % \t",intensity[currentPeriod]);

            /* Set the intensity value of the LED */
            if(output-(intensity[currentPeriod]*1023/100)>=2 && diff>0){
                diff-=16;
            }
            else if(output-(intensity[currentPeriod]*1023/100)<2 && diff<1024){
                diff+=16;
            }

            if(output>0){
                highLevel_us=pwmPeriod_us*(diff)/1023;
            }
        }
        else{
            highLevel_us=0;
        }
        
        
        /* Safety value of PWM */
        if(highLevel_us>pwmPeriod_us){
            highLevel_us=pwmPeriod_us;
        }
        else if(highLevel_us<0){
            highLevel_us=0;
        }
        
        printk("Output= %d \t diff= %d\t",output,diff);

        printk("Execution time task CONTROL: %lld ms\n",k_uptime_get()-init_time);

        k_sem_give(&sem_manControl2pwm);
    }

}

/**
 * @brief thread_PWM_code function shows the result by a LED
 *
 * This function gets the filtered result from task CONTROL and show
 * the result so that it is proportional to a PWM duty cicle of a LED.
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything 
 *
 */

void thread_PWM_code(void *argA,void *argB,void *argC)
{

    /* PWM control in function of environment light */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    
    int ret=0;                              /* Generic return value variable */

    uint64_t init_time=0;

    /* Return pointer to device structure with the given name */
    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
        printk("Error: Failed to bind to PWM0 r\n");
        return;
    }

    printk("Thread PWM init\n\n");

    while(1){
        k_sem_take(&sem_manControl2pwm,  K_FOREVER);

        init_time=k_uptime_get();

        printk("Task PWM at time: %lld ms\t",k_uptime_get());
        printk("highLevel_us value: %d \t",highLevel_us);

        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)(pwmPeriod_us-highLevel_us), PWM_POLARITY_NORMAL);

        if (ret) {
            printk("Error %d: failed to set pulse width\n", ret);
            return;
        }

        if(dark){
            dark=false;
        }
        printk("Execution time task PWM: %lld ms\n\n",k_uptime_get()-init_time);

    }
}

