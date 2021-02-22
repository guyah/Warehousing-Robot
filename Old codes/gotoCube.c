// Old code to go from the starting point to the origin cube
// There is no calibration in the rotation yet, this is why we pass 70 inside of 90 to the function that makes the robot rotate
// In this early version, in fact, the robot rotates too fast and the gyroscope can't send the stop signal to the motors in time
// We temporarily solved this issue by rotating less than needed (70 degrees instead of 90 in this case)
// In the final code we set up a calibration code that improves a lot the precision of the rotation

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

//useful constants
enum device{MOTOR, SONAR, GYRO};    //thread index
enum port{PORT_A=65, PORT_B=66, PORT_C=67, PORT_D=68};  //motor ports
enum stage{IDLE, FORWARD, BACKWARD, ROTCW, ROTCCW}; //each stage represents a way the robot can move
enum motor_side{LEFT, RIGHT};   //left or right wheel

//threads
void *motorThread();
void *sonarThread();
void *gyroThread();

//utility functions
void set_action(int next_stage, float threshold);
void wait_main();
void set_motor_param(uint8_t sn);
void set_motor_pol(uint8_t sn, int tmpstage, int side);
int check_condition(int value, int threshold, int tmpstage);

//global variables
int stage = IDLE;
int stop = 0;   //used to stop the wheels
float sonar_th; //used by sonar
float degrees;  //used by gyro

//mutexes and condition variables
pthread_mutex_t barrier;
pthread_cond_t cond_stage_main;
pthread_cond_t cond_stage_motor;
pthread_cond_t cond_stage_sonar;
pthread_cond_t cond_stage_gyro;
pthread_cond_t cond_stop;

int main(void)
{
    int i;
    char s[256];
    int val;
    uint32_t n, ii;
    void *join_ret;
    char *ev3_brick_addr;
    
    //declare threads
    pthread_t thread_id[3];
    #ifndef __ARM_ARCH_4T__
    /* Disable auto-detection of the brick (you have to set the correct address below) */
    ev3_brick_addr = "192.168.0.204";
    
    #endif
    if (ev3_init() == -1) return (1);
    
    #ifndef __ARM_ARCH_4T__
    printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );
    
    #else
    printf( "Waiting tacho is plugged...\n" );
    
    #endif
    while ( ev3_tacho_init() < 1 ) Sleep( 1000 );
    
    printf( "*** ( EV3 ) Hello! ***\n" );
    
    //print motors found
    printf( "Found tacho motors:\n" );
    for ( i = 0; i < DESC_LIMIT; i++ ) {
        if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
            printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
            printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }
    
    //Init sensors
    ev3_sensor_init();
    
    //print sensors found
    printf( "Found sensors:\n" );
    for(i=0; i<DESC_LIMIT; i++){
        if(ev3_sensor[i].type_inx != SENSOR_TYPE__NONE_){
            printf("  type = %s\n", ev3_sensor_type(ev3_sensor[i].type_inx));
            printf("  port = %s\n", ev3_sensor_port_name(i, s));
            if(get_sensor_mode(i, s, sizeof(s))){
                printf("  mode = %s\n", s);
            }
            if(get_sensor_num_values(i, &n)){
                for(ii=0; ii<n; ii++){
                    if(get_sensor_value(ii, i, &val)){
                        printf("  value%d = %d\n", ii, val);
                    }
                }
            }
        }
    }
    
    //make sure that the motors are stopped at the beginning
    stop = 0;
    
    //initialize mutexes and condition variables
	pthread_mutex_init(&barrier, NULL);
	pthread_cond_init(&cond_stage_main, NULL);
	pthread_cond_init(&cond_stage_motor, NULL);
	pthread_cond_init(&cond_stage_sonar, NULL);
	pthread_cond_init(&cond_stage_gyro, NULL);
	pthread_cond_init(&cond_stop, NULL);		

    //create threads
	pthread_create(&thread_id[MOTOR], NULL, motorThread, NULL);
	pthread_create(&thread_id[SONAR], NULL, sonarThread, NULL);
	pthread_create(&thread_id[GYRO], NULL, gyroThread, NULL);
    
    //List of actions performed by the robot
    printf("Going forward.\n");
    set_action(FORWARD, 500.0);
    wait_main();
    printf("Rotating.\n");
    set_action(ROTCW, 70.0);
    wait_main();
    printf("Going forward.\n");
    set_action(FORWARD, 150.00);
    wait_main();
    
    //cancel threads
    pthread_cancel(thread_id[MOTOR]);
    pthread_cancel(thread_id[SONAR]);
    pthread_cancel(thread_id[GYRO]);
	
    //wait for threads to end
    pthread_join(thread_id[MOTOR], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Motor thread successfully canceled.\n");
    pthread_join(thread_id[SONAR], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Sonar thread successfully canceled.\n");
    pthread_join(thread_id[GYRO], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Gyro thread successfully canceled.\n");
    
    //cancel mutexes and condition variables
    pthread_mutex_destroy(&barrier);
	pthread_cond_destroy(&cond_stage_main);
	pthread_cond_destroy(&cond_stage_motor);
	pthread_cond_destroy(&cond_stage_sonar);
	pthread_cond_destroy(&cond_stage_gyro);
	pthread_cond_destroy(&cond_stop);
    
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    
    return (0);
}

// set the needed global variables depending on the stage
void set_action(int next_stage, float threshold){
    pthread_mutex_lock(&barrier);
    stage = next_stage;
    //if the motor has to move forward or backward, set the sonar threshold to the desired value
    if(stage==FORWARD || stage==BACKWARD)
        sonar_th = threshold;   //update global variable
    //if the motor has to rotate, set the gyro threshold to the desired value
    else if(stage==ROTCW || stage==ROTCCW)
        degrees = threshold;    //update global variable
    
    //unlock motors
    stop = 0;
    //send signal to sonar and gyro
    pthread_mutex_unlock(&barrier);
    pthread_cond_signal(&cond_stage_sonar);
    pthread_cond_signal(&cond_stage_gyro);
}

// used by the main to wait until the operations of the current stage have been completed
void wait_main(){
    pthread_mutex_lock(&barrier);
    while(stage != IDLE)
        pthread_cond_wait(&cond_stage_main, &barrier);
    pthread_mutex_unlock(&barrier);
}

void *motorThread(){
    int portLEFT = PORT_B;
    int portRIGHT = PORT_C;
    uint8_t snLEFT;
    uint8_t snRIGHT;
    
    if (!(ev3_search_tacho_plugged_in(portLEFT, 0, &snLEFT, 0) && ev3_search_tacho_plugged_in(portRIGHT, 0, &snRIGHT, 0))) {
        fprintf(stderr, "Arm motor NOT found\n");
        exit(-1);
    }
    
    //set asynchronous cancel, so that the thread can be canceled at any moment
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    pthread_mutex_lock(&barrier);
    while(1){
        // wait as long as the stage is IDLE
        while(stage == IDLE)
            pthread_cond_wait(&cond_stage_motor, &barrier);
        
        // run until receiving the stop signal from either the sonar or the gyro
        printf("Motors running.\n");
        while(!stop){
            // set the motor polarity and other parameters, then send the RUN_FOREVER command
            set_motor_pol(snLEFT, stage, LEFT);
            set_motor_pol(snRIGHT, stage, RIGHT);
            set_motor_param(snLEFT);
            set_motor_param(snRIGHT);
            set_tacho_command_inx(snLEFT, TACHO_RUN_FOREVER);
            set_tacho_command_inx(snRIGHT, TACHO_RUN_FOREVER);
    
            pthread_cond_wait(&cond_stop, &barrier);
        }

        // stop the motors when the signal is received
        printf("Received stop signal.\n");
        set_tacho_command_inx(snLEFT, TACHO_STOP);
        set_tacho_command_inx(snRIGHT, TACHO_STOP);
    }
}

//set speed and type of stop action (BRAKE here, because it's more reactive)
void set_motor_param(uint8_t sn){
    int max_speed;
    get_tacho_max_speed(sn, &max_speed);
    set_tacho_stop_action_inx(sn, TACHO_BRAKE);
    set_tacho_speed_sp(sn, max_speed * 1/4);
}

//set the polarity of the motors depending on the stage
void set_motor_pol(uint8_t sn, int tmpstage, int side){
    switch(tmpstage){
        case FORWARD: 
            set_tacho_polarity(sn, "normal");
            break;
        case BACKWARD:
            set_tacho_polarity(sn, "inversed");
            break;
        case ROTCW:
            if(side == LEFT)
                set_tacho_polarity(sn, "normal");
            else if(side == RIGHT)
                set_tacho_polarity(sn, "inversed");
            break;
        case ROTCCW:
            if(side == LEFT)
                set_tacho_polarity(sn, "inversed");
            else if(side == RIGHT)
                set_tacho_polarity(sn, "normal");
    }
}

void *sonarThread(){
    uint8_t sn;
    float value;
    float threshold;
    int tmpstage;
    
    if(!ev3_search_sensor(LEGO_EV3_US, &sn, 0)){
        fprintf(stderr, "Sonar NOT found.\n");
        exit(-1);
    }

    //set asynchronous cancel, so that the thread can be canceled at any moment
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(1){
        pthread_mutex_lock(&barrier);
        while(stage!=FORWARD && stage!=BACKWARD)    // wait until we are in a stage where the sonar is needed
            pthread_cond_wait(&cond_stage_sonar, &barrier);
        threshold = sonar_th;
        tmpstage = stage; 
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stage_motor);

        // get values continuously and send stop signal when the threshold has been reached
        if(!get_sensor_value0(sn, &value))
            value = 0;
        while(check_condition(value, threshold, tmpstage)){
            if(!get_sensor_value0(sn, &value))
                value = 0;
        }
        printf("Sonar sending stop signal.\n");
        pthread_mutex_lock(&barrier);
        stop = 1;
        stage = IDLE;
        pthread_mutex_unlock(&barrier);        
        pthread_cond_signal(&cond_stop);
        pthread_cond_signal(&cond_stage_main);
    }
}

void *gyroThread(){
    uint8_t sn;
    float value;
    float threshold;
    int tmpstage;

    if(!ev3_search_sensor(LEGO_EV3_GYRO, &sn, 0)){
        fprintf(stderr, "Gyro NOT found.\n");
        exit(-1);        
    }
    
    //set asynchronous cancel, so that the thread can be canceled at any moment
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(1){
        pthread_mutex_lock(&barrier);
        while(stage!=ROTCW && stage!=ROTCCW)    // wait until we are in a stage where the gyro is needed
            pthread_cond_wait(&cond_stage_gyro, &barrier);
        
        // the gyro gives a relative measurement, so we have to find the current gyro value in order to set the correct threshold
        if(!get_sensor_value0(sn, &value))
            value = 0;
        threshold = value+degrees;
        tmpstage = stage;
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stage_motor);
        
        // get values continuously and send stop signal when the threshold has been reached
        while(check_condition(value, threshold, tmpstage)){
            if(!get_sensor_value0(sn, &value))
                value = 0;
	    }
        printf("Gyro sending stop signal, value = %f.\n", value);
        pthread_mutex_lock(&barrier);
        stop = 1;
        stage = IDLE;
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stop);
        pthread_cond_signal(&cond_stage_main);
    }
}

// check if the threshold has been reached
// the condition is different depending on the stage
int check_condition(int value, int threshold, int tmpstage){
    int ret=0;
    switch(tmpstage){
        case FORWARD: 
        case ROTCCW:
            ret = value > threshold;
            break;
        case BACKWARD:
        case ROTCW:
            ret = value < threshold;
            break;
    }
    return ret;
}
