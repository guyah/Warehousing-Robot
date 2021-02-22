// Code that performs the final task of grabbing balls from the origin cube and bringing them to the destination cube or to the destination pyramid

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

#define TYRE_RADIUS 30  //radius of the wheels in cm, used to know how much time the motors should move in order to cover a certain distance
#define PI_CONST 3.141529   //pi constant
#define N_THREADS 4     //number of threads

//useful constants
enum device{MOTOR, SONAR, GYRO, ARM};    //thread index
enum port{PORT_A=65, PORT_B=66, PORT_C=67, PORT_D=68};  //motor ports
//each stage represents a way the robot can behave
enum stage{IDLE, FORWARD, BACKWARD, ROTCW, ROTCCW, FORWARD_TIMED, BACKWARD_TIMED, GRABBING, DROPPING, CALIBRATE, FORWARD_GRAB, BACKWARD_GRAB, READING}; 
enum motor_side{LEFT, RIGHT};   //left or right wheel

//threads
void *motorThread();
void *sonarThread();
void *gyroThread();
void *armThread();

//utility functions
void set_action(int next_stage, float threshold);
void wait_main();
void set_motor_param(uint8_t sn, int tmpstage);
float set_motor_time(uint8_t sn, float tmpdistance);
void set_motor_pol(uint8_t sn, int tmpstage, int side);
int check_condition(int value, int threshold, int tmpstage);
void cancel_and_join(pthread_t *thread_id);
void init_mutex_and_cond();
void destroy_mutex_and_cond();
float computeDiff(float initGyro, float angle);
void calibrate(float initGyro, float angle);

// hand and arm functions
void move_hand(int angle, uint8_t sn, int port, int sleep);
void move_arm(int angle, uint8_t sn, int port, int sleep);
void open_hand(uint8_t sn, int port);
void close_hand(uint8_t sn, int port, int sleep);

// used to check if the ball is currently being held
static bool _check_pressed(uint8_t sn);

// functions containing the steps to perform to go from a certain point of the stadium to another
void startToOrigin(float initGyro, float *angle_ptr);
void originToDestination(float initGyro, float *angle_ptr);
void destinationToOrigin(float initGyro, float *angle_ptr);
void originToPyramid(float initGyro, float *angle_ptr);
void destinationToOrigin(float initGyro, float *angle_ptr);
void pyramidToOrigin(float initGyro, float *angle_ptr);

//global variables
int stage = IDLE;
int stop = 0;   //used to stop the wheels
float sonar_th; //used by sonar
float degrees;  //used by gyro
float distance;  //used by motors when running for a certain amount of time
float currentGyro;  //used to calibrate the rotation

//mutexes and condition variables
pthread_mutex_t barrier;
pthread_cond_t cond_stage_main;
pthread_cond_t cond_stage_motor;
pthread_cond_t cond_stage_sonar;
pthread_cond_t cond_stage_gyro;
pthread_cond_t cond_stage_arm;
pthread_cond_t cond_stage_arm_grab;
pthread_cond_t cond_stop;

int main(void)
{
    int i;
    float initGyro;
    char s[256];
    int val;
    uint32_t n, ii;
    char *ev3_brick_addr;
    float diff = 0.0;
    float angle = 0.0;
    int iterations = 0;
    
    //declare threads
    pthread_t thread_id[N_THREADS];
    #ifndef __ARM_ARCH_4T__
    /* Disable auto-detection of the brick (you have to set the correct address below) */
    ev3_brick_addr = "192.168.43.197";
    
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
    init_mutex_and_cond();

    //create threads
    pthread_create(&thread_id[MOTOR], NULL, motorThread, NULL);
    pthread_create(&thread_id[SONAR], NULL, sonarThread, NULL);
    pthread_create(&thread_id[GYRO], NULL, gyroThread, NULL);
    pthread_create(&thread_id[ARM], NULL, armThread, NULL);
    
    //List of actions performed by the robot

    // Initial gyro measurement
    set_action(CALIBRATE,0.0);
    wait_main();
    initGyro = currentGyro;
    printf("initGyro %f\n",initGyro);

    //Go from the starting point to the origin cube
    startToOrigin(initGyro, &angle);
    
    //If we successfully grab a ball each time we try to do it, 2 iterations are sufficient, because we can put in place 2 balls per loop
    //(one in the cube and the other in the pyramid)
    for(iterations=0; iterations<2; iterations++){
        /*
        * Phase 1:
        * - The robot is in front of the origin cube
        * - Grab a ball inside the cube
        * - Go from the origin cube to the destination cube
        * - Drop the ball inside the destination cube
        * - Go from the destination cube to the origin cube
        * - Grab a new ball
        */
        set_action(GRABBING,0.0);
        wait_main();
        Sleep(2000);
        originToDestination(initGyro, &angle);
        set_action(DROPPING,0.0);
        wait_main();
        destinationToOrigin(initGyro, &angle);
        set_action(GRABBING,0.0);
        wait_main();
        
        /*
        * Phase 2:
        * - Having grabbed the ball, go from the origin cube to the pyramid
        * - Drop the ball inside the pyramid
        * - Go from the pyramid to the origin cube
        * - Restart from the beginning and grab a new ball
        */
        originToPyramid(initGyro, &angle);
        set_action(DROPPING,0.0);
        wait_main();
        Sleep(2000);
        pyramidToOrigin(initGyro, &angle);
    }
    
    /*NOTE:
     * As explained later in the section about the arm thread, even if a touch sensor is mounted on the hand, we are not able to detect in a reliable way
     * whether we have grabbed a ball or not. In principle, if this approach was reliable, we would be able to count how many balls we have grabbed, thus
     * knowing when all the balls inside a cube have been grabbed. If we could do so, we could exit the loop when we are sure to have grabbed all the balls,
     * thus avoiding to put a hard-coded limit of iterations equal to 2.
     */
    
    //cancel threads and wait for them to end
    cancel_and_join(thread_id);
    
    //destroy mutexes and condition variables
    destroy_mutex_and_cond();
    
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    
    return (0);
}

/* set_action() function
 * this function sets the needed global variables depending on the stage
 * next_stage is the stage we want to go into
 * threshold has a different meaning depending on the stage:
 * - FORWARD/BACKWARD: threshold value read by the sonar
 * - ROTCW/ROTCCW: threshold value read by the gyro
 * - FORWARD_TIMED/BACKWARD_TIMED: distance to be covered
 * - Other states: not used, a dummy value is passed
 */
void set_action(int next_stage, float threshold){
    pthread_mutex_lock(&barrier);
    stage = next_stage;
    
    //if the motors control themselves (they run for an amount of time which depends on the distance to be traveled)
    if(stage==FORWARD_TIMED || stage==BACKWARD_TIMED){
        distance = threshold;   //update global variable
        //send signal to motors
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stage_motor);
    }
    //if we're grabbing or dropping a ball
    else if(stage==GRABBING || stage==DROPPING){
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stage_arm);
    }
    //if we're calibrating the rotation
    else if(stage==CALIBRATE){
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stage_gyro);
    }
    //if we use sonar and gyro to control the motors
    else {
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
}

// used by the main to wait until the operations of the current stage have been completed
void wait_main(){
    pthread_mutex_lock(&barrier);
    while(stage != IDLE)
        pthread_cond_wait(&cond_stage_main, &barrier);
    printf("Main thread received signal.\n");
    pthread_mutex_unlock(&barrier);
}

void *motorThread(){
    int portLEFT = PORT_B;
    int portRIGHT = PORT_C;
    uint8_t snLEFT;
    uint8_t snRIGHT;
    int runTime;
    
    //check if the motors are correctly plugged
    if (!(ev3_search_tacho_plugged_in(portLEFT, 0, &snLEFT, 0) && ev3_search_tacho_plugged_in(portRIGHT, 0, &snRIGHT, 0))) {
        fprintf(stderr, "Wheel motor NOT found\n");
        exit(-1);
    }
    
    //set asynchronous cancel, so that the thread can be canceled at any moment
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    pthread_mutex_lock(&barrier);
    while(1){
        while(stage==IDLE || stage==GRABBING || stage==DROPPING || stage==CALIBRATE || stage==READING)
            pthread_cond_wait(&cond_stage_motor, &barrier);
        printf("Motors running.\n");
        
        //set the parameters of the motors
        set_motor_pol(snLEFT, stage, LEFT);
        set_motor_pol(snRIGHT, stage, RIGHT);
        set_motor_param(snLEFT,stage);
        set_motor_param(snRIGHT,stage);
            
        //if the motors control themselves (they run for an amount of time which depends on the distance to be traveled)
        if(stage==FORWARD_TIMED || stage==BACKWARD_TIMED || stage==FORWARD_GRAB || stage==BACKWARD_GRAB){
            //set how long the motors must move depending on the distance they must travel
            //the time the motors run is saved so that we can know how much they should sleep
            runTime = set_motor_time(snLEFT, distance);
            set_motor_time(snRIGHT, distance);
            set_tacho_command_inx(snLEFT, TACHO_RUN_TIMED);
            set_tacho_command_inx(snRIGHT, TACHO_RUN_TIMED);
        
            //sleep until the motors stop, then send signal to the main thread
            Sleep(runTime);
            if(stage==BACKWARD_GRAB || stage==FORWARD_GRAB){
                pthread_cond_signal(&cond_stage_arm_grab);
            }
            else {
                pthread_cond_signal(&cond_stage_main);
            }
            //set the stage to IDLE so that when the loop starts again the thread waits on the condition variable and releases the mutex
            stage = IDLE;
        }
        else {
            //run until sonar or gyro send stop signal to the motors
            while(!stop){
                set_tacho_command_inx(snLEFT, TACHO_RUN_FOREVER);
                set_tacho_command_inx(snRIGHT, TACHO_RUN_FOREVER);
                pthread_cond_wait(&cond_stop, &barrier);
            }

            //stop when receiving signal from sonar or gyro
            printf("Motors received stop signal.\n");
            set_tacho_command_inx(snLEFT, TACHO_STOP);
            set_tacho_command_inx(snRIGHT, TACHO_STOP);
        }
    }
}

void *sonarThread(){
    uint8_t sn;
    float value;
    float threshold;
    int tmpstage;

    //check if sonar is correctly plugged
    if(!ev3_search_sensor(LEGO_EV3_US, &sn, 0)){
        fprintf(stderr, "Sonar NOT found.\n");
        exit(-1);
    }

    //set asynchronous cancel, so that the thread can be canceled at any moment (TO BE CHECKED)
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(1){
        pthread_mutex_lock(&barrier);
        while(stage!=FORWARD && stage!=BACKWARD && stage!=READING)    //run only if the robot is moving forward or backward, otherwise wait
            pthread_cond_wait(&cond_stage_sonar, &barrier);
        //update variables
        tmpstage = stage;
        threshold = sonar_th;
        pthread_mutex_unlock(&barrier);
        
        //If we are in reading stage, we are just taking one value in order to know whether the random cube is present
        if(tmpstage == READING){
            //Sleep 1 second in order to avoid taking the value while the robot is still moving
            Sleep(1000);
            if(!get_sensor_value0(sn, &value))
                value = 0;
            printf("Distance = %f\n", value);
            pthread_mutex_lock(&barrier);
            distance = value;   //store the value in the corresponding global variable
            stage = IDLE;
            pthread_mutex_unlock(&barrier);
            pthread_cond_signal(&cond_stage_main);
        }       
        else {
            // Here we are in FORWARD or BACKWARD stage: send signal to the motors so that they can start
            pthread_cond_signal(&cond_stage_motor); 
            //get values continuously and send stop signal when the threshold has been reached (see function check_condition())
            if(!get_sensor_value0(sn, &value))
                value = 0;
            while(check_condition(value, threshold, tmpstage)){
                if(!get_sensor_value0(sn, &value))
                    value = 0;
            }
            //send signals when the threshold has been reached
            printf("Sonar sending stop signal.\n");
            pthread_mutex_lock(&barrier);
            stop = 1;
            stage = IDLE;
            pthread_mutex_unlock(&barrier);        
            pthread_cond_signal(&cond_stop);    //send signal to motors
            pthread_cond_signal(&cond_stage_main);  //send signal to main thread
        }
    }
}

void *gyroThread(){
    uint8_t sn;
    float value;
    float threshold;
    int tmpstage;
    float tmpGyro = 0.0;
    float tmpGyro2;
    
    //check if gyro is correctly plugged
    if(!ev3_search_sensor(LEGO_EV3_GYRO, &sn, 0)){
        fprintf(stderr, "Gyro NOT found.\n");
        exit(-1);        
    }
    
    //set asynchronous cancel, so that the thread can be canceled at any moment (TO BE CHECKED)
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(1){
        pthread_mutex_lock(&barrier);
        while(stage!=ROTCW && stage!=ROTCCW && stage!=CALIBRATE)    // wait until we are in a stage where the gyro is needed
            pthread_cond_wait(&cond_stage_gyro, &barrier);
        
        //if we are calibrating, we take the current value read by the gyroscope by averaging 10 readings
        if (stage == CALIBRATE){
            // Averaging gyro recordings
            for(int i = 0; i < 10; i++){
                Sleep(20);
                get_sensor_value0(sn, &tmpGyro2);
                tmpGyro+= tmpGyro2;    
            }
            tmpGyro = tmpGyro/10;
            currentGyro = tmpGyro;
            printf("current Gyro: %f\n",currentGyro);
        }
        //otherwise, rotate
        else{
            // the gyro gives a relative measurement, so we have to find the current gyro value in order to set the correct threshold
            if(!get_sensor_value0(sn, &value))
                value = 0;
            //update variables
            threshold = value+degrees;
            tmpstage = stage;
            pthread_mutex_unlock(&barrier);
            pthread_cond_signal(&cond_stage_motor); //send signal to the motors so that they can start
            
            //get values continuously and stop when the threshold has been reached (see function check_condition())
            while(check_condition(value, threshold, tmpstage)){
                if(!get_sensor_value0(sn, &value))
                    value = 0;
            }
            //send signals when the threshold has been reached
            printf("Gyro sending stop signal, value = %f.\n", value);
            pthread_mutex_lock(&barrier);    
        }

        stop = 1;
        stage = IDLE;
        pthread_mutex_unlock(&barrier);
        pthread_cond_signal(&cond_stop);    //send signal to motors
        pthread_cond_signal(&cond_stage_main);  //send signal to main thread
    }
}

void *armThread(){
    int portHand = PORT_A;
    int portArm = PORT_D;
    uint8_t snHand, snArm, snTouch;
    int tmpstage;
    
    //check if the motors are correctly plugged
    if (!(ev3_search_tacho_plugged_in(portArm, 0, &snArm, 0))) {
        fprintf(stderr, "Arm motor NOT found\n");
        exit(-1);
    }
    if (!(ev3_search_tacho_plugged_in(portHand, 0, &snHand, 0))) {
        fprintf(stderr, "Hand motor NOT found\n");
        exit(-1);
    }
    //check if the touch sensor is correctly plugged
    if(!(ev3_search_sensor(LEGO_EV3_TOUCH, &snTouch, 0))){
        fprintf(stderr, "Touch sensor NOT found\n");
        exit(-1);
    }
    //set asynchronous cancel, so that the thread can be canceled at any moment
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    while(1){
        pthread_mutex_lock(&barrier);
        while(stage!=GRABBING && stage!=DROPPING)    //run only if the robot is grabbing, otherwise wait
            pthread_cond_wait(&cond_stage_arm, &barrier);
        
        /* Grabbing process:
         * In order to maximize the probability of grabbing a ball, these are the steps performed:
         * 1. the robot makes the arm go down with the hand open
         * 2. then, it moves backwards for a few cm (8 in this case)
         * 3. by doing this, it can pull balls towards the front side of the cube, thus increasing the probability of grabbing a ball
         * 4. then, it closes the hand and possibly grabs a ball
         * 5. finally, it goes forward by the same amount of cm at point 2 and moves the arm up
         * 
         * NOTE: In our initial ideas, the robot should have been able to identify whether a ball has been grabbed or not thanks to a touch sensor mounted on the hand.
         * In fact, the ball should press the touch sensor when it is grabbed. However, this technique has proven not to be so reliable: in practice it can happen 
         * that even if the ball has been grabbed it is not able to correctly press the touch sensor. For this reason, even if in our code the touch sensor 
         * is present, it just informs the user when it is pressed, without having an active role. Our idea was to create a loop where the robot repeatedly tries 
         * to grab a ball until the touch sensor is pressed, but due to the unrealiability of this approach we had to abandon it and just try to grab the ball once.
         */
        if(stage == GRABBING){
            printf("Started grabbing");
            // open hand
            open_hand(snHand, portHand);
            // arm down
            move_arm(-70, snArm, portArm, 2000);

            //move backward
            stage = BACKWARD_GRAB;
            distance = 80.0;
            pthread_mutex_unlock(&barrier);
            pthread_cond_signal(&cond_stage_motor);
            pthread_cond_wait(&cond_stage_arm_grab, &barrier);

            // close hand and try to catch a ball
            close_hand(snHand, portHand, 500);
            move_arm(20, snArm, portArm, 2000);
            if(_check_pressed(snTouch)){
                printf("Ball grabbed\n");
            }

            //move forward
            stage = FORWARD_GRAB;
            distance = 80.0;
            pthread_mutex_unlock(&barrier);
            pthread_cond_signal(&cond_stage_motor);
            pthread_cond_wait(&cond_stage_arm_grab, &barrier);
            
            // arm up
            move_arm(60, snArm, portArm, 2000);
            printf("Grabbing ending.\n");
        }
        else if(stage == DROPPING){
            printf("Started dropping");
            // arm down
            move_arm(-15, snArm, portArm, 2000);
            // open hand
            move_hand(180, snHand, portHand, 2000);
            // arm up
            move_arm(45, snArm, portArm, 2000); 
            printf("Dropping ending.\n");           
        }
        
        //send signal to the main thread when the ball has been grabbed or dropped
        stage = IDLE;
        pthread_mutex_unlock(&barrier);        
        printf("Arm sending signal to main thread.\n");
        pthread_cond_signal(&cond_stage_main);  //send signal to main thread
    }
}

void move_hand(int angle, uint8_t sn, int port, int sleep){
    int max_speed;
    int count_per_rot;

    printf("Hand moving.\n");
    get_tacho_max_speed(sn, &max_speed);
    set_tacho_stop_action_inx(sn, TACHO_COAST);
    set_tacho_speed_sp(sn, max_speed * 1/5.0);
    get_tacho_count_per_rot(sn,&count_per_rot);
    // Set the position the hand motor has to reach
    set_tacho_position_sp(sn, 0.5*angle*count_per_rot/360.0);
    set_tacho_polarity(sn, "normal");
    set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
    Sleep(sleep);
}

void move_arm(int angle, uint8_t sn, int port, int sleep){
    int max_speed;
    int count_per_rot;

    printf("Arm moving.\n");
    get_tacho_max_speed(sn, &max_speed);
    set_tacho_stop_action_inx(sn, TACHO_COAST);
    set_tacho_speed_sp(sn, max_speed * 1/5.0);
    get_tacho_count_per_rot(sn,&count_per_rot);
    // Set the position the arm motor has to reach
    set_tacho_position_sp(sn, 5*angle*count_per_rot/360.0);
    set_tacho_polarity(sn, "normal");
    set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
    Sleep(sleep);
}

void open_hand(uint8_t sn, int port){
    move_hand(-110,sn,port,2000);
    move_hand(140,sn,port,2000);
}

void close_hand(uint8_t sn, int port, int sleep){
    move_hand(-350,sn,port,sleep);
}

//set speed and type of stop action (BRAKE here, because it's more reactive)
void set_motor_param(uint8_t sn,int tmpstage){
    int max_speed;
    get_tacho_max_speed(sn, &max_speed);
    set_tacho_stop_action_inx(sn, TACHO_BRAKE);
    if(tmpstage == ROTCW || tmpstage == ROTCCW){
        set_tacho_speed_sp(sn, max_speed * 1.0/70.0);
    } else if(tmpstage == BACKWARD || tmpstage == BACKWARD_TIMED) {
        set_tacho_speed_sp(sn, max_speed * 1.0/8.0);
    } else {
        set_tacho_speed_sp(sn, max_speed * 1.0/4.0);
    }
}

//compute how long the motors must move from the distance they must travel 
float set_motor_time(uint8_t sn, float tmpdistance){
    float speed, x, t, runTime;
    int max_speed,count_per_rot;
    get_tacho_max_speed(sn, &max_speed);
    get_tacho_count_per_rot(sn, &count_per_rot);
    speed = (float)(1.0/4.0 * max_speed);
    x = (float)tmpdistance/(2*PI_CONST*TYRE_RADIUS);
    t = (float) count_per_rot/ speed;
    runTime = (float) x*t*1000;
    set_tacho_time_sp(sn, runTime);
    
    return runTime; //return the computed time, so that the motor thread knows how long it should sleep
}

//set the polarity of the motors depending on the stage
void set_motor_pol(uint8_t sn, int tmpstage, int side){
    switch(tmpstage){
        case FORWARD:
        case FORWARD_TIMED:
        case FORWARD_GRAB:
            set_tacho_polarity(sn, "normal");
            break;
        case BACKWARD:
        case BACKWARD_TIMED:
        case BACKWARD_GRAB:
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
            break;
    }
}

/*Calibrate the rotation:
 * Setting the stage to CALIBRATE we take the current value read by the gyro
 * Then, we take the difference between the computed value and the expected one (initGyro+angle)
 * We then return this difference, which is used by the navigation functions in order to calibrate the rotation: the angle of the next rotation
 * depends from diff, so that it compensates if it has rotated more or less than needed: this means that at each rotation we compensate for the
 * error we made at the previous rotation
 */
float computeDiff(float initGyro, float angle){
    float diff;
    
    set_action(CALIBRATE,0.0);
    wait_main();
    diff = currentGyro - (initGyro + angle);
    diff = (float)((int)diff % 360);
    printf("Diff: %f\n",diff);    
    return diff;
}

/* Additional calibration:
 * Sometimes, the compensation made using the difference computed by computeDiff() may not be enough. Therefore, when we need to be absolutely precise (for
 * instance, when we are approaching the pyramid), an additional calibration step may be needed. In this function we perform this second calibration. After 
 * a rotation, we compute the difference with respect to the expected value (initGyro+angle) and we rotate by that difference, either clockwise or 
 * counterclockwise, depending on the value of diff (i.e, depending on whether we have rotated more or less than needed).
 */
void calibrate(float initGyro, float angle){
    float diff = computeDiff(initGyro, angle);
    if(diff < 0){
        set_action(ROTCW,-diff);    
    } else if(diff > 0) {
        set_action(ROTCCW,-diff);    
    }
    wait_main();    //wait until the rotation is completed
}

//Check the value of the sonar to find whether the random cube is present or not
//Not used in this code, it's part of the algorithm that finds the random cube
int checkForCube(float target){
    int ret;
    pthread_mutex_lock(&barrier);
    if(distance < target)
        ret = 1;
    else
        ret = 0;
    pthread_mutex_unlock(&barrier);
    return ret;
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

//check if the touch sensor is being pressed
static bool _check_pressed(uint8_t sn){
  int val;

  if(sn == SENSOR__NONE_){
    return(ev3_read_keys((uint8_t *)&val) && (val & EV3_KEY_UP));
  }
  return(get_sensor_value(0, sn, &val) && (val != 0));
}

//cancel threads and wait for them to end
void cancel_and_join(pthread_t *thread_id){
    void *join_ret;
    
    pthread_cancel(thread_id[MOTOR]);
    pthread_cancel(thread_id[SONAR]);
    pthread_cancel(thread_id[GYRO]);
    pthread_cancel(thread_id[ARM]);
    
    pthread_join(thread_id[MOTOR], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Motor thread successfully canceled.\n");
    pthread_join(thread_id[SONAR], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Sonar thread successfully canceled.\n");
    pthread_join(thread_id[GYRO], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Gyro thread successfully canceled.\n");    
    pthread_join(thread_id[ARM], &join_ret);
    if(join_ret == PTHREAD_CANCELED)
        printf("Arm thread successfully canceled.\n");
}

//initialize mutexes and condition variables
void init_mutex_and_cond(){
    pthread_mutex_init(&barrier, NULL);
    pthread_cond_init(&cond_stage_main, NULL);
    pthread_cond_init(&cond_stage_motor, NULL);
    pthread_cond_init(&cond_stage_sonar, NULL);
    pthread_cond_init(&cond_stage_gyro, NULL);
    pthread_cond_init(&cond_stage_arm, NULL);
    pthread_cond_init(&cond_stage_arm_grab, NULL);
    pthread_cond_init(&cond_stop, NULL);
}

//destroy mutexes and condition variables
void destroy_mutex_and_cond(){
    pthread_mutex_destroy(&barrier);
    pthread_cond_destroy(&cond_stage_main);
    pthread_cond_destroy(&cond_stage_motor);
    pthread_cond_destroy(&cond_stage_sonar);
    pthread_cond_destroy(&cond_stage_gyro);
    pthread_cond_destroy(&cond_stage_arm);
    pthread_cond_destroy(&cond_stage_arm_grab);
    pthread_cond_destroy(&cond_stop);
}

/* ROBOT STEPS
 * The following functions contain the steps performed by the robot to go from one point of the stadium to another
 * The actions can be:
 * - FORWARD/BACKWARD: go forward or backward until the sonar sends the stop signal;
 * - FORWARD_TIMED/BACKWARD_TIMED: go forward or backward for a certain amount of time, depending on the distance to be covered;
 * - ROTCW/ROTCCW: rotate clockwise or counterclockwise until the gyroscope sends the stop signal;
 */

//From starting point to the origin cube
void startToOrigin(float initGyro, float *angle_ptr){
    float diff;
    float angle = *angle_ptr;   //cumulative angle, used to keep track of each rotation, so that we know how we should be oriented
    
    set_action(FORWARD_TIMED, 1200.0);
    wait_main();
    set_action(ROTCW, 90.0);
    wait_main();
    angle += 90.0; // update angle: robot is at 90 degrees
    set_action(FORWARD, 240.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCCW, -90.0-diff); //here we rotate more or less than 90 degrees in order to compensate for possible previous errors in the rotation
    wait_main();
    angle += -90.0; // update angle: robot is at 0 degrees
    calibrate(initGyro, angle); //here we perform an additional calibration, because we are approaching the origin cube, so we want to be very precise
    set_action(FORWARD, 30.0);
    wait_main();
    set_action(FORWARD_TIMED, 15.0);
    wait_main();
    calibrate(initGyro, angle); //further calibration step
    wait_main();
    
    *angle_ptr = angle; //save the current angle at which the robot is oriented
}

//From origin cube to destination cube
void originToDestination(float initGyro, float *angle_ptr){
    float diff;
    float angle = *angle_ptr;
    
    set_action(BACKWARD_TIMED,50.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCCW,-90.0-diff);
    wait_main();
    angle += -90.0; // update angle: robot is at -90 degrees
    set_action(FORWARD, 30.0);
    wait_main();
    set_action(FORWARD_TIMED, 15.0);
    wait_main();
    calibrate(initGyro, angle);
    
    *angle_ptr = angle;
}

//From destination cube to origin cube
void destinationToOrigin(float initGyro, float *angle_ptr){
    float diff;
    float angle = *angle_ptr;
    
    set_action(BACKWARD_TIMED, 200.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCW, 180.0-diff);
    wait_main();
    angle += 180.0; // update angle: robot is at 90 degrees
    calibrate(initGyro, angle);
    set_action(FORWARD,240.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCCW, -90.0-diff);
    wait_main();
    angle += -90.0; // update angle: robot is at 0 degrees
    calibrate(initGyro, angle);
    set_action(FORWARD, 30.0);
    wait_main();
    calibrate(initGyro, angle);
    
    *angle_ptr = angle;
}

//From pyramid to origin cube
void pyramidToOrigin(float initGyro, float *angle_ptr){
    float diff;
    float angle = *angle_ptr;
    
    set_action(BACKWARD_TIMED, 200.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCW, 180.0-diff);
    wait_main();
    angle += 180.0; // update angle: robot is at 90 degrees
    calibrate(initGyro, angle);
    set_action(FORWARD,240.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCCW, -90.0-diff);
    wait_main();
    angle += -90.0; // update angle: robot is at 0 degrees
    calibrate(initGyro, angle);
    set_action(FORWARD, 30.0);
    wait_main();
    calibrate(initGyro, angle);
    
    *angle_ptr = angle;
}

//From origin cube to pyramid
void originToPyramid(float initGyro, float *angle_ptr){
    float diff;
    float angle = *angle_ptr;
    
    set_action(BACKWARD_TIMED,50.0);
    wait_main();
    set_action(BACKWARD,810.0);
    wait_main();
    diff = computeDiff(initGyro, angle);
    set_action(ROTCCW,-90.0-diff);
    wait_main();
    
    //Now, since we are going towards the pyramid, which requires high precision, we go forward in 2 steps, calibrating each time, in order
    //to be sure to be moving horizontally
    angle += -90.0; // update angle: robot is at -90 degrees
    calibrate(initGyro, angle);
    set_action(FORWARD, 400.0); //it should be around 80 cm
    wait_main();
    calibrate(initGyro, angle);
    set_action(FORWARD, 40.0); //it should be around 80 cm
    wait_main();
    calibrate(initGyro, angle);
    
    *angle_ptr = angle;
}
