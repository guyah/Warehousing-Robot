// Code used to grab the ball (either in front of the robot or inside a cube)

#include <stdio.h>
#include <stdlib.h>
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
#define Sleep(msec) usleep((msec) * 1000)
#define PORT_A 65
#define PORT_B 66
#define PORT_C 67
#define PORT_D 68

//////////////////////////////////////////////////
#endif

void open_hand();
void close_hand();
void move_hand(int dir, int sleep);
void move_arm(int dir, int sleep); 

int main(int argc, char **argv)
{
    int i;
    uint8_t sn;
    char s[256];
    char *ev3_brick_addr;

    #ifndef __ARM_ARCH_4T__
    /* Disable auto-detection of the brick (you have to set the correct address below) */
    //ev3_brick_addr = "192.168.43.197";
    ev3_brick_addr = "192.168.1.23";
    
    #endif
    if (ev3_init() == -1) return (1);
    
    #ifndef __ARM_ARCH_4T__
    printf("The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr);
    
    #else
    printf("Waiting tacho is plugged...\n");
    
    #endif
    while (ev3_tacho_init() < 1) Sleep(1000);
    
   
    printf("*** (EV3) Hello! ***\n");
    
    printf("Found tacho motors:\n");
    for (i = 0; i < DESC_LIMIT; i++) {
        if (ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_) {
            printf("  type = %s\n", ev3_tacho_type(ev3_tacho[ i ].type_inx));
            printf("  port = %s\n", ev3_tacho_port_name(i, s));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }
    
    // open hand
    open_hand();
    // arm down
    move_arm(-1,2000);
    // close hand
    close_hand();
    // arm up
    move_arm(1,2000);
    
    ev3_uninit();
    printf("*** (EV3) Bye! ***\n");
    
    return (0);
}

void open_hand(){
	// close hand
	move_hand(-1,100);
	move_hand(-1,2000);
	// open hand
	move_hand(1,100);
	move_hand(1,2000);
}

void close_hand(){
	move_hand(-1,100);
	move_hand(-1,2000);
}

void move_hand(int dir, int sleep){
    uint8_t sn;
    int port = PORT_A;
    int max_speed;
    int count_per_rot;
    
    if (ev3_search_tacho_plugged_in(port,0, &sn, 0)) {
        get_tacho_count_per_rot(sn,&count_per_rot);
        printf("Hand motor found\n");
        get_tacho_max_speed(sn, &max_speed);
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn, max_speed * 1/5);
        // Set the position the hand motor has to reach
        set_tacho_position_sp(sn, 0.5*dir*100*count_per_rot/360.0);
        set_tacho_polarity(sn, "normal");
        set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
        Sleep(sleep);
    } 
    else {
        printf("Hand motor NOT found\n");
    }
}

void move_arm(int dir, int sleep){
    uint8_t sn;
    int port = PORT_D;
    int max_speed;
    int count_per_rot;
    int degrees = 70;
    if(dir>0){
    	degrees= 85; 
    }
    
    if (ev3_search_tacho_plugged_in(port,0, &sn, 0)) {
        get_tacho_count_per_rot(sn,&count_per_rot);
        printf("Arm motor found\n");
        get_tacho_max_speed(sn, &max_speed);
        set_tacho_stop_action_inx(sn, TACHO_COAST);
        set_tacho_speed_sp(sn, max_speed * 1/3);
        // Set the position the arm motor has to reach
        set_tacho_position_sp(sn, 5*dir*degrees*count_per_rot/360.0);
        set_tacho_polarity(sn, "normal");
        set_tacho_command_inx(sn, TACHO_RUN_TO_REL_POS);
        Sleep(sleep);
    } 
    else {
        printf("Arm motor NOT found\n");
    }
}
