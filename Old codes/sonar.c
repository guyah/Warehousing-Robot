// Code used to test the sonar

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
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

int main( void )
{
  uint8_t sn_sonar;
  float value;
  char *ev3_brick_addr;
#ifndef __ARM_ARCH_4T__
  /* Disable auto-detection of the brick (you have to set the correct address below) */
  ev3_brick_addr = "192.168.0.204";

#endif
  if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
  printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
  
#endif
  
  //Run all sensors
  ev3_sensor_init();

  for ( ; ; ){
    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
      printf("SONAR found, reading sonar...\n");
      if ( !get_sensor_value0(sn_sonar, &value )) {
        value = 0;
      }
      printf( "\r(%f) \n", value);
      fflush( stdout );
    }
  }

  ev3_uninit();
  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}
