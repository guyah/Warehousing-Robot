# OS robot competition

Code developed for the robot competition organized as part of the Operating Systems course at EURECOM.

Link to the OS course website: http://soc.eurecom.fr/OS/projects_fall2020.html

The robot used is a LEGO Mindstorms EV3 Intelligent Brick (https://www.lego.com/en-us/product/ev3-intelligent-brick-45500) provided by EURECOM.

## Description of the competition

Here is the link to our robot's website: https://warhouser.wordpress.com

There you can find a description of the challenge together with an explanation of our design choices (shape of the robot and code).

## Preparing the robot

At this link you can find instructions on how to flash a Debian distribution on the EV3 and connect it to the Internet: https://www.ev3dev.org/docs/getting-started/

We used a Debian Jessie distribution (ev3dev-jessie-2017-09-14) that can be found at https://github.com/ev3dev/ev3dev/releases

After having flashed the SD card, these are the steps to be performed in order to have a fully operational robot:

- Update the robot with the latest packages:

    ```
    $ sudo apt-get update&&sudo apt-get upgrade
    ```
- Install extra packages needed for the project:

    ```
    $ sudo apt-get install gcc make
    ```
- Install the development environment on the robot:

    ```
    $ GIT_SSL_NO_VERIFY=true git clone https://github.com/in4lio/ev3dev-c.git
    $ cd ~/ev3dev-c/source/ev3
    $ make
    $ sudo make install
    ```

## Running our code

At this point, you can copy the code in this project to the robot: just use a USB cable or set up a ssh connection (default password: _maker_):

```
$ ssh robot@<your_IP_address>
```
To compile the code, type `make <executable>` in a terminal, where `<executable>` is simply the name of the C file you want to compile, without the ".c" part:

- File `tester.c` can be used to test that all motors and sensors are working. 
- File `playGame.c` is the code that runs the warehouse game: the robot will go to the origin cube, try to pick balls and bring them to the destination cube or to the pyramid, alternating between the two. 
- File `findCube.c`, finally, performs the task of finding the randomly placed cube.
