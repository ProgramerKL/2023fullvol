/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kyle Liu                                                  */
/*    Created:      10/3/2023, 6:34:30 PM                                     */
/*    Description:  used for worlds                                           */
/*    Major Changes:                                                          */
/*      Non Supply zone Auto Route                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the IQ2 brain screen
vex::brain       Brain;
               
// define your global instances of motors and other devices here
inertial Inertial = inertial();
controller Controller = controller();
motor motor1 = motor (PORT2,false);
motor motor2=motor (PORT1,true);
motor motor3=motor (PORT8,true);
motor motor4=motor (PORT7,false);
motor leftmotor=motor(PORT6,true);
motor rightmotor=motor(PORT12,false);

motor_group motorgroup1=motor_group(motor1, motor2);
motor_group motorgroup2=motor_group(motor3, motor4);

touchled TouchLED1 = touchled(PORT5);
touchled TouchLED2 = touchled(PORT10);

/*
A: Pneumatic2  cylinder2 closest to intake
B: Pneumantic1 cylinder2 middle to intake
C: pneumatic1  cylinder1 farthest to intake

2 motor drive:
    pneumatic1 retract cylinder1

6 motor drive:
    pneumatic1 extend cylinder1
    pneumatic1 retract cylinder2
    pneumatic2 extend cylinder2

4 motor intake:
    Pneumatic2 retract cylinder2
    Pneumatic1 retract cylinder2
    Pneumatic1 retract cylinder1;

4 motor elevator
    Pneumatic1 extend cylinder1
    Pneumatic1 extend cylinder2


Purple:
Penumantic2 cylinder1

intake lift
Pneumatic3 cyliner2
*/
pneumatic Pneumatic1=pneumatic(PORT3);
pneumatic Pneumatic2=pneumatic(PORT11);
pneumatic Pneumatic3=pneumatic(PORT4);

distance Distancesensor = distance(PORT9);

//define your global varibals here
int deadband=5;
int drivemode=2;//drive mode, 2 or 6
int greenupmode;
bool start=false;//variable to start of the game
bool intake_running=false;//variable to track intake
bool green_running=false;//variable to track green
bool purple_dumping=false;
bool purple_lift=false;
bool ispurplegateopen;
bool Led=false;//variable to control whether to use LED function
bool intakestucked=false;
bool preventionfuncrunning=false;
bool insupplyzone=false;
bool rollerstucked=false;

float greendistance1 = 150;
float greendistance2 = 199;

float purpletimerstart;
float purpletimernow;
float timernow;
float timerstart;
float programtimer;
//counter:
int pumpcounter=0;
int intakecounter=0;
int stuckcounter=0;
int greencounter=0;
float bucketdistance;
float kp=1.4;
float kp6mdrive = 2;
int velocity=85;
int velocity6mdrive=85;
int turnvelocity;

int shiftwaittime=350;
//thread
thread autothread;
thread intakethread;
thread greenthread;


//event
event event_pump;
event eventintakelifter = event();
event eventintakeputdown = event();
event event_backinital;
event event_changepreventfunc;
event event_greenup;
//function:
void shift2mdrive();
void shift6mdrive();
void shift4mintake();
void shift4melevator();
void drivemodeselect();
void pumpControl();
void intakecontrol();
void intakeliftup();
void intakeliftdown();
void intakestopfunc();
void intakemanuallift();
bool isrollerstuck();
void rollerfunc();
void initial();
void preventionfunc();
void greencontrol();
void greendown();
void greenup();
void greenupquick();
void halfgreen();
void moveelevatordown();
void moveelevatorup();
void ledcontrol();
void backpurpleinital();
void purpleintakelift();
void purplegateopen();
void purplegateclose();
void part2();
void stopeverything();
void clearfield();
void changepreventfunc();
void Pid(float distance, float heading,int velocity,float kp,float timeout,bool stalldetection);
void Preciseturn(float heading,int velocity,float momentum, float timeout,bool stalldetection);
//####################################################################################

void DEBUG(){
    while(true){
        if(Controller.ButtonFUp.pressing()){
            break;
        }else{
            TouchLED1.setBlink(red,0.1);
            TouchLED2.setBlink(red,0.1);
        }
        wait(20,msec);
    }
    
}

void stopeverything(){
    intakethread.interruptAll();
    leftmotor.setStopping(coast);
    rightmotor.setStopping(coast);
    motorgroup1.setStopping(coast);
    motorgroup2.setStopping(coast);
    leftmotor.stop();
    rightmotor.stop();
    motorgroup1.stop();
    motorgroup2.stop();
    Pneumatic1.pumpOff();
    Pneumatic2.pumpOff();
}

void preventionfunc(){
    Brain.playSound(alarm2);
    Brain.playSound(alarm2);
    Brain.playSound(alarm2);
    //greendown
    // if(Distancesensor.objectDistance(mm)>45){// this is to show wether the distance is larger than a certain number proving that it is up or not
    if (green_running==true){// bring green down if it is running
        preventionfuncrunning=true;
        event_changepreventfunc.broadcast();//change prevenfuncrunning to false few seconds later

        greenthread.interrupt();
        drivemode=2;
        greencounter++;
        moveelevatordown();  
        wait(100,msec);
        
        Pneumatic1.retract(cylinder2);//elevator shifter
        Pneumatic1.extend(cylinder1);//shifter for six motor drive

        motorgroup1.setStopping(coast);//allow drivetrain use motor as coast
        motorgroup2.setStopping(coast);
        motorgroup1.stop();
        motorgroup2.stop();
        drivemode=6;
        green_running=false;
    }
}

void changepreventfunc(){
    wait(2,seconds);//change prevenfuncrunning to false few seconds later
    preventionfuncrunning=false;
}

void backpurpleinital(){
    Pneumatic2.extend(cylinder1);
    wait(0.1,seconds);
    Pneumatic2.extend(cylinder1);
    wait(0.7,seconds);
    Pneumatic2.retract(cylinder1);
}

void initial(){//used for initalization
    purplegateclose();// after last game always put purple gate down
    intakeliftdown();
    shift4mintake();
    start=false;
    drivemode = 2;
    intake_running = false;
    green_running = false;
    purple_dumping = false;
    preventionfuncrunning=false;
    pumpcounter=0;
    intakecounter=0;
    stuckcounter=0;
    greencounter=0;
    
    Led=false;
    TouchLED1.setBlink(red,0.1,0.1);//set it to red blink 
    TouchLED2.setBlink(red,0.1,0.1);
    event_pump.broadcast();//pump after all counter reset
    wait(5,seconds);
    if (Led==false){
        TouchLED1.setColor(green);
        TouchLED2.setColor(green);// set it to green after the end of initalization
    }
    
}

/******************************
Drivetrain related function
*******************************/

void shift2mdrive(){// Shift to 2 motor drive
    Brain.playSound(siren2);
    // event_pump.broadcast();
    Pneumatic1.retract(cylinder1);
    wait(shiftwaittime,msec);
    drivemode=2;
    
    motorgroup1.stop();//shift from 6 to 2, need to stop motor1group1
    motorgroup2.stop();
    //intake_running=true;
    //green_running=false;
}

void shift6mdrive(){// Shift to 6 motor drive 
    // event_pump.broadcast();
    motorgroup1.setStopping(coast);
    motorgroup2.setStopping(coast);

    Pneumatic1.extend(cylinder1);//shifter for six motor drive
    Pneumatic1.retract(cylinder2);
    Pneumatic2.extend(cylinder2);
    Pneumatic1.extend(cylinder1);//ensure that the pneumatic is extended
    wait(shiftwaittime,msec);//wait to allow penumatic to shift
    drivemode=6;
    intake_running=false;
    green_running=false;
    intakecounter=2;
}

void shift4mintake(){// Shift to 4 motor intake
    // event_pump.broadcast();
    Pneumatic2.retract(cylinder2);
    Pneumatic1.retract(cylinder2);
    Pneumatic1.retract(cylinder1);

    wait(shiftwaittime,msec);
    drivemode=2;
    intake_running=true;
    green_running=false;
}

void shift4melevator(){// Shift to 4 motor elevator
    // event_pump.broadcast();
    Pneumatic2.extend(cylinder2);//disengage intake
    Pneumatic1.extend(cylinder2);
    Pneumatic1.retract(cylinder1);
    wait(shiftwaittime,msec);
    drivemode=2;
    intake_running=false;
    green_running=true;
    intakecounter=2;
}

void drivemodeselect(){//Shift to 6 motor drive
    // Choose whether 6 motor drive or 2 drive 
    if(drivemode==6){
        shift2mdrive();
    }else{
        shift6mdrive();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// pump area

void pumpControl(){// Choose wheter the pump is on
    if(Controller.ButtonL3.pressing()&&Controller.ButtonR3.pressing()){
        pumpcounter+=1;// this increases the pumpcounter   
    }

    if (pumpcounter%2==0){//this checks and divides the pump counter
        Pneumatic2.pumpOff();
        Pneumatic1.pumpOff();
    }else{// this keeps it on forever unless stoped manually
        Pneumatic2.pumpOn();
        Pneumatic1.pumpOn();
    }
}

void pumpfunc(){
    Pneumatic1.pumpOn();
    Pneumatic2.pumpOn();

    if(intakecounter==0){// this is the first pump when the program is turned on
        wait(20,seconds);
    }else{// this is to pump when the func is called (though never called more than one)
        wait(6,seconds);    
    }
    
    if (pumpcounter==0){//only turn off pump if game has not started
        Pneumatic1.pumpOff();
        Pneumatic2.pumpOff();
    }
    
}

void movechasis (int velocity,float output){
    leftmotor.setVelocity(velocity+output,percent);
    rightmotor.setVelocity(velocity-output,percent);
    leftmotor.spin(forward);
    rightmotor.spin(forward);
    wait(20,msec);
}

void movechasis2(int velocity,float output){
    leftmotor.setVelocity(velocity+output,percent);
    rightmotor.setVelocity(velocity-output,percent);
    motorgroup1.setVelocity((velocity+output)*-1,percent);
    motorgroup2.setVelocity((velocity-output)*-1,percent);
    leftmotor.spin(forward);
    rightmotor.spin(forward);
    motorgroup1.spin(forward);
    motorgroup2.spin(forward);
    wait(20,msec);
}

bool c1 (int velocity,float distance){
    if (velocity>0){
        if(leftmotor.position(degrees)<distance){
            return true;
        }else{
            return false;
        }
    }else{
        if(leftmotor.position(degrees)>distance*-1){
            return true;
        }else{
            return false;
        }
    }
}

bool c2(float timeout){
    float timerduration;
    if(timeout==0){
        return true;
    }else{
        if(rollerstucked == false){// this is to stop the timeout when roller is stuck
            timernow = Brain.Timer.value();
        }
        timerduration = timernow - timerstart;
        if(timerduration>timeout){
            return false;
        }else{
            return true;
        }
    }
}

bool c3(bool stalldetection){//c3 is to detect whether left or right motor is stucked
    // printf("left current is %.2f\n",leftmotor.current());
    // printf("left v is %.2f\n",leftmotor.velocity(percent));
    // printf("right current is %.2f\n",rightmotor.current());
    // printf("right v is %.2f\n",rightmotor.velocity(percent));
    // printf("\n");
    if(stalldetection){
        //if((leftmotor.current() >0.8 && leftmotor.velocity(percent)==0)||(rightmotor.current() >0.8 && rightmotor.velocity(percent)==0)){//checks whether its on a wall or stuck (same logic as the is roller stuck function)
        if(leftmotor.current() >0.7 && leftmotor.velocity(percent)==0){//checks whether its on a wall or stuck (same logic as the is roller stuck function)
            Brain.playSound(tada);  
            return false;//stuck or on wall
        }else{
            return true;//not stuck or on wall
        }
    }else{
        return true;//not stuck or on wall
    }
}

void counterclockwise(){
    leftmotor.spin(reverse);
    rightmotor.spin(forward);
}

void clockwise(){
    leftmotor.spin(forward);
    rightmotor.spin(reverse);
}

void counterclockwise6m(){
    leftmotor.spin(reverse);
    rightmotor.spin(forward);
    motorgroup1.spin(reverse);
    motorgroup2.spin(forward);
}

void clockwise6m(){
    leftmotor.spin(forward);
    rightmotor.spin(reverse);
    motorgroup1.spin(forward);
    motorgroup2.spin(reverse);
}

void Preciseturn(float heading,int velocity,float momentum, float timeout,bool stalldetection){
    timerstart = Brain.Timer.value();
    if(heading>Inertial.rotation()){//this means that is is turning clockwise
        while(heading-momentum>Inertial.rotation()&&c2(timeout)&&c3(stalldetection)){
            leftmotor.setVelocity(velocity,percent);
            rightmotor.setVelocity(velocity,percent);
            clockwise();
            wait(20,msec);
        }
    }else{
        while(heading+momentum<Inertial.rotation()&&c2(timeout)&&c3(stalldetection)){
            leftmotor.setVelocity(velocity,percent);
            rightmotor.setVelocity(velocity,percent);
            counterclockwise();
            wait(20,msec);
        }
    }
    leftmotor.stop();
    rightmotor.stop();
    wait(0.1,seconds);//for momentum
}

void Preciseturn6m(float heading,int velocity,float momentum, float timeout,bool stalldetection){
    timerstart = Brain.Timer.value();
    if(heading>Inertial.rotation()){//this means that is is turning clockwise
        while(heading-momentum>Inertial.rotation()&&c2(timeout)&&c3(stalldetection)){
            leftmotor.setVelocity(velocity,percent);
            rightmotor.setVelocity(velocity,percent);
            motorgroup1.setVelocity(velocity*-1,percent);
            motorgroup2.setVelocity(velocity*-1,percent);
            clockwise6m();
            wait(20,msec);
        }
    }else{
        while(heading+momentum<Inertial.rotation()&&c2(timeout)&&c3(stalldetection)){
            leftmotor.setVelocity(velocity,percent);
            rightmotor.setVelocity(velocity,percent);
            motorgroup1.setVelocity(velocity*-1,percent);
            motorgroup2.setVelocity(velocity*-1,percent);
            counterclockwise6m();
            wait(20,msec);
        }
    }
    leftmotor.stop();
    rightmotor.stop();
    motorgroup1.stop();
    motorgroup2.stop();
    wait(0.1,seconds);//for momentum
}

void Pid6mdrive(float distance, float heading,int velocity,float kp,float timeout,bool stalldetection){
    ledcontrol();
    timerstart = Brain.Timer.value();
    float error;
    float output;
    motorgroup1.setVelocity(100,percent);
    motorgroup2.setVelocity(100,percent);
    leftmotor.setPosition(0,degrees);
    rightmotor.setPosition(0,degrees);
    motorgroup1.setPosition(0,degrees);
    motorgroup2.setPosition(0,degrees);
    leftmotor.setStopping(brake);
    rightmotor.setStopping(brake);
    while(c1(velocity,distance)&& c2(timeout)&&c3(stalldetection)){
        error = heading - Inertial.rotation();
        output = kp * error;
        if(isrollerstuck()){
            if (insupplyzone){
                motorgroup1.stop();
                motorgroup2.stop();
                leftmotor.stop();
                rightmotor.stop();
            }
            wait(0.1,seconds);
        }else{
            movechasis2(velocity,output);
        }
    }
    leftmotor.stop();
    rightmotor.stop();
    motorgroup1.stop();
    motorgroup2.stop();
    wait(20,msec);

}

void Pid(float distance, float heading,int velocity,float kp,float timeout,bool stalldetection){// manually call 2 motor drive
    timerstart = Brain.Timer.value();
    float error;
    float output;
    leftmotor.setPosition(0,degrees);
    rightmotor.setPosition(0,degrees);
    leftmotor.setStopping(brake);
    rightmotor.setStopping(brake);
    while(c1(velocity,distance)&& c2(timeout)&&c3(stalldetection)){
        error = heading - Inertial.rotation();
        output = kp * error;
        if(isrollerstuck()){
            if (insupplyzone){
                leftmotor.stop();
                rightmotor.stop();
            }
            wait(0.1,seconds);
        }else{
            movechasis(velocity,output);
        }
    }
    leftmotor.stop();
    rightmotor.stop();
    wait(20,msec);
}

void waitgreen(){
    // wait(0.5,seconds);
    while(1){
        if(motorgroup1.current()>0.6){
            leftmotor.stop();
            rightmotor.stop();
            break;
        }else{
            leftmotor.spin(forward);
            rightmotor.spin(forward);
        }
    }
}

/******************************
Intake related function
*******************************/

void supplylift(){// only use for getting in supply zone
    Pneumatic3.extend(cylinder2);  
    wait(1.7,seconds);
    Pneumatic3.retract(cylinder2);
    wait(0.1,seconds);
    Pneumatic3.retract(cylinder2);
}


void intakecontrol(){                                                                                                                                                                      
    intakecounter++;
    motorgroup1.setVelocity(100,percent);
    motorgroup2.setVelocity(100,percent);
    // start off
    if(intakecounter==1){// when the intake starts it only happens once
        motorgroup1.spin(forward);
        motorgroup2.spin(forward);
        Brain.playSound(headlightsOn);
        Led=true;// this is for when everything is ready then we give the control over to ledcontrol
        pumpcounter=1;
        pumpControl();// start the pump forever unless the pump is stopped manually
        Brain.Timer.reset();// reset the timer
        start=true;// allow the prevention func to start

        event_backinital.broadcast();

    }

    // lift   
    if(intake_running){//this replaces the counter
        while(Controller.ButtonLUp.pressing()){
            intakeliftup();
        }
        intakeliftdown();
    }

    // normal intake
    drivemode=2;//change drive before shift    
    intake_running=true;
    shift4mintake();
    intakethread = thread(rollerfunc);//start the intake thread
}

void rollerfunc(){
    while(true){
        if(intake_running){//intake is running
            if (isrollerstuck()){
                    stuckcounter++;
                    wait(0.5,seconds);
                    motorgroup1.spin(reverse);
                    motorgroup2.spin(reverse);
            }else{
                stuckcounter=0;//no longer stucked,reset stuck counter
                motorgroup1.setVelocity(100,percent);
                motorgroup2.setVelocity(100,percent);
                motorgroup1.spin(forward);   
                motorgroup2.spin(forward);
            }
        }//end of if intakerunning
        wait(20,msec);
    }//end of while
}

bool isrollerstuck(){
    if((motorgroup1.current(amp)>1.6) && motorgroup1.velocity(rpm)==0){
        Brain.playSound(alarm2);
        intakestucked=true;
        return true;
    }else{
        intakestucked=false;
        return false;
    }
}

void intakestopfunc(){
    intakethread.interrupt();

    drivemode=6;//change drive mode before shifting
    
    shift6mdrive();
    //intakecounter++;
}

void intakeliftup(){
    // event_pump.broadcast();
    Pneumatic3.extend(cylinder2);
}

void intakeliftdown(){
    // event_pump.broadcast();
    Pneumatic3.retract(cylinder2);
}


void intakemanuallift(){
    if(Controller.ButtonLDown.pressing()){//manual /liftup
        intakestopfunc();
    }
}    

/******************************
green related function
*******************************/
void greencontrol(){
    greencounter++;

    if (Controller.ButtonRDown.pressing()){//full green up, two stage
        greenupmode=1;
    }else if(Controller.ButtonFDown.pressing()){//half green
        greenupmode=2;
    }else if (Controller.ButtonEDown.pressing()){//quick green
        greenupmode=3;
    }
    
    
    greenthread.interrupt();
    motorgroup1.stop();
    motorgroup2.stop();

    

    if(greencounter%2==0){//greendown and shift to intake
        greenthread= thread(greendown);
    }else{
        shift4melevator();

        if (greenupmode==1){//full green
            greenthread=thread(greenup);
        }else if(greenupmode==2){//quick green
            greenthread=thread(halfgreen);
        }else if (greenupmode==3){
            greenthread= thread(greenupquick);
        }
    }
}

void moveelevatorup(){
    //forward: with intake
    //reverse: without intake
    motorgroup1.spin(forward);
    motorgroup2.spin(forward);
}

void moveelevatordown(){
    motorgroup1.spin(reverse);
    motorgroup2.spin(reverse);
}

void greenup(){
    printf("in greenup\n\n");
    while(true){
        motorgroup1.setVelocity(100,percent);
        motorgroup2.setVelocity(100,percent);

        if (bucketdistance<=greendistance2){
            Brain.playSound(fillup);
            if ((bucketdistance>=greendistance1-5)&&(bucketdistance<=greendistance1+5)){
                motorgroup1.setStopping(hold);
                motorgroup2.setStopping(hold);
                motorgroup1.stop();
                motorgroup2.stop();
                wait(0.5,seconds);//wait at level1
                moveelevatorup();//move up after the wait
                wait(0.5,seconds);//allow previous move up to pass greendistance1. so the the if is false, and contiune wiht level2
            }
            moveelevatorup();//continue greenup to level2
        }else{
            if(bucketdistance!=9999){//distance sensor error
                motorgroup1.setStopping(hold);
                motorgroup2.setStopping(hold);
                motorgroup1.stop();
                motorgroup2.stop();

                Pneumatic2.extend(cylinder2);//dis engage intake
                break;
            }
        }
        wait(30,msec);
    }
}

void halfgreen(){
    printf("in halfgreen\n\n");
    while(true){
        motorgroup1.setVelocity(100,percent);
        motorgroup2.setVelocity(100,percent);                                     

        if (bucketdistance<=greendistance1){                                                     
            Brain.playSound(fillup);                                                                                                       
            moveelevatorup();//dump green level1
        }else{                                                                      
            if(bucketdistance!=9999){//distance sensor error
                motorgroup1.setStopping(hold);                                                                    
                motorgroup2.setStopping(hold);                                                                    
                motorgroup1.stop();
                motorgroup2.stop();
                
                Pneumatic2.extend(cylinder2);//dis engage intake
                break;
            }
        }
        wait(30,msec);
    }
}

void greenupquick(){
    printf("in greenupquick\n\n");
    while(true){
        motorgroup1.setVelocity(100,percent);
        motorgroup2.setVelocity(100,percent);

        if (bucketdistance<=greendistance2){
            Brain.playSound(fillup);
            moveelevatorup();//dump green level2
        }else{
            if(bucketdistance!=9999){//distance sensor error
                motorgroup1.setStopping(hold);
                motorgroup2.setStopping(hold);
                motorgroup1.stop();
                motorgroup2.stop();

                Pneumatic2.extend(cylinder2);//dis engage intake
                break;
            }
        }
        wait(30,msec);
    }
}

void greendown(){
    moveelevatordown();  
    wait(0.25,seconds);
    motorgroup1.setStopping(coast);//allow drivetrain use motor as coast
    motorgroup2.setStopping(coast);
    motorgroup1.stop();
    motorgroup2.stop();

    TouchLED1.setColor(white);
    TouchLED2.setColor(white);
    drivemode=6;//change drive mode before shift
    shift6mdrive();

}

/******************************
purple related function
*******************************/

void purplegateopen(){
    Pneumatic2.extend(cylinder1);
    ispurplegateopen=true;
}

void purplegateclose(){
    Pneumatic2.retract(cylinder1);
    ispurplegateopen=false;
}

void purpledispense(){//purple dispense
    // event_pump.broadcast();
    while(1){
        if(Controller.ButtonRUp.pressing()){
            purple_dumping=true;
            purplegateopen();
        }else{
            purple_dumping=false;  
            purplegateclose();
            break;
        }
        wait(20,msec);
    }
}

void purpledispense2(){//purple dispense with intake lift
    purpletimerstart=Brain.Timer.value();
    while(1){
        if(Controller.ButtonRUp.pressing()){
            purpletimernow=Brain.Timer.value();//keeps updating purple timer while pressing purple
            if(green_running){// this is to show wether the distance is larger than a certain number proving that it is up or not
                greencounter++;
                greendown();//we need to do greendown so that it can move down automaticallyand not miss any purple's when we dump the purple 
                wait(0.25,seconds);//wait for elevator to come down
            }
            purple_dumping=true;
            purplegateopen();
            purpleintakelift();

        }else{
            purpletimernow=0;//reset timer if purple is not dumping
            purple_dumping=false;  
            purplegateclose();

            if(purple_lift){//if purpleintakelife is not lifted, no need to shift to 6 motor
                intakeliftdown();
                wait(shiftwaittime,msec);
                shift6mdrive();//always shift to 6 motor drive when finish purple;
            }
            purple_lift=false;//reset variable after each purple activation
            break;
        }
        wait(10,msec);
    }
}



void purpleintakelift(){
    float purpleduration=purpletimernow-purpletimerstart;//calculate how long purple has been pressed
    printf("duration %.2f\n",purpleduration);
    if (purpleduration>0.5){  //if purple has been holding for more than 1 second,if we use simple wait then we always have to wait and continue onto the next func
    //but if we use duration we can choose wether we wait or not if we release the Rup then we basically cut the loop 
        if(intake_running==false){
            shift4mintake();
            intakecontrol();
        }
        // else{
        //     wait(shiftwaittime,msec);
        // }
        purple_lift=true;//intake lift happend
        intakeliftup();
    }

}

void purplegatecontrol(){
    if(ispurplegateopen==true){
        purplegateclose();
    }else{
        purplegateopen();
    }
}




/******************************
LED function
*******************************/
void ledcontrol(){
    if(Led){// meaning whether the led is being used by any other function not included in the led func
        if(preventionfuncrunning){
            TouchLED1.setColor(yellow);
            TouchLED2.setColor(yellow);
        }else if(green_running){
            TouchLED1.setBlink(green,0.1,0.1);
            TouchLED2.setBlink(green,0.1,0.1);
        }else if(purple_dumping){
            TouchLED1.setBlink(purple,0.1,0.1);
            TouchLED2.setBlink(purple,0.1,0.1);
        }else if(intakestucked){
            TouchLED1.setBlink(red,0.1,0.1);
            TouchLED2.setBlink(red,0.1,0.1);
        }else if(drivemode ==2){            
            TouchLED1.setBlink(green,0.2,0.2);
            TouchLED2.setBlink(green,0.2,0.2);
        }else if(drivemode ==6){
            TouchLED1.setBlink(blue,0.1,0.1);
            TouchLED2.setBlink(blue,0.1,0.1);
        }
    }
}

void manualreverse(){
    intakethread.interrupt();
    while(Controller.ButtonFUp.pressing()){
        motorgroup1.spin(reverse);
        motorgroup2.spin(reverse);
    }
    intakethread = thread(rollerfunc);
}

void installitioncontrol(){
    if(motor1.installed()==false){
       Brain.playSound(siren2);
       printf("motor1\n");
       Brain.Screen.setCursor(3,1);
       Brain.Screen.setFillColor(red);
       Brain.Screen.print("motor1 disconnected!!");
    }
    if(motor2.installed()==false){
        Brain.playSound(siren2);
        printf("motor2\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("motor2 disconnected!!");
    }
    if(motor3.installed()==false){
        Brain.playSound(siren2);
        printf("motor3\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("motor3 disconnected!!");
        
    }
    if(motor4.installed()==false){
        Brain.playSound(siren2);
        printf("motor4\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("motor4 disconnected!!");
    }
    if(leftmotor.installed()==false){
        Brain.playSound(siren2);
        printf("left\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("left disconnected!!");
    }
    if(rightmotor.installed()==false){
        Brain.playSound(siren2);
        printf("right\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("right disconnected!!");
    }
    if(Pneumatic1.installed()==false){
        Brain.playSound(siren2);
        printf("p1\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("p1 disconnected!!");
    }
    if(Pneumatic2.installed()==false){
        Brain.playSound(siren2);
        printf("p2\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("p2 disconnected!!");
    }
    if(Pneumatic3.installed()==false){
        Brain.playSound(siren2);
        printf("p3\n");
        Brain.Screen.setCursor(3,1);
        Brain.Screen.setFillColor(red);
        Brain.Screen.print("p3 disconnected!!");
    }
}
/******************************
Action function
*******************************/
void test (){ // used for testing
    shift6mdrive();
    //Pid6mdrive(2550,0,85,0.5,0,true);
    //Preciseturn6m(-280,100,0,1,true);
}


void clearfield(){
    Brain.Timer.reset();
    event_backinital = event (backpurpleinital);
    
    
    kp = 1.9;
    kp6mdrive = 0.5;
    turnvelocity = 50;
    intakethread = thread (intakecontrol);
    wait(0.1,seconds);
    /*step1*/
    Pid(550,0,velocity,kp,0,false);//get greens
    
    /***********
     * Step2
    ************/
    Preciseturn(43,turnvelocity,24,0,false);
    //wait(0.3,seconds);
    
    Pid(300,39,velocity-20,kp,0,false);
    Pid(300,44,velocity-30,kp,0,false);
    Pid(285,40,velocity-30,kp,0,false);//get 2 purple
    wait(0.7,seconds);
    Pid(350,43,velocity-30,kp,0,false);//get first flower
    wait(1,seconds);
    // waitgreen();
    Pid(170,43,velocity-40,kp,0,true);//get rest of first flower
    wait(0.8,seconds);
    Pid(425,20,velocity*-1,kp,0,true);//backup

    /***********
     * Step4
    ************/
    Pid(150,-45,velocity,kp,0,false);//get second flower 
    Pid(250,-54,velocity,kp,1.5,false);//turn  counter clock to get out second flow    
    /***********
     * Step5
    ************/
    Pid(500,-145,velocity,kp,0,false);//leave to get third flower
    Pid(600,-145,velocity,kp,0,true);//leave to get third flower
    wait(0.2,seconds);
    Preciseturn(-210,100,23,0,true);
    shift6mdrive();
    /***********
     * Step6
    ************/
    Pid6mdrive(1500,-210,-100,kp6mdrive,0,true);//drive to dump purple                                                       
    leftmotor.setStopping(hold);    
    rightmotor.setStopping(hold);   
    leftmotor.stop();                                                                           
    rightmotor.stop();                                                      
    
    purplegateopen();
    intakeliftup();
    wait(1,seconds);
    intakeliftdown();

    /***********
     * Step7
    ************/
    //shift6mdrive();
    Pid6mdrive(1725,-227,velocity6mdrive,kp6mdrive,0,false);
    purplegateclose();
    wait(200,msec);
    Preciseturn6m(-285,100,0,1,true);//turn to wall
    printf("rotatoin: %.2f\n",Inertial.rotation(degrees));
    Pid6mdrive(1000,-285,velocity6mdrive*-1,kp6mdrive,0,true);//drive back to dump green;
    shift4melevator();
    event_greenup.broadcastAndWait();
    wait(0.7,seconds);
    leftmotor.setStopping(hold);
    rightmotor.setStopping(hold);
    leftmotor.stop();
    rightmotor.stop();
    
    programtimer=Brain.Timer.value();
    printf("Part 1 timer:%.2fs\n",programtimer);//part 1: 30s
    
    part2();

}

void part2(){
    /***********
     * Step8
    ************/
   int supplyzonevelocity = 65;
    //DEBUG();
    //Brain.Timer.reset();
    //Brain.Timer.reset();
    Inertial.setRotation(-270,degrees);
    shift6mdrive();
    Pid6mdrive(1250,-281.5,velocity6mdrive,kp6mdrive,0,false);//long curve 1

    intakethread = thread (intakecontrol);
    /***********
     * Step9
    ************/
    Pid(1300,-349,velocity,kp,0,false);
    Preciseturn(-260,30,25,0,false);
    intakeliftup();
    Pid(300,-268.5,55,kp,2.35,false);//enter supply zone
    intakeliftdown();
    programtimer = Brain.Timer.value();
    printf("time entering supply zone : %.2fs\n",programtimer);
    Preciseturn(-200,30,10,1.35,false);//turn to ledge to collet all cubes on side
    Pid(100,-240,supplyzonevelocity,kp,0.7,false);
    wait(0.6,seconds);
    Pid(50,-245,velocity*-1,kp,0.8,false);
    Pid(140,-246,supplyzonevelocity,kp,1.3,false);
    wait(0.2,seconds);// wait to collect a bit
    Pid(80,-260,-60,kp,1.3,false);
    Pid(155,-265,velocity,kp,1.3,false);
    wait(0.4,seconds);
    Pid(100,-267,-60,kp,1,false);
    Pid(120,-265,velocity,kp,1,false);
    wait(0.4,seconds);
    Pid(900,-265,-60,kp,1.7,false);// leave supply zone
    Preciseturn(-190,60,10,0,false);
    Pid(850,-207,velocity-30,kp,0,false);//collect flower
    Pid(1000,-300,velocity-30,kp,1.6,false);// drive to bar
    printf("The rotation is%.2f\n",Inertial.rotation());
    Preciseturn(-375,100,0,0,false);
    Pid(1000,-370,velocity*-1,kp,0,true);
    shift4melevator();
    event_greenup.broadcastAndWait();
    wait(0.5,seconds);
    programtimer = Brain.Timer.value();
    printf("end of phase 2: %2.fs\n",programtimer);    
    shift6mdrive();
    Pid6mdrive(3000,-362,60,kp6mdrive,0,false);
}

void threadcontrol(){
    autothread.interrupt();
    if(TouchLED2.pressing()){
        autothread=thread(clearfield);
    }
    else if(TouchLED1.pressing()){
        autothread=thread(clearfield);
    }else if (Controller.ButtonEUp.pressing()){
        autothread=thread(clearfield);
    }else if(Controller.ButtonRUp.pressing()){
        stopeverything();
    }else if(Controller.ButtonFDown.pressing()){
        autothread=thread(part2);
    }
    else if(Controller.ButtonEDown.pressing()){
        autothread=thread(test);
    }
}

//#########################################################################################################
int main() {
    motorgroup1.setStopping(coast);                                                                              
    motorgroup2.setStopping(coast);                                                                                                                                  
    motorgroup1.setMaxTorque(100,percent);
    motorgroup2.setMaxTorque(100,percent);
    leftmotor.setMaxTorque(100,percent);                                                                
    rightmotor.setMaxTorque(100,percent);
    leftmotor.setStopping(coast);
    rightmotor.setStopping(coast);
    leftmotor.stop();
    rightmotor.stop();
    //pneumatic starting
    initial();
    Pneumatic1.pumpOff();
    Pneumatic2.pumpOff();
    Pneumatic3.pumpOff();
    //calabration
    Inertial.calibrate();// used for supply zone
    while(Inertial.isCalibrating()){
        wait(20,msec);
        TouchLED1.setBlink(red);
        TouchLED2.setBlink(red);
    }
    //declaration
    TouchLED1.setBrightness(100);
    TouchLED2.setBrightness(100);
    TouchLED1.setColor(green);
    TouchLED2.setColor(green);
    wait(20,msec);
    
    //register button
    Controller.ButtonR3.pressed(initial);

    //Controller.ButtonLDown.pressed(test);
    Controller.ButtonRUp.pressed(stopeverything);
    Controller.ButtonEUp.pressed(threadcontrol);
    Controller.ButtonEDown.pressed(threadcontrol);
    Controller.ButtonFDown.pressed(threadcontrol);

    TouchLED1.pressed(clearfield);
    TouchLED2.pressed(clearfield);
    //register event
    event_pump(pumpfunc);
    event_backinital(backpurpleinital);
    event_changepreventfunc(changepreventfunc);
    event_greenup(greenupquick);

    //everything ready
    Brain.playSound(tada);
    Brain.Screen.setFont(mono12);
    while(1) {
        installitioncontrol();
        
        Brain.Screen.setPenColor(white);
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Auto V21-nosupply");
        Brain.Screen.setPenColor(red);
        // Brain.Screen.setCursor(3,1);
        // Brain.Screen.print("DriveMode: %d",drivemode);
        // Brain.Screen.setCursor(4,1);
        // Brain.Screen.print("intake_running: %s",intake_running?"TRUE":"FALSE");
        // Brain.Screen.setCursor(5,1);
        // Brain.Screen.print("green_running: %s",green_running?"TRUE":"FALSE");
        
        Brain.Screen.setPenColor(white);
        Brain.Screen.setCursor(6,1);
        Brain.Screen.clearLine();
        Brain.Screen.print("distance: %.2fmm",Distancesensor.objectDistance(mm));

        Brain.Screen.setPenColor(red);
        Brain.Screen.setCursor(7,1);
        Brain.Screen.clearLine();
        Brain.Screen.print("m1: %.f l: %.f RPM",motor1.velocity(rpm),leftmotor.velocity(rpm));
        // printf("left velocity%.2f right %.2f motor1 %.2f RPM\n",leftmotor.velocity(rpm),rightmotor.velocity(rpm),motor1.velocity(rpm));
        //printf("1: %.0f 2:%.0f 3:%.0f 4:%.0f\n\n",motor1.velocity(rpm),motor2.velocity(rpm),motor3.velocity(rpm),motor4.velocity(rpm));
        //printf("1: %.2f 2:%.2f 3:%.2f 4:%.2f amp\n\n",motor1.current(amp) ,motor2.current(amp),motor3.current(amp),motor4.current(amp));
        //printf("x-axis: %.2f \n",Inertial.roll(degrees));
        
        bucketdistance=Distancesensor.objectDistance(mm);
        // printf("bucketdistance %.2f\n",bucketdistance);
        
        if (start==true){//timer for prevention func
            if((Brain.Timer.value()>=58.5)&&(Brain.Timer.value()<=61)){
                preventionfunc();
                start=false;//only does preventionfunc once
            }
        }
        
        ledcontrol();

        if(Inertial.roll(degrees)<-10){
            purplegateopen();
        }
        wait(40,msec);                                          
    }                                               
}     