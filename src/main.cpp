//***********************************************************************
// Hexapod Program
// Code for Arduino Mega
// by Mark W
//***********************************************************************

//***********************************************************************
// IK and Hexapod gait references:
//  https://www.projectsofdan.com/?cat=4
//  http://www.gperco.com/2015/06/hex-inverse-kinematics.html
//  http://virtual-shed.blogspot.com/2012/12/hexapod-inverse-kinematics-part-1.html
//  http://virtual-shed.blogspot.com/2013/01/hexapod-inverse-kinematics-part-2.html
//  https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
//  http://arduin0.blogspot.com/2012/01/inverse-kinematics-ik-implementation.html?utm_source=rb-community&utm_medium=forum&utm_campaign=inverse-kinematic-equations-for-lynxmotion-3dof-legs
//***********************************************************************

//***********************************************************************
// Код принадлежит его автору, я лишь модифицировал убрал лишнее и добавил функцию реверса фемур и тибии с левой и правой стороны соответсвенно под свою кострукцию.
// Надо добавить возможность выбора пульта ДУ PS2 или NRF2401
// Так же добавить возможность использовать серводрайвер LSC-24/LSC-32 или любой другой драйвер который общается по serial
// убрать void one_leg_lift(); и добавить режим ручной калибровки или режим квадропода
// рапзобраться с амплитудой и скоростью
//***********************************************************************

//***********************************************************************
// Includes
//***********************************************************************
#define M_PI 3.141592653589793238462643  //для того что бы visual code не ругался
#define _USE_MATH_DEFINES
#include <PS2X_lib_DUE.h>   //reference: http://www.billporter.info/  //модифицированая библиотека PS2_Remote специально для DUE, но работает на MEGA
#include <Servo.h>  //Контролшь серво
#include <math.h> 
//***********************************************************************
// Constant Declarations
//**********************************************************************           
#define PS2_DAT       14  //14     //gamepad port definitions
#define PS2_CMD        15  //15
#define PS2_SEL        16  //16
#define PS2_CLK       17
#define RUMBLE  true
#define PRESSURES  false
//const int RUMBLE = true;
//const int PRESSURES = false;

//***********************************************************************
// servo port definitions
//***********************************************************************
const int COXA1_SERVO  = 41;         
const int FEMUR1_SERVO = 40;
const int TIBIA1_SERVO = 39;
const int COXA2_SERVO  = 35;
const int FEMUR2_SERVO = 34;
const int TIBIA2_SERVO = 33;
const int COXA3_SERVO  = 29;
const int FEMUR3_SERVO = 28;
const int TIBIA3_SERVO = 27;
const int COXA4_SERVO  = 8;
const int FEMUR4_SERVO = 9;
const int TIBIA4_SERVO = 10;
const int COXA5_SERVO  = 2;
const int FEMUR5_SERVO = 3;
const int TIBIA5_SERVO = 4;
const int COXA6_SERVO  = 21;
const int FEMUR6_SERVO = 20;
const int TIBIA6_SERVO = 19;
//***********************************************************************
//leg part lengths
//***********************************************************************
const int COXA_LENGTH = 63;           
const int FEMUR_LENGTH = 77;
const int TIBIA_LENGTH = 122;
//***********************************************************************
//translate and rotate const
//***********************************************************************
const int TRAVEL = 30;                //translate and rotate travel limit constant
const long A12DEG = 209440;           //12 degrees in radians x 1,000,000
const long A30DEG = 523599;           //30 degrees in radians x 1,000,000
//***********************************************************************
// frame time (20msec = 50Hz)
//***********************************************************************
const int FRAME_TIME_MS = 10;        
//***********************************************************************
// coxa-to-toe home positions
//original
//const float HOME_X[6] = {  82.0,   0.0, -82.0,  -82.0,    0.0,  82.0};  //coxa-to-toe home positions
//const float HOME_Y[6] = {  82.0, 116.0,  82.0,  -82.0, -116.0, -82.0};
//const float HOME_Z[6] = { -100.0, -100.0, -100.0,  -100.0,  -100.0, -100.0};
//***********************************************************************
const float HOME_X[6] = {  110.0,   0.0, -110.0,  -110.0,    0.0,  110.0};  
const float HOME_Y[6] = {  110.0, 144.0,  110.0,  -110.0, -144.0, -110.0};
const float HOME_Z[6] = { -130.0, -130.0, -130.0,  -130.0,  -130.0, -130.0};
//***********************************************************************
// body center-to-coxa servo distances 
//***********************************************************************
const float BODY_X[6] = { 126.5,  0.0, -126.5, -126.5,    0.0, 126.5};  
const float BODY_Y[6] = {  76.5, 80,   76.5,  -76.5,  -80, -76.5};
const float BODY_Z[6] = {   0.0,  0.0,    0.0,    0.0,    0.0,   0.0};
//***********************************************************************
// servo calibration constants
//***********************************************************************
const int COXA_CAL[6]  = {0, 0, 0,3, 0, 0};                       
const int FEMUR_CAL[6] = {2, 2,  3, 0,  1,  1};
const int TIBIA_CAL[6] = {0, 0, 0, 0,0,0};
//***********************************************************************
// gamepad variables
//***********************************************************************
int gamepad_error;                    
byte gamepad_type;
byte gamepad_vibrate;
//***********************************************************************
// frame timer variables
//***********************************************************************
unsigned long currentTime;            
unsigned long previousTime;
//***********************************************************************
// mode and control variables
//***********************************************************************
int temp;                             
int mode;
int gait;
int gait_speed;
int reset_position;
int capture_offsets;
//***********************************************************************
// inverse kinematics variables
//***********************************************************************
float L0, L3;                         
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;
//***********************************************************************
// leg lift mode variables
//***********************************************************************
int leg1_IK_control, leg6_IK_control; 
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;
//***********************************************************************
//  positioning and walking variables
//***********************************************************************
int leg_num;                          
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR, commandedS;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];
//***********************************************************************
// sequence of leg movement for gaits
//***********************************************************************
int tripod_case[6]   = {1,2,1,2,1,2};     //for tripod gait walking
int ripple_case[6]   = {2,6,4,1,3,5};     //for ripple gait
int wave_case[6]     = {1,2,3,4,5,6};     //for wave gait
int tetrapod_case[6] = {1,3,2,1,2,3};     //for tetrapod gait
//***********************************************************************
// Functions declaratrions
//***********************************************************************
void process_gamepad();
void leg_IK(int leg_number,float X,float Y,float Z);
void tripod_gait();
void wave_gait();
void ripple_gait();
void tetrapod_gait();
void compute_strides();
void compute_amplitudes();
void translate_control();
void rotate_control();
void one_leg_lift();
void set_all_90();

void print_debug();
//***********************************************************************
// Object PS2 gamepad controller
//***********************************************************************
PS2X ps2x;              
//***********************************************************************
// Object 18 servos
//***********************************************************************
Servo coxa1_servo;      
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

//***********************************************************************
// Setup func
//***********************************************************************
void setup()
{
  //***********************************************************************
  // Serial
  //***********************************************************************

  //Serial.begin(115200);
  
  //***********************************************************************
  // attach servos
  //***********************************************************************
  coxa1_servo.attach(COXA1_SERVO,610,2400);
  femur1_servo.attach(FEMUR1_SERVO,610,2400);
  tibia1_servo.attach(TIBIA1_SERVO,610,2400);
  coxa2_servo.attach(COXA2_SERVO,610,2400);
  femur2_servo.attach(FEMUR2_SERVO,610,2400);
  tibia2_servo.attach(TIBIA2_SERVO,610,2400);
  coxa3_servo.attach(COXA3_SERVO,610,2400);
  femur3_servo.attach(FEMUR3_SERVO,610,2400);
  tibia3_servo.attach(TIBIA3_SERVO,610,2400);
  coxa4_servo.attach(COXA4_SERVO,610,2400);
  femur4_servo.attach(FEMUR4_SERVO,610,2400);
  tibia4_servo.attach(TIBIA4_SERVO,610,2400);
  coxa5_servo.attach(COXA5_SERVO,610,2400);
  femur5_servo.attach(FEMUR5_SERVO,610,2400);
  tibia5_servo.attach(TIBIA5_SERVO,610,2400);
  coxa6_servo.attach(COXA6_SERVO,610,2400);
  femur6_servo.attach(FEMUR6_SERVO,610,2400);
  tibia6_servo.attach(TIBIA6_SERVO,610,2400);
  //***********************************************************************
  //connect the gamepad
  //*********************************************************************** 
  gamepad_error= ps2x.config_gamepad(PRESSURES, RUMBLE);
  //gamepad_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, PRESSURES, RUMBLE);
  if(gamepad_error == 0)      Serial.println("Controller attached");
  else if(gamepad_error == 1) Serial.println("No controller found");
  else if(gamepad_error == 2) Serial.println("Controller found but not accepting commands");
  else if(gamepad_error == 3) Serial.println("Controller refusing to enter Pressures mode");
  //***********************************************************************
  //verify the gamepad type
  //*********************************************************************** 
  gamepad_type = ps2x.readType(); 
  if(gamepad_type == 0)      Serial.println("Unknown Controller type found");
  else if(gamepad_type == 1) Serial.println("DualShock Controller found");
  else if(gamepad_type == 2) Serial.println("GuitarHero Controller found");
  else if(gamepad_type == 3) Serial.println("Wireless Sony DualShock Controller found");
  //***********************************************************************
  //turn off gamepad vibration
  //*********************************************************************** 
  gamepad_vibrate = 0;
  //***********************************************************************
  //clear offsets
  //*********************************************************************** 
  for(leg_num=0; leg_num<6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 3.0;
  //***********************************************************************
  //init mode and gait
  //*********************************************************************** 
  mode = 0;
  gait = 0;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop() 
{
  //exit if no controller found or GuitarHero controller
  if((gamepad_error == 1) || (gamepad_type == 2))
  {
    Serial.println("Invalid Controller!");
    return; 
  }

  //Timer start for framerate
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime; 
    //read controller and process inputs
    ps2x.read_gamepad(false, gamepad_vibrate);      
    process_gamepad();
    //reset legs to home position when commanded
    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false; 
    }
    //position legs using IK calculations - unless set all to 90 degrees mode
    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num,current_X[leg_num]+offset_X[leg_num],current_Y[leg_num]+offset_Y[leg_num],current_Z[leg_num]+offset_Z[leg_num]);       
    }
    //reset leg lift first pass flags if needed
    if(mode != 4) 
    {
      leg1_IK_control = true; 
      leg6_IK_control = true; 
    }
                          
    print_debug();                            //print debug data

    //process modes (mode 0 is default 'home idle' do-nothing mode)
    if(mode == 1)                             //walking mode
    {
      if(gait == 0) tripod_gait();            //walk using gait 0
      if(gait == 1) wave_gait();              //walk using gait 1
      if(gait == 2) ripple_gait();            //walk using gait 2
      if(gait == 3) tetrapod_gait();          //walk using gait 3
    }
    if(mode == 2) translate_control();        //joystick control x-y-z mode
    if(mode == 3) rotate_control();           //joystick control y-p-r mode
    if(mode == 4) one_leg_lift();             //one leg lift mode
    if(mode == 99) set_all_90();              //set all servos to 90 degrees mode
  }
  
}


//***********************************************************************
// Process gamepad controller inputs
//***********************************************************************
void process_gamepad()
{
  if(ps2x.ButtonPressed(PSB_PAD_DOWN))    //stop & select gait 0
  {
    mode = 0;
    gait = 0;
    reset_position = true;
  }
  if(ps2x.ButtonPressed(PSB_PAD_LEFT))    //stop & select gait 1 
  {
    mode = 0;
    gait = 1;
    reset_position = true;
  }
  if(ps2x.ButtonPressed(PSB_PAD_UP))      //stop & select gait 2  
  {
    mode = 0;
    gait = 2;
    reset_position = true;
  }
  if(ps2x.ButtonPressed(PSB_PAD_RIGHT))   //stop & select gait 3
  {
    mode = 0;
    gait = 3;
    reset_position = true;
  }
  if(mode == 0)                          
  {
    
  }
  if(ps2x.ButtonPressed(PSB_TRIANGLE))    //select walk mode
  {
    mode = 1;
    reset_position = true;
  }
  if(ps2x.Button(PSB_TRIANGLE))           //vibrate controller if walk button held
    gamepad_vibrate = 64; 
  else 
    gamepad_vibrate = 0;
  if(ps2x.ButtonPressed(PSB_SQUARE))      //control x-y-z with joysticks mode
  {
    mode = 2;
    reset_position = true;
  }
  if(ps2x.ButtonPressed(PSB_CIRCLE))      //control y-p-r with joysticks mode
  {
    mode = 3;
    reset_position = true;
  }
  if(ps2x.ButtonPressed(PSB_CROSS))       //one leg lift mode
  {
    mode = 4;
    reset_position = true;
  }
  if(ps2x.ButtonPressed(PSB_START))       //change gait speed
  {
    if(gait_speed == 0)
      gait_speed = 1;
    else
      gait_speed = 0;
    
    
    
  }
  if(ps2x.ButtonPressed(PSB_SELECT))      //set all servos to 90 degrees for calibration
  {
    mode = 99;   
  }
  if((ps2x.ButtonPressed(PSB_L1)) || (ps2x.ButtonPressed(PSB_R1)))
  {
    //capture offsets in translate, rotate, and translate/rotate modes
    capture_offsets = true;
  }
  if((ps2x.ButtonPressed(PSB_L2)) || (ps2x.ButtonPressed(PSB_R2)))
  {
    for(leg_num=0; leg_num<6; leg_num++)  //clear offsets
    {
      offset_X[leg_num] = 0;
      offset_Y[leg_num] = 0;
      offset_Z[leg_num] = 0;
    }
    leg1_IK_control = true;               //reset leg lift first pass flags
    leg6_IK_control = true;
    step_height_multiplier = 3.0;         //reset step height multiplier 1.0
  }
}


//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number,float X,float Y,float Z)
{
  //compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  //process only if reach is within possible range (not too long or too short!)
  if((L3 < (TIBIA_LENGTH+FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH-FEMUR_LENGTH)))  
  {
    //compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))/(2*FEMUR_LENGTH*TIBIA_LENGTH));
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia,0.0,180.0);
  
    //compute femur angle
    gamma_femur = atan2(Z,L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))/(2*FEMUR_LENGTH*L3));
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur,0.0,180.0);  

    //compute coxa angle
    theta_coxa = atan2(X,Y)*RAD_TO_DEG + COXA_CAL[leg_number];

    //output to the appropriate leg
    switch(leg_number)
    {
      case 0:
        if(leg1_IK_control == true)                       //flag for IK or manual control of leg
        {
          theta_coxa = theta_coxa + 45.0;                 //compensate for leg mounting
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa1_servo.write(int(theta_coxa));
          theta_femur = map(theta_femur, 0 ,180 , 180, 0); //reverse
          femur1_servo.write(int(theta_femur)); 
          tibia1_servo.write(int(theta_tibia)); 
        }
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa2_servo.write(int(theta_coxa)); 
        theta_femur = map(theta_femur, 0 ,180 , 180, 0); //reverse
        femur2_servo.write(int(theta_femur)); 
        tibia2_servo.write(int(theta_tibia)); 
        break;
      case 2:
        theta_coxa = theta_coxa + 135.0;                 //compensate for leg mounting
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa3_servo.write(int(theta_coxa)); 
        theta_femur = map(theta_femur, 0 ,180 , 180, 0); //reverse
        femur3_servo.write(int(theta_femur)); 
        tibia3_servo.write(int(theta_tibia)); 
        break;
      case 3:
        if(theta_coxa < 0)                                //compensate for leg mounting
          theta_coxa = theta_coxa + 225.0;                // (need to use different
        else                                              //  positive and negative offsets 
          theta_coxa = theta_coxa - 135.0;                //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa4_servo.write(int(theta_coxa)); 
        femur4_servo.write(int(theta_femur)); 
        theta_tibia = map(theta_tibia, 0 ,180 , 180, 0); //reverse
        tibia4_servo.write(int(theta_tibia)); 
        break;
      case 4:
        if(theta_coxa < 0)                                //compensate for leg mounting
          theta_coxa = theta_coxa + 270.0;                // (need to use different
        else                                              //  positive and negative offsets 
          theta_coxa = theta_coxa - 90.0;                 //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa,0.0,180.0);
        coxa5_servo.write(int(theta_coxa)); 
        femur5_servo.write(int(theta_femur)); 
        theta_tibia = map(theta_tibia, 0 ,180 , 180, 0);//reverse
        tibia5_servo.write(int(theta_tibia)); 
        break;
      case 5:
        if(leg6_IK_control == true)                       //flag for IK or manual control of leg
        {
          if(theta_coxa < 0)                              //compensate for leg mounting
            theta_coxa = theta_coxa + 315.0;              // (need to use different
          else                                            //  positive and negative offsets 
            theta_coxa = theta_coxa - 45.0;               //  due to atan2 results above!)
          theta_coxa = constrain(theta_coxa,0.0,180.0);
          coxa6_servo.write(int(theta_coxa)); 
          femur6_servo.write(int(theta_femur)); 
          theta_tibia = map(theta_tibia, 0 ,180 , 180, 0);//reverse
          tibia6_servo.write(int(theta_tibia)); 
        }
        break;
    }
  }
}


//***********************************************************************
// Tripod Gait
// Group of 3 legs move forward while the other 3 legs provide support
//***********************************************************************
void tripod_gait()
{
  //read commanded values from controller
  commandedX = map(ps2x.Analog(PSS_RY),0,255,127,-127);
  commandedY = map(ps2x.Analog(PSS_RX),0,255,-127,127);
  commandedR = map(ps2x.Analog(PSS_LX),0,255,127,-127);
  commandedS = map(ps2x.Analog(PSS_LY),0,255,540,1620);  //read left Y for ciontrol speed
    
  //if commands more than deadband then process
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); //total ticks divided into the two cases
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tripod_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tripod_case[leg_num] = 2;
          break;
        case 2:                               //move foot back (on the ground)
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tripod_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Wave Gait
// Legs move forward one at a time while the other 5 legs provide support
//***********************************************************************
void wave_gait()
{
  //read commanded values from controller
  commandedX = map(ps2x.Analog(PSS_RY),0,255,127,-127);
  commandedY = map(ps2x.Analog(PSS_RX),0,255,-127,127);
  commandedR = map(ps2x.Analog(PSS_LX),0,255,127,-127);
  commandedS = map(ps2x.Analog(PSS_LY),0,255,540,1620); //read left Y for ciontrol speed

  //if commands more than deadband then process
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); //total ticks divided into the six cases
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(wave_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) wave_case[leg_num] = 6;
          break;
        case 2:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 1;
          break;
        case 3:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 2;
          break;
        case 4:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) 
            wave_case[leg_num] = 3;
          break;
        case 5:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 4;
          break;
        case 6:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num] = 5;
          break;
      }
    }
    //increment tick
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Ripple Gait
// Left legs move forward rear-to-front while right also do the same,
// but right side is offset so RR starts midway through the LM stroke
//***********************************************************************
void ripple_gait()
{
  //read commanded values from controller
  commandedX = map(ps2x.Analog(PSS_RY),0,255,127,-127);
  commandedY = map(ps2x.Analog(PSS_RX),0,255,-127,127);
  commandedR = map(ps2x.Analog(PSS_LX),0,255,127,-127);
   commandedS = map(ps2x.Analog(PSS_LY),0,255,540,1620); //read left Y for ciontrol speed

  //if commands more than deadband then process
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); //total ticks divided into the six cases
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(ripple_case[leg_num])
      {
        case 1:                               //move foot forward (raise)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 2;
          break;
        case 2:                               //move foot forward (lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*(numTicks+tick)/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 3;
          break;
        case 3:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 4;
          break;
        case 4:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 5;
          break;
        case 5:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 6;
          break;
        case 6:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }  
}


//***********************************************************************
// Tetrapod Gait
// Right front and left rear legs move forward together, then right  
// rear and left middle, and finally right middle and left front.
//***********************************************************************
void tetrapod_gait()
{
  //read commanded values from controller
  commandedX = map(ps2x.Analog(PSS_RY),0,255,127,-127);
  commandedY = map(ps2x.Analog(PSS_RX),0,255,-127,127);
  commandedR = map(ps2x.Analog(PSS_LX),0,255,127,-127);
  commandedS = map(ps2x.Analog(PSS_LY),0,255,540,1620); //read left Y for ciontrol speed
  //if commands more than deadband then process
  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick>0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0); //total ticks divided into the three cases
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tetrapod_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 2;
          break;
        case 2:                               //move foot back one-half (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 3;
          break;
        case 3:                               //move foot back one-half (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if(tick < numTicks-1) tick++;
    else tick = 0;
  } 
}


//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  //compute stride lengths
  strideX = 70*commandedX/127;
  strideY = 70*commandedY/127;
  strideR = 25*commandedR/127;

  //compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  //set duration for normal and slow speed modes
  if(gait_speed == 0) duration = commandedS; //1080 // left Y  = duration
  //if(gait_speed == 1) duration = 720;
  //if(gait_speed == 2) duration = 900;
  //if(gait_speed == 3) duration = 1080;
  else duration = 2160;//3240
}


//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  //compute total distance from center of body to toe
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  //compute rotational offset
  rotOffsetX = totalY*sinRotZ + totalX*cosRotZ - totalX;
  rotOffsetY = totalY*cosRotZ - totalX*sinRotZ - totalY;

  //compute X and Y amplitude and constrain to prevent legs from crashing into each other
  amplitudeX = ((strideX + rotOffsetX)/2.0);
  amplitudeY = ((strideY + rotOffsetY)/2.0);
  amplitudeX = constrain(amplitudeX,-50,50);
  amplitudeY = constrain(amplitudeY,-50,50);

  //compute Z amplitude
  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) /4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}
      

//***********************************************************************
// Body translate with controller (xyz axes)
//***********************************************************************
void translate_control()
{
  //compute X direction move
  translateX = map(ps2x.Analog(PSS_RY),0,255,-2*TRAVEL,2*TRAVEL);
  for(leg_num=0; leg_num<6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;
    
  //compute Y direction move
  translateY = map(ps2x.Analog(PSS_RX),0,255,2*TRAVEL,-2*TRAVEL);
  for(leg_num=0; leg_num<6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  //compute Z direction move
  translateZ = ps2x.Analog(PSS_LY);
  if(translateZ > 127)
    translateZ = map(translateZ,128,255,0,TRAVEL); 
  else
    translateZ = map(translateZ,0,127,-3*TRAVEL,0);    
  for(leg_num=0; leg_num<6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;

  //lock in offsets if commanded
  if(capture_offsets == true)
  {
    for(leg_num=0; leg_num<6; leg_num++)
    {
      offset_X[leg_num] = offset_X[leg_num] + translateX;
      offset_Y[leg_num] = offset_Y[leg_num] + translateY;
      offset_Z[leg_num] = offset_Z[leg_num] + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// Body rotate with controller (xyz axes)
//***********************************************************************
void rotate_control()
{
  //compute rotation sin/cos values using controller inputs
  sinRotX = sin((map(ps2x.Analog(PSS_RX),0,255,A12DEG,-A12DEG))/1000000.0);
  cosRotX = cos((map(ps2x.Analog(PSS_RX),0,255,A12DEG,-A12DEG))/1000000.0);
  sinRotY = sin((map(ps2x.Analog(PSS_RY),0,255,A12DEG,-A12DEG))/1000000.0);
  cosRotY = cos((map(ps2x.Analog(PSS_RY),0,255,A12DEG,-A12DEG))/1000000.0);
  sinRotZ = sin((map(ps2x.Analog(PSS_LX),0,255,-A30DEG,A30DEG))/1000000.0);
  cosRotZ = cos((map(ps2x.Analog(PSS_LX),0,255,-A30DEG,A30DEG))/1000000.0);

  //compute Z direction move
  translateZ = ps2x.Analog(PSS_LY);
  if(translateZ > 127)
    translateZ = map(translateZ,128,255,0,TRAVEL); 
  else
    translateZ = map(translateZ,0,127,-3*TRAVEL,0);    

  for(int leg_num=0; leg_num<6; leg_num++)
  {
    //compute total distance from center of body to toe
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];

    //perform 3 axis rotations
    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY         - totalY*sinRotX*cosRotY                                  + totalZ*cosRotX*cosRotY                                  - totalZ;

    // Calculate foot positions to achieve desired rotation
    current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
    current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;

    //lock in offsets if commanded
    if(capture_offsets == true)
    {
      offset_X[leg_num] = offset_X[leg_num] + rotOffsetX;
      offset_Y[leg_num] = offset_Y[leg_num] + rotOffsetY;
      offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if(capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// One leg lift mode
// also can set z step height using capture offsets
//***********************************************************************
void one_leg_lift()
{
  //read current leg servo 1 positions the first time
  if(leg1_IK_control == true)
  {
    leg1_coxa  = coxa1_servo.read(); 
    leg1_femur = femur1_servo.read(); 
    leg1_tibia = tibia1_servo.read(); 
    leg1_IK_control = false;
  }

  //read current leg servo 6 positions the first time
  if(leg6_IK_control == true)
  {
    leg6_coxa  = coxa6_servo.read(); 
    leg6_femur = femur6_servo.read(); 
    leg6_tibia = tibia6_servo.read(); 
    leg6_IK_control = false;
  }

  //process right joystick left/right axis
  temp = ps2x.Analog(PSS_RX);
  temp = map(temp,0,255,45,-45);
  coxa1_servo.write(constrain(int(leg1_coxa+temp),45,135));

  //process right joystick up/down axis
  temp = ps2x.Analog(PSS_RY);
  if(temp < 117)                                //if joystick moved up
  {
    temp = map(temp,116,0,0,24);                //move leg 1
    int femur1_var = constrain(int(leg1_femur+temp),0,170);
    femur1_var = map(femur1_var, 0, 170, 170, 0);
    femur1_servo.write(femur1_var);
    tibia1_servo.write(constrain(int(leg1_tibia+4*temp),0,170));
  }
  else                                          //if joystick moved down
  {
    z_height_right = constrain(temp,140,255);   //set Z step height
    z_height_right = map(z_height_right,140,255,1,8);
  }

  //process left joystick left/right axis
  temp = ps2x.Analog(PSS_LX);
  temp = map(temp,0,255,45,-45);
  coxa6_servo.write(constrain(int(leg6_coxa+temp),45,135));

  //process left joystick up/down axis
  temp = ps2x.Analog(PSS_LY);
  if(temp < 117)                                //if joystick moved up
  {
    temp = map(temp,116,0,0,24);                //move leg 6
    femur6_servo.write(constrain(int(leg6_femur+temp),0,170));
    int tibia6_var = constrain(int(leg6_tibia+4*temp),0,170);
    tibia6_var = map(tibia6_var, 0, 170, 170, 0);
    tibia6_servo.write(tibia6_var);
  }
  else                                          //if joystick moved down
  {
    z_height_left = constrain(temp,140,255);    //set Z step height
    z_height_left = map(z_height_left,140,255,1,8);
  }

  //process z height adjustment
  if(z_height_left>z_height_right) 
    z_height_right = z_height_left;             //use max left or right value
  
  if(capture_offsets == true)                   //lock in Z height if commanded
  {
    step_height_multiplier = 1.0 + ((z_height_right - 1.0) / 3.0);
    capture_offsets = false;
  }
}


//***********************************************************************
// Set all servos to 90 degrees
// Note: this is useful for calibration/alignment of the servos
// i.e: set COXA_CAL[6], FEMUR_CAL[6], and TIBIA_CAL[6] values in  
//      constants section above so all angles appear as 90 degrees
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90+COXA_CAL[0]); 
  femur1_servo.write(90+FEMUR_CAL[0]); 
  tibia1_servo.write(90+TIBIA_CAL[0]); 
  
  coxa2_servo.write(90+COXA_CAL[1]); 
  femur2_servo.write(90+FEMUR_CAL[1]); 
  tibia2_servo.write(90+TIBIA_CAL[1]); 
  
  coxa3_servo.write(90+COXA_CAL[2]); 
  femur3_servo.write(90+FEMUR_CAL[2]); 
  tibia3_servo.write(90+TIBIA_CAL[2]); 
  
  coxa4_servo.write(90+COXA_CAL[3]); 
  femur4_servo.write(90+FEMUR_CAL[3]); 
  tibia4_servo.write(90+TIBIA_CAL[3]); 
  
  coxa5_servo.write(90+COXA_CAL[4]); 
  femur5_servo.write(90+FEMUR_CAL[4]); 
  tibia5_servo.write(90+TIBIA_CAL[4]); 
  
  coxa6_servo.write(90+COXA_CAL[5]); 
  femur6_servo.write(90+FEMUR_CAL[5]); 
  tibia6_servo.write(90+TIBIA_CAL[5]);
}



//***********************************************************************
// Print Debug Data
//***********************************************************************
void print_debug()
{
  //output IK data
//  Serial.print(int(theta_coxa));
//  Serial.print(",");
//  Serial.print(int(theta_femur));
//  Serial.print(",");
//  Serial.print(int(theta_tibia));
//  Serial.print(",");

  //output XYZ coordinates for all legs
//  for(leg_num=0; leg_num<6; leg_num++)
//  {
//    Serial.print(int(current_X[leg_num]));
//    if(leg_num<5) Serial.print(",");
//  }
//  Serial.print("  ");
//  for(leg_num=0; leg_num<6; leg_num++)
//  {
//    Serial.print(int(current_Y[leg_num]));
//    if(leg_num<5) Serial.print(",");
//  }
//  Serial.print("  ");
//  for(leg_num=0; leg_num<6; leg_num++)
//  {
//    Serial.print(int(current_Z[leg_num]));
//    if(leg_num<5) Serial.print(",");
//  }
//  Serial.print("  ");

  //display elapsed frame time (ms) and battery voltage (V)
  //currentTime = millis();
  //Serial.print("Mode:");
 // Serial.print(mode);
 // Serial.print("; ");
  //Serial.print("Gait:");
 // Serial.print(gait);
  //Serial.print("; ");
 // Serial.print(currentTime-previousTime);
 // Serial.print(",");
  //Serial.print(float(batt_voltage)/100.0); 
 // Serial.print("\n");
}