/*

Bremner Nickisch 
Subistack 

*/

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
*/


//U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI speedoScreen(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);  //Initializing the screen. 
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI rpmScreen(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 6, /* dc=*/ 9, /* reset=*/ 7);  //Initializing the screen. 
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI speedoScreen(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9);// /* reset=*/ 8 );  
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI rpmScreen(U8G2_R0, /* cs=*/ 11, /* dc=*/ 12);// /* reset=*/ 8 );  
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI lampScreen(U8G2_R0, /* cs=*/ 6, /* dc=*/ 7);// /* reset=*/ 8 );  

/* TEST VALUES */
int test =56; 
float voltage = 10.20; 
float zts = 15.97;  //Value Assigned by zeroToSixyTime Method 
int timeT = 1958; 
float RPM = 4321.0;
double odom = 3230123.1; 
double tripOdom = 4450.1; 

int oilTemp = 125; 
int oilPress = 50; 
int waterTemp = 125; 

int const OIL_TEMP_MAX = 250; //Max Oil temp is around 250 degrees F 
int const OIL_PRESS_MAX = 100; //Max oil press of my sensor is 100PSI, which is probably well above the safe oil pressure of my system 
int const WATER_TEMP_MAX = 250; //Max water temp is 250 deg F

int fuel = 50; //percentage fuel remaining. 
float const FUEL_MAX = 15.1; //Max usable capacity of the fuel tank. 
/* END TEST VALUES */ 


//Pin declarations// 
int const VSS_IN = 22;  //Needs interupt, can be digital 
int const RPM_IN = 23;  //Needs interupt, can be digital 

int const = ROT_ENCODE_LEFT;  //Needs interupt, can be digital 
int const = ROT_ENCODE_RIGHT; //Needs interupt, can be digital 
int const = ROT_ENCODE_PRESS; //Needs interupt, can be digital 

int const FUEL_IN = 10;      //Needs to be analog
int const WATER_TEMP_IN;     //Needs to be analog
int const OIL_TEMP_IN;       //Needs to be analog
int const OIL_PRES_IN;       //Needs to be analog
int const VOLT_INl           //Needs to be analog

int const TURN_R_IN;         //Can be digital 
int const TURN_L_IN;         //Can be digital 
int const C_ENGI_IN;         //Can be digital 
int const ABS_IN;            //Can be digital 
int const HIGH_BEAMS_IN;     //Can be digital 
int const PARKING_BRAKE_IN;  //Can be digital 
int const AIRBAG_IN;         //Can be digital  
int const GAS_LIGHT_IN;      //Can be digital 
int const DOOR_FRONT_IN;     //Can be digital 
int const DOOR_REAR_IN;      //Can be digital 
int const DOOR_HATCH_IN;     //Can be digital 

unsigned long oldMillis = 0; 
volatile int millisPerCylce = 0; 
volatile int velocityPulses = 0; 
volatile int RPMPulses = 0;


//Lamps
bool turn_right = false;  /*turn signal right*/
bool turn_left = false; /*turn signal left*/
bool check_engine = false;  /*Check engine light, probably need to handle some other stuff or make it so there is a note when the CE light has come on.*/
bool ABS = false; /*ABS light is on*/
bool door_front_open = false; 
bool door_rear_open = false;
bool door_hatch_open = false;
bool high_beams = false;  /*High beams on*/
bool parking_brake = false; /*Parking break on*/ //TODO add code to flash MPH if PB is on 
bool airbag = false;  /*Airbag error*/
bool low_gas = false; /*Gas light is on. Remove bar and replace with "LOW" */
  
/* Speed = (((WHEEL_CIRC*pulses_per_cycle)/(VSS_CONST_REV))/(time_per_cycle))*MILLIS_HOUR/INCHES_MILE  */
int const RPM_CONST = 4; //NO fucking clue. Gonna have to brute force this one. 
float const RPM_MAX = 6500.0; // MAX RPM for EJ2.2 
float const VSS_CONST_REV = 4; //Sourced from interwebs. Apperently the VSS pulses 4 times per rev 
float const WHEEL_CIRC = 25.5*PI;  // diameter of wheel = * PI 
float const MILLIS_HOUR = 36000000;  //3.6 * 10^6 milliseconds per hour 
float const INCHES_MILE = 63360;  //63360 inches per mile 
//int velocity_look_up[120];
int velocity = 0; 
int rpm = 0; 

 //buffers for fprint
char speedoBuffer [50]; // This buffer is arbitrary and should be made smaller. 
char voltBuffer [50]; // This buffer is arbitrary and should be made smaller.
char ztsBuffer [50]; // This buffer is arbitrary and should be made smaller.
char timeBuffer [50]; // This buffer is arbitrary and should be made smaller.
char rpmBuffer [50]; // This buffer is arbitrary and should be made smaller.

//icons for thingy
//static const unsigned char Battery_Sprite_bits[] = {0x00, 0x42, 0xff, 0x85, 0xef, 0x85, 0xff, 0x00 }; //The battery sprite 8x8
//static const unsigned char Clock_Sprite_bits[] = { 0x38, 0x00, 0x54, 0x00, 0x92, 0x00, 0x11, 0x01, 0x71, 0x01, 0x01, 0x01, 0x82, 0x00, 0x44, 0x00, 0x38, 0x00 }; // The Clock Sprite 9x9



void setup(void) {
  /* Screen stuff */ 
  speedoScreen.begin();
  rpmScreen.begin();
  lampScreen.begin();
  speedoScreen.setContrast(0); //Set contrast uses values between 0-255. The difference between 0 and 255 is not great. 
  rpmScreen.setContrast(0);
  rpmScreen.setDisplayRotation(U8G2_R2); // rotate 180 
  //lampScreen.setDisplayRotation(U8G2_R3); //rotate 90


  /*  Pin stuff */
  pinMode(VSS_IN, INPUT);
  attachInterrupt(VSS_IN, incrementVSS, RISING);

  pinMode(RPM_IN, INPUT);
  attachInterrupt(RPM_IN, incrementRPM, RISING);  

  /*//Populate lookup table 
  for(int i=0; i = 119; i++){
    velocity_look_up[i] =  (((WHEEL_CIRC*velocityPulses)/(VSS_CONST_REV))/(100))*MILLIS_HOUR/INCHES_MILE;
    } */
  
}


/* LOOP START  */
void loop(void) {
  velocity =  (((WHEEL_CIRC*velocityPulses)/(VSS_CONST_REV))/(millisPerCylce))*MILLIS_HOUR/INCHES_MILE;
  rpm = (RPMPulses/RPM_CONST)/millisPerCylce*6000;

  millisPerCylce = millis()-oldMillis; 
  oldMillis = millis(); 
  
  velocityPulses = 0; 
  RPMPulses = 0; 

  /* test code */ 
  RPM+=10;
  if(RPM > RPM_MAX){
    RPM=0;}
 /*end test code */
  
  speedoScreen.clearBuffer();	
  rpmScreen.clearBuffer();// clear the internal memory		
  lampScreen.clearBuffer();	
  
  int sX = snprintf ( speedoBuffer, 50, "%03dMPH", test );
  int tX = snprintf ( timeBuffer, 50, "%d", timeT );
  int vX = snprintf ( voltBuffer, 50, " %d.%02dv", (int)voltage, (int)(voltage*100)%100 ); //arduino doesn't support float snprintf so we're moding by 100 to get the decimal value. 
  int dX = snprintf ( ztsBuffer, 50, "0-60: %d.%02ds", (int)zts, (int)(zts*100)%100 );
  int rX = snprintf ( rpmBuffer, 50, "%04dRPM", (int)RPM);

  printText(speedoScreen,'l' ,29,25, speedoBuffer); //Displays Speed 
  
  if(turn_left){
    printText(speedoScreen,'m' ,3,15, "LT");}  // Displays turn signal 
  if(turn_right){
  printText(speedoScreen,'m' ,112,15, "RT");}  // Displays turn signal
  
  printText(speedoScreen,'s' ,75,62, timeBuffer); //Display Time 
  printText(speedoScreen,'s' ,7,62, voltBuffer);  //Display Voltage

  if(!check_engine){  //if the check engine light is on we don't need the zero to sixty time. 
    printText(speedoScreen, 's', 29, 35, ztsBuffer);  //Display zero to 60 time 
    }
  else{
      printText(speedoScreen, 's', 29, 35, "Check Engine"); //Here's where we print zero to sixty 
    }
    printText(speedoScreen, 's', 112, 15, millisPerCylce); 
  //speedoScreen.drawXBMP(0,55,8,8,Battery_Sprite_bits);  //draw the battery image 
  //printBitmap(speedoScreen,60,54,9,9, &Clock_Sprite_bits);  //this does not work, something to do with it being in progmem 
  speedoScreen.sendBuffer(); // transfer internal memory to the display

/* 
 *  All data intended for the speedo screen must be above the send buffer line
 */
  
  rpmScreen.clearBuffer();
  printText(rpmScreen, 'l', 25, 25, rpmBuffer);

  printText(rpmScreen, 's', 112, 15, millisPerCylce); 

/* LEGACY TODO, REPLACE
  switch(door_open){  //Heres where we write the door status, probably need to allow mutiple states 
    case 1:
    printText(rpmScreen, 's', 35, 47, "Driver Door"); break;
    case 2:
    printText(rpmScreen, 's', 35, 47, "Rear Door");break;
    case 3:
    printText(rpmScreen, 's', 45, 47, "Hatch");break;
    }
    */
 
  
  if(high_beams){
    printText(rpmScreen,'m' ,3,15, "BR");}// Displays that the brights are on 
  
  if(!ABS && !airbag){  //The ABS light and the Airbag light superceed the RPM bar. 
    progressBar(rpmScreen, 25, 30, 82, 7, (int)RPM/RPM_MAX*100);
    }
  else{
    if(ABS){
      printText(rpmScreen, 's', 25, 35, "ABS");}
    if(airbag){
      printText(rpmScreen, 's', 72, 35, "Airbag");}
    }
  
  printText(rpmScreen, 's', 7, 62, odom);
  printText(rpmScreen, 's', 65, 62, "Tr:");
  printText(rpmScreen, 's', 85, 62, tripOdom);
  //rpmScreen.drawXBMP(60,54,9,9,Clock_Sprite_bits);
  rpmScreen.sendBuffer();

/* 
 *  All data intended for the speedo screen must be above the send buffer line  
 */

 lampScreen.clearBuffer();
 printText(lampScreen, 's', 0,64,"OIL-T");
 printText(lampScreen, 's', 3,8,oilTemp);
 progressBarVertical(lampScreen, 5, 12, 7, 30,(int)(((oilTemp*1.0)/(OIL_TEMP_MAX*1.0))*100));
 
 printText(lampScreen, 's', 35,64,"OIL-P");
 printText(lampScreen, 's', 38, 8, oilPress);
 progressBarVertical(lampScreen, 38, 12, 7, 30,(int)(((oilPress*1.0)/(OIL_PRESS_MAX*1.0))*100));
 
 printText(lampScreen, 's', 70,64,"H2O");
 printText(lampScreen, 's', 73, 8, waterTemp);
 progressBarVertical(lampScreen, 75, 12, 7, 30,(int)(((waterTemp*1.0)/(WATER_TEMP_MAX*1.0))*100));
 
 printText(lampScreen, 's', 100,64,"Fuel");
 printText(lampScreen, 's', 103, 8, (int)fuel);
 progressBarVertical(lampScreen, 103, 12, 7, 30, fuel);
 
 lampScreen.sendBuffer();

}
/*  LOOP END  */

void incrementVSS(){  //ISR method to increment the velocity pulses 
    velocityPulses++; 
  }

void incrementRPM(){ //ISR method to increment the RPM pulses 
 
  
    RPMPulses++;
}

void incrementRotCCW(){ //ISR method to increment the menu selection CCW
  }

void incrementRotCW(){ //ISR method to increment the menu selection CW
  }

void rotSelect(){ //ISR method to select menu item
  }

int zeroToSixyTime(){
  
  static long timeSinceLastZero; 
  static unsigned long timeAtLastZero; 
  if (timeSinceLastZero == NULL){
    timeSinceLastZero=0;}
  if(timeAtLastZero == NULL){
    timeAtLastZero=0;}

    
  timeSinceLastZero = millis()-timeAtLastZero;
    if(velocity <= 1){
       timeSinceLastZero = 0;
      timeAtLastZero = millis();
      }
    if(velocity==60 && timeSinceLastZero <= 30000){
         zts = (float)(timeSinceLastZero);
      }
  }

void checkLamps(){
  turn_right = (bool)digtalRead(TURN_R_IN);
  turn_left = (bool)digtalRead(TURN_L_IN);
  check_engine = (bool)digtalRead(C_ENGI_IN);
  ABS = (bool)digtalRead(ABS_IN);
  high_beams = (bool)digtalRead(HIGH_BEAMS_IN);
  parking_brake = (bool)digtalRead(PARKING_BRAKE_IN);
  airbag = (bool)digtalRead(AIRBAG_IN);
  low_gas = (bool)digtalRead(GAS_LIGHT_IN);
  door_front_open = (bool)digtalRead(DOOR_FRONT_IN);
  door_rear_open = (bool)digtalRead(DOOR_REAR_IN);
  door_hatch_open = (bool)digtalRead(DOOR_HATCH_IN);
  
  }

void progressBar(U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI iDisplay, int x_origin, int y_origin, int width, int depth, int percentage){ //drawiing the progress bar. 
  iDisplay.drawFrame(x_origin,y_origin,width,depth);
  iDisplay.drawBox(x_origin,y_origin,width*1.0*(percentage/100.0), depth);
}

void progressBarVertical(U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI iDisplay, int x_origin, int y_origin, int width, int depth, int percentage){ //drawiing the progress bar but vertically. This is broken at the moment //TODO FIX 
  iDisplay.drawFrame(x_origin,y_origin,width,depth);
  iDisplay.drawBox(x_origin,(int)y_origin+depth-depth*1.0*(percentage*1.0/100.0),width,y_origin+depth*1.0*(percentage/100.0) );
}

void printText(U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI iDisplay, char tSize, int x_loc, int y_loc, String value){
  switch(tSize){
    case 's':
    iDisplay.setFont(u8g2_font_profont11_tf  );
    break;
    case 'm':
    iDisplay.setFont(u8g2_font_profont15_tf );
    break;
    case 'l':
    iDisplay.setFont(u8g2_font_profont22_tf );
    break; 
  }
  iDisplay.setCursor(x_loc, y_loc);
  iDisplay.print(value);  
  
}

void printText(U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI iDisplay, char tSize, int x_loc, int y_loc, int value){
  switch(tSize){
    case 's':
    iDisplay.setFont(u8g2_font_profont11_tf  );
    break;
    case 'm':
    iDisplay.setFont(u8g2_font_profont15_tf );
    break;
    case 'l':
    iDisplay.setFont(u8g2_font_profont22_tf );
    break; 
  }
  //speedoScreen.setFont(u8g2_font_profont22_tf );
  iDisplay.setCursor(x_loc, y_loc);
  iDisplay.print(value);  
  
}

void printText(U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI iDisplay, char tSize, int x_loc, int y_loc, double value){
  char bufferVal [1000]; // This buffer is arbitrary and should be made smaller. 
  switch(tSize){
    case 's':
    iDisplay.setFont(u8g2_font_profont11_tf  );
    break;
    case 'm':
    iDisplay.setFont(u8g2_font_profont15_tf );
    break;
    case 'l':
    iDisplay.setFont(u8g2_font_profont22_tf );
    break; 
  }
  int success = snprintf ( bufferVal, 1000, "%d.%02d", (long)value, (long)(value*100)%100 );
  iDisplay.setCursor(x_loc, y_loc);
  iDisplay.print(bufferVal);  
  
}

/**
void printBitmap(U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI iDisplay, int x, int y, int siz_x, int siz_y, const unsigned char &bitmap){ //This method does not work 
  iDisplay.drawXBMP(x,y,siz_x,siz_y,bitmap);
  }
  **/
 
