
//****************** DcMotorSpeedControlMap.ino *******************************************************************//
//******************DC Encoder Motor Control with BJT and Bridge DC Motor Driver Module ***************************//
//******************Developed by Georg Mallebrein HS-Esslingen,  2024  on Workdesk ********************************//
//******************Only Speed Detection with 2 Encoders (each 6 events per revolution)****************************//

//Supply the Motor from External with 1 V to 8 V (sense of rotation is not important for BJT + FreeWheeling)
//Supply the Motor from External with 6 V to 8 V (sense of rotationcan be changed for Bridge)


// DC-Motor Control Pin pwm
int   pwmPin = 5;  // 5 at Nano 980 Hz
//int pwmPin = 4;  // at Mega 980 Hz

// DC-Motor Control Pins Direction (only Bridge)
int         in1 = 9;  //  only for Bridge
int         in2 = 8;  //  only for Bridge


// Speed detection with interrups and 2 encoders (6 edges / rev.)
volatile int  counterA;         // for interrupt loop
volatile int  counterB;         // for interrupt loop
volatile unsigned long revCtrA; // Revolution counter A
volatile unsigned long revCtrB; // Revolution counter B

// Calculation Grid and Printing Speed variables
int           millisGrid = 5;   // gridtime in ms 
unsigned long oldMillis;        // for grid generation

int           prnt = 1;         // for reducing of speed signal outputs
int           prntRedFac = 10 ;  // Printing every ... grid time   e.g. 20 => 5 ms grid => every 100 ms
unsigned long dMicroPlot;       // for plotting the smaller period time of A and B in "us * 0.2"

volatile unsigned long MillisWhatchDogA; // Idea: if for some time there is no interrupt in A of B then n = 0 rpm
volatile unsigned long MillisWhatchDogB;

// *********************** Speed Detection and Speed Control variables *******************************************//
float         n;                // Motor Shaft speed in rpm
float         w;
float         pi = 3.1415;
float         nA;               // Motor Shaft speed in rpm from Interrupt A
float         nB;               // Motor Shaft speed in rpm from Interrupt B

// Speed Controller Variables
float nDes;  // Desired Motor Shaft Speed
float wDes;
float pwm;
float tqIPart = 0;  // [mNm]
float tqPPart = 0;  // [mNm]
float tqPreCon;     // [mNm]  precontrol torque
float tqTotalInr;   // [mNm]  // tqTotalInr from Controller => later converted to pwm 
float tqPPlusPre;   // [mNm]
float tqAvoidStop;   // [mNm]

float pGain;             // good for low speeds, could be higher at high speeds  
float iGain;             // good for low speeds, could be higher at high speeds  

bool  flagAntiWindup;           // false or true
bool  flagPwmLim;               // controller output limited ? 


float facTqMin; // factor for modification of the precontrol value if speed is unequal the desired speed
float tqPreMin; // product of both tqPreCon and factqMin

const byte  nrSp=9; // number of setpoints to be defined fefore calling function //


//float facTqMinCur[nrSp][2]  = {{-2000  ,   0.0},
//                               {  -50  ,   0.0},
//                               {  -20  ,   0.7},  //0.7
//                               {  -10  ,   0.95}, //0.95
//                               {    0  ,   1.0},
//                               {   10  ,   1.05}, //1.05 
//                               {   20  ,   1.2},  //1.2
//                               {   50  ,   1.5},  //1.5
//                               { 2000  ,   1.5}};


float facTqMinCur[nrSp][2]  = {{    0  ,   1.0},
                               {   50  ,   1.0},
                               {  100  ,   1.0}, 
                               {  150  ,   1.0},
                               {  200  ,   0.666},
                               {  250  ,   0.333}, 
                               {  300  ,   0}, 
                               {  350  ,   0}, 
                               {  400  ,   0}};



float tqPreConCur[nrSp][2] = {{   0  ,    0.056},  // values in mNm
                              {  50  ,    0.0585},
                              { 100  ,    0.061},
                              { 200  ,    0.066},
                              { 500  ,    0.082},
                              { 900  ,    0.1},
                              {1200  ,    0.11},
                              {1500  ,    0.12},
                              {1800  ,    0.13}};



  //const byte a = 8;                                                   //The Size of the X axis
  //const byte b = 4;                                                   //The size of the Y axis



  const byte a = 24;                                                   //The Size of the X axis
  const byte b = 10;                                                   //The size of the Y axis

  int xin;
  int yin;
  
  
  int xax [a] = {0, 10,  20,  35,  50,  100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 1000, 1200, 1500, 1900};   //X axis Data detailled  1900 == 1.9 mNm
  int yax [b] = {0,  1000,  2000,  4000,  6000,  8000,  10000, 12000, 14000, 16000};                                                          //Y axis Data detailled

  int zdat[b] [a] = {                                                 //Z Data for testing with pwm in per mille 
        {0,  16,  27,  40,  51,  82,  114, 142, 170, 197, 225, 253, 281, 305, 330, 358, 383, 410, 437, 460, 562, 670, 820, 1000},
        {0,  20,  31,  43,  55,  94,  126, 158, 186, 217, 245, 273, 304, 332, 360, 388, 417, 442, 466, 495, 600, 720, 870, 1050},
        {0,  23,  35,  50,  63,  102, 138, 171, 205, 237, 265, 297, 328, 360, 390, 420, 450, 480, 505, 532, 645, 770, 930, 1100},
        {0,  27,  43,  62,  78,  122, 165, 201, 241, 276, 312, 344, 379, 415, 450, 478, 510, 545, 577, 610, 745, 880, 1030, 1100},
        {0,  35,  51,  74,  94,  146, 193, 237, 280, 320, 363, 403, 442, 478, 515, 550, 590, 630, 672, 710, 852, 980, 1100, 1100},
        {0,  43,  62,  87,  110, 173, 228, 280, 327, 378, 423, 469, 516, 563, 605, 653, 697, 740, 785, 830, 950, 1050, 1100, 1100},
        {0,  52,  74,  106, 133, 209, 272, 335, 394, 449, 508, 563, 618, 673, 728, 782, 837, 892, 925, 950, 1050, 1100, 1100, 1100},
        {0,  65,  94,  129, 164, 254, 334, 409, 484, 559, 629, 699, 770, 840, 912, 950, 973, 1000, 1020, 1040, 1080, 1100, 1100, 1100},
        {0,  80,  120, 170, 220, 326, 433, 538, 640, 738, 836, 933, 972, 996, 1020, 1040, 1060, 1080, 1090, 1100, 1100, 1100, 1100, 1100},
        {0,  120, 165, 230, 290, 470, 648, 810, 975, 1050, 1070, 1080, 1085, 1090, 1095, 1100, 1100, 1100, 1100, 1100,  1100, 1100, 1100, 1100}



    };



  byte xmin = 0;                                                         //container for the X min
  byte xmax = 0;                                                         //container for the X max
  byte ymin = 0;                                                         //container for the Y min
  byte ymax = 0;                                                         //container for the Y Max




//** Variables to calculate the time of the specific pulse pattern. Every change of encoder signal is detected.****//
//** We expect 6 edges per revolution. Adapt the number of "Micros" to the number of edges per revolution if neded

volatile unsigned long oldMicroA_1;
volatile unsigned long dMicroA_1 = 1000000; // small speed to start controller
volatile unsigned long oldMicroA_2;
volatile unsigned long dMicroA_2 = 1000000; // small speed to start controller
volatile unsigned long oldMicroA_3;
volatile unsigned long dMicroA_3 = 1000000; // small speed to start controller
volatile unsigned long oldMicroA_4;
volatile unsigned long dMicroA_4 = 1000000; // small speed to start controller
volatile unsigned long oldMicroA_5;
volatile unsigned long dMicroA_5 = 1000000; // small speed to start controller
volatile unsigned long oldMicroA_6;
volatile unsigned long dMicroA_6 = 1000000; // small speed to start controller


volatile unsigned long oldMicroB_1;
volatile unsigned long dMicroB_1 = 1000000; // small speed to start controller
volatile unsigned long oldMicroB_2;
volatile unsigned long dMicroB_2 = 1000000; // small speed to start controller
volatile unsigned long oldMicroB_3;
volatile unsigned long dMicroB_3 = 1000000; // small speed to start controller
volatile unsigned long oldMicroB_4;
volatile unsigned long dMicroB_4 = 1000000; // small speed to start controller

volatile unsigned long oldMicroB_5;
volatile unsigned long dMicroB_5 = 1000000; // small speed to start controller
volatile unsigned long oldMicroB_6;
volatile unsigned long dMicroB_6 = 1000000; // small speed to start controller

int interrupcionA = 1;                           // digital Pin to receive interrupt  0  
                                                // (0 stands for Pin 2, 1 would stand for pin 3)
int interrupcionB = 0;                           // digital Pin to receive interrupt     1
                                                // (0 stands for Pin 2, 1 would stand for pin 3)
//****************** Setup Part **********************************************************************************//
void setup()
{

  pinMode(pwmPin, OUTPUT);    
  pinMode(in1, OUTPUT);  // only for Bridge
  pinMode(in2, OUTPUT);  // only for Bridge

  Serial.begin(115200);  // declare Serial interface to Monitor/Plotter
  
  attachInterrupt(interrupcionA, functionInterrupcionA,CHANGE ); //declare interrupt func. (detect both edges)
  attachInterrupt(interrupcionB, functionInterrupcionB,CHANGE ); //declare interrupt func. (detect both edges) 
}
//****************** Loop Part ************************************************************************************//
void loop()
{

if (millis()-oldMillis >= millisGrid)
  {
  //****** Speed Detection in Calculation Grid  *******************************************************************//
  //*formula for speed in rpm with micros:  60 * 1000000/(time in MicroSeconds)  => here average of 6 values for time 
  nA = (float)min(20000,6*60000000/(dMicroA_1 + dMicroA_2 + dMicroA_3 + dMicroA_4 + dMicroA_5 + dMicroA_6));  // 
  nB = (float)min(20000,6*60000000/(dMicroB_1 + dMicroB_2 + dMicroB_3 + dMicroB_4 + dMicroB_5 + dMicroB_6));  // 
  
  n = min(nA,nB); 

  // no more interrupt: set speed to zero if it seems that the speed is smaller 250 rpm
  if  (millis() - MillisWhatchDogA > 20) n = 0;//20 ms no interrupt, 6events/rev => 120 ms revol.time => nMin=500 rpm
  if  (millis() - MillisWhatchDogB > 20) n = 0; 

  w = n*pi/30;

  //***** nDes Calculation ****************************************************************************************//
  //nDes = 900*30/pi + 700*30/pi*sin((float)millis()*pi/1000);  //          <====== vary here the desired sinusoidal speed profile 
  nDes = 18000;
  //nDes = 1500;                        // for antiwindup demo = 1500 <===== change here for modifying the desired speed
  //if (millis() > 6000)  nDes = 15000;   // for antiwindup demo = 15000  (3000 with additional inertia)
  //if (millis() > 8000) nDes = 1500;  // for antiwindup demo = 1500

  wDes = nDes * pi/30;


  //*****  Speed Controller in Calculation Grid comparable to Simulink Model ********************************//

   flagAntiWindup = false;   // false or true                       <==================  change here AntiWindup

   pGain = 0.0008;       // default 0.0001              <======= vary here the pGain
   iGain = 0.005;        // default 0.001               <======= vary here the iGain
   
   tqPreCon = curveInterp(w, tqPreConCur);  //  n-depending Precontrol if wanted   wDes wa here
   
   //tqPreCon = 0;  // here the precontrol can be set to zero      <==================  change here Precontrol

   tqPPart = (wDes - w) * pGain;
   tqPPlusPre =  tqPreCon + tqPPart; // P-Part + Pre-Control

   // **************   Avoid stops by triggering a faiding pulse to start again  ****************************//
   if (w < 60) tqAvoidStop = 0.1;  else tqAvoidStop = max(0,tqAvoidStop - (max(0,50 + w-wDes)* 0.00002));
   tqPPlusPre =  tqPPlusPre + tqAvoidStop;
       

   facTqMin = curveInterp(w, facTqMinCur);
   tqPreMin = 0.7*facTqMin * tqPreCon; // default 0.7 
   if ((tqTotalInr > tqPreMin)&& (pwm < 255)|| !flagAntiWindup || ((wDes-w) < 0) && (pwm == 255))
     tqIPart = max(-1.9,min(1.9,tqIPart + min(1,max(-10,(wDes - w)*iGain))*(float)millisGrid/1000));
   tqTotalInr = max(tqPreMin, min(1.9, (tqPPlusPre + tqIPart)));  // limit tq = 1.9 mNm (last setpoint map)
   

//*****  Conversion from torque to facPwm  *******************************************************************//

xin = min(1900,max(0, tqTotalInr*1000)); //Conversion to integer   xin  Motor torque (tqTotalInr in mNm) => Xin has the unit microNm   
yin = min(15000,max(0, n)) ;             //Conversion to integer   yin  Motor speed (n)                  => yin has still the unit rpm


//axis crawler  I want to make this a custom function as its called twice
  byte axisCell = 0;                            //variable to store the Cell Number 0 indexed!!
  while(axisCell < a) {                         //wile the cell number is less than size of axis
    if(xin > xax[axisCell]) axisCell++;         //if X input is less than the value store in current cell then increment the cell number
    else {                                      //else  
      axisCell--;
      if(axisCell < 0){                         //if the cell number less than 0 
        xmin=0;                                 //Xmin set 0
        xmax=1;                                 //Xmax set 1
      }
      else {                                    //else
        xmin=axisCell;                          //Xmin is Cell number 
        xmax=(axisCell+1);                      //Xmax is Cell number + 1  
      }
      axisCell = 0;                             //Set the cell number to its default value
      break;                                    //break loop min max found
    }
  }

//axis crawler for the Y axis
  while(axisCell < b) {
    if(yin > yax[axisCell]) axisCell++;
    else {
      axisCell--;
      if(axisCell < 0){
        ymin=0;
        ymax=1;
      }
      else {
        ymin=axisCell;
        ymax=(axisCell+1);
      }
      axisCell=0;
      break;
    }
   }
   
  int q11 = zdat [ymin] [xmin];                   //store the data for each corresoinding quadrant
  int q12 = zdat [ymin] [xmax];
  int q21 = zdat [ymax] [xmin];
  int q22 = zdat [ymax] [xmax];


//**************************facPwm Output ***********************************************************************//
  pwm = min(255,0.256*bilinIntrp(xin, xax[xmin], xax[xmax], yin, yax[ymin], yax[ymax], q11, q12, q21, q22));    
 
  //pwm = 120;  // to set it to a defined value if needed

   analogWrite(pwmPin, pwm);   // pwm controls motor speed (min 0 max. 255) 

  //****** Monitor Program Part ***********************************************************************************//
  if (prnt >= prntRedFac) // reduces Serial Print Frequency
  {
    dMicroPlot = 0.2* min(75000,max(dMicroA_1, dMicroB_1)); 
    //Serial.print(0000);     // 0
    //Serial.print(" ");
    //Serial.print(4500); // 5000
    //Serial.print(" ");
    //Serial.print(revCtrA*10);
    //Serial.print(" ");
    //Serial.print(revCtrB*10);
    //Serial.print(" ");
    Serial.print(tqIPart*5000);
    Serial.print(" ");
    //Serial.print(facTqMin,2);
    //Serial.print(" ");
    Serial.print(nDes); //nDes
    Serial.print(" ");
    Serial.print(tqPreCon*5000);
    //Serial.print(" ");
    //Serial.print(tqAvoidStop*50,3);
    Serial.print(" ");
    Serial.print(tqTotalInr*5000);
    //Serial.print(" ");
    //Serial.print(pwm/2.56);
    Serial.print(" ");
    Serial.println(n,0);
  
    //Serial.println(tqTotalInr,3);
    prnt = 0;   
  }
  prnt = prnt + 1; 
  oldMillis = oldMillis + millisGrid; // counts up the time to compare with millis() for the grid-generation

  }  // end of Time Grid for controller and print algorithm

} // end void loop
//*************End of Loop Part ***********************************************************************************//


//***** Function to catch the direct interrupt, running outside the void loop *************************************//        
void functionInterrupcionA() {       // 6 edges per revolution => adapt if there is a different number 
   
  if (counterA >= 6) {counterA = 0; revCtrA++;}
    ++counterA; 
    
    MillisWhatchDogA = millis();
         
    if (counterA == 1) { dMicroA_1 = micros() -oldMicroA_1; 
                         oldMicroA_1 = oldMicroA_1 + dMicroA_1;}
    if (counterA == 2) { dMicroA_2 = micros() -oldMicroA_2;  
                         oldMicroA_2 = oldMicroA_2 + dMicroA_2;}
    if (counterA == 3) { dMicroA_3 = micros() -oldMicroA_3;  
                         oldMicroA_3 = oldMicroA_3 + dMicroA_3;}
    if (counterA == 4) { dMicroA_4 = micros() -oldMicroA_4;  
                         oldMicroA_4 = oldMicroA_4 + dMicroA_4;}
    if (counterA == 5) { dMicroA_5 = micros() -oldMicroA_5;    
                         oldMicroA_5 = oldMicroA_5 + dMicroA_5;}
    if (counterA == 6) { dMicroA_6 = micros() -oldMicroA_6;  
                         oldMicroA_6 = oldMicroA_6 + dMicroA_6;}
  }

 //***** Function to catch the direct interrupt, running outside the void loop ************************************//        
 void functionInterrupcionB() {

if (counterB >= 6) {counterB = 0; revCtrB++;}
    ++counterB; 

     MillisWhatchDogB = millis();
         
    if (counterB == 1) { dMicroB_1 = micros() -oldMicroB_1; 
                        oldMicroB_1 = oldMicroB_1 + dMicroB_1;}
    if (counterB == 2) { dMicroB_2 = micros() -oldMicroB_2;  
                        oldMicroB_2 = oldMicroB_2 + dMicroB_2;}
    if (counterB == 3) { dMicroB_3 = micros() -oldMicroB_3;  
                        oldMicroB_3 = oldMicroB_3 + dMicroB_3;}
    if (counterB == 4) { dMicroB_4 = micros() -oldMicroB_4;  
                        oldMicroB_4 = oldMicroB_4 + dMicroB_4;}
    if (counterB == 5) { dMicroB_5 = micros() -oldMicroB_5; 
                        oldMicroB_5 = oldMicroB_5 + dMicroB_5;}
    if (counterB == 6) { dMicroB_6 = micros() -oldMicroB_6;  
                         oldMicroB_6 = oldMicroB_6 + dMicroB_6;}

 
  }


int bilinIntrp(float x, float x1, float x2, float y, float y1, float y2, float q11, float q12, float q21, float q22) {

  float r1 = 0;                                             
  float r2 = 0;
  float p = 0;
  r1 = ((x2-x)/(x2-x1)*q11)+((x-x1)/(x2-x1)*q12);    //linear interpolation between the lower two quadrants along the X axis
  r2 = ((x2-x)/(x2-x1)*q21)+((x-x1)/(x2-x1)*q22);    //linear interpolation between the upper two quadrants along the X axis                   
  p = ((y2-y)/(y2-y1)*r1)+((y-y1)/(y2-y1)*r2);       //linear interpolation between the two virtual points r1 and r2 along the Y axis
  return p;                                          //return the Magic number P value of our point
}

float curveInterp( float xValue, float arrayCurve[nrSp][2]){
  float result;
  byte i = 0;
  while (i < nrSp && xValue > arrayCurve[i][0]){i++;}  // passenden Eintrag im Array suchen
  result = (((arrayCurve[i][1] - arrayCurve[i-1][1]) * ( xValue - arrayCurve[i-1][0] ))/ (arrayCurve[i][0] - arrayCurve[i-1][0] )) + arrayCurve[i-1][1];    
  return result; 
}
  
