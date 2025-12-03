//******************DC Encoder Motor Control with BJT DC Motor Driver Module **************************************//
//******************Developed by Georg Mallebrein HS-Esslingen,  2023 *********************************************//
//******************Only Speed Detection with 2 Encoders (each 6 events per revolution)****************************//

//Supply the Motor from External with 0.6 V to 4 V (sense of rotation is not important)

volatile int  counterA;         // for interrupt loop
volatile int  counterB;         // for interrupt loop
volatile unsigned long revCtrA; // Revolution counter A
volatile unsigned long revCtrB; // Revolution counter B

int           millisGrid = 5;   // gridtime in ms 
unsigned long oldMillis;        // for grid generation

int           prnt = 1;         // for reducing of speed signal outputs
int           prntRedFac = 20;  // Printing every ... grid time   e.g. 20 => 5 ms grid => every 100 ms


float         n;                // Motor Shaft speed in rpm
float         nA;               // Motor Shaft speed in rpm from Interrupt A
float         nB;               // Motor Shaft speed in rpm from Interrupt B

unsigned long dMicroPlot;       // for plotting the smaller period time of A and B in "us * 0.2"

volatile unsigned long MillisWhatchDogA; // Idea: if for some time there is no interrupt in A of B then n = 0 rpm
volatile unsigned long MillisWhatchDogB;

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

  if  (millis() - MillisWhatchDogA > 100) n = 0;
  if  (millis() - MillisWhatchDogB > 100) n = 0; 
  
  //****** Monitor Program Part ***********************************************************************************//
  if (prnt >= prntRedFac) // reduces Serial Print Frequency
  {
    dMicroPlot = 0.2* min(75000,max(dMicroA_1, dMicroB_1)); 
    //Serial.print(0);
    //Serial.print(" ");
    //Serial.print(15000);
    //Serial.print(" ");
    //Serial.print(revCtrA*10);
    //Serial.print(" ");
    //Serial.print(revCtrB*10);
    //Serial.print(" ");
    //Serial.print(dMicroPlot);
    //Serial.print(millis());
    //Serial.print(" ");
    Serial.println(n);
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
