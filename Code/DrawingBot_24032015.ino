/*
 *
 * DrawingBot.ino
 *
 */

#include <SPI.h>
#include <SD.h>

#define UINT_MAX 65535

//#define DEBUG

// move-modes
#define MM_STOP 0
#define MM_STRAIGHT 1
#define MM_TURN_LEFT 2
#define MM_TURN_RIGHT 3
#define MM_TURN_ON_SPOT 4

#define MOTOR_LEFT_STEP 3
#define MOTOR_LEFT_DIR 2
#define MOTOR_RIGHT_STEP 5 
#define MOTOR_RIGHT_DIR 4


// SD Stuff
File file;

// SERVO Stuff
#define SERVO_PIN A0
volatile int pwmCount = 0;
volatile int pwmSpeed = 9; // up by default


//const int STEPS_PER_TURN = 4096;	// 64 * 64
const int STEPS_PER_TURN = 2048;	// 32 * 64
const float WHEEL_DIAMETER = 35.897;
const float WHEEL_BASE = 141;  // vorher 140.5 bei MIN_STEP_DELAY = 4500

const float mmToCounts = ((float)STEPS_PER_TURN) / (WHEEL_DIAMETER * M_PI);

const int RAMP_DELTA = 10;
const int STOP_RAMP_DIST = 500;


const unsigned int MIN_STEP_DELAY = 6000;
const unsigned int MAX_STEP_DELAY = 15000;
//const unsigned int MIN_STEP_DELAY = 10000;
//const unsigned int MAX_STEP_DELAY = 14000;

const int BRESENHAM_MULTIPLIER = 10000;      // needed for proper resolution of bresenham-algo. (turn-modes)

volatile int stepperPinsLeft[4] = {6, 7, 8, 9};	// muss dit hier auch volatile sein?
volatile unsigned char stepMotorPoleLeft;	// diese beiden var. sind jetzt auch volatile da step-motor in irq getriggert wird

volatile int stepperPinsRight[4] = {2, 3, 4, 5};	// muss dit hier auch volatile sein?
volatile unsigned char stepMotorPoleRight;	// diese beiden var. sind jetzt auch volatile da step-motor in irq getriggert wird

volatile unsigned char moveMode;

volatile long currentRight, currentLeft;
volatile int  leftMotorDir, rightMotorDir;

volatile unsigned int stepDelay;    // stepDelay is timer1 ocr value

volatile long stepsToGo;	// steps to go until end of current drawing command

volatile int rampDir;


// var's for bresenham algorithmus used in turning-wide state
volatile long bresenhamRatio;
volatile long bresenhamCounter;

int myStepDelay = 1000;


int state;
int programStep;

long targetRight, targetLeft;

const int rampLength = 1000;
const int startPower = 100; // minimalwert bei der die Anfahrrampe beginnt

/* bezier segment
Vec2 bezierPoint[3];  // point[4] is defines though point[3] + delta[3]
Vec2 bezierDelta[3];
*/

/********************* stepper macros ********************/

// defined here because neededin irq-handler

#define LEFT_STEP_CCW\
  unsigned char lastPole = stepMotorPoleLeft;\
  stepMotorPoleLeft--;\
  stepMotorPoleLeft &= 3;\
  digitalWrite(stepperPinsLeft[stepMotorPoleLeft], HIGH);\
  unsigned char secondPole = stepMotorPoleLeft - 1;\
  secondPole &= 3;\
  digitalWrite(stepperPinsLeft[secondPole], HIGH);\
  digitalWrite(stepperPinsLeft[lastPole], LOW);\
  currentLeft--;

void leftStepForward(){
  // unsigned char lastPole = stepMotorPoleLeft;\
  // stepMotorPoleLeft--;\
  // stepMotorPoleLeft &= 3;\
  // digitalWrite(stepperPinsLeft[stepMotorPoleLeft], HIGH);\
  // unsigned char secondPole = stepMotorPoleLeft - 1;\
  // secondPole &= 3;\
  // digitalWrite(stepperPinsLeft[secondPole], HIGH);\
  // digitalWrite(stepperPinsLeft[lastPole], LOW);\
  // currentLeft--;  
  
  // set direction
  digitalWrite(MOTOR_LEFT_DIR, LOW); 

  // make a step
  digitalWrite(MOTOR_LEFT_STEP, HIGH); 
  delayMicroseconds(myStepDelay); 
  digitalWrite(MOTOR_LEFT_STEP, LOW); 
  delayMicroseconds(myStepDelay); 



}

#define LEFT_STEP_CW\
  unsigned char lastPole = stepMotorPoleLeft;\
  stepMotorPoleLeft++;\
  stepMotorPoleLeft &= 3;\
  digitalWrite(stepperPinsLeft[stepMotorPoleLeft], HIGH);\
  unsigned char secondPole = stepMotorPoleLeft + 1;\
  secondPole &= 3;\
  digitalWrite(stepperPinsLeft[secondPole], HIGH);\
  digitalWrite(stepperPinsLeft[lastPole], LOW);\
  currentLeft++;

void leftStepBackward(){
  // unsigned char lastPole = stepMotorPoleLeft;\
  // stepMotorPoleLeft++;\
  // stepMotorPoleLeft &= 3;\
  // digitalWrite(stepperPinsLeft[stepMotorPoleLeft], HIGH);\
  // unsigned char secondPole = stepMotorPoleLeft + 1;\
  // secondPole &= 3;\
  // digitalWrite(stepperPinsLeft[secondPole], HIGH);\
  // digitalWrite(stepperPinsLeft[lastPole], LOW);\
  // currentLeft++;

  // set direction
  digitalWrite(MOTOR_LEFT_DIR, HIGH); 

  // make a step
  digitalWrite(MOTOR_LEFT_STEP, HIGH); 
  delayMicroseconds(myStepDelay); 
  digitalWrite(MOTOR_LEFT_STEP, LOW); 
  delayMicroseconds(myStepDelay); 
}

#define RIGHT_STEP_CCW\
  unsigned char lastPole = stepMotorPoleRight;\
  stepMotorPoleRight--;\
  stepMotorPoleRight &= 3;\
  digitalWrite(stepperPinsRight[stepMotorPoleRight], HIGH);\
  unsigned char secondPole = stepMotorPoleRight - 1;\
  secondPole &= 3;\
  digitalWrite(stepperPinsRight[secondPole], HIGH);\
  digitalWrite(stepperPinsRight[lastPole], LOW);\
  currentRight--;

void rightStepForward(){
  // unsigned char lastPole = stepMotorPoleRight;\
  // stepMotorPoleRight--;\
  // stepMotorPoleRight &= 3;\
  // digitalWrite(stepperPinsRight[stepMotorPoleRight], HIGH);\
  // unsigned char secondPole = stepMotorPoleRight - 1;\
  // secondPole &= 3;\
  // digitalWrite(stepperPinsRight[secondPole], HIGH);\
  // digitalWrite(stepperPinsRight[lastPole], LOW);\
  // currentRight--;

  // set direction
  digitalWrite(MOTOR_RIGHT_DIR, HIGH); 

  // make a step
  digitalWrite(MOTOR_RIGHT_STEP, HIGH); 
  delayMicroseconds(myStepDelay); 
  digitalWrite(MOTOR_RIGHT_STEP, LOW); 
  delayMicroseconds(myStepDelay); 
}

#define RIGHT_STEP_CW\
  unsigned char lastPole = stepMotorPoleRight;\
  stepMotorPoleRight++;\
  stepMotorPoleRight &= 3;\
  digitalWrite(stepperPinsRight[stepMotorPoleRight], HIGH);\
  unsigned char secondPole = stepMotorPoleRight + 1;\
  secondPole &= 3;\
  digitalWrite(stepperPinsRight[secondPole], HIGH);\
  digitalWrite(stepperPinsRight[lastPole], LOW);\
  currentRight++;  

void rightStepBackward(){
  // unsigned char lastPole = stepMotorPoleRight;\
  // stepMotorPoleRight++;\
  // stepMotorPoleRight &= 3;\
  // digitalWrite(stepperPinsRight[stepMotorPoleRight], HIGH);\
  // unsigned char secondPole = stepMotorPoleRight + 1;\
  // secondPole &= 3;\
  // digitalWrite(stepperPinsRight[secondPole], HIGH);\
  // digitalWrite(stepperPinsRight[lastPole], LOW);\
  // currentRight++;    

  // set direction
  digitalWrite(MOTOR_RIGHT_DIR, LOW); 

  // make a step
  digitalWrite(MOTOR_RIGHT_STEP, HIGH); 
  delayMicroseconds(myStepDelay); 
  digitalWrite(MOTOR_RIGHT_STEP, LOW); 
  delayMicroseconds(myStepDelay); 
}


// Macros für synchronisierten Zugriff auf 16 bit Register

// Schreiben: erst high dann low
// Lesen: erst low dann high

#define GET_TIMER1_ICP(val)\
val = ((uint8_t)ICR1L);\
val += ((uint8_t)ICR1H) << 8;

//timer 1 counter value
#define SET_TIMER1(val)\
TCNT1H = (uint8_t)(val >> 8);\
TCNT1L = (uint8_t)(val & 0xff);

#define GET_TIMER1(val)\
val = ((uint8_t)TCNT1L);\
val += ((uint8_t)TCNT1H) << 8;

//timer 1 counter value
#define SET_TIMER1_OCRA(val)\
OCR1AH = (uint8_t)(val >> 8);\
OCR1AL = (uint8_t)(val & 0xff);

#define GET_TIMER1_OCRA(val)\
val = ((uint8_t)OCR1AL);\
val += ((uint8_t)OCR1AH) << 8;



/****************************************************************
*
*		irq-handlers
*
*****************************************************************/

ISR (TIMER1_COMPA_vect)
{
  SET_TIMER1_OCRA(stepDelay)

  if (moveMode == MM_STOP) return;
  
  bool leftStep = false;
  bool rightStep = false;
  
 //  switch(moveMode)
 //  {
 //      // ---------------------- state driving -----------------------
		
	// case MM_STRAIGHT:
 //    {		
 //           if (leftMotorDir > 0)  // (rightMotorDir == leftMotorDir)
 //             stepsToGo = min(targetRight - currentRight, targetLeft - currentLeft);		// in der stepper-version reichts eigentlich auch eine seite abzufragen
 //           else 
 //             stepsToGo = -max(targetRight - currentRight, targetLeft - currentLeft);
				        
 //           if(stepsToGo <= 0)
 //           {
	//       moveMode = MM_STOP;
	// 	   } else {

	// 		leftStep = true;
	// 		rightStep = true;
	// 	}
 //    }
	// break;

 //    // ---------------------- state turning right -----------------------

 //        case MM_TURN_RIGHT:
 //        {
 //          stepsToGo = targetLeft - currentLeft;
 //          if (targetLeft < 0) stepsToGo = -stepsToGo;
     
 //          if(stepsToGo <= 0){
 //            moveMode = MM_STOP;          
 //          }
 //          else 
 //          {
	// 		leftStep = true;
 //            bresenhamCounter += BRESENHAM_MULTIPLIER;
 //            if (bresenhamCounter > bresenhamRatio)		// TODO eigentlich >= ?
 //            {
 //              rightStep = true;
 //              bresenhamCounter -= bresenhamRatio;  
 //            }            
 //          }
 //        }
 //        break;


 //    // ---------------------- state turning left -----------------------

 //        case MM_TURN_LEFT:
 //        {
 //          stepsToGo = targetRight - currentRight;
 //          if (targetRight < 0) stepsToGo = -stepsToGo;
          
 //          if(stepsToGo <= 0){
 //            moveMode = MM_STOP;
 //          }
 //          else 
 //          {
	// 		rightStep = true;
 //            bresenhamCounter += BRESENHAM_MULTIPLIER;
 //            if (bresenhamCounter > bresenhamRatio) {	// TODO eigentlich >= ?
 //              leftStep = true;
 //              bresenhamCounter -= bresenhamRatio;
 //            }           
 //          }
 //        }
 //        break;

      
 //        // ------------------- state turning on spot --------------------
      
 //        case MM_TURN_ON_SPOT:
 //        {  
 //          stepsToGo = targetLeft - currentLeft;
 //          if (leftMotorDir < 0) stepsToGo = -stepsToGo;
                 
 //          if (stepsToGo <= 0)
 //          {
 //            moveMode = MM_STOP;          
 //          }
 //          else
 //          {
	// 		leftStep = true;
	// 		rightStep = true;
 //          }
 //        }  
 //        break;
      
 //      }  // end of switch

      
	  
	  // actually move step motors
	  
	 //  if (leftStep)
	 //  {
		// if (leftMotorDir > 0)
		// {
		// 	LEFT_STEP_CW
		// }
		// else
		// {
		// 	LEFT_STEP_CCW
		// }  	  
	 //  }
	  
	 //  if (rightStep)
	 //  {
		// if (rightMotorDir > 0)
		// {
		// 	RIGHT_STEP_CW
		// }
		// else
		// {
		// 	RIGHT_STEP_CCW
		// } 
	 //  }  
}

ISR (TIMER2_OVF_vect)
{
  if(pwmCount == 0){
    digitalWrite(SERVO_PIN, HIGH);
  }
  else if (pwmCount == pwmSpeed){
    digitalWrite(SERVO_PIN, LOW);
  }
  pwmCount++;
  if(pwmCount == 150){pwmCount = 0;}


	// if (moveMode == MM_STOP) return;
	
	// if (rampDir < 0)
	// {
	// 	if (stepDelay > MIN_STEP_DELAY) stepDelay -= RAMP_DELTA;
	// 	else rampDir = 0;
	// }
	// else if (rampDir == 0)
	// {
	// 	if (stepsToGo < STOP_RAMP_DIST) rampDir = 1;
	// }
	// else
	// {
	// 	if (stepDelay < MAX_STEP_DELAY) stepDelay += RAMP_DELTA;
	// }

}

// evtl. noch für servo nötig:
// ISR (TIMER2_COMPA_vect)
// {

// }


/****************************************************************
*
*		initialization functions
*
*****************************************************************/


/************* init timer1 *************/

// here the stepper motors are controlled


/*
Timer1 prescaler values:

code		div
 5			1/1024
 4			1/256
 3			1/64
 2			1/8
 1			1/1
 0			stop
*/

// bei 16MHz und full-speed gibts bei compare-value von 5000 etwa 3200 Hz, die mindestfrequens beträgt 250hz
// also doch prescaler 1/8 ...

void initTimer1()
{
	SET_TIMER1(0);	// (nicht unbedingt nötig)

	// set timer-control registers
	TCCR1A = 0;	// op-mode 4 (WGM11 / WGM10 = 0)

	TCCR1B = (1 << WGM12) | (1 << CS11);	// op-mode = 4 (WGM12 = 1), prescaler = 2 -> 1 / 8

	#ifdef	TCCR1C
	// doku: must be set to zero for compatibility with futrure devices when writing TCCR1A using pwm-mode
	TCCR1C = 0;
	#endif

        SET_TIMER1_OCRA(10000)        

	// allow interrupt on output-compare A
	uint8_t irqflags = (1 << OCIE1A);
	TIMSK1 = irqflags;
	TIFR1 = irqflags;			// (clear corresponding irq-flag...just in case)
}


/************* init timer2 *************/

// timer 2 is used for speed control - sets frequency of timer1 irq

/*
Timer2 prescaler values:

code		div
 7			1/1024
 6			1/256
 5			1/128
 4			1/64
 3			1/32
 2			1/8
 1			1/1
 0			stop
*/

void initTimer2()
{	
	TCCR2A = 0; 	// normal mode
	// TCCR2B = (1 << CS22) | (1 << CS20);  // set preescaler to 128 ~ 488 Hz
 TCCR2B = (1 << CS21);  // set preescaler to 8
	TCNT2 = 0;

	uint8_t irqflags = (1 << TOIE2);		// timer2 overflow irq
	TIMSK2 = irqflags;
	TIFR2 = irqflags;
}


//****************** stepper motor *******************/

void setupStepMotors()
{
  for (int i=0; i<4; i++)
  {
    pinMode(stepperPinsLeft[i], OUTPUT);
    digitalWrite(stepperPinsLeft[i], LOW);
	
    pinMode(stepperPinsRight[i], OUTPUT);
    digitalWrite(stepperPinsRight[i], LOW);
  }
  
  stepMotorPoleLeft = 0;
  stepMotorPoleRight = 0;
}


void leftStep(int dir)
{
  if (dir > 0)
  {
    LEFT_STEP_CW
  }
  else
  {
    LEFT_STEP_CCW
  }  
}


void rightStep(int dir)
{
  if (dir > 0)
  {
    RIGHT_STEP_CW
  }
  else
  {
    RIGHT_STEP_CCW
  }  
}


/*********************** arduino setup **************************/

void setup(){

  // Servo
  pinMode(A0, OUTPUT);
   
	Serial.begin(38400);
  
  setupStepMotors();

  stepDelay = MAX_STEP_DELAY;
  moveMode = MM_STOP;
  // initTimer1();
  initTimer2();

// SD Card
  pinMode(10, OUTPUT);
  if(!SD.begin(10)){
    Serial.println("could not access sd card");
  }
  
  file = SD.open("path2.bin");

  if(!file){
      // if the file didn't open, print an error:
    Serial.println("error opening path.txt");
  }

  penUp();

/*
	 // encoder 0 
	pinMode(ENC_0_A,		INPUT); 
	//digitalWrite(ENC_0_A,	HIGH);       // turn on pullup resistor
	pinMode(ENC_0_B,		INPUT); 
	//digitalWrite(ENC_0_B,	HIGH);       // turn on pullup resistor
*/
	

        programStep = 0;
}

/****************************************************************
*
*		functions used during run (set/reset etc.)
*
*****************************************************************/

void startMove(int mode)
{
	//stepDelay = MAX_STEP_DELAY;
        currentRight = 0;
        currentLeft = 0;
	rampDir = -1;
	moveMode = mode;
}

void drive(float distance){
        
        if (distance >= 0)
        {
          leftMotorDir = 1;
          rightMotorDir = 1;
        }
        else
        {
          leftMotorDir = -1;
          rightMotorDir = -1;
        }

	targetRight = (long)(mmToCounts * distance);
	targetLeft = (long)(mmToCounts * distance);

        startMove(MM_STRAIGHT);
}


void turn(float radius, float distance)
{
  if (abs(radius) < 0.1)
  {
     turnOnSpot(distance * 360.0 / (2.0 * M_PI * radius));
     return;
  }
  
  targetRight = 0;
  targetLeft = 0;
  bresenhamCounter = 0;


  int turnSign = 1;
  if (radius < 0)
  {
     radius = -radius;
     turnSign = -1;
  }

  if (distance >= 0)
  {
    leftMotorDir = 1;
    rightMotorDir = 1;
  }
  else
  {
    distance = -distance;
    leftMotorDir = -1;
    rightMotorDir = -1;
  }
  
  float outerWheelDistance = distance * (radius + 0.5 * WHEEL_BASE) / radius;
  
  float speedRatio = (radius + 0.5 * WHEEL_BASE) / (radius - 0.5 * WHEEL_BASE); // hier kann div/0 auftreten, außerdem werte die den int (und sogar long) bereich übersteigen können
  if (speedRatio < 0.0)
  {
    speedRatio =- speedRatio;
    if (turnSign > 0) rightMotorDir = -rightMotorDir;
    else leftMotorDir = -leftMotorDir;
  }
  
  bresenhamRatio = (long)(speedRatio * (float)(BRESENHAM_MULTIPLIER));

  Serial.print(bresenhamRatio);
  Serial.print("\t\t");
  if (turnSign > 0)
  {
    targetLeft = (long)(outerWheelDistance * mmToCounts) * leftMotorDir;
      Serial.print(targetLeft);
      Serial.println(" (l)");
    startMove(MM_TURN_RIGHT);
  }
  else
  {
    targetRight = (long)(outerWheelDistance * mmToCounts) * rightMotorDir;
      Serial.print(targetRight);
        Serial.println(" (r)");

    startMove(MM_TURN_LEFT);
  }
   
}

void turnOnSpot(float angleDeg)
{
  float distance = angleDeg * M_PI * WHEEL_BASE / 360.0;
  targetLeft = (long)(distance * mmToCounts);
  targetRight = -targetLeft;
  
  if (distance >= 0) leftMotorDir = 1;
  else leftMotorDir = -1;
  
  rightMotorDir = -leftMotorDir;

  startMove(MM_TURN_ON_SPOT); 
}

void penUp(){
  pwmSpeed = 9;
}

void penDown(){
  pwmSpeed = 11;
}



/****************************************************************
*
*		arduino loop
*
*****************************************************************/

void loop(){	

  if(file.available()){
    byte b;

    b = file.read();

    byte b1 = b >> 4; // likes nibble
    byte b2 = b & 15; // rechtes nibble

    
    switch(b1){
      case 1:
        break;
      case 2:
        leftStepForward();
        break;
      case 0:
        leftStepBackward();
        break;
      case 3:
        penUp();
        break;
      case 4:
        penDown();
        break;
    }
   
    switch(b2){
      case 1:
        break;
      case 2:
        rightStepForward();
        break;
      case 0:
        rightStepBackward();
        break;
    }

    // read delay value
    b = file.read();
    // myStepDelay = 500 + (b * 5);
  }
  else{
    file.close();
  }
}



//  #ifdef DEBUG
//           Serial.print(programStep);
//           Serial.print("\t\t");
//           Serial.println(moveMode);
//  #endif
       
//  if(moveMode == MM_STOP)
//         {
//           switch (programStep) 
//           {
                        
//           case 0:
//           case 4:
//           case 8:
//           case 12:
//              drive(60);
//              break;
//            case 1:
//            case 5:
//            case 9:
//            case 13:
//              turn(30, 15*2*M_PI);
//              break;
//            case 2:
//            case 6:
//            case 10:
//              drive(60);
//              break;
//            case 3:           
//            case 7:           
//            case 11:           
//                turnOnSpot(-90);
//              break;

          

//           case 14:
//              drive(65.6);  // (0.6mm dazugepfuscht)
//              break;
//           case 15:
//                turnOnSpot(-90);
//                break;
//           case 16:
//                drive(60);
//                break;

//           case 20:
//           case 24:
//           case 28:
//              drive(65);
//              break;
//            case 17:
//            case 21:
//            case 25:
//            case 29:
//              turn(25, 12.5*2*M_PI);
//              break;
//            case 18:
//            case 22:
//            case 26:
//              drive(65);
//              break;
//            case 19:           
//            case 23:           
//            case 27:           
//                turnOnSpot(-90);
//              break;
             
//            case 30:
//              drive(60);
//             break;
//            case 31:           
//              turnOnSpot(-90);
//              break;
//            case 32:
//              drive(5);
//              break;

//            default:
//              delay(1000);
//              break;
//           }

// //        programStep = (programStep + 1) % 4;  // endless loop ...
//           programStep += 1;
//         }


/* 
            
          case 0:
             turn(20, 15* 2 * M_PI);
             break;
           case 1:
             turn(-4, 2*2*M_PI);
             break;
           case 2:
             drive(125);
             break;
           case 3:           
             turn(-4, 1 * 2*M_PI);
             break;
           case 4:
             turnOnSpot(90);
             break;
           case 5:
             turn(20, 10*2*M_PI);
             break;
           case 6:
             turnOnSpot(90);
             break;
           case 7:
             turn(-4, 1 * 2*M_PI);
             break;
           case 8:
             drive(125);
             break;
           case 9:
             turn(-4, 2 * 2 * M_PI);
             break;           
           case 10:
             turn(20, 2*15*M_PI);
             break;
           case 11:
             drive(85);
             break;
          default:
            delay(1000);
            break;
*/
