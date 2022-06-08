// Basic sketch for A4988 stepper motor driver
// This sketch uses interrupts to step the motor

// MIT licence applies
// Created by Bodmer 20/4/2016

// Microsteps per step   :     1      8     16   (Set with A4988 mode pins)
// RPM at 5kHz interrupt :   1500    187    93   (for a motor with 200 steps per rev)
// Approx. minimum RPM   :     74     10     5   (for a motor with 200 steps per rev)

// Use timer prescaler to allow lower speeds, e.g. divide clock by 8
// Use fewer microsteps per step to raise speeds

// gestion moteur3 en position
#include <AccelStepper.h>

// gestion ecran oled ssd1306
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <RotaryEncoder.h>

#include "display.h"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Motor settings
#define MICROSTEPS_PER_STEP  32  // DRV8825 can be set to 1, 8 16, 32
#define STEPS_PER_REV       200  // Test motor has 200 steps per rev
#define MAX_RPM             100  // RPM limit
#define AUTO_RAMP_DOWN           // Auto ramp down of speed at end of a movement

// Timer settings
#define CPU_MHZ             16000000L // 16 MHz oscillator
#define PRESCALER           8    // Timer 1 prescaler (no prescaler = 1, 8 for slower speeds)
#define TIMER_1_MIN         1000 // Stop speed for end of movement ramp down
#define TIMER_1_MAX         0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * MAX_RPM / 60L) )
#define TIMER_2_MIN         1000 // Stop speed for end of movement ramp down
#define TIMER_2_MAX         0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * MAX_RPM / 60L) )

// motor PULLER X sur ramps 1.4
#define MOTOR1_STEP_ENA 38               // A4988 enable pin
#define MOTOR1_STEP_CLK 54               // A4988 clock pin
#define MOTOR1_STEP_DIR 55               // A4988 direction pin


// motor WINDER Y sur ramps 1.4
//#define MOTOR2_STEP_ENA 56               // A4988 enable pin
//#define MOTOR2_STEP_CLK 60               // A4988 clock pin
//#define MOTOR2_STEP_DIR 61               // A4988 direction pin
#define MOTOR2_STEP_ENA 62               // A4988 enable pin
#define MOTOR2_STEP_CLK 46               // A4988 clock pin
#define MOTOR2_STEP_DIR 48               // A4988 direction pin
#define MOTOR2_STEP_PER_TURN    (200 * 2 * 32)
// motor SLIDER Z sur ramps 1.4
//#define MOTOR3_STEP_ENA 62               // A4988 enable pin
//#define MOTOR3_STEP_CLK 46               // A4988 clock pin
//#define MOTOR3_STEP_DIR 48               // A4988 direction pin
#define MOTOR3_STEP_ENA 56               // A4988 enable pin
#define MOTOR3_STEP_CLK 60               // A4988 clock pin
#define MOTOR3_STEP_DIR 61               // A4988 direction pin


#define SLIDER_MAX_PIN  3
#define SLIDER_MIN_PIN  14

#define WINDER_MAX_PIN  57  
#define WINDER_MIN_PIN  18

#define POTENTIMETER_SPEED_PIN    A11       //         
// parametres pour moteurs sous interupt

unsigned int timer1_counter = 1;
unsigned int timer2_counter = 1;
volatile unsigned int new_counter = 1;
volatile unsigned int new_counter2 = 1;

volatile int  steps = 0;    // Changes outside of interrupt handler, so make it volatile
volatile int  steps2 = 0;    // Changes outside of interrupt handler, so make it volatile
volatile bool done  = true; // Used outside interrupt handler, so make it volatile
volatile bool done2  = true; // Used outside interrupt handler, so make it volatile
volatile uint32_t motor2_step = 0;
int8_t motor3_dir = 1;   // direction du moteur3
long motor3_position = 0;

#define WINDER_MAX_SPEED 40*8
#define WINDER_MIN_SPEED 1

uint32_t winder_speed = WINDER_MIN_SPEED;

#define MAX_MOTOR3_POSITION 4000
#define MOTOR3_STEP ((200 * 32) /8)
//(200 * 32)  //1tour

int motorSpeed = 16;     //max puller speed

bool step_forward = true;

// parametres pour moteur asservi en position
AccelStepper stepper;

// Encoder parameters
const int clkPin  = 32;
const int dtPin  = 47;
const int swPin  = 45;

const int minrotary = 1;
const int maxrotary = 32;

//Variables
int rotVal  = 0;
bool clkState  = LOW;
bool clkLast  = HIGH;
bool swState  = HIGH;
bool swLast  = HIGH;


RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  if (encoder != nullptr)
    encoder->tick(); // just call tick() to check the state.
}

void initRotary ()
{
  pinMode(clkPin,INPUT);
  pinMode(dtPin,INPUT);
  pinMode(swPin,INPUT_PULLUP);
  encoder = new RotaryEncoder(clkPin, dtPin, RotaryEncoder::LatchMode::FOUR0);
  
  attachInterrupt(digitalPinToInterrupt(clkPin), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dtPin), checkPosition, CHANGE);
}

int readRotary() { /* function readRotary */
 ////Test routine for Rotary
 // gestion position
 clkState = digitalRead(clkPin);
 if ((clkLast == LOW) && (clkState == HIGH)) {//rotary moving
   //Serial.print("Rotary position ");
   if (digitalRead(dtPin) == HIGH) {
     rotVal = rotVal - 1;
     if ( rotVal < minrotary ) {
       rotVal = minrotary;
     }
   }
   else {
     rotVal++;
     if ( rotVal > maxrotary ) {
       rotVal = maxrotary;
     }
   }
   //Serial.println(rotVal);
   delay(200);
 }
 clkLast = clkState;
 //gestion bouton
 swState = digitalRead(swPin);
 if (swState == LOW && swLast == HIGH) {
   Serial.println("Rotary pressed");
   delay(100);//debounce
 }
 swLast = swState;
 return (rotVal);
}
// fin encoder

uint32_t timewinder = 0;
void HandleWinderSpeed ()
{
  if(millis ()- timewinder > 10)
  {
    if(digitalRead (WINDER_MAX_PIN) ==HIGH)
    {
      //plus vite
      if (winder_speed < WINDER_MAX_SPEED)
        winder_speed += 1;
    }
    
    if (digitalRead(WINDER_MIN_PIN) == HIGH)
    {
      winder_speed = WINDER_MIN_SPEED;
      
    }
    timewinder =millis();
  }  
}

// gere la position du moteur 3 en fonction du moteur2
void HandleMotor3 ()
{
#if 0
    if (digitalRead(SLIDER_MAX_PIN) == HIGH || digitalRead(SLIDER_MIN_PIN)==HIGH)
    {
        motor3_position = stepper.currentPosition ();
        stepper.moveTo(motor3_position); 
    }
#endif    
    if(digitalRead (SLIDER_MAX_PIN) == HIGH && motor3_dir > 0)
    {
        Serial.println ("slider motor on max");
        motor3_dir = -1;
        motor3_position = stepper.currentPosition() - MOTOR3_STEP;
        stepper.moveTo (motor3_position);
        Serial.print ("slider motor on position ");
        Serial.println (stepper.currentPosition());
    
    }
    
    if(digitalRead (SLIDER_MIN_PIN) == HIGH && motor3_dir < 0)
    {
        Serial.println ("slider motor on min");
        motor3_dir = 1;
        motor3_position = stepper.currentPosition() + MOTOR3_STEP;
        stepper.moveTo (motor3_position);
        Serial.print ("slider motor on position ");
        Serial.println (stepper.currentPosition());
    
    }  
    
    //Serial.print ("slider motor on position ");
    //Serial.println (stepper.currentPosition());
    
    if (motor2_step > MOTOR2_STEP_PER_TURN) 
    {
      //inverse le le sens du moteur si on touche un fin de course
      
      
      if (motor3_dir > 0)
      {
        motor3_position += MOTOR3_STEP;
      }
      else
      {
        motor3_position -= MOTOR3_STEP;
        
      }
       
       stepper.moveTo (motor3_position);
       Serial.print ("slider motor go to position ");
       Serial.print (motor3_dir);
       Serial.print (" ");
       Serial.println (motor3_position);
    
       noInterrupts();
       motor2_step -= MOTOR2_STEP_PER_TURN;
       interrupts ();
   }
    
}

void setup()
{
  Serial.begin(115200);

// initialisation ecran
  display.begin(0X3C, true);   // initialisation de l'objet afficheur
  display.clearDisplay();
  display.display ();
  //afficheur.setTextSize(2);                                              // choix de la taille des caractères
  //afficheur.setTextColor(1);                                            // caractères en blanc
  //afficheur.setCursor(32,16);                                         // positionnement du curseur
  //afficheur.print("ARDUINO");                                         // affichage de ARDUINO
  //afficheur.drawBitmap(0, 0,  love, 64, 32, 1);             // affichage de l'image love
  display.drawPixel(10, 10, SH110X_WHITE);
  for (int x = 8; x < 120; x++)
    display.drawPixel(x, 20, SH110X_WHITE);
  
  display.fillRect (24, 24, 64, 64, SH110X_WHITE);
  display.setCursor (24, 2);
  display.setTextColor (SH110X_WHITE);
  display.setTextSize (1);
  display.print ("Hello !");
  display.display ();                      

  // init GPIO
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR1_STEP_ENA, OUTPUT);
  pinMode(MOTOR1_STEP_CLK, OUTPUT);
  pinMode(MOTOR1_STEP_DIR, OUTPUT);
  pinMode(MOTOR2_STEP_ENA, OUTPUT);
  pinMode(MOTOR2_STEP_CLK, OUTPUT);
  pinMode(MOTOR2_STEP_DIR, OUTPUT);

  pinMode (SLIDER_MAX_PIN,INPUT);
  pinMode (SLIDER_MIN_PIN,INPUT);

  pinMode(WINDER_MAX_PIN,INPUT);
  pinMode(WINDER_MIN_PIN,INPUT);
  
  digitalWrite(MOTOR1_STEP_ENA, HIGH); // Disable the driver
  digitalWrite(MOTOR2_STEP_ENA, HIGH); // Disable the driver

  startTimer1();                // Start the timer 1 interrupt moto E0
  startTimer2();                // start the timer 2 interrupt motor E1
  digitalWrite(MOTOR1_STEP_ENA, LOW);  // Enable the driver
  digitalWrite(MOTOR2_STEP_ENA, LOW);  // Enable the driver

  
  // initialise le codeur rotatif
  initRotary ();

  // initialise le moteur position
  pinMode(MOTOR3_STEP_ENA, OUTPUT);
  pinMode(MOTOR3_STEP_CLK, OUTPUT);
  pinMode(MOTOR3_STEP_DIR, OUTPUT);

  stepper = AccelStepper (AccelStepper::DRIVER, MOTOR3_STEP_CLK, MOTOR3_STEP_DIR, 0, 0, true);
  stepper.setEnablePin (MOTOR3_STEP_ENA);
  stepper.setPinsInverted  ( false, // dir
        false, // step
        true    // enable
        );   

  stepper.enableOutputs ();
  stepper.setMaxSpeed(20000.0);
  stepper.setAcceleration(8000.0);
}

uint32_t time_display = 0;
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  encoder->tick();

  
  int pot = analogRead(POTENTIMETER_SPEED_PIN);
  int valencoder = encoder->getPosition ();

  if (valencoder < 0)
    valencoder =0;
  if(valencoder > maxrotary)  
    valencoder =maxrotary;
    
  int spd = map(valencoder, minrotary, maxrotary, 1, motorSpeed);
  
  
  
  #if 1
  Serial.print (motor2_step);
  Serial.print (" ");
  Serial.print (valencoder);
  Serial.print (" ");
  Serial.println (spd);
#endif
  // vitesse des 2 premiers moteurs
  motorRPM(spd);  // moteur1 en fonction du pontentiometre
  motor2RPM(winder_speed);  // moteur2 a vitesse fixe
  
  
  stepMotor(+6400 * 2); // Positive for forward movement  400 steps * 16 microsteps
  stepMotor2(-6400 * 2); // Positive for forward movement  400 steps * 16 microsteps
      
  // Blink LED
  HandleMotor3();

  //get winder speed
  HandleWinderSpeed();
    
  stepper.run ();
  
  digitalWrite(LED_BUILTIN, LOW);
  //delay(100);

  if(millis () - time_display > 200)
  {
    
    uint32_t speedmm = (156 / (64 /spd))*60;  
    display.clearDisplay();
    //display.drawBitmap(0, 0, home, 128 , 64, SH110X_WHITE);
    display.drawBitmap(0, 0, speed, 128 , 64, SH110X_WHITE);
    
  
    display.setCursor (48+16, 24);
    display.setTextColor (SH110X_WHITE);
    display.setTextSize (2);
    
    display.print (speedmm);
    display.setTextSize (1);
    display.setCursor (48+16, 48);
    display.print("mm/m");
    display.display ();

    time_display=millis ();
  }
  
}

/***************************************************************************************
** Function name:           stepMotor
** Description:             Step motor "us" microsteps, +ve for forward, -ve for reverse
***************************************************************************************/
void stepMotor(long us)
{
  if ( us < 0 )
  {
    digitalWrite(MOTOR1_STEP_DIR, LOW);
    us = -us;
  }
  else digitalWrite(MOTOR1_STEP_DIR, HIGH);

  // Updating steps variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  steps = us;
  interrupts();
}

/***************************************************************************************
** Function name:           stepMotor2
** Description:             Step motor "us" microsteps, +ve for forward, -ve for reverse
***************************************************************************************/
void stepMotor2(long us)
{
  if ( us < 0 )
  {
    digitalWrite(MOTOR2_STEP_DIR, LOW);
    us = -us;
  }
  else digitalWrite(MOTOR2_STEP_DIR, HIGH);

  // Updating steps variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  steps2 = us;
  interrupts();
}

/***************************************************************************************
** Function name:           motorRPM
** Description:             Set the motor RPM
***************************************************************************************/
void motorRPM(long rpm)
{
  if ( rpm > MAX_RPM) rpm = MAX_RPM; // Interrup rate limit may lower this again

  //Serial.print("Set rpm = "); Serial.print(rpm);

  long tc = 0xFFFF - ( (CPU_MHZ / PRESCALER) / (STEPS_PER_REV * MICROSTEPS_PER_STEP * rpm / 60L) );

  if (tc < 1) tc = 1;                           // This is the minimum speed value for the counter

  // To keep the interrupt handler burden acceptable a rate of 5000 Hz maximum is set (this also limits the max RPM)
  if ( (PRESCALER ==  1) && tc > 62336) tc = 62336;
  if ( (PRESCALER ==  8) && tc > 65136) tc = 65136;
  if ( (PRESCALER == 64) && tc > 65486) tc = 65486;

  //Serial.print(", tc = "); Serial.println(tc);

  // Updating counter variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  timer1_counter = tc;
  interrupts();
}

/***************************************************************************************
** Function name:           motorRPM
** Description:             Set the motor RPM
***************************************************************************************/
void motor2RPM(long rpm)
{
  if ( rpm > MAX_RPM) rpm = MAX_RPM; // Interrup rate limit may lower this again

  //Serial.print("Set rpm = "); Serial.print(rpm);

  long tc = 0xFFFF - ( CPU_MHZ / PRESCALER / (STEPS_PER_REV * MICROSTEPS_PER_STEP * rpm / 60L) );

  if (tc < 1) tc = 1;                           // This is the minimum speed value for the counter

  // To keep the interrupt handler burden acceptable a rate of 5000 Hz maximum is set (this also limits the max RPM)
  if ( (PRESCALER ==  1) && tc > 62336) tc = 62336;
  if ( (PRESCALER ==  8) && tc > 65136) tc = 65136;
  if ( (PRESCALER == 64) && tc > 65486) tc = 65486;

  //Serial.print(", tc = "); Serial.println(tc);

  // Updating counter variable is non-atomic so turn off interrupts before changing it
  noInterrupts();
  timer2_counter = tc;
  interrupts();
}

/***************************************************************************************
** Function name:           startTimer1
** Description:             Start the timer interrupt firing
***************************************************************************************/
/*
  CS12 CS11 CS10  Mode Description
  0    0    0    Stop Timer/Counter 1
  0    0    1    No Prescaler (Timer Clock = System Clock)
  0    1    0    divide clock by 8
  0    1    1    divide clock by 64
  1    0    0    divide clock by 256
  1    0    1    divide clock by 1024
  1    1    0    increment timer 1 on T1 (pin 5) falling edge
  1    1    1    increment timer 1 on T1 (pin 5) rising edge
*/
void startTimer1(void)
{
  // Initialise timer 1
  noInterrupts();                      // disable interrupts
  TCCR1A = 0;                          // stop timer
  TCCR1B = 0;

  timer1_counter = TIMER_1_MIN;        // preload timer 65536-16MHz/1/frequency

  TCNT1 = timer1_counter;              // preload timer

  if (PRESCALER == 1) TCCR1B |= (1 << CS10);  // No prescaler
  if (PRESCALER == 8) TCCR1B |= (1 << CS11);  // divide clock by 8

  // Other unused prescaler divisions available
  //if (PRESCALER == 64) TCCR1B |= (1 << CS10) | (1 << CS11);   // divide clock by 64
  //if (PRESCALER == 256) TCCR1B |= (1 << CS12);                // divide clock by 256
  //if (PRESCALER == 1024) TCCR1B |= (1 << CS12) | (1 << CS10); // divide clock by 1024

  TIMSK1 |= (1 << TOIE1);              // enable timer overflow interrupt
  interrupts();                        // enable interrupts
}

/***************************************************************************************
** Function name:           startTimer2
** Description:             Start the timer interrupt firing
***************************************************************************************/
/*
  CS12 CS11 CS10  Mode Description
  0    0    0    Stop Timer/Counter 1
  0    0    1    No Prescaler (Timer Clock = System Clock)
  0    1    0    divide clock by 8
  0    1    1    divide clock by 64
  1    0    0    divide clock by 256
  1    0    1    divide clock by 1024
  1    1    0    increment timer 1 on T1 (pin 5) falling edge
  1    1    1    increment timer 1 on T1 (pin 5) rising edge
*/
void startTimer2(void)
{
  // Initialise timer 2
  noInterrupts();                      // disable interrupts
  TCCR3A = 0;                          // stop timer
  TCCR3B = 0;

  timer2_counter = TIMER_2_MIN;        // preload timer 65536-16MHz/1/frequency

  TCNT3 = timer2_counter;              // preload timer

  if (PRESCALER == 1) TCCR3B |= (1 << CS10);  // No prescaler
  if (PRESCALER == 8) TCCR3B |= (1 << CS11);  // divide clock by 8

  // Other unused prescaler divisions available
  //if (PRESCALER == 64) TCCR2B |= (1 << CS10) | (1 << CS11);   // divide clock by 64
  //if (PRESCALER == 256) TCCR2B |= (1 << CS12);                // divide clock by 256
  //if (PRESCALER == 1024) TCCR2B |= (1 << CS12) | (1 << CS10); // divide clock by 1024
  
  TIMSK3 |= (1 << TOIE3);              // enable timer overflow interrupt
  interrupts();                        // enable interrupts
}

/***************************************************************************************
** Function name:           stopTimer1
** Description:             Stop the timer interrupt from firing
***************************************************************************************/
void stopTimer1(void)
{
  noInterrupts();           // disable all interrupts
  TIMSK1 &= ~(1 << TOIE1);  // disable timer overflow interrupt
  interrupts();             // enable all interrupts
}

/***************************************************************************************
** Function name:           stopTimer2
** Description:             Stop the timer interrupt from firing
***************************************************************************************/
void stopTimer2(void)
{
  noInterrupts();           // disable all interrupts
  TIMSK3 &= ~(1 << TOIE3);  // disable timer overflow interrupt
  interrupts();             // enable all interrupts
}

/***************************************************************************************
** Function name:           TIMER1_OVF_vect
** Description:             Called by interrupt when timer 1 overflows
***************************************************************************************/
ISR(TIMER1_OVF_vect)
{
  // First load the timer
  TCNT1 = timer1_counter;

  if ( steps == 0 )
  {
    done = true;
    new_counter = 1;
  }
  // If there are still motor steps left
  else
  {
    // Pulse the clock line to move on one step
    digitalWrite(MOTOR1_STEP_CLK, HIGH);
    digitalWrite(MOTOR1_STEP_CLK, LOW);

    // Decrease the steps remaining
    steps--;

    // The speed ramp up rate can be set by the controlling funtion

#ifdef AUTO_RAMP_DOWN
    // Motor will stop in 64 microsteps, so ramp down the speed to reduce jerk
    if (steps < 64 && timer1_counter > 1000) timer1_counter -= 1000;
#endif
  }
}

/***************************************************************************************
** Function name:           TIMER3_OVF_vect
** Description:             Called by interrupt when timer 1 overflows
***************************************************************************************/
ISR(TIMER3_OVF_vect)
{
  // First load the timer
  TCNT3 = timer2_counter;
  digitalWrite(LED_BUILTIN, HIGH);
  if ( steps2 == 0 )
  {
    done = true;
    new_counter2 = 1;
  }
  // If there are still motor steps left
  else
  {
    // Pulse the clock line to move on one step
    digitalWrite(MOTOR2_STEP_CLK, HIGH);
    digitalWrite(MOTOR2_STEP_CLK, LOW);

    // Decrease the steps remaining
    steps2--;

    // increase total step counter
    motor2_step++;
    
    // The speed ramp up rate can be set by the controlling funtion

#ifdef AUTO_RAMP_DOWN
    // Motor will stop in 64 microsteps, so ramp down the speed to reduce jerk
    if (steps < 64 && timer2_counter > 1000) timer2_counter -= 1000;
#endif
  }
}
/***************************************************************************************
***************************************************************************************/
