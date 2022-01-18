#include <Arduino.h>
#include <PID.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* Controller parameters */
#define PID_KP 40.0f
#define PID_KI 2.80f
#define PID_KD 0.80f

#define PID_TAU 0.005f

#define PID_LIM_MIN 0.00f
#define PID_LIM_MAX 100.00f

#define PID_LIM_MIN_INT 0.0f
#define PID_LIM_MAX_INT 30.0f

#define SAMPLE_TIME_S 0.01f

/* I/O definitions */
#define KP 5       // Proportional gain
#define KI A1       // Integral gain
#define KD A2       // Derivative gain
#define Set_Point A0 // Set point
#define CV 3        // Calculated value
#define SN A3       // Sensor value

// which analog pin to connect
#define THERMISTORPIN SN
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

volatile bool detection = 0;
volatile long Value = 0;
volatile bool last_CH1_state = 0;
unsigned long timenow = 0;

int samples[NUMSAMPLES];


// Read and calculate KP
float readKP()
{
  return ((float)map(analogRead(KP), 0, 1023, 0, 10000)) / 100.00f;
}

// Read and calculate KI
float readKI()
{
  return ((float)map(analogRead(KI), 0, 1023, 0, 1000000)) / 10000.00f;
}

// Read and calculate KD
float readKD()
{
  return ((float)map(analogRead(KD), 0, 1023, 0, 10000)) / 10000.00f;
}

// Read and calculate KD
float readSet_Point()
{
  return ((float)map(analogRead(Set_Point), 0, 1023, 150, 1000)) / 10.00f;
}

float measureTemp()
{
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++)
  {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++)
  {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;          // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C

  return steinhart;
}

/* Initialise PID controller */
PIDController pid = {PID_KP, PID_KI, PID_KD,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};

// Set the LCD address to 0x27 for a 20 chars and 4 line diSet_Pointlay
LiquidCrystal_I2C lcd(0x27, 20, 4);

int SST = 0;
float MIN = 100.00f;
float MAX = 0.00f;
bool reachedSP = false;
bool temp_state = false;
int cnt = 0;
float setpoint = 45.00f;
void setup()
{
  lcd.begin();
  lcd.backlight();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= B00000010;
  pinMode(KP, INPUT);
  pinMode(KI, INPUT);
  pinMode(KD, INPUT);
  pinMode(Set_Point, INPUT);
  pinMode(CV, OUTPUT);
  pinMode(2, INPUT_PULLUP);

  PIDController_Init(&pid);
   
  // pid.Kp = 40.00f;
  // pid.Ki = 15.00;
  // pid.Kd = 0.80;

  cli();                      //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  
  /*3. We enable compare match mode on register A*/
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  
  /*4. Set the value of register A to 625*/
  OCR1A = 625;             //Finally we set compare register A to this value  
  sei();            
}
float measurement;
void loop()
{
  // float setpoint = readSet_Point();
  // pid.Kp = readKP();
  // pid.Ki = readKI();
  // pid.Kd = readKD();
  
  measurement = measureTemp();

  

  lcd.setCursor(0, 0);
  lcd.print("SP:" + String(setpoint));
  lcd.setCursor(0, 1);
  lcd.print("KP:" + String(pid.Kp));
  lcd.setCursor(0, 2);
  lcd.print("KI:" + String(pid.Ki));
  lcd.setCursor(0, 3);
  lcd.print("PID:" + String(pid.out));
  lcd.setCursor(10, 0);
  lcd.print("TMP:" + String(measurement));
  lcd.setCursor(10, 1);
  lcd.print("PRO:" + String(pid.proportional_));
  lcd.setCursor(10, 2);
  lcd.print("INT:" + String(pid.integrator));
  lcd.setCursor(10, 3);
  lcd.print("DEF:" + String(pid.differentiator));

  // if (measurement >= setpoint && !reachedSP && !temp_state && cnt < 10)
  // {
  //   temp_state = true;
  //   cnt++;
  // }
  // if (measurement < setpoint)
  // {
  //   temp_state = false;
  // }
  
  // if (cnt >= 10)
  // {
  //   reachedSP = true;
  //   SST = millis() / 1000;
  //   cnt = 0;
  // }
  
  // if (reachedSP)
  // {
  //   if (measurement < MIN)
  //   {
  //     MIN = measurement;
  //   }
  //   if (measurement > MAX)
  //   {
  //     MAX = measurement;
  //   }
  // }

}

ISR(PCINT0_vect)
{
  Value = map(pid.out, 0, 100, 16200, 0);
  if (PINB & B00000010)
  { // We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if (last_CH1_state == 0)
    {                // If the last state was 0, then we have a state change...
      detection = 1; // We haev detected a state change!
    }
  }
  else if (last_CH1_state == 1)
  {                     // If pin 8 is LOW and the last state was HIGH then we have a state change
    detection = 1;      // We haev detected a state change!
    last_CH1_state = 0; // Store the current state into the last state for the next loop
  }
  if (detection)
  {
    delayMicroseconds(Value); // This delay controls the power
    digitalWrite(3, HIGH);
    delayMicroseconds(16200 - Value);
    digitalWrite(3, LOW);
    detection = 0;
  }
}

//With the settings above, this IRS will trigger each 10ms.
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  /* Compute new control signal */
  PIDController_Update(&pid, setpoint, measurement);
}