#include <Arduino.h>
#include "Wire.h"

// Team members: Eric Lin, Dean Lee

#define MPRSensorAddr 0x18

// systole (heart contraction) from beginning to before the max peak oscillation rises
// diastole (heart relaxation) from when the max peak oscillation first rises up

// Find: (1) heart rate (2) systolic (3) diastolic
// (1) heart rate: frequency of the pressure oscillations - time difference between peaks
// counted peaks / time length
// (2) systolic: point where blood stops flowing - peak pressure of 1st oscillation
// (3) 
// max = pressure at peak of max oscillation
// diastolic: max - (systolic - max)

uint32_t pMin = 0;
uint32_t pMax = 300;
uint32_t outputMin = 0x00FFFFFF * 0.025;      // calibration is 2.5% to 22.5%
uint32_t outputMax = 0x00FFFFFF * 0.225;
volatile uint32_t timeCounter = 0;            // in milliseconds

// volatile float pressureArray[1000] = {1.0}; // too large 160% RAM
bool pressureIncrease = true;
const int num = 3;
float pressurePeak[num];
uint32_t timePeak[num];
float pressureDip[num];
uint32_t timeDip[num];
uint32_t numPeaks = 0;

// slope should be about -4mmHg/sec interval - in (-5, -3)
float pressure = 0;
uint32_t pressureTime = 0;       // timeCounter of pressure
uint32_t pressureTimeDiff = 0;    // time difference(ms)
float pressureSlope = 0;          // current slope
float generalSlope = 0;           // general pressure change slope

// smooth it out with a MA-point moving average function
const uint8_t MA = 10;
float pressureList[MA];
uint32_t timeList[MA];
bool pressListEmpty = true;
uint8_t pressPos = 0;             // keeps track of latest pressure position [0, MA - 1]

// How many peaks and bottoms do we need to keep track of?
// point where oscillation first starts
// systolic peak and bottom
float systolic = -1;
float diastolic = -1;
float heartRate = -1;

// I2C OUTPUT MEASUREMENT COMMAND + pressure conversion
float pressureMeasurement(uint8_t deviceAddress) {
  // DeviceAddress = 0x18 (default)
  uint8_t status = 0;
  uint8_t sensorData8 = 0;
  uint32_t sensorData = 0;      // Only need <23:0>

  // Note: The frequency is SCLK is 100-400kH
  // The Minimum SCLK clock low width is 0.6 usec
  // Unless Status is busy, this function should end way faster than 1 msec

  // step 1: exit Standby Mode and enter Operating Mode
  Wire.beginTransmission(deviceAddress); // Transmit to device number (0x18)
  Wire.write((uint8_t)0xAA);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();        // Stop transmitting
  
  // step 2
  // Option 1: Wait until the busy flag in the Status Byte clears
  // Option 2: Wait for at least 5 ms for the data conversion to occur.
  // Option 3: Wait for the EOC indicator.
  status |= (1<<5);   // Busy Flag is bit 5 of status
  while ( (status & (1<<5)) == (1<<5) ) {
    Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);  // Request 1 byte from slave device 0x18
    if (Wire.available()==1)
      status = Wire.read();  // returns next byte
    delay(1);
  }

  // step 3
  // To read the 24-bit pressure output along with the 8-bit Status Byte:
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)4);  // Request 4 byte from slave device 0x18
  if (Wire.available()==4) {
    status = Wire.read();
    //check memory integrity and math saturation bits
    if((status & (1<<2)) || (status & (1<<0)))
    {
      Serial.println("Error: memory integrity and math saturation bits are set!");
      return -1;
    }

    for(uint8_t i=0; i<3; i++)
    {
      sensorData = sensorData<<8; // left shift the bits by 8
      sensorData8 = Wire.read();  // get the next byte
      sensorData |= sensorData8;
    }
  }

  // Step 4: Pressure Conversion
  //convert from 24-bit to float psi value
  //float pressure;
  //pressure = (SensorData - outputMin) * (Pmax - Pmin);
  //pressure = (pressure / (outputMax - outputMin)) + Pmin;
  float press;
  press = sensorData;
  press -= outputMin;
  press *= (pMax - pMin);
  press = (press / (outputMax - outputMin)) + pMin;
  return press;
}

// initialize the pressure list (each pressure is at least 0.01 s apart)
void initPressList(void) {
  for (int i = 0; i < MA; i++) {
    pressureList[i] = pressureMeasurement(MPRSensorAddr);
    timeList[i] = timeCounter;
    delay(10);
  }
  pressPos = MA - 1;
  pressListEmpty = false;    // pressCT = 1, pressPos = 0
}

// read pressure values of at least 0.01 s apart into the list
void addPressVal() {
  if (pressListEmpty)
    initPressList();
  else {
    pressPos++;         // next position
    if (pressPos >= MA)
      pressPos = 0;
    pressure = pressureMeasurement(MPRSensorAddr);
    pressureTime = timeCounter;
    pressureList[pressPos] = pressure;
    timeList[pressPos] = pressureTime;
    delay(10);
  }
}

float findPressSlope(void) {
  if (pressListEmpty)
    initPressList();
  int firstPos = pressPos + 1;
  if (firstPos >= MA)
    firstPos = 0;
  float p1 = pressureList[pressPos];
  uint32_t t1 = timeList[pressPos];
  float p2 = pressureList[firstPos];
  uint32_t t2 = timeList[firstPos];
  return (p1 - p2) * 1000 / (t1 - t2);  // units: mmHg / sec
}

void serialPrintPressure(float pres) {
  Serial.print("Pressure is ");
  Serial.print(pres);
  Serial.println("mmHg");
}

void LEDState(float slope) {
  if (pressureSlope < -5) {
        PORTD |= (1<<2);
        PORTD &= ~(1<<3);
      }
      else if (pressureSlope > -3) {
        PORTD &= ~(1<<2);
        PORTD |= (1<<3);
      }
      else {
        PORTD &= ~(1<<2);
        PORTD &= ~(1<<3);
      }
}

// check if the pressure increased beyond 150mmHg
// make sure that decrease is about -4mmHg / sec
// range: (-5, -3) by the time pressure is 140mmHg
// if it's not within range when pressure decreases below 140mmHg, then cancel the run
bool setupPressureCuff(void) {
  Serial.println("Please begin inflating the blood pressure cuff to above 150mmHg.");
  bool withinRange = false;
  pressureIncrease = true;
  timeCounter = 0;
  pressListEmpty = true;
  addPressVal();
  delay(5);
  //start going up to 150 mmHg
  while(pressure < 150) {
    Serial.println("Keep inflating cuff until pressure is above 150mmHg");
    serialPrintPressure(pressure);
    addPressVal();
  }
  pressureIncrease = false;
  Serial.println("\nYou can stop inflating.");

  // releasing valve decreases pressure
  // make sure pressureSlope is about -4mmHg/sec (-5 , -3) before the value decreases to 140mmHg
  uint32_t ctr = 0;
  while(pressure > 140) {
    // read the pressure values every ms
    addPressVal();
    ctr++;
    // every MAms, inform whether to loosen or tighten valve
    if (ctr == (MA / 2)) {
      ctr = 0;
      pressureSlope = findPressSlope();
      Serial.println(pressure);
      
      Serial.print("Slope of pressure change is ");
      Serial.print(pressureSlope);
      Serial.println(" mmHg / sec");

      LEDState(pressureSlope);

      if (pressureSlope < -5) {
        Serial.println("Pressure decrease is too fast - TIGHTEN valve");
        withinRange = false;
      }
      else if (pressureSlope > -3) {
        Serial.println("Pressure decrease is too slow - LOOSEN valve");
        withinRange = false;
      }
      else {
        Serial.println("Pressure decrease is about 4mmHg / sec!");
        withinRange = true;
      }
    }
  }
  return withinRange;
}

void risingPeakValue(int ctr) {
  pressureIncrease = true;
  // repeat until about (MA/2) pressure values are below 0
  while (findPressSlope() > 0) {
    addPressVal();
    if (pressure > pressurePeak[ctr]) {
      pressurePeak[ctr] = pressure;
      timePeak[ctr] = timeCounter;
    }
  }
  pressureIncrease = false;
}

void fallingDipValue(int ctr) {
  pressureIncrease = false;
  // repeat until about (MA/2) pressure values are above 0
  while (findPressSlope() < 0) {
    addPressVal();
    if (pressure < pressureDip[ctr]) {
      pressureDip[ctr] = pressure;
      timeDip[ctr] = timeCounter;
    }
  }
  pressureIncrease = true;
}

// Find and print the systolic, diastolic, and heart rate measurements
void findHeartValues(void) {
  // First get the general downward slope
  pressListEmpty = true;
  addPressVal();
  generalSlope = findPressSlope();
  pressureIncrease = false;
  int ctr = 0;

  Serial.println("\nYou should not be inflating.");
  Serial.print("The slope of the pressure change is about ");
  Serial.print(generalSlope);
  Serial.println(" mmHg / second");

  // the systolic which is less than 120 mmHg
  while (pressure > 120)
    addPressVal();

  // find the start of the first ripple(oscillation)
  // The slope should rise above 0 so set pressureIncrease = true
  // Keep saving the individual pressure values until the peak is found and it declines
  // set pressureIncrease = false and save that first peak pressure as the systolic
  pressureDip[ctr] = pressure;
  while (findPressSlope() < 0) {
    addPressVal();
    if (pressure < pressureDip[ctr]) {
      pressureDip[ctr] = pressure;
      timeDip[ctr] = timeCounter;
    }
  }
  pressurePeak[ctr] = pressure;
  timePeak[ctr] = timeCounter;

  // get the systolic value
  risingPeakValue(ctr);
  // count the peak and measure the difference between the peak and bottom before it
  numPeaks++;
  Serial.println("Ripple is starting!");

  // find the ripple with the highest amplitude
  // use ctr==1 the save the highest value
  // use ctr==2 to compare with ctr==1 values
  pressurePeak[2] = pressurePeak[1] = pressurePeak[0];
  timePeak[2] = timePeak[1] = timePeak[0];
  pressureDip[2] = pressureDip[1] = pressureDip[0];
  timeDip[2] = timeDip[1] = timeDip[0];
  ctr = 2;

  // we want to check over (maxRippleCheck) ripples to see if ripple amplitude decreased
  uint8_t smallerRipple = 0;  // index and counter for the smaller ripples after maxDiff
  uint8_t maxRippleCheck = 1;
  float maxDiff = pressurePeak[ctr] - pressureDip[ctr];
  float diff[maxRippleCheck] = {};

  while (smallerRipple < maxRippleCheck) {
    fallingDipValue(ctr);
    addPressVal();
    risingPeakValue(ctr);
    numPeaks++;
    /*
    if ((numPeaks % 10) == 0) {
      Serial.println(numPeaks); 
      Serial.print("Pressure is ");
      Serial.println(pressure);
    }*/
    // get the current ripple height
    diff[smallerRipple] = pressurePeak[ctr] - pressureDip[ctr];

    // if smaller, increment the counter for the number of ripples that passed smaller.
    if (diff[smallerRipple] < maxDiff) {
      smallerRipple++;
    }
    // save if larger ripple
    else {
      pressurePeak[1] = pressurePeak[2];
      timePeak[1] = timePeak[2];
      pressureDip[1] = pressureDip[2];
      timeDip[1] = timeDip[2];
      // reset counter for smaller ripples
      maxDiff = diff[smallerRipple];
      smallerRipple = 0;
    }
  }

  // we found the maximum ripple
  // now modify the global values for the final results
  systolic = pressurePeak[0];
  float peakOfMaxRipple = pressurePeak[1];
  diastolic = peakOfMaxRipple - (systolic - peakOfMaxRipple);

  // find heart rate
  float lastBeat = timePeak[2];
  float firstBeat = timePeak[0];
  heartRate = (numPeaks - 1) / (lastBeat - firstBeat) * 1000;
}

// setup up timer that will increment timeCounter each millisecond
void timerSetup(void) {
  //// Timer Setup
  TCCR1A = 0x00;        //Disconnect all pins, simple timer count up to OCR3A
  TCCR1B = 0b00001010;  //CPS=8, WG bits set for counter
  TCCR1C = 0x00;        //All disabled
  OCR1A = 1000;        //Compare A interrupt - every 1ms
  // 8,000,000 / 8 = 1,000,000 ticks per second
  // OCR1A counts per millisec
  TIMSK1 = 0b00000011;  //Output Compare A Match Interrupt Enable, Enable Overflow Interrupt
}

ISR(TIMER1_COMPA_vect) {
  timeCounter++;
  // if ((timeCounter % 100) == 0) Serial.println(timeCounter);  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();       // Join I2C bus
  timerSetup();       // setup the timer to timeCounter++ every ms

  DDRD |= (1<<2);     // output: PD2 - D0(RX)
  DDRD |= (1<<3);     // output: PD3 - D1(TX)
  PORTD &= ~(1<<2);
  PORTD &= ~(1<<3);

  // setup the pressure cuff to over 150 mmHg
  while(!setupPressureCuff()) {
    Serial.println("The rate of pressure change is not about -4mmHg / sec.\nPlease try again.");
    delay(1);
  }
  Serial.println("The rate of pressure change is about -4mmHg / sec.\nPlease do not touch the valve.");

  // Find the value of systolic and diastolic, and the heart rate
  findHeartValues();

  // print the values
  Serial.print("The heart rate is ");
  Serial.print(heartRate);
  Serial.println(" beats per second");
  Serial.print("The systolic value is ");
  Serial.print(systolic);
  Serial.println(" mmHg");
  Serial.print("The diastolic value is ");
  Serial.print(systolic);
  Serial.println(" mmHg");
}


void loop() {
  // put your main code here, to run repeatedly:
}
