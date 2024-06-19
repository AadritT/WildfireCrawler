/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

#include <SoftwareSerial.h>

#define TX_PIN_ESP 8
#define RX_PIN_ESP 9
#define TX_PIN_MKR 5
#define RX_PIN_MKR 6

SoftwareSerial mySerial_ESP(RX_PIN_ESP, TX_PIN_ESP);  // RX, TX
SoftwareSerial mySerial_MKR(RX_PIN_MKR, TX_PIN_MKR);  // RX, TX


Adafruit_MotorShield AFMS1(0x61);
Adafruit_MotorShield AFMS2(0x62);  


//driver motor on mid shield
Adafruit_StepperMotor *sh1D1 = AFMS1.getStepper(16 * 64, 1);
Adafruit_StepperMotor *sh1D2 = AFMS1.getStepper(16 * 64, 2);

//top shield
Adafruit_StepperMotor *sh2A1 = AFMS2.getStepper(200, 1);
Adafruit_StepperMotor *sh2A2 = AFMS2.getStepper(200, 2);

const int stepsPerRev = 16 * 64;  //in steps for 28byj-48 gear ratio 1:16
const int di = 28;                //in mm


void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  while (!Serial)
    delay(100);

  Serial.println("serial beginning (UNO to Computer)");

  AFMS1.begin();
  AFMS2.begin();  // Start the top shield
}

int i;

bool armMove = true;
bool driveMove = true;

int count = 0;

void loop() {
  // Serial.println("Waiting for command from comp serial");

  // while (!Serial.available())
  //   ;
  // Serial.print("recieving from Computer: ");
  //   Serial.print(Serial.read());

  while (count < 3) {
    delay(1000);
    Serial.println(". Beginning motor movement.");
    int speedDrive = 1;  //inversely proportional to how fast
    int speedArm = 5;

    int distanceDrive = 130; //mm
    int rotateArm = 1130;

    ///*
    //arm bottom
    if (armMove) {
      sh2A1->release();
      sh2A2->release();

      for (i = 0; i < rotateArm; i++) {
        sh2A1->onestep(BACKWARD, INTERLEAVE);
        sh2A2->onestep(FORWARD, INTERLEAVE);
        delay(speedArm);
      }
      delay(1000);
      // for (i = 0; i < rotateArm; i++) {
      //   sh2A1->onestep(FORWARD, INTERLEAVE);
      //   sh2A2->onestep(BACKWARD, INTERLEAVE);
      //   delay(speedArm);
      // }

      armMove = false;
    }


    delay(1000);
    //drivers
    if (driveMove) {
      for (i = 0; i < rotationStepsCalc(distanceDrive); i++) {
        sh1D1->onestep(FORWARD, INTERLEAVE);
        sh1D2->onestep(FORWARD, INTERLEAVE);
        delay(speedDrive);
      }
      // driveMove = false;
    }

    mySerial_ESP.begin(9600);  // set up Serial library at 9600 bps
    while (!mySerial_ESP)
      delay(100);
    Serial.println("serial beginning (UNO to ESP)");

    Serial.println("Motors done. Sending command to esp");
    char ch = '1';
    mySerial_ESP.write(ch);
    while (!mySerial_ESP.available()) {
      delay(100);
    }
    Serial.print("Camera done. Recieving prob from esp: ");
    uint8_t prob = mySerial_ESP.read();
    Serial.println(prob);
    mySerial_ESP.end();

    mySerial_MKR.begin(9600);  // set up Serial library at 9600 bps
    while (!mySerial_MKR)
      delay(100);
    Serial.println("serial beginning (UNO to MKR)");

    Serial.println("Sending prob to mkr");
    mySerial_MKR.write(prob);

    while (!mySerial_MKR.available())
      ;
    mySerial_MKR.read();
    Serial.println("Things sent. Back to start.");
    mySerial_MKR.end();

    count++;
  }
}

//dist in mm
int rotationStepsCalc(int distance) {
  return (((stepsPerRev) / (PI * di)) * distance);
}