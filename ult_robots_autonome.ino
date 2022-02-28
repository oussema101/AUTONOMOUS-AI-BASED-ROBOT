#include <LiquidCrystal_I2C.h>
#include <FlexiTimer2.h>

//#define TEST

#define RoueRadius          30
#define pi                  (float)3.1415
#define RouePerimeter       (2 * pi * RoueRadius)
#define entraxeRadial       145
#define entraxeAxial        (sqrt(2) * sqrt(entraxeRadial))
#define resolutionNema       200

#define StepsPerMillimeter  (float)(resolutionNema / (RoueRadius * 2 * pi))
#define MillimeterPerAngle  (float)((entraxeRadial * pi) / 180)
#define StepsPerAngle       (float)(StepsPerMillimeter * MillimeterPerAngle)

#define en      8
#define dir1    5
#define step1   2
#define dir2    6
#define step2   3
#define dir3    7
#define step3   4
#define dir4   13
#define step4  12

#define FrontSensor    9
#define BackSensor    10
#define Starter       11

LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile uint8_t EstimatedScore = 0;

uint8_t receivedChar[5] = "", cc = "";


void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(en , OUTPUT);
  pinMode(dir1 , OUTPUT);
  pinMode(dir2 , OUTPUT);
  pinMode(dir3 , OUTPUT);
  pinMode(dir4 , OUTPUT);
  pinMode(step1 , OUTPUT);
  pinMode(step2 , OUTPUT);
  pinMode(step3 , OUTPUT);
  pinMode(step4 , OUTPUT);

  pinMode(FrontSensor , INPUT_PULLUP);
  pinMode(BackSensor , INPUT_PULLUP);
  pinMode(Starter , INPUT_PULLUP);
  pinMode(step4 , OUTPUT);

  digitalWrite(en, 1);


  lcd.clear();
  lcd.print("     Ready!");

  while (!digitalRead(Starter));
  lcd.clear();
  lcd.print("       GO!");
  lcd.setCursor(0, 0);
  lcd.print(" a:   b:   c:   ");
  lcd.setCursor(0, 1);
  lcd.print("  Result: ");
  delay(500);
}


void loop()
{

#ifdef TEST
  while (true)
  {
    Serial.print("Front Sensor: ");
    Serial.print(digitalRead(FrontSensor));
    Serial.print("  Back Sensor: ");
    Serial.print(digitalRead(BackSensor));
    Serial.print("  Starter: ");
    Serial.println(digitalRead(Starter));
    //DroiteLateralle(1000, 3000);
  }
#endif
  // Go forward until the wall
  GoUntil(1, 4000);
  GoUntil(1, 6000);
  delay(500);


  /***** Start Calibration X *****/
  GoRobot(0, 200, 1500);
  lcd.setCursor(0, 0);
  delay(500);

  TurnRobot(0, 90, 3000);
  delay(500);

  GoUntil(1, 2000);
  delay(1000);
  GoUntil(1, 6000);
  delay(500);

  GoRobot(0, 450, 1500); // 70
  delay(500);

  TurnRobot(0, 90, 3000);
  delay(500);
  /****** End Calibration X ******/

  //Go forward to read first number
  GoRobot(1, 100, 1500);
  delay(500);
  //GaucheLateralle(40, 3000);
  //delay(500);
  Serial.print('A');
  while (cc == "" || cc == '\n' || cc == '\n')
  {
    while (Serial.available() > 0)
    {
      //Serial.print('A');
      cc = Serial.read();
      receivedChar[0] = cc;
      delay(10);
      cc = "";
    }
  }

  //Print first number
  lcd.setCursor(3, 0);
  lcd.print(receivedChar[0]);
  delay(500);

  //Go to second number
  DroiteLateralle(100, 3000);
  delay(1000);

  Serial.print('B');
  while (cc == "" || cc == '\n' || cc == '\n')
  {
    while (Serial.available() > 0)
    {
      //Serial.print('B');
      cc = Serial.read();
      receivedChar[1] = cc;
      delay(10);
      cc = "";
    }
  }

  //Print second number
  lcd.setCursor(9, 0);
  lcd.print(receivedChar[1]);
  delay(500);

  //Go to second number
  DroiteLateralle(100, 3000);
  delay(500);

  Serial.print('C');
  while (cc == "" || cc == '\n' || cc == '\n')
  {
    while (Serial.available() > 0)
    {
      //Serial.print('C');
      cc = Serial.read();
      receivedChar[2] = cc;
      delay(10);
      cc = "";
    }
  }
  //Print second number
  lcd.setCursor(13, 0);
  lcd.print(receivedChar[2]);
  delay(500);

  //Go to QR
  DroiteLateralle(100, 3000);
  delay(500);
  Serial.print('Q');
  while (cc == "" || cc == '\n' || cc == '\n')
  {
    while (Serial.available() > 0)
    {
      Serial.print('Q');
      cc = Serial.read();
      receivedChar[3] = cc;
      delay(10);
      cc = "";
    }
  }

  lcd.setCursor(13, 1);
  lcd.print("Result: ");
  lcd.setCursor(13, 1);
  lcd.print(receivedChar[2]);
  delay(500);
  //Correct orientayion errors
  GoUntil(1, 2000);
  GoUntil(1, 6000);

  //Back to home
  GoRobot(0, 3000, 1500);
  delay(1000);
  ReleaseSteppers();
}


void StartStepper(uint8_t stepperNumber,
                  uint8_t sens,
                  float distance,
                  uint16_t vitesse)
{
  uint8_t s, dir, stp;
  switch (stepperNumber)
  {
    case 1:
      dir = dir1;
      stp = step1;
      s = !sens;
      break;
    case 2:
      dir = dir2;
      stp = step2;
      s = sens;
      break;
    case 3:
      dir = dir3;
      stp = step3;
      s = sens;
      break;
    case 4:
      dir = dir4;
      stp = step4;
      s = !sens;
      break;
  }
  digitalWrite(en, LOW);
  digitalWrite(dir, s);
  for (uint16_t i = 0; i < distance; i++)
  {
    digitalWrite(stp, LOW);
    delayMicroseconds(vitesse);
    digitalWrite(stp, HIGH);
    delayMicroseconds(vitesse);
  }
}

void GoRobot(uint8_t sens,
             float distance,
             uint16_t vitesse)
{
  digitalWrite(en, LOW);
  digitalWrite(dir1, sens);
  digitalWrite(dir2, !sens);
  digitalWrite(dir3, !sens);
  digitalWrite(dir4, sens);
  uint16_t desiredDistance = (distance * resolutionNema) / RouePerimeter;
  for (uint16_t i = 0; i < desiredDistance; i++)
  {
    digitalWrite(step1, LOW);
    digitalWrite(step2, LOW);
    digitalWrite(step3, LOW);
    digitalWrite(step4, LOW);
    delayMicroseconds(i < desiredDistance / 4 || i > 3 * (desiredDistance / 4) ? vitesse * 1.5 : vitesse);
    digitalWrite(step1, HIGH);
    digitalWrite(step2, HIGH);
    digitalWrite(step3, HIGH);
    digitalWrite(step4, HIGH);
    delayMicroseconds(i < desiredDistance / 4 || i > 3 * (desiredDistance / 4) ? vitesse * 1.5 : vitesse);
  }
}

void GoUntil(uint8_t sens, uint16_t vitesse)
{
  uint16_t i = 0;
  digitalWrite(en, LOW);
  digitalWrite(dir1, sens);
  digitalWrite(dir2, !sens);
  digitalWrite(dir3, !sens);
  digitalWrite(dir4, sens);
  while (sens ? !digitalRead(FrontSensor) : !digitalRead(BackSensor))
  {
    i++;
    vitesse = i > 500 ? 1500 : (3000 - (uint32_t)(1500.0 * ((500.0 - i) / 500.0)));
    digitalWrite(step1, LOW);
    digitalWrite(step2, LOW);
    digitalWrite(step3, LOW);
    digitalWrite(step4, LOW);
    delayMicroseconds(vitesse);
    digitalWrite(step1, HIGH);
    digitalWrite(step2, HIGH);
    digitalWrite(step3, HIGH);
    digitalWrite(step4, HIGH);
    delayMicroseconds(vitesse);
  }
}

void TurnRobot(uint8_t sens,
               float angle,
               uint16_t vitesse)
{
  digitalWrite(en, LOW);
  digitalWrite(dir1, !sens);
  digitalWrite(dir2, !sens);
  digitalWrite(dir3, !sens);
  digitalWrite(dir4, !sens);

  for (uint16_t i = 0; i < (angle * StepsPerAngle); i++)
  {
    digitalWrite(step1, LOW);
    digitalWrite(step2, LOW);
    digitalWrite(step3, LOW);
    digitalWrite(step4, LOW);
    delayMicroseconds(vitesse);
    digitalWrite(step1, HIGH);
    digitalWrite(step2, HIGH);
    digitalWrite(step3, HIGH);
    digitalWrite(step4, HIGH);
    delayMicroseconds(vitesse);
  }
}

void DroiteLateralle(float distance, uint16_t vitesse)
{
  digitalWrite(en, LOW);

  digitalWrite(dir1, 0);
  digitalWrite(dir2, 0);
  digitalWrite(dir3, 1);
  digitalWrite(dir4, 1);

  uint32_t desiredDistance = (distance * resolutionNema) / RouePerimeter;
  long n = (desiredDistance * 7) / 6;
  for (int i = 0; i < n; i++)
  {
    digitalWrite(step1, LOW);
    digitalWrite(step2, LOW);
    digitalWrite(step3, LOW);
    digitalWrite(step4, LOW);
    delayMicroseconds(vitesse);
    digitalWrite(step1, HIGH);
    digitalWrite(step2, HIGH);
    digitalWrite(step3, HIGH);
    digitalWrite(step4, HIGH);
    delayMicroseconds(vitesse);
  }
}

void GaucheLateralle(float distance, uint16_t vitesse)
{
  digitalWrite(en, LOW);

  digitalWrite(dir1, 1);
  digitalWrite(dir2, 1);
  digitalWrite(dir3, 0);
  digitalWrite(dir4, 0);

  uint32_t desiredDistance = (distance * resolutionNema) / RouePerimeter;
  long n = (desiredDistance * 7) / 6;
  for (int i = 0; i < n; i++)
  {
    digitalWrite(step1, LOW);
    digitalWrite(step2, LOW);
    digitalWrite(step3, LOW);
    digitalWrite(step4, LOW);
    delayMicroseconds(vitesse);
    digitalWrite(step1, HIGH);
    digitalWrite(step2, HIGH);
    digitalWrite(step3, HIGH);
    digitalWrite(step4, HIGH);
    delayMicroseconds(vitesse);
  }
}

void ReleaseSteppers()
{
  digitalWrite(en, HIGH);
}

void PrintingCallback()
{
  // lcd.setCursor(0,0);
  lcd.print("Estimation:");
  //lcd.setCursor(0,12);
  //lcd.print(EstimatedScore);
}
