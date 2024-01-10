#include <DS3231.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#include <Servo.h>

DS3231 rtc(SDA, SCL);

LiquidCrystal_I2C lcd(0x27, 24, 2);

HX711_ADC LoadCell(12, 13);

Servo myservo;

Time t;

int buttonValue = 0;
int analogInput = A0;
int remDays = 15;
int endDay = 0;
const int relayMixer = 7;
const int relayAirter = 8;
const int relayPump = 6;
const int relayPressdown = 5;
const int relayPressup = 4;
const int relayValveopen = 9;
const int relayValveclose = 10;
int extractAttempt = 0;
bool extractOn = true;
int ThermistorPin = A1;
int Vo;
int pos = 0;
float R1 = 10000;
float logR2;
float R2;
float tempValue;
float c1 = 1.009249522e-03;
float c2 = 2.378405444e-04;
float c3 = 2.019202697e-07;
long timeInt;
float initialWeight;
double flow;
double total = 0;
double LS = 0;
double fiveLiters = 5;
double tenLiters = 10;
int flowsensor = 2;
unsigned long currentTime;
unsigned long lastTime;
unsigned long pulse_freq;

void pulse()
{
  pulse_freq++;
}

void setup()
{
  Serial.begin(9600);
  rtc.begin();
  lcd.init();
  lcd.backlight();

  attachInterrupt(0, pulse, RISING);
  currentTime = millis();
  lastTime = currentTime;
  pinMode(relayMixer, OUTPUT);
  pinMode(relayAirter, OUTPUT);
  pinMode(relayPump, OUTPUT);
  pinMode(relayPressdown, OUTPUT);
  pinMode(relayPressup, OUTPUT);
  pinMode(relayValveopen, OUTPUT);
  pinMode(relayValveclose, OUTPUT);
  pinMode(flowsensor, INPUT);
  digitalWrite(relayMixer, LOW);
  digitalWrite(relayAirter, LOW);
  digitalWrite(relayPump, LOW);
  digitalWrite(relayPressdown, LOW);
  digitalWrite(relayPressup, LOW);
  digitalWrite(relayValveopen, LOW);
  digitalWrite(relayValveclose, LOW);
  myservo.attach(11);
  lcd.setCursor(5, 0);
  lcd.print("OLFerM");
  lcd.setCursor(0, 1);
  lcd.print("...TURNING ON...");
  LoadCell.begin();
  long stabilisingtime = 2000;
  LoadCell.start(stabilisingtime);
  LoadCell.setCalFactor(-19.5);
  delay(3000);
  lcd.clear();
}

void flowSensor()
{
  currentTime = millis();
  if (currentTime >= (lastTime + 1000))
  {
    lastTime = currentTime;
    flow = (pulse_freq / 7.5);
    LS = (flow / 45);
    total = total + LS;
    pulse_freq = 0;
    Serial.println(total);
    delay(100);
  }
}

void tempSensor()
{
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  tempValue = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  tempValue = tempValue - 273.15;
  delay(500);
}

void heatAir()
{
  digitalWrite(relayAirter, HIGH);
  Serial.print("HEATER ON");
}

void heatairOff()
{
  digitalWrite(relayAirter, LOW);
  Serial.print("HEATER OFF");
}

void orgmixOn()
{
  digitalWrite(relayMixer, HIGH);
  Serial.println("MIXER ON");
}

void orgmixOff()
{
  digitalWrite(relayMixer, LOW);
  Serial.println("MIXER OFF");
}

void extractProcess()
{
  digitalWrite(relayMixer, LOW);
  digitalWrite(relayAirter, LOW);
  digitalWrite(relayPump, LOW);

  while (extractOn == true)
  {
    if (extractAttempt == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Extracting.....");
      digitalWrite(relayValveopen, HIGH);
      delay(3500);
      digitalWrite(relayValveopen, LOW);
      lcd.setCursor(0, 1);
      lcd.print("Valve Opened...");
      delay(240000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Extracting.....");
      lcd.setCursor(0, 1);
      lcd.print("Pressing....");
      digitalWrite(relayPressdown, HIGH);
      delay(3700);
      digitalWrite(relayPressdown, LOW);
      delay(240000);
      extractAttempt++;
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Press Returning");
      digitalWrite(relayPressup, HIGH);
      delay(4700);
      digitalWrite(relayPressup, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Press Returned..");
      delay(500);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Valve Closing...");
      digitalWrite(relayValveclose, HIGH);
      delay(3500);
      digitalWrite(relayValveclose, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Valve Closed..");
      delay(500);
      extractOn = false;
    }
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Process Done...");
  delay(1000);
  exit;
}

void startProcess()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Processing ");
  String getTimes = rtc.getTimeStr();
  String nowTime = getTimes;
  delay(1500);

  while (remDays != endDay)
  {
    String getTimes = rtc.getTimeStr();
    tempSensor();
    lcd.setCursor(13, 0);
    lcd.print(int(tempValue));
    lcd.setCursor(15, 0);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print(remDays);
    lcd.setCursor(2, 1);
    lcd.print("Days Remaining");

    if (getTimes == nowTime)
    {
      remDays--;
    }

    Serial.println(getTimes);
    Serial.println(tempValue);
    Serial.println(nowTime);

    if (tempValue >= 55)
    {
      heatairOff();
      orgmixOff();
    }
    else if (tempValue <= 45)
    {
      heatAir();
      orgmixOn();
    }
  }
  if (remDays == endDay)
  {
    extractProcess();
  }
}

void weightScale()
{
  LoadCell.update();

  if (millis() > timeInt + 250)
  {
    initialWeight = LoadCell.getData();
    Serial.println(initialWeight);
    timeInt = millis();
  }
}

void servoDump()
{
  weightScale();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DUMPING..");
  lcd.setCursor(0, 1);
  lcd.print("Organic Material");
  delay(2000);
  myservo.write(180);
  delay(10000);
  myservo.write(90);
  delay(10000);
  myservo.write(0);
  delay(10000);
  myservo.write(90);
  delay(5000);
}

void weighNPKthree()
{
  lcd.clear();
  while (initialWeight < 3000)
  {
    weightScale();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Put 3KG NPK");
    lcd.setCursor(0, 1);
    lcd.print(initialWeight);
    lcd.setCursor(7, 1);
    lcd.print("Grams");
    delay(1000);
  }
  if (initialWeight >= 3000)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Inputted 3KG NPK");
    delay(3000);
    servoDump();
  }
}

void weighNPKsix()
{
  lcd.clear();
  while (initialWeight < 6000)
  {
    weightScale();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Put 6KG NPK");
    lcd.setCursor(0, 1);
    lcd.print(initialWeight);
    lcd.setCursor(7, 1);
    lcd.print("Grams");
    delay(1000);
  }
  if (initialWeight >= 6000)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Inputted 6KG NPK");
    delay(3000);
    servoDump();
  }
}

void loop()
{

  lcd.setCursor(0, 0);
  lcd.print("YELLOW BTN = 3KG");
  lcd.setCursor(0, 1);
  lcd.print("GREEN BTN = 6KG");
  t = rtc.getTime();

  buttonValue = analogRead(analogInput);

  if (buttonValue >= 200 && buttonValue <= 400)
  {
    total = 0;
    lcd.clear();
    weighNPKthree();

    while (total < fiveLiters)
    {
      delay(1000);
      flowSensor();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("3kg Pump Working.");
      lcd.setCursor(0, 1);
      lcd.print(total);
      lcd.setCursor(5, 1);
      lcd.print(" L");
      digitalWrite(relayPump, HIGH);
    }

    if (total >= fiveLiters)
    {
      digitalWrite(relayPressdown, HIGH);
      delay(2000);
      digitalWrite(relayPressdown, LOW);
      delay(2000);
      lcd.clear();
      lcd.setCursor(0, 0);
      Serial.println("Pump Stop..");
      digitalWrite(relayPump, LOW);
      delay(2000);
      startProcess();
    }
  }

  if (buttonValue >= 1000 && buttonValue <= 1023)
  {
    total = 0;
    lcd.clear();
    weighNPKsix();

    while (total < tenLiters)
    {
      delay(1000);
      flowSensor();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("6kg Pump Working.");
      lcd.setCursor(0, 1);
      lcd.print(total);
      lcd.setCursor(5, 1);
      lcd.print(" L");
      digitalWrite(relayPump, HIGH);
    }

    if (total >= tenLiters)
    {
      digitalWrite(relayPressdown, HIGH);
      delay(2000);
      digitalWrite(relayPressdown, LOW);
      delay(2000);
      lcd.clear();
      lcd.setCursor(0, 0);
      Serial.println("Pump Stop..");
      digitalWrite(relayPump, LOW);
      delay(2000);
      startProcess();
    }
  }
  Serial.println(buttonValue);
  delay(1000);
}
