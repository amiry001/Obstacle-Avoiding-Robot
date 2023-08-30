#include <AFMotor.h>
#include <Servo.h>      
#define trigPin1 A0 //orta
#define echoPin1 A1
#define trigPin2 A2 //sag
#define echoPin2 A3
#define trigPin3 A5 //sol
#define echoPin3 A4
#define LEDgreenRight 9
#define LEDgreenLeft 13
#define LEDredRight 7  
#define LEDredLeft 11
#define Buzzer 5    
long duration, distance, MidSensor,RightSensor,LeftSensor;

#include <SoftwareSerial.h>
#define BT_SERIAL_RX 0
#define BT_SERIAL_TX 1
SoftwareSerial BluetoothSerial(BT_SERIAL_RX, BT_SERIAL_TX);

AF_DCMotor rightBack(1);                       
AF_DCMotor rightFront(2);
AF_DCMotor leftFront(3);
AF_DCMotor leftBack(4);
Servo servoLook; 

byte maxDist = 300;   //en fazla ölçülen mesafe                       
byte stopDist = 55;                         
float timeOut = 2*(maxDist+10)/100/340*1000000;  //süreyi mesafeye çevirme formülü

byte motorSpeed = 100;                          
int motorOffset = 10;                          
int turnSpeed = 200;                        

void setup() {
  rightBack.setSpeed(motorSpeed);               
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed); 
  leftBack.setSpeed(motorSpeed);
  rightBack.run(RELEASE);    //ilk çalıştırdığımızda hepsi bekleme halinde oluyor         
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
  servoLook.attach(10);      //servomotor yeri pin 10 olarak atadik               
  pinMode(trigPin1,OUTPUT);      //trig = OUTPUT                
  pinMode(echoPin1,INPUT);       //echo = INPUT
  Serial.begin(9600);        //bılgısayar ıle arduıno arasinda bağlantı hızı (Serial.begin bağlantıyı sağlıyor) 
  pinMode(LEDgreenRight, OUTPUT); // Declare the LED as an output
  pinMode(LEDgreenLeft, OUTPUT); // Declare the LED as an output
  pinMode(LEDredRight, OUTPUT); // Declare the LED as an output
  pinMode(LEDredLeft, OUTPUT); // Declare the LED as an output
  pinMode(Buzzer, OUTPUT); // Declare the Buzzer as an output
  BluetoothSerial.begin(9600); //bluetooth baglatisini kurmak     
}

void loop() {
  servoLook.write(110);           //öne bakış dereces1 180 ile 0 arasındadır                   
  delay(750);                     //ilk basladiginda one bakmiyorsa öne döndükten sonra ya da bakiyorsa bekleme süresi
  int distance = getDistance();                  
  if(distance >= stopDist)                       
  {
    moveForward();
  }
  while(distance >= stopDist)           
  {
    distance = getDistance();
    delay(250);           //0,25 sanıye boyunca getDistance() fonksiyonunu çalıştır
  }
  stopMove();       
  digitalWrite(Buzzer, HIGH);    
  delay(200);      
  digitalWrite(Buzzer, HIGH);    
  delay(200);                  
  int turnDir = checkDirection();               
  Serial.print(turnDir);  
  switch (turnDir)                              
  {
    case 0:      
    Back();
    delay(500);
    turnLeft (750);
    break;

    case 1:   
    Back();
    delay(500);
    turnRight (750);
}
}

int getDistance()
{
  unsigned long pulseTime;                      
  int distance;                                  
  digitalWrite(trigPin1, HIGH);    //high dalga gönermesi için                  
  delayMicroseconds(10);           // 10 microsecond boyunca dalga gönder
  digitalWrite(trigPin1, LOW);     //artık dalga gönderme 
  pulseTime = pulseIn(echoPin1, HIGH, timeOut);     //dalga alma komutu başlatmak   
  distance = (float)pulseTime * 340 / 2 / 10000;  //??????????????  
  return distance;

  BluetoothSerial.println("Mid Sensor");
  BluetoothSerial.println("Distance: ");
  BluetoothSerial.println(distance);
  BluetoothSerial.println(" cm");
  Serial.println("test"); 
  delay(100);  
           
  digitalWrite(trigPin2, HIGH);                      
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  pulseTime = pulseIn(echoPin2, HIGH, timeOut);        
  distance = (float)pulseTime * 340 / 2 / 10000;    
  return distance;

  BluetoothSerial.println("Right Sensor");
  BluetoothSerial.println("Distance: ");
  BluetoothSerial.println(distance);
  BluetoothSerial.println(" cm");
  Serial.println("test"); 
  delay(100);  

  digitalWrite(trigPin3, HIGH);                      
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  pulseTime = pulseIn(echoPin3, HIGH, timeOut);        
  distance = (float)pulseTime * 340 / 2 / 10000;    
  return distance;

  BluetoothSerial.println("Left Sensor");
  BluetoothSerial.println("Distance: ");
  BluetoothSerial.println(distance);
  BluetoothSerial.println(" cm");
  Serial.println("test"); 
  delay(100);
}
int getDistance2()                            
{
  unsigned long pulseTime;                      
  int distance;                                  
  digitalWrite(trigPin2, HIGH);                      
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  pulseTime = pulseIn(echoPin2, HIGH, timeOut);        
  distance = (float)pulseTime * 340 / 2 / 10000;    
  return distance;

  BluetoothSerial.println("Right Sensor");
  BluetoothSerial.println("Distance: ");
  BluetoothSerial.println(distance);
  BluetoothSerial.println(" cm");
  Serial.println("test"); 
  delay(100);  
}
int getDistance3()                            
{
  unsigned long pulseTime;                      
  int distance;                                  
  digitalWrite(trigPin3, HIGH);                      
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  pulseTime = pulseIn(echoPin3, HIGH, timeOut);        
  distance = (float)pulseTime * 340 / 2 / 10000;    
  return distance;

  BluetoothSerial.println("Left Sensor");
  BluetoothSerial.println("Distance: ");
  BluetoothSerial.println(distance);
  BluetoothSerial.println(" cm");
  Serial.println("test"); 
  delay(100);  
}

void moveForward()                            
{
  rightBack.setSpeed(100); 
  rightBack.run(FORWARD); 
  rightFront.setSpeed(100); 
  rightFront.run(FORWARD); 
  leftBack.setSpeed(100);
  leftBack.run(FORWARD); 
  leftFront.setSpeed(100);
  leftFront.run(FORWARD);

  BluetoothSerial.println("Moving Forward..");
}
void stopMove()                               
{
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  BluetoothSerial.println("Stopped!");
}

void turnLeft(int duration)                          
{
  digitalWrite(LEDredRight, HIGH); 
  digitalWrite(LEDgreenLeft, HIGH); 
  rightBack.setSpeed(motorSpeed+turnSpeed);               
  rightFront.setSpeed(motorSpeed+turnSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset+turnSpeed);
  leftBack.setSpeed(motorSpeed+motorOffset+turnSpeed);
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  delay(duration);
  rightBack.setSpeed(motorSpeed);                         
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  BluetoothSerial.println("Turning Left");
}

void turnRight(int duration)                         
{
  digitalWrite(LEDgreenRight, HIGH); 
  digitalWrite(LEDredLeft, HIGH); 
  rightBack.setSpeed(motorSpeed+turnSpeed);             
  rightFront.setSpeed(motorSpeed+turnSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset+turnSpeed);
  leftBack.setSpeed(motorSpeed+motorOffset+turnSpeed);
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  delay(duration);
  rightBack.setSpeed(motorSpeed);                     
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  BluetoothSerial.println("Turning Right");

}

void Back()
{
  rightBack.setSpeed(100);
  rightBack.run(BACKWARD); 
  rightFront.setSpeed(100); 
  rightFront.run(BACKWARD);
  leftBack.setSpeed(100);
  leftBack.run(BACKWARD); 
  leftFront.setSpeed(100); 
  leftFront.run(BACKWARD); 
}


int checkDirection()                                         
{
  int distances [2] = {0,0};                    
  int turnDir = 0;                                   
  servoLook.write(180);      //servonun açısı                               
  delay(500);                //yarım saniye boyunca bak
  distances [0] = getDistance2();                            
  servoLook.write(40);                                         
  delay(1000);
  distances [1] = getDistance3();                      
  if (distances[0]>=300 && distances[1]>=300)                
    turnDir = 0;
  else if (distances[0]<=stopDist && distances[1]<=stopDist) 
    turnDir = 0;
  else if (distances[0]>=distances[1])                        
    turnDir = 0;
  else if (distances[0]<distances[1])                          
    turnDir = 1;
  return turnDir;
}

int accelerate()                                 
{
  for (int i=0; i<motorSpeed; i++)                
  {
    rightBack.setSpeed(i+motorOffset);                  
    rightFront.setSpeed(i+motorOffset);
    leftFront.setSpeed(i+motorOffset);
    leftBack.setSpeed(i+motorOffset);
    delay(10);
  }
}

int decelerate()                               
{
  for (int i=motorSpeed; i!=0; i--)           
  {
  rightBack.setSpeed(i+motorOffset);                       
    rightFront.setSpeed(i+motorOffset);
    leftFront.setSpeed(i+motorOffset);
    leftBack.setSpeed(i+motorOffset);
    delay(10);
  }
}
