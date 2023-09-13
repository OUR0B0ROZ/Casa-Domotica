
#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo

#include <SPI.h>
#include <MFRC522.h>
 
#define SS_PIN 2
#define RST_PIN 15

#define BLUE 4 
#define GREEN 22
#define RED 5 
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Crea la instancia MFRC522
 
int pos = 0;    // variable to store the servo position
int servoPin = 13;

int pinZumbador=14;
int canal=0, frec=2000, resolucion=8;
//Inicializamos las variables de la salid
void setup() 
{
  Serial.begin(115200);   // Inicializa la comunicacion serial
  SPI.begin();          // Inicializa el bus SPI
  mfrc522.PCD_Init();   // Inicializa el MFRC522
  pinMode(BLUE, OUTPUT);
  digitalWrite(BLUE, LOW);
  pinMode(GREEN, OUTPUT);
  digitalWrite(GREEN, LOW);
  pinMode(RED, OUTPUT);
  digitalWrite(RED, LOW);
  Serial.println("Ponga su Tarjeta para la lectura...");
  Serial.println();
  
  //Declaramos pin donde se conecta el zumbador

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 1000, 2000); 

}
void loop() 
{
  // Mirando para nuevas tarjeras
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Selecciona una de las tarjetas
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }
  // Muestra el UID sobre el Monitor Serial
  Serial.print("UID tag :");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  if (content.substring(1) == "33 D5 24 94") //Cambie aqui el UID de las tarjetas que usted desea dar acceso
  {
  Serial.println("Acceso Denegado");
  myservo.write(180);    // tell servo to go to position in variable 'pos'            // waits 15ms for the servo to reach the position
    // tell servo to go to position in variable 'pos'  
  }

  if (content.substring(1) == "F3 B5 E0 10") //Cambie aqui el UID de las tarjetas que usted desea dar acceso
  {
  Serial.println("Acceso Autorizado");
  myservo.write(0);
delay(1000);
  myservo.write(180);  
   
  }
    if (content.substring(1) == "1A 6B 11 B1") //Cambie aqui el UID de las tarjetas que usted desea dar acceso
  {
    Serial.println("Acceso Autorizado");
   myservo.write(0);
   delay(1000);
  myservo.write(180);  
  }
  
}
