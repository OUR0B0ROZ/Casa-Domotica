#include <Arduino.h>
#ifdef ESP32
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"

// Rename the credentials.sample.h file to credentials.h and 
// edit it according to your router configuration

#define WIFI_SSID "linksys"
#define WIFI_PASS "siegheil"

fauxmoESP fauxmo;

// -----------------------------------------------------------------------------

#define SERIAL_BAUDRATE     115200

#define BLYNK_TEMPLATE_ID "TMPL2L6vw1NFC"
#define BLYNK_TEMPLATE_NAME "PROYECTO CASA DOMOTICA"
#define BLYNK_AUTH_TOKEN "UDn-1BJJ_eKzBCqo3P-YpZwqLcqz07QO"

#define BLYNK_PRINT Serial
#include <WiFi.h>
//#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp32.h>

#include <DHT.h>


char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "linksys";  // type your wifi name
char pass[] = "siegheil";  // type your wifi password

BlynkTimer timer;

const int paro = 35;  // the number of the pushbutton pin
const int paro_led = 14;    // the number of the LED pin

const int arranque = 34;  // the number of the pushbutton pin
const int arranque_led = 12;    // the number of the LED pin
// variables will change:
int arranqueState = 0;  // variable for reading the pushbutton status
int paroState = 0;  // variable for reading the pushbutton status

int arranque_paro = 0;  // variable for reading the pushbutton status

#define DHTPIN 4 //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);


#define LED_BLUE            26
#define LED_WHITE           25

#define MOTOR 13 //Connect Out pin to D2 in NODE MCU


#define ID_BLUE             "patio"
#define ID_WHITE            "piscina"
int lectura_gas = 0 ;
#define         MQ1                       (33)     //define la entrada analogica para el sensor
#define         RL_VALOR             (5)     //define el valor de la resistencia mde carga en kilo ohms
#define         RAL       (9.83)  // resistencia del sensor en el aire limpio / RO, que se deriva de la                                             tabla de la hoja de datos
#define         GAS_LP                      (33)
float           LPCurve[3]  =  {2.3,0.21,-0.47};
float           Ro           =  10;
int pin;
BLYNK_WRITE(V4)
{
  pin=param.asInt();
 
  
  }
void sendSensor()
{
    lectura_gas =porcentaje_gas(lecturaMQ(MQ1)/Ro,GAS_LP);
  
    paroState = digitalRead(paro);
  arranqueState = digitalRead(arranque);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (paroState == HIGH || pin == 0) {
    // turn LED on:
    arranque_paro = 0;
    }
  if (arranqueState == HIGH || pin == 1 ) {
    // turn LED on:
      arranque_paro = 1;
  } 
  if (arranque_paro== 1) {
    // turn LED on:
      digitalWrite(arranque_led,HIGH);
      digitalWrite(paro_led,LOW);
  } 
  if (arranque_paro== 0) {
    // turn LED on:
        digitalWrite(arranque_led,LOW);
      digitalWrite(paro_led,HIGH);
  }
  
  int estado_motor = 0;
  int analogVolts = analogReadMilliVolts(33);
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
   if ( lectura_gas >= 750)
    {
      estado_motor = 1;
    }
     if ( lectura_gas < 750)
    {
      estado_motor = 0;
    }
    digitalWrite(MOTOR,estado_motor);

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V0, t);
    Blynk.virtualWrite(V1, h);
    Blynk.virtualWrite(V2, lectura_gas);
    Blynk.virtualWrite(V3, estado_motor);
    Serial.print("Temperature : ");
    Serial.print(t);
    Serial.print("    Humidity : ");
    Serial.print(h);
    Serial.print(" Gas : ");
    Serial.printf("ADC analog value = %d\n",analogVolts);
       Serial.print("LP:");
   Serial.print(lectura_gas);
   Serial.print( "ppm" );
    Serial.print(" Estado motor: ");
    Serial.print(estado_motor);
    Serial.println(" ");
   
} 
void wifiSetup() {

    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

    // Connect
    Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Connected!
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

}

void setup() {

    // Init serial port and clean garbage
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println();
    Serial.println();

    // LEDs
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_WHITE, OUTPUT);
    pinMode(MOTOR, OUTPUT);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_WHITE, LOW);
    digitalWrite(MOTOR, LOW);
    pinMode(paro_led , OUTPUT);
    pinMode(arranque_led , OUTPUT);
    // initialize the pushbutton pin as an input:
    pinMode(paro, INPUT);
    pinMode(arranque, INPUT);

    // Wifi
    wifiSetup();

    // By default, fauxmoESP creates it's own webserver on the defined port
    // The TCP port must be 80 for gen3 devices (default is 1901)
    // This has to be done before the call to enable()
    fauxmo.createServer(true); // not needed, this is the default value
    fauxmo.setPort(80); // This is required for gen3 devices

    // You have to call enable(true) once you have a WiFi connection
    // You can enable or disable the library at any moment
    // Disabling it will prevent the devices from being discovered and switched
    fauxmo.enable(true);

    // You can use different ways to invoke alexa to modify the devices state:
    // "Alexa, turn yellow lamp on"
    // "Alexa, turn on yellow lamp
    // "Alexa, set yellow lamp to fifty" (50 means 50% of brightness, note, this example does not use this functionality)

    // Add virtual devices
    fauxmo.addDevice(ID_BLUE);
    fauxmo.addDevice(ID_WHITE);

    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
        
        // Callback when a command from Alexa is received. 
        // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
        // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
        // Just remember not to delay too much here, this is a callback, exit as soon as possible.
        // If you have to do something more involved here set a flag and process it in your main loop.
        
        Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

        // Checking for device_id is simpler if you are certain about the order they are loaded and it does not change.
        // Otherwise comparing the device_name is safer.

        if (strcmp(device_name, ID_BLUE)==0) {
            digitalWrite(LED_BLUE, state ? HIGH : LOW);
        } else if (strcmp(device_name, ID_WHITE)==0) {
            digitalWrite(LED_WHITE, state ? HIGH : LOW);
        }


    });
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  timer.setInterval(700L, sendSensor);
   Serial.println("Iniciando ...");
   //configuracion del sensor
  Serial.print("Calibrando...\n");
  Ro = Calibracion(MQ1);                        //Calibrando el sensor. Por favor de asegurarse que el sensor se encuentre en una zona de aire limpio mientras se calibra
  Serial.print("Calibracion finalizada...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
}

void loop() {

    // fauxmoESP uses an async TCP server but a sync UDP server
    // Therefore, we have to manually poll for UDP packets
    fauxmo.handle();

    // This is a sample code to output free heap every 5 seconds
    // This is a cheap way to detect memory leaks
    static unsigned long last = millis();
    if (millis() - last > 5000) {
        last = millis();
        Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
    }

    // If your device state is changed by any other means (MQTT, physical button,...)
    // you can instruct the library to report the new state to Alexa on next request:
    // fauxmo.setState(ID_YELLOW, true, 255);
     Blynk.run();
     timer.run();
}


float calc_res(int raw_adc)
{
  return ( ((float)RL_VALOR*(1023-raw_adc)/raw_adc));
}
 
float Calibracion(float mq_pin){
  int i;
  float val=0;
    for (i=0;i<50;i++) {                                                                               //tomar múltiples muestras
    val += calc_res(analogRead(mq_pin));
    delay(500);
  }
  val = val/50;                                                                                         //calcular el valor medio
  val = val/RAL;
  return val;
}
 
float lecturaMQ(int mq_pin){
  int i;
  float rs=0;
  for (i=0;i<5;i++) {
    rs += calc_res(analogRead(mq_pin));
    delay(50);
  }
rs = rs/5;
return rs;
}
 
int porcentaje_gas(float rs_ro_ratio, int gas_id){
   if ( gas_id == GAS_LP ) {
     return porcentaje_gas(rs_ro_ratio,LPCurve);
   }
  return 0;
}
 
int porcentaje_gas(float rs_ro_ratio, float *pcurve){
  return (pow(10, (((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
