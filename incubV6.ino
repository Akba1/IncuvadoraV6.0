#include <Wire.h>
#include <ESP8266WiFi.h>

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <EEPROM.h> 

#include <Adafruit_AHT10.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Servo.h>
#include <PID_v1.h> 
#include "RTClib.h"
#include "time.h"


#define Start       16//D0
//#define SCL       D1//D1
//#define SDA       D2//D2
#define Indicadores 0//D3
#define humificador 2//D4
#define Vent_EXT    14//D5
#define Movimiento  12//D6
#define Calor       13//D7 + en boot
#define Vent_INT    15//D8


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET   -1 

unsigned int myWiFiTimeout        =  15000L;  //  3.2s WiFi connection timeout 

//Week Days
String dds;
String daysOfTheWeek[7]={"Domingo", "Lunes", "Martes", "Miércoles", "Jueves", "Viernes", "Sábado"};
//Month names
//String months[12]={"Enero", "Febrero", "Marzo", "Abril", "Mayo", "Junio", "Julio", "Agosto", "Septiembre", "Octubre", "Noviembre", "Diciembre"};
/// Fase Inicial
int8_t inicial=0;
bool fase_1 = true;
bool fase_2 = true;
int8_t rst = 0;
////////PARAMEPTROS PID///PID_IN/PID_OUT/SET POINTS/
///const double Kp=20,Ki=10,Kd=0; USADO HASTA 10/09/2021:23:50
//const double Kp=1,Ki=13,Kd=0; USADO HASTA 15/09/2021:3:00
const double Kp=17,Ki=8,Kd=5; //USADO HASTA 17/09/2021:11:47 Funciona bien  
//const double Kp=18,Ki=5,Kd=4;
double Input,Output, SP_T, SP_H;
//Variables de Control
float temperature;  
float humidity;
////Angulos del Servo
int16_t angulo=0;
byte next = 0;
//byte direccion = 0;
//int8_t deltaAngulo = 30;
///Control de tiempos
int segundo,minuto,hora,dia,mes,anio;
///Variable auxiliar
String cuerda;
//HAY NIEBLA?
bool hay_niebla = false;
//PANTALLA
bool esta_encendida = false;
int8_t screen_state=2;
int8_t contador=0;
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_temp[] =
{ B00000000, B10000000,
  B00000001, B11100000,
  B00000001, B10000000,
  B00000001, B11100000,
  B00000001, B10000000,
  B00000001, B11100000,
  B00000001, B10000000,
  B00000001, B11100000,
  B00000001, B10000000,
  B00000001, B11100000,
  B00000011, B11000000,
  B00000111, B11100000,
  B00001111, B11110000,
  B00001111, B11110000,
  B00000111, B11100000,
  B00000011, B1000000 };

static const unsigned char PROGMEM logo_hum[] =
{ B00000000, B10000000,
  B00000001, B11000000,
  B00000011, B1100000,
  B00000111, B1100000,
  B00000111, B11100000,
  B00001111, B11110000,
  B00001111, B11110000,
  B00011111, B11111000,
  B00011111, B11111000,
  B00111111, B11111100,
  B00111111, B11111100,
  B00111111, B11111100,
  B00001111, B11110000,
  B00001111, B11110000,
  B00000111, B11100000,
  B00000001, B10000000 };
static const unsigned char PROGMEM fase2[] =
{ B00000001, B10000000,
  B00000011, B11000000,
  B00000111, B11100000,
  B00000111, B11100000,
  B00000111, B11100000,
  B00001111, B11110000,
  B00001111, B11110000,
  B00011111, B11111000,
  B00011111, B11110000,
  B00101110, B11101100,
  B00010101, B01011100,
  B00111011, B10111100,
  B00001111, B11110000,
  B00001111, B11110000,
  B00000111, B11100000,
  B00000001, B10000000 };
static const unsigned char PROGMEM fase1[] =
{ B00000001, B10000000,
  B00000011, B11000000,
  B00000111, B11100000,
  B00000111, B11100000,
  B00000111, B11100000,
  B00001111, B11110000,
  B00001111, B11110000,
  B00011111, B11111000,
  B00011111, B11111000,
  B00111111, B11111100,
  B00111111, B11111100,
  B00111111, B11111100,
  B00001111, B11110000,
  B00001111, B11110000,
  B00000111, B11100000,
  B00000001, B10000000 };
  static const unsigned char PROGMEM fase0[] =
{ B00000001, B10000000,
  B00000010, B01000000,
  B00000100, B00100000,
  B00000100, B00100000,
  B00000100, B00100000,
  B00001000, B00010000,
  B00001001, B10010000,
  B00010011, B11001000,
  B00010111, B11101000,
  B00101111, B11110100,
  B00101111, B11110100,
  B00101111, B11110100,
  B00001011, B11010000,
  B00001000, B00010000,
  B00000100, B00100000,
  B00000001, B10000000 };

const char* ssid = "Vissoni Toj";
const char* password = "082020Xareni";

//AsyncWebServer server(80);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);// Crear un objeto SCREEN
WiFiClient  client; //Crear un cliente wifi
RTC_DS3231 rtc;//Crear un objeto RTC
Adafruit_AHT10 aht;//Creando un objeto AHT10
Servo MG995_Servo; //Greando un objeto Servo
PID myPID(&Input, &Output, &SP_T, Kp, Ki, Kd, DIRECT);//Creanto un objeto PID
void initWiFi() {
    WiFi.mode(WIFI_STA);
    unsigned long startWiFi = millis();
    WiFi.begin(ssid, password);
    display.clearDisplay();
    display.setCursor(0,0);
    cuerda = ".";
    do
    {
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.print(cuerda);
      display.display();
      delay(500);
      if ( (WiFi.status() == WL_CONNECTED) || (millis() > startWiFi + myWiFiTimeout) )
        break;
    } while(WiFi.status() != WL_CONNECTED);
    
    display.clearDisplay();
    IPAddress broadCast = WiFi.localIP();
    Serial.print(broadCast);
    broadCast[4] = 255;
    //String(WiFi.localIP().toString);
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(broadCast);
    display.display();
    delay(500);
    //WiFi.setAutoReconnect(true);
    //WiFi.persistent(true);
    display.setCursor(0,20);
    display.print("TRY");
    ArduinoOTA.begin();
    display.display();
}
void initOled(){
  while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    //digitalWrite(Indicadores,1);
    delay(900);
    //digitalWrite(Indicadores,0);
  }
}
void initRTC(){
  while(!rtc.begin()) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("RTC!ERROR");
    display.display();
    delay(500);
   }
  if (rtc.lostPower()) {
      // Fijar a fecha y hora de compilacion
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      
      // Fijar a fecha y hora específica. En el ejemplo, 21 de Enero de 2016 a las 03:00:00
      // rtc.adjust(DateTime(2016, 1, 21, 3, 0, 0));
   }
   display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("RTC!CHECK");
    display.display();
    delay(500);
   }
void initAHT(){
  while(!aht.begin(&Wire,0x3C)) {
    display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("AHT!ERR");
  display.display();
  delay(500);
  display.clearDisplay();
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("AHT!CHECK");
  display.clearDisplay();
  display.display();
  display.clearDisplay();
}

void getSensorReadings(){
  sensors_event_t hum, temp;
  aht.getEvent(&hum, &temp);
  temperature = temp.temperature;
  humidity = hum.relative_humidity;
}

void setPoints(double set_temp, double set_humi){
  Input = temperature;
  SP_T = set_temp;
  SP_H = set_humi;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(80, 255);
}
void step_servo(){
  int16_t retardo = 4;
  byte avance = 2;
  switch(next){
    case 0:
      for(angulo = 0;angulo<60;angulo=angulo+avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 1;
      break;
    case 1:
      for(angulo = 60;angulo<90;angulo=angulo+avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 2;
      break;
    case 2:
      for(angulo = 90;angulo<130;angulo=angulo+avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 3;
      break;
    case 3:
      for(angulo = 130;angulo<170;angulo=angulo+avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 4;
      break;
    case 4:
      for(angulo = 170;angulo>130;angulo=angulo-avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 5;
      break;
    case 5:
      for(angulo = 130;angulo>90;angulo=angulo-avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 6;
      break;
    case 6:
      for(angulo = 90;angulo>44;angulo=angulo-avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 7;
      break;
    case 7:
      for(angulo = 44;angulo>0;angulo=angulo-avance){
        MG995_Servo.write(angulo);
        delay(retardo); 
      }
      next = 0;
      break;
    default:
      next = 0;
      break;
  } 
}
void boton_Humedad(){
    digitalWrite(humificador,LOW);
    delay(1000);
    digitalWrite(humificador,HIGH);
}
void ControlHUM(){
  if((humidity < SP_H-0.5)and(not hay_niebla)){
        boton_Humedad();
        hay_niebla = true;
  }else if((humidity > SP_H+0.5)and(hay_niebla)){
        boton_Humedad();
        hay_niebla = false;
  }
}
void Control(){
  getSensorReadings();
  Input = temperature;
  myPID.Compute();
  analogWrite(Calor, Output);
  EEPROM.write(13,Output);      
  EEPROM.commit(); 
  ControlHUM();
  if(temperature>=38){
     analogWrite(Vent_EXT,100);  
  }else if(temperature<=37.9){
     analogWrite(Vent_EXT,255);  
  }
  if((inicial==1)and(minuto%15==0)and(segundo==0)){
      step_servo(); 
  }
}

void resetEe(){
  for (int i = 0; i < 20; i++) {
      EEPROM.write(i, 0);
      delay(100);
    }
  EEPROM.commit();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("RESET EEPROM");
  display.display();
  delay(800);
}

void LeerFecha(int aa, int mm, int dd, int hh, int mi, int se){
    dia = int(EEPROM.read(dd));
    mes = int(EEPROM.read(mm));
    anio = int(EEPROM.read(aa));
    hora = int(EEPROM.read(hh));
    minuto = int(EEPROM.read(mi));
    segundo = int(EEPROM.read(se));
    cuerda = String(dia) + "-" + String(mes) + "-20" + String(anio);
}
void Iicializar(int lapso, int a, int m, int d, int h, int mi, int se){
  DateTime now = rtc.now();      // D  /H/M/S/
  DateTime future (now + TimeSpan(lapso,0,0,0));
  anio = future.year();
  mes = future.month();
  dia = future.day();
  hora = future.hour();
  minuto = future.minute();
  segundo = future.second();
  
  EEPROM.write(a, anio-2000);
  EEPROM.write(m, mes);
  EEPROM.write(d, dia);
  EEPROM.write(h, hora);
  EEPROM.write(mi, minuto);
  EEPROM.write(se, segundo);
  if (EEPROM.commit()) {
    cuerda = "GUARDADO  "+String(anio)+"-"+String(mes)+"-"+String(dia)+" "+String(hora)+":"+String(minuto)+":"+String(segundo);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(cuerda);
    display.display();
    delay(500);
  } else {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("ERROR! Fallo al guardar");
    display.display();
    delay(500);
  }

}

void leerRTC()
{
   DateTime now = rtc.now();
    LeerFecha(1,2,3,4,5,6);
    if((now.year()==anio+2000)and(now.month()==mes)and(now.day()==dia)){
      if((now.hour()==hora)and(now.minute()==minuto)and(now.second()>=segundo)){
        EEPROM.write(0,2);      
        EEPROM.commit();
      }
    }
    LeerFecha(7,8,9,10,11,12);
    if((now.year()==anio+2000)and(now.month()==mes)and(now.day()==dia)){
      if((now.hour()==hora)and(now.minute()==minuto)and(now.second()>=segundo)){
        resetEe();
        digitalWrite(Indicadores,0);
        if(hay_niebla){
          boton_Humedad();
          hay_niebla = false;
        }
        analogWrite(Calor,0);
        analogWrite(Vent_INT,255);
        analogWrite(Vent_EXT,255);
      }
    }
    anio = now.year();
    mes = now.month();
    dia = now.day();
    hora = now.hour();
    minuto = now.minute();
    segundo = now.second();
    dds = daysOfTheWeek[now.dayOfTheWeek()];    
}
void reiniciar(){
    rst=0;
    resetEe();
    digitalWrite(Indicadores,0);
    if(hay_niebla){
        boton_Humedad();
        hay_niebla = false;
    }
    fase_1 = true;
    fase_2 = true;
    analogWrite(Calor,0);
    analogWrite(Vent_INT,255);
    analogWrite(Vent_EXT,255);
    while(angulo!=0){
      step_servo(); 
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("REINICIADO");
    display.display();
    delay(800);
}
void oled_print(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  cuerda = String(hora)+ ":" + String(minuto) + ":" + String(segundo) + "   FOCO"+String(Output);
  display.setCursor(0,35);
  display.print(cuerda);
  cuerda = "SERVO"+String(angulo)+"   NIEBLA "+String(hay_niebla);
  display.setCursor(0,41);
  display.print(cuerda);
  
  display.setTextSize(2);
  if(inicial==0)
        display.drawBitmap(56, 24, fase1, LOGO_WIDTH,LOGO_HEIGHT, WHITE);
  if(inicial!=0){
    cuerda = String(temperature)+"C";
    display.drawBitmap(0, 0, logo_temp, LOGO_WIDTH,LOGO_HEIGHT, WHITE);
    display.setCursor(16, 0);
    display.print(cuerda);
    cuerda = String(humidity)+"%";
    display.drawBitmap(0, 16, logo_hum, LOGO_WIDTH,LOGO_HEIGHT, WHITE);
    display.setCursor(16, 16);
    display.print(cuerda);
    if(inicial==1)
      display.drawBitmap(102, 0, fase1, LOGO_WIDTH,LOGO_HEIGHT, WHITE);
    if(inicial==2)
      display.drawBitmap(102, 0, fase2, LOGO_WIDTH,LOGO_HEIGHT, WHITE);
  
  }
  if(rst>0){
    display.setCursor(0,48);
    display.print(String(rst));
  }
  display.display();
  delay(900);   
}  
void setup() {
  pinMode(humificador,OUTPUT);
  digitalWrite(humificador,HIGH);
  //delay(10);
  MG995_Servo.attach(Movimiento,1000,2400);
  //angulo = EEPROM.read(13);
  MG995_Servo.write(angulo);
  pinMode(Indicadores,OUTPUT);
  digitalWrite(Indicadores,1);
  pinMode(Calor,OUTPUT);
  Output = EEPROM.read(13);
  analogWrite(Calor,Output);
  digitalWrite(Indicadores,0);
  pinMode(Vent_INT,OUTPUT);
  analogWrite(Vent_INT,0);
  pinMode(Vent_EXT,OUTPUT);
  analogWrite(Vent_EXT,255);
  
  EEPROM.begin(20);
  initOled();
  display.clearDisplay();
  display.display();
  initWiFi();
  display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("LLAMAR RTC");
    display.display();
    delay(500);
  initRTC();
  display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("LLAMAR AHT");
    display.display();
    delay(500);
  initAHT();
}

void loop() {  
  //ArduinoOTA.handle();
  //LEE EL BOTÓN. Única entrada
  //display.clearDisplay();
  int lectura = digitalRead(Start);
  leerRTC(); //Lee el RTC y verifica la fase
  inicial = int(EEPROM.read(0));
  display.clearDisplay();
  if(lectura==0 or esta_encendida == true)
    {
      contador = contador + 1;
      esta_encendida = true;
      oled_print();
      if(contador == 45){
          contador = 0;
          esta_encendida = false;
          display.clearDisplay();
          display.display();
        }
    }else{
      delay(900);  
    }
  switch(inicial){
    case 0:
      myPID.SetMode(MANUAL);
      analogWrite(Vent_INT,255);
      if(lectura == 0){
        EEPROM.write(0, 1);
        Iicializar(18,1,2,3,4,5,6);
        Iicializar(26,7,8,9,10,11,12);
        analogWrite(Vent_INT,0);
        myPID.SetMode(AUTOMATIC);
      }
      break;
   case 1:
       if(fase_1 = true){
              //setPoints(37.77,67.50);
              setPoints(37.35,67.50);
              fase_1 = false;
       }
        Control();    //ESTA LINEA CONTROLA SERVO, FOCO, HUMIFICADOR, VENTILADORES
        if(lectura == 0){
          
          rst+=1;
          if(rst>5){
            reiniciar();
          }
        }else{
           rst=0;  
        }
        break;
   case 2:
        if(fase_2 = true){
                //setPoints(36.00,70.00);
                setPoints(35.80,70.00);
              fase_2 = false;
       }
        Control();    //ESTA LINEA CONTROLA SERVO, FOCO, HUMIFICADOR, VENTILADORES
        if(lectura == 0){
          
          rst+=1;
          if(rst>5){
            reiniciar();
          }
        }else{
           rst=0;  
        }
        break;
   default:
       EEPROM.write(0,0);
       EEPROM.commit();
       break;
  } 
}
