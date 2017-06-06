/*
 * rosserial SRF08 Ultrasonic Ranger 
 * test ROS<-->Arduino nano
 *
 */
/* ******************************************************************* 
  Radioelf - junio 2017
 http://radioelf.blogspot.com.es/
 
  Copyright (c) 2017 Radioelf.  All rights reserved.
  Test  Open Source Arduino Due and SRF08.
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 
                      
//Configuramos publicador y nodos 
std_msgs::Float32MultiArray sonar_msg;
ros::Publisher pub_sonar("SRF08", &sonar_msg);
ros::NodeHandle nh;

// direccion SRF08
#define   ADDRESS (0xE2 >> 1)                                 // direccion I2C, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF4 , 0xF6, 0xF8, 0xFA, 0XFC y 0xFE
//comandos SRF08
#define   RX_pulgadas        0X50                             // Modo calculo distancia - Resultado en pulgadas
#define   RX_centimetros     0X51                             // Modo calculo distancia - Resultado en centimetros
#define   RX_milis           0X52                             // Modo calculo distancia - Resultado en microsegundos
#define   RX_ANN_pulgadas    0X53                             // Modo ANN - Resultado en pulgadas
#define   RX_ANN_centimetros 0X54                             // Modo ANN - Resultado en centímetros
#define   RX_ANN_milis       0X55                             // Modo ANN - Resultado en micro-segundos
#define   Sec1_cambio        0XA0                             // 1º en la secuencia para cambiar la dirección I2C
#define   Sec2_cambio        0XAA                             // 2º en la secuencia para cambiar la dirección I2C 
#define   Sec3_cambio        0XA5                             // 3º en la secuencia para cambiar la dirección I2C
// registros SRF08
#define  revision            0x00                             // Revision de Software  Registro de comando
#define  LDR                 0x01                             // Sensor de luz,  registro de ganancia max. (por defecto 31)
#define  ECO                 0x02                             // uint8_t alto de 1º eco, registro de alcance de distancia (por defecto 255)
// config SRF08
#define ganacia              0x1F                             // ganacia 1-31
#define rango                0x8C                             // rango de lectura 0X00->43mm, 0X8C->6mts (rango x 43 + 43)

const int led_pin = 13;
float sensorReading = 0.0;
float sensorLDR = 0; 
uint8_t version_SRF;
int reading = 0;
unsigned long publisher_timer;
//*************************************************************************************************
void setup(){
  Wire.begin();                                               // Config. I2C
  pinMode(led_pin, OUTPUT);
  uint8_t SRF08status = read_Soft();
  if (SRF08status !=0 && SRF08status !=255){
    Conf_Gain(ganacia);                                        // configuramos la ganancia SRF08
    delay(100);
    Conf_Range(rango);                                         // configuramos le rango de lectura SRF8
    nh.initNode();
    sonar_msg.data_length = 3;                                 // longitud del mensaje
    sonar_msg.data=(float*)malloc(sizeof(float)*3);            // reservamos espacio
    nh.advertise(pub_sonar);
  }else{
    while (true){
      digitalWrite(led_pin, HIGH-digitalRead(led_pin));
      delay(200);
      SRF08status = read_Soft();
      if (SRF08status !=0 && SRF08status !=255){
        delay(500);
        asm volatile (" jmp 0");                              // reset
      }
    }
  }
}

void loop(){
  if (millis() > publisher_timer) {
    read_SRF08(RX_centimetros, ECO);
    read_SRF08(RX_centimetros, LDR);
    sonar_msg.data[0] = sensorReading;                        // distancia    
    sonar_msg.data[1] = sensorLDR;                            // % LDR
    sonar_msg.data[2] = millis();                             // milisegundos run..
    pub_sonar.publish(&sonar_msg);
    publisher_timer = millis() + 100;                         // publicamos cada 100ms
  }
  nh.spinOnce();
}
//********************************************************************************************
// Config. ganancia
void Conf_Gain(uint8_t gain){
  Wire.beginTransmission(ADDRESS); 
  Wire.write(uint8_t(0x01));                         
  Wire.write(uint8_t(gain));                 
  Wire.endTransmission();   
}
// Config. rango
void Conf_Range(uint8_t range){
  Wire.beginTransmission(ADDRESS); 
  Wire.write(uint8_t(0x02));                         
  Wire.write(uint8_t(range));                 
  Wire.endTransmission();   
}
// Leemos la version del software
uint8_t read_Soft(){                                     
  Wire.beginTransmission(ADDRESS);             
  Wire.write(uint8_t(0x00));                                 
  Wire.endTransmission();
  
  Wire.requestFrom(ADDRESS,uint8_t (1));                      // peticion 1 byte
  while(Wire.available() < 0);                                         
  return(Wire.read());                               
}
// lectura sensor SRF08
void read_SRF08(uint8_t comando, uint8_t registro){                              
  Wire.beginTransmission(ADDRESS);
  Wire.write(uint8_t(0x00));                                   // Direccion interna 0x00 (registro de comandos)
  Wire.write(comando);                                         // Enviar comando a ejecutar
  Wire.endTransmission(); 
  delay(70);                                                   // >65ms                                                       
  
  Wire.beginTransmission(ADDRESS);             
  Wire.write(uint8_t(registro));                           
  Wire.endTransmission();

  if (registro ==2){
    Wire.requestFrom(ADDRESS,uint8_t (2));                    // peticion de 2 bytes 
    while(Wire.available() < 2);                     
    reading = Wire.read();  
    reading =  reading << 8;    
    reading |= Wire.read();                                   // lectura del ECO
    sensorReading =float (reading);                                
    }else{
        Wire.requestFrom(ADDRESS,uint8_t (1));                // peticion 1 byte
        while(Wire.available() < 0);   
        if (registro ==0) version_SRF = (Wire.read());        // lectura de la version de software                                
        else sensorLDR  =float (Wire.read()/2.48);            // lectura del sensor LDR en porcentaje (0-248)
    }
}

