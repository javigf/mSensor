/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two different nodes,
 * connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
 * which responds by sending the value back.  The ping node can then see how long the whole cycle
 * took.
 */
 
 /* Internal Volt Meter https://code.google.com/p/tinkerit/wiki/SecretVoltmeter */
 /* RF24 Home Page http://maniacbug.github.io/RF24/index.html */
 

#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <SPI.h>
#include <Wire.h>
#include "binary_const.h" // with this we can use something B8(01010101) that it will convert to 85 at compile time
// the above cames really handy and readable when doing per bit configuration in the ADXL345 registers
//#include <NRF24.h>

#include <LowPower.h>

#define DEVICE (0x53)    //ADXL345 device address (with SDO tied to ground)
#define TO_READ (6)      //num of bytes we are going to read each time (two bytes for each axis)

#define INTERRUPTPIN 2   // Arduino pin which is connected to INT1 from the ADXL345

byte led = 8;

// Sensor Data 
char client [] = "Cliente X";

struct device {
                char location [20]  ;
                char name [20] 		  ;
                char function [20]  ;
                char status [50]    ;
                float lat           ;
                float lon           ;
};

device sensor  {
                 "Baradero"   ,
                 "Sensor 1"   ,
                 "Movimiento" ,
                 "DATA"       ,  
                  0           ,
                  0
								};


char data[33] = {"ABC*"} ;

// TIMING 
unsigned long interval 							= 8000;
unsigned long previousMillis 				= 0;																		// millis() returns an unsigned long.

// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(9,10); 		// MEGA RF24 radio(9,53);

// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
//const int role_pin = 7;

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

byte buff[TO_READ];    							//6 bytes buffer for saving data read from the device
char str[64];                      //string buffer to transform data before sending it to the serial port
boolean inspected = 0;

/*----------------------------- () START INTERRUPT SERVICE ROUTINE _ISR () ----------------------------*/

void wakeUp(){
    // Just a handler for the pin interrupt.
}

void setup(void){
  
	pinMode (led,OUTPUT);
	Serial.begin(57600);
  printf_begin();
	setupADXL345 ();
	rfSetup ();
	rfSendData ();
	
	blink ();

 
}

/* --------------- NO UTILIZAR SERIAL PRINT en LOOP ------------------- */

void loop(void){
	
	radio.stopListening();
	
	if (readADXL345){
	
	}
	
	//readAxisData ();
	rfSendData ();
	delay (1000);
	blink ();
	blink ();
  
}

byte readADXL345 (void){
		
	byte activity = 0;
		
	if(digitalRead(INTERRUPTPIN)) {
    activity = 1;
		
		int interruptSource = readByte(DEVICE, R_INT_SOURCE);
    //Serial.print("### ");
    //Serial.println(interruptSource, BIN);
    
    
    if(interruptSource & B8(100)) {
      //Serial.println("### FREE_FALL");
    }
    if(interruptSource & B8(1000)) {
      //Serial.println("### Inactivity");
      // we don't need to put the device in sleep because we set the AUTO_SLEEP bit to 1 in R_POWER_CTL
      // set the LOW_POWER bit to 1 in R_BW_RATE: with this we get worst measurements but we save power
      int bwRate = readByte(DEVICE, R_BW_RATE);
      writeTo(DEVICE, R_BW_RATE, bwRate | B8(10000));
			
			goToSleep ();
			
    }
    if(interruptSource & B8(10000)) {
				
			//Serial.println("### Activity");
      
      // get current power mode
      int powerCTL = readByte(DEVICE, R_POWER_CTL);
      // set the device back in measurement mode
      // as suggested on the datasheet, we put it in standby then in measurement mode
      // we do this using a bitwise and (&) so that we keep the current R_POWER_CTL configuration
      writeTo(DEVICE, R_POWER_CTL, powerCTL & B8(11110011));
      delay(1); // let's give it some time (not sure if this is needed)
      writeTo(DEVICE, R_POWER_CTL, powerCTL & B8(11111011));
      
      // set the LOW_POWER bit to 0 in R_BW_RATE: get back to full accuracy measurement (we will consume more power)
      int bwRate = readByte(DEVICE, R_BW_RATE);
      writeTo(DEVICE, R_BW_RATE, bwRate & B8(01111));
								
			rfSendData ();					
								
			delay (100);
			goToSleep ();

    }
    if(interruptSource & B8(100000)) {
      Serial.print("### DOUBLE_TAP Axes: ");
      printTapAxes();
      Serial.println(""); // closing Axes line
    }
    else if(interruptSource & B8(1000000)) { // when a double tap is detected also a signle tap is deteced. we use an else here so that we only print the double tap
      Serial.print("### SINGLE_TAP Axes: ");
      printTapAxes();
      Serial.println(""); // closing Axes line
    }
    delay(150);
  }
  readAxisData ();
		
  delay (100);
	//goToSleep ();
	
	return activity ;
}



/*----------------- FUNCION PARA ENVIAR LOS DATOS DEL STRUCT ------------------------- */

byte rfSendData (void){
  
	radio.stopListening ();
	delay (100);

	
	
	printf("Now sending data... -> "); 
	bool ok = radio.write( &data, sizeof(data));  
	delay (20);

	radio.startListening ();
	// Wait here until we get a response, or timeout (250ms)
	unsigned long started_waiting_at = millis();
	bool timeout = false;
	while ( ! radio.available() && ! timeout )
		if (millis() - started_waiting_at > 250 )
			timeout = true;

// Describe the results
  if ( timeout ){
    printf("Failed, response timed out.\n\r");
  }
  else{
// Grab the response, compare, and send to debugging spew
    unsigned long got_time;
    radio.read( &data, sizeof(data) );

// Spew it
    printf("Datos recibidos %d \r\n", data[0]);
  }
  printf("Sleeping ... zzzz \r\n");

  return 0;  
}

/*----------------- FUNCION PARA CONFIGURAR EL SENSOR DE MOV ------------------------- */

byte setupADXL345 (void){
	
  Wire.begin();        // join i2c bus (address optional for master)
	pinMode(INTERRUPTPIN, INPUT); 
    
  // interrupts setup
  writeTo(DEVICE, R_INT_MAP, 0); 							// send all interrupts to ADXL345's INT1 pin
  writeTo(DEVICE, R_INT_ENABLE, B8(1111100)); // enable signle and double tap, activity, inactivity and free fall detection
    
  // free fall configuration
  writeTo(DEVICE, R_TIME_FF, 0x14); 					// set free fall time
  writeTo(DEVICE, R_THRESH_FF, 0x0A);					// set free fall threshold
  
  // single tap configuration
  writeTo(DEVICE, R_DUR, 0x1F); 							// 625us/LSB
  writeTo(DEVICE, R_THRESH_TAP, 48); 					// 62.5mg/LSB  <==> 3000mg/62.5mg = 48 LSB as datasheet suggestion
  writeTo(DEVICE, R_TAP_AXES, B8(111)); 			// enable tap detection on x,y,z axes

  // double tap configuration
  writeTo(DEVICE, R_LATENT, 0x40);
  writeTo(DEVICE, R_WINDOW, 0xFF);
  
  // inactivity configuration
  writeTo(DEVICE, R_TIME_INACT, 8); 					// 1s / LSB
  writeTo(DEVICE, R_THRESH_INACT, 6); 				// 62.5mg / LSB
  // also working good with high movements: R_TIME_INACT=5, R_THRESH_INACT=16, R_ACT_INACT_CTL=B8(00000111)
  // but unusable for a quite slow movements
  
  // activity configuration
  writeTo(DEVICE, R_THRESH_ACT, 6); 					// 62.5mg / LSB
  
  // activity and inctivity control
  writeTo(DEVICE, R_ACT_INACT_CTL, B8(11111111)); // enable activity and inactivity detection on x,y,z using ac
  
  // set the ADXL345 in measurement and sleep Mode: this will save power while while we will still be able to detect activity
  // set the Link bit to 1 so that the activity and inactivity functions aren't concurrent but alternatively activated
  // set the AUTO_SLEEP bit to 1 so that the device automatically goes to sleep when it detects inactivity
  writeTo(DEVICE, R_POWER_CTL, B8(111100));

	return 0;
}

/*----------------- FUNCION PARA LEER LOS EJES DEL SENSOR MOV ------------------------- */

int readAxisData (void){
	int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  int x, y, z;
  
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3]) << 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  
  //we send the x y z values as a string to the serial port
  sprintf(str, "%d %d %d", x, y, z);  
  Serial.print(str);
  Serial.println("");
  
  //It appears that delay is needed in order not to clog the port
  delay(150);
	
	return 0;
}



void printTapAxes() {
  int tapStatus = readByte(DEVICE, R_ACT_TAP_STATUS);
  if(tapStatus & B8(100)) {
    Serial.print("x ");
  }
  if(tapStatus & B8(10)) {
    Serial.print("y ");
  }
  if(tapStatus & B8(1)) {
    Serial.print("z ");
  }
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

// read a single bite and returns the readed value
byte readByte(int device, byte address) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        		//sends address to read from
  Wire.endTransmission(); 				//end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, 1);    // request 1 byte from device
  
  int readed = 0;
  if(Wire.available())
  { 
    readed = Wire.read(); 				// receive a byte
  }
  Wire.endTransmission(); 				//end transmission
  return readed;
}


byte rfSetup (void){
	
  //
  // Setup and configure rf radio
  //

  radio.begin();
	
	// Setup Connection speed enum (RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS)
	
	radio.setDataRate(RF24_250KBPS);

	// optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

	radio.enableDynamicPayloads();
	
  // optionally, reduce the payload size.  seems to
  // improve reliability
  // radio.setPayloadSize(PayLoadSize);
	
	//radio.setAutoACK (true);

  //
  // Open pipes to other nodes for communication
  //

  // This simple sketch opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  radio.openWritingPipe(pipes[0]);		// --> 0xF0F0F0F0E1LL
  radio.openReadingPipe(1,pipes[1]); 	// --> 0xF0F0F0F0D2LL

	//radio.setChannel (0);
	
  //
  // Start listening
  //

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //
  radio.printDetails();
	return 0;
}

byte goToSleep (void){
	//byte a = 13;
	
   attachInterrupt(0, wakeUp, CHANGE);
  
  /*

  http://arduino.cc/en/pmwiki.php?n=Reference/AttachInterrupt

  Board         int.0 int.1 int.2 int.3 int.4 int.5
  Uno, Ethernet 2     3        
  Mega2560      2     3     21    20    19    18
  Leonardo      3     2     0     1     7  
  Due (see below)

  */

	delay (5);
	radio.powerDown ();
	
	//for (byte i=0; i<a; i++)
	//LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

	delay (100);
	radio.powerUp ();
	delay (100);

  detachInterrupt(0); 
	
	return 0;
}

byte readVcc(void) {
  long result;
  float resultado;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  
	resultado=(float)result/1000;
	sprintf(sensor.status, "%f", resultado);

	Serial.print ("VCC: ");
	Serial.print (sensor.status);
	Serial.println (" V");
	
	return 0;
}


byte blink (void){
	
	digitalWrite (led,HIGH);
	delay (250);
	digitalWrite (led,LOW);
	delay (250);
	
	return 0;
}
