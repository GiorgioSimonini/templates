/*----------------------------------------------------------------------------------------------------------------*
	Author: Giorgio Simonini
	Title:
    Data:
	Description:
	Functionalities:
    To do:
    Problems:

------------------------------------------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>


//-----------------//
//     DEFINES     //
//-----------------//

#define 	DEBUG			0

// DATA_TX (lenght and indexes)
#define 	TX_LEN			3			// exact number of byte to send
#define 	DATA_TX_1		0			// indexes...
#define 	DATA_TX_2		1
#define 	DATA_TX_3		2

// DATA_RX
#define 	DEATH_TIME		100			// (ms) time to return at default if no packet arrives
#define 	MAX_RX_LEN 		255			// max message lenght
#define 	RX_LEN			3			// exact number of byte received(must be exactly as sended)
#define 	DATA_RX_1		0			// indexes...
#define 	DATA_RX_2		1
#define 	DATA_RX_3		2

// TIMINGS (periods) [ms]
#define 	TASK_PER		10			// period of generic task
#define 	DEBUG_PER		200			// serial debug period
#define 	UDP_RX_PER		5			// udp receive period
#define 	UDP_TX_PER		10			// udp trasmitt period


//-------------------//
//     VARIABLES     //
//-------------------//

// NETWORK
const char* ssid = "netword_name";
const char* password = "password";

WiFiUDP Udp;							// UDP class
IPAddress remote_ip(x,x,x,x);
unsigned int local_port = xxxx;  		// local port to listen on
unsigned int remote_port = xxxx;

byte data_tx[TX_LEN];					// message to send
/*
	if data is float:
		float data[len];
		byte data_b[len*sizeof(float)];
	and use data_b to send and receive (or Udp.write((const uint8_t *)data, len*sizeof(float));)
*/
byte data_rx[RX_LEN];					// contain received message
byte incomingPacket[MAX_RX_LEN];  		// buffer for incoming packets

// PINS
byte ledPin = D0; 						// integrated led nodeMCU
byte analogInPin = A0;					// analog input from steer motor potentiometer

// time management (stores last execution time)
unsigned long lastUdpTime_rx;
unsigned long lastUdpTime_tx;
unsigned long lastPacketTime;
unsigned long lastDebugTime;
unsigned long lastFun1Time;


//-------------------//
//     FUNCTIONS     //
//-------------------//

// COMPARE: compare two bytes array from 0 to len
bool compare(char* str1, char* str2, int len){
	for (int i=0; i<len; i++){
		if (str1[i] != str2[i])
			return false;
	}
	return true;
}


//---------------//
//     SETUP     //
//---------------//
void setup() {

	Serial.begin(9600);
	delay(10);

	pinMode(ledPin, OUTPUT);			// set led pin to output
	
	digitalWrite(ledPin, HIGH);			// led active low

	// Connect to WiFi network
	Serial.println();
	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);					// start wifi connection

	while (WiFi.status() != WL_CONNECTED) {		// wait for connection
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected");

	if (!Udp.begin(local_port)){				// start udp
		// error management
	}

	// Print the IP address
	Serial.print("Use this IP to connect: ");
	Serial.print("http://");
	Serial.print(WiFi.localIP());
	Serial.println("/");

	Serial.printf("port UDP: %i ", local_port);
	Serial.println("");

	//Serial.printf("start python script");

	unsigned long temp = millis();				// get the current "time" (actually the number of milliseconds since the program started)
	lastUdpTime_rx = temp;  	
	lastUdpTime_tx = temp;
	lastDebugTime = temp;
	lastPacketTime = temp;
}


//--------------//
//     LOOP     //
//--------------//
void loop() {

	//----- UDP RX TASK -----//
	if (millis() - lastUdpTime_rx >= UDP_RX_PER){

		// DEFAULT
		if (millis() - lastPacketTime > DEATH_TIME){
			// set data_rx to default
		}

		// GET PACKET
		int packetSize = Udp.parsePacket();
		if (packetSize){
			// receive incoming UDP packets
			int len = Udp.read(incomingPacket, MAX_RX_LEN);
			if (len > 0){
				incomingPacket[len] = 0;		// useful in case of char* communication
				if (len == RX_LEN){
					lastPacketTime = millis();
					/* update data_rx, if floats:
					for (unsigned int i=0; i<RX_LEN*sizeof(float); i++){
						data_rx_b[i] = incomingPacket[i];
					}
					*/
				}
			}
			Udp.endPacket();
		}
		// TIMING
		lastUdpTime_rx = millis();
	}

	//----- UDP TX TASK -----//
	if (millis() - lastUdpTime_tx >= UDP_TX_PER){
		
		/* set data_tx
		data_tx[DATA_TX_1] = 1;
		data_tx[DATA_TX_2] = 2;
		data_tx[DATA_TX_3] = 3;
		*/
		if (Udp.beginPacket(remote_ip, remote_port)){				// open connection
			Udp.write(data_tx);										// send
			// Udp.write((const uint8_t *)data_tx, TX_LEN*sizeof(float)); 	// if float
			Udp.endPacket();
		} else{
			// error management
		}

		// TIMING
		lastUdpTime_tx = millis();
	}

	//----- FUNCTION 1 -----//
	if (millis() - lastFun1Time >= TASK_PER){

		// DO SOMETHING

		// TIMING
		lastFun1Time = millis();
	}

	//----- DEBUG -----//
	if (DEBUG){
		if (millis() - lastDebugTime >= DEBUG_PER){

			Serial.printf("debug information");

			lastDebugTime = millis();
		}
	}
}
