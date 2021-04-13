// template for UDP comunication

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>


//-----------------//
//     DEFINES     //
//-----------------//

#define 	DEBUG			0

// DATA_TX (lenght and indexes)
#define 	TX_LEN			3
#define 	DATA_TX_1		0
#define 	DATA_TX_2		1
#define 	DATA_TX_3		2

// DATA_RX
#define 	DEATH_TIME		0.1			// time to return at default if no packet arrives
#define 	MAX_RX_LEN 		255			// max message lenght
#define 	RX_LEN			3			// number of byte received(must be exactly as sended)
#define 	DATA_RX_1		0			// indexes of element in messages
#define 	DATA_RX_2		1
#define 	DATA_RX_3		2

// TIMINGS (periods) [ms]
#define 	FUN1_PER		10
#define 	DEBUG_PER		200
#define 	UDP_RX_PER		10
#define 	UDP_TX_PER		10


//-------------------//
//     VARIABLES     //
//-------------------//

// NETWORK
const char* ssid = "netword_name";
const char* password = "password";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  		// local port to listen on
//unsigned int hostUdpPort = 4209;

char data_tx[TX_LEN];
char data_rx[RX_LEN];
char incomingPacket[MAX_RX_LEN];  		// buffer for incoming packets

// PINS
char ledPin = D0; 						// integrated led nodeMCU
char analogInPin = A0;					// analog input from steer motor potentiometer

// time management
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

	pinMode(ledPin, OUTPUT);
	
	digitalWrite(ledPin, HIGH);			// led active low

	// Connect to WiFi network
	Serial.println();
	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected");

	Udp.begin(localUdpPort);

	// Print the IP address
	Serial.print("Use this IP to connect: ");
	Serial.print("http://");
	Serial.print(WiFi.localIP());
	Serial.println("/");

	Serial.printf("port UDP: %i ", localUdpPort);
	Serial.println("");

	//Serial.printf("start python script");

	// setup PWM
	//pwm2.setup_pin_hz(steer_pin1, PWM_FREQ, PWM_RES, 0);

	unsigned long temp = millis();
	lastUdpTime_rx = temp;  	//get the current "time" (actually the number of milliseconds since the program started)
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
					data_rx[DATA_RX_1] = incomingPacket[DATA_RX_1];
					data_rx[DATA_RX_2] = incomingPacket[DATA_RX_2];
					data_rx[DATA_RX_3] = incomingPacket[DATA_RX_3];
				}
			}
			Udp.endPacket();
		}
		// TIMING
		lastUdpTime_rx = millis();
	}

	//----- UDP TX TASK -----//
	if (millis() - lastUdpTime_rx >= UDP_RX_PER){
		
		// set data_tx
		data_tx[DATA_TX_1] = 1;
		data_tx[DATA_TX_2] = 2;
		data_tx[DATA_TX_3] = 3;
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
		Udp.write(data_tx);

		// TIMING
		lastUdpTime_tx = millis();
	}

	//----- FUNCTION 1 -----//
	if (millis() - lastFun1Time >= FUN1_PER){

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
