/**************************************************************************
    Souliss - 503 box Relè control

	It use static IP Addressing

    Load this code on ESP8266 board using the porting of the Arduino core
    for this platform.
        
***************************************************************************/
// Ultima cifra dell'indirizzo IP
#define IP_ADDRESS 137
#define HOSTNAME "sala-503"
//#define SERIAL_DEBUG
#define	VNET_RESETTIME_INSKETCH
#define VNET_RESETTIME			0x00042F7	// ((20 Min*60)*1000)/70ms = 17143 => 42F7
//#define VNET_RESETTIME			0x0000359	// 857 -> x359 1 minuto
#define VNET_HARDRESET			ESP.reset()

// Configure the framework
#include "bconf/MCU_ESP8266.h"              // Load the code directly on the ESP8266

// **** Define the WiFi name and password ****
#include "D:\__User\Administrator\Documents\Privati\ArduinoWiFiInclude\wifi.h"
//To avoide to share my wifi credentials on git, I included them in external file
//To setup your credentials remove my include, un-comment below 3 lines and fill with
//Yours wifi credentials
//#define WIFICONF_INSKETCH
//#define WiFi_SSID               "wifi_name"
//#define WiFi_Password           "wifi_password"    

// Include framework code and libraries
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "Souliss.h"

// Define the network configuration according to your router settings
uint8_t ip_address[4]  = {192, 168, 1, IP_ADDRESS};
uint8_t subnet_mask[4] = {255, 255, 255, 0};
uint8_t ip_gateway[4]  = {192, 168, 1, 1};


// This identify the number of the Slot
#define T_RELE_1		0      
#define T_RELE_2		1
#define T_LED			2
#define T_WIFI_STRDB	3	//It takes 2 slots
#define T_WIFI_STR		5	//It takes 2 slots

//Deadband for analog values
#define NODEADBAND		0				//Se la variazione è superio del 0,1% aggiorno

// **** Define here the right pin for your ESP module **** 
#define	PIN_RELE_1		4
#define	PIN_RELE_2		5
#define PIN_BUTT_1		14
#define PIN_BUTT_2		12
#define PIN_BUTT_3		13
#define PIN_BUTT_LED	0
#define PIN_LED			16

//Useful Variable
byte led_status = 0;
byte joined = 0;
U8 value_hold = 0x068;


void setup()
{  
	#ifdef SERIAL_DEBUG
		Serial.begin(115200);
		Serial.println("Node Starting");
	#endif


	//Delay the startup. In case of power outage, this give time to router to start WiFi
	#ifndef SERIAL_DEBUG
		//Inserire qua l'ultima cifra dell'indirizzo IP per avere un delay all'avvio diverso per ogni nodo
		delay((IP_ADDRESS - 128) * 5000);
		//delay(15000);
	#endif
	Initialize();

	#ifdef SERIAL_DEBUG
		Serial.println("Node Inizializing");
	#endif

	//Pin Setup
	pinMode(PIN_RELE_1, OUTPUT);
	pinMode(PIN_RELE_2, OUTPUT);
	pinMode(PIN_BUTT_1, INPUT);
	pinMode(PIN_BUTT_2, INPUT);
	pinMode(PIN_BUTT_3, INPUT);
	pinMode(PIN_BUTT_LED, INPUT);
	pinMode(PIN_LED, OUTPUT);

	// Connect to the WiFi network with static IP
	Souliss_SetIPAddress(ip_address, subnet_mask, ip_gateway);

	Set_SimpleLight(T_RELE_1);			// T11 Relè 1
	Set_SimpleLight(T_RELE_2);			// T11 Relè 2
	Set_SimpleLight(T_LED);				// T11 Gestione Led
	Souliss_SetT51(memory_map, T_WIFI_STRDB);	//Imposto il tipico per contenere il segnale del Wifi in decibel
	Souliss_SetT51(memory_map, T_WIFI_STR);	//Imposto il tipico per contenere il segnale del Wifi in barre da 1 a 5

	// queste 4 righe servono per annullare l'effetto del Toggle al primo giro del loop
	// in modo da non avere le uscite alte all'avvio. Solo per i DigIn2State
	Souliss_DigIn2State(PIN_BUTT_1, Souliss_T1n_ToggleCmd, Souliss_T1n_ToggleCmd, memory_map, T_RELE_1);
	Souliss_DigIn2State(PIN_BUTT_2, Souliss_T1n_ToggleCmd, Souliss_T1n_ToggleCmd, memory_map, T_RELE_2);
	mInput(T_RELE_1) = 0;
	mInput(T_RELE_2) = 0;

	//Enable OTA
	ArduinoOTA.setHostname(HOSTNAME);
	ArduinoOTA.begin();

	#ifdef SERIAL_DEBUG
		Serial.print("MAC: ");
		Serial.println(WiFi.macAddress());
		Serial.print("IP:  ");
		Serial.println(WiFi.localIP());
		Serial.print("Subnet: ");
		Serial.println(WiFi.subnetMask());
		Serial.print("Gateway: ");
		Serial.println("Node Initialized");
	#endif



}

void loop()
{ 
    // Here we start to play
    EXECUTEFAST() {                     
        UPDATEFAST();   
		FAST_70ms() {   // We process the logic and relevant input and output every 50 milliseconds

			// Gestisce il pulsante di Reset e il Led
			// se si preme il punsante di fa il toggle del led, se si preme a lungo si resetta la scheda
			U8 invalue = LowDigInHold(PIN_BUTT_LED, Souliss_T1n_ToggleCmd, value_hold, T_LED);
			if (invalue == Souliss_T1n_ToggleCmd) {
				#ifdef SERIAL_DEBUG
					Serial.println("TOGGLE");
				#endif
				mInput(T_LED) = Souliss_T1n_ToggleCmd;
			} else if (invalue == value_hold) {
				// reset
				#ifdef SERIAL_DEBUG
					Serial.println("REBOOT");
				#endif
				delay(1000);
				ESP.reset();
			}
		
		}
		FAST_210ms() {
			//Processa le logiche per il segnale WiFi
			Souliss_Logic_T51(memory_map, T_WIFI_STRDB, NODEADBAND, &data_changed);
			Souliss_Logic_T51(memory_map, T_WIFI_STR, NODEADBAND, &data_changed);
		}
		FAST_110ms() { 
			//Verifico che l'interruttore non cambi posizione in caso invio il toggle
			
			//Se si usano pulsanti utilizzare le seguenti 2 righe commentate
			//Souliss_LowDigIn(PIN_BUTT_1, Souliss_T1n_ToggleCmd, memory_map, T_RELE_1);
			//Souliss_LowDigIn(PIN_BUTT_2, Souliss_T1n_ToggleCmd, memory_map, T_RELE_2);
			
			Souliss_DigIn2State(PIN_BUTT_1, Souliss_T1n_ToggleCmd, Souliss_T1n_ToggleCmd, memory_map, T_RELE_1);
			Souliss_DigIn2State(PIN_BUTT_2, Souliss_T1n_ToggleCmd, Souliss_T1n_ToggleCmd, memory_map, T_RELE_2);

			//Apply logic if statuses changed
			Logic_SimpleLight(T_RELE_1);
			Logic_SimpleLight(T_RELE_2);

			//Gestisco i Relè
			DigOut(PIN_RELE_1, Souliss_T1n_Coil, T_RELE_1);
			DigOut(PIN_RELE_2, Souliss_T1n_Coil, T_RELE_2);
			DigOut(PIN_LED, Souliss_T1n_Coil, T_LED);
	
		}
		FAST_510ms() {
			//Check if joined to gateway
			check_if_joined();

		}
		FAST_2110ms() {
			#ifdef SERIAL_DEBUG
				Serial.println("Heartbeat");
			#endif
		}
        FAST_PeerComms();                                        
		ArduinoOTA.handle();
	}
	EXECUTESLOW() {
		UPDATESLOW();
		SLOW_10s() {
			//Check wifi signal
			check_wifi_signal();
		}
	}
}

void check_wifi_signal() {
	long rssi = WiFi.RSSI();
	int bars = 0;

	if (rssi > -55) {
		bars = 5;
	}
	else if (rssi < -55 & rssi > -65) {
		bars = 4;
	}
	else if (rssi < -65 & rssi > -70) {
		bars = 3;
	}
	else if (rssi < -70 & rssi > -78) {
		bars = 2;
	}
	else if (rssi < -78 & rssi > -82) {
		bars = 1;
	}
	else {
		bars = 0;
	}
	float f_rssi = (float)rssi;
	float f_bars = (float)bars;
	Souliss_ImportAnalog(memory_map, T_WIFI_STRDB, &f_rssi);
	Souliss_ImportAnalog(memory_map, T_WIFI_STR, &f_bars);
	#ifdef SERIAL_DEBUG
		Serial.print("wifi rssi:");
		Serial.println(rssi);
		Serial.print("wifi bars:");
		Serial.println(bars);
	#endif
}

//This routine check for peer is joined to Souliss Network
//If not blink the led every 500ms, else led is a mirror of relè status
void check_if_joined() {
	if (JoinInProgress() && joined == 0) {
		joined = 0;
		if (led_status == 0) {
			digitalWrite(PIN_LED, HIGH);
			led_status = 1;
		}
		else {
			digitalWrite(PIN_LED, LOW);
			led_status = 0;
		}
	}
	else {
		joined = 1;
	}
}

