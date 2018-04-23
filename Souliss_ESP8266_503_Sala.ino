/**************************************************************************
    Souliss - 503 box Relè control

	It use static IP Addressing

    Load this code on ESP8266 board using the porting of the Arduino core
    for this platform.
        
***************************************************************************/
// Ultima cifra dell'indirizzo IP
#define IP_ADDRESS 137
#define HOSTNAME "sala-503"
#define SERIAL_DEBUG
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
#define T_RELE_1	0      
#define T_RELE_2	1
#define T_LED		2
     

// **** Define here the right pin for your ESP module **** 
#define	PIN_RELE_1		5
#define	PIN_RELE_2		4
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
        
		FAST_50ms() {   // We process the logic and relevant input and output every 50 milliseconds

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

			//Le 2 righe seguenti gestiscono gli interruttori facendoli diventare Bistabili/Deviatori con l'App
			Souliss_DigIn2State(PIN_BUTT_1, Souliss_T1n_ToggleCmd, Souliss_T1n_ToggleCmd, memory_map, T_RELE_1);
			Souliss_DigIn2State(PIN_BUTT_2, Souliss_T1n_ToggleCmd, Souliss_T1n_ToggleCmd, memory_map, T_RELE_2);
			//Le 2 righe seguenti gestiscono i pulsanti
			//Souliss_LowDigIn(PIN_BUTT_1, Souliss_T1n_ToggleCmd, memory_map, T_RELE_1);
			//Souliss_LowDigIn(PIN_BUTT_2, Souliss_T1n_ToggleCmd, memory_map, T_RELE_2);

			
		}
			FAST_70ms() {
			//Gestisco i Relè
			DigOut(PIN_RELE_1, Souliss_T1n_Coil, T_RELE_1);
			DigOut(PIN_RELE_2, Souliss_T1n_Coil, T_RELE_2);
			//Gestisco il Led
			DigOut(PIN_LED, Souliss_T1n_Coil, T_LED);
			//Check if joined and take control of the led
			//If not Joined the led blink, if Joined the led reflect the T11 Status
			if (joined == 1) {
				if (mOutput(T_LED) == 1) {
					digitalWrite(PIN_LED, HIGH);
				}
				else {
					digitalWrite(PIN_LED, LOW);
				}
			}
		}
		FAST_90ms() { 
			//Apply logic if statuses changed
			Logic_SimpleLight(T_RELE_1);
			Logic_SimpleLight(T_RELE_2);
			Logic_SimpleLight(T_LED);

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

