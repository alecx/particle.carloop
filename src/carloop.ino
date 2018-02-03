/*
 * Copyright 2016 Emerson Garland
 * Free to modify, share and do whatever, just give me credit if you like it!
 * This code will publish chosen obdii compliant messages to blynk for data visualization.
 */

#include "carloop.h"
#include "base85.h"
#define DEBUG_ON

#define disconnectDelay 15000 // After fifteen seconds of no WiFi, disconnect
#define checkInterval 60000 // How often to check for WiFi once disconnected

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

void sendObdRequest();
void waitForObdResponse();
void delayUntilNextRequest();
void publishValuesAtInterval();
void sendToInflux();


String dumpMessage(const CANMessage &message);

Carloop<CarloopRevision2> carloop;

int canMessageCount = 0;
long canMessageCountSent = 0;
long canMessageCountRcvd = 0;
long canMessageCountRcvd1 = 0;
long canMessageCountRcvd2 = 0;
int delayInterval = 8;
IPAddress INFLUXDB_HOST(100, 100, 100, 159);
unsigned int INFLUXDB_PORT = 8089;


float ENGINE_LOAD;
float ENGINE_RPM;
int VEHICLE_SPEED;
float THROTTLE;
float OBD_STANDARDS;
int ENGINE_RUN_TIME;
float SUPPORTED_PIDS_21_40;
float FUEL_TANK_LEVEL_INPUT;
int DISTANCE_TRAVELED_SINCE_CODES_CLEARED;
float SUPPORTED_PIDS_41_60;
int AMBIENT_AIR_TEMPERATURE;
float MAX_VALUES;
float MAX_VALUES_2;
float SUPPORTED_PIDS_61_80;

///////////////////////////////////////////////////////////////////////////
//GLOBAL INTEGERS FOR USE IN PERFORMING MATH AND EXTRACTION OF OBDII DATA//
///////////////////////////////////////////////////////////////////////////
int data0;
int data1;
int data2;
int data3;
int data4;
int data5;
int data6;
int data7;


// OBD CAN MESSAGE IDs
const auto OBD_CAN_BROADCAST_ID    = 0X7DF;
const auto OBD_CAN_REQUEST_ID      = 0x7E0;
const auto OBD_CAN_REPLY_ID_MIN    = 0x7E8;
const auto OBD_CAN_REPLY_ID_MAX    = 0x7EF;

// OBD MODES
const auto OBD_MODE_CURRENT_DATA = 0x01;

//const auto OBD_MODE_REQUEST_VEHICLE_DATA = 0x09;

const auto OBD_PID_SUPPORTED_PIDS_01_20                  = 0x00;
const auto OBD_PID_ENGINE_LOAD                           = 0x04;
const auto OBD_PID_ENGINE_RPM                            = 0x0c;
const auto OBD_PID_VEHICLE_SPEED                         = 0x0d;
const auto OBD_PID_THROTTLE    	                         = 0x11;
const auto OBD_PID_OBD_STANDARDS                         = 0x1c;
const auto OBD_PID_ENGINE_RUN_TIME                       = 0x1f;
const auto OBD_PID_SUPPORTED_PIDS_21_40                  = 0x20;
const auto OBD_PID_FUEL_TANK_LEVEL_INPUT                 = 0x2f;
const auto OBD_PID_DISTANCE_TRAVELED_SINCE_CODES_CLEARED = 0x31;
const auto OBD_PID_SUPPORTED_PIDS_41_60                  = 0x40;
const auto OBD_PID_AMBIENT_AIR_TEMPERATURE               = 0x46;
const auto OBD_PID_SUPPORTED_PIDS_61_80                  = 0x60;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SUM THE TOTAL AMOUNT OF PIDS YOU WOULD LIKE TO REQUEST AND PLACE THAT IN const size_t NUM_PIDS_TO_REQUEST//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

const size_t NUM_PIDS_TO_REQUEST = 9;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMMENT OUT OR REMOVE THE PIDS THAT YOU DO NOT HAVE TO INCREASE EFFECIENCY BUT BE SURE TO UPDATE THE ABOVE CONSTANT//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t pidsToRequest[NUM_PIDS_TO_REQUEST] = {
// OBD_PID_SUPPORTED_PIDS_01_20,
OBD_PID_ENGINE_LOAD,
OBD_PID_ENGINE_RPM,
OBD_PID_VEHICLE_SPEED,
OBD_PID_THROTTLE,
OBD_PID_OBD_STANDARDS,
OBD_PID_ENGINE_RUN_TIME,
// OBD_PID_SUPPORTED_PIDS_21_40,
OBD_PID_FUEL_TANK_LEVEL_INPUT,
OBD_PID_DISTANCE_TRAVELED_SINCE_CODES_CLEARED,
// OBD_PID_SUPPORTED_PIDS_41_60,
OBD_PID_AMBIENT_AIR_TEMPERATURE,
// OBD_PID_SUPPORTED_PIDS_61_80,
};


uint8_t pidIndex = NUM_PIDS_TO_REQUEST - 1;

String dumpForPublish;

auto *obdLoopFunction = sendObdRequest;
unsigned long transitionTime = 0;
uint8_t lastMessageData[8];
static const unsigned long interval2 = 60000;
static unsigned long lastDisplay2 = 0;
static const unsigned long interval = 1000;
static unsigned long lastTime = 0;
static unsigned long last_pub = 0;


const int NTP_PACKET_SIZE= 1024;
char packetBuffer[NTP_PACKET_SIZE];
UDP Udp;


void setup() {
  Serial.begin(115200);
  carloop.begin();
  Particle.connect();
  Particle.publish("status", "setup", 60, PRIVATE);
  transitionTime = millis();
  // Udp.begin(9000);
}

void sendToInflux() {
  if (!WiFi.ready()) {
    return;
  }
  Serial.println("Transmitting udp...");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  int len = snprintf(packetBuffer, NTP_PACKET_SIZE, "car,vehicle=golf km_tot=%d,fuel=%f,bat=%f,temp=%d,rtm=%d\n", DISTANCE_TRAVELED_SINCE_CODES_CLEARED, FUEL_TANK_LEVEL_INPUT, carloop.battery(), AMBIENT_AIR_TEMPERATURE, ENGINE_RUN_TIME);

  if (len > 0 && len < NTP_PACKET_SIZE) {
    // Udp.beginPacket(IPAddress(100, 100, 100, 159), 8089);
    // Udp.write((unsigned char*)packetBuffer, len);
    // int st = Udp.endPacket();
    Udp.begin(9000);
    int st = Udp.sendPacket(packetBuffer, len, IPAddress(100, 100, 100, 159), 8089);
    Udp.stop();
    Serial.println(WiFi.localIP());
    Serial.println(String::format("sent udp %d out of %d", st, len));
  } else {
    Serial.println("failed udp");
  }
}

void sendToInfluxInt(const char *key, int val) {
  if (!WiFi.ready()) {
    return;
  }
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  int len = snprintf(packetBuffer, NTP_PACKET_SIZE, "car,vehicle=golf2016 %s=%d\n", key, val);

  if (len > 0 && len < NTP_PACKET_SIZE) {
    Udp.begin(9000);
    int st = Udp.sendPacket(packetBuffer, len, IPAddress(100, 100, 100, 159), 8089);
    Udp.stop();
  } else {
    Serial.println("failed udp");
    Particle.publish("error", "invalid udp len", 60, PRIVATE);
  }
}

void sendToInfluxFloat(const char *key, float val) {
  if (!WiFi.ready()) {
    return;
  }
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  int len = snprintf(packetBuffer, NTP_PACKET_SIZE, "car,vehicle=golf2016 %s=%f\n", key, val);

  if (len > 0 && len < NTP_PACKET_SIZE) {
    Udp.begin(9000);
    int st = Udp.sendPacket(packetBuffer, len, IPAddress(100, 100, 100, 159), 8089);
    Udp.stop();
  } else {
    Serial.println("failed udp");
    Particle.publish("error", "invalid udp len", 60, PRIVATE);
  }
}

unsigned int lastAttempt, lastDisconnect;
bool connected = true;

void checkConnection() {
    if (Particle.connected() == false && millis() - lastDisconnect >= disconnectDelay){
      lastDisconnect = millis();
      WiFi.off();
      connected = false;
    }

    if (millis() - lastAttempt >= checkInterval) {
      WiFi.on();
      Particle.connect();
      lastAttempt = millis();
    }

    if (millis() - lastAttempt >= disconnectDelay) {
      connected = Particle.connected();
    }
}



void loop() {
  // checkConnection();
  carloop.update();
  obdLoopFunction();
  waitForObdResponse();
  math();
  publishValuesAtInterval();
}

bool delay() {
  if(millis() - lastTime < interval) {
    return true;
  }
  lastTime = millis();
  return false;
}

/*************** Begin: OBD Loop Functions ****************/

/* For help understanding the OBD Query format over CAN bus,
 * see: https://en.wikipedia.org/wiki/OBD-II_PIDs#Query
 *
 * For help understanding why the first data byte is 0x02,
 * see: http://hackaday.com/2013/10/29/can-hacking-protocols/
 *
 * For help understanding modes and PIDs,
 * see: https://en.wikipedia.org/wiki/OBD-II_PIDs#Modes
 * and: https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs
 */
void sendObdRequest() {
  if (delay()) {
    return;
  }
	pidIndex = (pidIndex + 1) % NUM_PIDS_TO_REQUEST;

	CANMessage message;
	message.id = OBD_CAN_BROADCAST_ID;
	message.len = 8; // just always use 8
	message.data[0] = 0x02; // 0 = single-frame format, 2  = num data bytes
	message.data[1] = OBD_MODE_CURRENT_DATA; // OBD MODE
	message.data[2] = pidsToRequest[pidIndex]; // OBD PID

	carloop.can().transmit(message);
  canMessageCountSent++;
	obdLoopFunction = waitForObdResponse;
	transitionTime = millis();
}

void waitForObdResponse() {
	if (millis() - transitionTime >= 10) {
		obdLoopFunction = delayUntilNextRequest;
		transitionTime = millis();
		return;
	}
	CANMessage message;
	while (carloop.can().receive(message)) {
		canMessageCountRcvd++;
		if (message.id >= OBD_CAN_REPLY_ID_MIN && message.id <= OBD_CAN_REPLY_ID_MAX) {
      canMessageCountRcvd2++;
      data0 = message.data[0];
      data1 = message.data[1];
      data2 = message.data[2];
      data3 = message.data[3];
      data4 = message.data[4];
      data5 = message.data[5];
      data6 = message.data[6];
      data7 = message.data[7];
      process_message(message);
      return;
		}
    if (message.id == 401604624) {
      return;
    }
    Particle.publish("unknown_message", dumpMessage(message), 60, PRIVATE);
	}
}


void delayUntilNextRequest() {
	if (millis() - transitionTime >= delayInterval) {
		obdLoopFunction = sendObdRequest;
		transitionTime = millis();
	}
}

/*************** End: OBD Loop Functions ****************/


void publishValuesAtInterval() {
    if(millis() - lastDisplay2 < interval2) {
        return;
    }
    lastDisplay2 = millis();
    sendToInflux();
    publishValues();
}

void publishValues() {
    if (!WiFi.ready()) {
      return;
    }
    String str = String::format("KM: %d", DISTANCE_TRAVELED_SINCE_CODES_CLEARED);
    str += String::format(" Fuel: %.2f", FUEL_TANK_LEVEL_INPUT);
    // str += String::format(" Load: %.2f", ENGINE_LOAD);
    str += String::format(" Temp: %d", AMBIENT_AIR_TEMPERATURE);
    str += String::format(" RPM: %.2f", ENGINE_RPM);
    // str += String::format(" KMH: %d", VEHICLE_SPEED);
    // str += String::format(" THR: %.2f", THROTTLE);
    str += String::format(" RTM: %d", ENGINE_RUN_TIME);
    str += String::format(" BAT: %.2f", carloop.battery());
    // str += String::format(" SNT: %d", canMessageCountSent);
    // str += String::format(" RCV: %d", canMessageCountRcvd);
    Particle.publish("state", str, 60, PRIVATE);
    // Particle.variable("SNT", canMessageCountSent);
    // Particle.variable("RCV", canMessageCountRcvd);
    // Particle.variable("RCV1", canMessageCountRcvd1);
    // Particle.variable("RCV2", canMessageCountRcvd2);
    // if (DISTANCE_TRAVELED_SINCE_CODES_CLEARED > 0) {
    //   Particle.variable("km", DISTANCE_TRAVELED_SINCE_CODES_CLEARED);
    //   Particle.variable("fuel", (double)FUEL_TANK_LEVEL_INPUT);
    // }
}

//////////////////////////////////////
//ALMOST EVERY MODE 1 OBDII PID MATH//
//////////////////////////////////////

void math() {

//void mathENGINE_LOAD() {

    if (data2 == 4) {
        float load;
        load = (data3)/2.55;
        ENGINE_LOAD = load;
        return;
    }
//}


//void mathENGINE_RPM() {
    if (data2 == 12) {
        float RPM1;
        float RPM2;
        RPM1 = data3;
        RPM2 = data4;
        ENGINE_RPM = ((RPM1*256)+RPM2)/4;
        return;
    }
//}

//void mathVEHICLE_SPEED() {

    if (data2 == 13) {
        VEHICLE_SPEED = data3;
        return;
    }
//}

//void mathTHROTTLE() {

    if (data2 == 17) {
        float throttle;
        throttle = (data3)/2.55;
        THROTTLE = throttle;
        return;
    }
//}


//void mathENGINE_RUN_TIME() {

    if (data2 == 31) {
        ENGINE_RUN_TIME = (data3*256+data4);
        // char *key = "rtm";
        // sendToInfluxInt(key, ENGINE_RUN_TIME);
        return;
    }
//}


//void mathOBD_PID_FUEL_TANK_LEVEL_INPUT() {

    if (data2 == 47) {
        float fuelTankLevelInput;
        fuelTankLevelInput = data3/2.55;
        FUEL_TANK_LEVEL_INPUT = fuelTankLevelInput;
        // sendToInfluxFloat("fuel", FUEL_TANK_LEVEL_INPUT);
        return;
    }
//}


//void mathOBD_PID_DISTANCE_TRAVELED_SINCE_CODES_CLEARED() {

    if (data2 == 49) {
        DISTANCE_TRAVELED_SINCE_CODES_CLEARED = 256*data3 + data4;
        // sendToInfluxInt("km_tot", DISTANCE_TRAVELED_SINCE_CODES_CLEARED);
        return;
    }

    if (data2 == 70) {
        int ambientAirTemperature = data3 - 40;
        AMBIENT_AIR_TEMPERATURE = ambientAirTemperature;
        // sendToInfluxInt("temp", AMBIENT_AIR_TEMPERATURE);
        return;
    }
}

void process_message(const CANMessage &message) {
    if (message.data[2] == 31) {
        int val = (message.data[3]*256+message.data[4]);
        char *key = "rtm";
        sendToInfluxInt(key, val);
        return;
    }

    if (message.data[2] == 47) {
        float fuelTankLevelInput;
        fuelTankLevelInput = message.data[3]/2.55;
        char *key = "fuel";
        sendToInfluxFloat(key, fuelTankLevelInput);
        return;
    }

    if (message.data[2] == 49) {
        int val = 256*message.data[3] + message.data[4];
        char *key = "km_tot";
        sendToInfluxInt(key, val);
        return;
    }

    if (message.data[2] == 70) {
        int ambientAirTemperature = message.data[3] - 40;
        char *key = "temp";
        sendToInfluxInt(key, ambientAirTemperature);
        return;
    }
}


String dumpMessage(const CANMessage &message) {
	String str = String::format("id %x len %d - ", message.id, message.len);
	for (int i = 0; i < message.len; i++) {
		str += String::format("%d:%02x|", i, message.data[i]);
	}
	return str;
}
