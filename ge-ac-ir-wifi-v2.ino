/*
   IRrecord: record and play back IR signals as a minimal
   An IR detector/demodulator must be connected to the input RECV_PIN.
   An IR LED must be connected to the output PWM pin 3.
   A button must be connected between the input SEND_BUTTON_PIN and ground.
   A visible LED can be connected to STATUS_PIN to provide status.

   The logic is:
   If the button is pressed, send the IR code.
   If an IR code is received, record it.

   Version 0.11 September, 2009
   Copyright 2009 Ken Shirriff
   http://arcfn.com
*/

#include <IRremote.h>
#include <WiFiNINA.h>
#include <SPI.h>

// For AirLift Breakout/Wing/Shield: Configure the following to match the ESP32 Pins!
#if !defined(SPIWIFI_SS)
#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    10   // Chip select pin
#define ESP32_RESETN  5    // Reset pin
#define SPIWIFI_ACK   7    // a.k.a BUSY or READY pin
#define ESP32_GPIO0   6
#endif
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


#if defined(ESP32)
int IR_RECEIVE_PIN = 15;
int SEND_BUTTON_PIN = 16; // RX2 pin
#else
int IR_RECEIVE_PIN = 11;
int SEND_BUTTON_PIN = 12;
#endif
int STATUS_PIN = LED_BUILTIN;

static const unsigned char freq = 38U;
static const unsigned int irPower[] PROGMEM = { 8350, 4300, 450, 1700, 450, 1650, 450, 1650, 450, 650, 450, 1650, 450, 650, 400, 1700, 400, 1700, 450, 4300, 400, 1700, 450, 1650, 450, 650, 400, 1700, 450, 650, 400, 650, 400, 650, 450, 650, 500 };
static const unsigned int irCool[] PROGMEM = { 8350, 4350, 450, 1650, 450, 1650, 450, 1700, 450, 600, 450, 1650, 450, 650, 400, 1700, 450, 1650, 450, 4300, 450, 650, 400, 700, 350, 700, 400, 650, 400, 700, 350, 700, 400, 650, 400, 700, 450 };
static const unsigned int irFan[] PROGMEM = { 8400, 4300, 400, 1700, 450, 1650, 450, 1700, 400, 700, 350, 1700, 450, 650, 450, 1650, 400, 1700, 450, 4300, 450, 1650, 450, 650, 400, 650, 400, 700, 400, 650, 450, 600, 450, 650, 400, 650, 500 };
static const unsigned int irTempInc[] PROGMEM = { 8400, 4300, 450, 1700, 400, 1700, 450, 1700, 400, 700, 350, 1700, 450, 650, 400, 1700, 400, 1700, 450, 4300, 400, 700, 400, 650, 400, 1700, 450, 650, 400, 650, 400, 650, 450, 650, 400, 650, 500 };
static const unsigned int irTempDec[] PROGMEM = { 8400, 4350, 400, 1700, 450, 1650, 450, 1700, 450, 650, 400, 1650, 450, 650, 450, 1650, 450, 1650, 450, 4300, 450, 1650, 450, 650, 400, 1700, 450, 650, 400, 700, 350, 700, 400, 650, 400, 700, 450 };
// unsigned const int irHigh[] = { 8350, 4300, 450, 1700, 450, 1650, 450, 1650, 450, 650, 450, 1650, 450, 650, 400, 1700, 400, 1700, 450, 4300, 400, 1700, 450, 1650, 450, 650, 400, 1700, 450, 650, 400, 650, 400, 650, 450, 650, 500 };
// unsigned const int irMed[] = { 8350, 4300, 450, 1700, 450, 1650, 450, 1650, 450, 650, 450, 1650, 450, 650, 400, 1700, 400, 1700, 450, 4300, 400, 1700, 450, 1650, 450, 650, 400, 1700, 450, 650, 400, 650, 400, 650, 450, 650, 500 };
// unsigned const int irLow[] = { 8350, 4300, 450, 1700, 450, 1650, 450, 1650, 450, 650, 450, 1650, 450, 650, 400, 1700, 400, 1700, 450, 4300, 400, 1700, 450, 1650, 450, 650, 400, 1700, 450, 650, 400, 650, 400, 650, 450, 650, 500 };

#define WLAN_SSID "Ministry of Sound"
#define WLAN_PASS "anjunadeep"
int status = WL_IDLE_STATUS;

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883                   // use 8883 for SSL, 1883 for non-SSL
#define AIO_USERNAME    "roadmapper"
#define AIO_KEY         "aio_gXII16qiqTi0XJybosAMW1kZx42Q"
//
//// io.adafruit.com SHA1 fingerprint
//static const char *fingerprint PROGMEM = "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF";

// WiFiSSLClient for SSL/TLS support
WiFiSSLClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe onoff = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/aircon");


IRrecv irrecv(IR_RECEIVE_PIN);
IRsend irsend;
decode_results results;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// On the Zero and others we switch explicitly to SerialUSB
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif

void setup() {
  Serial.begin(115200);
#if defined(__AVR_ATmega32U4__)
  while (!Serial); //delay for Leonardo, but this loops forever for Maple Serial
#endif
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__));

  irrecv.enableIRIn(); // Start the receiver
  pinMode(SEND_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_PIN, OUTPUT);

  Serial.print(F("Ready to receive IR signals at pin "));
  Serial.println(IR_RECEIVE_PIN);
  Serial.print(F("Ready to send IR signals at pin "));
  Serial.println(IR_SEND_PIN);

  inputString.reserve(200);

  // if the AirLift's pins were defined above...
#ifdef SET_PINS
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
#endif

  // check for the wifi module
  while (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WiFi module failed!");
    delay(1000);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0")
  {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(WLAN_SSID);
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  do
  {
    status = WiFi.begin(WLAN_SSID, WLAN_PASS);
    delay(100); // wait until connected
  } while (status != WL_CONNECTED);
  Serial.println("Connected to wifi");
  printWiFiStatus();
  //
  //    // check the fingerprint of io.adafruit.com's SSL cert
  //    client.setFingerprint(fingerprint);
  onoff.setCallback(onoffcallback);
  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&onoff);
}

// Storage for the recorded code
int codeType = -1; // The type of code
unsigned long codeValue; // The code value if not raw
unsigned int rawCodes[RAW_BUFFER_LENGTH]; // The durations if raw
int codeLen; // The length of the code
int toggle = 0; // The RC5/6 toggle state

// Stores the code for later playback
// Most of this code is just logging
//void storeCode(decode_results *results) {
//    codeType = results->decode_type;
////  int count = results->rawlen;
//    if (codeType == UNKNOWN) {
//        Serial.println("Received unknown code, saving as raw");
//        codeLen = results->rawlen - 1;
//        // To store raw codes:
//        // Drop first value (gap)
//        // Convert from ticks to microseconds
//        // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
//        for (int i = 1; i <= codeLen; i++) {
//            if (i % 2) {
//                // Mark
//                rawCodes[i - 1] = results->rawbuf[i] * MICROS_PER_TICK - MARK_EXCESS_MICROS;
//                Serial.print(" m");
//            } else {
//                // Space
//                rawCodes[i - 1] = results->rawbuf[i] * MICROS_PER_TICK + MARK_EXCESS_MICROS;
//                Serial.print(" s");
//            }
//            Serial.print(rawCodes[i - 1], DEC);
//        }
//        Serial.println("");
//    } else {
//        if (codeType == NEC) {
//            Serial.print("Received NEC: ");
//            if (results->value == REPEAT) {
//                // Don't record a NEC repeat value as that's useless.
//                Serial.println("repeat; ignoring.");
//                return;
//            }
//        } else if (codeType == SONY) {
//            Serial.print("Received SONY: ");
//        } else if (codeType == SAMSUNG) {
//            Serial.print("Received SAMSUNG: ");
//        } else if (codeType == PANASONIC) {
//            Serial.print("Received PANASONIC: ");
//        } else if (codeType == JVC) {
//            Serial.print("Received JVC: ");
//        } else if (codeType == RC5) {
//            Serial.print("Received RC5: ");
//        } else if (codeType == RC6) {
//            Serial.print("Received RC6: ");
//        } else {
//            Serial.print("Unexpected codeType ");
//            Serial.print(codeType, DEC);
//            Serial.println("");
//        }
//        Serial.println(results->value, HEX);
//        codeValue = results->value;
//        codeLen = results->bits;
//    }
//}

void sendCode(int repeat) {
  //    if (codeType == NEC) {
  //        if (repeat) {
  //            irsend.sendNEC(REPEAT, codeLen);
  //            Serial.println("Sent NEC repeat");
  //        } else {
  //            irsend.sendNEC(codeValue, codeLen);
  //            Serial.print("Sent NEC ");
  //            Serial.println(codeValue, HEX);
  //        }
  //    } else if (codeType == SONY) {
  //        irsend.sendSony(codeValue, codeLen);
  //        Serial.print("Sent Sony ");
  //        Serial.println(codeValue, HEX);
  //    } else if (codeType == PANASONIC) {
  //        irsend.sendPanasonic(codeValue, codeLen);
  //        Serial.print("Sent Panasonic");
  //        Serial.println(codeValue, HEX);
  //    } else if (codeType == JVC) {
  //        irsend.sendJVC(codeValue, codeLen, false);
  //        Serial.print("Sent JVC");
  //        Serial.println(codeValue, HEX);
  //    } else if (codeType == RC5 || codeType == RC6) {
  //        if (!repeat) {
  //            // Flip the toggle bit for a new button press
  //            toggle = 1 - toggle;
  //        }
  //        // Put the toggle bit into the code to send
  //        codeValue = codeValue & ~(1 << (codeLen - 1));
  //        codeValue = codeValue | (toggle << (codeLen - 1));
  //        if (codeType == RC5) {
  //            Serial.print("Sent RC5 ");
  //            Serial.println(codeValue, HEX);
  //            irsend.sendRC5(codeValue, codeLen);
  //        } else {
  //            irsend.sendRC6(codeValue, codeLen);
  //            Serial.print("Sent RC6 ");
  //            Serial.println(codeValue, HEX);
  //        }
  //} else if (codeType == UNKNOWN /* i.e. raw */) {
  // Assume 38 KHz
  //irsend.sendRaw_P(rawCodes, codeLen, 38);
  Serial.println("Sent raw");
  //}
}

int lastButtonState;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  //    Adafruit_MQTT_Subscribe *subscription;
  //    while ((subscription = mqtt.readSubscription(1000))) {
  //      Serial.println("Checking...");
  //      if (subscription == &onoff) {
  //        Serial.print(F("Got: "));
  //        Serial.println((char *)onoff.lastread);
  //      }
  //    }

  //    // If button pressed, send the code.
  int buttonState = digitalRead(SEND_BUTTON_PIN); // Button pin is active LOW
  //    if (lastButtonState == LOW && buttonState == HIGH) {
  //        Serial.println("Released");
  //        irrecv.enableIRIn(); // Re-enable receiver
  //    }

  if (buttonState == LOW) {
    Serial.println("Pressed, sending power");
    digitalWrite(STATUS_PIN, HIGH);
    //sendCode(lastButtonState == buttonState);
    irsend.sendRaw(irPower, sizeof(irPower) / sizeof(irPower[0]), 38);
    digitalWrite(STATUS_PIN, LOW);
    delay(50); // Wait a bit between retransmissions
  } else if (irrecv.decode(&results)) {
    digitalWrite(STATUS_PIN, HIGH);
    //storeCode(&results);

    irrecv.resume(); // resume receiver
    digitalWrite(STATUS_PIN, LOW);
  } else if (stringComplete) {
    Serial.println(inputString);
    Serial.println("Pressed, sending " + inputString);
    digitalWrite(STATUS_PIN, HIGH);
    unsigned int irBuf[200];
    if (inputString.equals("power")) {
      irsend.sendRaw_P(irPower, sizeof(irPower) / sizeof(irPower[0]), freq);
    } else if (inputString.equals("cool")) {
      irsend.sendRaw_P(irCool, sizeof(irCool) / sizeof(irCool[0]), freq);
    } else if (inputString.equals("fan")) {
      irsend.sendRaw_P(irFan, sizeof(irFan) / sizeof(irFan[0]), freq);
    } else if (inputString.equals("temp+")) {
      irsend.sendRaw_P(irTempInc, sizeof(irTempInc) / sizeof(irTempInc[0]), freq);
    } else if (inputString.equals("temp-")) {
      irsend.sendRaw_P(irTempDec, sizeof(irTempDec) / sizeof(irTempDec[0]), freq);
    } else {
      Serial.println("Invalid command");
    }
    Serial.println("Sent raw");
    digitalWrite(STATUS_PIN, LOW);
    delay(50); // Wait a bit between retransmissions

    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  //lastButtonState = buttonState;

  // this is our 'wait for incoming subscription packets and callback em' busy subloop
  // try to spend your time here:
  mqtt.processPackets(1000);

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds

  //    if(!mqtt.ping()) {
  //      mqtt.disconnect();
  //    }
}

void sendIrP(PGM_VOID_P buf, unsigned int len, unsigned int hz) {
  irsend.enableIROut(hz);

  for (unsigned int i = 0; i < len; i++) {
    uint16_t duration = pgm_read_word_near(buf + sizeof(uint16_t) * i);
    if (i & 1) {
      irsend.space(duration);
    } else {
      irsend.mark(duration);
    }
  }
  irsend.space(0);  // Always end with the LED off
}

void onoffcallback(char *data, uint16_t len) {
  //Print the new value to the serial monitor
  Serial.print("onoff: ");
  Serial.println(data);
  String text(data);
  Serial.println("text: " + text);

  //If the new value is  "ON", turn the light on.
  //Otherwise, turn it off.
  //unsigned int irPowerP[] PROGMEM = { 8350, 4300, 450, 1700, 450, 1650, 450, 1650, 450, 650, 450, 1650, 450, 650, 400, 1700, 400, 1700, 450, 4300, 400, 1700, 450, 1650, 450, 650, 400, 1700, 450, 650, 400, 650, 400, 650, 450, 650, 500 };
  if (text.equals("power")) {
    sendIrP(irPower, sizeof(irPower) / sizeof(irPower[0]), freq);
  } else if (text.equals("cool")) {
    sendIrP(irCool, sizeof(irCool) / sizeof(irCool[0]), freq);
  } else if (text.equals("fan")) {
    sendIrP(irFan, sizeof(irFan) / sizeof(irFan[0]), freq);
  } else if (text.equals("temp+")) {
    sendIrP(irTempInc, sizeof(irTempInc) / sizeof(irTempInc[0]), freq);
  } else if (text.equals("temp-")) {
    sendIrP(irTempDec, sizeof(irTempDec) / sizeof(irTempDec[0]), freq);
  } else {
    Serial.println("Invalid command");
  }
  Serial.println("Sent raw");
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      inputString = inputString.substring(0, inputString.length() - 1);
    }
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds
    retries--;
    if (retries == 0)
    {
      // basically die and wait for WDT to reset me
      while (1)
        ;
    }
  }

  Serial.println("MQTT Connected!");
}

void printWiFiStatus()
{
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
