
#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

char ssid[] = "TestAP";           // your network SSID (name)
char pass[] = "12345678";         // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)

float temp;                       // declare variable for temperature
int humid, pres, light;           // declare variables for humidity, pressure, light  
const int motionSensor = 3;       // declare pin for PIR-sensor
boolean pirState = false;         // initalize state of movement detection

const char* pirStateStr[] = {"EI", "KYLLÄ"};   // save descriptions for possible PIR sensor states

Adafruit_BME280 bme; // I2C

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // initalize BME280
  bme.begin(0x76); 

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT);

  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // check if TSL-sensor responds
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }

  //Configure the light sensor
  configureSensor();

  Serial.println("Access Point Web Server");
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}


void loop() {
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  // Temperature reading (BME280)
  temp = bme.readTemperature();   // temperature in degrees celsius

  // Humidity reading (BME280)
  humid = bme.readHumidity();   // humidity in %

  // Pressure reading (BME280)
  pres = bme.readPressure() / 100.0F; // pressure in hPA / mBar

  //Light intensity reading (TSL2591)
  lightRead();
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta charset=\"utf-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\"><style> ");
           
            // CSS to style
            client.println("h1 {text-align: center;}");
            client.println("table {margin-left:auto;margin-right:auto;width: auto;}");
            client.println("td, th {text-align: left;padding-left: 15px;padding-right: 15px;}");
            client.println("button {margin-left:auto;margin-right:auto;}");
            
            // Web Page Content
            client.println("</style></head><body><h1>Mittaukset</h1>");

            client.println("<table><tr><td>Lämpötila</td><td>");
            client.println(temp);                     // enviroment temperature
            client.println(" °C</td></tr>");
            
            client.println("<tr><td>Kosteus</td><td>");
            client.println(humid);                    // humidity
            client.println(" %</td></tr>");
            
            client.println("<tr><td>Paine</td><td>");
            client.println(pres);                     // atmospheric pressure
            client.println(" mBar</td></tr>");
            
            client.println("<tr><td>Valoisuus</td><td>");
            client.println(light);                    // light intensity value
            client.println(" lux</td></tr>");

            client.println("<tr><td>Paikalla?</td><td>");
            client.println(pirStateStr[pirState]);    // get description based on boolean value (0 = POIS, 1 = KYLLÄ)
            client.println("</td></tr>");
            client.println("</table></body><br>");

            // Update button
            client.println("<a href=./><button type=button style=margin:auto;display:block>UPDATE</button></a>");
            client.println("</html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }

    // set PIR movement-detected state back to false
    // This shows the user if movement was detected BETWEEN two latest page-refreshes
    pirState = false;
    
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

// if movement is detected after latest page refresh, then save that info to pirState
void detectsMovement() {
  pirState = true;
}

// read light intesity
void lightRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  light = tsl.calculateLux(full, ir);
}

// perform initial TSL-sensor configuration
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  
  tsl2591Gain_t gain = tsl.getGain();
}


// Esplora Built-In by Arduino Version 1.0.4 INSTALLED
// Bridge Built-In by Arduino Version 1.7.0 INSTALLED
// Firmata Built-In by Firmata Developers Version 2.5.8 INSTALLED
// Ethernet Built-In by Arduino Version 2.0.0 INSTALLED
// Robot Control Built-In by Arduino Version 1.0.4 INSTALLED
// Robot IR Remote Built-In by Arduino Version 2.0.0 INSTALLED
// Robot Motor Built-In by Arduino Version 1.0.3 INSTALLED
// SD Built-In by Arduino, SparkFun Version 1.2.4 INSTALLED
// Servo Built-In by Michael Margolis, Arduino Version 1.1.8 INSTALLED
// SpacebrewYun Built-In by Julio Terra Version 1.0.2 INSTALLED
//   SPI Built-In by Arduino Version 1.0.0 INSTALLED
//   Wire Built-In by Arduino Version 1.0.0 INSTALLED
// HID(samd) Built-In by Arduino Version 1.0.0 INSTALLED
// I2S Built-In by Arduino Version 1.0.0 INSTALLED
// SAMD_AnalogCorrection Built-In by Arduino Version 1.0.0 INSTALLED
// SAMD_BootloaderUpdater Built-In by Arduino Version 1.0.0 INSTALLED
// SBU Built-In by Arduino Version 1.0.0 INSTALLED
// SDU Built-In by Arduino Version 1.0.0 INSTALLED
// SFU Built-In by Arduino Version 1.0.0 INSTALLED
// SNU Built-In by Arduino Version 1.0.2 INSTALLED
// SPI(samd) Built-In by Jonathan BAUDIN, Thibaut VIARD, Arduino Version 1.0.0 INSTALLED
// SSU Built-In by Arduino Version 1.0.0 INSTALLED
// USBHost(samd) Built-In by Arduino Version 1.0.0 INSTALLED
// Wire(samd) Built-In by Jonathan BAUDIN, Thibaut VIARD, Arduino Version 1.0.0 INSTALLED