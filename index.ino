//Arduino Nano33 Iot (Lämpötila, kosteus, paine, valoisuus, paikalla)

#include <SPI.h>
#include <WiFiNINA.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // anturin tunnisteen numero (käytetään myöhemmin)

char ssid[] = "TestAP";           // verkon SSID (nimi)
char pass[] = "12345678";         // verkkon salasana
int keyIndex = 0;                 // verkkoavaimen indexinumero (WEP)

float temp;                       // Lämpötilan muuttuja
int humid, pres, light;           // Kosteuden, paineen ja valoisuuden muuttujat
const int motionSensor = 3;       // Liiketunnistimen muuttuja
boolean pirState = false;         // Liikkeen havaitsemisen tilan alustus

const char* pirStateStr[] = {"EI", "KYLLÄ"};   // Talleta liiketunnistimen tila

Adafruit_BME280 bme; // I2C

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  //Alustaa sarjaportin ja odottaa että on auennut
  Serial.begin(9600);
  //while (!Serial) {
   //; //odottaa että sarjaportti on kytkeytynyt
  //}

  // alusta BME280
  bme.begin(0x76); 

  // PIR liiketunnistimen tila INPUT_PULLUP
  pinMode(motionSensor, INPUT);

  // Aseta liiketunnistimen nasta keskeytykseksi, määritä keskeytystoiminto ja aseta RISING-tila
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Tarkista, vastaako TSL-anturi
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }

  //Konfiguroi valo-anturi
  configureSensor();

  Serial.println("Access Point Web Server");
  // tarkista WiFi moduuli:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // Älä jatka
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // oletuksena paikallinen IP-osoite on 192.168.4.1
  // voit ohittaa sen seuraavilla tavoilla:
  // WiFi.config(IP-osoite(10, 0, 0, 1));

  // Tulosta verkon nimi (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Luo avoin verkko
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // Älä jatka
    while (true);
  }

  //odota 10 sekuntia yhteyden muodostamista
  delay(10000);

  // käynnistä verkkopalvelin portista 80
  server.begin();

  // olet nyt yhteydessä, tulosta tila
  printWiFiStatus();
}


void loop() {
  // vertaa edellistä tilaa nykyiseen tilaan
  if (status != WiFi.status()) {
    // se on muuttunut päivitä muuttuja
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // laite on yhdistetty AP
      Serial.println("Device connected to AP");
    } else {
      // laite on katkaistu AP, ja on kuuntelutilassa
      Serial.println("Device disconnected from AP");
    }
  }

  // Lämpötilan lukema (BME280)
  temp = bme.readTemperature();   // lämpötila celsiusasteina

  // Kosteuslukema (BME280)
  humid = bme.readHumidity();   // kosteus %

  // Paine lukema (BME280)
  pres = bme.readPressure() / 100.0F; // paine hPA / mBar

  //valon voimakkuuden lukema (TSL2591)
  lightRead();
  
  WiFiClient client = server.available();   // kuuntelee saapuvia asiakkaita

  if (client) {                             // Jos saat asiakkaan,
    Serial.println("new client");           // tulostaa viestin sarjaportista
    String currentLine = "";                // Tee merkkijono, joka sisältää asiakkaalta saapuvat tiedot
    while (client.connected()) {            // silmukan, kun asiakas on yhteydessä
      delayMicroseconds(10);                // Tämä vaaditaan Arduino Nano RP2040 kytkennällä - muuten silmukka toimii niin nopeasti, että SPI:tä ei suoriudu.
      if (client.available()) {             // jos asiakkaalta on luettu tavuja
        char c = client.read();             // lue sitten tavu
        Serial.write(c);                    // tulosta se sarjanäytölle
        if (c == '\n') {                    // jos tavu on rivinvaihtomerkki

          //jos nykyinen rivi on tyhjä, sinulla on kaksi rivinvaihtomerkkiä peräkkäin
          // tämä on asiakkaan HTTP-pyynnön loppu, joten lähetä vastaus
          if (currentLine.length() == 0) {
            // HTTP-otsikot alkavat aina vastauskoodilla (e.g. HTTP/1.1 200 OK)
            // ja sisältötyyppi, jotta asiakas tietää, mitä on tulossa, sitten tyhjä rivi:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Näytä HTML-verkkosivu
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta charset=\"utf-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\"><style> ");
           
            // CSS tyyli
            client.println("h1 {text-align: center;}");
            client.println("table {margin-left:auto;margin-right:auto;width: auto;}");
            client.println("td, th {text-align: left;padding-left: 15px;padding-right: 15px;}");
            client.println("button {margin-left:auto;margin-right:auto;}");
            
            // Web-sivun sisältö
            client.println("</style></head><body><h1><u>MITTAUKSET</u></h1>");

            client.println("<table><tr><td>Lämpötila</td><td>");
            client.println(temp);                     // Ympäristön lämpötila
            client.println(" °C</td></tr>");
            
            client.println("<tr><td>Kosteus</td><td>");
            client.println(humid);                    // Koskteus
            client.println(" %</td></tr>");
            
            client.println("<tr><td>Paine</td><td>");
            client.println(pres);                     // Ilmakehän paine
            client.println(" mBar</td></tr>");
            
            client.println("<tr><td>Valoisuus</td><td>");
            client.println(light);                    // Valon voimakkuus arvo
            client.println(" lux</td></tr>");

            client.println("<tr><td>Paikalla?</td><td>");
            client.println(pirStateStr[pirState]);    // kuvaus loogisen arvon perusteella (0 = POIS, 1 = KYLLÄ)
            client.println("</td></tr>");
            client.println("</table></body><br>");

            // Päivitä Button
            client.println("<a href=./><button type=button style=margin:auto;display:block;font-size:300%>UPDATE</button></a>");
            client.println("</html>");

            // HTTP-vastaus päättyy toiseen tyhjään riviin:
            client.println();
            // keskeytä ehile-silmukka
            break;
          }
          else {      // jos on rivinvaihto, tyhjennä nykyinen rivi:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // jos on rivinvaihto merkki
          currentLine += c;      // lisää se nykyisen rivin loppuun
        }
      }
    }

    // aseta PIR-liikkeen havaitsemistila takaisin epätosi
    // Tämä näyttää käyttäjälle, havaittiinko liikettä kahden viimeisimmän sivun päivityksen völillä
    pirState = false;
    
    // sulje yhteys:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // Tulostaa SSID verkon nimen
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Tulostaa WiFi IP osoitteet:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Tulosta linkki joka avataan selaimessa:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

// Jos liikettä havaitaan viimeisen sivun päivityksen jälkeen, tallenna tiedot pirStateen
void detectsMovement() {
  pirState = true;
}


void lightRead(void)
{
  //Edistyneemmän datan lukuesimerkki. Lue 32 bittiä ylhäällä 16 bitin IR:llä, alhaalla 16 bittiä täydellä spektrillä
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  light = tsl.calculateLux(full, ir);
}

// TSL-anturin konfigurointi
void configureSensor(void)
{
  // Voit muuttaa vahvistusta mukautuaksesi kirkkaampiin/himmeämpiin valotilanteisiin
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x vahvistus (kirkas valo)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x vahvistus
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x vahvistus
  
  // Integrointiajan muuttaminen antaa pidemmän ajan valon havaitsemiseen
  // pidemmät ajat ovat hitaampia, mutta toimivat paremmin heikossa valaistuksessa!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  
  tsl2591Gain_t gain = tsl.getGain();
}






