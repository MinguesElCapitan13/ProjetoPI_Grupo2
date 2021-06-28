/*
  WINDOWS 10 HOME v20H2
  Versao IDE 1.8.13
  Necessária a instalação das bibliotecas:
  ArduinoBLE
  SPI
  WiFiNINA
  WiFiUdp
  NTPClient


  Este código é uma tentativa de sincronizaçao entre os 3 arduinos recetores. O código começa com NTP e apenas arranca a cada 30 ou 0s.
  O código BLE funciona num ciclo de um perido de amostragem de 15s ( os 15s sao justificáveis pelo periodo de utilizaçao do arduino o qual após 2/3min sobreaquece e nao tira medidas)
  Formato envio: MAC RSSI

*/

// Definiçao das bibliotecas
#include <ArduinoBLE.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

//Drivers
#include "utility/wifi_drv.h"   //Drivers que precisam ser re-inicializados por cada ciclo BLE/WI-FI

// Definiçao de arrays
#define ARRAYSIZE 12 // Definição do tamanho do array de envio de dados
String mac[ARRAYSIZE];  //Array de Strings MAC
int rssi_val[ARRAYSIZE];  //Array de inteiros com RSSI
String compare[ARRAYSIZE];

//Definiçao de variáveis
String v1;  //MAC
String v2;  //RSSI
String v3;  //TS
int maxT = 10; //Tempo máximo de medidas
int entries;
int WIFItimer;
int cont;
int found = 0;
int count = 0;
int devices = 0;
int tests = 0;
int contador;
int teste = 0;
//int firstTime = 0;
int ts = 0;
int segundos;
int minutos;
int horas;
int dia;
int start;
int offset = 5;
int occ = 0;
int deltat = 0;

boolean firstTime;
boolean myflag;

// Definiçoes para Wi-Fi
#include "arduino_secrets.h"  //Ficheiro com os secrets wifi
//char ssidName[] = SECRET_SSID;
//char ssidPass[] = SECRET_PASS;
char ssid[] = SECRET_SSID;  // your WPA2 enterprise network SSID (name)
char user[] = SECRET_USER;  // your WPA2 enterprise username
char pass[] = SECRET_PASS;  // your WPA2 enterprise password
int status_wifi = WL_IDLE_STATUS; //flag status
WiFiServer server( 80 );      //porta
int status = WL_IDLE_STATUS;    //flag status
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).

//LEDS
#define LED_PIN 13 // Pino do LED 
bool state = 0; //  ON/OFF LED

//Variáveis para o timer
//ritmo de amostragem em millisegundos, determina de quanto em quanto tempo o TC5_Handler é chamado
uint32_t sampleRate = 1000; 
uint32_t onesecondinterval = 0;




void setup()
{
  Serial.begin( 9600 );
  while ( !Serial );
  pinMode(LED_PIN, OUTPUT); //this configures the LED pin
  firstTime = true; //flag para inicializaçao
  myflag = true;

}


void loop()
{
  contador = 0;

  //Quando é a primeira vez
  if ( firstTime == true) {
    Serial.println("Switching to WIFI");
    wifiMode(myflag);

  }
  if (start == 1) {   // Flag start para começar o timer
    start = 0;
    Serial.println("First time");
    TimerInit();  //Init Timer
    tcStartCounter(); //Start Counter
  }


  maxT = 15;  //Tempo limite condiçao
  while (onesecondinterval < maxT) {
    //send-data
    Serial.println("A inicializar BLE");
    bleInit();
    bleMode();
    Serial.println("------------------------");
    for (int  s = 0; s <= found - 1; s++)
    {

      Serial.println(mac[s]);                                    //Escrita na COM
      Serial.println(rssi_val[s]);

    }
    Serial.println("------------------------");
    delay(600);                                            //delay  para conseguir tirar valores e imprimir
  }



  if (onesecondinterval == maxT) {                  //Quando chegar aos 15s inicializa wi-fi
    tcReset();
    Serial.println("A inicializar WI-FI");
    myflag = true;
    switch2WiFiMode();
    wifiMode(myflag);
    TimerInit();
    onesecondinterval = 0;
    tcStartCounter();
    Serial.println("A sair do wi-fi");
  }



}

void TimerInit() {


  //tcDisable(); //Stop/pause the timer
  //tcReset(); //Reset
  tcConfigure(sampleRate); //Configuração do sample rate <sampleRate>Hertz
  //tcStartCounter(); //Start
}


//CONFIGURAÇÃO DOS TIMERS

//this function gets called by the interrupt at <sampleRate>Hertz

void TC5_Handler (void) {
  onesecondinterval++;
  Serial.print("Timer count:");
  Serial.println(onesecondinterval);

  if (state == true) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  state = !state;

  if (onesecondinterval > maxT) {
    tcReset();
    TimerInit();
    //      onesecondinterval = 0;

  }

  TC5->COUNT16.INTFLAG.bit.MC0 = 1;                 //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}

/*
    TIMER SPECIFIC FUNCTIONS FOLLOW
    you shouldn't change these unless you know what you're doing
*/

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
void tcConfigure(int sampleRate)
{
  // select the generic clock generator used as source to the generic clock multiplexer
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter 5 Mode to 16 bits, it will become a 16bit counter ('mode1' in the datasheet)
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 waveform generation mode to 'match frequency'
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler
  //the clock normally counts at the GCLK_TC frequency, but we can set it to divide that frequency to slow it down
  //you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get a different range
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; //it will divide GCLK_TC frequency by 1024
  //set the compare-capture register.
  //The counter will count up to this value (it's a 16bit counter so we use uint16_t)
  //this is how we fine-tune the frequency, make it count to a lower or higher value
  //system clock should be 1MHz (8MHz/8) at Reset by default
  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate);
  while (tcIsSyncing());

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing()); //wait until TC5 is done syncing
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}





void bleInit() {
  //Inicialização
  while (!BLE.begin()) // BLE.begin() 0-Falha 1-Sucesso
  {
    Serial.println("Falha na inicialização BLE!");
  }
  Serial.println("Scan BLE");
  BLE.scan(); // BLE.scan() 0-Falha 1-Sucesso
}


void NTPConnect() {

  timeClient.update();

  Serial.println(timeClient.getFormattedTime());

}

void bleMode() {
  BLEDevice peripheral = BLE.available();     //procura de dispositivos perifericos
  // Condição de busca de um dispositivo periférico encontrado
  if (peripheral) {
    Serial.println("Found one device");
    if (found != ARRAYSIZE) {
      found++;
    }

    String v1 = peripheral.address(); //escrever no array o endereço mac do dispositivo
    int v2 = peripheral.rssi();       //escrever no array o valor de RSSI
    mac[devices] = v1;
    rssi_val[devices] = v2;
    devices++;                        //dispositivo incrementa
    if (devices == 12) {   //se o array de devices estiver cheio vai rescrever no primeiro slot do array
      devices = 0;         //reset
    }
  }
}



void wifiMode(bool flag)
{
  firstTime = flag; // flag para NTP
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  while (status != WL_CONNECTED) {
    Serial.print("A tentar ligar à rede: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    //Conexão para redes com segurançao WPA/WPA2. Para redes abertas ou WEP comentar a linha seguinte.
    //status = WiFi.begin(ssidName, ssidPass);
    // Connect to WPA2 enterprise network:
    // - You can optionally provide additional identity and CA cert (string) parameters if your network requires them:
    //      WiFi.beginEnterprise(ssid, user, pass, identity, caCert)
    status = WiFi.beginEnterprise(ssid, user, pass);
  }


  do {

    timeClient.update();
    segundos = timeClient.getSeconds();
    Serial.println(timeClient.getFormattedTime());
    delay(1000);
    if (segundos == 0 || segundos == 1 || segundos == 30 || segundos == 31) { //Esperar pelo t=0, t=30 -> t=1,t=31 porque ao fim de um tempo ele conta de 2 em 2 ( razões desconhecidas )

      firstTime = false;
      start = 1;

    }

  }
  while (firstTime == true);

  Serial.println("Aguardando conexão ao servidor web");
  server.begin();
  // you're connected now, so print out the status:
  printWiFiStatus();

  //-----------------------------------------Conexao webserver------------------------------------------
  contador = 0;
  do {
    WiFiClient client = server.available();
    if (client) {
      Serial.println("New client");
      
      boolean currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          Serial.write(c);
          if (c == '\n' && currentLineIsBlank) {  
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");  // A conexao é fechada após o término da resposta 
            client.println("Refresh: 2");  // Refresh a cada 2s
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            client.println("<p>");

            cont = 0;
            do {
              //Envio das infos
              client.println("<p>");
              client.println("MAC: " + mac[cont] + "; RSSI: " + rssi_val[cont]);
              client.println("<p>");
              cont = cont + 1;

            } while (cont != (ARRAYSIZE));

            Serial.println("-----------------------");
            client.println("<br />");
            client.println("</html>");
            Serial.println("Espere...");
            client.println("</html>");
            break;
          }
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
          } else if (c != '\r') {
            // you've gotten a character on the current line
            currentLineIsBlank = false;
          }
        }
      }
      
      // Atraso para garantir o envio dos dados
      delay(600);
      

      // close the connection:
      client.stop();
      Serial.println("Client disconectado");
    }

    delay(1000);


    contador++;
    Serial.print("Contador final:");
    Serial.println(contador);

  } while (contador <= 3);
}


bool switch2BleMode()
{
  if ( !BLE.begin() )
  {
    Serial.println("starting BLE failed!");
    return false;
  }

  Serial.println("BLE scan");
  //BLE.advertise();
  // start scanning for peripheral
  BLE.scan();
  //Serial.println("Delay 1s");
  //delay(1000);        //ATENÇÃO QUE PODE HAVER PROBLEMAS COM ESTE DELAY: REPENSAR SE PRECISAMOS DELE

  return true;
}


bool switch2WiFiMode()
{
  BLE.stopAdvertise();
  BLE.end();
  status = WL_IDLE_STATUS;

  // Re-initialize the WiFi driver
  // This is currently necessary to switch from BLE to WiFi
  wiFiDrv.wifiDriverDeinit();
  wiFiDrv.wifiDriverInit();

  return true;
}

void printWiFiStatus()
{
  Serial.print( "SSID: " );
  Serial.println( WiFi.SSID() );
  
  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);
  
  IPAddress ip = WiFi.localIP();
  Serial.print( "Endreço IP: " );
  Serial.println( ip );

  long rssi_wifi = WiFi.RSSI();
  Serial.print( "Potência do sinal (RSSI):" );
  Serial.print( rssi_wifi );
  Serial.println( " dBm" );
  
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("Endereço MAC: ");
  printMacAddress(mac);
  
  // Tipo de encriptação:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
  
  Serial.print("Marcador temporal->");
  NTPConnect();
  Serial.println();
  segundos = timeClient.getSeconds();
  minutos = timeClient.getMinutes();
  horas = timeClient.getHours();
  dia = timeClient.getDay();  
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
