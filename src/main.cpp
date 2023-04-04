#include <Arduino.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include "serial.h"


static const char * ssid = "qwerty";
static const char * password = "12345678";

static AsyncUDP udp;
static IPAddress LastIP;
static tcpip_adapter_if_t LastIntf;
static uint16_t LastPort;


void setup()
{
    Serial.begin(115200);
    Uart2Start();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, 10,0,1);

    IPAddress IP = WiFi.softAPIP();

    Serial.print("AP IP address: ");
    Serial.println(IP);

    if(udp.listen(5000)) {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet) {
            Uart2Send(packet.data(), packet.length());
          //   Serial.print("UDP Packet Type: ");
          //   Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
          //   Serial.print(", From: ");
          //   Serial.print(packet.remoteIP());
          //   Serial.print(":");
          //   Serial.print(packet.remotePort());
          //   Serial.print(", To: ");
          //   Serial.print(packet.localIP());
          //   Serial.print(":");
          //   Serial.print(packet.localPort());
          //   Serial.print(", Length: ");
          //   Serial.print(packet.length());
          //  // Serial.print(", Data: ");
          //  // Serial.write(packet.data(), packet.length());
          //   Serial.println();
            LastIP = packet.remoteIP();
            LastIntf = packet.interface();
            LastPort = packet.remotePort();
        });
    }
}

void loop()
{
  while (write_count < read_count)
  {
       size_t nw =  read_count - write_count;
       if ((nw >= CONFIG_TCP_MSS) || UartBreack)     
       { 
         size_t w = udp.writeTo(&buf[write_count], nw, LastIP, LastPort, LastIntf);
         write_count +=w;
        //  Serial.print("write_count: ");
        //  Serial.print(write_count);
        //  Serial.print(" read_count: ");
        //  Serial.print(read_count);
        //  Serial.print(" send w: ");
        //  Serial.println(w);
         taskYIELD();
       }
  }
  taskYIELD(); 
  Uart2TurboHandler();
}
