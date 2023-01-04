// #include <Ps3Controller.h>
#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>

//esp_now_peer_info_t slave;


int player = 0;
int battery = 0;
float rctime=0.0;
volatile uint8_t Connect_flag = 0;

//Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
const uint8_t addr[6] = {0x4C, 0x75, 0x25, 0xAD, 0xB6, 0x6C};

esp_now_peer_info_t peerInfo;

//RC
volatile float Stick[16];

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) 
{
  Connect_flag = 1;

  uint8_t* d_int;
  int16_t d_short;
  float d_float;

  d_int = (uint8_t*)&d_short;
  d_int[0]=recv_data[0];
  d_int[1]=recv_data[1];
  Stick[RUDDER]=(float)d_short;

  d_int[0]=recv_data[2];
  d_int[1]=recv_data[3];
  Stick[THROTTLE]=(float)d_short;

  d_int = (uint8_t*)&d_float;
  d_int[0] = recv_data[4];
  d_int[1] = recv_data[5];
  d_int[2] = recv_data[6];
  d_int[3] = recv_data[7];
  Stick[AILERON]  = d_float;

  d_int[0] = recv_data[8];
  d_int[1] = recv_data[9];
  d_int[2] = recv_data[10];
  d_int[3] = recv_data[11];
  Stick[ELEVATOR]  = d_float;

  Stick[BUTTON] = recv_data[12];
  Stick[BUTTON_A] = recv_data[13];
  Stick[CONTROLMODE] = recv_data[14];
  
  Stick[LOG] = 0.0;

  //Normalize
  Stick[RUDDER] /= -RUDDER_MAX;
  Stick[THROTTLE] /= THROTTLE_MAX;
  Stick[AILERON] /= (0.5*3.14159);
  Stick[ELEVATOR] /= (0.5*3.14159);
  if(Stick[THROTTLE]<0.0) Stick[THROTTLE]=0.0;
}

void rc_init(void)
{
  //Initialize Stick list 
  for (uint8_t i = 0;i<16;i++)Stick[i]=0.0;

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("MAC ADDRESS: %s\r\n", (WiFi.macAddress()).c_str());

  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  //ペアリング
  
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        Serial.println("Failed to add peer");
        return;
  }

  // ESP-NOWコールバック登録
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW Ready.");
  Serial.println("Wait Contoroller ready....");
  //while(Connect_flag==0);
  Serial.println("Contoroller ready !");

}

void telemetry_send(uint8_t* data, uint16_t datalen)
{
  //uint8_t data[1];
  //data[0]=0xff;
  esp_err_t result = esp_now_send(peerInfo.peer_addr, data, datalen);
  //Serial.printf("%d\r\n", sizeof(data));
}

void rc_end(void)
{
    // Ps3.end();
}

bool rc_isconnected(void)
{
    // return Ps3.isConnected();
    return 1;
}

void rc_demo()
{
    // if(!Ps3.isConnected())
    //     return;

    // //-------------------- Player LEDs -------------------
    // Serial.print("Setting LEDs to player "); Serial.println(player, DEC);
    // Ps3.setPlayer(player);

    // player += 1;
    // if(player > 10) player = 0;

    // delay(2000);
}












