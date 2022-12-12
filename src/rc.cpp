// #include <Ps3Controller.h>
#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>

esp_now_peer_info_t slave;


int player = 0;
int battery = 0;
float rctime=0.0;

//RC
volatile float Stick[16];

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) {
  char macStr[18];
  char msg[1];
  char strdata[200];

  //snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
  //        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.printf("Last Packet Recv from: %s\n", macStr);
  //Serial.printf("Last Packet Recv Data(%d): ", data_len);
  //for ( int i = 0 ; i < data_len ; i++ ) {
  //  msg[1] = data[i];
  //  Serial.print(data[i]);
  //  Serial.print(" ");
  //}
  //Serial.println("");

  Stick[RUDDER] = (short)(recv_data[0]*256+recv_data[1]);
  Stick[THROTTLE] = (short)(recv_data[2]*256+recv_data[3]);
  Stick[AILERON]  = (short)(recv_data[4]*256+recv_data[5])/1000.0;
  Stick[ELEVATOR] = (short)(recv_data[6]*256+recv_data[7])/1000.0;
  Stick[BUTTON] = recv_data[10];
  Stick[LOG] = 0.0;

  //Normalize
  Stick[RUDDER] /= -RUDDER_MAX;
  Stick[THROTTLE] /= THROTTLE_MAX;
  Stick[AILERON] /= (0.5*3.14159);
  Stick[ELEVATOR] /= (0.5*3.14159);
  if(Stick[THROTTLE]<0.0) Stick[THROTTLE]=0.0;

//  sprintf(strdata, "THR:%7.3f RUD:%7.3f AIL: %7.3f ELE: %7.3f Btn: %2d\n", 
//    Stick[THROTTLE], Stick[RUDDER], Stick[AILERON], Stick[ELEVATOR], (int)Stick[BUTTON]);
//  sprintf(strdata, "%7.3f %7.3f %7.3f %7.3f %7.3f %2d\r\n", 
//    rctime, Stick[THROTTLE], Stick[RUDDER], Stick[AILERON], Stick[ELEVATOR], (int)Stick[BUTTON]);
//  Serial.print(strdata);
  rctime = rctime + 0.01;
}

void rc_init(void)
{
  //Initialize Stick list 
  for (uint8_t i = 0;i<16;i++)Stick[i]=0.0;

  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }
  
  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  // esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

    // Ps3.attach(notify);
    // Ps3.attachOnConnect(onConnect);
    // Ps3.begin(BTID);
    Serial.println("Ready.");
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












