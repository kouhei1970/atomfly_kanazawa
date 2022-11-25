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


// void notify()
// {
//     //--- Digital cross/square/triangle/circle button events ---
//     if( Ps3.event.button_down.circle )
//     {
//         //Serial.println("Started pressing the circle button");
//         Stick[LOG] = 1;
//     }
//     if( Ps3.event.button_up.circle )
//     {
//         //Serial.println("Released the circle button");
//     }
//     if( Ps3.event.button_down.cross )
//     {
//         //Serial.println("Started pressing the cross button");
//         Stick[LOG] = 0;
//     }
//     if( Ps3.event.button_up.cross )
//     {
//         //Serial.println("Released the cross button");
//     }

//     if( Ps3.event.button_down.up )
//     {
//         Stick[DPAD_UP]=1;
//     }
//     if( Ps3.event.button_up.up ){
//         Stick[DPAD_UP]=0;
//     }
//     if( Ps3.event.button_down.down )
//     {
//         Stick[DPAD_DOWN]=1;
//     }
//     if( Ps3.event.button_up.down ){
//         Stick[DPAD_DOWN]=0;
//     }
//     if( Ps3.event.button_down.left )
//     {
//         Stick[DPAD_LEFT]=1;
//     }
//     if( Ps3.event.button_up.left ){
//         Stick[DPAD_LEFT]=0;
//     }
//     if( Ps3.event.button_down.right )
//     {
//         Stick[DPAD_RIGHT]=1;
//     }
//     if( Ps3.event.button_up.right ){
//         Stick[DPAD_RIGHT]=0;
//     }


//     //---------- Digital select/start/ps button events ---------
//     if( Ps3.event.button_down.select )
//         Serial.println("Started pressing the select button");
//     if( Ps3.event.button_up.select )
//         Serial.println("Released the select button");

//     if( Ps3.event.button_down.start )
//         Serial.println("Started pressing the start button");
//     if( Ps3.event.button_up.start )
//         Serial.println("Released the start button");

//     if( Ps3.event.button_down.ps )
//         Serial.println("Started pressing the Playstation button");
//     if( Ps3.event.button_up.ps )
//         Serial.println("Released the Playstation button");


//     //---------------- Analog stick value events ---------------
//    if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
//        //Serial.print("Moved the left stick:");
//        //Serial.print(" x="); Serial.print(Ps3.data.analog.stick.lx, DEC);
//        //Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ly, DEC);
//        //Serial.println();

//     }

//    if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
//        //Serial.print("Moved the right stick:");
//        //Serial.print(" x="); Serial.print(Ps3.data.analog.stick.rx, DEC);
//        //Serial.print(" y="); Serial.print(Ps3.data.analog.stick.ry, DEC);
//        //Serial.println();
//    }

//    //---------------------- Battery events ---------------------
//     if( battery != Ps3.data.status.battery ){
//         battery = Ps3.data.status.battery;
//         Serial.print("The controller battery is ");
//         if( battery == ps3_status_battery_charging )      Serial.println("charging");
//         else if( battery == ps3_status_battery_full )     Serial.println("FULL");
//         else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
//         else if( battery == ps3_status_battery_low)       Serial.println("LOW");
//         else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
//         else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
//         else Serial.println("UNDEFINED");
//     }

//     //Throttle
//     Stick[THROTTLE] = (float)Ps3.data.analog.stick.ry/THROTTLE_MIN;
//     if (Stick[THROTTLE] < 0) Stick[THROTTLE] = 0;
//     //Rudder
//     Stick[RUDDER] =  Ps3.data.analog.stick.rx;
//     if (Stick[RUDDER]>0) Stick[RUDDER] =  Stick[RUDDER]/RUDDER_MAX;
//     else if (Stick[RUDDER]<0) Stick[RUDDER] = -Stick[RUDDER]/RUDDER_MIN;
//     Stick[RUDDER] = -Stick[RUDDER];
//     //Elevator
//     Stick[ELEVATOR] =  Ps3.data.analog.stick.ly;
//     if (Stick[ELEVATOR]>0) Stick[ELEVATOR] =  Stick[ELEVATOR]/ELEVATOR_MAX;
//     else if (Stick[ELEVATOR]<0) Stick[ELEVATOR] = -Stick[ELEVATOR]/ELEVATOR_MIN;
//     //Aileron
//     Stick[AILERON]  =   Ps3.data.analog.stick.lx;
//     if (Stick[AILERON]>0) Stick[AILERON] = Stick[AILERON]/AILERON_MAX;
//     else if (Stick[AILERON]<0) Stick[AILERON] = - Stick[AILERON]/AILERON_MIN;

//     //Serial.printf("%6.3f %6.3f %6.3f %6.3f %4d\n",Stick_throttle, Stick_roll, Stick_pitch, Stick_yaw, Log_sw);

// }

// void onConnect(){
//     Serial.println("Connected.");
// }

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

  #if 0
  Stick[THROTTLE] = -(float)data[4]/THROTTLE_MIN - 1;
  if (Stick[THROTTLE] < 0) Stick[THROTTLE] = 0;
  //Rudder
  Stick[RUDDER] =  (data[3] - 127);//10;
  if (Stick[RUDDER]>0) Stick[RUDDER] =  - Stick[RUDDER]/RUDDER_MAX;
  else if (Stick[RUDDER]<0) Stick[RUDDER] = Stick[RUDDER]/RUDDER_MIN;
  Stick[RUDDER] = -Stick[RUDDER];
  //Elevator
  Stick[ELEVATOR] =  (data[6] - 127);//10;
  if (Stick[ELEVATOR]>0) Stick[ELEVATOR] =  -Stick[ELEVATOR]/ELEVATOR_MAX;
  else if (Stick[ELEVATOR]<0) Stick[ELEVATOR] = Stick[ELEVATOR]/ELEVATOR_MIN;
  //Aileron
  Stick[AILERON]  =   (data[5] - 127);//10;
  if (Stick[AILERON]>0) Stick[AILERON] = - Stick[AILERON]/AILERON_MAX;
  else if (Stick[AILERON]<0) Stick[AILERON] = Stick[AILERON]/AILERON_MIN;
  Stick[DPAD_UP] = (data[7]);
  if (Stick[DPAD_UP] != 0 ) ESP.restart();
  #endif

//  sprintf(strdata, "THR:%7.3f RUD:%7.3f AIL: %7.3f ELE: %7.3f Btn: %2d\n", 
//    Stick[THROTTLE], Stick[RUDDER], Stick[AILERON], Stick[ELEVATOR], (int)Stick[BUTTON]);
  sprintf(strdata, "%7.3f %7.3f %7.3f %7.3f %7.3f %2d\n", 
    rctime, Stick[THROTTLE], Stick[RUDDER], Stick[AILERON], Stick[ELEVATOR], (int)Stick[BUTTON]);
  Serial.print(strdata);
  rctime = rctime + 0.01;
  #if 0
  Serial.print(Stick[THROTTLE]);
  Serial.print(",");
  Serial.print(Stick[RUDDER]);
  Serial.print(",");
  Serial.print(Stick[ELEVATOR]);
  Serial.print(",");
  Serial.println(Stick[AILERON]);
  #endif
}


void rc_init(void)
{
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












