/* Controller for AtomFly2 */
#include <M5StickCPlus.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "MadgwickAHRS.h"


#define ANGLECONTROL 0
#define RATECONTROL 1
#define ANGLECONTROL_W_LOG 2
#define RATECONTROL_W_LOG 3

Madgwick Ahrs;

esp_now_peer_info_t peerInfo;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float gyroX_bias=0.0;
float gyroY_bias=0.0;
float gyroZ_bias=0.0;
float Ax, Ay, Az;
float Wp, Wq, Wr;
float Phi, Theta, Psi;
float Phi_bias =0.0;
float Theta_bias = 0.0;
float Psi_bias =0.0;
short xstick_bias = 0.0;
short ystick_bias = 0.0;
short xstick=0;
short ystick=0;
uint8_t button=0;
uint8_t buttonA=0;
uint8_t buttonB=0;
uint8_t buttonB_cnt=0;
uint8_t buttonB_pressed_flag=0;
uint8_t Mode=ANGLECONTROL;
volatile uint8_t Loop_flag = 0;

unsigned long stime,etime,dtime;
byte axp_cnt=0;

char data[140];
uint8_t senddata[15];
uint8_t disp_counter=0;

//Kouhei AtomFlyのMAC ADDRESS E8:9F:6D:06:D3:A0
//B MAC ADDRESS E8:9F:6D:07:B4:84
//4C:75:25:AE:27:FC
const uint8_t addr[6] = {0xE8, 0x9F, 0x6D, 0x06, 0xD3, 0xA0};
//E8:9F:6D:06:D3:A0

void rc_init(void);
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void data_send(void);
void show_battery_info();

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

  //ペアリング
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        Serial.println("Failed to add peer");
        return;
  }
  //esp_wifi_set_channel(8, WIFI_SECOND_CHAN_NONE);

}

#if 0
// 送信コールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //char macStr[18];
  //snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Sent to: ");
  //Serial.println(macStr);
  //Serial.print("Last Packet Send Status: ");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void data_send(void)
{
  //uint8_t data[13] = {72, 101, 108, 108, 111, 32, 69, 83, 80, 45, 78, 79, 87};
  //esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
  //Serial.print("Send Status: ");
  //if (result == ESP_OK) {
  //  Serial.println("Success");
  //} else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
  //  Serial.println("ESPNOW not Init.");
  //} else if (result == ESP_ERR_ESPNOW_ARG) {
  //  Serial.println("Invalid Argument");
  //} else if (result == ESP_ERR_ESPNOW_INTERNAL) {
  //  Serial.println("Internal Error");
  //} else if (result == ESP_ERR_ESPNOW_NO_MEM) {
  //  Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  //} else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
  //  Serial.println("Peer not found.");
  //} else {
  //  Serial.println("Not sure what happened");
  //}  
}
#endif

uint8_t mpu6886_byte_read(uint8_t reg_addr)
{
  uint8_t data;
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.endTransmission();
  Wire1.requestFrom(MPU6886_ADDRESS, 1);
  data = Wire1.read();
  return data;
}

void mpu6886_byte_write(uint8_t reg_addr, uint8_t data)
{
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.write(data);
  Wire1.endTransmission();
}

void imu_init(void)
{
  uint8_t data;
  const uint8_t filter_config = 3;
  //Cutoff frequency
  //filter_config Gyro Accel
  //0 250    218.1
  //1 176    218.1
  //2 92     99.0
  //3 41     44.8
  //4 20     21.2
  //5 10     10.2
  //6 5      5.1
  //7 3281   420.0

  M5.IMU.Init();
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き
  Wire1.begin(21,22,400000UL);

  //Gyro
  //F_CHOICE_B
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  Serial.printf("GYRO_CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_GYRO_CONFIG, data & 0b11111100);
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  Serial.printf("Update GYRO_CONFIG %d\r\n", data);

  //DLPG_CFG
  data = mpu6886_byte_read(MPU6886_CONFIG);
  Serial.printf("CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_CONFIG, (data&0b11111100)|filter_config);
  data = mpu6886_byte_read(MPU6886_CONFIG);
  Serial.printf("Update CONFIG %d\r\n", data);

  //Accel
  //ACCEL_FCHOCE_B & A_DLPF_CFG
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  Serial.printf("ACCEL_CONFIG2 %d\r\n", data);
  mpu6886_byte_write(MPU6886_ACCEL_CONFIG2, (data & 0b11110111) | filter_config);
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  Serial.printf("Update ACCEL_CONFIG2 %d\r\n", data);
}

hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() 
{
  Loop_flag = 1;
}

void setup() {

  M5.begin();
  Serial.begin(115200);
  Wire.begin(0, 26);
  Wire1.begin(21, 22);
  //M5.IMU.Init();
  imu_init();

  Ahrs.begin(100.0);
  rc_init();
  M5.Axp.ScreenBreath(8);
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.println("Gyro calibration...");           

  for(int i=0;i<200;i++)
  {
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    gyroX_bias += gyroX;
    gyroY_bias += gyroY;
    gyroZ_bias += gyroZ;
    delay(10);
  }
  gyroX_bias /= 200;
  gyroY_bias /= 200;
  gyroZ_bias /= 200;

  //Display init
  M5.Lcd.fillScreen(RED);       // 画面全体の塗りつぶし
  //M5.Lcd.setCursor(9, 10);      // カーソル位置の指定
  M5.Lcd.setTextFont(1);        // フォントの指定
  M5.Lcd.setTextSize(2);        // フォントサイズを指定（倍数）
  M5.Lcd.setTextColor(WHITE, RED);
  //M5.Lcd.println("AtomFly2.0");           
  //for (uint8_t i=0;i<50;i++)show_battery_info();

  byte error, address;
  int nDevices;

  Serial.println("Scanning... Wire");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

////////////////////////////////////////////////////////
  Serial.println("Scanning... Wire1");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  //割り込み設定
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
  delay(100);

}



void loop() {
  byte rx_data[5];
  short _xstick,_ystick;
  
  //100Hz
  while(Loop_flag==0);
  Loop_flag=0;

  etime = stime;
  stime = micros();

  M5.update();

  Serial.printf("ok %d\n", dtime);

#if 1
  if(M5.BtnA.isPressed())
  {
    buttonA = 1; 
  }
  else buttonA = 0;

  if(M5.BtnB.isPressed())
  {
    buttonB = 1; 
  }
  else buttonB = 0;

  if (buttonB==1)
  {
    buttonB_cnt++;
    if (buttonB_cnt>10)buttonB_cnt=10;
    buttonB_pressed_flag = 1;
  }
  else
  {
    if (buttonB_pressed_flag == 1)
    {
      Mode+=1;
      if(Mode>3)Mode=0;
      //if (Mode == ANGLECONTROL)Mode = RATECONTROL;
      //else Mode = ANGLECONTROL;
      buttonB_pressed_flag = 0;
    }
    buttonB_cnt = 0;
  }


  Wire.beginTransmission(0x54);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x10);//x軸指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(0x54, 2);  //Request 3 bytes from the slave device.  
  if (Wire.available()) { //If data is received.
    rx_data[0] = Wire.read();
    rx_data[1] = Wire.read();
    _xstick=rx_data[1]*256+rx_data[0];
  }

  Wire.beginTransmission(0x54);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x12);//y軸指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(0x54, 2);  //Request 3 bytes from the slave device.  
  if (Wire.available()) { //If data is received.
    rx_data[2] = Wire.read();
    rx_data[3] = Wire.read();
    _ystick=rx_data[3]*256+rx_data[2];
    
  }

  Wire.beginTransmission(0x54);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x30);//ボタン指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(0x54, 1);  //Request 3 bytes from the slave device.  
  if (Wire.available()) { //If data is received.
    rx_data[4] = Wire.read();
    button=rx_data[4];
  }


////////////////////////////////////////////////////////////

  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  gyroX = gyroX - gyroX_bias;
  gyroY = gyroY - gyroY_bias;
  gyroZ = gyroZ - gyroZ_bias;

  Ahrs.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

  float _theta = Ahrs.getRoll()*DEG_TO_RAD;
  float _phi = Ahrs.getPitch()*DEG_TO_RAD;
  float _psi = Ahrs.getYaw()*DEG_TO_RAD;

  if(button==0)
  {
    Phi_bias = _phi;
    Theta_bias = _theta;
    Psi_bias = _psi;
    xstick_bias = _xstick;
    ystick_bias = _ystick;
  }

  xstick = _xstick - xstick_bias;
  ystick = _ystick - ystick_bias;
  Phi = _phi - Phi_bias;
  Theta = _theta - Theta_bias;
  Psi = _psi - Psi_bias;

  uint8_t* d_int;
  
  d_int = (uint8_t*)&xstick;
  senddata[0]=d_int[0];
  senddata[1]=d_int[1];

  d_int = (uint8_t*)&ystick;
  senddata[2]=d_int[0];
  senddata[3]=d_int[1];

  d_int = (uint8_t*)&Phi;
  senddata[4]=d_int[0];
  senddata[5]=d_int[1];
  senddata[6]=d_int[2];
  senddata[7]=d_int[3];

  d_int = (uint8_t*)&Theta;
  senddata[8]=d_int[0];
  senddata[9]=d_int[1];
  senddata[10]=d_int[2];
  senddata[11]=d_int[3];

  senddata[12]=button;

  senddata[13]=buttonA;

  senddata[14]=Mode;

  esp_err_t result = esp_now_send(peerInfo.peer_addr, senddata, sizeof(senddata));

  //Out put to UART
  //sprintf(data, "x:%4d y:%4d Phi: %7.3f Theta: %7.3f Psi: %7.3f Btn: %2d Delta:%6d\n", 
  //  xstick, ystick, Phi, Theta, Psi, button, dtime);
  //Serial.print(data);


  //Display information
  float vbat = M5.Axp.GetBatVoltage();
  int8_t bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  
  M5.Lcd.setCursor(2, 5+disp_counter*17);
  switch (disp_counter)
  {
    case 0:
      M5.Lcd.printf("AtomFly2.0");
      break;
    case 1:
      M5.Lcd.printf("X:%4d",xstick);
      break;
    case 2:
      M5.Lcd.printf("Y:%4d",ystick);
      break;
    case 3:
      M5.Lcd.printf("Phi:%5.1f",Phi*180/3.14159);
      break;
    case 4:
      M5.Lcd.printf("Tht:%5.1f",Theta*180/3.14159);
      break;
    case 5:
      M5.Lcd.printf("Psi:%5.1f",Psi*180/3.14159);
      break;
    case 6:
      M5.Lcd.printf("FPS:%5.1f",1000000.0/dtime);
      break;
    case 7:
      M5.Lcd.printf("Vlt:%4.1fV", vbat);
      break;
    case 8:
      M5.Lcd.printf("Chg:%3d%%", bat_charge_p);
      break;
    case 9:
      disp_counter++;
      M5.Lcd.setCursor(2, 5+disp_counter*17);
      if( Mode == ANGLECONTROL )            M5.Lcd.printf("-STABILIZE-");
      else if ( Mode == RATECONTROL )       M5.Lcd.printf("-ACRO-     ");
      else if ( Mode == ANGLECONTROL_W_LOG) M5.Lcd.printf("-STABILIZE.L-");
      else if ( Mode == RATECONTROL_W_LOG ) M5.Lcd.printf("-ACRO.L-     ");

  }
  disp_counter++;
  if(disp_counter==11)disp_counter=0;

  //Reset
  if( M5.Axp.GetBtnPress() == 2 ){
    // 電源ボタンクリック
    //M5.Lcd.println("AtomFly2.0"); 
    esp_restart();
  } 
#endif

  //etime = micros();
  
  dtime = stime - etime;
  //etime = stime;
  //if((10000-dtime)>0)delay((10000-dtime)/1000);
}

void show_battery_info(){
  // バッテリー電圧表示
  double vbat = 0.0;
  int8_t bat_charge_p = 0;

  vbat = M5.Axp.GetBatVoltage();
  M5.Lcd.setCursor(5, 100);
  //M5.Lcd.setTextSize(1);
  M5.Lcd.printf("Volt:\n %8.2fV", vbat);

  // バッテリー残量表示
  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  M5.Lcd.setCursor(5, 140);
  M5.Lcd.printf("Charge:\n %8d%%", bat_charge_p);
}
