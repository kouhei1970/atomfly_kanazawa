// #include <Ps3Controller.h>
#include "rc_sbus.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/uart.h>

//esp_now_peer_info_t slave;


int player = 0;
int battery = 0;
float rctime=0.0;
volatile uint8_t Connect_flag = 0;

//Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
const uint8_t addr[6] = {0x4C, 0x75, 0x25, 0xAD, 0xB6, 0x6C};

esp_now_peer_info_t peerInfo;

//RC Stick Init
volatile float Stick[16];

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle UART interrupt.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

const int uart_buffer_size = (BUF_SIZE * 2);
static QueueHandle_t uart_queue;

// Both definition are same and valid
//static uart_isr_handle_t *handle_console;
static intr_handle_t handle_console;



// Receive buffer to collect incoming data
uint8_t rxbuf[256];
// Register to collect data length
uint16_t urxlen;

// 受信コールバック
static void IRAM_ATTR uart_intr_handle(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t i;
  Serial.printf("Receive!\n");
  #if 0
  status = UART0.int_st.val; // read UART interrupt Status
  rx_fifo_len = UART0.status.rxfifo_cnt; // read number of bytes in UART buffer
  
  while(rx_fifo_len){
   rxbuf[i++] = UART0.fifo.rw_byte; // read all bytes
   rx_fifo_len--;
 }
 #endif
}




void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) 
{


  Stick[RUDDER]=0;
  Stick[THROTTLE]=0;
  Stick[AILERON]  = 0;
  Stick[ELEVATOR]  = 0;
  Stick[BUTTON] = 0;
  Stick[BUTTON_A] = 0;
  Stick[CONTROLMODE] = 0;  
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
  esp_now_init();

  //Initialize Stick list 
  for (uint8_t i = 0;i<16;i++)Stick[i]=0.0;

  /* Configure parameters of an UART driver,
	* communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
  // Configure UART parameters
  uart_param_config(UART_NUM_1, &uart_config);
    //Set pin
  //uart_set_pin(UART_NUM_1, 32, 26, -1, -1);
  // Install UART driver using an event queue here
  //uart_driver_install(UART_NUM_1, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0);
  //uart_isr_free(UART_NUM_1);
  //uart_isr_register(UART_NUM_1,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);
  //uart_enable_rx_intr(UART_NUM_1);

}

void telemetry_init(void)
{

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
  
  //telemetry 相手とペアリング
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 5;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        Serial.println("Failed to add peer");
        return;
  }

  // ESP-NOWコールバック登録
  //esp_now_register_recv_cb(OnDataRecv);
  //Serial.println("ESP-NOW Ready.");
  //Serial.println("Wait Contoroller ready....");
  //while(Connect_flag==0);
  //Serial.println("Contoroller ready !");
  esp_wifi_set_channel(5, WIFI_SECOND_CHAN_NONE);

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
}

bool rc_isconnected(void)
{
    return 1;
}

void rc_demo()
{
}












