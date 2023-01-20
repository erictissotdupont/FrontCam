/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"


#include "ArduCAM.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "doorSAP"
#define EXAMPLE_ESP_WIFI_PASS      "password"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static bool bConnected = false;
static int s_retry_num = 0;

static char g_GW_ip_str[16];


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT )
  {
    if( event_id == WIFI_EVENT_STA_START )
    {
      esp_wifi_connect();
    } 
    else if ( event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
      bConnected = false;
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP (Attempt :%d)", s_retry_num );
    }
    else if( event_id == WIFI_EVENT_STA_CONNECTED )
    {
      ESP_LOGI(TAG,"connected to AP");
    }
  }
  else if (event_base == IP_EVENT )
  {
    if( event_id == IP_EVENT_STA_GOT_IP) 
    {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
      ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
      s_retry_num = 0; 
      xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
  }
}

void wifi_init_sta(void)
{
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &event_handler,
                                                      NULL,
                                                      &instance_any_id));
                                                      
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &event_handler,
                                                      NULL,
                                                      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta = {
          .ssid = EXAMPLE_ESP_WIFI_SSID,
          .password = EXAMPLE_ESP_WIFI_PASS,
          /* Setting a password implies station will connect to all security modes including WEP/WPA.
           * However these modes are deprecated and not advisable to be used. Incase your Access point
           * doesn't support WPA2, these mode can be enabled by commenting below line */
     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

          .pmf_cfg = {
              .capable = true,
              .required = false
          },
      },
  };
  
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* The event will not be processed after unregister */
    //ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    //ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    //vEventGroupDelete(s_wifi_event_group);
}

void wifi_wait_to_connect(void)
{
  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
          pdTRUE,
          pdFALSE,
          portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
  if (bits & WIFI_CONNECTED_BIT) 
  {
    tcpip_adapter_ip_info_t info;
    tcpip_adapter_get_ip_info( TCPIP_ADAPTER_IF_STA, &info );  
    sprintf(g_GW_ip_str, IPSTR, IP2STR(&info.gw));
    ESP_LOGI(TAG, "connected to: %s", g_GW_ip_str);
    bConnected = true;
  }
  else if (bits & WIFI_FAIL_BIT) 
  {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
               EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  }
  else
  {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include "esp_netif.h"
//#include "esp_eth.h"

#include <sys/socket.h>

#define CAMERA_PORT      50462
#define INTERCOM_GROUP   "225.0.0.50"
#define HEADER           "MSGCOM2.0"
#define MAX_MSG_SIZE     500

#define MAX_SPI_BURST  1000

#define CAM_ID         1
#define PIXEL_SIZE     2
#define IMG_H_SIZE     320
#define IMG_V_SIZE     240
#define IMG_SIZE       (IMG_H_SIZE*IMG_V_SIZE*PIXEL_SIZE)
#define IMG_BLOB_SIZE  40
#define IMG_ROW_SIZE   (IMG_H_SIZE*IMG_BLOB_SIZE*PIXEL_SIZE)
#define ROW_COUNT      (IMG_V_SIZE/IMG_BLOB_SIZE)
#define COL_COUNT      (IMG_H_SIZE/IMG_BLOB_SIZE)


void send_image( uint8_t image_idx, uint8_t* image, uint32_t size, uint8_t blob )
{
  static int fd = -1;
  struct sockaddr_in addr;
  static char message[MAX_MSG_SIZE];
  
  if( size > 0xFFFF )
  {
    ESP_LOGE(TAG,"Image blob is too big" );
    return;
  }

  if( fd < 0 )
  {
    int broadcastEnable=1;
  
    // create what looks like an ordinary UDP socket
    if ((fd=socket(AF_INET,SOCK_DGRAM,0)) < 0) {
      ESP_LOGE(TAG,"Create socket failed." );
      return;
    }

    if( setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0 ) {
      ESP_LOGE(TAG,"setsockopt failed.");
      return;
    }
  }

  // set up destination address
  memset(&addr,0,sizeof(addr));
  addr.sin_family=AF_INET;
  //addr.sin_addr.s_addr=inet_addr(INTERCOM_GROUP);
  //addr.sin_addr.s_addr=inet_addr("192.168.86.25");
  addr.sin_addr.s_addr=inet_addr(g_GW_ip_str);
  addr.sin_port=htons(CAMERA_PORT);
  
  uint8_t frame = 0;
  uint16_t offset = 0;
  
  message[0] = 'I';
  message[1] = 'M';
  message[2] = 'G';
  message[3] = 0;
  message[4] = IMG_BLOB_SIZE; // 40
  message[5] = COL_COUNT;
  message[6] = ROW_COUNT;
  message[7] = ( image_idx & 0x0F ) | ( CAM_ID << 4 );

  message[8] = blob;
    
  while( size > 0 )
  {
    uint32_t msgLen;
  
    message[9] = (offset & 0xFF00) >> 8;
    message[10] = (offset & 0xFF);  
    message[11] = frame;
    
#define MSG_HDR_SZ 12
    
    if( size > (MAX_MSG_SIZE - MSG_HDR_SZ))
      msgLen = MAX_MSG_SIZE;
    else
      msgLen = size + MSG_HDR_SZ;
    
    memcpy( &message[MSG_HDR_SZ], &image[offset], msgLen - MSG_HDR_SZ );
   
    if (sendto(fd,message,msgLen,0,(struct sockaddr *) &addr, sizeof(addr)) < 0) 
    {
      ESP_LOGE(TAG,"sendTo failed.");
      // This blob is going to be incomplete anyway. this will
      // get retried on the next image captured
      break;
    }
    
    frame++;
    size -= (msgLen - MSG_HDR_SZ);
    offset += (msgLen - MSG_HDR_SZ);

    vTaskDelay( 10 / portTICK_PERIOD_MS );
  }
}

void app_main(void)
{
  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  
  ESP_LOGW(TAG, "Tick = %d", portTICK_PERIOD_MS );
  
  CAM_init(OV2640);
  
  //CAM_set_format( JPEG );
  CAM_set_format( RAW );
  
  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

  wifi_init_sta();  
  
  do
  {
    if( !bConnected )
    {
      wifi_wait_to_connect( );
    }
    
    CAM_resume( );
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    
    if( !CAM_start( ))
    {
      ESP_LOGE(TAG,"Failed to communicate with the camera" );
    }
    else
    {
      CAM_OV2640_set_JPEG_size(OV2640_320x240);
      
      vTaskDelay( 300 / portTICK_PERIOD_MS );

      CAM_set_bit( ARDUCHIP_TIM, MODE_MASK);    
      CAM_flush_fifo( );
      CAM_clear_fifo_flag( );

      vTaskDelay( 30 / portTICK_PERIOD_MS );
      CAM_start_capture( );
      
      ESP_LOGD(TAG,"Timing control register is 0x%02X", CAM_read_reg(ARDUCHIP_TIM));
      
      while( bConnected )
      {  
        if( CAM_get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
        {
          
          CAM_standby( );
          
          uint32_t fifo_len = CAM_read_fifo_length( );
          ESP_LOGD(TAG, "Fifo Len = %d", fifo_len);
          
          //uint8_t* image = (uint8_t*)malloc(fifo_len +4); // +1 is for the SPI dummy byte

  #define PIXEL_SIZE     2
  #define MAX_SPI_BURST  1000
  #define IMG_H_SIZE     320
  #define IMG_V_SIZE     240
  #define IMG_SIZE       (IMG_H_SIZE*IMG_V_SIZE*PIXEL_SIZE)

  #define IMG_BLOB_SIZE      40
  #define IMG_BLOB_BYTE_SIZE (IMG_BLOB_SIZE*IMG_BLOB_SIZE*PIXEL_SIZE)
  #define IMG_ROW_SIZE       (IMG_H_SIZE*IMG_BLOB_SIZE*PIXEL_SIZE)
  #define ROW_COUNT          (IMG_V_SIZE/IMG_BLOB_SIZE)

          if( fifo_len != IMG_SIZE + 8 )
          {
            ESP_LOGW(TAG,"Data in FIFO does not match image size. Fifo = %d. Img = %d",
              fifo_len,
              IMG_SIZE + 8 );
          }

          uint8_t* blob[COL_COUNT];
          for(int i=0; i<COL_COUNT; i++) 
          {
            blob[i] = (uint8_t*)malloc(IMG_BLOB_BYTE_SIZE + 1);
            if( blob[i] == NULL )
            {
              ESP_LOGE(TAG,"Failed to allocate blob %d for %d bytes", i, IMG_BLOB_BYTE_SIZE + 1 );
              // TODO : THIS WILL LEAK
              break;
            }
          }
          
          static uint8_t image_idx = 0;
          image_idx++;
          
          for( int r=0; r<ROW_COUNT; r++)
          {
            for( int line=0; line<IMG_BLOB_SIZE; line++ )
            {
              for( int c=0; c<COL_COUNT; c++ )
              {
                uint8_t* pt = blob[c] + line * (IMG_BLOB_SIZE * PIXEL_SIZE);
                uint8_t dummy = *pt; // remember what is at the dummy byte location
          
                //ESP_LOGI(TAG,"Getting line %d of col %d", line, c );
          
                CAM_set_fifo_burst( pt, (IMG_BLOB_SIZE * PIXEL_SIZE) + 1 );
              
                *pt = dummy; // restore what was overwritten by the dummy SPI byte
              }            
              // ESP_LOGI(TAG,"SPI read %d bytes",read_size-1);
            }
            
            ESP_LOGI( TAG, "Sending img:%d, row:%d", image_idx, r );
            
            for( int c=0; c<COL_COUNT; c++ )
            {           
              send_image( image_idx, blob[c]+1, IMG_BLOB_BYTE_SIZE, r * COL_COUNT + c );
            }
          
            /*
            for( int i=0;i<(IMG_ROW_SIZE/4); i++ )
            {
              //if((i%8)==0) printf( "%08X - ", i*4 );
              
              printf("0x%02X%02X%02X%02X%s",
                row[4*i + 3],
                row[4*i + 2],
                row[4*i + 1],
                row[4*i + 0],
                ((i+1)%8)== 0 ? ",\n" : ",");
                
              if((i%32)==0)
              {
                esp_task_wdt_reset( );
                vTaskDelay( 30 / portTICK_PERIOD_MS );
              }
            }
            printf( "\n" );
            */
          }
          for( int i=0;i<COL_COUNT;i++) free(blob[i]);
          break;        
        }
        else
        {        
          vTaskDelay( 100 / portTICK_PERIOD_MS );
          //ESP_LOGI(TAG, "Fifo Len = %d", CAM_read_fifo_length( ));
        }
      }      
      // ESP_LOGI(TAG, "Fifo size is %d", CAM_read_fifo_length( ));
    }
    
    // ESP_LOGI( TAG, "Sleeping 5sec..." );
    // vTaskDelay( 1000 / portTICK_PERIOD_MS );
    
  } while( 1 );
    
  
  // Does not really go to standby and can't communicate anymore...
  
    
}
