#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>


#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>

#define READINGS_NUM 6

#define MPU6050_SENSOR 0
#define ULTRASSONIC_SENSOR 1
#define ADXL345_SENSOR 1

#if MPU6050_SENSOR
  #include <Adafruit_MPU6050.h>
#endif

#if ADXL345_SENSOR
  #include <Adafruit_ADXL345_U.h>
#endif


uint8_t requesterMAC[6];


struct mpu6050_data
{
  float x;
  float y;
  float z;
};




#if ULTRASSONIC_SENSOR
QueueHandle_t ultrassound_queue;
QueueHandle_t ultrassonic_queue_signal_enable;
#endif

#if MPU6050_SENSOR
QueueHandle_t mpu6050_queue;
QueueHandle_t mpu6050_queue_signal_enable;
#endif

#if ADXL345_SENSOR
QueueHandle_t adxl345_queue;
QueueHandle_t adxl345_queue_signal_enable;
#endif

SemaphoreHandle_t i2c_mutex;




typedef struct struct_message
{
  float ax = 0;
  float ay = 0;
  float az = 0;
  float gx = 0;
  float gy = 0;
  float gz = 0;
  int distance = 0;
  int tempo = 0;
} struct_message;


  
  esp_now_peer_info_t peerInfo;


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);


void MPU6050_task(void *pvParameters);
void ultrassonic_sensor_task(void *pvParameters);
void Send_data(void *pvParameters);
void ADXL345_task(void *pvParameters);

void setup()
{
   WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    while (1)
    {
      Serial.println("Error initializing ESP-NOW");
    }
    
    return;
  }
  Serial.println("ESP-NOW initialized");
  esp_now_register_recv_cb(OnDataRecv);


  i2c_mutex = xSemaphoreCreateMutex();
    



  #if MPU6050_SENSOR
  mpu6050_queue_signal_enable = xQueueCreate(1,sizeof(int));
  mpu6050_queue = xQueueCreate(1, sizeof(struct mpu6050_data));
  xTaskCreate(MPU6050_task, "MPU6050_task", 2048, NULL, 2, NULL);
  #endif

  #if ULTRASSONIC_SENSOR
  ultrassonic_queue_signal_enable = xQueueCreate(1,sizeof(int));
  ultrassound_queue = xQueueCreate(1, sizeof(int));
  xTaskCreate(ultrassonic_sensor_task, "ultrassonic_sensor_task", 2048, NULL, 3, NULL);
  #endif
#if ADXL345_SENSOR
  adxl345_queue_signal_enable = xQueueCreate(1,sizeof(int));
  adxl345_queue = xQueueCreate(1, sizeof(struct mpu6050_data));
  xTaskCreate(ADXL345_task, "ADXL345_task", 2048, NULL, 2, NULL);
#endif

  xTaskCreate(Send_data, "Send_data", 2048, NULL, 1, NULL);

  // Initialize mutexes
  



}

void loop()
{
  vTaskDelete(NULL);
}

void MPU6050_task(void *pvParameters)
{
#if MPU6050_SENSOR
xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  Adafruit_MPU6050 mpu;
  struct mpu6050_data data;
  sensors_event_t a, g, temp;
  if (!mpu.begin())
  {
    while (1)
    {
      xSemaphoreGive(i2c_mutex);
      vTaskDelay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  xSemaphoreGive(i2c_mutex);
  while (1)
  {
    xSemaphoreTake(mpu6050_mutex,portMAX_DELAY);
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    mpu.getEvent(&a, &g, &temp);
    xSemaphoreGive(i2c_mutex);
    data.x = a.acceleration.x;
    data.y = a.acceleration.y;
    data.z = a.acceleration.z;
    xQueueOverwrite(mpu6050_queue, &data);
    xSemaphoreGive(mpu6050_mutex);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
#else
  vTaskDelete(NULL);
#endif
}

void ultrassonic_sensor_task(void *pvParameters)
{
#if ULTRASSONIC_SENSOR
  SoftwareSerial mySerial(4, 5);
  mySerial.begin(9600);
  unsigned char data[4];
  int distance = 0;
  int status = 0;
  while (1)
  {
xQueueReceive(ultrassonic_queue_signal_enable, &status, pdMS_TO_TICKS(0));
if(status == 1){
    do
    {
      for (int i = 0; i < 4; i++)
      {
        data[i] = mySerial.read();
      }
    } while (mySerial.read() == 0xff);
    mySerial.flush();
    if (data[0] == 0xff)
    {
      int sum;
      sum = (data[0] + data[1] + data[2]) & 0x00FF;
      if (sum == data[3])
      {
        distance = (data[1] << 8) + data[2];
        if (distance > 280)
        {
          distance = distance / 10;
        }
        else
        {
          distance = -1;
        }
      }
    }

    xQueueOverwrite(ultrassound_queue, &distance);
    vTaskDelay(150);
}
    vTaskDelay(pdMS_TO_TICKS(150));
  }
#else
  vTaskDelete(NULL);
#endif
}

void Send_data(void *pvParameters)
{
  struct_message sensor_data[READINGS_NUM];
  Serial.begin(9600);
  struct mpu6050_data data;
  int distance = 0;
  while (1)
  {
    #if ULTRASSONIC_SENSOR
    if(xQueueReceive(ultrassound_queue, &distance, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      Serial.print(distance);
      Serial.print("\t"); 
    }
    
#endif
#if MPU6050_SENSOR
    xQueueReceive(mpu6050_queue, &data, portMAX_DELAY);
    
    Serial.print(data.x);
    Serial.print("\t");
    Serial.print(data.y);
    Serial.print("\t");
    Serial.println(data.z);
#endif
#if ADXL345_SENSOR
  xQueueReceive(adxl345_queue, &data, portMAX_DELAY);
    
    Serial.print(data.x);
    Serial.print("\t");
    Serial.print(data.y);
    Serial.print("\t");
    Serial.println(data.z);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
}



void ADXL345_task(void *pvParameters)
{
  
#if ADXL345_SENSOR
  sensors_event_t event;
  struct  mpu6050_data data;
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
  
  if (!accel.begin())
  {
    while (1)
    {
      xSemaphoreGive(i2c_mutex);
      vTaskDelay(1000);
    }
  }

  accel.setRange(ADXL345_RANGE_16_G);
  xSemaphoreGive(i2c_mutex);
  int status = 0;
  while (1)
  {
    xQueueReceive(adxl345_queue_signal_enable, &status, pdMS_TO_TICKS(0));
    if(status == 1){
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    accel.getEvent(&event);
    xSemaphoreGive(i2c_mutex);
    data.x = event.acceleration.x;
    data.y = event.acceleration.y;
    data.z = event.acceleration.z;
    xQueueOverwrite(adxl345_queue, &data);
    vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
#else
  vTaskDelete(NULL);
#endif
}



void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
  int status = 0;
  memcpy(requesterMAC, mac_addr, 6); // Store the requester's MAC address
   
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, requesterMAC, 6);
  peerInfo.channel = 0; // Canal padrão (ajuste se necessário)
  peerInfo.encrypt = false;
  memcpy(&status, incomingData,sizeof(int));
  // Adiciona o p0,se ainda não foi adicionado
  if (!esp_now_is_peer_exist(requesterMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }
  #if ULTRASSONIC_SENSOR
  xQueueOverwrite(ultrassonic_queue_signal_enable, &status);
  #endif
  #if MPU6050_SENSOR
  xQueueOverwrite(mpu6050_queue_signal_enable, &status);
  #endif
  #if ADXL345_SENSOR
  xQueueOverwrite(adxl345_queue_signal_enable, &status);
  #endif
}