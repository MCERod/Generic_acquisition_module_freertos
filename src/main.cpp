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

#define READINGS_NUM 4

#define MPU6050_SENSOR 1
#define ULTRASSONIC_SENSOR 1
#define ADXL345_SENSOR 0

#if MPU6050_SENSOR
  #include <Adafruit_MPU6050.h>
#endif

#if ADXL345_SENSOR
  #include <Adafruit_ADXL345_U.h>
#endif


uint8_t requesterMAC[6] = {0, 0, 0, 0, 0, 0}; // Initialize with zeros or a default MAC


struct mpu6050_data
{
  float x;
  float y;
  float z;
  float gx;
  float gy;
  float gz;
};




#if ULTRASSONIC_SENSOR
QueueHandle_t ultrassound_queue;
QueueHandle_t ultrassonic_queue_signal_enable;
QueueHandle_t ultrassonic_timestamp_queue;
#endif

#if MPU6050_SENSOR
QueueHandle_t mpu6050_queue;
QueueHandle_t mpu6050_queue_signal_enable;
#endif

#if ADXL345_SENSOR
QueueHandle_t adxl345_queue;
QueueHandle_t adxl345_queue_signal_enable;
#endif

QueueHandle_t imu_processing_queue;
QueueHandle_t timestamp_ref;
QueueHandle_t send_data_queue;

SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t peerMutex;

float calcularMediana(float arr[], int tamanho);
int comparar(const void *a, const void *b);


typedef struct struct_message
{
  volatile float ax = 0;
  volatile float ay = 0;
  volatile float az = 0;
  volatile float gx = 0;
  volatile float gy = 0;
  volatile float gz = 0;
  volatile int distance = 0;
  volatile int tempo = 0;
} struct_message;


  
  esp_now_peer_info_t peerInfo;


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

int comparar(const void *a, const void *b); 
float calcularMediana(float arr[], int tamanho);


void MPU6050_task(void *pvParameters);
void ultrassonic_sensor_task(void *pvParameters);
void Send_data(void *pvParameters);
void ADXL345_task(void *pvParameters);
void IMU_Processing_task(void *pvParameters);
void IMU_Filtering_task(void *pvParameters);




void setup() {
    Serial.begin(115200);
    
    // Initialize WiFi in STA mode
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi Mode set to STA");
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.println("ESP-NOW Initialized");
    
    // Register callback
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Receive Callback Registered");
    
    // Initialize the mutex
    peerMutex = xSemaphoreCreateMutex();
    if (peerMutex == NULL) {
        Serial.println("Mutex creation failed");
    } else {
        Serial.println("Mutex Created");
    }
    
    // Create the send_data_queue with length 1
    send_data_queue = xQueueCreate(1, sizeof(int));
    if (send_data_queue == NULL) {
        Serial.println("Send data queue creation failed");
    } else {
        Serial.println("Send Data Queue Created");
    }
    
    #if ULTRASSONIC_SENSOR
    ultrassonic_timestamp_queue = xQueueCreate(1, sizeof(int));
    ultrassonic_queue_signal_enable = xQueueCreate(1, sizeof(int)); // Length 1
    if (ultrassonic_queue_signal_enable == NULL) {
        Serial.println("Ultrasonic queue creation failed");
    } else {
        Serial.println("Ultrasonic Queue Created");
    }
    #endif
    
    #if MPU6050_SENSOR
    mpu6050_queue_signal_enable = xQueueCreate(1, sizeof(int)); // Length 1
    if (mpu6050_queue_signal_enable == NULL) {
        Serial.println("MPU6050 queue creation failed");
    } else {
        Serial.println("MPU6050 Queue Created");
    }
    #endif
    
    #if ADXL345_SENSOR
    adxl345_queue_signal_enable = xQueueCreate(1, sizeof(int)); // Length 1
    if (adxl345_queue_signal_enable == NULL) {
        Serial.println("ADXL345 queue creation failed");
    } else {
        Serial.println("ADXL345 Queue Created");
    }
    #endif
    
    // Create other queues with length 1
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        Serial.println("I2C mutex creation failed");
        // Handle error
    } else {
        Serial.println("I2C Mutex Created");
    }

    imu_processing_queue = xQueueCreate(1, sizeof(mpu6050_data));
    if (imu_processing_queue == NULL) {
        Serial.println("IMU processing queue creation failed");
    } else {
        Serial.println("IMU Processing Queue Created");
    }
    
    timestamp_ref = xQueueCreate(1, sizeof(unsigned long));
    if (timestamp_ref == NULL) {
        Serial.println("Timestamp reference queue creation failed");
    } else {
        Serial.println("Timestamp Reference Queue Created");
    }
    
    #if MPU6050_SENSOR
    mpu6050_queue = xQueueCreate(1, sizeof(struct mpu6050_data));
    if (mpu6050_queue == NULL) {
        Serial.println("MPU6050 data queue creation failed");
    } else {
        Serial.println("MPU6050 Data Queue Created");
    }
    xTaskCreate(MPU6050_task, "MPU6050_task", 2048, NULL, 4, NULL);
    #endif

    #if ULTRASSONIC_SENSOR
    ultrassound_queue = xQueueCreate(1, sizeof(int)); // Length 1
    if (ultrassound_queue == NULL) {
        Serial.println("Ultrasonic queue creation failed");
    } else {
        Serial.println("Ultrasonic Queue Created");
    }
    xTaskCreate(ultrassonic_sensor_task, "ultrassonic_sensor_task", 2048, NULL, 4, NULL);
    #endif
    
    #if ADXL345_SENSOR
    adxl345_queue = xQueueCreate(1, sizeof(struct mpu6050_data)); // Ensure correct data type and length
    if (adxl345_queue == NULL) {
        Serial.println("ADXL345 queue creation failed");
    } else {
        Serial.println("ADXL345 Queue Created");
    }
    xTaskCreate(ADXL345_task, "ADXL345_task", 2048, NULL, 4, NULL);
    #endif
    
    // Add peer
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, requesterMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    } else {
        Serial.println("Peer Added Successfully");
    }
    xTaskCreate(Send_data, "Send_data", 2048, NULL, 2, NULL);
    xTaskCreate(IMU_Filtering_task, "IMU_FILETRING", 2048, NULL, 3, NULL);
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
      Serial.println("Failed to find MPU6050 chip");
      vTaskDelay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  xSemaphoreGive(i2c_mutex);
  while (1)
  {
   // xSemaphoreTake(mpu6050_mutex,portMAX_DELAY);
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    mpu.getEvent(&a, &g, &temp);
    xSemaphoreGive(i2c_mutex);
    data.x = a.acceleration.x;
    data.y = a.acceleration.y;
    data.z = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    xQueueOverwrite(mpu6050_queue, &data);
    xSemaphoreGive(i2c_mutex);
    vTaskDelay(10);
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
  int timestamp = 0;
  TickType_t xLastWakeTime;
  while (1)
  {
    unsigned long prev_time = millis();
xLastWakeTime = xTaskGetTickCount();
xQueueReceive(ultrassonic_queue_signal_enable, &status, pdMS_TO_TICKS(0));
//Serial.println(millis() - timestamp);
timestamp = millis();
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
    xQueueOverwrite(ultrassonic_timestamp_queue, &timestamp);
    xQueueOverwrite(ultrassound_queue, &distance);
    
}
    //vTaskDelay(150);

    vTaskDelayUntil(&xLastWakeTime, 100);
    
    //Serial.println(millis() - prev_time);
  }
#else
  vTaskDelete(NULL);
#endif
}

void Send_data(void *pvParameters)
{
  int timestamp = 0;

  int status = 0;
  struct_message sensor_data[READINGS_NUM];
  Serial.begin(9600);
  struct mpu6050_data data;
  int distance = 0;
  int counter = 0;
  unsigned long _timestamp = 0;
  int time = 0;
  while (1)
  {
   // xQueueReceive(timestamp_ref, &timestamp, pdMS_TO_TICKS(0));
    #if ULTRASSONIC_SENSOR
    if(xQueueReceive(ultrassound_queue, &distance, portMAX_DELAY) == pdTRUE)
    {
     // Serial.print(distance);
      //Serial.print("\t"); 
      sensor_data[counter].distance = distance;
    }
    
#endif
#if MPU6050_SENSOR
    xQueueReceive(imu_processing_queue, &data, portMAX_DELAY);
    

    sensor_data[counter].ax = data.x;
    sensor_data[counter].ay = data.y;
    sensor_data[counter].az = data.z;
    sensor_data[counter].gx = data.gx;
    sensor_data[counter].gy = data.gy;
    sensor_data[counter].gz = data.gz;
    
#endif
#if ADXL345_SENSOR
  xQueueReceive(adxl345_queue, &data, portMAX_DELAY);
    
    Serial.print(data.x);
    Serial.print("\t");
    Serial.print(data.y);
    Serial.print("\t");
    Serial.println(data.z);
    sensor_data[counter].ax = data.x;
    sensor_data[counter].ay = data.y;
    sensor_data[counter].az = data.z;
#endif
  xQueueReceive(ultrassonic_timestamp_queue, &timestamp, portMAX_DELAY);
  //Serial.println(timestamp - _timestamp);
  _timestamp = timestamp;
  Serial.println(timestamp);
  sensor_data[counter].tempo = timestamp;


  xQueueReceive(send_data_queue, &status, pdMS_TO_TICKS(0));
  
  if(counter >= READINGS_NUM - 1 && status == 1){
  //  Serial.println("Almost ready to send");
   if(xSemaphoreTake(peerMutex, pdMS_TO_TICKS(0)) == pdTRUE){
        esp_err_t result = esp_now_send(requesterMAC, (uint8_t *)&sensor_data, sizeof(sensor_data));
        xSemaphoreGive(peerMutex);
        
        if (result != ESP_OK) {
            Serial.print("Send failed with error code: ");
            Serial.println(result);
            
            switch (result) {
                case ESP_ERR_ESPNOW_NOT_INIT:
                    Serial.println("Error: ESP-NOW not initialized.");
                    break;
                case ESP_ERR_ESPNOW_ARG:
                    Serial.println("Error: Invalid argument.");
                    break;
                case ESP_ERR_ESPNOW_INTERNAL:
                    Serial.println("Error: Internal error.");
                    break;
                case ESP_ERR_ESPNOW_NO_MEM:
                    Serial.println("Error: No memory available.");
                    break;
                case ESP_ERR_ESPNOW_NOT_FOUND:
                    Serial.println("Error: Peer not found.");
                    break;
                default:
                    Serial.println("Error: Unknown ESP-NOW error.");
                    break;
            }
        }else{
         //   Serial.println("Data sent");
        }
    } else {
      //  Serial.println("Failed to acquire mutex");
    }
  }
    if(counter < READINGS_NUM - 1){
      counter++;
    } else {
      counter = 0;
    }

    vTaskDelay(1);
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
    vTaskDelay(10);
    }
  }
#else
  vTaskDelete(NULL);
#endif
}



void IMU_Processing_task(void *pvParameters){

}



void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
    int status = 0;
    
   
    
    // Validate incomingData length
    if (len < sizeof(int)) {
        Serial.println("Error: Incoming data is too short.");
        return;
    }
    
    memcpy(requesterMAC, mac_addr, 6); // Store the requester's MAC address
    
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, requesterMAC, 6);
    peerInfo.channel = 0; // Default channel
    peerInfo.encrypt = false;
    
    memcpy(&status, incomingData, sizeof(int));
    
    // Add the peer if it doesn't exist
    if (!esp_now_is_peer_exist(requesterMAC)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add peer");
            return; // Avoid using while(1);
        } else {
            Serial.println("Peer added successfully within callback");
        }
    }
    
   // Serial.print("Status received: ");
   // Serial.println(status);
    
    // Overwrite send_data_queue with error checking
    if (xQueueOverwrite(send_data_queue, &status) != pdTRUE) {
        Serial.println("Failed to overwrite send_data_queue");
    }
    
    #if ULTRASSONIC_SENSOR
    if (xQueueOverwrite(ultrassonic_queue_signal_enable, &status) != pdTRUE) {
        Serial.println("Failed to overwrite ultrassonic_queue_signal_enable");
    }
    #endif
    
    #if MPU6050_SENSOR
    if (xQueueOverwrite(mpu6050_queue_signal_enable, &status) != pdTRUE) {
        Serial.println("Failed to overwrite mpu6050_queue_signal_enable");
    }
    #endif
    
    #if ADXL345_SENSOR
    if (xQueueOverwrite(adxl345_queue_signal_enable, &status) != pdTRUE) {
        Serial.println("Failed to overwrite adxl345_queue_signal_enable");
    }
    #endif
    //unsigned long time = millis();
    //xQueueOverwrite(timestamp_ref, &time);
   // Serial.println(time);
    
}


void IMU_Filtering_task(void *pvParameters){
    struct mpu6050_data data[READINGS_NUM];
    struct mpu6050_data received_data;

    
    int pointer = 0;
    while(1){
        
        if(pointer >= READINGS_NUM-1){
            pointer = 0;
        }
        xQueueReceive(mpu6050_queue, &received_data, portMAX_DELAY);
        data[pointer] = received_data;
        pointer++;
        float x_values[READINGS_NUM];
        float y_values[READINGS_NUM];
        float z_values[READINGS_NUM];
        float gx_values[READINGS_NUM];
        float gy_values[READINGS_NUM];
        float gz_values[READINGS_NUM];

        for (int i = 0; i < READINGS_NUM; i++) {
            x_values[i] = data[i].x;
            y_values[i] = data[i].y;
            z_values[i] = data[i].z;
            gx_values[i] = data[i].gx;
            gy_values[i] = data[i].gy;
            gz_values[i] = data[i].gz;
        }

        float median_x = calcularMediana(x_values, READINGS_NUM);
        float median_y = calcularMediana(y_values, READINGS_NUM);
        float median_z = calcularMediana(z_values, READINGS_NUM);
        float median_gx = calcularMediana(gx_values, READINGS_NUM);
        float median_gy = calcularMediana(gy_values, READINGS_NUM);
        float median_gz = calcularMediana(gz_values, READINGS_NUM);

        struct mpu6050_data filtered_data;
        filtered_data.x = median_y;
        filtered_data.y = median_y;
        filtered_data.z = median_z;
        filtered_data.gx = median_gx;
        filtered_data.gy = median_gy;
        filtered_data.gz = median_gz;

        xQueueOverwrite(imu_processing_queue, &filtered_data);
        vTaskDelay(10);
    }
}

int comparar(const void *a, const void *b) {
    if (*(float*)a < *(float*)b) return -1;
    if (*(float*)a > *(float*)b) return 1;
    return 0;
}

float calcularMediana(float arr[], int tamanho) {
    // Ordena o array
    qsort(arr, tamanho, sizeof(float), comparar);

    // Calcula a mediana
    if (tamanho % 2 == 0) {
        int meio1 = tamanho / 2 - 1;
        int meio2 = tamanho / 2;
        return (arr[meio1] + arr[meio2]) / 2.0;
    } else {
        return arr[tamanho / 2];
    }
}
