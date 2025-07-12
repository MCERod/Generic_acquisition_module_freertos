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

#define MPU6050_SENSOR 0
#define ULTRASSONIC_SENSOR 1
#define ADXL345_SENSOR 0
#define MPU9250_SENSOR 1
#define VL53L0X_SENSOR 1
#define TEMPERATURE_SENSOR 0


#if MPU6050_SENSOR
  #include <Adafruit_MPU6050.h>
#endif

#if ADXL345_SENSOR
  #include <Adafruit_ADXL345_U.h>
#endif

#if MPU9250_SENSOR
  #include <MPU9250.h>
#endif

#if VL53L0X_SENSOR
  #include <Adafruit_VL53L0X.h>
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

struct mpu9250_data
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
};

#if ULTRASSONIC_SENSOR
QueueHandle_t ultrassound_queue;
QueueHandle_t ultrassonic_queue_signal_enable;
QueueHandle_t ultrassonic_timestamp_queue;
#endif

#if VL53L0X_SENSOR
QueueHandle_t vl53l0x_queue;
QueueHandle_t vl53l0x_queue_signal_enable;
QueueHandle_t vl53l0x_timestamp_queue;
#endif

QueueHandle_t mpu6050_queue;
#if MPU6050_SENSOR
QueueHandle_t mpu6050_queue_signal_enable;
#endif

#if ADXL345_SENSOR
QueueHandle_t adxl345_queue;
QueueHandle_t adxl345_queue_signal_enable;
#endif

#if MPU9250_SENSOR
QueueHandle_t mpu9250_queue;
QueueHandle_t mpu9250_queue_signal_enable;
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
  //volatile float mx = 0;  // Added for magnetometer
  //volatile float my = 0;  // Added for magnetometer
  //volatile float mz = 0;  // Added for magnetometer
  volatile int distance = 0;
  volatile int tempo = 0;
} struct_message;

esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

int comparar(const void *a, const void *b); 
float calcularMediana(float arr[], int tamanho);

void MPU6050_task(void *pvParameters);
void MPU9250_task(void *pvParameters); // New task function
void ultrassonic_sensor_task(void *pvParameters);
void VL53L0X_task(void *pvParameters);
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
    ultrassonic_queue_signal_enable = xQueueCreate(1, sizeof(int)); 
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
    
    #if MPU9250_SENSOR
    mpu9250_queue_signal_enable = xQueueCreate(1, sizeof(int)); // Length 1
    if (mpu9250_queue_signal_enable == NULL) {
        Serial.println("MPU9250 queue creation failed");
    } else {
        Serial.println("MPU9250 Queue Created");
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

    imu_processing_queue = xQueueCreate(1, sizeof(mpu9250_data));
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

    #if MPU9250_SENSOR
    mpu9250_queue = xQueueCreate(1, sizeof(struct mpu9250_data));
    if (mpu9250_queue == NULL) {
        Serial.println("MPU9250 data queue creation failed");
    } else {
        Serial.println("MPU9250 Data Queue Created");
    }
    xTaskCreate(MPU9250_task, "MPU9250_task", 2048, NULL, 4, NULL);
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

    #if VL53L0X_SENSOR
    vl53l0x_queue = xQueueCreate(1, sizeof(int)); // Length 1
    if (vl53l0x_queue == NULL) {
        Serial.println("VL53L0X queue creation failed");
    } else {
        Serial.println("VL53L0X Queue Created");
    }
    xTaskCreate(VL53L0X_task, "VL53L0X_task", 2048, NULL, 4, NULL);
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
  Serial.begin(9600);
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
    Serial.println("MPU6050");
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


void VL53L0X_task(void *pvParameters)
{
#if VL53L0X_SENSOR
  Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
  VL53L0X_RangingMeasurementData_t measure;
  int distance = 0;
  int status = 0;
  TickType_t xLastWakeTime;
  
  #if TEMPERATURE_SENSOR
  // Define analog temperature sensor pin
  #define TEMP_SENSOR_PIN 34  // Adjust to the ADC pin you're using
  #define NUM_TEMP_SAMPLES 10 // Number of samples for median filtering
  
  // Configure ADC for temperature reading
  analogReadResolution(12);  // ESP32 uses 12-bit resolution
  analogSetAttenuation(ADC_11db);  // Full range
  
  // Take multiple temperature readings and apply median filter
  float tempReadings[NUM_TEMP_SAMPLES];
  
  // Collect temperature samples
  for (int i = 0; i < NUM_TEMP_SAMPLES; i++) {
    // Read analog value
    int rawValue = analogRead(TEMP_SENSOR_PIN);
    
    // Convert to voltage (for ESP32 with 3.3V reference)
    float voltage = rawValue * (3.3 / 4095.0);
    
    // Convert to temperature (adjust formula based on your specific sensor)
    // Example for TMP36: temp(°C) = (voltage - 0.5) * 100
    tempReadings[i] = (voltage - 0.5) * 100.0;
    
    // Small delay between readings
    vTaskDelay(5);
  }
  
  // Apply median filter to temperature readings
  float temperature = calcularMediana(tempReadings, NUM_TEMP_SAMPLES);
  
  Serial.print("Calibration temperature (median of 10 samples): ");
  Serial.print(temperature);
  Serial.println(" °C");
  #endif
  
  // Initialize sensor once at startup
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  if (!sensor.begin()) {
    xSemaphoreGive(i2c_mutex);
    Serial.println("Failed to initialize VL53L0X");
    while(1) {
      vTaskDelay(1000);
    }
  }
  
  // Optimized configuration for 2cm to 1.2m range
  sensor.setDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING);
  
  // Adjust timing budget for best balance of speed and accuracy
  sensor.setMeasurementTimingBudgetMicroSeconds(100000); // 100ms
  
  // Set signal rate limit correctly
  sensor.setLimitCheckEnable(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
  
  // Optimize for short-to-medium range and higher accuracy
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  sensor.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  
  // Minimum count rate for valid measurement - helps with very short range (2cm)
  sensor.setLimitCheckEnable(VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
  sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5 * 65536));
  
  #if TEMPERATURE_SENSOR
  // Apply temperature compensation based on the filtered temperature reading
  
  // Temperature calibration for VL53L0X
  // The VL53L0X is factory calibrated at 23°C
  // We can make minor adjustments to timing and thresholds based on temperature
  
  if (temperature < 10 || temperature > 40) {
    // For extreme temperatures, adjust signal rate limit
    float tempCompFactor = 0.25;
    if (temperature < 10) {
      // When cold, lower the signal threshold to compensate for reduced IR signal
      tempCompFactor = 0.20; // 20% lower threshold when cold
    } else if (temperature > 40) {
      // When hot, increase threshold to filter more noise
      tempCompFactor = 0.30; // 20% higher threshold when hot
    }
    
    sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 
                             (FixPoint1616_t)(tempCompFactor * 65536));
    
    Serial.println("Applied temperature compensation to VL53L0X");
  }
  
  // Perform reference calibration with temperature compensation
  sensor.setRefCalibration();
  #endif
  
  xSemaphoreGive(i2c_mutex);
  
  Serial.println("VL53L0X initialized in high accuracy mode");
  xLastWakeTime = xTaskGetTickCount();
  
  while (1)
  {
    xQueueReceive(vl53l0x_queue_signal_enable, &status, pdMS_TO_TICKS(0));
    if(status == 1){
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      sensor.rangingTest(&measure, false);
      xSemaphoreGive(i2c_mutex);
      
      if (measure.RangeStatus != 4)
      {
        distance = measure.RangeMilliMeter;
        Serial.print("Distance (mm): ");
        Serial.println(distance);
      }
      else
      {
        distance = -1;
        Serial.println("Out of range");
      }
      
      int timestamp = millis();
      xQueueOverwrite(vl53l0x_timestamp_queue, &timestamp);
      xQueueOverwrite(vl53l0x_queue, &distance);
    }
    
    vTaskDelayUntil(&xLastWakeTime, 100); // Match the timing budget
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
  
  #if MPU6050_SENSOR
  struct mpu6050_data data_mpu6050;
  #endif
  
  #if MPU9250_SENSOR
  struct mpu9250_data data_mpu9250;
  #endif
  
  int distance = 0;
  int counter = 0;
  unsigned long _timestamp = 0;
  int time = 0;
  
  while (1)
  {
    #if ULTRASSONIC_SENSOR
    if(xQueueReceive(ultrassound_queue, &distance, portMAX_DELAY) == pdTRUE)
    {
      sensor_data[counter].distance = distance;
    }
    #endif
    
    #if MPU6050_SENSOR
    xQueueReceive(imu_processing_queue, &data_mpu6050, portMAX_DELAY);
    
    sensor_data[counter].ax = data_mpu6050.x;
    sensor_data[counter].ay = data_mpu6050.y;
    sensor_data[counter].az = data_mpu6050.z;
    sensor_data[counter].gx = data_mpu6050.gx;
    sensor_data[counter].gy = data_mpu6050.gy;
    sensor_data[counter].gz = data_mpu6050.gz;
    #endif
    
    #if MPU9250_SENSOR
    xQueueReceive(mpu9250_queue, &data_mpu9250, portMAX_DELAY);
    
    sensor_data[counter].ax = data_mpu9250.ax;
    sensor_data[counter].ay = data_mpu9250.ay;
    sensor_data[counter].az = data_mpu9250.az;
    sensor_data[counter].gx = data_mpu9250.gx;
    sensor_data[counter].gy = data_mpu9250.gy;
    sensor_data[counter].gz = data_mpu9250.gz;
    //sensor_data[counter].mx = data_mpu9250.mx;
    //sensor_data[counter].my = data_mpu9250.my;
    //sensor_data[counter].mz = data_mpu9250.mz;
    #endif
    
    #if ADXL345_SENSOR
    xQueueReceive(adxl345_queue, &data_mpu6050, portMAX_DELAY);
    
    sensor_data[counter].ax = data_mpu6050.x;
    sensor_data[counter].ay = data_mpu6050.y;
    sensor_data[counter].az = data_mpu6050.z;
    #endif
    
    xQueueReceive(ultrassonic_timestamp_queue, &timestamp, portMAX_DELAY);
    _timestamp = timestamp;
    sensor_data[counter].tempo = timestamp;
    
    #if VL53L0X_SENSOR
    int vl53l0x_distance = 0;
    if(xQueueReceive(vl53l0x_queue, &vl53l0x_distance, pdMS_TO_TICKS(0)) == pdTRUE)
    {
        sensor_data[counter].distance = vl53l0x_distance;
    }

    // If you want to use the VL53L0X timestamps:
    int vl53l0x_timestamp = 0;
    if(xQueueReceive(vl53l0x_timestamp_queue, &vl53l0x_timestamp, pdMS_TO_TICKS(0)) == pdTRUE)
    {
        _timestamp = vl53l0x_timestamp;
        sensor_data[counter].tempo = vl53l0x_timestamp;
    }
    #endif
    
    // Check if we need to send data
    xQueueReceive(send_data_queue, &status, pdMS_TO_TICKS(0));
    
    if(counter >= READINGS_NUM - 1 && status == 1){
      if(xSemaphoreTake(peerMutex, pdMS_TO_TICKS(0)) == pdTRUE){
        esp_err_t result = esp_now_send(requesterMAC, (uint8_t *)&sensor_data, sizeof(sensor_data));
        xSemaphoreGive(peerMutex);
        
        if (result != ESP_OK) {
          Serial.print("ESP-NOW send failed with error code: ");
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
        } else {
          Serial.println("Data sent successfully via ESP-NOW");
        }
      } else {
        Serial.println("Could not take peerMutex for sending");
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

    #if MPU9250_SENSOR
    if (xQueueOverwrite(mpu9250_queue_signal_enable, &status) != pdTRUE) {
        Serial.println("Failed to overwrite mpu9250_queue_signal_enable");
    }
    #endif

    #if VL53L0X_SENSOR
    if (xQueueOverwrite(vl53l0x_queue_signal_enable, &status) != pdTRUE) {
        Serial.println("Failed to overwrite vl53l0x_queue_signal_enable");
    }
    #endif
    //unsigned long time = millis();
    //xQueueOverwrite(timestamp_ref, &time);
   // Serial.println(status);
    
}

void IMU_Filtering_task(void *pvParameters){
  #if MPU9250_SENSOR
    struct mpu9250_data data[READINGS_NUM];
    struct mpu9250_data received_data;
    
    int pointer = 0;
    while(1){
        if(pointer >= READINGS_NUM-1){
            pointer = 0;
        }
        
        if(xQueueReceive(mpu9250_queue, &received_data, portMAX_DELAY) == pdTRUE) {
            data[pointer] = received_data;
            pointer++;
            
            float ax_values[READINGS_NUM];
            float ay_values[READINGS_NUM];
            float az_values[READINGS_NUM];
            float gx_values[READINGS_NUM];
            float gy_values[READINGS_NUM];
            float gz_values[READINGS_NUM];
            float mx_values[READINGS_NUM];
            float my_values[READINGS_NUM];
            float mz_values[READINGS_NUM];

            for (int i = 0; i < READINGS_NUM; i++) {
                ax_values[i] = data[i].ax;
                ay_values[i] = data[i].ay;
                az_values[i] = data[i].az;
                gx_values[i] = data[i].gx;
                gy_values[i] = data[i].gy;
                gz_values[i] = data[i].gz;
                mx_values[i] = data[i].mx;
                my_values[i] = data[i].my;
                mz_values[i] = data[i].mz;
            }

            float median_ax = calcularMediana(ax_values, READINGS_NUM);
            float median_ay = calcularMediana(ay_values, READINGS_NUM);
            float median_az = calcularMediana(az_values, READINGS_NUM);
            float median_gx = calcularMediana(gx_values, READINGS_NUM);
            float median_gy = calcularMediana(gy_values, READINGS_NUM);
            float median_gz = calcularMediana(gz_values, READINGS_NUM);
            float median_mx = calcularMediana(mx_values, READINGS_NUM);
            float median_my = calcularMediana(my_values, READINGS_NUM);
            float median_mz = calcularMediana(mz_values, READINGS_NUM);

            struct mpu9250_data filtered_data;
            filtered_data.ax = median_ax;
            filtered_data.ay = median_ay;
            filtered_data.az = median_az;
            filtered_data.gx = median_gx;
            filtered_data.gy = median_gy;
            filtered_data.gz = median_gz;
            filtered_data.mx = median_mx;
            filtered_data.my = median_my;
            filtered_data.mz = median_mz;

            xQueueOverwrite(imu_processing_queue, &filtered_data);
        }
        
        vTaskDelay(10);
    }
  #else
    // If no sensor is enabled, just have the task stay alive
    while(1) {
        vTaskDelay(1000);
    }
  #endif
}

int comparar(const void *a, const void *b) {
    if (*(float*)a < *(float*)b) return -1;
    if (*(float*)a > *(float*)b) return 0;
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

// Task to read from MPU9250
void MPU9250_task(void *pvParameters)
{
#if MPU9250_SENSOR
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  MPU9250 mpu;
  
  Wire.begin();
  
  if (!mpu.setup(0x68)) {  // Change to your MPU9250 I2C address
    while (1) {
      xSemaphoreGive(i2c_mutex);
      Serial.println("MPU9250 connection failed");
      vTaskDelay(1000);
    }
  }
  
  mpu.calibrateAccelGyro();  // Calibrate sensors
  mpu.calibrateMag();        // Calibrate magnetometer
  xSemaphoreGive(i2c_mutex);

  TickType_t xLastWakeTime;
  int status = 0;
  struct mpu9250_data data;

  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    xQueueReceive(mpu9250_queue_signal_enable, &status, pdMS_TO_TICKS(0));
    
    if (status == 1) {
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      mpu.update();
      
      data.ax = mpu.getAccX();
      data.ay = mpu.getAccY();
      data.az = mpu.getAccZ();
      data.gx = mpu.getGyroX();
      data.gy = mpu.getGyroY();
      data.gz = mpu.getGyroZ();
      data.mx = mpu.getMagX();
      data.my = mpu.getMagY();
      data.mz = mpu.getMagZ();
      
      xSemaphoreGive(i2c_mutex);
      
      xQueueOverwrite(mpu9250_queue, &data);
      xQueueOverwrite(imu_processing_queue, &data);
    }
    
    vTaskDelayUntil(&xLastWakeTime, 10);  // 100Hz sample rate
  }
#else
  vTaskDelete(NULL);
#endif
}
