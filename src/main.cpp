#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define TDS_PIN 34
#define PH_PIN 35
#define TURBIDITY_PIN 32
#define ONE_WIRE_BUS 4

#define VREF 3.3
#define ADC_RES 4095.0
#define SAMPLE_COUNT 30

const char *SERVER_NAME = "192.168.1.100";
const uint16_t PORT = 5000;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

QueueHandle_t sensorQueue;

typedef struct
{
  float temperature;
  float tds;
  float ph;
  float turbidity;
} SensorData;

float readADCVoltage(int pin)
{
  return analogRead(pin) * (VREF / ADC_RES);
}

void sampleSensor(int pin, float *buffer)
{
  for (int i = 0; i < SAMPLE_COUNT; i++)
  {
    buffer[i] = readADCVoltage(pin);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

float getMedian(float *arr)
{
  float temp[SAMPLE_COUNT];
  memcpy(temp, arr, sizeof(float) * SAMPLE_COUNT);
  for (int i = 0; i < SAMPLE_COUNT - 1; i++)
  {
    for (int j = i + 1; j < SAMPLE_COUNT; j++)
    {
      if (temp[i] > temp[j])
      {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[SAMPLE_COUNT / 2];
}

float getAverageWithoutOutliers(float *arr)
{
  float minVal = arr[0], maxVal = arr[0], sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++)
  {
    if (arr[i] < minVal)
      minVal = arr[i];
    if (arr[i] > maxVal)
      maxVal = arr[i];
    sum += arr[i];
  }
  return (sum - minVal - maxVal) / (SAMPLE_COUNT - 2);
}

float calculateTDS(float voltage, float temperature)
{
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensatedVoltage = voltage / compensationCoefficient;
  return (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage - 255.86 * compensatedVoltage * compensatedVoltage + 857.39 * compensatedVoltage) * 0.5;
}

float calculatePH(float voltage)
{
  float slope = -5.7;
  float offset = 21.34;
  return slope * voltage + offset;
}

float calculateTurbidity(float voltage)
{
  if (voltage < 2.5)
    return 3000;
  return -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;
}

void sensorTask(void *pvParameters)
{
  float tdsBuffer[SAMPLE_COUNT];
  float phBuffer[SAMPLE_COUNT];
  float turbBuffer[SAMPLE_COUNT];

  while (true)
  {
    sampleSensor(TDS_PIN, tdsBuffer);
    sampleSensor(PH_PIN, phBuffer);
    sampleSensor(TURBIDITY_PIN, turbBuffer);

    float tdsVoltage = getMedian(tdsBuffer);
    float phVoltage = getMedian(phBuffer);
    float turbVoltage = getAverageWithoutOutliers(turbBuffer);

    ds18b20.requestTemperatures();
    float temperature = ds18b20.getTempCByIndex(0);
    if (temperature == -127)
      continue;

    SensorData data;
    data.temperature = temperature;
    data.tds = calculateTDS(tdsVoltage, temperature);
    data.ph = calculatePH(phVoltage);
    data.turbidity = calculateTurbidity(turbVoltage);

    xQueueSend(sensorQueue, &data, portMAX_DELAY);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void networkTask(void *pvParameters)
{
  WiFiClient client;

  while (true)
  {
    SensorData data;

    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE)
    {
      if (!client.connect(SERVER_NAME, PORT))
      {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        continue;
      }

      JsonDocument doc;
      doc["temperature"] = data.temperature;
      doc["tds"] = data.tds;
      doc["ph"] = data.ph;
      doc["turbidity"] = data.turbidity;

      String payload;
      serializeJson(doc, payload);

      client.println(payload);
      client.stop();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ds18b20.begin();

  WiFiManager wm;
  wm.autoConnect("ESP32_Setup");

  sensorQueue = xQueueCreate(5, sizeof(SensorData));

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(networkTask, "NetworkTask", 6144, NULL, 1, NULL, 0);
}

void loop() {}
