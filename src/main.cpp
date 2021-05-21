#include <Arduino.h>
#include <driver/dac.h>
#include "painlessMesh.h"
#include <ArduinoJson.h>

#define RXD2 16
#define TXD2 17

#define PI 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899
const int potPin = 34;
float potValue = 0;
bool up = true;
int count = 0;
double m = 0.0;
float vol = 0.1;
float SR = 1000000;
float p = 0;
float out = 0;
float mainVol = 0;
int level = 255;
bool playSineWave = false;
bool playSqrWave = false;
bool playSawWave = false;
bool playTriWave = false;
float freq = 200;
int wave = 0;

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// User stub
void sendMessage(); // Prototype so PlatformIO doesn't complain

Task taskSendMessage(TASK_SECOND * 1, TASK_FOREVER, &sendMessage);

void sendMessage()
{
  mesh.sendBroadcast(String(freq));
  taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5));
}

// Needed for painless library
void receivedCallback(uint32_t from, String &msg)
{
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId)
{
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback()
{
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset)
{
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

float sine(float freq, float volume)
{
  float SI = freq * 2 * PI / SR;
  p = fmod(micros() * SI, 2 * PI);
  out = (sin(p) * volume + 1) / 2;
  return out;
}
float noise(float volume)
{
  out = random(10000) - 5000 / 10000.0 * volume;
  return out;
}

float saw(float freq, float volume)
{
  float SI = freq * 2 * PI / SR;
  p = fmod(micros() * SI, 2 * PI);
  out = ((p / PI - 1) * volume + 1) / 2;
  return out;
}

float tri(float freq, float volume)
{
  float SI = freq * 2 * PI / SR;
  p = fmod(micros() * SI, 2 * PI);
  float sig = (p / PI < 1 ? p / PI * 2 : 2 - (p / PI - 1) * 2) - 1;
  out = (sig * volume + 1) / 2;
  return out;
}

float square(float freq, float volume)
{
  float SI = freq * 2 * PI / SR;
  p = fmod(micros() * SI, 2 * PI);
  float sig = (p / PI > 1 ? 2 : 0) - 1;
  out = (sig * volume + 1) / 2;
  return out;
}

void vis(int out)
{
  for (int i = 0; i < out / 4; i++)
  {
    Serial.print("_");
  }
  Serial.println();
}

void playSine()
{
  float output = sine(freq, 1);

  int out = output * level;

  dac_output_voltage(DAC_CHANNEL_1, out);
}
void playSaw()
{
  float output = saw(freq, 1);

  int out = output * level;

  dac_output_voltage(DAC_CHANNEL_1, out);
}
void playTri()
{
  float output = tri(freq, 1);

  int out = output * level;

  dac_output_voltage(DAC_CHANNEL_1, out);
}
void playSqr()
{
  float output = square(freq, 1);

  int out = output * level;

  dac_output_voltage(DAC_CHANNEL_1, out);
}

void setup()
{
  Serial.begin(115200);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes(ERROR | STARTUP); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  Serial.println("Started up...");
  dac_output_enable(DAC_CHANNEL_1);
}

void loop()
{
  if (playSineWave)
  {
    playSqrWave = false;
    playSawWave = false;
    playTriWave = false;
    playSine();
  }
  if (playSawWave)
  {
    playSqrWave = false;
    playSineWave = false;
    playTriWave = false;
    playSaw();
  }
  if (playTriWave)
  {
    playSqrWave = false;
    playSawWave = false;
    playSineWave = false;
    playTri();
  }
  if (playSqrWave)
  {
    playSineWave = false;
    playSawWave = false;
    playTriWave = false;
    playSqr();
  }

  if (Serial2.available())
  {
    // Allocate the JSON document
    // This one must be bigger than for the sender because it must store the strings
    StaticJsonDocument<300> doc;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc, Serial2);

    if (err == DeserializationError::Ok)
    {
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      Serial.println(doc["ready"].as<bool>());
      if (doc["ready"].as<bool>())
      {
        freq = doc["freq"].as<float>();
        String waveform = doc["waveshape"].as<String>();
        if (waveform.equals("tri"))
        {
          playTriWave = true;
        }
        if (waveform.equals("sine"))
        {
          playSineWave = true;
        }
        if (waveform.equals("square"))
        {
          playSqrWave = true;
        }
        if (waveform.equals("saw"))
        {
          playSawWave = true;
        }
      }
    }
    else
    {
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());

      // Flush all bytes in the "link" serial port buffer
      while (Serial2.available() > 0)
        Serial2.read();
    }
  }
  mesh.update();
}
