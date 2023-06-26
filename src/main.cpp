#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <TaskScheduler.h>

Scheduler SchedulerA;
Adafruit_INA219 ina219;

int vel = 2000;

float BusVoltage = 0;

typedef union
{
  float cfloat;
  uint8_t cbytes[4];
} current;

typedef union
{
  float efloat;
  uint8_t ebytes[4];
} encoderpulsos;

current currentmA;

encoderpulsos encoderCount;

void Encoder()
{
  encoderCount.efloat++;
}

void EncoderSend()
{

  Serial.write('S');

  for (int i = 0; i < 4; i++)
  {
    Serial.write(encoderCount.ebytes[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    Serial.write(currentmA.cbytes[i]);
  }

  Serial.write('\n');
}

Task EncoderTask(5, TASK_FOREVER, &EncoderSend);

void setup(void)
{
  Serial.begin(115200);
  SchedulerA.addTask(EncoderTask);
  EncoderTask.enable();

  encoderCount.efloat = 0;

  // Iniciar el INA219
  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
    {
      delay(10);
    }
  }

  // ina219.setCalibration_32V_1A();
  ina219.setCalibration_16V_400mA();

  Serial.println("INA219 iniciado...");

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(8, INPUT);
  attachInterrupt(digitalPinToInterrupt(8), Encoder, FALLING);
  analogWriteResolution(12);
}

void loop(void)
{
  SchedulerA.execute();

  // Obtener mediciones
  // shunt = ina219.getShuntVoltage_mV();
  // BusVoltage = ina219.getBusVoltage_V();
  currentmA.cfloat = ina219.getCurrent_mA();
  // ina219.getCurrent_mA();

  // delay(2000);

  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  analogWrite(2, vel); // 0 a 4095
}