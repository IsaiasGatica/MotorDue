#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <TaskScheduler.h>

Scheduler SchedulerA;
Adafruit_INA219 ina219;

int vel = 2000;

float current_mA = 0;
float BusVoltage = 0;

u_int16_t encoderCount = 0;
byte buffer[4];

void Encoder()
{
  encoderCount++;
}
void EncoderSend()
{
  buffer[0] = 'S';
  buffer[1] = (encoderCount >> 8) & 0xFF;
  buffer[2] = (encoderCount & 0xFF);
  buffer[3] = '\n';

  // Serial.write(buffer, 4);
}

Task EncoderTask(5, TASK_FOREVER, &EncoderSend);

void setup(void)
{
  Serial.begin(115200);
  SchedulerA.addTask(EncoderTask);
  EncoderTask.enable();

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
  // SchedulerA.execute();

  // Obtener mediciones
  // shunt = ina219.getShuntVoltage_mV();
  BusVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  // power_mW = ina219.getPower_mW();
  // loadvoltage = busvoltage + (shuntvoltage / 1000);

  // Mostrar mediciones
  Serial.print(BusVoltage);
  Serial.println(" V");
  Serial.print(current_mA);
  Serial.println(" mA");

  delay(2000);

  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  analogWrite(2, vel); // 0 a 4095
}