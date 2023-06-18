#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <TaskScheduler.h>

Scheduler SchedulerA;

Adafruit_INA219 ina219;

int vel = 2000;

unsigned long timeold;

u_int16_t encoderCount = 0;
byte buffer[4];

void Encoder()
{
  encoderCount++;
}
void EncoderSend()
{
  buffer[0] = 'E';
  buffer[1] = encoderCount >> 8;
  buffer[2] = encoderCount & 0x00FF;
  buffer[3] = '\n';

  Serial.write(buffer, 4);
}

Task EncoderTask(5, TASK_FOREVER, &EncoderSend);

void setup(void)
{
  Serial.begin(250000);
  SchedulerA.addTask(EncoderTask);
  EncoderTask.enable();

  // Iniciar el INA219
  // if (!ina219.begin())
  // {
  //   Serial.println("Failed to find INA219 chip");
  //   while (1)
  //   {
  //     delay(10);
  //   }
  // }

  // por defecto, inicia a 32V y 2A

  // Opcionalmente, cambiar la sensibilidad del sensor
  // ina219.setCalibration_32V_1A();
  // // ina219.setCalibration_16V_400mA();

  // Serial.println("INA219 iniciado...");

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(8, INPUT);
  attachInterrupt(digitalPinToInterrupt(8), Encoder, FALLING);
  timeold = 0;
  analogWriteResolution(12);
}

void loop(void)
{
  SchedulerA.execute();

  // float rpm;
  // float shuntvoltage = 0;
  // float busvoltage = 0;
  // float current_mA = 0;
  // float loadvoltage = 0;
  // float power_mW = 0;

  // // Obtener mediciones
  // shuntvoltage = ina219.getShuntVoltage_mV();
  // busvoltage = ina219.getBusVoltage_V();
  // current_mA = ina219.getCurrent_mA();
  // power_mW = ina219.getPower_mW();
  // loadvoltage = busvoltage + (shuntvoltage / 1000);

  // // Mostrar mediciones
  // Serial.print("Bus Voltaje:   ");
  // Serial.print(busvoltage);
  // Serial.println(" V");
  // Serial.print("Shunt Voltaje: ");
  // Serial.print(shuntvoltage);
  // Serial.println(" mV");
  // Serial.print("Load Voltaje:  ");
  // Serial.print(loadvoltage);
  // Serial.println(" V");
  // Serial.print("Corriente:       ");
  // Serial.print(current_mA);
  // Serial.println(" mA");
  // Serial.print("Potencia:         ");
  // Serial.print(power_mW);
  // Serial.println(" mW");
  // Serial.println("");

  // delay(2000);

  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  analogWrite(2, vel); // 0 a 4095

  // if (millis() - timeold >= 5)
  // {
  //   // __disable_irq();

  //   // rpm = float((60.0 * 1000.0 / 64.0) / (millis() - timeold) * encoderCount);

  //   timeold = millis();
  //   // encoderCount = 0;

  //   // __enable_irq();

  //   // Serial.println(rpm);

    // }
}