#include <BMP_New_Library.h>

float SEA_LEVEL_PRESSURE = 1018.0;
float temp;

float getHumidity(float temp, int pin){
  return (161*(analogRead(pin)/1023.)-25.8)/(1.0546-0.0026*temp);
}

void setup(void) {
  delay(3000);
  digitalWrite(10, LOW);
  SerialUSB.begin(9600);
}

void loop(void) {
  BMP_180 Sensor;
  Sensor.start();
  while(true){
  
  if(Sensor.update() > 0){
    temp = Sensor.getTemp();
    SerialUSB.print(temp);
    SerialUSB.print("C  ");
    SerialUSB.print(Sensor.getPressure());
    SerialUSB.print("hPa  ");
    SerialUSB.print(Sensor.getAltitude(SEA_LEVEL_PRESSURE));
    SerialUSB.print("m  ");
    SerialUSB.print(getHumidity(temp, A5));
    SerialUSB.println("%");
  }
  delay(1);
  }
}

