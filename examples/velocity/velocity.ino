#include <adra_api.h>

void setup() {
  // put your setup code here, to run once:
  // serial baudrate
  Serial.begin(500000);
  // if the rxPin is 2 pin, txPin is 3 pin, rtsPin is 4 pin. you can init like this.
  AdraApi adraApi(2,3,4);
  // adra baudrate
  adraApi.connect(115200);
  int ret;
  // adra id
  int id = 1;
  char buffer[40];
  
  ret = adraApi.into_motion_mode_vel(id);
  sprintf(buffer, "into_motion_mode_vel %d ", ret);
  Serial.println(buffer);
  
  ret = adraApi.into_motion_enable(id);
  sprintf(buffer, "into_motion_enable %d ", ret);
  Serial.println(buffer);
  delay(1000); 

  ret = adraApi.set_vel_target(id,31.4);
  sprintf(buffer, "set_vel_target %d ", ret);
  Serial.println(buffer);

  delay(10000);

  ret = adraApi.set_vel_target(id,-31.4);
  sprintf(buffer, "set_vel_target %d ", ret);
  Serial.println(buffer);
  delay(10000);
  ret = adraApi.into_motion_disable(id);
}

void loop() {
  // put your main code here, to run repeatedly:

}
