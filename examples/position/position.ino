#include <adra_api.h>
// if the rxPin is 2 pin, txPin is 3 pin, rtsPin is 4 pin. you can init like this.
AdraApi adraApi(2,3,4);
void setup() {
  // put your setup code here, to run once:
  // serial baudrate
  Serial.begin(500000);
  // adra baudrate
  adraApi.connect(115200);
  int ret;
  // adra id
  int id = 1;
  float value_fp;
  char buffer[40];
  
  ret = adraApi.into_motion_mode_pos(id);
  sprintf(buffer, "into_motion_mode_pos %d ", ret);
  Serial.println(buffer);
  
  ret = adraApi.into_motion_enable(id);
  sprintf(buffer, "into_motion_enable %d ", ret);
  Serial.println(buffer);
  delay(1000); 

  ret = adraApi.set_pos_target(id,31.4);
  sprintf(buffer, "set_pos_target %d ", ret);
  Serial.println(buffer);
  
  ret = adraApi.get_pos_current(id,&value_fp);
  sprintf(buffer, "get_pos_current %d ", ret);
  Serial.print(buffer);
  Serial.println(value_fp); 
  delay(3000);

  ret = adraApi.set_pos_target(id,-31.4);
  sprintf(buffer, "set_pos_target %d ", ret);
  Serial.println(buffer);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
