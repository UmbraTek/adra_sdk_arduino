#include <adra_api.h>


void setup() {
  // put your setup code here, to run once:
  // serial baudrate
  Serial.begin(9600);
  // if Use HardwareSerial Serial1 rtsPin is 4 pin. you can init like this.
  AdraApi adraApi(1,4);
  // adra baudrate
  adraApi.connect(115200);
  int ret;
  // adra id
  int id = 1;
  float value_fp;
  float value_nfp[2];
  uint8_t value_u8[2];
  int8_t value_int8[2];
  char buffer[40];
  
  char buf[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  ret = adraApi.get_uuid(id,buf);
  sprintf(buffer, "get_uuid %d value %s", ret, buf);
  Serial.println(buffer);
  
  char buf1[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  ret = adraApi.get_sw_version(id,buf1);
  sprintf(buffer, "get_sw_version %d value %s", ret, buf1);
  Serial.println(buffer);

  char buf2[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  ret = adraApi.get_hw_version(id,buf2);
  sprintf(buffer, "get_hw_version %d value %s", ret, buf2);
  Serial.println(buffer);

  ret = adraApi.get_multi_version(id,buf1);
  sprintf(buffer, "get_multi_version %d value %s", ret, buf1);
  Serial.println(buffer);

  ret = adraApi.get_mech_ratio(id, &value_fp);
  sprintf(buffer,"get_mech_ratio   : %d, value =", ret);
  Serial.print(buffer);
  Serial.println(value_fp); 

  adraApi.set_elec_ratio(id, 1.2);
  ret = adraApi.get_elec_ratio(id, &value_fp);
  sprintf(buffer,"get_elec_ratio   : %d, value =", ret);
  Serial.print(buffer);
  Serial.println(value_fp);

  ret = adraApi.get_motion_dir(id, value_u8);
  sprintf(buffer,"get_motion_dir: %d, value = %d\n", ret, value_u8[0]);
  Serial.print(buffer);

  ret = adraApi.get_temp_limit(id, &value_int8[0], &value_int8[1]);
  sprintf(buffer,"get_temp_limit: %d, value = %d %d\n", ret, value_int8[0], value_int8[1]);
  Serial.print(buffer);

  ret = adraApi.get_volt_limit(id, &value_u8[0], &value_u8[1]);
  sprintf(buffer,"get_volt_limit: %d, value = %d %d\n", ret, value_u8[0], value_u8[1]);
  Serial.print(buffer);

  ret = adraApi.get_curr_limit(id, &value_fp);
  sprintf(buffer,"get_curr_limit   : %d, value =", ret);
  Serial.print(buffer);
  Serial.println(value_fp);

  ret = adraApi.get_motion_mode(id, value_u8);
  sprintf(buffer,"get_motion_mode  : %d, value = %d\n", ret, value_u8[0]);
  Serial.print(buffer);
}

void loop() {
  // put your main code here, to run repeatedly:

}