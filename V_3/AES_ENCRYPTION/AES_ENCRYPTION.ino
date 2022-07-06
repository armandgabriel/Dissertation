
#include <AESLib.h>

uint8_t key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
char data[] = "HELLO MY FRIEND1";
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  aes128_enc_single(key, data);
  Serial.print("ENCRYPTED:" );
  Serial.println(data);
  aes128_dec_single(key, data);
  Serial.print("DECRYPTED: ");
  Serial.println(data);
}

void loop() {
  // put your main code here, to run repeatedly:

}
