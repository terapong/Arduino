#include <IRremote.h>
int RECV_PIN = 15;
int led = 2;
#define code1 16753245  // แก้ค่าปุ่มกดที่ 1
#define code2 16736925  // แก้ค่าปุ่มกดที่ 2
IRrecv irrecv(RECV_PIN);
decode_results results;
void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();  // Start the receiver
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}
void loop() {
  if (irrecv.decode(&results)) {
    Serial.print("รหัสปุ่มกด: ");
    Serial.println(results.value);
    if (results.value == code1) {  // ถ้าค่าที่ส่งมาตรงกับ ค่าในตัวแปร code1 ให้ทำใน ปีกกา if
      Serial.println("LED ON");
      digitalWrite(led, HIGH);
    }
    if (results.value == code2) {  // ถ้าค่าที่ส่งมาตรงกับ ค่าในตัวแปร code2 ให้ทำใน ปีกกา if
      Serial.println("LED OFF");
      digitalWrite(led, LOW);
    }
    irrecv.resume();  // Receive the next value
  }
}

// #include <Arduino.h>
// #include <IRremoteESP8266.h>
// #include <IRrecv.h>
// #include <IRutils.h>

// // An IR detector/demodulator is connected to GPIO pin 5 (D1 on a NodeMCU
// // board).
// // Note: GPIO 16 won't work on the ESP8266 as it does not have interrupts.
// const uint16_t kRecvPin = 5;
// unsigned long key_value = 0;
// // Control LEDs with the 
// const int greenPin = 0; //connected to GPIO pin 0 (D3 on a NodeMCU board).
// const int yellowPin = 2; //connected to GPIO pin 2 (D4 on a NodeMCU board).

// IRrecv irrecv(kRecvPin);

// decode_results results;

// void setup() {
//   Serial.begin(115200);
//   irrecv.enableIRIn();  // Start the receiver
//   while (!Serial)  // Wait for the serial connection to be establised.
//     delay(50);
//   Serial.println();
//   Serial.print("IRrecvDemo is now running and waiting for IR message on Pin ");
//   Serial.println(kRecvPin);
//   pinMode(greenPin, OUTPUT);
//   pinMode(yellowPin, OUTPUT);
// }

// void loop() {
//   if (irrecv.decode(&results)) {
//     // print() & println() can't handle printing long longs. (uint64_t)
//     serialPrintUint64(results.value, HEX);
//     Serial.println("");

//     switch(results.value){
//           case 0xFFA25D:
//           Serial.println("CH-");
//           break;
//           case 0xFF629D:
//           Serial.println("CH");
//           break;
//           case 0xFFE21D:
//           Serial.println("CH+");
//           break;
//           case 0xFF22DD:
//           Serial.println("|<<");
//           break;
//           case 0xFF02FD:
//           Serial.println(">>|");
//           break ;  
//           case 0xFFC23D:
//           Serial.println(">|");
//           break ;               
//           case 0xFFE01F:
//           Serial.println("-");
//           break ;  
//           case 0xFFA857:
//           Serial.println("+");
//           break ;  
//           case 0xFF906F:
//           Serial.println("EQ");
//           break ;  
//           case 0xFF6897:
//           Serial.println("0");
//           break ;  
//           case 0xFF9867:
//           Serial.println("100+");
//           break ;
//           case 0xFFB04F:
//           Serial.println("200+");
//           break ;
//           case 0xFF30CF:
//           Serial.println("1");
//           break ;
//           case 0xFF18E7:
//           Serial.println("2");
//           // green LED on for 2 seconds
//           digitalWrite(greenPin, HIGH);
//           delay(2000);
//           digitalWrite(greenPin, LOW);
//           break ;
//           case 0xFF7A85:
//           Serial.println("3");
//           break ;
//           case 0xFF10EF:
//           Serial.println("4");
//           break ;
//           case 0xFF38C7:
//           Serial.println("5");
//           // yellow LED on for 2 seconds
//           digitalWrite(yellowPin, HIGH);
//           delay(2000);
//           digitalWrite(yellowPin, LOW);
//           break ;
//           case 0xFF5AA5:
//           Serial.println("6");
//           break ;
//           case 0xFF42BD:
//           Serial.println("7");
//           break ;
//           case 0xFF4AB5:
//           Serial.println("8");
//           break ;
//           case 0xFF52AD:
//           Serial.println("9");
//           break ;      
//         }
//         key_value = results.value;
        
//     irrecv.resume();  // Receive the next value
//   }
//   delay(100);
// }
