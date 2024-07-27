// Creator: Sean Ruiz
// File Name: ESP32DroneTransmitter
// Including code from:
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
*/
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0xDC, 0x69, 0xCC};

// Structure to send data
// Must match the receiver structure
typedef struct struct_message {
  int throttle_x_adc;
  int throttle_y_adc;
  int pitch_x_adc;
  int pitch_y_adc;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// CHANGE right_x_pin to 32 IF NOT WORKING WITH WIFI LATER ON (WIFI needs ADC2 pins, may need to stay on ADC1)
const int right_x_pin = 32; // RIGHT JOYSTICK X 
const int right_y_pin = 33;  // RIGHT JOYSTICK Y

const int left_x_pin = 34; // LEFT JOYSTICK X
const int left_y_pin = 35; // LEFT JOYSTICK Y


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // PINMODE SETUP
	pinMode(right_x_pin, INPUT);
	pinMode(right_y_pin, INPUT);

  pinMode(left_x_pin, INPUT);
  pinMode(left_y_pin, INPUT);

  Serial.begin(115200);   /* Define baud rate for serial communica+tion */
}

void loop() {
  // Joystick initialization
  int right_x_adc, right_y_adc, left_x_adc, left_y_adc;
  int mapped_right_x_adc, mapped_right_y_adc, mapped_left_x_adc, mapped_left_y_adc;

  // Analog read
  right_x_adc = analogRead(right_x_pin); 
  right_y_adc = analogRead(right_y_pin);

  left_x_adc = analogRead(left_x_pin);
  left_y_adc = analogRead(left_y_pin);

  if (right_x_adc <= 1800) {
    // Map the lower half (0 to 1750) to 0 to 1000
    mapped_right_x_adc = map(right_x_adc, 0, 1800, 1000, 1500);
  } else {
    // Map the upper half (1750 to 4095) to 1000 to 2000
    mapped_right_x_adc = map(right_x_adc, 1800, 4095, 1500, 2000);
  }

  if (right_y_adc <= 1800) {
    // Map the lower half (0 to 1750) to 0 to 1000
    mapped_right_y_adc = map(right_y_adc, 0, 1800, 2000, 1500);  // reversed mapping due to hardware
  } else {
    // Map the upper half (1750 to 4095) to 1000 to 2000
    mapped_right_y_adc = map(right_y_adc, 1800, 4095, 1500, 1000);
  }

  if (left_x_adc <= 1800) {
    // Map the lower half (0 to 1750) to 0 to 1000
    mapped_left_x_adc = map(left_x_adc, 0, 1800, 2000, 1500); // reversed mapping due to hardware
  } else {
    // Map the upper half (1750 to 4095) to 1000 to 2000
    mapped_left_x_adc = map(left_x_adc, 1800, 4095, 1500, 1000);
  }

  if (left_y_adc <= 1800) {
    // Map the lower half (0 to 1750) to 0 to 1000
    mapped_left_y_adc = map(left_y_adc, 0, 1800, 0, 1000);
  } else {
    // Map the upper half (1750 to 4095) to 1000 to 2000
    mapped_left_y_adc = map(left_y_adc, 1800, 4095, 1000, 2000);
  }
  // Serial print ADC values
  Serial.print("Yaw = ");
  Serial.print(mapped_left_x_adc);
  Serial.print("\t");
  Serial.print("Throttle = ");
  Serial.print(mapped_left_y_adc);

  Serial.print("\t");
  Serial.print("Roll = ");
  Serial.print(mapped_right_x_adc);
  Serial.print("\t");
  Serial.print("Pitch = ");
  Serial.println(mapped_right_y_adc);

  // Set values to send (GOING TO ONLY SEND ADC VALUES SINCE CONVERSION WILL BE EASIER with map())
  myData.throttle_x_adc, mapped_left_x_adc;
  myData.throttle_y_adc = mapped_left_y_adc;
  myData.pitch_x_adc = mapped_right_x_adc;
  myData.pitch_y_adc = mapped_right_y_adc;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  /*
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  delay(20);
  }
  */
  delay(250);
}