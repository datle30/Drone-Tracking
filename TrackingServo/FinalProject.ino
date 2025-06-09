#include <Servo.h>

// ==== SERVO SETUP ====
Servo servoX;             // Servo truc x
Servo servoY;             // Servo truc y

int servoPinX = 9;        // Servo x noi chan D9
int servoPinY = 10;       // Servo y noi chan D10

int currentX = 90;        // Goc hien tai servo x = 90
int currentY = 90;        // Goc hien tai servo y = 90

const int deadzone = 2;   // Bo qua sai so nho hon 2
const int maxStep = 3;    // Thay doi toi da 3 do

// ==== LASER & BUTTON SETUP ====
const int buttonPin = 7;      // Nut an noi chan D7
const int laserPin = 8;       // Laser noi chan D8

bool laserState = true;       // Trang thai ban dau la bat
bool lastButtonState = HIGH;  // Trang thai truoc cua nut la khong an
bool currentButtonState;      // Trang thai hien tai cua nut
bool buttonReleased = true;   // Flag xu ly 1 lan duy nhat khi nha nut

void setup() 
{
  // Serial & Servo
  Serial.begin(115200);                 // Khoi dong Serial 115200 bps
  servoX.attach(servoPinX);             // Gan servo x vao chan D9
  servoY.attach(servoPinY);             // Gan servo y vao chan D10
  servoX.write(currentX);               // Goc ban dau servo x = 90 
  servoY.write(currentY);               // Goc ban dau servo y = 90
  delay(500);                           // Delay 500ms de on dinh

  // Laser & Button
  pinMode(buttonPin, INPUT_PULLUP);     // Nut nhan voi dien tro keo len
  pinMode(laserPin, OUTPUT);            // Laser la dau ra
  digitalWrite(laserPin, laserState);   // Bat laser
}

void loop() 
{
  // ===== SERVO CONTROL =====
  if (Serial.available())                                         // Neu co du lieu gui qua Serial
  {                                       
    String data = Serial.readStringUntil('\n');                   // Doc chuoi du lieu den khi gap \n
    int commaIndex = data.indexOf(',');                           // Tim dau , de tach chuoi  

    if (commaIndex > 0) 
    {
      int targetX = data.substring(0, commaIndex).toInt();        // Lay dau chuoi la goc servo x
      int targetY = data.substring(commaIndex + 1).toInt();       // Lay cuoi chuoi la goc servo y

      targetX = constrain(targetX, 0, 180);                       // Gioi han goc servo x
      targetY = constrain(targetY, 60, 180);                      // Gioi han goc servo y

      if (abs(targetX - currentX) > deadzone)                     // Neu sai so lon hon deadzone moi cap nhat goc x
      {                   
        if (targetX > currentX)                                   
          currentX = min(currentX + maxStep, targetX);            // Tang goc toi da maxStep
        else
          currentX = max(currentX - maxStep, targetX);            // Giam goc toi da maxStep
        servoX.write(currentX);                                   // Lenh quay servo x
      }

      if (abs(targetY - currentY) > deadzone)                     // Neu sai so lon hon deadzone moi cap nhat goc y
      {                  
        if (targetY > currentY)
          currentY = min(currentY + maxStep, targetY);            // Tang goc toi da maxStep
        else
          currentY = max(currentY - maxStep, targetY);            // Giam goc toi da maxStep
        servoY.write(currentY);                                   // Lenh quay servo x
      }

      Serial.print("Servo X: "); Serial.print(currentX);          // In ra trang thai goc ra Serial
      Serial.print(" | Servo Y: "); Serial.println(currentY);
    }
  }

  // ===== LASER CONTROL =====
  currentButtonState = digitalRead(buttonPin);                    // Doc trang thai hien tai cua nut nhan

  if (currentButtonState == LOW && buttonReleased)                // Neu nut duoc an
  {              
    while (digitalRead(buttonPin) == LOW);                        // Cho nut duoc tha 
    delay(50);                                                    // Delay chong doi nut

    laserState = !laserState;                                     // Doi trang thai laser
    digitalWrite(laserPin, laserState);                           // Lenh bat/tat laser

    buttonReleased = false;                                       // Danh dau da xu ly nhan
  }

  if (currentButtonState == HIGH)                                 // Neu nut da duoc tha thi cho phep nhan lan tiep theo
  {
    buttonReleased = true;
  }
}
