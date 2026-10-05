// TechsPassion Pad Starter
// Drive your own ESP32 robot with the "Controller" mode of the TechsPassion app,
// then add your own ideas in loop(). The phone is just a pad,
// and YOUR code decides what every move and button does.
//
// The app sends the whole pad state about 10 times a second:
//   GET /pad?x=-100..100&y=-100..100&b=0..15
//   x: joystick left (-100) to right (100)    y: down (-100) to up (100)
//   b: buttons held: A = 1, B = 2, C = 4, D = 8 (use button('A') below)
// Whatever text you set with say("...") is shown in the app.
//
// Board: any ESP32 in the Arduino IDE (esp32 by Espressif, version 3.x).
// The pins below match the TechsPassion Lite Robot (ESP32-S3 + OSOYOO Model Y driver).
// Using another board or driver? Change the pins and the FLIP settings.

#include <WiFi.h>
#include <WebServer.h>

// ---------- Wi-Fi ----------
// The robot makes its own hotspot. Join it on your phone, then connect the app to 192.168.4.1.
// WPA2 needs a password of at least 8 characters. Change it!
const char* AP_NAME     = "TP-PadBot";
const char* AP_PASSWORD = "change-me-123";

// ---------- Motors (DIR1, DIR2, PWM per side) ----------
const int LEFT_DIR1  = 38, LEFT_DIR2  = 39, LEFT_PWM  = 40;
const int RIGHT_DIR1 = 47, RIGHT_DIR2 = 48, RIGHT_PWM = 21;
const bool FLIP_LEFT  = false;  // set true if this side drives backwards
const bool FLIP_RIGHT = true;   // the Lite Robot's right motors are mounted mirrored
const int  MIN_PWM = 120;       // slowest PWM that still moves your motors
const int  MAX_PWM = 255;

// ---------- Safety ----------
const unsigned long PAD_TIMEOUT_MS = 500;  // stop if the phone goes quiet

WebServer server(80);
int padX = 0, padY = 0, padB = 0;
unsigned long lastPad = 0;
String message = "";

bool stopped = true;

// button('A') is true while A is held (also 'B', 'C', 'D').
bool button(char name) {
  int bit = name - 'A';
  if (bit < 0 || bit > 3) return false;
  return (padB >> bit) & 1;
}

// Show a short message in the app.
void say(const String& text) {
  if (message != text) message = text;
}

// One side: speed from -100 (full reverse) to 100 (full forward).
void setSide(int dir1, int dir2, int pwmPin, int speed, bool flip) {
  if (flip) speed = -speed;
  int pwm = 0;
  if (speed != 0) pwm = map(abs(speed), 1, 100, MIN_PWM, MAX_PWM);
  digitalWrite(dir1, speed > 0 ? HIGH : LOW);
  digitalWrite(dir2, speed < 0 ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

// Tank drive: left and right speeds from -100 to 100.
// If one side asks for more than 100, both are scaled down so turns keep their shape.
void drive(int left, int right) {
  int biggest = max(abs(left), abs(right));
  if (biggest > 100) {
    left = left * 100 / biggest;
    right = right * 100 / biggest;
  }
  setSide(LEFT_DIR1, LEFT_DIR2, LEFT_PWM, left, FLIP_LEFT);
  setSide(RIGHT_DIR1, RIGHT_DIR2, RIGHT_PWM, right, FLIP_RIGHT);
  stopped = (left == 0 && right == 0);
}

void stopMotors() {
  if (!stopped) drive(0, 0);
}

// The app calls this about 10 times a second.
void handlePad() {
  padX = constrain(server.arg("x").toInt(), -100, 100);
  padY = constrain(server.arg("y").toInt(), -100, 100);
  padB = constrain(server.arg("b").toInt(), 0, 15);
  lastPad = millis();
  server.sendHeader("Access-Control-Allow-Origin", "*");  // lets the app's web version connect too
  server.sendHeader("Cache-Control", "no-store");
  if (message.length() > 0) server.send(200, "text/plain", message);
  else server.send(200, "text/plain", "OK");
}

// Opening the robot's address in a browser shows this.
void handleRoot() {
  server.send(200, "text/plain", "TechsPassion Pad Starter is running. Use the app's Controller mode.");
}

void setup() {
  Serial.begin(115200);
  int pins[] = {LEFT_DIR1, LEFT_DIR2, LEFT_PWM, RIGHT_DIR1, RIGHT_DIR2, RIGHT_PWM};
  for (int i = 0; i < 6; i++) pinMode(pins[i], OUTPUT);
  stopped = false;
  stopMotors();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME, AP_PASSWORD);
  Serial.print("Join Wi-Fi \"");
  Serial.print(AP_NAME);
  Serial.print("\", then connect the app to ");
  Serial.println(WiFi.softAPIP());

  server.on("/pad", handlePad);
  server.on("/", handleRoot);
  server.begin();
  say("Hello from your robot!");
}

void loop() {
  server.handleClient();

  // Safety first: no update for a while means the phone is gone. Stop.
  if (millis() - lastPad > PAD_TIMEOUT_MS) {
    padX = padY = padB = 0;
    stopMotors();
    return;
  }

  // ===== Your code starts here =====

  // Joystick -> tank drive. Hold A for full speed, otherwise 60%.
  int limit = 60;
  if (button('A')) limit = 100;
  int left  = (padY + padX) * limit / 100;
  int right = (padY - padX) * limit / 100;
  drive(left, right);

  // Buttons can do anything: light an LED, move a servo, play a sound...
  if (button('B')) say("B is pressed!");
  else if (button('A')) say("Turbo!");
  else say("Ready");

  // ===== Your code ends here =====
}
