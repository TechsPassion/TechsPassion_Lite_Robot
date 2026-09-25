// Wi-Fi credentials. Kept out of the main sketch so the .ino can be shared
// without leaking the password. Fill these in before uploading to the robot.
#pragma once

// Your home Wi-Fi (must be 2.4 GHz - the ESP32 can't join 5 GHz networks).
#define WIFI_SSID     "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Fallback hotspot the robot creates when it can't reach the Wi-Fi above.
// WPA2 requires the password to be at least 8 characters. Change it!
#define AP_SSID       "TPRLite"
#define AP_PASSWORD   "password"
