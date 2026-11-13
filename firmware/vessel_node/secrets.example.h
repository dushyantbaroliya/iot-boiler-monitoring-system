// Copy this file to secrets.h (same folder) and fill in your values.
// secrets.h is git-ignored so keys never reach GitHub.
#pragma once

#define VESSEL_ID 1                       // 1, 2 or 3: one board per vessel

#define WIFI_SSID     "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// One ThingSpeak channel per vessel. Field 1 = temperature (C), field 2 = pressure (bar g).
#define TS_CHANNEL_ID 0000000UL
#define TS_WRITE_KEY  "YOUR_WRITE_API_KEY"
