// -*- c -*-
// WaveSensor Arduino sketch
//
// Copyright (c) 2013 Dave Sieh
//
// See LICENSE.txt for details.

#include <WaveHC.h>
#include <WaveUtil.h>
#include <VCNL4000.h>

#define ENABLED 1
#define DISABLED 0
#define MOTION ENABLED
#define PROXIMITY DISABLED

SdReader card;    // This object holds the information for the card
FatVolume vol;    // This holds the information for the partition on the card
FatReader root;   // This holds the information for the volumes root directory
FatReader f;
WaveHC wave;      // This is the only wave (audio) object

#if PROXIMITY==ENABLED
VCNL4000 proximitySensor;
boolean state = false;
#endif

#if MOTION==ENABLED
#define SENSOR_DETECT_PIN 7
#define LED_PIN 13

// How long to wait for no change to identify a stopped status (ms)
#define STOP_THRESHOLD 1000L

// How long to wait between reading the sensor. You may want to
// tweak this value or the STOP THRESHOLD depending on how fast
// the item is being moved. If the thing is moving fast, you may
// want to shorten this time so you don't miss too many measurements.
// If the thing is moving slow, you can probably leave this alone
// or increase it, but you might need to increase the STOP_THRESHOLD.
// 
// Think about it.
#define DELAY_TIME 200

enum MotionState {
  Stopped,
  Moving
};

int previousValue = -1;
long lastChangeTime = 0L;
MotionState currentState = Stopped;
#endif // If Motion Sensor

#define RAILROAD_CROSSING "RRCRBELL.WAV"
#define BOMB_SIREN "BOMBSIRN.WAV"

#define error(msg) error_P(PSTR(msg))

void setupWave();
void playSound(char *);

void setup() {
  Serial.begin(9600);

#if PROXIMITY==ENABLED
  proximitySensor.begin();
#endif

#if MOTION==ENABLED
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_DETECT_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
#endif

  setupWave();

  Serial.println("Ready!");
}

void loop() {
#if PROXIMITY==ENABLED
  uint16_t distance = proximitySensor.readProximity();

  if (distance > 2150) {
    if (!state) {
      Serial.println("InRange");
      state = true;
      playSound(RAILROAD_CROSSING);
    }
  } else {
    if (state) {
      Serial.println("OutOfRange");
      state = false;
      if (wave.isplaying) {
	wave.stop();
      }
    }
  }

  delay(100);
#endif

#if MOTION==ENABLED
  int sampleTime = millis();
  int value = digitalRead(SENSOR_DETECT_PIN);

  // Deal with the startup case
  if (previousValue < 0) {
    previousValue = value;
    lastChangeTime = sampleTime;
    delay(DELAY_TIME);
    return;
  }

  // Now deal with the real world
  if (value != previousValue) {
    // Wait 10ms for debounce of the switch
    delay(10);
    handleValueChange(value);
  } else if (currentState == Moving) {
    handleValueNoChange(sampleTime);
  }

  delay(DELAY_TIME); 
#endif
}

void setupWave() {
  Serial.println("Setting up Wave");

  Serial.print("Free RAM: ");
  Serial.println(FreeRam());

  if (!card.init()) {
    error("Card initialization failed!");
  }

  card.partialBlockRead(true);

  uint8_t part;
  for (part = 0; part < 5; part++) {
    if (vol.init(card, part)) {
      break;
    }
  }
  if (part == 5) {
    error("No valid FAT partition!");
  }

  Serial.print("Using partition: ");
  Serial.print(part, DEC);
  Serial.print(", type is FAT");
  Serial.println(vol.fatType(), DEC);

  if (!root.openRoot(vol)) {
    error("Can't open root directory!");
  }

  Serial.println("Done Setting up Wave");
}

void playSound(char *name) {
  // see if the wave object is currently doing something
  if (wave.isplaying) {// already playing something, so stop it!
    wave.stop(); // stop it
  }

  // look in the root directory and open the file
  if (!f.open(root, name)) {
    putstring("Couldn't open file "); Serial.print(name); return;
  }

  // OK read the file and turn it into a wave object
  if (!wave.create(f)) {
    putstring_nl("Not a valid WAV"); return;
  }
  
  // ok time to play! start playback
  wave.play();
}

void error_P(const char *str) {
  PgmPrint("Error: ");
  SerialPrint_P(str);
  sdErrorCheck();
  while(1);
}

/*
 * print error message and halt if SD I/O error, great for debugging!
 */
void sdErrorCheck(void) {
  if (!card.errorCode()) return;
  PgmPrint("\r\nSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  PgmPrint(", ");
  Serial.println(card.errorData(), HEX);
  while(1);
}

#if MOTION==ENABLED
void handleValueChange(int value) {
  previousValue = value;
  lastChangeTime = millis();
  if (currentState == Stopped) {
    currentState = Moving;
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Now moving...");
    playSound(RAILROAD_CROSSING);
  }
}

void handleValueNoChange(int sampleTime) {
  long duration = sampleTime - lastChangeTime;
  if (duration > STOP_THRESHOLD) {
    currentState = Stopped;
    digitalWrite(LED_PIN, LOW);
    Serial.print("No change for too long (");
    Serial.print(duration);
    Serial.println(") ms. Stopped");
    if (wave.isplaying) {
      wave.stop();
    }
  }
}
#endif
