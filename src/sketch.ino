// -*- c -*-
// WaveSensor Arduino sketch
//
// Copyright (c) 2013 Dave Sieh
//
// See LICENSE.txt for details.
#define ENABLED 1
#define DISABLED 0
#define MOTION DISABLED
#define PROXIMITY DISABLED
#define BLUE_TOOTH ENABLED
#define SOUND DISABLED
#define I2C DISABLED
#define SERVOS DISABLED

#include <WaveHC.h>
#include <WaveUtil.h>
#include <VCNL4000.h>
#if BLUE_TOOTH==ENABLED
#include <boards.h>
#include "BLEFirmata.h"
#include <ble_shield.h>
#endif

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
//
// So I had this set to 200ms originally. It worked OK, I guess, but
// if the switch changes occurred too fast, it was filtering out changes
// and made the thing hang in play mode with no way to turn it off short
// of a reset. I tried 100ms and it improved somewhat. Knocking it down
// to 10ms removed almost all the sensitivity. I'm going to stick with
// this.
//
#define DELAY_TIME 10

enum MotionState {
  Stopped,
  Moving
};

int previousValue = -1;
long lastChangeTime = 0L;
MotionState currentState = Stopped;
#endif // If Motion Sensor

#if BLUE_TOOTH==ENABLED
#define MINIMUM_SAMPLING_INTERVAL 10
/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting
/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written
int samplingInterval = 38;          // how often to run the main loop (in ms)
#endif

#define RAILROAD_CROSSING "RRCRBELL.WAV"
#define BOMB_SIREN "BOMBSIRN.WAV"

#define error(msg) error_P(PSTR(msg))

void setupWave();
void playSound(char *);

// Callbacks for firmata
void analogWriteCallback(byte, int);
void digitalWriteCallback(byte, int);
void reportAnalogCallback(byte, int);
void reportDigitalCallback(byte, int);
void setPinModeCallback(byte, int);
void sysexCallback(byte, byte, byte *);
void systemResetCallback();

void setup() {
#if BLUE_TOOTH==ENABLED
  BleFirmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  BleFirmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  BleFirmata.attach(REPORT_ANALOG, reportAnalogCallback);
  BleFirmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  BleFirmata.attach(SET_PIN_MODE, setPinModeCallback);
  BleFirmata.attach(START_SYSEX, sysexCallback);
  BleFirmata.attach(SYSTEM_RESET, systemResetCallback);

  systemResetCallback();  // reset to default config
#endif

  Serial.begin(9600);

#if PROXIMITY==ENABLED
  proximitySensor.begin();
#endif

#if MOTION==ENABLED
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_DETECT_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
#endif

#if SOUND==ENABLED
  setupWave();
#endif

#if BLUE_TOOTH==ENABLED
  // Set up the bluetooth stuff
  ble_begin();
#endif

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
#if BLUE_TOOTH==ENABLED
  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();  
  
  /* SERIALREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while(BleFirmata.available())
    BleFirmata.processInput();

  if (!ble_connected())
    systemResetCallback();
 
  ble_do_events();
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

#if BLUE_TOOTH==ENABLED

void analogWriteCallback(byte port, int value) {
  // Nothing to do here. Not doing any analog writes...
  Serial.print("analogWriteCallback(");
  Serial.print(port);
  Serial.print(", ");
  Serial.print(value);
  Serial.println(")");
}

void digitalWriteCallback(byte port, int value) {
  Serial.print("digitalWriteCallback(");
  Serial.print(port);
  Serial.print(", ");
  Serial.print(value);
  Serial.println(")");
}

void reportAnalogCallback(byte port, int value) {
  Serial.print("reportAnalogCallback(");
  Serial.print(port);
  Serial.print(", ");
  Serial.print(value);
  Serial.println(")");
}

void reportDigitalCallback(byte port, int value) {
  Serial.print("reportDigitalCallback(");
  Serial.print(port);
  Serial.print(", ");
  Serial.print(value);
  Serial.println(")");
}

void setPinModeCallback(byte port, int value) {
  Serial.print("setPinModeCallback(");
  Serial.print(port);
  Serial.print(", ");
  Serial.print(value);
  Serial.println(")");
}

void sysexCallback(byte command, byte argc, byte *argv) {
  Serial.println("sysexCallback");
  byte mode;
  byte slaveAddress;
  byte slaveRegister;
  byte data;
  unsigned int delayTime; 
  
  switch(command) {
#if I2C==ENABLED
  case I2C_REQUEST:
    mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
    if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
      BleFirmata.sendString("10-bit addressing mode is not yet supported");
      return;
    }
    else {
      slaveAddress = argv[0];
    }

    switch(mode) {
    case I2C_WRITE:
      Wire.beginTransmission(slaveAddress);
      for (byte i = 2; i < argc; i += 2) {
        data = argv[i] + (argv[i + 1] << 7);
        #if ARDUINO >= 100
        Wire.write(data);
        #else
        Wire.send(data);
        #endif
      }
      Wire.endTransmission();
      delayMicroseconds(70);
      break;
    case I2C_READ:
      if (argc == 6) {
        // a slave register is specified
        slaveRegister = argv[2] + (argv[3] << 7);
        data = argv[4] + (argv[5] << 7);  // bytes to read
        readAndReportData(slaveAddress, (int)slaveRegister, data);
      }
      else {
        // a slave register is NOT specified
        data = argv[2] + (argv[3] << 7);  // bytes to read
        readAndReportData(slaveAddress, (int)REGISTER_NOT_SPECIFIED, data);
      }
      break;
    case I2C_READ_CONTINUOUSLY:
      if ((queryIndex + 1) >= MAX_QUERIES) {
        // too many queries, just ignore
        BleFirmata.sendString("too many queries");
        break;
      }
      queryIndex++;
      query[queryIndex].addr = slaveAddress;
      query[queryIndex].reg = argv[2] + (argv[3] << 7);
      query[queryIndex].bytes = argv[4] + (argv[5] << 7);
      break;
    case I2C_STOP_READING:
	  byte queryIndexToSkip;      
      // if read continuous mode is enabled for only 1 i2c device, disable
      // read continuous reporting for that device
      if (queryIndex <= 0) {
        queryIndex = -1;        
      } else {
        // if read continuous mode is enabled for multiple devices,
        // determine which device to stop reading and remove it's data from
        // the array, shifiting other array data to fill the space
        for (byte i = 0; i < queryIndex + 1; i++) {
          if (query[i].addr = slaveAddress) {
            queryIndexToSkip = i;
            break;
          }
        }
        
        for (byte i = queryIndexToSkip; i<queryIndex + 1; i++) {
          if (i < MAX_QUERIES) {
            query[i].addr = query[i+1].addr;
            query[i].reg = query[i+1].addr;
            query[i].bytes = query[i+1].bytes; 
          }
        }
        queryIndex--;
      }
      break;
    default:
      break;
    }
    break;
#endif
#if I2C==ENABLED
  case I2C_CONFIG:
    delayTime = (argv[0] + (argv[1] << 7));

    if(delayTime > 0) {
      i2cReadDelayTime = delayTime;
    }

    if (!isI2CEnabled) {
      enableI2CPins();
    }
    
    break;
#endif
#if SERVOS==ENABLED
  case SERVO_CONFIG:
    if(argc > 4) {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte pin = argv[0];
      int minPulse = argv[1] + (argv[2] << 7);
      int maxPulse = argv[3] + (argv[4] << 7);

      if (IS_PIN_SERVO(pin)) {
        if (servos[PIN_TO_SERVO(pin)].attached())
          servos[PIN_TO_SERVO(pin)].detach();
        servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
        setPinModeCallback(pin, SERVO);
      }
    }
    break;
#endif
  case SAMPLING_INTERVAL:
    if (argc > 1) {
      samplingInterval = argv[0] + (argv[1] << 7);
      if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
        samplingInterval = MINIMUM_SAMPLING_INTERVAL;
      }      
    } else {
      //Firmata.sendString("Not enough data");
    }
    break;
  case EXTENDED_ANALOG:
    if (argc > 1) {
      int val = argv[1];
      if (argc > 2) val |= (argv[2] << 7);
      if (argc > 3) val |= (argv[3] << 14);
      analogWriteCallback(argv[0], val);
    }
    break;
  case CAPABILITY_QUERY:
    ble_write(START_SYSEX);
    ble_write(CAPABILITY_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_DIGITAL(pin)) {
        ble_write((byte)INPUT);
        ble_write(1);
        ble_write((byte)OUTPUT);
        ble_write(1);
      }
      if (IS_PIN_ANALOG(pin)) {
        ble_write(ANALOG);
        ble_write(10);
      }
      if (IS_PIN_PWM(pin)) {
        ble_write(PWM);
        ble_write(8);
      }
      if (IS_PIN_SERVO(pin)) {
        ble_write(SERVO);
        ble_write(14);
      }
      if (IS_PIN_I2C(pin)) {
        ble_write(I2C);
        ble_write(1);  // to do: determine appropriate value 
      }
      ble_write(127);
    }
    ble_write(END_SYSEX);
    break;
  case PIN_STATE_QUERY:
    if (argc > 0) {
      byte pin=argv[0];
      ble_write(START_SYSEX);
      ble_write(PIN_STATE_RESPONSE);
      ble_write(pin);
      if (pin < TOTAL_PINS) {
        ble_write((byte)pinConfig[pin]);
	ble_write((byte)pinState[pin] & 0x7F);
	if (pinState[pin] & 0xFF80) ble_write((byte)(pinState[pin] >> 7) & 0x7F);
	if (pinState[pin] & 0xC000) ble_write((byte)(pinState[pin] >> 14) & 0x7F);
      }
      ble_write(END_SYSEX);
    }
    break;
  case ANALOG_MAPPING_QUERY:
    ble_write(START_SYSEX);
    ble_write(ANALOG_MAPPING_RESPONSE);
    for (byte pin=0; pin < TOTAL_PINS; pin++) {
      ble_write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
    }
    ble_write(END_SYSEX);
    break;
  }
}

void systemResetCallback() {
  // initialize a defalt state

  for (byte i=0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;	// until activated
    previousPINs[i] = 0;
  }

  // pins with analog capability default to analog input
  // otherwise, pins default to digital output
  for (byte i=0; i < TOTAL_PINS; i++) {

    // skip pin 8, 9 for BLE Shield
    if ((i == 8) || (i == 9))
      continue;
    
    // skip SPI pins
    if ( (i==MOSI) || (i==MISO) || (i==SCK) || (i==SS) )
      continue;
     
     // Default all to digital pins  
//    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
//      setPinModeCallback(i, ANALOG);
//    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
//    }
  }
  
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

 }

void checkDigitalInputs() {
  // Check to see if the motion state has changed. If so,
  // report it to the BlueTooth device.
}
#endif
