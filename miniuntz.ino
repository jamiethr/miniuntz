/****************************************************************************
MIT License

Copyright (c) 2024 jamie.thorup@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
****************************************************************************/

#include <Wire.h>
#include <Adafruit_Trellis.h>
#include <MIDIUSB.h>

#define TRELLIS_I2C_ADDR  0x70

#define LED     LED_BUILTIN // Pin for heartbeat LED (shows code is working)
#define CHANNEL 1           // MIDI channel number

#define MIDI_MSG_NOTE_ON  0x90
#define MIDI_MSG_NOTE_OFF 0x80
#define MIDI_MSG_CC       0xB0

Adafruit_Trellis trellis;

/*
 * Do not enable without connecting pots to the inputs.
 */
//#define ANALOG_INPUT

uint8_t       heart        = 0;  // Heartbeat LED counter
unsigned long prevReadTime = 0L; // Keypad polling timer
#ifdef ANALOG_INPUT
uint8_t       mod;
uint8_t       vel;
uint8_t       fxc;
uint8_t       rate;
#endif

// 60, ..., 63 => C3 (middle C) to A2
uint8_t note[] = {
  60, 61, 62, 63,
  56, 57, 58, 59,
  52, 53, 54, 55,
  48, 49, 50, 51
};

// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

/// Sends a "note on" message to the host
void noteOn(byte channel, byte pitch, byte velocity) {
  // midiEventPacket_t noteOn = {0x09, (byte)(0x90 | channel), pitch, velocity};

  // The midi packet's header uses the upper nibble of the MIDI message's status byte, so we must bit-shift down by 4
  // FIXME: replace the bit-shift with a macro?
  midiEventPacket_t noteOn = {(MIDI_MSG_NOTE_ON >> 4), (byte)(MIDI_MSG_NOTE_ON | channel), pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {(MIDI_MSG_NOTE_OFF >> 4), (byte)(MIDI_MSG_NOTE_OFF | channel), pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  //midiEventPacket_t event = {0x0B, (byte) (0xB0 | channel), control, value};
  midiEventPacket_t event = {(MIDI_MSG_CC >> 4), (byte) (MIDI_MSG_CC | channel), control, value};
  MidiUSB.sendMIDI(event);
}

void setup() {
  pinMode(LED, OUTPUT);
  trellis.begin(TRELLIS_I2C_ADDR);
#ifdef __AVR__
  // Default Arduino I2C speed is 100 KHz, but the HT16K33 supports
  // 400 KHz.  We can force this for faster read & refresh, but may
  // break compatibility with other I2C devices...so be prepared to
  // comment this out, or save & restore value as needed.
  TWBR = 12;
#endif
  trellis.clear();
  trellis.writeDisplay();
#ifdef ANALOG_INPUT
  mod = map(analogRead(0), 0, 1023, 0, 127);
  vel = map(analogRead(1), 0, 1023, 0, 127);
  fxc = map(analogRead(2), 0, 1023, 0, 127);
  rate = map(analogRead(3),0, 1023, 0, 127);
  controlChange(CHANNEL,  1, mod);
  controlChange(CHANNEL, 11, vel);
  controlChange(CHANNEL, 12, fxc);
  controlChange(CHANNEL, 13, rate);
#endif
}

void loop() {
  unsigned long t = millis();
  if((t - prevReadTime) >= 20L) { // 20ms = min Trellis poll time
    if(trellis.readSwitches()) {  // Check if the state of buttons on the Trellis have changed 

      for(uint8_t i=0; i<16; i++) { // Update the state of each button
        if(trellis.justPressed(i)) {
          noteOn(CHANNEL, note[i], 127);

          trellis.setLED(i);
        } else if(trellis.justReleased(i)) {
          noteOn(CHANNEL, note[i], 0);
          trellis.clrLED(i);
        }
      }
      trellis.writeDisplay();   // Update the LEDs on the Trellis to match each button's state
    }
#ifdef ANALOG_INPUT
    uint8_t newModulation = map(analogRead(0), 0, 1023, 0, 127);
    if(mod != newModulation) {
      mod = newModulation;
      controlChange(CHANNEL, 1, mod);
    }
    uint8_t newVelocity = map(analogRead(1), 0, 1023, 0, 127);
    if(vel != newVelocity) {
      vel = newVelocity;
      controlChange(CHANNEL, 11, vel);
    }
    uint8_t newEffect = map(analogRead(2), 0, 1023, 0, 127);
    if(fxc != newEffect) {
      fxc = newEffect;
      controlChange(CHANNEL, 12, fxc);
    }
    uint8_t newRate = map(analogRead(3), 0, 1023, 0, 127);
    if(rate !=newRate) {
      rate = newRate;
      controlChange(CHANNEL, 13, rate);
    }
#endif
    prevReadTime = t;
    digitalWrite(LED, ++heart & 32); // Blink = alive
    MidiUSB.flush();
  }
  (void)MidiUSB.read(); // Discard incoming MIDI messages
}
