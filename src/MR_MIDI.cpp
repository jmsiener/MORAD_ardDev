#include <Arduino.h>
#include <MR_MIDI.h>
#include <MIDI.h>

struct SerialMIDISettings : public midi::DefaultSettings{
  static const long BaudRate = 31250;
};

// must use HardwareSerial for extra UARTs
HardwareSerial MIDISerial(2);
// instantiate the serial MIDI library
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, MIDISerial, MIDI, SerialMIDISettings);


MR_MIDI::MR_MIDI() {
  MIDISerial.begin(31250, SERIAL_8N1, MIDIRX,MIDITX ); // midi port
  
}

void MR_MIDI::Init() {
    //
    //MR_MIDI mr_midi;
    // set up serial MIDI library callbacks
    //MIDI.setHandleNoteOn(mr_midi.HandleNoteOn);  // Put only the name of the function
    //MIDI.setHandleNoteOff(mr_midi.HandleNoteOff);
    //MIDI.setHandleControlChange(mr_midi.HandleControlChange);
    // Initiate serial MIDI communications, listen to all channels
    //MIDI.begin(MIDI_CHANNEL_OMNI); 
}

#define MIN_GATE_OFF_TIME 500 // minimum gate off time in uS. fix for apps that send very rapid note off-note on sequences

// MIDI command handlers

void MR_MIDI::HandleNoteOn(byte channel, byte note, byte velocity) {/*
  // handle channels set up for note to CV 
 for( int i=0 ; i<(NUM_CV_OUTS);++i) {  // scan through CV configs
    if ((cvout[i].MIDIchannel==channel) && (cvout[i].type==NOTE_CV)) {  // see if MIDI channel matches and type is note to CV 
       if ((note >= LOWEST_NOTE) &&( note<= HIGHEST_NOTE)) { // don't play notes out of DAC range
         CVout(i,MIDInote_to_DACvalue(note,i));  // set the CV 
       }
    }
  }

  for( int i=0 ; i<(NUM_GATE_OUTS);++i) {  // scan through gate configs
    if ((gateout[i].MIDIchannel==channel) && (gateout[i].type==NOTES_GATE)) {  // see if MIDI channel matches and type is note to gate on/off 
       if ((note >= LOWEST_NOTE) &&( note<= HIGHEST_NOTE)) { // don't play notes out of DAC range
         GATEout(i,1); //turn on the gate
       }
    }
    if ((gateout[i].MIDIchannel==channel) && (gateout[i].type==NOTE_TRIGGER)) {  // see if MIDI channel matches and type is note trigger
       if (note == gateout[i].CC_NOTE_num) { // trigger on a specific note
         GATEout(i,1); //turn on the gate
       }
    }
  }*/
}

void MR_MIDI::HandleNoteOff(byte channel, byte note, byte velocity) {
  /*
  for( int i=0 ; i<(NUM_GATE_OUTS);++i) {  // scan through HW unit configs
    if ((gateout[i].MIDIchannel==channel) && (gateout[i].type==NOTES_GATE)) {  // see if MIDI channel matches and type is note to gate on/off 
       if ((note >= LOWEST_NOTE) &&( note<= HIGHEST_NOTE)) { // don't play notes out of DAC range
         GATEout(i,0); //turn off the gate. 
         delayMicroseconds(MIN_GATE_OFF_TIME);  // kludgy fix for apps that send rapid note off-note on sequence
       }
    }
    if ((gateout[i].MIDIchannel==channel) && (gateout[i].type==NOTE_TRIGGER)) {  // see if MIDI channel matches and type is note trigger
       if (note == gateout[i].CC_NOTE_num) { // trigger on a specific note
         GATEout(i,0); //turn off the gate
         delayMicroseconds(MIN_GATE_OFF_TIME);
       }
    }
  } 
  */ 
}


void MR_MIDI::HandleControlChange(byte channel, byte number, byte value) {
/*
  // handle channels set up for CC to CV 
  for( int i=0 ; i<(NUM_CV_OUTS);++i) {  // scan through CV configs
    if ((cvout[i].MIDIchannel==channel) && (cvout[i].type==CC_CV)) {  // see if MIDI channel matches and type is CC to CV 
      CVout(i,value*32); // don't worry about CV accuracy - CC is only 7 bits anyway.
    }
  }

  for( int i=0 ; i<(NUM_GATE_OUTS);++i) {  // scan through gate configs
    if ((gateout[i].MIDIchannel==channel) && (gateout[i].type==CC_GATE)) {  // see if MIDI channel matches and type is CC to gate
       if (value< 64) GATEout(i,0); // CC < 64=gate low. 
       else GATEout(i,1); // CC > 64=gate high. 
    }
  }*/
}