#ifndef MR_MIDI_H
#define MR_MIDI_H

#include <Arduino.h>
#include <MIDI.H>

#define MIDIRX 16
#define MIDITX 17




class MR_MIDI {
    public:
        MR_MIDI();
        void Init();
        void HandleNoteOn(byte channel, byte note, byte velocity);
        void HandleNoteOff(byte channel, byte note, byte velocity);
        void HandleControlChange(byte channel, byte number, byte value);
    protected:

    private:

};

#endif //MR_MIDI_H