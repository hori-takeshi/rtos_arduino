#include "ZumoBuzzer.h"
#include <Arduino.h>

#define BUZZER_PIN 6

const char *buzzerSequence = 0;

// declaring these globals as static means they won't conflict
// with globals in other .cpp files that share the same name
static volatile unsigned int buzzerTimeout = 0;    // tracks buzzer time limit
static char play_mode_setting = PLAY_AUTOMATIC;

extern volatile unsigned char buzzerFinished;  // flag: 0 while playing
extern const char *buzzerSequence;


static unsigned char use_program_space; // boolean: true if we should
                    // use program space

// music settings and defaults
static unsigned char octave = 4;        // the current octave
static unsigned int whole_note_duration = 2000;  // the duration of a whole note
static unsigned int note_type = 4;              // 4 for quarter, etc
static unsigned int duration = 500;        // the duration of a note in ms
static unsigned int volume = 15;        // the note volume
static unsigned char staccato = 0;       // true if playing staccato

// staccato handling
static unsigned char staccato_rest_duration;  // duration of a staccato
                        //  rest, or zero if it is time
                        //  to play a note

static void nextNote();

ZumoBuzzer::ZumoBuzzer()
{
}

inline void ZumoBuzzer::init()
{
}

void ZumoBuzzer::init2()
{
}

void ZumoBuzzer::playFrequency(unsigned int freq, unsigned int dur, 
                     unsigned char volume)
{
    tone(BUZZER_PIN, freq, dur);    
}

void ZumoBuzzer::playNote(unsigned char note, unsigned int dur,
                 unsigned char volume)
{
      unsigned int freq = 0;
  unsigned char offset_note = note - 16;

  if (note == SILENT_NOTE || volume == 0)
  {
    freq = 1000;  // silent notes => use 1kHz freq (for cycle counter)
    playFrequency(freq, dur, 0);
    return;
  }

  if (note <= 16)
    offset_note = 0;
  else if (offset_note > 95)
    offset_note = 95;

  unsigned char exponent = offset_note / 12;

  // frequency table for the lowest 12 allowed notes
  //   frequencies are specified in tenths of a Hertz for added resolution
  switch (offset_note - exponent * 12)  // equivalent to (offset_note % 12)
  {
    case 0:        // note E1 = 41.2 Hz
      freq = 412;
      break;
    case 1:        // note F1 = 43.7 Hz
      freq = 437;
      break;
    case 2:        // note F#1 = 46.3 Hz
      freq = 463;
      break;
    case 3:        // note G1 = 49.0 Hz
      freq = 490;
      break;
    case 4:        // note G#1 = 51.9 Hz
      freq = 519;
      break;
    case 5:        // note A1 = 55.0 Hz
      freq = 550;
      break;
    case 6:        // note A#1 = 58.3 Hz
      freq = 583;
      break;
    case 7:        // note B1 = 61.7 Hz
      freq = 617;
      break;
    case 8:        // note C2 = 65.4 Hz
      freq = 654;
      break;
    case 9:        // note C#2 = 69.3 Hz
      freq = 693;
      break;
    case 10:      // note D2 = 73.4 Hz
      freq = 734;
      break;
    case 11:      // note D#2 = 77.8 Hz
      freq = 778;
      break;
  }

  if (exponent < 7)
  {
    freq = freq << exponent;  // frequency *= 2 ^ exponent
    if (exponent > 1)      // if the frequency is greater than 160 Hz
      freq = (freq + 5) / 10;  //   we don't need the extra resolution
    else
      freq += DIV_BY_10;    // else keep the added digit of resolution
  }
  else
    freq = (freq * 64 + 2) / 5;  // == freq * 2^7 / 10 without int overflow

  if (volume > 15)
    volume = 15;
  playFrequency(freq, dur, volume);  // set buzzer this freq/duration
}

// Returns 1 if the buzzer is currently playing, otherwise it returns 0
unsigned char ZumoBuzzer::isPlaying()
{
    if(buzzerSequence) {
        return 1;
    }
    else {
        return 0;
    }
}

#include "ZumoShield_cfg.h"
#include "r2ca.h"

void
zumobuzzer_task(intptr_t exinf) {
    while(buzzerSequence)  {
        nextNote();
    }
}

void ZumoBuzzer::play(const char *notes)
{
    buzzerSequence = notes;
    use_program_space = 0;
    staccato_rest_duration = 0;
    act_tsk(ZUMOBUZZER_TASK);
}

void ZumoBuzzer::playFromProgramSpace(const char *notes_p)
{
    buzzerSequence = notes_p;
    use_program_space = 1;
    staccato_rest_duration = 0;
    act_tsk(ZUMOBUZZER_TASK);
}

void ZumoBuzzer::stopPlaying()
{
}

// Gets the current character, converting to lower-case and skipping
// spaces.  For any spaces, this automatically increments sequence!
static char currentCharacter()
{
  char c = 0;
  do
  {
    if(use_program_space)
      c = pgm_read_byte(buzzerSequence);
    else
      c = *buzzerSequence;

    if(c >= 'A' && c <= 'Z')
      c += 'a'-'A';
  } while(c == ' ' && (buzzerSequence ++));

  return c;
}

// Returns the numerical argument specified at buzzerSequence[0] and
// increments sequence to point to the character immediately after the
// argument.
static unsigned int getNumber()
{
  unsigned int arg = 0;

  // read all digits, one at a time
  char c = currentCharacter();
  while(c >= '0' && c <= '9')
  {
    arg *= 10;
    arg += c-'0';
    buzzerSequence ++;
    c = currentCharacter();
  }

  return arg;
}

static void nextNote()
{
  unsigned char note = 0;
  unsigned char rest = 0;
  unsigned char tmp_octave = octave; // the octave for this note
  unsigned int tmp_duration; // the duration of this note
  unsigned int dot_add;

  char c; // temporary variable

  // if we are playing staccato, after every note we play a rest
  if(staccato && staccato_rest_duration)
  {
    ZumoBuzzer::playNote(SILENT_NOTE, staccato_rest_duration, 0);
    staccato_rest_duration = 0;
    return;
  }

 parse_character:

  // Get current character
  c = currentCharacter();
  buzzerSequence ++;

  // Interpret the character.
  switch(c)
  {
  case '>':
    // shift the octave temporarily up
    tmp_octave ++;
    goto parse_character;
  case '<':
    // shift the octave temporarily down
    tmp_octave --;
    goto parse_character;
  case 'a':
    note = NOTE_A(0);
    break;
  case 'b':
    note = NOTE_B(0);
    break;
  case 'c':
    note = NOTE_C(0);
    break;
  case 'd':
    note = NOTE_D(0);
    break;
  case 'e':
    note = NOTE_E(0);
    break;
  case 'f':
    note = NOTE_F(0);
    break;
  case 'g':
    note = NOTE_G(0);
    break;
  case 'l':
    // set the default note duration
    note_type = getNumber();
    duration = whole_note_duration/note_type;
    goto parse_character;
  case 'm':
    // set music staccato or legato
    if(currentCharacter() == 'l')
      staccato = false;
    else
    {
      staccato = true;
      staccato_rest_duration = 0;
    }
    buzzerSequence ++;
    goto parse_character;
  case 'o':
    // set the octave permanently
    octave = getNumber();
    tmp_octave = octave;
    goto parse_character;
  case 'r':
    // Rest - the note value doesn't matter.
    rest = 1;
    break;
  case 't':
    // set the tempo
    whole_note_duration = 60*400/getNumber()*10;
    duration = whole_note_duration/note_type;
    goto parse_character;
  case 'v':
    // set the volume
    volume = getNumber();
    goto parse_character;
  case '!':
    // reset to defaults
    octave = 4;
    whole_note_duration = 2000;
    note_type = 4;
    duration = 500;
    volume = 15;
    staccato = 0;
    // reset temp variables that depend on the defaults
    tmp_octave = octave;
    tmp_duration = duration;
    goto parse_character;
  default:
    buzzerSequence = 0;
    return;
  }

  note += tmp_octave*12;

  // handle sharps and flats
  c = currentCharacter();
  while(c == '+' || c == '#')
  {
    buzzerSequence ++;
    note ++;
    c = currentCharacter();
  }
  while(c == '-')
  {
    buzzerSequence ++;
    note --;
    c = currentCharacter();
  }

  // set the duration of just this note
  tmp_duration = duration;

  // If the input is 'c16', make it a 16th note, etc.
  if(c > '0' && c < '9')
    tmp_duration = whole_note_duration/getNumber();

  // Handle dotted notes - the first dot adds 50%, and each
  // additional dot adds 50% of the previous dot.
  dot_add = tmp_duration/2;
  while(currentCharacter() == '.')
  {
    buzzerSequence ++;
    tmp_duration += dot_add;
    dot_add /= 2;
  }

  if(staccato)
  {
    staccato_rest_duration = tmp_duration / 2;
    tmp_duration -= staccato_rest_duration;
  }
  
  // this will re-enable the timer1 overflow interrupt
  ZumoBuzzer::playNote(rest ? SILENT_NOTE : note, tmp_duration, volume);
  delay(tmp_duration*1.5);
}

void ZumoBuzzer::playMode(unsigned char mode)
{
}

unsigned char ZumoBuzzer::playCheck()
{
    return 0;    
}
