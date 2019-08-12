/*
  xdrv_23_stepper.ino - Stepper motor support for Sonoff-Tasmota

  Copyright (C) 2019 Gabor Simon

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_STEPPER

#define XDRV_23              23

const char kStepperCommands[] PROGMEM =
  D_CMND_CALIBRATE "|" D_CMND_SPEED "|" D_CMND_STEP;

void (* const StepperCommand[])(void) PROGMEM = {
  &StepperCmndCalibrate, &StepperCmndSpeed, &StepperCmndStep };

unsigned long next_tick_time = 0;
unsigned long tick_ms = 10;

uint8_t stepper_A_ena_pin = 0;
uint8_t stepper_A_pos_pin = 0;
uint8_t stepper_A_neg_pin = 0;

uint8_t stepper_B_ena_pin = 0;
uint8_t stepper_B_pos_pin = 0;
uint8_t stepper_B_neg_pin = 0;

uint8_t stepper_ovld_pin  = 0;

bool enabled = false;
bool have_enable_pins = false;

struct stepper_state_t {
  uint8_t A_pos:1;
  uint8_t B_pos:1;
  uint8_t A_neg:1;
  uint8_t B_neg:1;
  uint8_t A_ena:1;
  uint8_t B_ena:1;
};

// Off-state: every control as disabled as it can be
stepper_state_t const all_off = {0,0,0,0, 0,0};

// Stepper control schemes: http://users.ece.utexas.edu/~valvano/Datasheets/Stepper_ST.pdf

/* Rotating wave: 

    ENA = ENB = 1 always
         ____                _
     A _|    |____.____.____| 
              ____
     B _.____|    |____.____._
                   ____
    #A _.____.____|    |____._
                        ____
    #B _.____.____.____|    |_

        |<-0-|--1-|--2-|--3>|
*/
stepper_state_t const rotating_wave[] PROGMEM = {
  {1,0,0,0, 1,1}, // phase 0
  {0,1,0,0, 1,1}, // phase 1
  {0,0,1,0, 1,1}, // phase 2
  {0,0,0,1, 1,1}  // phase 3
};


/* Full step:

    ENA = ENB = 1 always, #A = not A, #B = not B
         _________           _
     A _|         |____.____| 
              _________
     B _.____|         |____._
       _           _________
    #A  |____.____|         |_
       _.____           ____._
    #B       |____.____|      

        |<-0-|--1-|--2-|--3>|
*/
stepper_state_t const full_step[] PROGMEM = {
  {1,0,0,1, 1,1}, // phase 0
  {1,1,0,0, 1,1}, // phase 1
  {0,1,1,0, 1,1}, // phase 2
  {0,0,1,1, 1,1}  // phase 3
}; 


/* Half step always-on:

    ENA = ENB = 1 always

         ______________
     A _|              |____.____.____.____.____._
                   ______________
     B _.____.____|              |____.____.____._
                             ______________
    #A _.____.____.____.____|              |____._
       _.____                          ____.____._
    #B       |____.____.____.____.____|         

        |<-0-|--1-|--2-|--3-|--4-|--5-|--6-|--7>|
*/
stepper_state_t const half_step[] PROGMEM = {
  {1,0,0,1, 1,1}, // phase 0
  {1,0,0,0, 1,1}, // phase 1
  {1,1,0,0, 1,1}, // phase 2
  {0,1,0,0, 1,1}, // phase 3
  {0,1,1,0, 1,1}, // phase 4
  {0,0,1,0, 1,1}, // phase 5
  {0,0,1,1, 1,1}, // phase 6
  {0,0,0,1, 1,1}  // phase 7
};


/* Half step with inhibition

    #A = not A, #B = not B, NOTE: different order of signals!
         ____.____.____.____                     _
     A _|                  \|____.____.____.___/|
         ____.____.____      ____.____.____      _
   EnA _|              |____|              |____|
       _                     ____.____.____.____
    #A  |____.____.____.___/|                  \|_

                   ____.____.____.____
     B _.____.___/|                  \|____.____._
       _.____      ____.____.____      ____.____._
   EnB       |____|              |____|          
       _.____.____                     ____.____._
    #B           \|____.____.____.___/|          

        |<-0-|--1-|--2-|--3-|--4-|--5-|--6-|--7>|
*/

stepper_state_t const half_step_inhibit[] PROGMEM = {
  {1,0,0,1, 1,1}, // phase 0
  {1,0,0,1, 1,0}, // phase 1
  {1,1,0,0, 1,1}, // phase 2
  {1,1,0,0, 0,1}, // phase 3
  {0,1,1,0, 1,1}, // phase 4
  {0,1,1,0, 1,0}, // phase 5
  {0,0,1,1, 1,1}, // phase 6
  {0,0,1,1, 0,1}  // phase 7
};

stepper_state_t const *current_scheme;
int8_t current_phase, max_phase;
int32_t current_pos, wanted_pos;
bool idle;
bool lock_when_done;

/*
  NOTE:
  PlatformIO generates declarations for our functions to the beginning of the
  merged giant .cpp source, but that's way sooner than our type definitions
  here, so if we use a locally defined type in a function argument (and not
  just as a pointer to it), then it will generate errors.

  To prevent this, we need to outsmart PlatformIOs function recognition
  pattern, and one way to do it is to make the return type consist of more
  than two words, like 'static const void'.

  It seems that this workaround for the declaration order problems is causing
  worse issues than what it is intended to solve...
*/

static const void StepperSet(const stepper_state_t &st_P) {
  stepper_state_t st;
  memcpy_P(&st, &st_P, sizeof(stepper_state_t));

  // to reduce the transitional hazards:
  // if an Enable pin falls, set that first; if it rises, set that last
  if (have_enable_pins) {
    if (!st.A_ena) {
      digitalWrite(stepper_A_ena_pin, 0);
    }
    if (!st.B_ena) {
      digitalWrite(stepper_B_ena_pin, 0);
    }
  }

  digitalWrite(stepper_A_pos_pin, st.A_pos);
  digitalWrite(stepper_A_neg_pin, st.A_neg);

  digitalWrite(stepper_B_pos_pin, st.B_pos);
  digitalWrite(stepper_B_neg_pin, st.B_neg);

  if (have_enable_pins) {
    if (st.A_ena) {
      digitalWrite(stepper_A_ena_pin, 1);
    }
    if (st.B_ena) {
      digitalWrite(stepper_B_ena_pin, 1);
    }
  }
}


void StepperTick(void) {
  if (wanted_pos < current_pos) {
    --current_phase;
    if (current_phase < 0) {
      current_phase = max_phase - 1;
    }
    --current_pos;
    StepperSet(current_scheme[current_phase]);
  }
  else if (wanted_pos > current_pos) {
    ++current_phase;
    if (current_phase >= max_phase) {
      current_phase = 0;
    }
    ++current_pos;
    StepperSet(current_scheme[current_phase]);
  }
  else {
    if (lock_when_done) {
      StepperSet(current_scheme[current_phase]);
    }
    else
    {
      StepperSet(all_off);
    }
    idle = true;
  }
}


void StepperCmndCalibrate(void)
{
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperCmndCalibrate;"));
  // assume the current state as position 0
  current_pos = 0;
  wanted_pos = 0;
  idle = true;
  StepperSet(all_off);
  ResponseCmndDone();
}

void StepperCmndSpeed(void)
{
  // arguments: tick_ms, scheme code
  char *buffer = XdrvMailbox.data;
  uint32_t buffer_length = XdrvMailbox.data_len;
  char *p, *str, *tok;

  buffer[buffer_length] = '\0';
  do {
    // parse @tick_ms
    str = strtok_r(buffer, ",", &tok);
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      tick_ms = (unsigned long)strtol(str, &p, 0);
      if (p == str) {
        ResponseCmndChar(D_ERROR);
        return;
      }
    }

    // parse @scheme_code
    str = strtok_r(nullptr, ",", &tok);
    if (!str)
      break;
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      int scheme_code = (int)strtol(str, &p, 0);
      if (p == str) {
        ResponseCmndChar(D_ERROR);
        return;
      }

      switch (scheme_code) {
        case 0:
          current_scheme = rotating_wave;
          max_phase = 4;
          break;
        case 1:
          current_scheme = full_step;
          max_phase = 4;
          break;
        case 2:
          current_scheme = half_step;
          max_phase = 8;
          break;
        case 3:
          current_scheme = half_step_inhibit;
          max_phase = 8;
          break;
        default:
        ResponseCmndChar(D_ERROR);
        return;
      }
      current_phase = 0;
    }
  } while (0);
  ResponseCmndDone();
}

void StepperCmndStep(void)
{
  // arguments: req_pos, is_relative, lock_when_done
  char *buffer = XdrvMailbox.data;
  uint32_t buffer_length = XdrvMailbox.data_len;
  char *p, *str, *tok;

  // argument defaults (no args: stay there and unlock)
  bool is_relative = true;
  int32_t req_pos = 0;

  lock_when_done = false;

  buffer[buffer_length] = '\0';
  do {
    // parse @req_pos
    str = strtok_r(buffer, ",", &tok);
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      req_pos = (int32_t)strtol(str, &p, 0);
      if (p == str) {
        ResponseCmndChar(D_ERROR);
        return;
      }
    }

    // parse @is_relative
    str = strtok_r(nullptr, ",", &tok);
    if (!str)
      break;
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      int i = (int)strtol(str, &p, 0);
      if (p == str) {
        i = GetStateNumber(str);
      }
      if ((i != 0) && (i != 1)) {
        ResponseCmndChar(D_ERROR);
        return;
      }
      is_relative = (i != 0);
    }

    // parse @lock_when_done
    str = strtok_r(nullptr, ",", &tok);
    if (!str)
      break;
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      int i = (int)strtol(str, &p, 0);
      if (p == str) {
        i = GetStateNumber(str);
      }
      if ((i != 0) && (i != 1)) {
        ResponseCmndChar(D_ERROR);
        return;
      }
      lock_when_done = (i != 0);
    }
  } while (0);

  // process the arguments
  if (is_relative) {
    wanted_pos = current_pos + req_pos;
  }
  else {
    wanted_pos = req_pos;
  }
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperCmndStep; is_relative=%d, req_pos=%d, lock_when_done=%d"), is_relative, req_pos, lock_when_done);

  idle = false;
  ResponseCmndDone();
}


void StepperInit(void)
{
  stepper_A_ena_pin = pin[GPIO_STEPPER_A_ENA];
  stepper_A_pos_pin = pin[GPIO_STEPPER_A_POS];
  stepper_A_neg_pin = pin[GPIO_STEPPER_A_NEG];

  stepper_B_ena_pin = pin[GPIO_STEPPER_B_ENA];
  stepper_B_pos_pin = pin[GPIO_STEPPER_B_POS];
  stepper_B_neg_pin = pin[GPIO_STEPPER_B_NEG];

  stepper_ovld_pin = pin[GPIO_STEPPER_OVLD];

  enabled = (stepper_A_pos_pin < 99) && (stepper_A_neg_pin < 99) && (stepper_B_ena_pin < 99) && (stepper_B_neg_pin < 99);
  have_enable_pins = enabled && (stepper_A_ena_pin < 99) && (stepper_B_ena_pin < 99);
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperInit; enabled=%d, have_enable_pins=%d"), enabled, have_enable_pins);

  if (enabled) {
    if (have_enable_pins) {
      pinMode(stepper_A_ena_pin, OUTPUT);
      digitalWrite(stepper_A_ena_pin, 0);
      pinMode(stepper_B_ena_pin, OUTPUT);
      digitalWrite(stepper_B_ena_pin, 0);
    }
    pinMode(stepper_A_pos_pin, OUTPUT);
    digitalWrite(stepper_A_pos_pin, 0);
    pinMode(stepper_A_neg_pin, OUTPUT);
    digitalWrite(stepper_A_neg_pin, 0);
    pinMode(stepper_B_pos_pin, OUTPUT);
    digitalWrite(stepper_B_pos_pin, 0);
    pinMode(stepper_B_neg_pin, OUTPUT);
    digitalWrite(stepper_B_neg_pin, 0);
  }

  tick_ms = 10;
  current_scheme = half_step;
  max_phase = 8;

  // NOTE: if we store the position in Settings
  current_phase = 0;
  current_pos = 0;
  wanted_pos = 0;
  idle = true;
  lock_when_done = false;
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv23(uint8_t function)
{
  bool result = false;
  if (function == FUNC_INIT) {
    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperMain; function='FUNC_INIT'"));
    // this sets @enabled, so it must be handled before checking that  
    StepperInit();
  }
  else if (enabled) {
    // from now we don't ever need to check @enabled
    switch (function) {
      case FUNC_COMMAND:
        AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperMain; function='FUNC_COMMAND'"));
        result = DecodeCommand(kStepperCommands, StepperCommand);
        break;
      case FUNC_LOOP:
        if (!idle && TimeReached(next_tick_time)) {
          SetNextTimeInterval(next_tick_time, tick_ms);
          StepperTick();
        }
        break;
    }
  }
  return result;
}

#endif  // USE_STEPPER
// vim: set ft=c sw=2 ts=2 fo-=ro cino=t0,c2:
