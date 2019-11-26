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

/* NOTE:
  Precise output timing cannot be done using FUNC_LOOP, because the framework
  will block when it handles wifi traffic, therefore we reuse Timer1 for this
  purpose.

  Timer1 is normally driving the PWMs, so this motor control and the PWMs are
  *mutually exclusive*.

  On the other hand, we can then reuse the PWM-related settings as well, so
  no new setting allocation is needed.
*/

// pin mapping
uint8_t stepper_A_ena_pin;
uint8_t stepper_A_pos_pin;
uint8_t stepper_A_neg_pin;

uint8_t stepper_B_ena_pin;
uint8_t stepper_B_pos_pin;
uint8_t stepper_B_neg_pin;

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

// Stepper control control_schemes
// See http://users.ece.utexas.edu/~valvano/Datasheets/Stepper_ST.pdf

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
         ______________                          _
     A _|              |____.____.____.____.____|
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

struct stepper_control_scheme_t {
  stepper_state_t const *phases;
  uint8_t max_phase;
};

stepper_control_scheme_t const control_schemes[] PROGMEM = {
  { rotating_wave, sizeof(rotating_wave) / sizeof(stepper_state_t) },
  { full_step, sizeof(full_step) / sizeof(stepper_state_t) },
  { half_step, sizeof(half_step) / sizeof(stepper_state_t) },
  { half_step_inhibit, sizeof(half_step_inhibit) / sizeof(stepper_state_t) }
};
#define SCHEMES_MAX (sizeof(control_schemes) / sizeof(stepper_control_scheme_t))

// transient state variables
bool enabled = false;
bool have_enable_pins;
bool idle;
int8_t current_phase;
int16_t wanted_pos;
bool lock_when_done;

// settings + persistent state variables
uint16_t tick_us;
uint8_t current_scheme_index;
stepper_control_scheme_t current_scheme;
int16_t current_pos;


void SaveCurrentPos() {
  Settings.pwm_value[0] = (uint8_t)( current_pos       & 0xff);
  Settings.pwm_value[1] = (uint8_t)((current_pos >> 8) & 0xff);
}


/* NOTE:
  PlatformIO generates declarations for our functions to the beginning of the
  merged giant .cpp source, but that's way sooner than our type definitions
  here, so if we use a locally defined type in a function argument, then it
  will generate errors.

  To prevent this, we need to outsmart PlatformIOs function recognition
  pattern, and one way to do it is to make the return type consist of more
  than two words, like 'static const void'.

  It seems that this forced generation of declarations is causing worse issues
  than what it is intended to solve. And, trying to 'parse' C++ with regexes is
  not the best idea...
*/
static const void StepperSet(const stepper_state_t &st_P) {
  stepper_state_t st;
  memcpy_P(&st, &st_P, sizeof(stepper_state_t)); // that's one byte

  // reducing the transitional hazards:
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


void ICACHE_RAM_ATTR stepper_timer_isr(void)
{
  // ~/.platformio/packages/framework-arduinoespressif8266/cores/esp8266/esp8266_peri.h
  TEIE &= ~TEIE1;
  T1I = 0;

  if (wanted_pos < current_pos) {
    --current_phase;
    if (current_phase < 0) {
      current_phase = current_scheme.max_phase - 1;
    }
    --current_pos;
    StepperSet(current_scheme.phases[current_phase]);
  }
  else if (wanted_pos > current_pos) {
    ++current_phase;
    if (current_phase >= current_scheme.max_phase) {
      current_phase = 0;
    }
    ++current_pos;
    StepperSet(current_scheme.phases[current_phase]);
  }
  else {
    // needed for the case when we won't move it, just change the lockedness
    StepperSet(lock_when_done ? current_scheme.phases[current_phase] : all_off);
    SaveCurrentPos();
    idle = true;
  }

  if (!idle) {
    T1L = (ESP8266_CLOCK / 1000000) * tick_us;
    // T1L has 23 bits, the multiplier is 80 (-> 7 bits), so
    // 23 - 7 = 16 bits are indeed enough for @tick_us
    TEIE |= TEIE1;
  }
}


bool SelectScheme(int n) {
  if ((n < 0) || (n >= SCHEMES_MAX)) {
    return false;
  }
  if (n == current_scheme_index) {
    return true;
  }

  TEIE &= ~TEIE1;
  current_scheme_index = n;
  wanted_pos = current_pos; // stop moving
  current_phase = 0;
  memcpy_P(&current_scheme, &control_schemes[n], sizeof(stepper_control_scheme_t));
  Settings.pwm_value[5] = current_scheme_index;
  if (!idle) {
    TEIE |= ~TEIE1;
  }
  return true;
}


void StepperCmndCalibrate(void)
{
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperCmndCalibrate;"));
  // assume the current state as position 0
  TEIE &= ~TEIE1;
  T1L = 0;
  current_pos = 0;
  wanted_pos = 0;
  idle = true;
  StepperSet(all_off);
  SaveCurrentPos();
  ResponseCmndDone();
}


void StepperCmndSpeed(void)
{
  // arguments: tick_us, scheme code
  char *buffer = XdrvMailbox.data;
  uint32_t buffer_length = XdrvMailbox.data_len;
  char *p, *str, *tok;
  uint16_t req_tick_us;
  int scheme_code = -1;

  buffer[buffer_length] = '\0';
  do {
    // parse @tick_us
    str = strtok_r(buffer, ",", &tok);
    if (!str)
      break;
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      req_tick_us = (uint16_t)strtol(str, &p, 0);
      if (p == str) {
        ResponseCmndChar(D_ERROR);
        return;
      }
      if (req_tick_us != tick_us) {
        Settings.pwm_frequency = tick_us = req_tick_us;
      }
    }

    // parse @scheme_code
    str = strtok_r(nullptr, ",", &tok);
    if (!str)
      break;
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      scheme_code = (int)strtol(str, &p, 0);
      if (p == str) {
        ResponseCmndChar(D_ERROR);
        return;
      }
      if (!SelectScheme(scheme_code)) {
        ResponseCmndChar(D_ERROR);
        return;
      }
    }
  } while (0);
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperCmndSpeed; tick_us=%d, scheme=%d"), req_tick_us, scheme_code);
  ResponseCmndDone();
}

void StepperCmndStep(void)
{
  // arguments: req_pos, is_absolute, lock_when_done
  char *buffer = XdrvMailbox.data;
  uint32_t buffer_length = XdrvMailbox.data_len;
  char *p, *str, *tok;

  // argument defaults (no args: return home and unlock)
  bool is_absolute = true;
  int16_t req_pos = 0;

  lock_when_done = false;

  buffer[buffer_length] = '\0';
  do {
    // parse @req_pos
    str = strtok_r(buffer, ",", &tok);
    if (!str)
      break;
    for (; *str && isspace(*str); ++str)
      ;
    if (*str) {
      req_pos = (int16_t)strtol(str, &p, 0);
      if (p == str) {
        ResponseCmndChar(D_ERROR);
        return;
      }
    }

    // parse @is_absolute
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
      is_absolute = (i != 0);
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
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperCmndStep; is_absolute=%d, req_pos=%d, lock_when_done=%d"), is_absolute, req_pos, lock_when_done);

  TEIE &= ~TEIE1;
  wanted_pos = is_absolute ? req_pos : current_pos + req_pos;
  if (idle) {
    idle = false;
    T1L = 1;
  }
  TEIE |= TEIE1;

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

  enabled = (stepper_A_pos_pin < 99) && (stepper_A_neg_pin < 99) && (stepper_B_pos_pin < 99) && (stepper_B_neg_pin < 99);
  have_enable_pins = enabled && (stepper_A_ena_pin < 99) && (stepper_B_ena_pin < 99);

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

    TEIE &= ~TEIE1;
    timer1_disable();
    ETS_FRC_TIMER1_INTR_ATTACH(NULL, NULL);
    ETS_FRC_TIMER1_NMI_INTR_ATTACH(stepper_timer_isr);
    timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
  }

  idle = true;
  wanted_pos = 0;
  lock_when_done = false;

  // reuse Settings.pwm_frequency for @tick_us
  tick_us = Settings.pwm_frequency;
  // reuse Settings.pwm_value[0..1] for current_pos
  current_pos = (int16_t)(Settings.pwm_value[0] + (((uint16_t)Settings.pwm_value[1]) << 8));
  // reuse Settings.pwm_value[5] for control scheme index
  if (!SelectScheme(Settings.pwm_value[5])) {
    SelectScheme(1);
  }
  // still free pwm-related: pwm_value[2..4], pwm_range
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

/*
const char HTTP_MSG_SLIDER1[] PROGMEM =
  "<div><span class='p'>" D_COLDLIGHT "</span><span class='q'>" D_WARMLIGHT "</span></div>"
  "<div><input type='range' min='153' max='500' value='%d' onchange='lc(value)'></div>";
const char HTTP_MSG_SLIDER2[] PROGMEM =
  "<div><span class='p'>" D_DARKLIGHT "</span><span class='q'>" D_BRIGHTLIGHT "</span></div>"
  "<div><input type='range' min='1' max='100' value='%d' onchange='lb(value)'></div>";
*/
bool Xdrv23(uint8_t function)
{
  bool result = false;
  switch (function) {
    case FUNC_INIT:
      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperMain; function='FUNC_INIT'"));
      StepperInit();
      break;
    case FUNC_WEB_ADD_MAIN_BUTTON:
      //WSContentSpaceButton(BUTTON_FIRMWARE_UPGRADE); // button with space above
      //WSContentButton(BUTTON_INFORMATION); // plain button
      //WSContentSend_P(PSTR("<tr>"));
      //WSContentSend_P(HTTP_MSG_SLIDER1, LightGetColorTemp());
      //WSContentSend_P(HTTP_MSG_SLIDER2, Settings.light_dimmer);
      WSContentSend_P(PSTR("<div></div>"));            // 5px padding
      WSContentSend_P(PSTR("<p><form action=\"javascript:la('%s')\" method='get'><button>%s</button></form></p>"),
          "/cm?cmnd=step%2032767", "Up");
      WSContentSend_P(PSTR("<p><form action=\"javascript:la('%s')\" method='get'><button>%s</button></form></p>"),
          "/cm?cmnd=step%200,1", "Stop");
      WSContentSend_P(PSTR("<p><form action=\"javascript:la('%s')\" method='get'><button>%s</button></form></p>"),
          "/cm?cmnd=step%20-32768", "Down");
      break;
    case FUNC_COMMAND:
      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("StepperMain; function='FUNC_COMMAND'"));
      if (enabled) {
        result = DecodeCommand(kStepperCommands, StepperCommand);
      }
      break;
  }
  return result;
}

#endif  // USE_STEPPER
// vim: set ft=c sw=2 ts=2 fo-=ro cino=t0,c2:
