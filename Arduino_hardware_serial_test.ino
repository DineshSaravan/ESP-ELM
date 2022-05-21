#include "ELMduino.h"


#define ELM_PORT Serial2


ELM327 myELM327;

typedef enum { IDLE,
               COOLANT_TEMP,
               OIL_TEMP,
               ENG_RPM,
               THROTTLE,
               RELATIVE_THROTTLE,
               ABS_THROTTLE_POS_B,
               ABS_THROTTLE_POS_C,
               ABS_THROTTLE_POS_D,
               ABS_THROTTLE_POS_E,
               ABS_THROTTLE_POS_F,
               COMMAND_THROTTLE_ACTUATOR,
               ENGINE_LOAD,
               FUEL_TYPE,
               FUEL_INJECTION_TIMING,
               BATT_VOLT } obd_pid_states;

obd_pid_states obd_state = IDLE;
float coolant_temperature = 0.0;
float oil_temperature = 0.0;
float battery_voltage = 0.0;
float eng_rpm = 0.0;
float throttle = 0.0;
float relativeThrottle = 0.0;
float absThrottlePosB = 0.0;
float absThrottlePosC = 0.0;
float absThrottlePosD = 0.0;
float absThrottlePosE = 0.0;
float absThrottlePosF = 0.0;
float commandedThrottleActuator = 0.0;
float engineLoad = 0.0;
float fuelInjectTiming = 0.0;
uint8_t  fuelType;
uint32_t slow_data_last_time = millis();
uint32_t fast_data_last_time = millis();


void setup()
{
#if LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  Serial.begin(38400);
  ELM_PORT.begin(38400);

  Serial.println("Attempting to connect to ELM327...");

  if (!myELM327.begin(ELM_PORT, true, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner");
    while (1);
  }

  Serial.println("Connected to ELM327");
}


void loop()
{
  float tempRPM = myELM327.rpm();

  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    rpm = (uint32_t)tempRPM;
    Serial.print("RPM: "); Serial.println(rpm);
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
    myELM327.printError();

  switch (obd_state) {
    case IDLE:
      Serial.println("=========================");
      // Schedule PIDs to be read at different rates (slow PIDs and fast PIDs)
      if (millis() > slow_data_last_time + 5000) {
        slow_data_last_time = millis();
        obd_state = BATT_VOLT;
        Serial.println("IDLE, query SLOW PIDs at %.3f s\n", slow_data_last_time / 1000.0);
      } else if (millis() > fast_data_last_time + 1000) {
        fast_data_last_time = millis();
        obd_state = COOLANT_TEMP;  // Specify the first slow PID here
        Serial.println("IDLE, query FAST PIDs at %.3f s\n", fast_data_last_time / 1000.0);
      } else {
        // Serial.println("[%.3f s] IDLE\n", millis() / 1000.0);
      }
      break;

    case COOLANT_TEMP:
      coolant_temperature = myELM327.engineCoolantTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Coolant temp = %.0f\n", coolant_temperature);
        obd_state = OIL_TEMP;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = OIL_TEMP;
      }
      break;

    case OIL_TEMP:
      oil_temperature = myELM327.oilTemp();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Oil temp = %.0f\n", oil_temperature);
        obd_state = ENG_RPM;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = ENG_RPM;
      }
      break;

    case ENG_RPM:
      eng_rpm = myELM327.rpm();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Engine RPM = %.0f\n", eng_rpm);
        obd_state = THROTTLE;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = THROTTLE;
      }
      break;
      
    case THROTTLE:
      throttle = myELM327.throttle();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Throttle = %.0f\n", throttle);
        obd_state = THROTTLE;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = THROTTLE;
      }
      break;

    case RELATIVE_THROTTLE:
      relativeThrottle = myELM327.relativeThrottle();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Relative Throttle = %.0f\n", relativeThrottle);
        obd_state = ABS_THROTTLE_POS_B;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = ABS_THROTTLE_POS_B;
      }
      break;

    case ABS_THROTTLE_POS_B:
      absThrottlePosB = myELM327.absThrottlePosB();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("ABS_THROTTLE_POS_B = %.0f\n", absThrottlePosB);
        obd_state = ABS_THROTTLE_POS_C;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = ABS_THROTTLE_POS_C;
      }
      break;

    case ABS_THROTTLE_POS_C:
      absThrottlePosC = myELM327.absThrottlePosC();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("ABS_THROTTLE_POS_C = %.0f\n", absThrottlePosC);
        obd_state = ABS_THROTTLE_POS_D;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = ABS_THROTTLE_POS_D;
      }
      break;

    case ABS_THROTTLE_POS_D:
      absThrottlePosD = myELM327.absThrottlePosD();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("ABS_THROTTLE_POS_D = %.0f\n", absThrottlePosD);
        obd_state = ABS_THROTTLE_POS_E;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = ABS_THROTTLE_POS_E;
      }
      break;

    case ABS_THROTTLE_POS_E:
      absThrottlePosE = myELM327.absThrottlePosE();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("ABS_THROTTLE_POS_E = %.0f\n", absThrottlePosE);
        obd_state = ABS_THROTTLE_POS_F;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = ABS_THROTTLE_POS_F;
      }
      break;

    case ABS_THROTTLE_POS_F:
      absThrottlePosF = myELM327.absThrottlePosF();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("ABS_THROTTLE_POS_F = %.0f\n", absThrottlePosF);
        obd_state = COMMAND_THROTTLE_ACTUATOR;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = COMMAND_THROTTLE_ACTUATOR;
      }
      break;

    case COMMAND_THROTTLE_ACTUATOR:
      commandedThrottleActuator = myELM327.commandedThrottleActuator();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Command Throttle Actuator = %.0f\n", commandedThrottleActuator);
        obd_state = COMMAND_THROTTLE_ACTUATOR;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = COMMAND_THROTTLE_ACTUATOR;
      }
      break;

    case ENGINE_LOAD:
      engineLoad = myELM327.engineLoad();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Engine Load = %.0f\n", engineLoad);
        obd_state = FUEL_TYPE;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = FUEL_TYPE;
      }
      break;

    case FUEL_TYPE:
      fuelType = myELM327.fuelType();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("FUEL TYPE = %.0f\n", fuelType);
        obd_state = FUEL_INJECTION_TIMING;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = FUEL_INJECTION_TIMING;
      }
      break;

    case FUEL_INJECTION_TIMING:
      fuelInjectTiming = myELM327.fuelInjectTiming();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("FUEL INJECTION TIMING = %.0f\n", fuelInjectTiming);
        obd_state = IDLE;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = IDLE;
      }
      break;
      
    case BATT_VOLT:
      battery_voltage = myELM327.batteryVoltage();
      if (myELM327.nb_rx_state == ELM_SUCCESS) {
        Serial.println("Battery voltage = %.1f\n", battery_voltage);
        obd_state = IDLE;
      } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
        myELM327.printError();
        obd_state = IDLE;
      }
      break;
  }
}