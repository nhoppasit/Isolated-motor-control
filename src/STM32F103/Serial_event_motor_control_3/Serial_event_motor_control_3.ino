/*
PAPERX-2 
18-05-2020
There is ONLY SHUTTER.
*/
#define UNLOCK_KEY "@96331733933349c946f0d178ada618e0" // ENERGETIC-PAPERX2-042020
#define DEVICE_NAME "PAPER-X V2.3"
#define _DEBUG_SERIAL_ 0
bool _OPT_SHOW_INPUT_ = 0;
bool _OPT_SHOW_PWM_ = 0;

//
#define LIMIT_N_IDX 3
#define LIMIT_P_IDX 4
#define OPEN_ITERATION 2
#define CLOSE_SPEED1 65535
#define CLOSE_SPEED2 40000
#define OPEN_SPEED1 65535
#define ACC_TIME 250
#define DEC_TIME 250
#define CLOSE_PERIOD1 200
#define CLOSE_PERIOD2 8000
#define OPEN_PERIOD 6000
#define AUTO_OPEN_PERIOD 4000
#define HOME_SPEED 45000
#define HOME_PEROID 6000
//
// !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_INFO "?"          // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_BLINK_OFF "b"     // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_BLINK_ON "B"      // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_RELAY_OFF_ALL "r"     // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_RELAY_OFF "E"      // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_RELAY_ON "R"      // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_MOTOR_OFF "m"     // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_MOTOR_ON_CW "M"   // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_MOTOR_ON_CCW "N"  // !!! CAREFUL! CMD NEEDS TO ADD isCommand() 8
#define CMD_TEST_U "TO"       // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_HOME_U "D0"       // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_OPEN_U "D1"       // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_CLOSE_U "D2"      // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_SENSOR_U "AS"     // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_SHOW_INPUT_U "SI" // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_HIDE_INPUT_U "HI" // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_SHOW_PWM_U "SP"   // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_HIDE_PWM_U "HP"   // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_RESET_OPT_U "HA"  // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_TEST_L "to"       // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_HOME_L "d0"       // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_OPEN_L "d1"       // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_CLOSE_L "d2"      // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_SENSOR_L "as"     // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_SHOW_INPUT_L "si" // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_HIDE_INPUT_L "hi" // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_SHOW_PWM_L "sp"   // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_HIDE_PWM_L "hp"   // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define CMD_RESET_OPT_L "ha"  // !!! CAREFUL! CMD NEEDS TO ADD isCommand()
#define INDEX_1_CMD 9
#define CMD_COUNT 29
String CMDs[] = {CMD_INFO, CMD_BLINK_OFF, CMD_BLINK_ON, CMD_RELAY_OFF_ALL, CMD_RELAY_OFF, CMD_RELAY_ON, CMD_MOTOR_OFF, CMD_MOTOR_ON_CW, CMD_MOTOR_ON_CCW,
                 CMD_TEST_L,
                 CMD_HOME_L,
                 CMD_OPEN_L,
                 CMD_CLOSE_L,
                 CMD_SENSOR_L,
                 CMD_SHOW_INPUT_L,
                 CMD_SHOW_PWM_L,
                 CMD_HIDE_INPUT_L,
                 CMD_HIDE_PWM_L,
                 CMD_RESET_OPT_L,
                 CMD_TEST_U,
                 CMD_HOME_U,
                 CMD_OPEN_U,
                 CMD_CLOSE_U,
                 CMD_SENSOR_U,
                 CMD_SHOW_INPUT_U,
                 CMD_SHOW_PWM_U,
                 CMD_HIDE_INPUT_U,
                 CMD_HIDE_PWM_U,
                 CMD_RESET_OPT_U};
//
#define STX ':'
#define ETX '\n'
#define ACK '*'
#define NAK '!'
String inputString = "";     // a String to hold incoming data
bool stringComplete = false; // whether the string is complete
bool STX_COME = false;

// Unlocked flag
bool IsUnlocked = false;
//
unsigned long t0Blink = 0;
bool blinkState = 0;
bool blinkFlag = false;
int blinkTime = 500;
//
// SHUTTER PINS
int RPWM = PB6; // pwmWrite จะกำหนดค่า duty ได้ตั้งแต่ 0 - 65535 เนื่องจาก Timer บน STM32 มีขนาด 16bit
int LPWM = PB7;

int LR_EN = PB8;
int LR_EN_X = PB9;

// RELAY MODULE, KEYES
#define RELAY_COUNT 4
int RELAY_PIN[] = {PB12, PB13, PB14, PB15}; // 4 pins

// INPUT
#define INPUT_COUNT 7
#define INPUT_EDGE_TIME 10                               //ms
int INPUT_PIN[] = {PB11, PB10, PB1, PB0, PA7, PA6, PA5}; // 7 pins
bool INPUT_FLAG[] = {false, false, false, false, false, false, false};

/*
-----------------------------------------------------------------------------
SETUP
-----------------------------------------------------------------------------
*/
void setup()
{

  // PWM PIN MODE
  pinMode(RPWM, PWM);
  pinMode(LPWM, PWM);

  // OUTPUT PIN MODE
  pinMode(LR_EN, OUTPUT);
  pinMode(LR_EN_X, OUTPUT);
  for (int i = 0; i < RELAY_COUNT; i++)
  {
    pinMode(RELAY_PIN[i], OUTPUT);
  }

  // INPUT PIN MODE
  for (int i = 0; i < INPUT_COUNT; i++)
  {
    pinMode(INPUT_PIN[i], INPUT);
  }

  delay(10);
  stop_all_motor();
  delay(10);
  //
  Serial.begin(9600);
  delay(250);

  Serial.print(DEVICE_NAME);
  Serial.println(" START.");

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, 1); // LED OFF
  t0Blink = millis();
} // SETUP END.

void loop()
{
  serialEvent();
  update_input();
  blink(blinkFlag);
  blinkON(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_BLINK_ON));
  blinkOFF(IsUnlocked && stringComplete && inputString.equals(CMD_BLINK_OFF));
  unlock(stringComplete && inputString.equals(UNLOCK_KEY));
  lock(IsUnlocked && stringComplete && !(inputString.equals(UNLOCK_KEY) || isCommand(inputString)));
  info(IsUnlocked && stringComplete && inputString.equals(CMD_INFO));
  TestOutput(IsUnlocked && stringComplete && (inputString.equals(CMD_TEST_L) || inputString.equals(CMD_TEST_U)));
  home_shutter(IsUnlocked && stringComplete && (inputString.equals(CMD_HOME_L) || inputString.equals(CMD_HOME_U)),
               HOME_SPEED, HOME_SPEED, HOME_PEROID, HOME_PEROID);
  open_shutter(IsUnlocked && stringComplete && (inputString.equals(CMD_OPEN_L) || inputString.equals(CMD_OPEN_U)),
               OPEN_SPEED1, ACC_TIME, DEC_TIME, OPEN_PERIOD);
  close_shutter(IsUnlocked && stringComplete && (inputString.equals(CMD_CLOSE_L) || inputString.equals(CMD_CLOSE_U)),
                OPEN_ITERATION, CLOSE_SPEED1, CLOSE_SPEED2, ACC_TIME, DEC_TIME, CLOSE_PERIOD1, CLOSE_PERIOD2);
  talk_all_sensor(IsUnlocked && stringComplete && (inputString.equals(CMD_SENSOR_L) || inputString.equals(CMD_SENSOR_U)));
  //
  showInputs(IsUnlocked && stringComplete && (inputString.equals(CMD_SHOW_INPUT_L) || inputString.equals(CMD_SHOW_INPUT_U)));
  hideInputs(IsUnlocked && stringComplete && (inputString.equals(CMD_HIDE_INPUT_L) || inputString.equals(CMD_HIDE_INPUT_U)));
  showPWM(IsUnlocked && stringComplete && (inputString.equals(CMD_SHOW_PWM_L) || inputString.equals(CMD_SHOW_PWM_U)));
  hidePWM(IsUnlocked && stringComplete && (inputString.equals(CMD_HIDE_PWM_L) || inputString.equals(CMD_HIDE_PWM_U)));
  resetOptions(IsUnlocked && stringComplete && (inputString.equals(CMD_RESET_OPT_L) || inputString.equals(CMD_RESET_OPT_U)));
  //
  motorON_CW(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_MOTOR_ON_CW));
  motorON_CCW(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_MOTOR_ON_CCW));
  motorOFF(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_MOTOR_OFF));
  //
  relayON(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_RELAY_ON));
  relayOFF(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_RELAY_OFF));
  relayOFF_All(IsUnlocked && stringComplete && inputString.substring(0, 1).equals(CMD_RELAY_OFF_ALL));
  //
  ClearSerialEvent(stringComplete);
} // LOOP END.

#pragma region Manual MOTOR
/*
-----------------------------------------------------------------------------
MANUAL MOTOR
-----------------------------------------------------------------------------
*/
void motorON_CW(bool flag)
{
  if (flag)
  {
    Serial.println(ACK);
    int val = inputString.substring(1).toInt();
    if (val <= 0)
    {
      val = 0;
    }
    if (65535 < val)
    {
      val = 65535;
    }
    pwmWrite(LPWM, 0);
    pwmWrite(RPWM, val); //full => 65535
    digitalWrite(LR_EN, HIGH);
    digitalWrite(LR_EN_X, HIGH);
    Serial.print("MOTOR ON with CW direction of ");
    Serial.println(val);
  }
}
void motorON_CCW(bool flag)
{
  if (flag)
  {
    Serial.println(ACK);
    int val = inputString.substring(1).toInt();
    if (val <= 0)
    {
      val = 0;
    }
    if (65535 < val)
    {
      val = 65535;
    }
    pwmWrite(RPWM, 0); //full => 65535
    pwmWrite(LPWM, val);
    digitalWrite(LR_EN, HIGH);
    digitalWrite(LR_EN_X, HIGH);
    Serial.print("MOTOR ON with CCW direction of ");
    Serial.println(val);
  }
}
void motorOFF(bool flag)
{
  if (flag)
  {
    Serial.println(ACK);
    stop_all_motor();
    Serial.println("Motor stopped.");
  }
}
#pragma endregion

#pragma region Manual RELAYS
void relayON(bool flag)
{
  if (flag)
  {
    Serial.println(ACK);
    int val = inputString.substring(1).toInt();
    if (val <= 0)
    {
      val = 0;
    }
    if (4 < val)
    {
      val = 4;
    }
    digitalWrite(RELAY_PIN[val - 1], HIGH);
    Serial.print("RELAY ON for R");
    Serial.println(val);
  }
}
void relayOFF(bool flag)
{
  if (flag)
  {
    Serial.println(ACK);
    int val = inputString.substring(1).toInt();
    if (val <= 0)
    {
      val = 0;
    }
    if (4 < val)
    {
      val = 4;
    }
    digitalWrite(RELAY_PIN[val - 1], LOW);
    Serial.print("RELAY OFF for R");
    Serial.println(val);
  }
}
void relayOFF_All(bool flag)
{
  if (flag)
  {
    Serial.println(ACK);
    for (int i = 0; i < RELAY_COUNT; i++)
    {
      digitalWrite(RELAY_PIN[i], LOW);
    }
    Serial.println("All relay OFF.");
  }
}

#pragma endregion

#pragma region BLINK
/*
-----------------------------------------------------------------------------
BLINK CONTROL
-----------------------------------------------------------------------------
*/
void blinkON(bool flag)
{
  if (flag)
  {
    blinkTime = inputString.substring(1).toInt();
    if (blinkTime <= 0)
    {
      blinkTime = 500;
    }
    if (10e3 < blinkTime)
    {
      blinkTime = 10e3;
    }
    blinkFlag = true;
    Serial.println(ACK);
    Serial.print("BLINK ON with time of ");
    Serial.print(blinkTime);
    Serial.println(" ms.");
  }
}
void blinkOFF(bool flag)
{
  if (flag)
  {
    digitalWrite(PC13, 1);
    blinkFlag = false;
    Serial.println(ACK);
    Serial.println("Blink OFF.");
  }
}
//
void blink(bool flag)
{
  if (flag)
  {
    if (blinkTime == 0)
      blinkTime = 500;
    if (blinkTime < (millis() - t0Blink))
    {
      blinkState = !blinkState;
      if (blinkState)
      {
        digitalWrite(PC13, 1);
        //Serial.print("OFF");
      }
      else
      {
        digitalWrite(PC13, 0);
        //Serial2.print("ON");
      }
      t0Blink = millis();
    }
  }
}
#pragma endregion

#pragma region TEST
/*
-----------------------------------------------------------------------------
TEST OUTPUT
-----------------------------------------------------------------------------
*/
void TestOutput(bool flag)
{
  if (flag)
  {
    Serial.print(ACK);
    Serial.print("1");
    //Motor
    digitalWrite(LR_EN, HIGH);
    delay(1);
    pwmWrite(RPWM, 0);
    pwmWrite(LPWM, 20000);
    unsigned long current_t = 0;
    int period = 2000;
    unsigned long t0 = millis();
    while (true)
    {
      current_t = millis();
      if (period < current_t - t0)
      {
        break;
      }

      // SENSOR-1
      if (Serial.available())
      {
        stop_all_motor();
        Serial.print(NAK);
        Serial.println("ET");
        return; // WHILE2
      }
    }
    pwmWrite(RPWM, 0);
    pwmWrite(LPWM, 0);
    Serial.print("/");
    period = 1000;
    t0 = millis();
    while (true)
    {
      current_t = millis();
      if (period < current_t - t0)
      {
        break;
      }

      // SENSOR-1
      if (Serial.available())
      {
        stop_all_motor();
        Serial.print(NAK);
        Serial.println("ET");
        return; // WHILE2
      }
    }
    pwmWrite(RPWM, 20000);
    pwmWrite(LPWM, 0);
    Serial.print("2");
    period = 2000;
    t0 = millis();
    while (true)
    {
      current_t = millis();
      if (period < current_t - t0)
      {
        break;
      }

      // SENSOR-1
      if (Serial.available())
      {
        stop_all_motor();
        Serial.print(NAK);
        Serial.println("ET");
        return; // WHILE2
      }
    }
    digitalWrite(LR_EN, LOW);
    pwmWrite(RPWM, 0);
    pwmWrite(LPWM, 0);

    //Relay---------------------------------------------------
    Serial.print("/R/");
    for (int i = 0; i < RELAY_COUNT; i++)
    {
      period = 2000;
      t0 = millis();
      while (true)
      {
        current_t = millis();
        if (period < current_t - t0)
        {
          break;
        }

        // SENSOR-1
        if (Serial.available())
        {
          Serial.print(NAK);
          Serial.println("ET");
          break; // WHILE2
        }
      }
      digitalWrite(RELAY_PIN[i], HIGH);
      Serial.print(i);
      Serial.print("H");
      period = 2000;
      t0 = millis();
      while (true)
      {
        current_t = millis();
        if (period < current_t - t0)
        {
          break;
        }

        // SENSOR-1
        if (Serial.available())
        {
          Serial.print(NAK);
          Serial.println("ET");
          break; // WHILE2
        }
      }
      digitalWrite(RELAY_PIN[i], LOW);
      Serial.print(i);
      Serial.print("L");
    }
    for (int i = 0; i < RELAY_COUNT; i++)
    {
      digitalWrite(RELAY_PIN[i], LOW);
      Serial.print(i);
      Serial.print("L");
    }
    Serial.println("=");
  }
}

#pragma endregion

#pragma region OPTIONS
/*
-----------------------------------------------------------------------------
OPTIONs
-----------------------------------------------------------------------------
*/
void resetOptions(bool flag)
{
  if (flag)
  {
    _OPT_SHOW_INPUT_ = 0;
    _OPT_SHOW_PWM_ = 0;
    Serial.println(ACK);
  }
}
void showInputs(bool flag) // show inputs
{
  if (flag)
  {
    _OPT_SHOW_INPUT_ = 1;
    Serial.println(ACK);
  }
}
void hideInputs(bool flag) // hide inputs
{
  if (flag)
  {
    _OPT_SHOW_INPUT_ = 0;
    Serial.println(ACK);
  }
}
void showPWM(bool flag) // show PWM
{
  if (flag)
  {
    _OPT_SHOW_PWM_ = 1;
    Serial.println(ACK);
  }
}
void hidePWM(bool flag) // hide PWM
{
  if (flag)
  {
    _OPT_SHOW_PWM_ = 0;
    Serial.println(ACK);
  }
}
#pragma endregion

#pragma region INPUTS
/*
-----------------------------------------------------------------------------
SENSOR
-----------------------------------------------------------------------------
*/
void update_input()
{
  for (int i = 0; i < INPUT_COUNT; i++)
  {
    INPUT_FLAG[i] = digitalRead(INPUT_PIN[i]);
  }
  delay(10);
  for (int i = 0; i < INPUT_COUNT; i++)
  {
    INPUT_FLAG[i] = INPUT_FLAG[i] && digitalRead(INPUT_PIN[i]);
  }
  if (_OPT_SHOW_INPUT_)
  {
    for (int i = 0; i < INPUT_COUNT; i++)
    {
      Serial.print(INPUT_FLAG[i]);
    }
    Serial.println();
  }
}

void talk_all_sensor(bool flag)
{
  if (flag)
  {
    for (int i = 0; i < INPUT_COUNT; i++)
    {
      Serial.print(INPUT_FLAG[i]);
    }
    Serial.println();
  }
}

void read_input(int IDX)
{
  INPUT_FLAG[IDX] = digitalRead(INPUT_PIN[IDX]);
  delay(INPUT_EDGE_TIME);
  INPUT_FLAG[IDX] = INPUT_FLAG[IDX] && digitalRead(INPUT_PIN[IDX]);
}

#pragma endregion

/*
-----------------------------------------------------------------------------
CLOSE SHUTTER
-----------------------------------------------------------------------------
*/
// void close_shutter(bool flag, int checkLoop = 3, // Check time
//                 int Speed1 = 65535,
//                 int Speed2 = 18000, // Start up and safe speed
//                 int tAcc = 250,     // MUST BE MORE THAN THE DENOTATION
//                 int tDec = 250,     // MUST BE MORE THAN THE DENOTATION
//                 int tSpeed1 = 500,
//                 int tSpeed2 = 4000)
void close_shutter(bool flag, int checkLoop, // Check time
                   int Speed1,
                   int Speed2, // Start up and safe speed
                   int tAcc,   // MUST BE MORE THAN THE DENOTATION
                   int tDec,   // MUST BE MORE THAN THE DENOTATION
                   int tSpeed1,
                   int tSpeed2)
{
  if (flag)
  {
    int _PWM = RPWM;
    int SEN_IDX = LIMIT_P_IDX;
    bool isTimeout = true;

    unsigned long t0 = millis();
    unsigned long current_t = t0;
    unsigned long ta = t0;
    int speed = Speed1 / 5;
    int dt = tAcc / 5;

    read_input(SEN_IDX);
    if (INPUT_FLAG[SEN_IDX] || Serial.available())
    {
      stop_all_motor();
      Serial.print(ACK);
      Serial.println("CLOSE0=");
      return;
    }
    Serial.print(ACK);

    for (int ii = 0; ii < checkLoop; ii++) // AUTO close loop
    {
      Serial.print("CLOSE");
      t0 = millis();
      ta = t0;
      isTimeout = true;

      // put your main code here, to run repeatedly:
      digitalWrite(LR_EN, HIGH);

      // Acceleration
      read_input(SEN_IDX);
      while (!INPUT_FLAG[SEN_IDX])
      {
        if (_OPT_SHOW_PWM_)
          Serial.println(speed);
        pwmWrite(_PWM, speed); //<-----------------MOVE1

        current_t = millis();
        if (dt < current_t - t0)
        {
          t0 = millis();
          speed = speed + Speed1 / 5;
        }

        if (Speed1 <= speed)
          break; // WHILE1

        // SENSOR-CLOSE
        read_input(SEN_IDX);
        if (INPUT_FLAG[SEN_IDX] || Serial.available())
        {
          stop_all_motor();
          isTimeout = false;
          break; // WHILE1
        }
      }

      // Hold max speed 1
      speed = Speed1;
      if (_OPT_SHOW_PWM_)
        Serial.println(speed);
      pwmWrite(_PWM, speed); //<----------------MOVE2

      //
      t0 = millis();
      current_t = t0;
      read_input(SEN_IDX);
      while (!INPUT_FLAG[SEN_IDX])
      {
        current_t = millis();
        if (tSpeed1 - dt < current_t - t0)
        {
          break;
        }

        // SENSOR-1
        read_input(SEN_IDX);
        if (INPUT_FLAG[SEN_IDX] || Serial.available())
        {
          stop_all_motor();
          isTimeout = false;
          break; // WHILE2
        }
      }

      // Hold max speed 2
      speed = Speed2;
      if (_OPT_SHOW_PWM_)
        Serial.println(speed);
      pwmWrite(_PWM, speed); //<------------------MOVE3

      t0 = millis();
      current_t = t0;
      read_input(SEN_IDX);
      while (!INPUT_FLAG[SEN_IDX])
      {
        current_t = millis();
        if (tSpeed2 - dt < current_t - t0)
        {
          break;
        }

        // SENSOR-1
        read_input(SEN_IDX);
        if (INPUT_FLAG[SEN_IDX] || Serial.available())
        {
          stop_all_motor();
          isTimeout = false;
          break;
        }
      }

      // Deceleration
      dt = tDec / 5;
      speed = speed - Speed2 / 5;
      t0 = millis();
      while (!INPUT_FLAG[SEN_IDX])
      {
        if (_OPT_SHOW_PWM_)
          Serial.println(speed);
        pwmWrite(_PWM, speed); //<----------------MOVE4

        current_t = millis();
        if (dt < current_t - t0)
        {
          t0 = millis();
          speed = speed - Speed2 / 5;
        }
        if (speed <= 0)
          break;

        // SENSOR-1
        read_input(SEN_IDX);
        if (INPUT_FLAG[SEN_IDX] || Serial.available())
        {
          stop_all_motor();
          isTimeout = false;
          break;
        }
      }

      stop_all_motor();
      Serial.print(current_t - ta);

      // Open for secure customer
      if (isTimeout)
      {
        delay(250);
        open_shutter_2(true, OPEN_SPEED1, ACC_TIME, DEC_TIME, AUTO_OPEN_PERIOD); // >>> OPEN DOOR <<<
        delay(250);
      }
      else
      {
        break;
      }
    } // FOR 3-LOOP
    //
    //
    stop_all_motor();
    if (_OPT_SHOW_PWM_)
    {
      Serial.println();
      Serial.println("MOTOR STOP");
    }
    //
    if (isTimeout)
    {
      Serial.print(NAK);
      Serial.println("TO");
    }
    else
    {
      Serial.println("=");
    }
  }
}

/*
-----------------------------------------------------------------------------
OPEN DOOR
-----------------------------------------------------------------------------
*/
// void open_shutter(bool flag,
//                 int Speed1 = 65535,
//                 int tAcc = 250,     // MUST BE MORE THAN THE DENOTATION
//                 int tDec = 250,     // MUST BE MORE THAN THE DENOTATION
//                 int tSpeed1 = 7000)
void open_shutter(bool flag,
                  int Speed1,
                  int tAcc, // MUST BE MORE THAN THE DENOTATION
                  int tDec, // MUST BE MORE THAN THE DENOTATION
                  int tSpeed1)
{
  if (flag)
  {
    int _PWM = LPWM;
    int SEN_IDX = LIMIT_N_IDX;
    bool isTimeout = true;

    unsigned long t0 = millis();
    unsigned long current_t = t0;
    unsigned long ta = t0;
    int speed = Speed1 / 5;
    int dt = tAcc / 5;

    read_input(SEN_IDX);
    if (INPUT_FLAG[SEN_IDX] || Serial.available())
    {
      stop_all_motor();
      Serial.print(ACK);
      Serial.println("OPEN0=");
      return;
    }
    Serial.print(ACK);
    Serial.print("OPEN");

    // put your main code here, to run repeatedly:
    digitalWrite(LR_EN, HIGH);

    // Acceleration
    t0 = millis();
    read_input(SEN_IDX);
    while (!INPUT_FLAG[SEN_IDX])
    {
      if (_OPT_SHOW_PWM_)
        Serial.println(speed);
      pwmWrite(_PWM, speed); //<-----------------MOVE1

      current_t = millis();
      if (dt < current_t - t0)
      {
        t0 = millis();
        speed = speed + Speed1 / 5;
      }

      if (Speed1 <= speed)
        break; // WHILE1

      // SENSOR-CLOSE
      read_input(SEN_IDX);
      if (INPUT_FLAG[SEN_IDX] || Serial.available())
      {
        stop_all_motor();
        isTimeout = false;
        break; // WHILE1
      }
    }

    // Hold max speed 1
    speed = Speed1;
    if (_OPT_SHOW_PWM_)
      Serial.println(speed);
    pwmWrite(_PWM, speed); //<----------------MOVE2

    //
    t0 = millis();
    current_t = t0;
    read_input(SEN_IDX);
    while (!INPUT_FLAG[SEN_IDX])
    {
      current_t = millis();
      if (tSpeed1 - dt < current_t - t0)
      {
        break;
      }

      // SENSOR-1
      read_input(SEN_IDX);
      if (INPUT_FLAG[SEN_IDX] || Serial.available())
      {
        stop_all_motor();
        isTimeout = false;
        break; // WHILE2
      }
    }

    // Deceleration
    dt = tDec / 5;
    speed = speed - Speed1 / 5;
    t0 = millis();
    while (!INPUT_FLAG[SEN_IDX])
    {
      if (_OPT_SHOW_PWM_)
        Serial.println(speed);
      pwmWrite(_PWM, speed); //<----------------MOVE4

      current_t = millis();
      if (dt < current_t - t0)
      {
        t0 = millis();
        speed = speed - Speed1 / 5;
      }
      if (speed <= 0)
        break;

      // SENSOR-1
      read_input(SEN_IDX);
      if (INPUT_FLAG[SEN_IDX] || Serial.available())
      {
        stop_all_motor();
        isTimeout = false;
        break;
      }
    }

    Serial.print(current_t - ta);

    //
    //
    stop_all_motor();
    if (_OPT_SHOW_PWM_)
    {
      Serial.println();
      Serial.println("MOTOR STOP");
    }
    //
    if (isTimeout)
    {
      Serial.print(NAK);
      Serial.println("TO");
    }
    else
    {
      Serial.println("=");
    }
  }
} // END OPEN SHUTTER.

/*
-----------------------------------------------------------------------------
SUDDEN OPEN DOOR
-----------------------------------------------------------------------------
*/
// void open_shutter_2(bool flag,
//                 int Speed1 = 65535,
//                 int tAcc = 250,     // MUST BE MORE THAN THE DENOTATION
//                 int tDec = 250,     // MUST BE MORE THAN THE DENOTATION
//                 int tSpeed1 = 7000)
void open_shutter_2(bool flag,
                    int Speed1,
                    int tAcc, // MUST BE MORE THAN THE DENOTATION
                    int tDec, // MUST BE MORE THAN THE DENOTATION
                    int tSpeed1)
{
  if (flag)
  {
    int _PWM = LPWM;
    int SEN_IDX = LIMIT_N_IDX;
    bool isTimeout = true;

    unsigned long t0 = millis();
    unsigned long current_t = t0;
    unsigned long ta = t0;
    int speed = Speed1 / 5;
    int dt = tAcc / 5;

    read_input(SEN_IDX);
    if (INPUT_FLAG[SEN_IDX] || Serial.available())
    {
      stop_all_motor();
      Serial.print(ACK);
      Serial.print("OPEN0=");
      return;
    }

    Serial.print(ACK);
    Serial.print("OPEN");

    // put your main code here, to run repeatedly:
    digitalWrite(LR_EN, HIGH);

    // Acceleration
    t0 = millis();
    read_input(SEN_IDX);
    while (!INPUT_FLAG[SEN_IDX])
    {
      if (_OPT_SHOW_PWM_)
        Serial.println(speed);
      pwmWrite(_PWM, speed); //<-----------------MOVE1

      current_t = millis();
      if (dt < current_t - t0)
      {
        t0 = millis();
        speed = speed + Speed1 / 5;
      }

      if (Speed1 <= speed)
        break; // WHILE1

      // SENSOR-CLOSE
      read_input(SEN_IDX);
      if (INPUT_FLAG[SEN_IDX] || Serial.available())
      {
        stop_all_motor();
        isTimeout = false;
        break; // WHILE1
      }
    }

    // Hold max speed 1
    speed = Speed1;
    if (_OPT_SHOW_PWM_)
      Serial.println(speed);
    pwmWrite(_PWM, speed); //<----------------MOVE2

    //
    t0 = millis();
    current_t = t0;
    read_input(SEN_IDX);
    while (!INPUT_FLAG[SEN_IDX])
    {
      current_t = millis();
      if (tSpeed1 - dt < current_t - t0)
      {
        break;
      }

      // SENSOR-1
      read_input(SEN_IDX);
      if (INPUT_FLAG[SEN_IDX] || Serial.available())
      {
        stop_all_motor();
        isTimeout = false;
        break; // WHILE2
      }
    }

    // Deceleration
    dt = tDec / 5;
    speed = speed - Speed1 / 5;
    t0 = millis();
    while (!INPUT_FLAG[SEN_IDX])
    {
      if (_OPT_SHOW_PWM_)
        Serial.println(speed);
      pwmWrite(_PWM, speed); //<----------------MOVE4

      current_t = millis();
      if (dt < current_t - t0)
      {
        t0 = millis();
        speed = speed - Speed1 / 5;
      }
      if (speed <= 0)
        break;

      // SENSOR-1
      read_input(SEN_IDX);
      if (INPUT_FLAG[SEN_IDX] || Serial.available())
      {
        stop_all_motor();
        isTimeout = false;
        break;
      }
    }

    Serial.print(current_t - ta);

    //
    //
    stop_all_motor();
    if (_OPT_SHOW_PWM_)
    {
      Serial.println();
      Serial.println("MOTOR STOP");
    }
    //
    if (isTimeout)
    {
      Serial.print("/TO/");
    }
    else
    {
      Serial.print("/");
    }
  }
}

// /*
// -----------------------------------------------------------------------------
// HOME SHUTTER
// -----------------------------------------------------------------------------
// */
// void home_shutter(bool flag,
//                   int openSpeed = 45000, //
//                   int closeSpeed = 45000,
//                   int tAcc = 250, // MUST BE MORE THAN THE DENOTATION
//                   int tDec = 250, // MUST BE MORE THAN THE DENOTATION
//                   int tOpen = 8000,
//                   int tClose = 8000)
void home_shutter(bool flag,
                  int openSpeed, //
                  int closeSpeed,
                  int tOpen,
                  int tClose)
{
  if (flag)
  {
    int _PWM = LPWM;
    Serial.print(ACK);
    Serial.print("OPEN");

    // put your main code here, to run repeatedly:
    digitalWrite(LR_EN, HIGH);

    unsigned long t0 = millis();
    unsigned long current_t = t0;
    unsigned long ta = t0;
    int speed = openSpeed / 5;

    // Hold max speed
    speed = openSpeed;
    pwmWrite(_PWM, speed); //<---------------------------------MOVE OPEN

    t0 = millis();
    current_t = t0;
    read_input(LIMIT_N_IDX);
    while (!INPUT_FLAG[LIMIT_N_IDX])
    {
      current_t = millis();
      if (tOpen < current_t - t0)
      {
        //break;
        stop_all_motor();
        Serial.print(current_t - ta);
        Serial.print(NAK);
        Serial.println("TO");
        return;
      }
      //
      read_input(LIMIT_N_IDX);
      if (INPUT_FLAG[LIMIT_N_IDX])
      {
        stop_all_motor();
        break;
      }
      //
      if (Serial.available())
      {
        stop_all_motor();
        Serial.print(current_t - ta);
        Serial.println("=");
        return;
      }
    }
    stop_all_motor();
    Serial.print(current_t - ta);
    Serial.print("/");
    // DELAY-------------------------------
    t0 = millis();
    current_t = t0;
    ta = t0;
    while (true)
    {
      current_t = millis();
      if (1000 < current_t - t0)
      {
        break;
      }
      //
      if (Serial.available())
      {
        Serial.print(current_t - ta);
        Serial.println("=");
        return;
      }
    }
    Serial.print(current_t - ta);
    Serial.print(ACK);
    Serial.print("CLOSE");
    // CLOSE--------------------------------
    _PWM = RPWM;
    digitalWrite(LR_EN, HIGH);

    // Hold max speed
    speed = closeSpeed;
    pwmWrite(_PWM, speed); //<---------------------------------MOVE CLOSE

    t0 = millis();
    current_t = t0;
    ta = t0;
    read_input(LIMIT_P_IDX);
    while (!INPUT_FLAG[LIMIT_P_IDX])
    {
      current_t = millis();
      if (tClose < current_t - t0)
      {
        //break;
        stop_all_motor();
        Serial.print(current_t - ta);
        Serial.print(NAK);
        Serial.println("TO");
        return;
      }
      //
      read_input(LIMIT_P_IDX);
      if (INPUT_FLAG[LIMIT_P_IDX])
      {
        stop_all_motor();
        break;
      }
      //
      if (Serial.available())
      {
        stop_all_motor();
        Serial.print(current_t - ta);
        Serial.println("=");
        return;
      }
    }

    stop_all_motor();
    Serial.print(current_t - ta);
    Serial.println("=");
  }
} // home_door END.

/*
-----------------------------------------------------------------------------
STOP MOTOR
-----------------------------------------------------------------------------
*/
void stop_all_motor()
{
  // Stop
  pwmWrite(RPWM, 0); //full => 65535
  pwmWrite(LPWM, 0);
  digitalWrite(LR_EN, LOW);
  digitalWrite(LR_EN_X, LOW);
}

#pragma region LOCKING

/*
-----------------------------------------------------------------------------
UNLOCK DEVICE
-----------------------------------------------------------------------------
*/
bool isCommand(String str)
{
  bool flag = false;
  for (int i = 0; i < INDEX_1_CMD; i++)
  {
    if (str.substring(0, 1).equals(CMDs[i]))
    {
      flag = true;
      //
      // Serial.print("Command: ");
      // Serial.println(str);
      // Serial.println(flag);
      return flag;
    }
  }
  for (int i = INDEX_1_CMD; i < CMD_COUNT; i++)
  {
    if (str.equals(CMDs[i]))
    {
      flag = true;
      //
      // Serial.print("Command: ");
      // Serial.println(str);
      // Serial.println(flag);
      return flag;
    }
  }
}
void unlock(bool flag)
{
  if (flag)
  {
    IsUnlocked = true;
    blinkFlag = true;
    Serial.print(ACK);
    Serial.println(DEVICE_NAME);
  }
} // UNLOCK END.

void lock(bool flag)
{
  if (flag)
  {
    IsUnlocked = false;
    blinkFlag = false;
    blinkOFF(true);
    _OPT_SHOW_INPUT_ = false;
    _OPT_SHOW_PWM_ = false;
    Serial.print(NAK);
    Serial.println("UL");
  }
} // Lock Notification END.

#pragma endregion

#pragma region SERIAL PORT / UART

/*
-----------------------------------------------------------------------------
ECHO DEVICE INFO
-----------------------------------------------------------------------------
*/
void info(bool flag)
{
  if (flag)
  {
    Serial.print(DEVICE_NAME);
    Serial.println();
  }
} // INFO END.

/*
------------------------------------------------------------------------
 Serial event
------------------------------------------------------------------------
*/
void serialEvent()
{
  if (Serial.available())
  {

    // get the new byte:
    char inChar = (char)Serial.read();
#if _DEBUG_SERIAL_
    Serial.println(inChar);
#endif

    _OPT_SHOW_INPUT_ = false;
    _OPT_SHOW_PWM_ = false;
    stop_all_motor();

    // add it to the inputString:
    if (STX_COME)
    {
      if (inChar == ETX)
      {
        stringComplete = true;
#if _DEBUG_SERIAL_
        Serial.println("ETX come.");
#endif
        return;
      }
      if (inChar != STX && inChar != '\r' && inChar != ETX)
      {
        inputString += inChar;
      }
      return;
    }

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == STX)
    {
      STX_COME = true;
      stringComplete = false;
      inputString = "";
#if _DEBUG_SERIAL_
      Serial.println("STX come.");
#endif
      return;
    }

    if (!STX_COME && IsUnlocked && inChar != ETX && inChar != '\r')
    {
      IsUnlocked = false;
      blinkFlag = false;
      blinkOFF(true);
      Serial.print(NAK);
      Serial.println("CL");
      return;
    }
  }
}
//
void ClearSerialEvent(bool flag)
{
  if (flag)
  {
#if _DEBUG_SERIAL_
    Serial.println("Clear serial event.");
#endif
    STX_COME = false;
    stringComplete = false;
    inputString = "";
  }
}

#pragma endregion
