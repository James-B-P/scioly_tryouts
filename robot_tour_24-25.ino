// ROBOT INSTRUCTIONS: Should be the only modified part during competitions. Path is programmed
// as a matrix of points it passes. Currently, adjacent points must share either an x value or
// a y value.
// INSTRUCTIONS_LEN must be equal to the number of points in the instructions matrix.
const int INSTRUCTIONS_LEN = 10; 
// Example matrix of instructions to trace a figure eight:
const int instructions[INSTRUCTIONS_LEN][2] = 
{
  {  0,    0},
  { 16,    0},
  { 16,   32},
  {-16,   32},
  {-16,    0},
  { 16,    0},
  { 16,  -32},
  {-16,  -32},
  {-16,    0},
  {  0,    0}
};
// Robot's starting position and orientation
volatile int robot_direction = 0;
volatile int robot_x = 0;
volatile int robot_y = 0;

// Pin Constants
const int MLE = 7;
const int MLA = 6;
const int MLB = 5;
const int MRA = 4;
const int MRB = 3;
const int MRE = 2;
const int INL = 18;
const int INR = 19;
const int BUTTON = 21;

// Motor control constants
const float K_P = 0.125;
const float K_I = 0.000005;
const float K_D = 250;
const int DELAY = 50;
const float L2R_RATIO = 0.7;
const float TICKS_PER_CM = 1;

// Pulse constants
const int PULSE_POWER = 255;
const int PULSE_DELAY = 50;

// Motor class: Used to simplify interactions between program and physical motors
class Motor
{
  public:
    Motor(int InA, int InB, int En):InA(InA), InB(InB), En(En){};
    volatile float power = 0;
    volatile int ticks = 0;
    volatile float prev_power = 1;

    // Updates signal to L298N
    void run()
    {
      analogWrite(En, (int)(power));
      digitalWrite(InA, LOW);
      digitalWrite(InB, HIGH);
      if (power < 0)
      {
        analogWrite(En, (int)(-power));
        digitalWrite(InA, HIGH);
        digitalWrite(InB, LOW);
      }
    }

    void set_power(float newPower)
    {
      if (newPower == 0 && power != 0)
        prev_power = power;
      power = min(255, max(-255, newPower));
      run();
    }

    void inc_power(float delta)
    {
      set_power(power+delta);
    }

    void dec_power(float delta)
    {
      set_power(power-delta);
    }

  private:
    const int InA;
    const int InB;
    const int En;
};

// Motor setup
Motor left_motor(MLA, MLB, MLE);
Motor right_motor(MRA, MRB, MRE);

// Tracks wheel encoder ticks
void incL()
{
  if (left_motor.power == 0)
  {
    left_motor.ticks += abs(left_motor.prev_power)/left_motor.prev_power;
  } else
  {
    left_motor.ticks += abs(left_motor.power)/left_motor.power;
  }
}

void incR()
{
  if (right_motor.power == 0)
  {
    right_motor.ticks += abs(right_motor.prev_power)/right_motor.prev_power;
  } else
  {
    right_motor.ticks += abs(right_motor.power)/right_motor.power;
  }
}

int left_ticks()
{
  return left_motor.ticks;
}

int right_ticks()
{
  return right_motor.ticks;
}

// Gives a short pulse to the motors to either quickly slow down or speed up
//   • left_direction and right_direction are ints either -1 or 1
void retro_pulse(int left_direction, int right_direction)
{
  left_motor.set_power(left_direction*PULSE_POWER);
  right_motor.set_power(right_direction*PULSE_POWER);
  delay(PULSE_DELAY);
  left_motor.set_power(0);
  right_motor.set_power(0);
}

// Using PID motor control, rotates motors in a given direction for a given length
//   • left_direction and right_direction are ints either -1 or 1
//   • avg_power is an int between 0 and 255
//   • total_ticks is a positive int corresponding to the total number of wheel encoder 
//     ticks counted in the movement
void move_PID(int left_direction, int right_direction, int avg_power, int total_ticks)
{
  int avg_ticks;
  int tick_dif;
  int accum_tick_dif;
  float PID;
  int prev_tick_dif;

  left_motor.ticks = 0;
  right_motor.ticks = 0;
  avg_ticks = 0;
  accum_tick_dif = 0;

  retro_pulse(left_direction, right_direction);

  left_motor.set_power(left_direction*avg_power*L2R_RATIO);
  right_motor.set_power(right_direction*avg_power);

  while (avg_ticks < total_ticks)
  {
    // Encoder ticks update
    avg_ticks = (left_ticks()*left_direction+right_ticks()*right_direction)/2;
    
    // PID control
    tick_dif = left_direction*left_ticks()-right_direction*right_ticks();
    accum_tick_dif += tick_dif;
    PID = K_P*tick_dif + K_I*accum_tick_dif*DELAY + K_D*(tick_dif-prev_tick_dif)/DELAY;
    prev_tick_dif = tick_dif;
    left_motor.dec_power(left_direction*PID);
    right_motor.inc_power(right_direction*PID);

    // Delay
    delay(DELAY);
  }

  retro_pulse(-left_direction, -right_direction);  
}

void forward(int cm)
{
  move_PID(1, 1, 100, (int)(TICKS_PER_CM*cm));
  delay(200);
}

void turn(int degrees_clockwise)
{
  if (degrees_clockwise > 0)
  {
    move_PID(1, -1, 100, degrees_clockwise*17/180);
  } else
  {
    move_PID(-1, 1, 100, -degrees_clockwise*17/180);
  } 
  delay(200);
}

// Lines robot up for next movement
//   • new_direction is an int between 0 and 3 with each value corresponding
//     to a different direction by increments of 90°
void rotate_to(int new_direction)
{
  int dir_change = (new_direction-robot_direction+2)%4-2;
  if (dir_change != 0)
  {
    turn(90*dir_change);
  }
  robot_direction += dir_change;
}

volatile bool paused = true;

void startProgram()
{
  paused = false;
}

void setup()
{
  // General setup and wait for button press to start program
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(INL), incL, RISING);
  attachInterrupt(digitalPinToInterrupt(INR), incR, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON), startProgram, FALLING);
  while (paused){}

  delay(1000);
  int x_move;
  int y_move;

  // Iterate through instructions matrix and move accordingly
  for (int i = 0; i < INSTRUCTIONS_LEN; i++)
  {
    x_move = instructions[i][0]-robot_x;
    y_move = instructions[i][1]-robot_y;

    if (x_move > 0)
    {
      rotate_to(1);
      forward(x_move);
    } else if (x_move < 0)
    {
      rotate_to(3);
      forward(-x_move);
    } else if (y_move > 0)
    {
      rotate_to(0);
      forward(y_move);
    } else if (y_move < 0)
    {
      rotate_to(2);
      forward(-y_move);
    }
    robot_x += x_move;
    robot_y += y_move;
  }
}

void loop()
{

}
