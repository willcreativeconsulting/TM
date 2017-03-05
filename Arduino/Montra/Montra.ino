#include <Servo.h>

//---------------------Motor do relogio
#define enc_a 2 
#define enc_b 3
#define out_a 6
#define out_b 7

//---------------------Motor da ampulheta
#define enc_c 20 
#define enc_d 21
#define out_c 8
#define out_d 9

#define motor_enable            A7
#define motor_enable_ampulheta  A6
#define position_0_motor        12
#define position_0_ampulheta    11

//--------------------Reles
#define main_switch A0
#define relay_0 16
#define relay_1 17


#define STEPS_PER_REV 1920
#define MAX_SPEED_PER_REV 400 //value in millis 
#define TIME_CONSTANT 100

#define POS_TOLERANCE 10
#define AMPULHETA_STOPTIME_SECONDS 5

static long  enc_count[2]         = {0,0};
static float rev_sec[2]           = {0,0};
static float deg_position[2]      = {0,0};
static float desired_position[2]  = {0,0};
static float desired_speed[2]     = {0,0};
static bool  desired_position_reached[2]  = {false, false};
static bool  calibrate_position[2]        = {false, false};

static long prev_enc_count[2]     = {0,0};
static unsigned long prev_time[2] = {0,0};

static int prev_state[2] = {0,0};

#define K_P 0.5
#define K_I 0.1
#define K_D 0.5
#define K_PWM 95  // 255/max_speed 

static float error_i[2]       = {0,0};
static float prev_error_p[2]  = {0,0};

Servo lever[2];
static unsigned long timer_servo[2];
static int lever_deg[2]       = {0,0};
static int lever_pin[2]       = {14,15};
static int prev_lever_deg[2]  = {0,0};
static bool lever_working[2]  = {false, false};

static int year_received      = 0;
static int last_year_received = 0;

#define BUFFER_SIZE 6
static char rx_buffer[BUFFER_SIZE];

enum states 
{ 
  START,
  WAITING_NEW_YEAR,
  WAITING_TO_REACH,
  SET_LEVERS_FORWARD,
  SET_LEVERS_FORWARD_WAITING,
  SET_LEVERS_BACK_WAITING,
  AMPULHETA_0,
  AMPULHETA_180,
};

#define _1910_POS   36
#define _1920_POS   36*2
#define _1930_POS   36*3
#define _1940_POS   36*4
#define _1950_POS   36*5
#define _1960_POS   36*6
#define _1970_POS   36*7
#define _1980_POS   36*8

#define AMPULHETA_HIGH  0
#define AMPULHETA_LOW   180

#define MOTOR_RELOGIO   0
#define MOTOR_AMPULHETA 1

#define SERVO_1   0
#define SERVO_2   1

static enum states sm_motor[2] = {START, START};
static unsigned long sm_timer[2] = {millis(), millis()};

void setup() 
{
  //Serial that will be use to comunicate with the Raspberry
  Serial.begin(9600);

  //Serial that will be use to comunicate with the 7 segments display micro
  Serial1.begin(9600);
 
  pinMode(enc_a, INPUT);
  pinMode(enc_b, INPUT);

  pinMode(enc_c, INPUT);
  pinMode(enc_d, INPUT);

  pinMode(position_0_motor, INPUT);
  pinMode(position_0_ampulheta, INPUT);
  pinMode(main_switch, INPUT_PULLUP);


  //link the interupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(enc_a), update_enc_relogio, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_b), update_enc_relogio, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(enc_c), update_enc_ampulheta, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_d), update_enc_ampulheta, CHANGE);
  
  pinMode(out_a, OUTPUT);
  pinMode(out_b, OUTPUT); 
  
  pinMode(out_c, OUTPUT);
  pinMode(out_d, OUTPUT);

  pinMode(motor_enable, OUTPUT);
  pinMode(motor_enable_ampulheta, OUTPUT);

  pinMode(relay_0, OUTPUT);
  pinMode(relay_1, OUTPUT);


}

//Function that will be associated with the encoder interrupts (Relogio)
void update_enc_relogio()
{
  int actual_state = 0;

  actual_state = digitalRead(enc_b) | (digitalRead(enc_a)<<1);
  
  if(actual_state != prev_state[MOTOR_RELOGIO]){
    switch(actual_state){
      case 0:
        if(prev_state[MOTOR_RELOGIO] == 1)
          enc_count[MOTOR_RELOGIO]++;
        else
          enc_count[MOTOR_RELOGIO]--;
      break;
      case 1:
        if(prev_state[MOTOR_RELOGIO] == 3)
          enc_count[MOTOR_RELOGIO]++;
        else
          enc_count[MOTOR_RELOGIO]--;
      break;
      case 2:
        if(prev_state[MOTOR_RELOGIO] == 0)
          enc_count[MOTOR_RELOGIO]++;
        else
          enc_count[MOTOR_RELOGIO]--;
      break;
      case 3:
        if(prev_state[MOTOR_RELOGIO] == 2)
          enc_count[MOTOR_RELOGIO]++;
        else
          enc_count[MOTOR_RELOGIO]--;
      break;
    }
    
    prev_state[MOTOR_RELOGIO] = actual_state;
  }

  //Calc the encoder position  
}


//Function that will be associated with the encoder interrupts (ampulheta)
void update_enc_ampulheta()
{
  int actual_state = 0;

  actual_state = digitalRead(enc_d) | (digitalRead(enc_c)<<1);
  
  if(actual_state != prev_state[MOTOR_AMPULHETA])
  {
    switch(actual_state){
      case 0:
        if(prev_state[MOTOR_AMPULHETA] == 1)
          enc_count[MOTOR_AMPULHETA]++;
        else
          enc_count[MOTOR_AMPULHETA]--;  
      break;
      case 1:
        if(prev_state[MOTOR_AMPULHETA] == 3)
          enc_count[MOTOR_AMPULHETA]++;
        else
          enc_count[MOTOR_AMPULHETA]--;
      break;
      case 2:
        if(prev_state[MOTOR_AMPULHETA] == 0)
          enc_count[MOTOR_AMPULHETA]++;
        else
          enc_count[MOTOR_AMPULHETA]--;
      break;
      case 3:
        if(prev_state[MOTOR_AMPULHETA] == 2)
          enc_count[MOTOR_AMPULHETA]++;
        else
          enc_count[MOTOR_AMPULHETA]--;
      break;
    }
    
    prev_state[MOTOR_AMPULHETA] = actual_state;
  }

  //Calc the encoder position  
}

void set_PWM(bool direction, char speed, int id)
{
  if(direction)
  {
    if(id == 0)
    {
      analogWrite(out_a, speed);
      analogWrite(out_b, 0);  
    }
    else
    {
      analogWrite(out_c, speed);
      analogWrite(out_d, 0);  
    }
  }
  else 
  {
    if(id == 0)
    {
      analogWrite(out_a, 0);
      analogWrite(out_b, speed);  
    }
    else
    {
      analogWrite(out_c, 0);
      analogWrite(out_d, speed);  
    }
  }
}


void set_speed(float new_speed, int id)
{
  desired_speed[id] = new_speed;
  
  /*
  Serial.println(" ");
  Serial.print(" desired_speed[%i]:", id);
  Serial.println((float) desired_speed[id] );
  Serial.println(" ");
  */
}


void set_position(float deg, int id)
{
  desired_position[id] = deg;
  
  /*
  Serial.println(" ");
  Serial.print(" desired_position[%i]:", id);
  Serial.println((float) desired_position[id] );
  Serial.println(" ");
  */
}
 
void position_controller(int id)
{
  float low_limit = desired_position[id] - POS_TOLERANCE/2;
  float high_limit = desired_position[id] + POS_TOLERANCE/2;

  /*
  Serial.print(" low_limit");
  Serial.print((float) low_limit);
  Serial.print(" high_limit");
  Serial.println((float) high_limit);
  */
   
  if( !(low_limit< deg_position[id] && high_limit > deg_position[id]))
  {
    desired_position_reached[id] = false;
    //not on the correct position
    if(deg_position[id] < low_limit)
    {
      set_speed(0.2, id);
    } 
    else 
    {
      set_speed(-0.2, id);
    }
  } 
  else 
  {
    desired_position_reached[id] = true;
    set_speed(0, id);
  }  
}

void speed_controller(int id)
{
  float error_p = 0;
  float error_d = 0; //AC: to chek if should not be statical
  int pwm = 0;

  error_p = desired_speed[id] - rev_sec[id];
  error_i[id]+= error_p;
  
  error_d+= error_p - prev_error_p[id];
  prev_error_p[id] = error_p;

  pwm = (int)((desired_speed[id] + error_p * K_P + error_i[id] * K_I + error_d * K_D )*K_PWM);
  if(pwm > 255)
    pwm = 255;
  if(pwm < -255)
    pwm = -255;

  /* 
  Serial.print(" desired_speed[%i]:", id);
  Serial.print((float)desired_speed[id]);
  Serial.print(" actual speed[%i]:", id);
  Serial.print((float)rev_sec[id]);
  
  Serial.print(" error_p[%i]:", id);
  Serial.print((float)error_p[id]);
  Serial.print(" error_i[%i]:", id);
  Serial.print((float)error_i[id]);
  Serial.print(" error_d[%i]:", id);
  Serial.print((float)error_d[id]);
  Serial.print(" pwm:");
  Serial.println(pwm);
  */

  if(pwm>=0)
    set_PWM(false, abs(pwm), id);
  else
    set_PWM(true, abs(pwm), id);  
}


bool calibrate_position_zero_motor(int id)
{
  bool fb = false;
  digitalWrite(motor_enable,1);
  digitalWrite(motor_enable_ampulheta,1);

  if(id == 0)
  {
    fb = digitalRead(position_0_motor); 
  } 
  else 
  {
    fb = digitalRead(position_0_ampulheta);
  }
  
  if(fb)
  {
    set_speed(0.2, id);
    
    return false;
  } 
  else
  {
    set_speed(0, id);
    enc_count[id] = 0;
    prev_enc_count[id] = 0;
    
    return true;
  }
}


void update_motor(int id)
{
  unsigned long actual_time = millis();
  
  if( (actual_time - prev_time[id]) > TIME_CONSTANT)
  {
    long enc_steps      = enc_count[id] - prev_enc_count[id];  
    prev_enc_count[id]  = enc_count[id];
    prev_time[id]       = actual_time;
    
    //expected steps per second
    enc_steps*=(1000/TIME_CONSTANT);
    rev_sec[id] = (float) (enc_steps*1.1/(STEPS_PER_REV*1.1));

    /*   
    Serial.print(" enc_steps:");
    Serial.print((int)enc_steps);
    Serial.print(" enc_count[%i]:", id);
    Serial.print((int)enc_count[id]);
    Serial.print(" prev_enc_count[%i]:", id);
    Serial.print((int)prev_enc_count[id]);
    Serial.print(" rev_sec[%s]:", id);
    Serial.println((float)rev_sec[id]);
    // */    
    //deg_position[id] = ((float)((enc_count[id]*1.0)/(STEPS_PER_REV*1.0))- (int)(enc_count[id]/STEPS_PER_REV))*360;
    deg_position[id] = ((float)((enc_count[id]*1.0)/(STEPS_PER_REV*1.0)))*360;
    
    /*Serial.print(" enc_count[%i]:", id);
    Serial.print((int)enc_count[id]);
    Serial.print(" deg_position[%i]:", id);
    Serial.println((float)deg_position[id]);*/
        
    speed_controller(id);
    
    position_controller(id);
  }
}

void update_lever(int id)
{
  if(lever_working[id] && timer_servo[id] < millis())
  {
    lever[id].detach();
    lever_working[id] = false;
  }
  
  if(lever_deg[id] != prev_lever_deg[id] && !lever_working[id])
  {
    lever_working[id] = true;
    lever[id].attach(lever_pin[id]);
    lever[id].write(lever_deg[id]);
    prev_lever_deg[id] = lever_deg[id];
    timer_servo[id] = millis() + 1000;
  }
}

void set_lever(int deg, int id)
{
    lever_deg[id] = deg;
}

int process_serial()
{
  static int prev_value = 9999;
  int value = 0;
  char i = 0;
  char test_number = 0;
  
  if(rx_buffer[0] == '-' && rx_buffer[BUFFER_SIZE - 1] == '/')
  {
    for(i=1;i<BUFFER_SIZE - 1;i++)
    {
      if(rx_buffer[i] < '0' || rx_buffer[i] >'9')
        test_number = -1;
    }
    if(test_number == 0)
    {
      value = (rx_buffer[1]-'0')*1000 + (rx_buffer[2]-'0')*100 + (rx_buffer[3]-'0')*10 + (rx_buffer[4]-'0');
      if(prev_value != value)
      {
        prev_value = value;
        year_received = value;
      }
    }
  }
  
  return prev_value;
}

void send_new_year(int year)
{
  if(year == 0){
    Serial1.print("-0000/");
  }
  Serial1.print("-");
  Serial1.print(year,DEC);
  Serial1.print("/");  
}



void ampulheta_state_machine()
{
  switch(sm_motor[MOTOR_AMPULHETA])
  {
    case START:
      if(calibrate_position_zero_motor(MOTOR_AMPULHETA))
      {
        sm_motor[MOTOR_AMPULHETA] = AMPULHETA_180;
        sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
      }
      break;
    case AMPULHETA_180:
      if(sm_timer[MOTOR_AMPULHETA] < millis())
      {
        set_position(AMPULHETA_HIGH, MOTOR_AMPULHETA);
        sm_motor[MOTOR_AMPULHETA] = AMPULHETA_0;
        sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
      }
    break;
    case AMPULHETA_0:
      if(sm_timer[MOTOR_AMPULHETA] < millis())
      {
        set_position(AMPULHETA_0, MOTOR_AMPULHETA);
        sm_motor[MOTOR_AMPULHETA] = AMPULHETA_180;
        sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
      }
    break;
  }
}

void motor_statemachine()
{
  switch(sm_motor[MOTOR_RELOGIO])
  {
    case START:
      if(calibrate_position_zero_motor(MOTOR_RELOGIO))
      {
        sm_motor[MOTOR_RELOGIO] = WAITING_NEW_YEAR;
      }
      break;
    case WAITING_NEW_YEAR:
      if(last_year_received != year_received)
      {       
        switch(year_received)
        {
           case 1910:
            set_position(_1910_POS, MOTOR_RELOGIO);
            break;
          case 1920:
            set_position(_1920_POS, MOTOR_RELOGIO);
            break;
          case 1930:
            set_position(_1930_POS, MOTOR_RELOGIO);
            break;
          case 1940:
            set_position(_1940_POS, MOTOR_RELOGIO);
            break;
          default:
            goto label_year_undefined;
            break;
        }
        
        send_new_year(9999);
        last_year_received = year_received;
        sm_timer[MOTOR_RELOGIO] = millis() + 100;
        sm_motor[MOTOR_RELOGIO] = WAITING_TO_REACH;

label_year_undefined:
        int a=0; //dummy label
      }
      break;
    case WAITING_TO_REACH:
      if(desired_position_reached[MOTOR_RELOGIO] && sm_timer[MOTOR_RELOGIO] < millis())
      {
        Serial.print('r');
        send_new_year(last_year_received);
        sm_motor[MOTOR_RELOGIO] = SET_LEVERS_FORWARD;
      }
      break;
    case SET_LEVERS_FORWARD:
      set_lever(1,SERVO_1);
      set_lever(179,SERVO_2);
      sm_timer[MOTOR_RELOGIO] = millis()+1000;
      sm_motor[MOTOR_RELOGIO] = SET_LEVERS_FORWARD_WAITING;
      break;
    case SET_LEVERS_FORWARD_WAITING:
      if(sm_timer[MOTOR_RELOGIO] < millis())
      {
        set_lever(179,SERVO_1);
        set_lever(1,SERVO_2);
        sm_timer[MOTOR_RELOGIO] = millis()+1000;
        sm_motor[MOTOR_RELOGIO] = SET_LEVERS_BACK_WAITING;
      }
      break;
    case SET_LEVERS_BACK_WAITING:
      if(sm_timer[MOTOR_RELOGIO] < millis())
      {
        sm_motor[MOTOR_RELOGIO] = WAITING_NEW_YEAR;
      }
      break;
  }
}


void check_onoff_switch()
{
  static bool prev_state = false;
  static unsigned long prev_timer = millis();  
  int i=0;

  if(prev_state != digitalRead(main_switch))
  {
    prev_state = digitalRead(main_switch);
    if(!digitalRead(main_switch))
    {
      digitalWrite(relay_0,0);
      digitalWrite(motor_enable,1);
      digitalWrite(motor_enable_ampulheta,1);     
    } 
    else
    {
      prev_timer = millis() + 10000;
      
      digitalWrite(motor_enable,0);
      digitalWrite(motor_enable_ampulheta,0);

      for(i = 0; i<2;i++)
      {
        //Reset the state machines
        enc_count[i] = 0;
        prev_enc_count[i] = 0;
        sm_motor[i] = START;

        //Send the shutdown signal to the raspberry
        Serial.print("X");
        Serial.print("X");
        Serial.print("X");
        Serial.print("X");
        Serial.print("X");     
      }
    }
  }
  
  if(digitalRead(main_switch))
  {
    if(prev_timer < millis())
    {
      digitalWrite(relay_0,1);    
      send_new_year(0);
    }  
  }
}

void loop() 
{ 
  // put your main code here, to run repeatedly:
  
  check_onoff_switch();
 
  if(!digitalRead(main_switch))
  {
    update_motor(MOTOR_RELOGIO);
    update_motor(MOTOR_AMPULHETA);
    
    update_lever(MOTOR_RELOGIO);
    update_lever(MOTOR_AMPULHETA);
    
    process_serial();
  
    motor_statemachine();
    ampulheta_state_machine();
  }
}


void serialEvent()
{ 
 char i = 0;
  while (Serial.available())
  {
    char received = (char)Serial.read();

    for(i=1;i<BUFFER_SIZE;i++)
    {
      rx_buffer[i-1]=rx_buffer[i];
    }
    
    rx_buffer[BUFFER_SIZE-1] = received;
  }

  int value_ = (rx_buffer[1]-'0')*1000 + (rx_buffer[2]-'0')*100 + (rx_buffer[3]-'0')*10 + (rx_buffer[4]-'0');
  //Serial.print(value_);
  
  if(last_year_received == value_)
  {
    Serial.print('r');
  }
}


