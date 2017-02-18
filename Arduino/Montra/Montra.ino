#include <Servo.h>

#define enc_a 2 
#define enc_b 3
#define out_a 6
#define out_b 7

#define lever1_pin  14
#define lever2_pin  15


#define STEPS_PER_REV 1920
#define MAX_SPEED_PER_REV 400 //value in millis 
#define TIME_CONSTANT 100

Servo lever1;
Servo lever2;


void setup() 
{
  //Serial that will be use to comunicate with the Raspberry
  Serial.begin(9600);

  //Serial that will be use to comunicate with the 7 segments display micro
  Serial1.begin(9600);
 
  pinMode(enc_a, INPUT);
  pinMode(enc_b, INPUT);

  //link the interupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(enc_a), update_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_b), update_enc, CHANGE);
  
  pinMode(out_a, OUTPUT);
  pinMode(out_b, OUTPUT); 
}

static long enc_count = 0;
static float rev_sec= 0;
static float deg_position = 0;
static float desired_position = 0;
static bool desired_position_reached = false;
static float desired_speed = 0;

//Function that will be associated with the encoder interrupts
void update_enc()
{
  static int prev_state = 0;
  int actual_state = 0;

  actual_state = digitalRead(enc_b) | (digitalRead(enc_a)<<1);
  
  if(actual_state != prev_state){
    switch(actual_state){
      case 0:
        if(prev_state == 1)
          enc_count++;
        else
          enc_count--;  
      break;
      case 1:
        if(prev_state == 3)
          enc_count++;
        else
          enc_count--;
      break;
      case 2:
        if(prev_state == 0)
          enc_count++;
        else
          enc_count--;
      break;
      case 3:
        if(prev_state == 2)
          enc_count++;
        else
          enc_count--;
      break;
    }
    
    prev_state = actual_state;
  }

  //Calc the encoder position  
}


#define K_P 0.5
#define K_I 0.1
#define K_D 0.5
#define K_PWM 95  // 255/max_speed 

void set_PWM(bool direction, char speed)
{
  if(direction){
    analogWrite(out_a, speed);
    analogWrite(out_b, 0);  
  } else {
    analogWrite(out_a, 0);
    analogWrite(out_b, speed);
  }
}


void set_speed(float new_speed)
{
  desired_speed = new_speed;
  /*
  Serial.println(" ");
  Serial.print(" New_speed:");
  Serial.println((float) new_speed );
  Serial.println(" ");
  */
}


void set_position(float deg)
{
  desired_position = deg;
  /*
  Serial.println(" ");
  Serial.print(" New_deg:");
  Serial.println((float) desired_position );
  Serial.println(" ");
  */
}


#define POS_TOLERANCE 10
 
void position_controller()
{
  float low_limit = desired_position - POS_TOLERANCE/2;
  float high_limit = desired_position + POS_TOLERANCE/2;

  /*
  Serial.print(" low_limit");
  Serial.print((float) low_limit);
  Serial.print(" high_limit");
  Serial.println((float) high_limit);
  */
  
  if( !(low_limit< deg_position && high_limit > deg_position))
  {
    desired_position_reached = false;
    //not on the correct position
    if(deg_position < low_limit)
    {
      set_speed(0.2);
    } 
    else 
    {
      set_speed(-0.2);
    }
  } 
  else 
  {
    desired_position_reached = true;
    set_speed(0);
  }  
}

void speed_controller()
{
  float error_p = 0;
  static float error_i= 0;
  static float prev_error_p = 0;
  float error_d = 0;
  int pwm = 0;

  error_p = desired_speed - rev_sec;
  error_i+= error_p;

  /*
  if(error_i > 50)
      error_i = 50;
  if(error_i < -50)
      error_i = -50;
  */
  
  error_d+= error_p - prev_error_p;
  prev_error_p = error_p;

  pwm = (int)((desired_speed + error_p * K_P + error_i * K_I + error_d * K_D )*K_PWM);
  if(pwm > 255)
    pwm = 255;
  if(pwm < -255)
    pwm = -255;

  //da para ver se o erro esta a crescer ou nao
  /* 
  Serial.print(" desired_speed:");
  Serial.print((float)desired_speed);
  Serial.print(" actual speed:");
  Serial.print((float)rev_sec);
  
  Serial.print(" error_p:");
  Serial.print((float)error_p);
  Serial.print(" error_i:");
  Serial.print((float)error_i);
  Serial.print(" error_d:");
  Serial.print((float)error_d);
  Serial.print(" pwm:");
  Serial.println(pwm);
  */

  if(pwm>=0)
    set_PWM(false, abs(pwm));
  else
    set_PWM(true, abs(pwm));  
}


void update_motor()
{
  static long prev_enc_count=0;
  static unsigned long prev_time = millis();
  unsigned long actual_time = millis();
  
  if( (actual_time - prev_time) > TIME_CONSTANT)
  {
    long enc_steps = enc_count - prev_enc_count;  
    prev_enc_count = enc_count;
    prev_time = actual_time;
    
    
    //expected steps per second
    enc_steps*=(1000/TIME_CONSTANT);
    rev_sec = (float) (enc_steps*1.1/(STEPS_PER_REV*1.1));
    
    //Verificar se esta a efectuar a contagem do motor correctamente
    
    /*Serial.print(" enc_steps:");
    Serial.print((int)enc_steps);
    Serial.print(" enc_count:");
    Serial.print((int)enc_count);
    Serial.print(" prev_enc_count:");
    Serial.print((int)prev_enc_count);
    Serial.print(" rev_sec:");
    Serial.println((float)rev_sec);*/
    
    //deg_position = ((float)((enc_count*1.0)/(STEPS_PER_REV*1.0))- (int)(enc_count/STEPS_PER_REV))*360;
    deg_position = ((float)((enc_count*1.0)/(STEPS_PER_REV*1.0)))*360;
    
    /*Serial.print(" enc_count:");
    Serial.print((int)enc_count);
    Serial.print(" deg_position:");
    Serial.println((float)deg_position);*/
        
    speed_controller();
    
    position_controller();
  }
}


int lever1_deg = 0, lever2_deg = 0;

void update_lever()
{
  static unsigned long timer1 = millis();
  static unsigned long timer2 = millis();
  static int prev_lever1_deg = 0, prev_lever2_deg = 0;
  static bool lever1_working = false, lever2_working= false;

  if(lever1_working && timer1 < millis())
  {
    lever1.detach();
    lever1_working = false;
  }
  
  if(lever1_deg != prev_lever1_deg && !lever1_working)
  {
    lever1_working = true;
    lever1.attach(lever1_pin);
    lever1.write(lever1_deg);
    prev_lever1_deg = lever1_deg;
    timer1 = millis() + 1000;
  }

  if(lever2_working && timer2 < millis())
  {
    lever2.detach();
    lever2_working = false;
  }
  if(lever2_deg != prev_lever2_deg && !lever2_working)
  {
    lever2_working = true;
    lever2.attach(lever2_pin);
    lever2.write(lever2_deg);
    prev_lever2_deg = lever2_deg;
    timer2 = millis() + 1000;
  }
}

void set_lever(int deg, char lever)
{
  if(lever == 1)
  {
    lever1_deg = deg;
  }
  if(lever == 2)
  {
    lever2_deg = deg;
  }
}


#define LEVER_1_FRONT_DEG 180
#define LEVER_1_BACK_DEG  0
#define LEVER_2_FRONT_DEG 180
#define LEVER_2_BACK_DEG  0

bool move_to_position(int deg_pos)
{
  static char sm = 0;
  static unsigned long timer = millis();
  static int prev_deg_pos = 0;
  
  switch(sm){
    case 0:
      if(prev_deg_pos == deg_pos)
      {
        return true;
      }
      else 
      {
        sm = 1;
        prev_deg_pos = deg_pos;
        set_lever(LEVER_1_FRONT_DEG,1);
        timer = millis()+750;
        
        return false;
      }
      break;
    case 1:
      if(timer<millis())
      {
        set_position(prev_deg_pos);
        timer = millis() + 250;
        sm = 2;
      }
      break;
    case 2:
      if(timer<millis())
      {
        set_lever(LEVER_1_BACK_DEG,1);
        sm = 3;
      }
      break;
    case 3:
      if(desired_position_reached){
        set_lever(LEVER_2_FRONT_DEG,2);
        timer = millis() + 2000;
        sm = 4;    
      }
      break;
    case 4:
      if(desired_position_reached)
      {
        set_lever(LEVER_2_BACK_DEG,2);
        timer = millis() + 2000;
        sm = 0;    
      }
  }  

  return false;
}


int year_received = 0;

#define BUFFER_SIZE 6
static char rx_buffer[BUFFER_SIZE];

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
  Serial1.print("-");
  Serial1.print(year,DEC);
  Serial1.print("/");  
}

enum states 
{ 
  START,
  WAITING_NEW_YEAR,
  WAITING_TO_REACH,
  SET_LEVERS_FORWARD,
  SET_LEVERS_FORWARD_WAITING,
  SET_LEVERS_BACK_WAITING
};

#define _1910_POS   10
#define _1920_POS   300
#define _1930_POS   180
#define _1940_POS   100

void loop() 
{
  static enum states sm = START;
  static unsigned long sm_timer = millis();

  static int last_year_received=0;
  
  // put your main code here, to run repeatedly:
  
  //set_lever(100,1);
  //set_lever(100,2);

  update_motor();
  update_lever();
  process_serial();

  switch(sm)
  {
    case START:
      sm = WAITING_NEW_YEAR;
      set_position(0);
      break;
    case WAITING_NEW_YEAR:
      if(last_year_received != year_received)
      {
        switch(year_received)
        {
           case 1910:
            set_lever(179,1);
            set_lever(1,2);
            update_lever();
            set_position(_1910_POS);
            break;
          case 1920:
            set_lever(1,1);
            set_lever(179,2);
            update_lever();
            set_position(_1920_POS);
            break;
          case 1930:
            set_lever(179,1);
            set_lever(1,2);
            update_lever();
            set_position(_1930_POS);
            break;
          case 1940:
            set_lever(1,1);
            set_lever(179,2);
            update_lever();
            set_position(_1940_POS);
            break;
        }
        
        send_new_year(9999);
        last_year_received = year_received;
        sm_timer = millis() + 100;
        sm = WAITING_TO_REACH;
      }
      break;
    case WAITING_TO_REACH:
      if(desired_position_reached && sm_timer < millis())
      {
        Serial.print('r');
        send_new_year(last_year_received);
        sm = SET_LEVERS_FORWARD;
      }
      break;
    case SET_LEVERS_FORWARD:
      set_lever(0,1);
      set_lever(180,2);
      sm_timer = millis()+1000;
      sm = SET_LEVERS_FORWARD_WAITING;
      break;
    case SET_LEVERS_FORWARD_WAITING:
      if(sm_timer < millis())
      {
        set_lever(180,1);
        set_lever(0,2);
        sm_timer = millis()+1000;
       sm = SET_LEVERS_BACK_WAITING;
      }
      break;
    case SET_LEVERS_BACK_WAITING:
      if(sm_timer < millis())
      {
        sm = WAITING_NEW_YEAR;
      }
      break;
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
}

