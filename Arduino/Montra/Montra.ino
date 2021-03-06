//#include <SparkFunDS1307RTC.h>

#include <Wire.h>
#include <Servo.h>

#include <avr/wdt.h>

#define OPEN_TIME 11
#define CLOSE_TIME 3

//---------------------Motor do relogio
#define enc_a 18 
#define enc_b 19
#define out_a 8
#define out_b 9

//---------------------Motor da ampulheta
#define enc_c 3 
#define enc_d 2
#define out_c 7
#define out_d 6

#define motor_enable            10
#define motor_enable_ampulheta  5
#define position_0_motor        A0
#define position_0_ampulheta    A1

//--------------------Reles
#define main_switch A2

#define NOUTPUTS 10
static int saida_state_pin[NOUTPUTS] = { 45, 43, 47, 49, 41, 39, 37, 35, 33, 31 };
static bool saida_state[NOUTPUTS] = { false, false, false, false, false, false, false, false, false, false };
static unsigned long sm_timer_lampada[3] = {millis(), millis(), millis()};

static bool bOnOff = false;

static unsigned long sm_timer_serie = 0;

//#define STEPS_PER_REV 1920
#define STEPS_PER_REV 6533
#define STEPS_PER_REV_AMP 6533

#define MAX_SPEED_PER_REV 400 //value in millis 
#define TIME_CONSTANT 100

#define POS_TOLERANCE 10
#define AMPULHETA_STOPTIME_SECONDS 30


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

static int prev_value = 9999;

static int iSerialRead = 0;

#define K_PA 1.5  //1.5
#define K_IA 1.2  //1 
#define K_DA 0.5  //0.2
#define K_PWMA 100  //100

#define K_P 2.5  //0.5
#define K_I 0.2  //0.1
#define K_D 0.4  //0.5
#define K_PWM 100  //95 255/max_speed 

static float error_i[2]       = {0,0};
static float prev_error_p[2]  = {0,0};

Servo lever[2];
static unsigned long timer_servo[2];
static int lever_deg[2]       = {0,0};
static int lever_pin[2]       = {11,12};
static int prev_lever_deg[2]  = {0,0};
static bool lever_working[2]  = {false, false};

static int year_received      = 0;
static int last_year_received = 0;

int timer_count = 0;
int timer_count_amp = 0;

#define BUFFER_SIZE 6
static char rx_buffer[BUFFER_SIZE] = {0,0,0,0,0,0};
static char rx_buffer_tmp[BUFFER_SIZE] = {0,0,0,0,0,0};

enum states 
{ 
  HALT,
  START,
  PAUSE,
  PAUSE_,
  SHUTDOWN,
  WAITING_NEW_YEAR,
  WAITING_TO_REACH,
  SET_LEVERS_FORWARD,
  SET_LEVERS_FORWARD_WAITING,
  SET_LEVERS_BACK_WAITING,
  AMPULHETA_0,
  AMPULHETA_180,
};

#define offset 0
#define offset_amp 0

#define _1927_POS   26*13 + offset
#define _1928_POS   26*13 + offset
#define _1967_POS   26*10 + offset
#define _1970_POS   26*9 + offset
#define _1980_POS   26*7 + offset
#define _1995_POS   26*4 + offset
#define _1998_POS   26*3 +7 + offset
#define _2013_POS   26*2 + offset
#define _2017_POS   26*1 + offset

#define AMPULHETA_HIGH  0
#define AMPULHETA_LOW   180

#define MOTOR_RELOGIO   0
#define MOTOR_AMPULHETA 1

#define SERVO_1   0
#define SERVO_2   1

static enum states sm_motor[2]      = {HALT, HALT};
static unsigned long sm_timer[2]    = {millis(), millis()};
static unsigned long sm_timer_pause = millis();

char received;

void setup() 
{ 
  //rtc.begin(); // Call rtc.begin() to initialize the library
  //rtc.setTime(30, 37, 15, 5, 14, 04, 17);  // Uncomment to manually set time
  //rtc.set24Hour(); 
  
  wdt_disable();

  //Serial that will be use to comunicate with the Raspberry
  Serial3.begin(9600);

  //Serial that will be use to debug
  Serial.begin(9600);

  //Serial that will be use to comunicate with the 7 segments display micro
  Serial2.begin(9600);
 
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

  for (int i=0; i<NOUTPUTS; i++)
  {
    ////serial.println(saida_state_pin[i]);
    
    pinMode(saida_state_pin[i], OUTPUT);
    digitalWrite(saida_state_pin[i], 1);
  }

  pinMode(A3, INPUT_PULLUP);

  bOnOff = digitalRead(main_switch);

  //Para garantir que em caso de wdt reset o raspberry descarrega  condensador
  delay(10000);

  wdt_enable(WDTO_8S);
}

//Function that will be associated with the encoder interrupts (Relogio)
void update_enc_relogio()
{
  int actual_state = 0;

  actual_state = digitalRead(enc_b) | (digitalRead(enc_a)<<1);
  
  if(actual_state != prev_state[MOTOR_RELOGIO])
  {
    switch(actual_state)
    {
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
  //serial.println(" ");
  //serial.print(" desired_speed[%i]:", id);
  //serial.println((float) desired_speed[id] );
  //serial.println(" ");
  */
}


void set_position(float deg, int id)
{
  desired_position[id] = deg;
  
  /*
  //serial.println(" ");
  //serial.print(" desired_position[%i]:", id);
  //serial.println((float) desired_position[id] );
  //serial.println(" ");
  */
}
 
void position_controller(int id)
{
  float low_limit = desired_position[id] - POS_TOLERANCE/2;
  float high_limit = desired_position[id] + POS_TOLERANCE/2;

  /*
  //serial.print(" low_limit");
  //serial.print((float) low_limit);
  //serial.print(" pos");
  //serial.print((float) deg_position[id]);
  //serial.print(" high_limit");
  //serial.println((float) high_limit);
  */
   
  if( !(low_limit< deg_position[id] && high_limit > deg_position[id]))
  {
    desired_position_reached[id] = false;
    //not on the correct position
    if(deg_position[id] < low_limit)
    {
      if(id == MOTOR_RELOGIO)
        set_speed(0.15, id);
      else
        set_speed(0.08, id);
    } 
    else 
    {
      if(id == MOTOR_RELOGIO)
        set_speed(-0.15, id);
      else
        set_speed(-0.08, id);
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
  float error_d = 0;
  int pwm = 0;

  error_p = desired_speed[id] - rev_sec[id];
  error_i[id]+= error_p;
  
  error_d+= error_p - prev_error_p[id];
  prev_error_p[id] = error_p;

  float KP = K_P;
  float KI = K_I;
  float KD = K_D;
  float KPWM = K_PWM;

  if(id == MOTOR_AMPULHETA)
  {
    KP = K_PA;
    KI = K_IA;
    KD = K_DA;
    KPWM = K_PWMA;
  }

  pwm = (int)((desired_speed[id] + error_p * KP + error_i[id] * KI + error_d * KD )*KPWM);
  if(pwm > 255)
    pwm = 255;
  if(pwm < -255)
    pwm = -255;

  /* 
  //serial.print(" desired_speed[%i]:", id);
  //serial.print((float)desired_speed[id]);
  //serial.print(" actual speed[%i]:", id);
  //serial.print((float)rev_sec[id]);
  
  //serial.print(" error_p[%i]:", id);
  //serial.print((float)error_p[id]);
  //serial.print(" error_i[%i]:", id);
  //serial.print((float)error_i[id]);
  //serial.print(" error_d[%i]:", id);
  //serial.print((float)error_d[id]);
  //serial.print(" pwm:");
  //serial.println(pwm);
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
    if(id == 0)
      set_speed(0.2, id);
    else
      set_speed(0.1, id);
    
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

    int steps_rev = STEPS_PER_REV;
    if(id == MOTOR_AMPULHETA)
    {
      steps_rev = STEPS_PER_REV_AMP;
    }
    
    rev_sec[id] = (float) (enc_steps*1.1/(steps_rev*1.1));

    /*   
    //serial.print(" enc_steps:");
    //serial.print((int)enc_steps);
    //serial.print(" enc_count[%i]:", id);
    //serial.print((int)enc_count[id]);
    //serial.print(" prev_enc_count[%i]:", id);
    //serial.print((int)prev_enc_count[id]);
    //serial.print(" rev_sec[%s]:", id);
    //serial.println((float)rev_sec[id]);
    // */    
    //deg_position[id] = ((float)((enc_count[id]*1.0)/(STEPS_PER_REV*1.0))- (int)(enc_count[id]/STEPS_PER_REV))*360;
    deg_position[id] = ((float)((enc_count[id]*1.0)/(steps_rev*1.0)))*360;
    
    /*//serial.print(" enc_count[%i]:", id);
    //serial.print((int)enc_count[id]);
    //serial.print(" deg_position[%i]:", id);
    //serial.println((float)deg_position[id]);*/
        
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
  int value = 0;
  int i = 0;
  int test_number = 0;
  
  if(rx_buffer[0] == '-' && rx_buffer[BUFFER_SIZE - 1] == '/')
  {
    for(i=1; i<(BUFFER_SIZE - 1); i++)
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

    rx_buffer[0] = '0';
    rx_buffer[1] = '0';
    rx_buffer[2] = '0';
    rx_buffer[3] = '0';
    rx_buffer[4] = '0';
    rx_buffer[5] = '0';
  }
  
  return prev_value;
}

void send_new_year(int year)
{
  if(year == 0){
    Serial2.print("-0000/");
  }
  Serial2.print("-");
  Serial2.print(year,DEC);
  Serial2.print("/");  
}



void print_state(enum states state){
  switch(state){
    case HALT:
      ////serial.println("HALT");
      break;
    case START:
      ////serial.println("START");
      break;
    case PAUSE:
      //////serial.println("PAUSE");
      break;
    case PAUSE_:
      ////serial.println("PAUSE_");
      break;
    case SHUTDOWN:
      //serial.println("SHUTDOWN");
      break;
    case WAITING_NEW_YEAR:
      //serial.println("WAITING_NEW_YEAR");
      break;
    case WAITING_TO_REACH:
      //serial.println("WAITING_TO_REACH");
      break;
    case SET_LEVERS_FORWARD:
      //serial.println("SET_LEVERS_FORWARD");
      break;
    case SET_LEVERS_FORWARD_WAITING:
      //serial.println("SET_LEVERS_FORWARD_WAITING");
      break;
    case SET_LEVERS_BACK_WAITING:
      //serial.println("SET_LEVERS_BACK_WAITING");
      break;
    case AMPULHETA_0:
      //serial.println("AMPULHETA_0");
      break;
    case AMPULHETA_180:
      //serial.println("AMPULHETA_180");
      break;
  }
}

void ampulheta_state_machine()
{
  static enum states prev_sm = HALT;

  
  switch(sm_motor[MOTOR_AMPULHETA])
  {
    case HALT:
      break;
    case START:
      if(calibrate_position_zero_motor(MOTOR_AMPULHETA))
      {
        if(offset_amp != 0)
        {
          set_position(offset_amp, MOTOR_AMPULHETA);
        }  
        sm_motor[MOTOR_AMPULHETA] = AMPULHETA_0;
        sm_timer[MOTOR_AMPULHETA] = millis() + (100);
        timer_count_amp = 1;
      }
      break;
    case AMPULHETA_180:
      if(sm_timer[MOTOR_AMPULHETA] < millis())
      {
        timer_count_amp = timer_count_amp + 1;
        if(timer_count_amp > 1)
        {
          timer_count_amp = 0;
          set_position(AMPULHETA_HIGH, MOTOR_AMPULHETA);
          sm_motor[MOTOR_AMPULHETA] = AMPULHETA_0;
          sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
        }
        else
        {
          sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
        }
      }     
      break;
    case AMPULHETA_0:
      if(sm_timer[MOTOR_AMPULHETA] < millis())
      {
        timer_count_amp = timer_count_amp + 1;
        if(timer_count_amp > 1)
        {
          timer_count_amp = 0;
          set_position(AMPULHETA_LOW, MOTOR_AMPULHETA);
          sm_motor[MOTOR_AMPULHETA] = AMPULHETA_180;
          sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
        }
        else
        {
          sm_timer[MOTOR_AMPULHETA] = millis() + (1000*AMPULHETA_STOPTIME_SECONDS);
        }
      }
      break;
    case SHUTDOWN:
      sm_motor[MOTOR_AMPULHETA] = HALT;
      break;
  }


  if(prev_sm != sm_motor[MOTOR_AMPULHETA]){
    //serial.print("New ampulheta:");
    print_state(sm_motor[MOTOR_AMPULHETA]);
  }
  prev_sm = sm_motor[MOTOR_AMPULHETA];
    
}

void control_rele(int rele, bool state)
{
  int i = 0;
  for(i=0; i< NOUTPUTS; i++)
  {
    if(saida_state_pin[i] == rele)
    {
      break;
    }
  }
  
  if(state == true)
  {
//sugiro
//pinMode(rele, OUTPUT);
//digitalWrite(rele, 0);

    digitalWrite(rele, 0);
    saida_state[i] = state;
  }
  else
  {
    digitalWrite(rele, 1);
    
//sugiro
//pinMode(rele, INPUT);
    
    saida_state[i] = state;
  }
}

void reset_reles()
{
  control_rele(saida_state_pin[0], false);
  control_rele(saida_state_pin[1], false);
  control_rele(saida_state_pin[2], false);
  control_rele(saida_state_pin[3], false);
  control_rele(saida_state_pin[4], false);
  control_rele(saida_state_pin[5], false);
  //control_rele(saida_state_pin[6], false);
  //control_rele(saida_state_pin[7], false);
  //control_rele(saida_state_pin[8], false);
  //control_rele(saida_state_pin[9], false);
}

void reset_lampadas_timer()
{
  sm_timer_lampada[0] = millis() + 500;
  sm_timer_lampada[1] = millis() + 1000;
  sm_timer_lampada[2] = millis() + 1500;
}

void motor_statemachine()
{
  static enum states prev_state = HALT;
    
  switch(sm_motor[MOTOR_RELOGIO])
  {
    
    case HALT:
      break;
    case START:
      sm_timer_serie = millis();
      //////wdt_enable(WDTO_8S);
      if(calibrate_position_zero_motor(MOTOR_RELOGIO))
      {
        //wdt_disable();
        
        if(offset != 0)
        {
          set_position(offset, MOTOR_RELOGIO);
        }
        sm_motor[MOTOR_RELOGIO] = WAITING_NEW_YEAR;
      }
      break;
    case WAITING_NEW_YEAR:
      /*//serial.print("last_year_received:");
      //serial.println((int) last_year_received);
      //serial.print("year_received:");
      //serial.println((int) year_received)´*/
      if((millis() - sm_timer_serie) > 150000) //2,5 minutes 
      {
        //wdt_enable(WDTO_15MS);
      }
      
      if(last_year_received != year_received)
      { 
        sm_timer_serie = millis();
         
        ////serial.println("last_year_received != year_received");
                     
        reset_lampadas_timer();
        
        switch(year_received)
        {
           case 1927:
            set_position(_1927_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;
          case 1928:
            set_position(_1928_POS, MOTOR_RELOGIO);
            //send_new_year(9998);
            //wdt_enable(WDTO_8S);
            break;
          case 1967:
            set_position(_1967_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;
          case 1970:
            set_position(_1970_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;
          case 1980:
            set_position(_1980_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;
          case 1995:
            set_position(_1995_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;    
          case 1998:
            set_position(_1998_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;
          case 2013:
            set_position(_2013_POS, MOTOR_RELOGIO);
            send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;
          case 2017:
            set_position(_2017_POS, MOTOR_RELOGIO);
            //send_new_year(9999);
            //wdt_enable(WDTO_8S);
            break;          
          default:
            goto label_year_undefined;
            break;
        }
        
        //send_new_year(9999);
        last_year_received = year_received;
        sm_timer[MOTOR_RELOGIO] = millis() + 100;

        if( last_year_received == 1928 )
        {
          sm_timer_pause = millis() + 100;
          sm_motor[MOTOR_RELOGIO] = PAUSE;          
        }
        else
        {
          sm_motor[MOTOR_RELOGIO] = WAITING_TO_REACH;
        }
label_year_undefined:
        int a=0; //dummy label
      }
      break;
    case PAUSE:
      if(desired_position_reached[MOTOR_RELOGIO] && sm_timer_pause < millis())
      {
        //wdt_disable();
        
        Serial3.print('r');
        Serial3.flush();

        //serial.print("r - Pause");

        timer_count = 0;
        send_new_year(9998);        
        sm_timer_pause = millis() + 1000;
        sm_motor[MOTOR_RELOGIO] = PAUSE_;
      }
      break;
    case PAUSE_: 
      if(sm_timer_pause < millis())
      {
        timer_count = timer_count + 1;
        if(timer_count > 110)
        {
          year_received = 2017;
          sm_motor[MOTOR_RELOGIO] = WAITING_NEW_YEAR;

          //wdt_enable(WDTO_8S);
        }
        else
        {
          sm_timer_pause = millis() + 1000;
        }
      }
      break;
    case WAITING_TO_REACH:     
      
      //serial.print("desired_position:");
      //serial.print(desired_position_reached[MOTOR_RELOGIO]);
      //serial.print(" sm_timer:");
      //serial.print(sm_timer[MOTOR_RELOGIO]);
      //serial.print(" millis:");
      //serial.println(millis());
      if(desired_position_reached[MOTOR_RELOGIO] && sm_timer[MOTOR_RELOGIO] < millis())
      {
        //wdt_disable();
        
        if(last_year_received != 2017)
        {      
          Serial3.print('r');
          Serial3.flush();

          //serial.print("r wait to reach");
        }
        
        if(last_year_received == 2017)
        {
          send_new_year(9997);
          sm_motor[MOTOR_RELOGIO] = WAITING_NEW_YEAR;

          ////wdt_enable(WDTO_15MS); 
        }
        else
        {
          send_new_year(last_year_received);
          sm_motor[MOTOR_RELOGIO] = SET_LEVERS_FORWARD;
        }

        reset_reles();
      }
      break;
    case SET_LEVERS_FORWARD:
      set_lever(40,SERVO_1);
      set_lever(140,SERVO_2);
      sm_timer[MOTOR_RELOGIO] = millis()+1000;
      sm_motor[MOTOR_RELOGIO] = SET_LEVERS_FORWARD_WAITING;
      break;
    case SET_LEVERS_FORWARD_WAITING:
    //serial.print("sm_timer:");
    //serial.print(sm_timer[MOTOR_RELOGIO]);
    //serial.print(" millis:");
    //serial.println(millis());
    
    
      if(sm_timer[MOTOR_RELOGIO] < millis())
      {
        set_lever(140,SERVO_1);
        set_lever(40,SERVO_2);
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
    case SHUTDOWN:
      if(sm_timer[MOTOR_RELOGIO] < millis())
      {
        ////serial.print("RELAY OFF");
        
        //digitalWrite(relay_0,1);
        reset_reles();
        control_rele(saida_state_pin[6], false);    
        send_new_year(0);

        sm_motor[MOTOR_RELOGIO] = HALT;
        
        desired_position[MOTOR_RELOGIO] = 0;
        desired_position[MOTOR_AMPULHETA] = 0;
        deg_position[MOTOR_RELOGIO] = 0;
        deg_position[MOTOR_AMPULHETA] = 0;

        //wdt_disable();
      }
      break;
  }

  if(prev_state != sm_motor[MOTOR_RELOGIO]){
    //serial.print("New motor state:");
    print_state(sm_motor[MOTOR_RELOGIO]);
  }
  prev_state = sm_motor[MOTOR_RELOGIO];
}

void caixa_statemachine()
{
  if(sm_motor[MOTOR_RELOGIO] == WAITING_TO_REACH)
  {
    if(!desired_position_reached[MOTOR_RELOGIO])
    {         
      if(sm_timer_lampada[0] < millis())
        control_rele(saida_state_pin[0], true);
      if(sm_timer_lampada[1] < millis())
        control_rele(saida_state_pin[1], true);
      if(sm_timer_lampada[2] < millis())
        control_rele(saida_state_pin[2], true);
        
      control_rele(saida_state_pin[3], true);
      control_rele(saida_state_pin[4], true);
      control_rele(saida_state_pin[5], true);
      //control_rele(saida_state_pin[6], true);
      //control_rele(saida_state_pin[7], true);
      //control_rele(saida_state_pin[8], true);
      //control_rele(saida_state_pin[9], true);
    }
  }
}

/*
bool verifica_work_time()
{
  rtc.update();
  
  int hour = rtc.getHour();

  rtc.update();
  int minute = rtc.getMinute();

  //int time = (hour*100)+minute;
  Serial.print("RTC Hour:");
  Serial.println(hour);
  Serial.print("RTC Minute:");
  Serial.println(minute);
  ////serial.println(time);
  
  //Verifica se esta fora do horario de trabalho
  //if((time >= (CLOSE_TIME*100+CLOSE_TIME_MINUTE)) && (time < (OPEN_TIME*100+OPEN_TIME_MINUTE)))
  if((hour >= CLOSE_TIME) && (hour < OPEN_TIME))
  {
    Serial.println("Not work time will be checked");
            
    int hour_tmp = 0;
    
    for(int k=0; k<10; k++)
    {
        rtc.update();

        hour_tmp = rtc.getHour();

        Serial.print("RTC Hour_tmp:");
        Serial.println(hour_tmp);
    
        if(hour_tmp != hour)
        {
          Serial.println("Failed to verify Not work time");
                  
          return true;
        }
    }

    Serial.println("Not work time");
            
    return false;
  }

  //Serial.println("work time");
  return true;
}
*/

bool check_switch_state()
{
  //Se estiver a desligar ignora o comando do switch
  //e mantem o estado anterior do switch
  if(sm_motor[MOTOR_RELOGIO] == SHUTDOWN)
  {
    ////serial.println("SWITCH ON");
    if(bOnOff)
    {
      ////serial.println("SWITCH OFF");    
      return false;
    }
    else
    {
      ////serial.println("SWITCH ON");
      return true;
    }    
  }

  bool bOnOff_tmp = digitalRead(main_switch);

  if(bOnOff_tmp)
  {
    ////serial.println("READ SWITCH: OFF");
  }
  else
  {
    ////serial.println("READ SWITCH: ON");
  }

  //Se detetar uma transicao de estado 
  //Confirma a transicao
  if(bOnOff_tmp != bOnOff)
  {
    //serial.println("Transition will be checked");

    //Efectua varias leituras para evitar debouncing
    for(int i=0; i<5; i++)
    {
      bool bOnOff_tmp_ = digitalRead(main_switch); 
      
      if(bOnOff_tmp_)
      {
        ////serial.println("#READ SWITCH: OFF");
      }
      else
      {
        ////serial.println("#READ SWITCH: ON");
      }

      //Se a leitura nao for consistente
      //devolve o estado anterior
      if(bOnOff_tmp_ != bOnOff_tmp)
      {
        //serial.println("Failed to Confirm Transition");
        
        if(bOnOff)
        {
          ////serial.println("SWITCH OFF");
          return false;
        }
        else
        {
          ////serial.println("SWITCH ON");
          return true;
        }
      }

      delay(500);
    }
    
    //Se a leitura for consistente
    //Faz o update do estado do switch e devolce o valor do mesmo
    //Switch detectado como ligado
    //Vai verificar o horario de funcionamento
    if(!bOnOff_tmp)
    {
      bOnOff = bOnOff_tmp;

      ////serial.println("SWITCH ON");
      return true;      
    }
    //Switch detectado como desligado
    else
    {
      bOnOff = bOnOff_tmp;

      ////serial.println("SWITCH OFF");
      return false;
    }
  }
  //Se nao detectar transicao
  //devolve o estado anterior
  else
  {
    if(bOnOff)
    {
      ////serial.println("SWITCH OFF");
      return false;
    }
    else
    {
      ////serial.println("SWITCH ON");
      return true;
    }
  }
  
  return true;
}

bool check_onoff_switch()
{ 
  bool bRet = true;
  
  //if(!digitalRead(main_switch))
  //Se o switch estiver ligado e estiver dentro do horario de trabalho
  //devolve true para o ciclo de controlo
  //if( (check_switch_state()) && (verifica_work_time()) )
  if(check_switch_state())
  {
    ////serial.println("ON");
        
    //digitalWrite(relay_0,0);
    control_rele(saida_state_pin[6], true);
    
    digitalWrite(motor_enable,1);
    digitalWrite(motor_enable_ampulheta,1);     

    int i=0;
    for(i=0; i<2 ;i++)
    {
      if(sm_motor[i] == HALT)
      {
        ////serial.print("ON");
        sm_motor[i] = START;

        last_year_received = 0;
        year_received = 0;
        prev_value = 9999;
        rx_buffer[0] = 0;
        rx_buffer[1] = 0;
        rx_buffer[2] = 0;
        rx_buffer[3] = 0;
      }
    }
  } 
  //Caso contrario devolve false para o ciclo de controlo
  else
  {   
    if(sm_motor[MOTOR_RELOGIO] != SHUTDOWN)
    {
      //serial.print("SHUTDOWN");
        
      digitalWrite(motor_enable,0);
      digitalWrite(motor_enable_ampulheta,0);

      int i=0;
      for(i=0; i<2 ;i++)
      {
        //Reset the state machines
        enc_count[i]      = 0;
        prev_enc_count[i] = 0;
        sm_motor[i]       = SHUTDOWN;
        sm_timer[i]       = millis() + 10000;
  
        //Send the shutdown signal to the raspberry
        Serial3.print("X");
        Serial3.print("X");
        Serial3.flush();   

        //serial.print('X');
      }
    }
    
    bRet = false;
  }

  return bRet;
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
  
void loop() 
{ 
  wdt_reset();
  
  /*static bool bInit = false;
  while(digitalRead(A3) && !bInit)
  {
    //serial.println("A");
  }
  bInit = true;*/
  
  if(check_onoff_switch())
  {
    update_motor(MOTOR_RELOGIO);
    update_motor(MOTOR_AMPULHETA);
    
    update_lever(MOTOR_RELOGIO);
    update_lever(MOTOR_AMPULHETA);
  
    process_serial();
  } else {
    //serial.println("not updating");
  }
  
  motor_statemachine();
  ampulheta_state_machine();
  caixa_statemachine();

  ////serial.println(freeRam()); 
}

void serialEvent3()
{ 
  uint8_t i = 0;
  uint8_t j = 0;
  while ((Serial3.available()) && (j<10))
  {
    received = (char)Serial3.read();
    
    for(i=1;i<BUFFER_SIZE;i++)
    {
      rx_buffer[i-1]=rx_buffer[i];
    }
    
    rx_buffer[BUFFER_SIZE-1] = received;

    j++;
  }

  //int value_ = (rx_buffer[1]-'0')*1000 + (rx_buffer[2]-'0')*100 + (rx_buffer[3]-'0')*10 + (rx_buffer[4]-'0');
  ////serial.print("year:");
  ////serial.print(value_);
  
  /*if(last_year_received == value_)
  {
    Serial3.print('r');
    Serial3.flush();

    //serial.print('r');
  }*/
}

