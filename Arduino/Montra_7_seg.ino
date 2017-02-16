#include <Servo.h>

#define S_A   36 //
#define S_B   24 // 
#define S_C   28 //
#define S_D   32 //
#define S_E   40 //    
#define S_F   42 //
#define S_G   44 //
#define S_DP  22 //

#define S_1   26 //
#define S_2   30 //
#define S_3   34 //
#define S_4   38 //


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(S_A, OUTPUT);
  pinMode(S_B, OUTPUT);
  pinMode(S_C, OUTPUT);
  pinMode(S_D, OUTPUT);
  pinMode(S_E, OUTPUT);
  pinMode(S_F, OUTPUT);
  pinMode(S_G, OUTPUT);
  pinMode(S_DP, OUTPUT);

  pinMode(S_1, OUTPUT);
  pinMode(S_2, OUTPUT);
  pinMode(S_3, OUTPUT);
  pinMode(S_4, OUTPUT);
   

  digitalWrite( S_A ,0);
  digitalWrite( S_B,0);
  digitalWrite( S_C,0);
  digitalWrite( S_D,0);
  digitalWrite( S_E,0);
  digitalWrite( S_F,0);
  digitalWrite( S_G,0);
  digitalWrite( S_DP,0);

  digitalWrite(S_1 ,1);
  digitalWrite(S_2 ,1);
  digitalWrite(S_3 ,1);
  digitalWrite(S_4 ,1);
  

  
}


void select_digit(int number){
  digitalWrite(S_A,1);
  digitalWrite(S_B,1);
  digitalWrite(S_C,1);
  digitalWrite(S_D,1);
  digitalWrite(S_E,1);
  digitalWrite(S_F,1);
  digitalWrite(S_G,1);
  digitalWrite(S_DP,1);

  switch(number){
    case 1:
      digitalWrite(S_B,0);
      digitalWrite(S_C,0);  
    break;
    case 2:
      digitalWrite(S_A,0);
      digitalWrite(S_B,0);  
      digitalWrite(S_G,0);  
      digitalWrite(S_E,0);  
      digitalWrite(S_D,0);  
    break;
    case 3:
      digitalWrite(S_A,0);  
      digitalWrite(S_B,0);
      digitalWrite(S_G,0);
      digitalWrite(S_C,0);
      digitalWrite(S_D,0);
      
    break;
    case 4:
      digitalWrite(S_F,0);
      digitalWrite(S_G,0);
      digitalWrite(S_B,0);
      digitalWrite(S_C,0);
        
    break;
    case 5:
      digitalWrite(S_A,0);
      digitalWrite(S_F,0);
      digitalWrite(S_G,0);
      digitalWrite(S_C,0);
      digitalWrite(S_D,0);
    break;
    case 6:
      //digitalWrite(S_A,0);
      digitalWrite(S_F,0);
      digitalWrite(S_G,0);
      digitalWrite(S_C,0);
      digitalWrite(S_D,0);
      digitalWrite(S_E,0);
        
    break;
    case 7:
      digitalWrite(S_A,0);
      digitalWrite(S_B,0);
      digitalWrite(S_C,0);
      
    break;
    case 8:
      digitalWrite(S_A,0);
      digitalWrite(S_B,0);
      digitalWrite(S_C,0);
      digitalWrite(S_D,0);
      digitalWrite(S_E,0);
      digitalWrite(S_F,0);
      digitalWrite(S_G,0);
      
    break;
    case 9:
      digitalWrite(S_A,0);
      digitalWrite(S_F,0);
      digitalWrite(S_G,0);
      digitalWrite(S_B,0);
      digitalWrite(S_C,0);
      //digitalWrite(S_D,0);
        
    break;
    case 0:
      digitalWrite(S_A,0);
      digitalWrite(S_B,0);
      digitalWrite(S_C,0);
      digitalWrite(S_D,0);
      digitalWrite(S_E,0);
      digitalWrite(S_F,0);
    break;
    
    
  }

  
}

void select_display(int digit){
  
  digitalWrite(S_1 ,0);
  digitalWrite(S_2 ,0);
  digitalWrite(S_3 ,0);
  digitalWrite(S_4 ,0);
  
  switch(digit){
      case 1:
        digitalWrite(S_1 ,1);
      break;
      case 2:
        digitalWrite(S_2 ,1);
      break;
      case 3:
        digitalWrite(S_3 ,1);
      break;
      case 4:
        digitalWrite(S_4 ,1);
      break;
  }
}

int digit_1 = 8;
int digit_2 = 8;
int digit_3 = 8;
int digit_4 = 8;


void set_7_segment_value(int value){
  static unsigned long timer = millis();
  static int rand_number = 0;

  if(value == 9999 && timer < millis()){
    
    timer = millis() + 100;
    rand_number = random(1,9999);
    
  }
  if(value == 9999)
    value = rand_number;

    
  digit_4 = value - (value /10)*10;
  value -= digit_4;
  digit_3 = (value - (value /100)*100)/10;
  value -= digit_3*10;
  digit_2 = (value - (value /1000)*1000)/100;
  value -= digit_2*100;
  digit_1 = (value - (value /10000)*10000)/1000;
  
  
}



void update_7_segment(){
  static unsigned long timer = millis();
  static int number = 0;

  if(timer < millis()){
    
    switch(number++){
      case 0:
        select_display(1);
        //timer = millis()+1;
      break;
      case 1: 
        select_display(0);
        select_digit(digit_2);
        break;
      case 2:
        select_display(2);
        //timer = millis()+1;
      break;
      case 3:
        select_display(0);
        select_digit(digit_3);
      case 4:
        select_display(3);
        //timer = millis()+1;
      break;
      case 5:
        select_display(0);
        select_digit(digit_4);
      case 6:
        select_display(4);
        //timer = millis()+1;
      break;    
      case 7:
        select_display(0);
        select_digit(digit_1);
        number = 0;
      break;

      
    }
    
    
    
  } 
  
  
}


enum states { 
  START,
  POINTER_1,
  POINTER_1_WAIT,
  POINTER_2,
  POINTER_2_WAIT
  
};


static char rx_buffer[6];

int process_serial(){
  static int prev_value = 9999;
  char i = 0;
  char test_number = 0;
  if(rx_buffer[0] == '-' && rx_buffer[5] == '/'){
    for(i=1;i<5;i++){
      if(rx_buffer[i] < '0' || rx_buffer[i] >'9')
        test_number = -1;
    }
    if(test_number == 0){
      prev_value = (rx_buffer[1]-'0')*1000 + (rx_buffer[2]-'0')*100 + (rx_buffer[3]-'0')*10 + (rx_buffer[4]-'0');
    }
  }
  
  return prev_value;
}

void loop() {
  static enum states sm = START;
  static unsigned long sm_timer = millis();

  update_7_segment();
  set_7_segment_value(process_serial());
}



void serialEvent1(){ 
  char i = 0;
  while (Serial1.available()) {
    char received = (char)Serial1.read();
    digitalWrite(13, !digitalRead(13));
    for(i=1;i<6;i++){
      rx_buffer[i-1]=rx_buffer[i];
    }
    rx_buffer[5] = received;
  }
  
}

