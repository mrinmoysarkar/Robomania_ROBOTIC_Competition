#include <Wire.h>
#include <math.h>
#include<UTFT.h>

#define HMC5883_WriteAddress 0x1E 
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand 0x00
#define HMC5883_DataOutputXMSBAddress  0x03

int regb=0x01;
int regbdata=0x40;
int outputData[6];

int lr_count = 0;
int gap_counter = 0;
int switch_count = 0;//1 for up 2 for down 3 for left 4 for right

boolean stop_flag = false;
boolean u_turn_flag = false;
boolean box_avoid_flag = false;
boolean l_flag = false;
boolean r_flag = false;
boolean intelligent_turn = false;
int turn_count = 0;
int no_of_sensor = 8;
int sen_input_pin[8] = {
  0,1,2,3,4,5,6,7};
int light_sensor_data_pid[8];
int light_sensor_data;
int light_sensor_calibrated_max_data[8] = {
  0,0,0,0,0,0,0,0};
int light_sensor_calibrated_min_data[8] = {
  1023,1023,1023,1023,1023,1023,1023,1023};
int thresh_value[8] = {
  128,128,128,128,128,128,128,128};

boolean left_flag = false;
boolean right_flag = false;
boolean straight_flag = false;
boolean forward_flag = false;


int angle_rot = 70;
int l_r_select = 0;//0 for left 1 for right
int base_speed = 150;
int thresh_distance = 15;

int angle_rotate = 75;
int speed_ba = 50;
int right_distance = 15;
int straight_distance = 30;
int straight_speed = 100;
int input_time = 2;
int calibration_time = 3000;

int count_ = 0;
int dus_flag = 100;

int sonor_vcc = 7;
int sonor_gnd = 4;
int sonor_trigger_pin = 6;
int sonor_input_pin = 5;

int motor_left_forward = 11;
int motor_left_backward = 10;
int motor_right_forward = 8;
int motor_right_backward = 9;

int compass_vcc = 19;
int compass_gnd = 18;



double start_angle = 0;



//tft-lcd
int sda = 28;//
int clk = 27;//
int cs = 26;
int rst = 30;//
int dc = 29;

//int lcd_ground = 25;
//int lcd_vcc = 27;
int lcd_bl = 12;

int led_yello = 2;
int led_white = 3;
int sw_right = 22;
int sw_left = 23;
int sw_down = 24;
int sw_up = 25;

int count = 0;
boolean count_flag = true;

int height = 0;
int width = 0;

boolean flag_t1 = false;
boolean flag_t2 = false;
boolean flag_t3 = false;
boolean flag_t4 = false;
boolean flag_t5 = false;


boolean flag = false;
boolean def_scr_flag = true;
double angle_l = 0;
String light_digital_data = "";

UTFT myGLCD(ST7735,sda,clk,cs,rst,dc);
extern uint8_t SmallFont[];
extern uint8_t BigFont[];


void setup()
{

  pinMode(led_yello,OUTPUT);
  pinMode(led_white,OUTPUT);

  pinMode(sw_right,INPUT);
  pinMode(sw_left,INPUT);
  pinMode(sw_down,INPUT);
  pinMode(sw_up,INPUT);

  pinMode(lcd_bl,OUTPUT);
  digitalWrite(lcd_bl,HIGH);

  randomSeed(analogRead(0));
  myGLCD.InitLCD(LANDSCAPE);
  myGLCD.setFont(SmallFont);
  myGLCD.clrScr();
  myGLCD.fillScr(200,25,155);
  height = myGLCD.getDisplayYSize();
  width = myGLCD.getDisplayXSize(); 
  myGLCD.setBackColor(VGA_FUCHSIA);

  pinMode(motor_left_forward,OUTPUT);
  pinMode(motor_left_backward,OUTPUT);
  pinMode(motor_right_forward,OUTPUT);
  pinMode(motor_right_backward,OUTPUT);

  pinMode(sonor_vcc,OUTPUT);
  digitalWrite(sonor_vcc,HIGH);
  pinMode(sonor_gnd,OUTPUT);
  digitalWrite(sonor_gnd,LOW);
  pinMode(sonor_trigger_pin,OUTPUT);
  pinMode(sonor_input_pin,INPUT);

  pinMode(compass_vcc,OUTPUT);
  pinMode(compass_gnd,OUTPUT);
  digitalWrite(compass_vcc,HIGH);
  digitalWrite(compass_gnd,LOW);


  Serial3.begin(9600);
  Serial.begin(9600);
  Wire.begin(); 
  //start_angle = read_angle();
  //Serial3.println("Start angle **********");
  default_screen();
  delay(400);
  //debug_rotation();

}

void loop()
{
  //debug_using_bluetooth();
  //debug_using_switch();
  //check_sensor_data_analog();
  debug_using_switch();
}
//end of loop

void flow_line(int speed_)
{  
  //Serial3.println("gap counter:");
  //Serial3.println(gap_counter);
  if(stop_flag)
  {
    //return;
  }
  if(u_turn_flag && (gap_counter > 20))
  {
    go_straight(20,1);
    gap_counter = 0;
    u_turn();
    //delay(2000);
  }
  get_light_sensor_data();

  int extra_rotate = light_sensor_data & 0x0041;//& 0x21;
  light_sensor_data >>=1; 
  light_sensor_data &=0x001F;
  if(extra_rotate == 0b1000001 && l_flag && r_flag)
  {
    if(lr_count != 1 || count_ > 10)
    {
      turn_count += 1;
      count_ = 0;
      lr_count = 1;
      if(intelligent_turn)
      {
        Left_motor_speed(0);
        Right_motor_speed(0);
        if(((turn_count == 2) && flag_t1)||((turn_count == 3) && flag_t2)||((turn_count == 8) && flag_t3)||((turn_count == 13) && flag_t4)||((turn_count == 14) && flag_t5))
        {
          right_priority(speed_*.5,true);
        }
        else
        {
          left_priority(speed_*.5,true);
        }
      }
      else
      {
        left_priority(speed_*.5,true);
      }
    }
    gap_counter = 0;
    l_flag = false;
    r_flag = false;
  }
  else if(extra_rotate == 0b1000000 && l_flag)//
  {
    if(lr_count != 2 || count_ > 10)
    {
      turn_count += 1;
      count_ = 0;
      lr_count = 2;
      if(intelligent_turn)
      {
        Left_motor_speed(0);
        Right_motor_speed(0);
        if(((turn_count == 2) && flag_t1)||((turn_count == 3) && flag_t2)||((turn_count == 8) && flag_t3)||((turn_count == 13) && flag_t4)||((turn_count == 14) && flag_t5))
        {
          right_priority(speed_*.5,false);
        }
        else
        {
          left_priority(speed_*.5,true);
        }
      }
      else
      {
        left_priority(speed_*.5,true);
      }
    }
    gap_counter = 0;
    l_flag = false;
    r_flag = false;
  }
  else if(extra_rotate == 0b0000001 && r_flag)
  {
    if(lr_count != 3 || count_ > 10)
    {
      turn_count += 1;
      count_ = 0;
      lr_count = 3;
      if(intelligent_turn)
      {
        Left_motor_speed(0);
        Right_motor_speed(0);
        if(((turn_count == 2) && flag_t1)||((turn_count == 3) && flag_t2)||((turn_count == 8) && flag_t3)||((turn_count == 13) && flag_t4)||((turn_count == 14) && flag_t5))
        {
          right_priority(speed_*.5,true);
        }
        else
        {
          left_priority(speed_*.5,false);
        }
      }
      else
      {
        left_priority(speed_*.5,false);
      }
    }
    gap_counter = 0;
    l_flag = false;
    r_flag = false;
  }

  count_++;
  get_light_sensor_data();
  light_sensor_data >>=1; 
  light_sensor_data &=0x001F;
  //else
  {
    switch(light_sensor_data)
    {
    case 0:
      {
        gap_counter++;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 4:
    case 14:
      {
        Left_motor_speed(speed_);
        Right_motor_speed(speed_);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 1:
      {
        Right_motor_speed(-speed_*.8);
        Left_motor_speed(speed_);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 16:
      {  
        Left_motor_speed(-speed_*.8);
        Right_motor_speed(speed_);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 2:
      { 
        Right_motor_speed(speed_*.7); 
        Left_motor_speed(speed_*1.4);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 8:
      {  
        Left_motor_speed(speed_*.7);
        Right_motor_speed(speed_*1.4);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 3:
      {
        Left_motor_speed(speed_*1.6);
        Right_motor_speed(speed_*.4);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 24:
      {
        Right_motor_speed(speed_*1.6);
        Left_motor_speed(speed_ *.4);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 6:
      {
        Right_motor_speed(speed_*1.2);
        Left_motor_speed(speed_*.7);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 12:
      {
        Right_motor_speed(speed_*.7);
        Left_motor_speed(speed_*1.2);
        gap_counter = 0;
        l_flag = false;
        r_flag = false;
      }
      break;
    case 7:
    case 15:
      {
        //Serial3.println("need right turn");
        //left_priority(speed_*.6,false);
        r_flag = true;
        gap_counter=0;
      }
      break;
    case 28:
    case 30:
      {
        gap_counter = 0;
        //Serial3.println("need left turn");
        l_flag = true;
        //left_priority(speed_*.6,true);
      }
      break;
    case 31:
      {
        gap_counter = 0;
        l_flag = true;
        r_flag = true;
        //Serial3.println("need both turn");
        //left_priority(speed_*.6,true); 
      }
      break;
    case 5:
    case 13:
      {
        //Serial3.println("in right 45 degree");
      }
      break;
    case 20:
    case 22:
      {
        //Serial3.println("in left 45 degree");
      }
      break;
    }
  }
  //Serial3.println(light_sensor_data,BIN);
}

void left_priority(int speed_,boolean left_right)//true for left false for right
{
  unsigned long time;
  Left_motor_speed(0);
  Right_motor_speed(0);
  go_straight(8,0);
  delay(10);

  get_light_sensor_data();
  if((light_sensor_data & 0b01111111) == 0b01111111)
  {
    //Serial3.println("in stop ");
    stop_flag = true;
    return;
  }
  if((light_sensor_data & 0b00011100) != 0 && !left_right && !left_flag)
  {
    go_straight(4,0);
    get_light_sensor_data();
    if((light_sensor_data & 0b00011100) == 0)
    {
      goto right_ten;
    }
    //Serial3.println("returning from straight\n");
    return;
  }
  //go_straight(3,1);
  if(left_flag || left_right)
  {
    left_flag = false;
    time = millis();
    //Serial3.println("in1:");
    while(true && (millis()-time)<1000)
    {
      get_light_sensor_data();
      if(((light_sensor_data & 0b00001000)==0)&&((light_sensor_data & 0b00100000) == 0b00100000 ||(light_sensor_data & 0b00000100) == 0b00000100 || (light_sensor_data & 0b00000010) == 0b00000010))
      {
        break;
      }
      Left_motor_speed(-speed_);
      Right_motor_speed(speed_);
      //Serial3.println(light_sensor_data,BIN);
    }
    //Left_motor_speed(-speed_);
    //Right_motor_speed(speed_);
    //delay(600);
    time = millis();
    while(true && (millis()-time)<1000)
    {
      get_light_sensor_data();
      if((light_sensor_data & 0b00101010)== 0b00001000)
      {
        break;
      }
      Left_motor_speed(-speed_);
      Right_motor_speed(speed_);
      //Serial3.println(light_sensor_data,BIN);
    }
    Left_motor_speed(0);
    Right_motor_speed(0);
    //Serial3.println("returning from Left\n");
    //go_straight(3,0);
    return;
  }  
right_ten:
  time = millis();
  if((light_sensor_data & 0b00111110) == 0 && !left_right)
  {
    time = millis();
    while(true && (millis()-time)<1200)
    {
      Left_motor_speed(speed_);
      Right_motor_speed(-speed_);
      get_light_sensor_data();
      //  Serial3.println(light_sensor_data,BIN);
      if((light_sensor_data & 0b00101010) == 0b00001000)
      {
        break;
      }
    }
    //go_straight(5,0);
    //Serial3.println("returning from Right\n");    
    return;
  }
}

void right_priority(int speed_,boolean left_right)//true for right false for left
{
  unsigned long time;
  Left_motor_speed(0);
  Right_motor_speed(0);
  go_straight(8,0);
  delay(10);

  get_light_sensor_data();
  if((light_sensor_data & 0b01111111) == 0b01111111)
  {
    Serial3.println("in stop ");
    stop_flag = true;
    return;
  }
  if((light_sensor_data & 0b00011100) != 0 && !left_right && !right_flag)
  {
    go_straight(4,0);
    get_light_sensor_data();
    if((light_sensor_data & 0b00011100) == 0)
    {
      goto left_ten;
    }
    //Serial3.println("returning from straight\n");
    return;
  }
  //go_straight(3,1);
  if(right_flag || left_right)
  {
    right_flag = false;
    time = millis();
    //Serial3.println("in1:");
    while(true && (millis()-time)<1000)
    {
      get_light_sensor_data();
      if(((light_sensor_data & 0b00001000)==0)&&((light_sensor_data & 0b00000010) == 0b00000010 ||(light_sensor_data & 0b00010000) == 0b00010000 || (light_sensor_data & 0b00100000) == 0b00100000))
      {
        break;
      }
      Left_motor_speed(speed_);
      Right_motor_speed(-speed_);
      //Serial3.println(light_sensor_data,BIN);
    }
    //Left_motor_speed(speed_);
    //Right_motor_speed(-speed_);
    //delay(600);
    time = millis();
    //Serial3.println("in2:");
    while(true && (millis()-time)<1000)
    {
      get_light_sensor_data();
      if((light_sensor_data & 0b00101010)== 0b00001000)
      {
        break;
      }
      Left_motor_speed(speed_);
      Right_motor_speed(-speed_);
      //Serial3.println(light_sensor_data,BIN);
    }
    Left_motor_speed(0);
    Right_motor_speed(0);
    Serial3.println("returning from Right\n");
    //go_straight(3,0);
    return;
  }  
left_ten:
  time = millis();
  if((light_sensor_data & 0b00111110) == 0 && !left_right)
  {
    time = millis();
    while(true && (millis()-time)<1200)
    {
      Left_motor_speed(-speed_);
      Right_motor_speed(speed_);
      get_light_sensor_data();
      //  Serial3.println(light_sensor_data,BIN);
      if((light_sensor_data & 0b00101010) == 0b00001000)
      {
        break;
      }
    }
    //go_straight(5,0);
    //  Serial3.println("returning from Right\n");    
    return;
  }
}

void u_turn()
{
  Left_motor_speed(-100);
  Right_motor_speed(100);
  get_light_sensor_data();
  unsigned long time = millis();
  while((light_sensor_data & 0b01111111) != 0 || (millis()-time)<300)
  {
    get_light_sensor_data();
  }
  Left_motor_speed(-70);
  Right_motor_speed(70);
  while((light_sensor_data & 0b00101010) != 0b00001000)
  {
    get_light_sensor_data();
  }
  Left_motor_speed(-30);
  Right_motor_speed(30);  
}

void rotate_using_time(double angle,int directi)//0 for left 1 for right
{
  angle = angle * 100;
  unsigned long time = millis();
  while((millis() - time) < angle)
  {
    if(directi == 0)
    {
      Left_motor_speed(-100);
      Right_motor_speed(100);
    }
    else
    {
      Left_motor_speed(100);
      Right_motor_speed(-100);
    }
  }
  Left_motor_speed(0);
  Right_motor_speed(0);
}

void switch_pressed()
{
  switch_count = 0;
  if(digitalRead(sw_up) == LOW)
  {
    switch_count = 1;
  }
  else if(digitalRead(sw_down) == LOW)
  {
    switch_count = 2;
  }
  else if(digitalRead(sw_left) == LOW)
  {
    switch_count = 3;
  }
  else if(digitalRead(sw_right) == LOW)
  {
    switch_count = 4;
  }
}

void value_setup(String s,int tag,int time)
{
}

void debug_using_switch()
{
  myGLCD.print("1.To calibrat prs up",0,0);
  myGLCD.print("2.To change spd down",0,12);
  myGLCD.print("3.To set flag left",0,24);
  myGLCD.print("4.To Run press right",0,36);
  switch_pressed();
  if(switch_count == 1)
  {
    switch_count = 0;
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
    myGLCD.print(" Calibrating sensor  ",0,0);
    switch_count = 0;
    delay(500);
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
    calibrate_light_sensor(base_speed);
  }
  else if(switch_count == 2)
  {
    switch_count = 0;
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
    while(true)
    {
      myGLCD.print(" press up to inc spd   ",0,0);
      myGLCD.print(" press down to dec spd   ",0,12);
      myGLCD.print(" press right to back   ",0,24);
      switch_pressed();
      if(switch_count == 1)
      {
        base_speed += 5;
        myGLCD.print("current speed: ",0,36);
        myGLCD.print("            ",110,36);
        myGLCD.printNumI(base_speed,110,36);
        myGLCD.print("            ",134,36);
        switch_count = 0;
      }
      else if(switch_count == 2)
      {
        switch_count = 0;
        base_speed -= 5;
        myGLCD.print("current speed: ",0,36);
        myGLCD.print("            ",110,36);
        myGLCD.printNumI(base_speed,110,36);
        myGLCD.print("             ",134,36);
      }
      else if(switch_count == 4)
      {
        switch_count = 0;
        myGLCD.print("  Returning         ",0,36);
        delay(500);
        myGLCD.clrScr();
        myGLCD.setColor(0,255,0);
        break;    
      }
    }
  }
  else if(switch_count == 3)
  {
    switch_count = 0;
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
    while(true)
    {
      myGLCD.print("u tern flag prs up",0,0);
      myGLCD.print("box flag prs down ",0,20);
      myGLCD.print("intelligent turn left ",0,40);
      switch_pressed();
      if(switch_count == 1)
      {
        myGLCD.clrScr();
        myGLCD.setColor(0,255,0);
        switch_count = 0;
        if(u_turn_flag)
        {
          u_turn_flag = false;
          myGLCD.print("u tern flag false",0,0);
        } 
        else
        {
          u_turn_flag = true; 
          myGLCD.print("u tern flag true",0,0);
        }
        break;
      }
      else if(switch_count == 2)
      {
        myGLCD.clrScr();
        myGLCD.setColor(0,255,0);
        switch_count = 0;
        if(box_avoid_flag)
        {
          box_avoid_flag = false;
          myGLCD.print("box avd flag false",0,0);
        } 
        else
        {
          box_avoid_flag = true; 
          myGLCD.print("box avd flag true",0,0);
        }
        break;
      }
      else if(switch_count == 3)
      {
        myGLCD.clrScr();
        myGLCD.setColor(0,255,0);
        switch_count = 0;
        if(intelligent_turn)
        {
          intelligent_turn = false;
          myGLCD.print("intelignt flag false",0,0);
        } 
        else
        {
          intelligent_turn = true; 
          myGLCD.print("intelignt flag true",0,0);
          boolean lop = true;
          while(lop)
          {
            String flg = "";
            myGLCD.print("U. flag_t1 ",0,15);
            flg = flag_t1 == true?"true":"false";
            myGLCD.print(flg,100,15);
            myGLCD.print("D. flag_t2 ",0,30);
            flg = flag_t2 == true?"true":"false";
            myGLCD.print(flg,100,30);
            myGLCD.print("L. flag_t3 ",0,45);
            flg = flag_t3 == true?"true":"false";
            myGLCD.print(flg,100,45);
            myGLCD.print("R. next",0,60);
            switch_pressed();
            switch(switch_count)
            {
            case 1:
              {
                flag_t1 = !flag_t1;
                flg = flag_t1 == true?"true":"false";
                myGLCD.print(flg,100,15);
                delay(100);
              }
              break;
            case 2:
              {
                flag_t2 = !flag_t2;
                flg = flag_t2 == true?"true":"false";
                myGLCD.print(flg,100,30);
                delay(100);
              }
              break;
            case 3:
              {
                flag_t3 = !flag_t3;
                flg = flag_t3 == true?"true":"false";
                myGLCD.print(flg,100,45);
                delay(100);
              }
              break;
            case 4:
              {
                while(true)
                {
                  myGLCD.print("U. flag_t4 ",0,75);
                  flg = flag_t4 == true?"true":"false";
                  myGLCD.print(flg,100,75);
                  myGLCD.print("D. flag_t5 ",0,90);
                  flg = flag_t5 == true?"true":"false";
                  myGLCD.print(flg,100,90);
                  myGLCD.print("R. BACK",30,115);
                  switch_pressed();
                  switch(switch_count)
                  {
                  case 1:
                    {
                      flag_t4 = !flag_t4;
                      flg = flag_t4 == true?"true":"false";
                      myGLCD.print(flg,100,75);
                      delay(100);
                    }
                    break;
                  case 2:
                    {
                      flag_t5 = !flag_t5;
                      flg = flag_t5 == true?"true":"false";
                      myGLCD.print(flg,100,90);
                      delay(100);
                    }
                    break;
                  }
                  if(switch_count == 4)
                  {
                    lop = false;
                    break;
                  }
                  switch_count = 0;
                }
              }
              break;

            }
            switch_count = 0;
          }
        }
        break;
      }
    }
    delay(3000);
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
  }
  else if(switch_count == 4)
  {
    switch_count = 0;
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
    myGLCD.print(" Starting the Race",0,20);
    myGLCD.print(" Pray for Me",0,40);
    win_the_race();
    delay(3000);
    myGLCD.clrScr();
    myGLCD.setColor(0,255,0);
  }
}
void win_the_race()
{
  unsigned long time = millis();
  boolean v = true;
  stop_flag = false;
  turn_count = 0;
  go_straight_from_start_point(40,0);
  while(v)
  {
    if(get_distance() < thresh_distance && box_avoid_flag)
    {
      avoid_box_time_version();
    }
    else
    {
      flow_line(base_speed);
    }
    if(stop_flag)
    {
      //break;
      unsigned long time1 = millis();
      while((millis()-time1)<100)
      {
        get_light_sensor_data();
        if((light_sensor_data & 0b01111111) != 0b01111111)
        {
          Left_motor_speed(0);
          Right_motor_speed(0);
          break;
        }
        Left_motor_speed(27);
        Right_motor_speed(27);
      }
      if((millis()-time1)>=90)
      {
        Left_motor_speed(0);
        Right_motor_speed(0);
        v = false;
      }
      else
      {
        stop_flag = false;
      }
    }
  }
  time = (millis()-time)/1000;
  myGLCD.print("LAP Time: ",0,40);
  myGLCD.printNumI(time,100,40);
  myGLCD.print("Seconds",60,60);
  myGLCD.print("Turn No: ",0,80);
  myGLCD.printNumI(turn_count,100,80);
  delay(3000);
}
void avoid_box()
{
  rotate(angle_rotate,speed_ba,0);//angle_rotate,speed_ba
  delay(10);
  go_straight(right_distance,0);//right_distance = 15
  delay(10);
  rotate(angle_rotate,speed_ba,1);//angle_rotate
  delay(10);
  go_straight(straight_distance,0);//straight_distance = 50
  delay(10);
  rotate(angle_rotate,speed_ba,1);
  delay(10);
  go_straight(right_distance,0);
  delay(10);
  rotate(angle_rotate,speed_ba,0);
  delay(10);
}
void avoid_box_time_version()
{
  rotate_using_time(6,0);//angle_rotate,speed_ba
  delay(10);
  go_straight_a(13,0);//right_distance = 15
  delay(10);
  rotate_using_time(5.6,1);//angle_rotate
  delay(10);
  go_straight_a(46,0);//straight_distance = 50
  delay(10);
  rotate_using_time(7,1);
  delay(10);
  go_straight_sp(20,0);
  delay(10);
  rotate_using_time(6,0);
  delay(10);
}
void go_straight(int distance,int direct)
{
  int time = 45 * distance + 183;
  unsigned long start = millis();
  double start_angle = read_angle();
  boolean gap = false;
  boolean state = false;
  get_light_sensor_data();
  if((light_sensor_data & 0b01111111) == 0 )
  {
    gap = true;
  }
  if((light_sensor_data & 0b01100011) == 0)
  {
    state = true;
  }
  while((millis() - start) < time)
  {

    if((direct == 0) && !gap && !state)
    {
      get_light_sensor_data();
      if((light_sensor_data & 0b01000000) != 0)
      {
        left_flag = true;
      }
      else if((light_sensor_data & 0b00000001) != 0)
      {
        right_flag = true;
      }
      else if((light_sensor_data & 0b01100011) == 0)
      {
        if((light_sensor_data & 0b01111111) != 0)
        {
          forward_flag = true;
        }
        break;
      }
    }
    if((direct == 0) && state)
    {
      get_light_sensor_data();
      if((light_sensor_data & 0b01111111) == 0)
      {
        break;
      }
    }
    go_using_compass(start_angle,direct,straight_speed);//straight_speed = 100
  }
  Left_motor_speed(0);
  Right_motor_speed(0);
}
void go_straight_from_start_point(int distance,int direct)
{
  int time = 45 * distance + 183;
  unsigned long start = millis();
  double start_angle = read_angle();
  boolean gap = false;
  boolean state = false;
  get_light_sensor_data();
  while((millis() - start) < time)
  {
    get_light_sensor_data();
    if((light_sensor_data & 0b01100011) == 0)
    {
      break;
    }
    go_using_compass(start_angle,direct,straight_speed);//straight_speed = 100
  }
  Left_motor_speed(0);
  Right_motor_speed(0);
}
void go_straight_sp(int distance,int direct)
{
  int time = 45 * distance + 183;
  unsigned long start = millis();
  double start_angle = read_angle();
  boolean first= false;
  boolean second = false;
  get_light_sensor_data();
  while((millis() - start) < time)
  {
    get_light_sensor_data();
    if((light_sensor_data & 0b01000001) > 0)
    {
      first = true;
      second = false;
    }
    if((light_sensor_data & 0b01000001) == 0)
    {
      second = true;
    }
    if(first && second)
    {
      break;
    }
    go_using_compass(start_angle,direct,straight_speed);//straight_speed = 100
  }
  Left_motor_speed(0);
  Right_motor_speed(0);
}
void go_straight_a(int distance,int direct)
{
  int time = 45 * distance + 183;
  unsigned long start = millis();
  double start_angle = read_angle();
  while((millis() - start) < time)
  {
    go_using_compass(start_angle,direct,straight_speed);//straight_speed = 100
  }
  Left_motor_speed(0);
  Right_motor_speed(0);
}
void rotate(double rotate_ang,int speed_,int direct)//direct = 1 right rotation direct = 0 left rotation
{
  Left_motor_speed(0);
  Right_motor_speed(0);
  rotate_ang = rotate_ang;
  double angle1 = read_angle();
  Serial3.println("Srarting angle");
  Serial3.println(angle1);
  Serial3.println("*************");
  double min_ang = (angle1 - rotate_ang) < 0 ? (360 + angle1 - rotate_ang):(angle1 - rotate_ang);
  double max_ang = angle1 + rotate_ang;
  max_ang = max_ang > 360?max_ang-360:max_ang;
  while(1)
  {
    double angle2 = read_angle();
    if(direct == 1 && abs(angle2 - max_ang) < 4)
    {
      Left_motor_speed(0);
      Right_motor_speed(0);
      Serial3.println("Stoping angle");
      Serial3.println(angle2);
      Serial3.println("*************");
      break;
    }
    else if(abs(angle2-min_ang) < 4 && direct == 0)
    {
      Left_motor_speed(0);
      Right_motor_speed(0);
      Serial3.println("Stoping angle");
      Serial3.println(angle2);
      Serial3.println("*************");
      break;
    }
    else if(direct == 0)
    {
      Left_motor_speed(-speed_);
      Right_motor_speed(speed_);
    }
    else
    {
      Left_motor_speed(speed_);
      Right_motor_speed(-speed_);
    }
  }
}
void go_using_compass(double start_angle,int direct,int spd)//direct=0 for forward direct=1 for backward
{
  double ang = read_angle();
  int speed_ = direct == 0?spd:-spd;
  if(abs(ang - start_angle)<3)
  {
    Left_motor_speed(speed_);
    Right_motor_speed(speed_);
    //Serial3.println(ang);
    //Serial3.println("start_ang: ");
    // Serial3.println(start_angle);
  }
  else if((ang-start_angle)<0 && (direct == 0))
  {
    Left_motor_speed(speed_ + speed_*.5);
    Right_motor_speed(speed_);
  }
  else if((ang-start_angle)>0 && (direct == 0))
  {
    Left_motor_speed(speed_);
    Right_motor_speed(speed_ + speed_*.5);
  }
  else if((ang-start_angle)<0 && (direct == 1))
  {
    Left_motor_speed(speed_);
    Right_motor_speed(speed_ + speed_*.5); 
  }
  else if((ang-start_angle)>0 && (direct == 1))
  {
    Left_motor_speed(speed_ + speed_*.5);
    Right_motor_speed(speed_);
  }
}


void calibrate_light_sensor(int spd)
{
  unsigned long time = millis();
  Left_motor_speed(constrain(spd,-255,255));
  Right_motor_speed(-constrain(spd,-255,255));
  while((millis() - time) < calibration_time)
  {
    for(int i = 0;i < no_of_sensor;i++)
    {
      int data = analogRead(sen_input_pin[i]);
      if(data >= light_sensor_calibrated_max_data[i])
      {
        light_sensor_calibrated_max_data[i] = data;
      }
      else if(data <= light_sensor_calibrated_min_data[i])
      {
        light_sensor_calibrated_min_data[i] = data;
      }
    }
  }
  myGLCD.setColor(0,255,0);
  for(int i = 0;i < no_of_sensor;i++)
  {
    myGLCD.printNumI(light_sensor_calibrated_max_data[i],0,i*14);
    myGLCD.printNumI(light_sensor_calibrated_min_data[i],70,i*14);
  }
  Left_motor_speed(0);
  Right_motor_speed(0);
  delay(3000);
  myGLCD.clrScr();
  myGLCD.setColor(0,255,0);


  Serial3.println("max sensor data");
  for(int i = 0;i < no_of_sensor;i++)
  {
    Serial3.print(light_sensor_calibrated_max_data[i]);
    Serial3.print("    ");
  }
  Serial3.println("\nmin sensor data");
  for(int i= 0;i < no_of_sensor;i++)
  {
    Serial3.print(light_sensor_calibrated_min_data[i]);
    Serial3.print("    ");
  }

  Left_motor_speed(0);
  Right_motor_speed(0);
}
void Left_motor_speed(int _speed)
{
  if(_speed >= 0)
  {
    analogWrite(motor_left_forward,constrain(_speed,-255,255));
    analogWrite(motor_left_backward,0);
  }
  else
  {
    analogWrite(motor_left_forward,0);
    analogWrite(motor_left_backward,constrain(-_speed,-255,255));
  }
}
void Right_motor_speed(int _speed)
{
  if(_speed >= 0)
  {
    analogWrite(motor_right_forward,constrain(_speed,-255,255));
    analogWrite(motor_right_backward,0);
  }
  else
  {
    analogWrite(motor_right_forward,0);
    analogWrite(motor_right_backward,constrain(-_speed,-255,255));
  }
}


double get_distance()
{
  digitalWrite(sonor_trigger_pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(sonor_trigger_pin,LOW);
  int i = pulseIn(sonor_input_pin,HIGH);
  double distance = 0.017 * i;//distance in cm
  delay(3);
  return distance;
}

void check_sensor_data_analog()
{
  Serial3.println("set start");
  Serial.println("set start");
  myGLCD.print("Analog Data:",0,108);
  for(int i = 0;i < no_of_sensor;i++)
  {
    Serial3.println(analogRead(sen_input_pin[i]));
    Serial.println(analogRead(sen_input_pin[i]));
    myGLCD.printNumI(analogRead(sen_input_pin[i]),40*(i%3),i<3?120:132);
    delay(100);
  }
  Serial3.println("set over");
  Serial.println("set over");
  delay(2000);
}

double read_angle() {

  int i,x,y,z;
  double angle;

  Wire.beginTransmission(HMC5883_WriteAddress);
  Wire.write(regb);
  Wire.write(regbdata);
  Wire.endTransmission();

  delay(50);
  Wire.beginTransmission(HMC5883_WriteAddress); //Initiate a transmission with HMC5883 (Write address).
  Wire.write(HMC5883_ModeRegisterAddress);       //Place the Mode Register Address in write-buffer.
  Wire.write(HMC5883_ContinuousModeCommand);     //Place the command for Continuous operation Mode in write-buffer.
  Wire.endTransmission();                       //write the write-buffer to HMC5883 and end the I2C transmission.
  delay(50);


  Wire.beginTransmission(HMC5883_WriteAddress);  //Initiate a transmission with HMC5883 (Write address).
  Wire.requestFrom(HMC5883_WriteAddress,6);      //Request 6 bytes of data from the address specified.

  delay(10);


  //Read the value of magnetic components X,Y and Z

  if(6 <= Wire.available()) // If the number of bytes available for reading be <=6.
  {
    for(i=0;i<6;i++)
    {
      outputData[i]=Wire.read();  //Store the data in outputData buffer
    }
  }

  x=outputData[0] << 8 | outputData[1]; //Combine MSB and LSB of X Data output register
  z=outputData[2] << 8 | outputData[3]; //Combine MSB and LSB of Z Data output register
  y=outputData[4] << 8 | outputData[5]; //Combine MSB and LSB of Y Data output register


  angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees


  //Serial3.println(": Angle between X-axis and the South direction ");
  //Serial3.println(angle,2);
  //Serial3.println(" Deg");
  return angle;
}

void debug_rotation()
{
  Serial3.println("enter rotate angle");
  while(!Serial3.available());
  int ang = Serial3.read();

  Serial3.println("angle rotate");
  Serial3.println(ang);
  Serial3.println("enter speed");
  while(!Serial3.available());
  int spd = Serial3.read();
  Serial3.println("bot speed");
  Serial3.println(spd);
  Serial3.println("enter direction 0 for left 1 for right");
  while(!Serial3.available());
  int d = Serial3.read();
  Serial3.println("direction");
  Serial3.println(d);
  Serial3.println("*************");
  rotate(ang,spd,d);
}
void debug_straight()
{
  Serial3.println("input direction 0 for forward 1 for backward");
  while(!Serial3.available());
  int d = Serial3.read();
  Serial3.println("direction");
  Serial3.println(d);
  Serial3.println("input distance");
  while(!Serial3.available());
  int tm = Serial3.read();
  Serial3.println("distance in cm");
  Serial3.println(tm);
  Serial3.println("*************");
  go_straight(tm,d);
}
void get_light_sensor_data()
{
  light_sensor_data = 0;
  for(int i = 0;i < no_of_sensor;i++)
  {
    int analog_data = constrain(analogRead(sen_input_pin[i]),light_sensor_calibrated_min_data[i],light_sensor_calibrated_max_data[i]);
    light_sensor_data_pid[i] = map(analog_data,light_sensor_calibrated_min_data[i],light_sensor_calibrated_max_data[i],0,255);
    if(light_sensor_data_pid[i] <= thresh_value[i])
    {
      light_sensor_data |= 1<<(no_of_sensor-1-i);
      //light_sensor_data_pid[i] = 1;
    }
    else
    {
      //light_sensor_data_pid[i] = 0;
    }
  }
}





void default_screen()
{
  myGLCD.setColor(0,0,0);
  myGLCD.setFont(BigFont);
  myGLCD.print("   ROBO     ",0,40);
  myGLCD.print("  KNIGHTS    ",0,56);
  delay(7000);
  myGLCD.setFont(SmallFont);
  myGLCD.clrScr();
  myGLCD.setColor(0,255,0);
}

void debug_using_bluetooth()
{
  Serial3.println("<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial3.println("enter /'0/' for debug rotation");
  Serial3.println("enter /'1/' for debug straight");
  Serial3.println("enter /'2/' for debug avoid box");
  Serial3.println("enter /'3/' for debug hill climb");
  Serial3.println("enter /'4/' for debug light sensor data");
  Serial3.println("enter /'5/' for debug line flower");
  Serial3.println("enter /'6/' for debug calibration");
  Serial3.println("enter /'7/' for debug read sensor data digital");
  Serial3.println("enter /'8/' for debug sonor");
  Serial3.println("enter /'9/' for debug box avoiding");
  Serial3.println("enter /'10/' for debug total code");
  Serial3.println("enter /'11/' for debug left Priority");
  Serial3.println("enter /'12/' for debug time rotation");
  Serial3.println("<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial3.print("<<<<moy>>>>");
  while(!Serial3.available());
  char ch = Serial3.read();
  switch(ch)
  {
  case 0:
    debug_rotation();
    break;
  case 1:
    debug_straight();
    break;
  case 2:
    {
      int min_dis = read_serial_data("Input min distance:");
      double start_angle = read_angle();
      while(1)
      {
        double distance = get_distance();
        if(distance < min_dis)
        {
          avoid_box();
          break;
        }
        else
        {
          go_using_compass(start_angle,0,100); 
        }
      }
    }
    break;
  case 3:
    {
      int time = read_serial_data("enter simulation time in s")*1000;
      int spd = read_serial_data("enter simulation speed 80-255");
      int d = read_serial_data("enter simulation direction");
      double st_angle = read_angle();
      Serial3.println("total time in ms");
      Serial3.println(time);
      unsigned long start = millis();
      Serial3.println("Start time in ms");
      Serial3.println(start);
      while((millis()-start) < time)
      {
        go_using_compass(st_angle,d,spd);
      }
      Left_motor_speed(0);
      Right_motor_speed(0);
    }
    break;
  case 4:
    {
      check_sensor_data_analog();
    }
    break;
  case 5:
    {
      int spd = read_serial_data("input base speed 30-255");
      int time = read_serial_data("enter simulation time in s")*1000;
      unsigned long strt = millis();
      while((millis() - strt)<time)
      {
        flow_line(spd);
      }
      Left_motor_speed(0);
      Right_motor_speed(0);
    }
    break;
  case 6:
    {
      int spd = read_serial_data("input base speed 30-255");
      calibrate_light_sensor(spd); 
    }
    break;
  case 7:
    {
      int time = read_serial_data("enter simulation time in s")*1000;
      unsigned long strt = millis();
      while((millis() - strt)<time)
      {
        get_light_sensor_data();
        Serial3.println("6 sensor");
        Serial3.println(light_sensor_data,BIN);
        light_sensor_data >>=1; 
        light_sensor_data &=0x000F;
        Serial3.println("4 sensor");
        Serial3.println(light_sensor_data,BIN);
        delay(1000);
      } 
    }
    break;
  case 8:
    {
      Serial3.print("Distance: ");
      Serial3.println(get_distance());
    }
    break;
  case 9:
    {
      unsigned long time = millis();
      avoid_box_time_version();
      Serial3.print("Time needed:  ");
      Serial3.println(millis()-time);
      Serial3.println();
    }
    break;
  case 10:
    {
      int time = read_serial_data("input simulation time in s: ")*1000;
      unsigned long st = millis();
      while((millis()-st)<time)
      {
        box_avoid_flag = true;
        win_the_race();
      }
      Left_motor_speed(0);
      Right_motor_speed(0);
    }
    break;
  case 11:
    {
      int left = read_serial_data("for left rotation enter 0 and 1 for right");
      int spd = read_serial_data("enter speed 0-255:");
      if(left == 0)
      {
        left_priority(spd,true);
      }
      else
      {
        left_priority(spd,false);
      }
      Left_motor_speed(0);
      Right_motor_speed(0);
    }
    break;
  case 12:
    {
      int direc_tion = read_serial_data("for left rotation enter 0 and 1 for right");
      int angle = read_serial_data("enter angle to rotate:");
      rotate_using_time(angle , direc_tion);
    }
    break;
  }  
}


int read_serial_data(String s)
{
  Serial3.println(s);
  while(!Serial3.available());
  int ch = Serial3.read();
  Serial3.println(ch);
  return ch;
}































