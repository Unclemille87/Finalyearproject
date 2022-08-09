#include <TimerOne.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>  
#include <RTClib.h>
//----------------------------------------------------------------------------------------------------------
 
// Load control algorithm
// 0 - NIGHT LIGHT: Load ON when there is no solar power and battery is above LVD 
// 1 - POWER DUMP: Load ON when there is solar power and the battery is above BATT_FLOAT (charged)

#define LOAD_ALGORITHM 0

#define SOL_VOLTS_CHAN 0               // defining the adc channel to read solar volts
#define BAT_VOLTS_CHAN 2               // defining the adc channel to read battery volts
#define INV_AMPS_CHAN 3                // Defining the adc channel to real inverter amps
#define SOL_AMPS_CHAN 1                // Defining the adc channel to read solar amps
#define AVG_NUM 20                     // number of iterations  to average the adc readings

// ACS 712 Current Sensor is used. Current Measured = (5/(1024 *0.185))*ADC - (2.5/0.185) 

#define SOL_AMPS_SCALE  0.0409 // the scaling value for raw adc reading to get solar amps   
#define SOL_VOLTS_SCALE 0.05385      // the scaling value for raw adc reading to get solar volts… (5/1024)*(R1+R2)/R2 // R1=100k and R2=20k
#define BAT_VOLTS_SCALE 0.05385      // the scaling value for raw adc reading to get battery volts 
#define INV_AMPS_SCALE 0.0409
#define PWM_PIN 9               // the output pin for the pwm ( avaliable for timer 1 at 50kHz)
#define ENABLE_PWM_PIN 8            // used to shut off IR2104 MOSFET 
#define PWM_FULL 1023                // the actual value used by the Timer1 routines for 100% pwm duty cycle
#define PWM_MAX 100                  
#define PWM_MIN 60                  
#define PWM_START 90                
#define PWM_INC 1                    //the value the increment to the pwm value for the ppt algorithm

#define TRUE 1
#define FALSE 0
#define ON TRUE
#define OFF FALSE

#define TURN_ON_MOSFETS digitalWrite(ENABLE_PWM_PIN, HIGH)      // enable MOSFET driver
#define TURN_OFF_MOSFETS digitalWrite(ENABLE_PWM_PIN, LOW)      // disable MOSFET driver
#define ONE_SECOND 50000   //count for number of interrupt in 1 second on interrupt period of 20us
#define BATT_FLOAT 13.60            
#define HIGH_BAT_VOLTS 13.00        
#define LVD 11.5                    
#define OFF_NUM 9                   
#define LOW_SOL_WATTS 7.00          
#define MIN_SOL_WATTS 2.00          
#define MIN_BAT_VOLTS 11.00         
#define MAX_BAT_VOLTS 14.00         
  
//------------------------------------------------------------------------------------------------------
//Defining led pins for indication
#define LED_GREEN 0  
#define LED_YELLOW 1 
#define LED_RED 4    
//-----------------------------------------------------------------------------------------------------
// Defining load control pin
#define INVERTER_PIN 6       // pin-2 is used to control the load
  
//-----------------------------------------------------------------------------------------------------
// Defining lcd back light pin
#define LCD_LIGHT_PIN 5       // pin-5 is used to control the lcd back light

//------------------------------------------------------------------------------------------------------
//////////////BIT MAP ARRAY///////////////////
//-------------------------------------------------------------------------------------------------------

byte battery_icons[6][8]=
{{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
}};
#define SOLAR_ICON 6
byte solar_icon[8] = //icon for termometer
{
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b00000
};
#define PWM_ICON 7
byte _PWM_icon[8]=
{
  0b11101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10111,
};
byte backslash_char[8]= 
{
  0b10000,
  0b10000,
  0b01000,
  0b01000,
  0b00100,
  0b00100,
  0b00010,
  0b00010,
};
//-------------------------------------------------------------------------------------------------------

// global variables
float inv_watts;
float inv_volts ;
float inv_amps;                        //inv amps
float sol_amps;                       // solar amps 
float sol_volts;                      // solar volts 
float bat_volts;                      // battery volts 
float sol_watts;                      // solar watts
float old_sol_watts = 0;             
unsigned int seconds = 0;             // seconds from timer routine
unsigned int prev_seconds = 0;        // seconds value from previous pass
unsigned int interrupt_counter = 0;   // counter for 20us interrrupt
unsigned long time = 0;               
int delta = PWM_INC;                  
int pwm = 0;                          
int LCD_LIGHT_PIN_State = 0;         // variable for storing the state of the backlight button
boolean load_status = false;          // variable for storing the load output state 
  
enum charger_mode {off, on, bulk, bat_float} charger_state;    
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//------------------------------------------------------------------------------------------------------
// This routine is automatically called at powerup/reset
//------------------------------------------------------------------------------------------------------

File myFile; 


int chipSelect = 10; 
RTC_DS3231 rtc;

void setup()                           // run once, when the sketch starts
{
pinMode(chipSelect, OUTPUT);
 
 if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);  
  }
  Serial.println("card initialized.");
 
 
 
 /*// Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
 */
  pinMode(ENABLE_PWM_PIN, OUTPUT);     // sets the digital pin as output
  TURN_OFF_MOSFETS;                    // turn off MOSFET driver chip
  charger_state = off;                 // start with charger state as off
  lcd.begin(20,4);                     // initialize the lcd for 16 chars 2 lines, turn on backlight

  // create the LCD special characters. Characters 0-5 are the various battery fullness icons
  // icon 7 is for the PWM icon, and icon 8 is for the solar array
  lcd.backlight();
  for (int batchar = 0; batchar <   6; ++batchar) {
    lcd.createChar(batchar, battery_icons[batchar]);
  }
  lcd.createChar(PWM_ICON,_PWM_icon);
  lcd.createChar(SOLAR_ICON,solar_icon);
  lcd.createChar('\\', backslash_char);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  Timer1.initialize(20);               // initialize timer1, and set a 20uS period
  Timer1.pwm(PWM_PIN, 0);              // setup pwm on pin 9, 0% duty cycle
  Timer1.attachInterrupt(callback);    // attaches callback() as a timer overflow interrupt
  Serial.begin(9600);                  // open the serial port at 9600 bps:

  pwm = PWM_START;                     //starting value for pwm  
  pinMode(LCD_LIGHT_PIN, INPUT);
  pinMode(INVERTER_PIN,OUTPUT);
  digitalWrite(INVERTER_PIN,LOW);          // default load state is OFF
  digitalWrite(LCD_LIGHT_PIN,LOW);    //  default LCd back light is OFF

  // display the constant stuff on the LCD
  lcd.setCursor(0, 0);
  lcd.print("SOL");
  lcd.setCursor(4, 0);
  lcd.write(SOLAR_ICON);
  lcd.setCursor(8, 0);
  lcd.print("BAT");
}

//------------------------------------------------------------------------------------------------------
// Main loop
//------------------------------------------------------------------------------------------------------
void loop()
{
DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(", ");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(": ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();


    Serial.println();


  
// make a string for assembling the data to log:
  String dataString = "  ";
    delay(1000);

//======Solar voltage ===== 
 
 dataString += String (sol_volts);
 dataString += "'";

//===Solar current====
  dataString += String (sol_amps);
 dataString += "'";

//====Battery Voltage====
  dataString += String (bat_volts);
 dataString += "'";

//==== Inverter amps ====
dataString += String (inv_amps);
dataString += "'";
//====Inverter watts====
dataString += String (inv_watts);
dataString += "'";
 
 
 
//Serial.println("");
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);

dataFile.print(now.year(), DEC);
    dataFile.print('/');
   dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.day(), DEC);
    dataFile.print(", ");
    dataFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
    dataFile.print(": ");
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.print(now.second(), DEC);
    dataFile.print(' ');
   // dataFile.println();



    
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  read_input();                         
  run_charger();                       
  print_data();                        
  load_control();                      
  led_output();                       
  lcd_display();                       

}

//------------------------------------------------------------------------------------------------------
// This routine reads and averages the analog inputs for this system, solar volts, solar amps and 
// battery volts. 
//------------------------------------------------------------------------------------------------------
int read_adc(int channel){
  
  int sum = 0;
  int now;
  int i;
  
  for (i=0; i<AVG_NUM; i++) {          
    now = analogRead(channel);        
    sum += now;                       
    delayMicroseconds(50);             
  }
  return(sum / AVG_NUM);               // divide sum by AVG_NUM to get average and return it
}


void read_data(void) {
  
  sol_amps = (read_adc(SOL_AMPS_CHAN) * SOL_AMPS_SCALE -20.7);    //input of solar amps//-20.61 //-13.51)
  sol_volts = read_adc(SOL_VOLTS_CHAN) * SOL_VOLTS_SCALE;          //input of solar volts 
  bat_volts = read_adc(BAT_VOLTS_CHAN) * BAT_VOLTS_SCALE;          //input of battery volts 
  sol_watts = sol_amps * sol_volts ;                               //calculations of solar watts  
  inv_amps = (read_adc(INV_AMPS_CHAN)* INV_AMPS_SCALE -20.7); 
  inv_watts = inv_amps * inv_volts;               
}

// This is interrupt service routine for Timer1 that occurs every 20uS.
void callback()
{
  if (interrupt_counter++ > ONE_SECOND) {        // increment interrupt_counter until one second has passed
    interrupt_counter = 0;                       
    seconds++;                                   
  }
}

// This routine uses the Timer1.pwm function to set the pwm duty cycle.
void set_pwm_duty(void) {

  if (pwm > PWM_MAX) {             // check limits of PWM duty cyle and set to PWM_MAX
    pwm = PWM_MAX;    
  }
  else if (pwm < PWM_MIN) {          // if pwm is less than PWM_MIN then set it to PWM_MIN
    pwm = PWM_MIN;
  }
  if (pwm < PWM_MAX) {
    Timer1.pwm(PWM_PIN,(PWM_FULL * (long)pwm / 100), 20);  // use Timer1 routine to set pwm duty cycle at 20uS period
    //Timer1.pwm(PWM_PIN,(PWM_FULL * (long)pwm / 100));
  }                       
  else if (pwm == PWM_MAX) {           // if pwm set to 100% it will be on full but we have 
    Timer1.pwm(PWM_PIN,(PWM_FULL - 1), 20);                // keep switching so set duty cycle at 99.9% 
    //Timer1.pwm(PWM_PIN,(PWM_FULL - 1));              
  }                       
} 


void run_charger(void) {
  
  static int off_count = OFF_NUM;

  switch (charger_state) {
    case on:                                        
      if (sol_watts < MIN_SOL_WATTS) {                      
        charger_state = off;                                
        off_count = OFF_NUM;                                
        TURN_OFF_MOSFETS; 
      }
      else if (bat_volts > (BATT_FLOAT - 0.1)) {            
        charger_state = bat_float;                          
      }
      else if (sol_watts < LOW_SOL_WATTS) {                 
        pwm = PWM_MAX;                                      
        set_pwm_duty();                         
      }                                                     
      else {                                          
        pwm = ((bat_volts * 10) / (sol_volts / 10)) + 5;    
        charger_state = bulk;                               
      }
      break;
    case bulk:
      if (sol_watts < MIN_SOL_WATTS) {                      
        charger_state = off;                                
        off_count = OFF_NUM;                                
        TURN_OFF_MOSFETS; 
      }
      else if (bat_volts > BATT_FLOAT) {                 
        charger_state = bat_float;                          
      }
      else if (sol_watts < LOW_SOL_WATTS) {                 
        charger_state = on;                                 
        TURN_ON_MOSFETS;                                    
      }
      else {                                                //  Maximum Power Point algorithm
        if (old_sol_watts >= sol_watts) {                   // if previous watts are greater change the value of
          delta = -delta;                 // delta to make pwm increase or decrease to maximize watts
        }
        pwm += delta;                                       // add delta to change PWM duty cycle for PPT algorithm (compound addition)
        old_sol_watts = sol_watts;                          
        set_pwm_duty();                   // set pwm duty cycle to pwm value
      }
      break;
    case bat_float:

      if (sol_watts < MIN_SOL_WATTS) {                      
        charger_state = off;                                
        off_count = OFF_NUM;                                
        TURN_OFF_MOSFETS; 
        set_pwm_duty();         
      }
      else if (bat_volts > BATT_FLOAT) {                    
        TURN_OFF_MOSFETS;                                   
        pwm = PWM_MAX;                                      
        set_pwm_duty();                                     
      }
      else if (bat_volts < BATT_FLOAT) {                    
        pwm = PWM_MAX;                                      
        set_pwm_duty();                                     
        TURN_ON_MOSFETS;    
        if (bat_volts < (BATT_FLOAT - 0.1)) {               
        charger_state = bulk;                               
        }
      }
      break;
    case off:                                               
      TURN_OFF_MOSFETS;
      if (off_count > 0) {                                  
        off_count--;                                        
      }                                                     
      else if ((bat_volts > BATT_FLOAT) && (sol_volts > bat_volts)) {
        charger_state = bat_float;                          
        TURN_ON_MOSFETS;
      }    
      else if ((bat_volts > MIN_BAT_VOLTS) && (bat_volts < BATT_FLOAT) && (sol_volts > bat_volts)) {
        charger_state = bulk;
        TURN_ON_MOSFETS;
      }
      break;
    default:
      TURN_OFF_MOSFETS; 
      break;
  }
}

//-----------------LOAD CONTROL///////////////// 

void load_control(){
#if LOAD_ALGORITHM == 0
  // turn on inverter at night when battery voltage is above LVD
  load_on(sol_watts < MIN_SOL_WATTS && bat_volts > LVD);
#else
  // drains excess solar energy 
  load_on(sol_watts > MIN_SOL_WATTS && bat_volts > BATT_FLOAT);
#endif
}

void load_on(boolean new_status) {
  if (load_status != new_status) {
    load_status = new_status;
    digitalWrite(INVERTER_PIN, new_status ? HIGH : LOW);
  }
}

// This routine prints all the data out to the serial port.

void print_data(void) {
  
  Serial.print(seconds,DEC);
  Serial.print("      ");

  Serial.print("Charging = ");
  if (charger_state == on) Serial.print("on   ");
  else if (charger_state == off) Serial.print("off  ");
  else if (charger_state == bulk) Serial.print("bulk ");
  else if (charger_state == bat_float) Serial.print("float");
  Serial.print("      ");

  Serial.print("pwm = ");
  if(charger_state == off)
  Serial.print(0,DEC);
  else
  Serial.print(pwm,DEC);
  Serial.print("      ");

  Serial.print("Current (panel) = ");
  Serial.print(sol_amps);
  Serial.print("      ");

  Serial.print("Voltage (panel) = ");
  Serial.print(sol_volts);
  Serial.print("      ");

  Serial.print("Power (panel) = ");
  Serial.print(sol_volts);
  Serial.print("      ");

  Serial.print("Battery Voltage = ");
  Serial.print(bat_volts);
  Serial.print("      ");

  Serial.print("\n\r");
  //delay(1000);
}

//---------------------------------Led Indication--------------------------------------------------
void light_led(char pin)
{
  static char last_lit;
  if (last_lit == pin)
      return;
  if (last_lit != 0)
      digitalWrite(last_lit, LOW);
  digitalWrite(pin, HIGH);
  last_lit = pin;
}
// display the current state via LED as follows:
// YELLOW (over 14.1 volts)
// RED  (under 11.9 volts)
// GREEN is between 11.9 and 14.1 volts
void led_output(void)
{
  static char last_lit;
  if(bat_volts > 14.1 ) // 
      light_led(LED_YELLOW);
  else if(bat_volts > 11.9 && bat_volts < 14.0)// added && bat_volts < 14.0
      light_led(LED_GREEN);
  else
      light_led(LED_RED);
}

//-------------------------- LCD DISPLAY --------------------------------------------------------------
void lcd_display()
{
  static bool current_backlight_state = -1;
  LCD_LIGHT_PIN_State = digitalRead(LCD_LIGHT_PIN);
  if (current_backlight_state != LCD_LIGHT_PIN_State) {
    current_backlight_state = LCD_LIGHT_PIN_State;
    if (LCD_LIGHT_PIN_State == HIGH)
      lcd.backlight();
    else
      lcd.noBacklight();
  }

  if (LCD_LIGHT_PIN_State == HIGH)
  {
    time = millis();                        // If any of the buttons are pressed, save the time in millis to "time"
  }
 
 lcd.setCursor(0, 1);
 lcd.print(sol_volts);
 lcd.print("V ");
 lcd.setCursor(0, 2);
 lcd.print(sol_amps);
 lcd.print("A ");  
 lcd.setCursor(0, 3);
 lcd.print(sol_watts);
 lcd.print("W "); 
 lcd.setCursor(8, 1);
 lcd.print(bat_volts);
 lcd.setCursor(8,2);

 if (charger_state == on) 
 lcd.print("on   ");
 else if (charger_state == off)
 lcd.print("off  ");
 else if (charger_state == bulk)
 lcd.print("bulk ");
 else if (charger_state == bat_float)
 {
 lcd.print("     ");
 lcd.setCursor(8,2);
 lcd.print("float");
 }
 
//--------------------Battery State Of Charge ---------------
int pct = 100.0*(bat_volts - 11.3)/(12.7 - 11.3);
 if (pct < 0)
     pct = 0;
 else if (pct > 100)
     pct = 100;

 lcd.setCursor(12,0);
 lcd.print((char)(pct*5/100));

 lcd.setCursor(8,3);
 pct = pct - (pct%10);
 lcd.print(pct);
 lcd.print("%  ");
 
//------------------Duty Cycle-----------------------------------------
lcd.setCursor(15,0);
 lcd.print("PWM");
 lcd.setCursor(19,0);
 lcd.write(PWM_ICON);
 lcd.setCursor(15,1);
 lcd.print("   ");
 lcd.setCursor(15,1);
 if( charger_state == off)
 lcd.print(0);
 else
 lcd.print(pwm); 
 lcd.print("% ");
//------------------------Load Status-----------------------------------
lcd.setCursor(15,2);
 lcd.print("INVTR");///Load
 lcd.setCursor(15,3);
 if (load_status)
 {
    lcd.print("On  ");
 }
 else
 {
   lcd.print("Off ");
 } 
 spinner();
 backLight_timer();                      // call the backlight timer function in every loop 
}

void backLight_timer(){
  if((millis() - time) <= 600000)         
      lcd.backlight();                   
  else 
      lcd.noBacklight();                 
}
void spinner(void) {
  static int cspinner;
  static char spinner_chars[] = { '*','*', '*', ' ', ' '};
  cspinner++;
  lcd.print(spinner_chars[cspinner%sizeof(spinner_chars)]);
}
