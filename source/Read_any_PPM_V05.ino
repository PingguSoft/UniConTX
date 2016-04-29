/*This program puts the servo values into an array,
 reagrdless of channel number, polarity, ppm frame length, etc...
 You can even change these while scanning!*/

#define PPM_Pin 3  //this must be 2 or 3
#define multiplier (F_CPU/8000000)  //leave this alone
int ppm[16];  //array for storing up to 16 servo signals
byte servo[] = {1,4,5,6,7,8,9,10};  //pin number of servo output
#define servoOut  //comment this if you don't want servo output
//#define DEBUG

void setup()
{
  Serial.begin(115200);
  Serial.println("ready");

  #if defined(servoOut)
  for(byte i=0; sizeof(servo)-1; i++) pinMode(servo[i], OUTPUT);
  #endif
 
  pinMode(PPM_Pin, INPUT);
  attachInterrupt(PPM_Pin - 2, read_ppm, CHANGE);

  TCCR1A = 0;  //reset timer1
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us
}

void loop()
{
  //You can delete everithing inside loop() and put your own code here
/*  int count;

  while(ppm[count] != 0){  //print out the servo values
    Serial.print(ppm[count]);
    Serial.print("  ");
    count++;
  }
  Serial.println("");*/
  delay(100);  //you can even use delays!!!
}



void read_ppm(){  //leave this alone
  static unsigned int pulse;
  static unsigned long counter;
  static byte channel;
  static unsigned long last_micros;

  counter = TCNT1;
  TCNT1 = 0;

  if(counter < 710*multiplier){  //must be a pulse if less than 710us
    pulse = counter;
    #if defined(servoOut)
    if(sizeof(servo) > channel) digitalWrite(servo[channel], HIGH);
    if(sizeof(servo) >= channel && channel != 0) digitalWrite(servo[channel-1], LOW);
    #endif
  }
  else if(counter > 1910*multiplier){  //sync pulses over 1910us
    channel = 0;
    #if defined(DEBUG)
    Serial.print("PPM Frame Len: ");
    Serial.println(micros() - last_micros);
    last_micros = micros();
    #endif
  }
  else{  //servo values between 710us and 2420us will end up here
    ppm[channel] = (counter + pulse)/multiplier;
    #if defined(DEBUG)
    Serial.print(ppm[channel]);
    Serial.print("  ");
    #endif
    
    channel++;
  }
}

