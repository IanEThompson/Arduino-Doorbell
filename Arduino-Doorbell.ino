/* Doorbell for Arduino V1.0
 *  ========================
 *  Ian Thompson 
 *  ianmelandkids@gmail.com
 *  0414270210
 *  
 *  Each button on a 12 button remote control plays a different song from the MP3 player's SD card
 *  MP3 files must be in a directory called "01" eg:
 *    \01\001xxx.mp3
 *    \01\002xxx.mp3
 *    \01\003xxx.mp3
 *    \01\004xxx.mp3
 *    \01\005xxx.mp3 etc
 *  
 *  Hardware:
 *  Arduino (tested on a nano, but should work on any Arduino if pinouts are ok 
 *  433MHz doorbell transmitter with 12 buttons https://www.aliexpress.com/item/Universal-433MHz-315MHz-12CH-DC12-V-RF-remote-control-learning-code-EV1527-Long-distance-rf-remote/32672825348.html?spm=a2g0s.9042311.0.0.104c4c4dBrUIxc
 *    
 *  433MHz reciever https://www.aliexpress.com/item/433-92Mhz-Superheterodyne-RF-Receiver-and-Transmitter-Module-ASK-low-power-kits-FOR-Arduino-ARM-MCU/32737335032.html?spm=a2g0s.9042311.0.0.27424c4d9kRUih
 *    connected to pin 2
 *    
 *  MP3 serial module http://www.da-share.com/misc/catalex-mp3-board-yx5300-chip/
 *    connected to pins 5 and 6 (configuable below)
 *  
 *  If the transmitter is replaced, run this program and use serial monitor to read out the codes
 *  and put the new transmitter's codes into the if statements below/
 *  
 *  NOTE - helpful status info is available through the serial monitor if the program is run while
 *  connected to a PC with the arduino IDE installed.
 *    
 */

#include <RCSwitch.h>         //required for RF reciever
#include <SoftwareSerial.h>   //required for MP3 player

//construct the RF receiver
RCSwitch doorSwitch = RCSwitch();

//define serial commands for the MP3 player
#define CMD_PLAY_W_INDEX 0X03
#define CMD_INC_VOLUME 0X04
#define CMD_DEC_VOLUME 0X05
#define CMD_SET_VOLUME 0X06
#define CMD_SEL_DEV 0X09
  #define DEV_TF 0X02
#define CMD_PLAY 0X0D
#define CMD_PAUSE 0X0E
#define CMD_STOP 0X16
#define CMD_SINGLE_CYCLE 0X19
  #define SINGLE_CYCLE_ON 0X00
  #define SINGLE_CYCLE_OFF 0X01
#define CMD_PLAY_W_VOL 0X22

//MP3 player pins
#define ARDUINO_RX 5//should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 6//connect to RX of the module

//intitialise mp3 player and create buffer for sending serial data
SoftwareSerial mp3Serial(ARDUINO_RX, ARDUINO_TX);
static int8_t Send_buf[8] = {0};

//define the play time for each song (in millisecs) and the volume (0-30)
const int playTime = 10000;
const int volume = 15;

//global variables to time song duration
bool mp3Playing;
unsigned long startTime = 0;

void setup() {
  //for sending status info to serial monitor (optional)
  Serial.begin(9600);
  
  // Receiver on interrupt 0 => that is pin #2
  doorSwitch.enableReceive(0);

  //initialise MP3
  mp3Serial.begin(9600);
  delay(500);
  Mp3Command(CMD_SEL_DEV, DEV_TF);//select the TF card
  Mp3Command(CMD_SET_VOLUME, volume);//set the volume
  delay(200);//wait for 200ms
}

void loop() {
  
  //has a button been pressed?
  if (doorSwitch.available()) {

    //read the button press info
    long receivedValue=doorSwitch.getReceivedValue(); 
    
    //debugging info for serial monitor
    Serial.print("Received ");
    Serial.print( receivedValue );
    Serial.print(" / ");
    Serial.print( doorSwitch.getReceivedBitlength() );
    Serial.print("bit ");
    Serial.print("Protocol: ");
    Serial.println( doorSwitch.getReceivedProtocol() );

    //play the appropriate MP3 according to the code received
    //change these codes if the transmitter is replaced!
    //user serial monitor to find the new codes
    if (receivedValue == 7235464){
      mp3Play(0x0101);
    }
    if (receivedValue == 7235460){
      mp3Play(0x0102);
    }
    if (receivedValue == 7235468){
      mp3Play(0x0103);
    }
    if (receivedValue == 7235458){
      mp3Play(0x0104);
    }
    if (receivedValue == 7235466){
      mp3Play(0x0105);
    }
    if (receivedValue == 7235462){
      mp3Play(0x0106);
    }
    if (receivedValue == 7235470){
      mp3Play(0x0107);
    }
    if (receivedValue == 7235457){
      mp3Play(0x0108);
    }
    if (receivedValue == 7235465){
      mp3Play(0x0109);
    }
    if (receivedValue == 7235461){
      mp3Play(0x010A);
    }
    if (receivedValue == 7235469){
      mp3Play(0x010B);
    }
    if (receivedValue == 7235459){
      fadeOut();
    }
    
    //reset the code and clear the rf input buffer
    receivedValue = 0;
    doorSwitch.resetAvailable();
  }

  //if a song has been playing more that "playTime", it's time stop
  if (mp3Playing && millis()-startTime > playTime){
    fadeOut();
  }
}

//Tells the MP3 player to play a particular file
void mp3Play(int trackNumber){
  startTime=millis();                   //note the start time
  Mp3Command(CMD_SET_VOLUME, 0x000F);   //set the volume
  Mp3Command(0x0F,trackNumber);         //start the track
  mp3Playing=true;                      //flag that a track is playing
}


//sends a command and optional data to the mp3 player
void Mp3Command(int8_t command, int16_t dat)
{
  delay(20);
  Send_buf[0] = 0x7e; //starting byte
  Send_buf[1] = 0xff; //version
  Send_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  Send_buf[3] = command; //
  Send_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
  Send_buf[5] = (int8_t)(dat >> 8);//datah
  Send_buf[6] = (int8_t)(dat); //datal
  Send_buf[7] = 0xef; //ending byte
  for(uint8_t i=0; i<8; i++)//
  {
    mp3Serial.write(Send_buf[i]) ;
  }
}

//fadeout the current song
void fadeOut(){
  Serial.print("Fading...");
  for (int i=volume; i>0; i--){
    Serial.print("*");
    Mp3Command(CMD_DEC_VOLUME, 0x0000);    //decrement volume
    delay(2000/volume);                    //over 2 seconds
  }
  Serial.println("");
  Mp3Command(CMD_STOP, 0x0000);           //stop the track completely
  mp3Playing=false;
}
