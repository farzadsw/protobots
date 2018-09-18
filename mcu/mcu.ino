#include <Arduino_FreeRTOS.h>

// INCLUDE MASSAGE
#include <AsciiMassagePacker.h>
#include <AsciiMassageParser.h>
// PACKER(FOR SENDING) AND PARSER(FOR RECEIVING) INSTANCES.
AsciiMassageParser inbound;
AsciiMassagePacker outbound;


#define CMD                 (byte)0x00              //  MD25 command byte of 0

#define WRITESP1            0x31                    // Byte writes speed to motor 1
#define WRITESP2            0x32                    // Byte writes speed to motor 2
#define WRITEACCEL          0x33                    // Byte writes a value of acceleration
#define RESETREG            0x35                    // Byte resets encoders
#define SETMODE             0x34                    // Byte sets mode
#define READIVS             0x2C                    // Byte reads motor currents and battery voltage        
#define READENCS            0x25                    // Byte reads both encoders
#define GET_VER             0x29
#define ENABLE_TIMEOUT      0x39

#define md25 Serial3

long encValue = 0;
byte softwareRev = 0;

int velo = 0;
int steer = 0;
int counter = 0;
#define TIMEOUT       10  // time out for the command
#define DEG_TO_RAD    0.0174533
#define WHEEL_R    5.0 //cm radius
#define WHEEL_BASE    20.0 //distance between wheels

// define two tasks for Blink & AnalogRead
void TaskComm( void *pvParameters );
void TaskDrive( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  
  md25.begin(38400);  //motor
  md25.write(CMD);                                            // Set MD25 accelleration value
  md25.write(WRITEACCEL);
  md25.write(10);
  delayMicroseconds(10);                                        // Wait for this to be processed
  md25.write(CMD);                                            // Reset the encoder registers to 0
  md25.write(RESETREG);         
  md25.write(CMD);                                            // Set mode to 2, Both motors controlled by writing to speed 1
  md25.write(SETMODE);
  md25.write(3);    
  md25.write(CMD);
  md25.write(ENABLE_TIMEOUT);    
  
  
  md25.write(CMD);                                            // Get software version of MD25
  md25.write(GET_VER);
  Serial.println("Checking connection ...");
  while(md25.available() < 1)
  {
    Serial.println("waiting for driver connection");
    delay(1000);  
  }                               // Wait for byte to become available         
  softwareRev = md25.read();  


  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskComm
    ,  (const portCHAR *)"Comminucation"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskDrive
    ,  (const portCHAR *) "MotorDrive"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
  delay(1500);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskComm(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    if ( inbound.parseStream( &Serial ) ) {
      if ( inbound.fullMatch("move") ) {
        velo = inbound.nextInt();
        steer = inbound.nextInt();
        counter = TIMEOUT;
        Serial.print("velocity: "); Serial.print(velo); Serial.print(" ,steer: ");Serial.println(steer);
      } else if (inbound.fullMatch("reset") ){  //reset encoders
         md25.write(CMD);                                            // Reset the encoder registers to 0
         md25.write(RESETREG);         
      }
      else {
        outbound.streamEmpty(&Serial, "what?");
      }
    }else{
      if(counter > 0) counter -=1;  //count down for the drive timeout  
    }
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 0.1 second
  }
}



void TaskDrive(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    //readVolts();
    readEncoder();
    if(counter > 0)
    {
      moveit(velo,steer);
    }else
    {
      moveit(0,0);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}




//#####################################

long readEncoder(){                        // Function to read and display the value of both encoders, returns value of first encoder
  long result1 = 0; 
  long result2 = 0;
  float dist = 0;
  float theta = 0;
  md25.write(CMD);
  md25.write(READENCS);
  while(md25.available() < 8){}          // Wait for 8 bytes, first 4 encoder 1 values second 4 encoder 2 values 
  result1 = md25.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += md25.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += md25.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += md25.read();               // Fourth byte for encoder 1, LL
  result2 = md25.read();
  result2 <<= 8;
  result2 += md25.read();
  result2 <<= 8;
  result2 += md25.read();
  result2 <<= 8;
  result2 += md25.read();

  dist = (result1 + result2)* DEG_TO_RAD * WHEEL_R / 2.0;
  theta = (result1 - result2) * WHEEL_R / WHEEL_BASE;  // degree

  Serial.print("Encoder 1:");               // Displays data to the LCD03 screen
  Serial.print(result1,DEC);
  Serial.print("Encoder 2:");
  Serial.print(result2,DEC);
  Serial.print(" Distance:");
  Serial.print(dist);
  Serial.print(" Theta:");
  Serial.print(theta);
  Serial.print("\n\r");
  return result1;                                   
}
  
void readVolts(){                                                 // Function reads current for both motors and battery voltage
  byte batteryVolts, mot1_current, mot2_current = 0;
  md25.write(CMD);
  md25.write(READIVS);                                          // Send byte to readbattery voltage and motor currents
  while(md25.available() < 3){}                                 // Wait for the 3 bytes to become available then get them
  batteryVolts = md25.read();
  mot1_current = md25.read();
  mot2_current = md25.read();

  Serial.print("Mot1 I:");
  Serial.print(mot1_current,DEC);
  Serial.print(" Mot2 I:");
  Serial.print(mot2_current,DEC);
  Serial.print(" "); 

  Serial.print("Rev:");
  Serial.print(softwareRev, DEC);
  Serial.print(" ");
  Serial.print("Volts:");
  Serial.print(batteryVolts/10,DEC);                               // Seperate whole number and descimal place of battery voltage and display
  Serial.print(".");  
  Serial.print(batteryVolts%10,DEC);
  Serial.println(" ");   
}

void moveit (byte v, byte s)
{
      md25.write(CMD);                  // Set motors to drive forward at full speed
      md25.write(WRITESP1);
      md25.write(v);
      md25.write(CMD);                  // Set motors to drive forward at full speed
      md25.write(WRITESP2);
      md25.write(s);
}

