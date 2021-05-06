#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>

//This demo can only work on OV2640_MINI_2MP_PLUS platform.
#define BMPIMAGEOFFSET 66
#define OV2640_CHIPID_HIGH 	0x0A
#define OV2640_CHIPID_LOW 	0x0B
// set pin 7 as the slave select for the digital pot:
const int CS = 7;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;
ArduCAM myCAM( OV2640, CS );
uint8_t read_fifo_burst(ArduCAM myCAM);
void OV2640CameraInit(void) //OV2640CameraInit
{
	// put your setup code here, to run once:
	uint8_t vid, pid;
	uint8_t temp;
  myCAM.InitConfig();
	Wire.begin();
	Serial.begin(115200);
	Serial.println(F("ACK CMD ArduCAM Start! END"));
	// set the CS as an output:
	pinMode(CS, OUTPUT);
	digitalWrite(CS, HIGH);
	// initialize SPI:
	SPI.begin();
	//Reset the CPLD
	myCAM.write_reg(0x07, 0x80);
	delay(100);
	myCAM.write_reg(0x07, 0x00);
	delay(100);
	while(1)
	{
	  //Check if the ArduCAM SPI bus is OK
	  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
	  temp = myCAM.read_reg(ARDUCHIP_TEST1);
	  if (temp != 0x55)
	  {
		Serial.println(F("ACK CMD SPI interface Error! END"));
		delay(1000);continue;
	  }
	  else
	  {
		Serial.println(F("ACK CMD SPI interface OK. END"));break;
	  }
	}

  while(1)
  {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
	{
      Serial.println(F("ACK CMD Can't find OV2640 module! END"));
      delay(1000);continue;
    }
    else
	{
      Serial.println(F("ACK CMD OV2640 detected. END"));break;
    } 
  }

	//Change to JPEG capture mode and initialize the OV5642 module
	myCAM.set_format(JPEG);
	myCAM.InitCAM();
	 myCAM.OV2640_set_JPEG_size(OV2640_320x240);
	delay(1000);
	myCAM.clear_fifo_flag();
	#if !(defined (OV2640_MINI_2MP_PLUS))
	myCAM.write_reg(ARDUCHIP_FRAMES,0x00);
	#endif
}

void CameraPhotograph(void)
{
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    //Start capture
    myCAM.start_capture();
    start_capture = 0; 
	while(!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      Serial.println(F("ACK CMD CAM Capture Done. END"));delay(50);
      read_fifo_burst(myCAM);
      //Clear the capture done flag
      myCAM.clear_fifo_flag();
    }
}


void loop() {
// put your main code here, to run repeatedly:
uint8_t temp = 0xff, temp_last = 0;
bool is_header = false;
if (Serial.available())
{
  temp = Serial.read();
  switch (temp)
  {
    case 0x10:
    mode = 1;
    temp = 0xff;
    start_capture = 1;
    Serial.println(F("ACK CMD CAM start single shoot. END"));
    break;
    case 0x11: 
    temp = 0xff;
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    #if !(defined (OV2640_MINI_2MP_PLUS))
    myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    #endif
    break;
    case 0x20:
    mode = 2;
    temp = 0xff;
    start_capture = 2;
    Serial.println(F("ACK CMD CAM start video streaming. END"));
    break;
    case 0x31:
    temp = 0xff;
    myCAM.set_format(BMP);
    myCAM.InitCAM();
    #if !(defined (OV2640_MINI_2MP_PLUS))        
    myCAM.clear_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    #endif
    myCAM.wrSensorReg16_8(0x3818, 0x81);
    myCAM.wrSensorReg16_8(0x3621, 0xA7);
    break;  
  }
}
if (mode == 1)
{
  if (start_capture == 1)
  {
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    //Start capture
    myCAM.start_capture();
    start_capture = 0;
  }
  while(!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    Serial.println(F("ACK CMD CAM Capture Done. END"));delay(50);
    read_fifo_burst(myCAM);
    //Clear the capture done flag
    myCAM.clear_fifo_flag();
  }
}
else if (mode == 2)
{
  while (1)
  {
    temp = Serial.read();
    if (temp == 0x21)
    {
      start_capture = 0;
      mode = 0;
      Serial.println(F("ACK CMD CAM stop video streaming. END"));
      break;
    }
    if (start_capture == 2)
    {
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      //Start capture
      myCAM.start_capture();
      start_capture = 0;
    }
    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      uint32_t length = 0;
      length = myCAM.read_fifo_length();
      if ((length >= MAX_FIFO_SIZE) | (length == 0))
      {
        myCAM.clear_fifo_flag();
        start_capture = 2;
        continue;
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();//Set fifo burst mode
      temp =  SPI.transfer(0x00);
      length --;
      while ( length-- )
      {
        temp_last = temp;
        temp =  SPI.transfer(0x00);
        if (is_header == true)
        {
          Serial.write(temp);
        }
        else if ((temp == 0xD8) & (temp_last == 0xFF))
        {
          is_header = true;
          Serial.println(F("ACK IMG END"));
          Serial.write(temp_last);
          Serial.write(temp);
        }
        if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
        break;
        delayMicroseconds(15);
      }
      myCAM.CS_HIGH();
      myCAM.clear_fifo_flag();
      start_capture = 2;
      is_header = false;
    }
  }
}
}
uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  length = myCAM.read_fifo_length();
  if (length >= MAX_FIFO_SIZE) //512 kb
  {
    Serial.println(F("ACK CMD Over size. END"));
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("ACK CMD Size is 0. END"));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  temp =  SPI.transfer(0x00);
  length --;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    if (is_header == true)
    {
      Serial.write(temp);
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      Serial.println(F("ACK IMG END"));
      Serial.write(temp_last);
      Serial.write(temp);
    }
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    break;
    delayMicroseconds(15);
  }
  myCAM.CS_HIGH();
  is_header = false;
  return 1;
}