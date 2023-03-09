#include "msg_ESP.h"

msg_ESP_class::msg_ESP_class(HardwareSerial* Serial_comm) : Serial_ESP(Serial_comm)
{
    
}

msg_ESP_class::~msg_ESP_class()
{
}

void msg_ESP_class::flush()
{
  if(Serial_ESP->available())
  {
    while(Serial_ESP->available())
    {
      Serial_ESP->read();
    }
  }
}


void msg_ESP_class::send_msg_sensor()
{
  safePrintSerialln("sending msg_ESP to esp...");
  typeToSend = msg_type::sensor_msg;
  Serial_ESP->write((uint8_t*)&typeToSend,sizeof(typeToSend));
  Serial_ESP->write((uint8_t *)&_msg_sensor, sizeof(msg_ESP));
  safePrintSerialln("...msg_sent");
}

void msg_ESP_class::send_msg_location()
{
  safePrintSerialln("sending msg_ESP to esp...");
  typeToSend = msg_type::gps_msg;
  Serial_ESP->write((uint8_t*)&typeToSend,sizeof(typeToSend));
  Serial_ESP->write((uint8_t *)&_msg_location, sizeof(location_msg));
  safePrintSerialln("...msg_sent");
}

bool msg_ESP_class::safeSendX1(msg_type type)
{
  uint32_t timer = millis();
  bool timeout = false;
  flush();
  while (Serial_ESP->available() == 0)
  {
    if(millis() > timer +TIMEOUT_MS)
    {

      safePrintSerialln("Timeout");
      return false;
    }
  }     //wait for data available

  String teststr = Serial_ESP->readStringUntil('\n');  //read until timeout
  teststr.trim();                        // remove any \r \n whitespace at the end of the String
  if ( teststr.startsWith("R") || teststr.endsWith("R"))
  {
    safePrintSerialln("Received Ready");
    flush();
    if(type == msg_type::gps_msg)
    {
      send_msg_location();
    }
    else
    {
      send_msg_sensor();
    }
    timer = millis();
    while (Serial_ESP->available() == 0)
    {
      if(millis() > timer +TIMEOUT_MS)
      {
      safePrintSerialln("Timeout");
      return false;
        
      }
    }     //wait for data available

    String teststr2 = Serial_ESP->readStringUntil('\n');  //read until timeout
    teststr2.trim();  
    if (teststr2.startsWith("K") || teststr2.endsWith("K")) 
    {
      safePrintSerialln("Received OK");
      return true;
    }
    else
    {
      safePrintSerialln("No Received  OK:");
      safePrintSerialln(teststr2);
      return false;
    }
  }
  else 
  {
    safePrintSerialln("No Received  Ready :");
    safePrintSerialln(teststr);
    return false;
  } 
    
}

bool msg_ESP_class::safeSendXN(msg_type type,int n)
{
    for(int i = 0; i<n; i++)
    {
        if(safeSendX1(type))
        {
            return true;
        }
    }
    return false;

}

uint16_t msg_ESP_class::updateCrc_sensor()
{
    msg_ESP* msg = &_msg_sensor;
    
    unsigned short crc = 0xffff;

    for(unsigned short i = 0; i< sizeof(msg_ESP) -2; i ++)
    {
        iteration_crc(*((unsigned char * )msg+i),&crc);
    }
    augment_message_crc(&crc);
    msg->crc = crc;
    return crc;
}

uint16_t msg_ESP_class::updateCrc_gps()
{
    location_msg* msg = &_msg_location;
    
    unsigned short crc = 0xffff;

    for(unsigned short i = 0; i< sizeof(location_msg) -2; i ++)
    {
        iteration_crc(*((unsigned char * )msg+i),&crc);
    }
    augment_message_crc(&crc);
    msg->crc = crc;
    return crc;
}

void msg_ESP_class::iteration_crc(unsigned short ch, unsigned short* crc)
{
    unsigned short i, v, xor_flag;
    
    /*
    Align test bit with leftmost bit of the message byte.
    */
    v = 0x80;
    unsigned short good_crc = *crc;

    for (i=0; i<8; i++)
    {
        if (good_crc & 0x8000)
        {
            xor_flag= 1;
        }
        else
        {
            xor_flag= 0;
        }
        good_crc = good_crc << 1;

        if (ch & v)
        {
            /*
            Append next bit of message to end of CRC if it is not zero.
            The zero bit placed there by the shift above need not be
            changed if the next bit of the message is zero.
            */
            good_crc= good_crc + 1;
        }

        if (xor_flag)
        {
            good_crc = good_crc ^ poly;
        }

        /*
        Align test bit with next bit of the message byte.
        */
        v = v >> 1;
    }
    *crc = good_crc;
} 

void msg_ESP_class::augment_message_crc(unsigned short* crc)
{
     unsigned short i, xor_flag;
     unsigned short good_crc = *crc;

    for (i=0; i<16; i++)
    {
        if (good_crc & 0x8000)
        {
            xor_flag= 1;
        }
        else
        {
            xor_flag= 0;
        }
        good_crc = good_crc << 1;

        if (xor_flag)
        {
            good_crc = good_crc ^ poly;
        }
    }
    *crc = good_crc;
} 

bool msg_ESP_class::iscrcOk(msg_type type)
{

    if(type == msg_type::sensor_msg)
    {
      msg_ESP* msg = &_msg_sensor;
      uint16_t crcToCheck = msg->crc;
      updateCrc_sensor();
      if(msg->crc == crcToCheck)
      {
          return true;
      }
      else
      {
          msg->crc = crcToCheck;
          return false;
      }
    }
    else if (type == msg_type::gps_msg)
    {
      location_msg *msg = &_msg_location;
      uint16_t crcToCheck = msg->crc;
      updateCrc_gps();
      if(msg->crc == crcToCheck)
      {
          return true;
      }
      else
      {
          msg->crc = crcToCheck;
          return false;
      }
    }
    return false;

}

