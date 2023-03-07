#include "msg_ESP.h"


void updateCrc(msg_ESP * msg)
{
    void iteration_crc(unsigned short ch, unsigned short* crc);
    void augment_message_crc(unsigned short* crc);
    
    unsigned short crc = 0xffff;

    for(unsigned short i = 0; i< sizeof(msg_ESP) -2; i ++)
    {
        iteration_crc(*((unsigned char * )msg+i),&crc);
    }
    augment_message_crc(&crc);
    msg->crc = crc;
}

void iteration_crc(unsigned short ch, unsigned short* crc)
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

void augment_message_crc(unsigned short* crc)
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