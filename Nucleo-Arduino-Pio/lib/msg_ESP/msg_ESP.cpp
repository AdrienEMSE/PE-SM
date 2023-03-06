#include "msg_ESP.h"

void updateCrc(msg_ESP * msg)
{
    unsigned short crc = 0xffff;
    for(int i = 0; i< sizeof(msg_ESP) -2; i ++)
    {
        iteration_crc((unsigned short)msg+i,&crc);
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

    for (i=0; i<8; i++)
    {
        if (*crc & 0x8000)
        {
            xor_flag= 1;
        }
        else
        {
            xor_flag= 0;
        }
        *crc = *crc << 1;

        if (ch & v)
        {
            /*
            Append next bit of message to end of CRC if it is not zero.
            The zero bit placed there by the shift above need not be
            changed if the next bit of the message is zero.
            */
            *crc= *crc + 1;
        }

        if (xor_flag)
        {
            *crc = *crc ^ poly;
        }

        /*
        Align test bit with next bit of the message byte.
        */
        v = v >> 1;
    }
} 

void augment_message_crc(unsigned short* crc)
{
    unsigned short i, xor_flag;

    for (i=0; i<16; i++)
    {
        if (*crc & 0x8000)
        {
            xor_flag= 1;
        }
        else
        {
            xor_flag= 0;
        }
        *crc = *crc << 1;

        if (xor_flag)
        {
            *crc = *crc ^ poly;
        }
    }
} 