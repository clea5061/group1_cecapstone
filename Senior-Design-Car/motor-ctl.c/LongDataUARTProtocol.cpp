#include "LongDataUARTProtocol.h"

void init_ldprotocol(struct LDProtocol* proto_handle)
{
    unsigned char* init_handle = (unsigned char*)proto_handle;

    for (unsigned int index = 0; index < sizeof(struct LDProtocol); index++)
    {
        init_handle[index] = '\0';
    }

    //pinMode(11, OUTPUT);
    //digitalWrite(11, HIGH);
}

void register_mailbox(char which, void* mailbox, char size, struct LDProtocol* proto_handle)
{
    if (size > MAX_MESG_SIZE || which > 7)
    {
        return;
    }

    proto_handle->mailbox_sizes[which] = size;
    proto_handle->mailboxes[which] = (unsigned char*)mailbox;
}

char bytes_to_packets(char bytes)
{
    int bits = bytes * 8;

    if (bits % 7)
    {
        return (char)(bits / 7 + 1);
    }

    return (char)(bits / 7);
}

void process_mesg(struct LDProtocol* proto_handle)
{
    char mailbox_addr = (proto_handle->mesg_buffer[0] & 0x70) >> 4;
    unsigned char byte_buffer[MAX_MESG_SIZE];
    for (char index = 0; index < MAX_MESG_SIZE; index++) { byte_buffer[index] = '\0'; }
    unsigned char checksum = '\0';

    for (char index = 0; index < proto_handle->packets_total; index++)
    {
        unsigned char packet = proto_handle->mesg_buffer[index + 1];
        checksum ^= packet;

        unsigned char new_bits_mask = 0x7F >> (index % 7);
        unsigned char remainder_mask = 0x7F & (0xFF << (7 - index % 7));

        unsigned char new_bits = (packet & new_bits_mask) << (1 + index % 7);
        unsigned char remainder_bits = (packet & remainder_mask) >> (7 - index % 7);

        if (remainder_mask && (index != 0))
        {
            byte_buffer[index - 1 - index / 7] |= remainder_bits;

            if (index < proto_handle->packets_total)
            {
                byte_buffer[index - index / 7] |= new_bits;
            }
        }
        else
        {
            byte_buffer[index - index / 7] |= new_bits;
        }
    }

    if (checksum == proto_handle->mesg_buffer[proto_handle->packets_total + 1])
    {
        unsigned char* mailbox = proto_handle->mailboxes[mailbox_addr];
        char mailbox_size = proto_handle->mailbox_sizes[mailbox_addr];

        for (char index = 0; index < mailbox_size; index++)
        {
            mailbox[index] = byte_buffer[index];
        }
    }
}

void recieve_message(struct LDProtocol* proto_handle)
{
    unsigned char packet = (unsigned char)Serial.read();

    switch (proto_handle->state)
    {
    case GET_HEADER:
        if (packet & 0x80)
        {
            proto_handle->packets_total = bytes_to_packets((packet & 0x0F) + 1);

            if (proto_handle->packets_total <= 16)
            {
                proto_handle->mesg_buffer[0] = packet;
                proto_handle->state = GET_DATA;
                proto_handle->packets_current = 0;
            }
        }
        break;

    case GET_DATA:
        if ((~packet) & 0x80)
        {
            proto_handle->mesg_buffer[proto_handle->packets_current + 1] = packet;

            if (++proto_handle->packets_current >= proto_handle->packets_total)
            {
                proto_handle->state = GET_CHECKSUM;
            }
        }
        else
        {
            proto_handle->state = GET_HEADER;
        }
        break;

    case GET_CHECKSUM:
        if ((~packet) & 0x80)
        {
            proto_handle->mesg_buffer[proto_handle->packets_current + 1] = packet;
            process_mesg(proto_handle);
        }
        proto_handle->state = GET_HEADER;
        break;
    }
}
