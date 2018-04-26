# Long Data UART Protocol

This protocol is designed to transmit multibyte data such as large integers,
double precision floats, etc over UART with addressing and error detection. It
is a one way protocol; the host sends the data and the client listens but never
responds unless a protocol developed over top of this one includes such
facilities.

## Terminology

Packet refers to a byte recieved over the UART line.

Byte refers to a byte of actual data before being divided into packets.


## Message Format

The message format is as follows:

    1.             2.                    3.
    1 XXX YYYY  |  { 0 ZZZ ZZZZ } ... |  0 CCC CCCC

1. This is the message header. Message headers are signaled by asserting the
   most significant bit. The message header contains a three-bit reciever address
   - XXX - and the number of bytes of data being transmitted - YYYY.

2. This is the data sequence. Data packets have the MSB cleared and contain 7
   left-aligned data bits - ZZZ ZZZZ.

3. This is the checksum packet. This follows the data packets and also has the
   MSB cleared. The lower 7 bits are the lower 7 bits of the bitwise XOR of all
   data packets sent.


## Specifics on the Message Header

Any packet that has its MSB set is considered a message header. The next
three most significant bits are the address bits. They are the identifier for
the reciever of the data / index of the mailbox that the data will be placed into
once recieved. The bottom four bits are the number of bytes to be transmitted
minus 1, i.e. if the four LSBs are 0, then 1 byte will be transmitted. The bytes
to be transmitted field should be in the range [0, 13] or in other words, 14 bytes
is the maximum message length.


## Specifics on the Data Sequence

Any packet that has its MSB cleared is considered a data or checksum packet. The
seven LSBs contain the data bits which are left aligned. What this means is that say a
message wishes to transmit 1 byte; the message will naturally have two data
packets since packets only contain seven data bits. The seven MSBs of the byte
will be contained in the first packet and the LSB will be contained in second
packet, immediately after the MSB / control bit:

    Byte:        1101 0001
    As Packets:  1: { 0 110 1000 }    2: { 0 100 000}

When dealing with multibyte messages, the same principle applies; bytes will end
up stradling packets. Consider a more complicated 4 byte message:

    Raw Bytes:   1:{1101 0001}   2:{1101 0001}   3:{1101 0001}   4:{1101 0001}
    As Packets:  1:{0 110 1000}  2:{0 111 0100}  3:{0 011 1010}  4:{0 001 1101}  5:{0 000 1000}

Unused bits in the final packet shall be zeroed.


## Specifics on the Checksum

This is an additional packet that comes after all significant packets have been
recieved. It is NOT part of the actual data. The checksum is simply a bitwise XOR
of all the packets. The MSB of the checksum is always cleared. This packet is
used to check for errors in the transmission such as if the reciever misses
several packets and becomes misaligned with the stream.


## The Protocol

A message begins with a header packet. If the message size field is greater than
13, then the message should be ignored and the reciever should wait for the next
message header. If it is less than 13, then the reciever should expect to recieve
however many bytes are indicated plus one (zero indicates transmission of 1 byte).
This means at least that many packets are needed; the receiver will need to
calculate the number of packets expected as:

    packets = bytes * 8 / 7 + min((bytes * 8 % 7), 1)

The reciever should buffer the message and only commit it to the mailbox once
all data has been transmitted and successfully checksummed. If the reciever
sees a header packet while the current message is incomplete, the data for
the current message should be discarded and the reciever shall begin listening
for the next message. Because recieving a header packet prematurley could be
caused by a bit flip, the reciever should not attempt to use an interrupting
header packet as the demarcation of a new message and instead should wait again
for a header packet following the interrupting header packet.

Once all data has been recieved and passes the checksum, it then shall be committed
to its associated mailbox.
