#ifndef __LONG_DATA_UART_PROTOCOL__
#define __LONG_DATA_UART_PROTOCOL__

#include <Arduino.h>

#define MAX_MESG_SIZE 14

/// Enum for the states of the finite state machine which implements the Long
/// Data UART Protocol.
enum LDProtocolState
{
    GET_HEADER,
    GET_DATA,
    GET_CHECKSUM,
};

/**
 * This struct is used to implement and track the state of the Long Data UART
 * Protocol as described in the LongDataUARTProtocol.md document. An instance of
 * this struct must be declared and initialized to use the protocol.
 *
 * To use the Long Data UART Protocol, the user must:
 *
 * 1. Create an instance of this struct and initialize all fields to zero either
 *    manually or using the init_ldprotocol() function.
 *
 * 2. Inform the protocol handle (this struct) of where the mailboxes for
 *    receiving data are located and how big these mailboxes are using the
 *    register_mailbox() function.
 *
 * 3. Finally, call the recieve_message() function in a loop.
 *
 * Note that serial initialization is NOT performed by the init_ldprotocol()
 * function. The user is responsible for initializing serial to their desired
 * preference.
 */
struct LDProtocol
{
    char mailbox_addr; ///< Mailbox to which the next message should be commited
    char state; ///< Current state of the protocol. Used for controlling the protocol's FSM.

    char bytes_total; ///< Total number of bytes the current message contains.
    char packets_total; ///< Total number of packets the current message contains. Includes checksum.
    char packets_current; ///< Sequence number of the last packet received.

    unsigned char mesg_buffer[18]; ///< Buffer to place the message into while still receiving data.

    char mailbox_sizes[8]; ///< Size of each mailbox in bytes.
    unsigned char* mailboxes[8]; ///< pointers to each mailbox.
};

/**
 * Initializes the given protocol handle to zero in all fields. Should be called
 * before attempting to register new mailboxes or receive_messages.
 *
 * @param proto_handle: pointer to an LDProtocol struct.
 */
void init_ldprotocol(struct LDProtocol* proto_handle);

/**
 * Informs the protocol handle of the location and size of a mailbox for data.
 * Must be called before the mailbox with address 'which' can be used.
 *
 * @param which: which mailbox to register. Must not be greater than 7.
 * @param mailbox: pointer to the buffer associated with the mailbox.
 * @param size: size of the buffer in bytes. messages recieved for this mailbox must be less than this.
 * @param proto_handle: pointer to the protocol handle.
 */
void register_mailbox(char which, void* mailbox, char size, struct LDProtocol* proto_handle);

/**
 * Calculates the number of packets in a message including checksum for a message
 * which contains some number of bytes.
 *
 * @param bytes: number of bytes the message contains.
 * @return Number of packets the message is composed of. Includes checksum.
 */
char bytes_to_packets(char bytes);

/**
 * This function operates the finite state machine for implementing the Long
 * Data UART Protocol. Once mailboxes have been registered, this function should
 * be called in a loop.
 *
 * @param proto_handle: pointer to the protocol handle.
 */
void recieve_message(struct LDProtocol* proto_handle);

#endif
