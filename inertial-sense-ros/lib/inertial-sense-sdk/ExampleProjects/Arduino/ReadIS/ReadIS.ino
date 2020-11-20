/*
  Read InertialSense
  This is a basic example of how to read data from the uINS using the InertialSense serial protocol
  The circuit:
  - inertialSense port H4 attached to Serial port 1
  - Serial Monitor open on Serial port 0
  created 26 Jan 2018
  by James Jackson
  This example code is in the public domain.
*/

#include "src/ISsdk/ISComm.h" // Run ImportSdkFiles(.bat or .sh) to copy needed SDK files into this project.
#include <stddef.h>

// This buffer is going to be used to hold messages as they come in.
// You can make this 512 size if memory is really tight.
static uint8_t s_buffer[1024];
static is_comm_instance_t comm;

static void handleINSMessage(ins_1_t *ins)
{
    Serial.print("Lat: ");
    Serial.print((float)ins->lla[0], 6);
    Serial.print("\t");
    Serial.print(", Lon: ");
    Serial.print((float)ins->lla[1], 6);
    Serial.print("\t");
    Serial.print(", Alt: ");
    Serial.print((float)ins->lla[2], 2);
    Serial.print("\t");
    Serial.print(", roll: ");
    Serial.print(ins->theta[0] * C_RAD2DEG_F);
    Serial.print("\t");
    Serial.print(", pitch: ");
    Serial.print(ins->theta[1] * C_RAD2DEG_F);
    Serial.print("\t");
    Serial.print(", yaw: ");
    Serial.print("\t");
    Serial.println(ins->theta[2] * C_RAD2DEG_F);
}

void setup()
{
    // Initialize both serial ports:
    Serial.begin(115200);
    Serial1.begin(115200);

    if (sizeof(double) != 8)
    {
        Serial.println("Inertial Sense SDK requires 64 bit double support");
        while (true)
        {
        };
    }

    Serial.println("initializing");

    // Initialize comm interface - call this before doing any comm functions
    is_comm_init(&comm, s_buffer, sizeof(s_buffer));

    // Stop all the broadcasts on the device
    int messageSize = is_comm_stop_broadcasts_all_ports(&comm);
    Serial1.write(comm.buf.start, messageSize); // Transmit the message to the inertialsense device

    // Ask for ins_1 message 20 times per second.  Ask for the whole thing, so
    // set 0's for the offset and size
    messageSize = is_comm_get_data(&comm, DID_INS_1, 0, sizeof(ins_1_t), 1000);
    Serial1.write(comm.buf.start, messageSize); // Transmit the message to the inertialsense device
}

void loop()
{

    // Read from port 1, and see if we have a complete inertialsense packet
    if (Serial1.available())
    {
        uint8_t inByte = Serial1.read();
        // This function returns the DID of the message that was just parsed, we can then point the buffer to
        // the right function to handle the message.  We can use a cast to interpret the s_buffer as the
        // kind of message that we received.
        uint32_t message_type = is_comm_parse_byte(&comm, inByte);
        switch (message_type)
        {
        case _PTYPE_INERTIAL_SENSE_DATA:
            switch (comm.dataHdr.id)
            {
            case DID_NULL:
                break;
            case DID_INS_1:
                handleINSMessage((ins_1_t *)(comm.dataPtr));
                break;
            default:
                Serial.print("Got an unexpected message DID: ");
                Serial.println(message_type, DEC);
            }
        }
    }
}
