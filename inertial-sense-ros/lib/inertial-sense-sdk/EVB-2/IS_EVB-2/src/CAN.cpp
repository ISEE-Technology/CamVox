/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* module_inst */

#include "asf.h"
#include "CAN.h"
#include "globals.h"

#define MCAN_TEST_SEND_ID_STD	0x125
#define MCAN_TEST_SEND_ID_EXT	0x100000A5
#define MCAN_TEST_RECV_ID_STD	0x126
#define MCAN_TEST_RECV_ID_EXT	0x100000A6

static struct mcan_module mcan_instance;

void CAN_init(void)
{
	mcan_stop(&mcan_instance);
	/* Initialize the module. */
	struct mcan_config config_mcan;
	mcan_get_config_defaults(&config_mcan);
	config_mcan.rx_fifo_0_overwrite = true;
	mcan_init(&mcan_instance, MCAN1, &config_mcan);
	mcan_set_baudrate(mcan_instance.hw, (uint32_t)g_flashCfg->CANbaud_kbps*1000);
	mcan_set_rx_filter(g_flashCfg->can_receive_address);

	mcan_start(&mcan_instance);
}

bool mcan_send_message(uint32_t id_value, uint8_t *data, uint32_t data_length)
{
	uint32_t status = mcan_tx_get_fifo_queue_status(&mcan_instance);

	//check if fifo is full
	if(status & MCAN_TXFQS_TFQF) {
		return false;
	}
	
	//Prevent sending more data than buffer size
	if(data_length > CONF_MCAN_ELEMENT_DATA_SIZE)
		data_length = CONF_MCAN_ELEMENT_DATA_SIZE;

	//get the put index where we put the next packet
	uint32_t put_index = (status & MCAN_TXFQS_TFQPI_Msk) >> MCAN_TXFQS_TFQPI_Pos;

	struct mcan_tx_element tx_element;
	mcan_get_tx_buffer_element_defaults(&tx_element);

	if(id_value >= 0x800)
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) | MCAN_TX_ELEMENT_T0_XTD;
	else
		tx_element.T0.reg |= MCAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
		
	tx_element.T1.bit.DLC = data_length;

	for (uint32_t i = 0; i < data_length; i++) {
		tx_element.data[i] = data[i];
	}

	mcan_set_tx_buffer_element(&mcan_instance, &tx_element, put_index);
	mcan_tx_transfer_request(&mcan_instance, (1 << put_index));

	return true;
}

/* Returns data length received or zero for no messages available.
*/
uint8_t mcan_receive_message(uint32_t *id_value, uint8_t *data)
{
	uint32_t fifo_status;
	
	//Check standard message fifo
	fifo_status = mcan_rx_get_fifo_status(&mcan_instance, 0);
	if (fifo_status & MCAN_RXF0S_F0FL_Msk)
	{
		struct mcan_rx_element_fifo_0 rx_element_buffer;
		uint32_t rx_fifo_index = (fifo_status >> MCAN_RXF0S_F0GI_Pos ) & 0x3fu;
			
		mcan_get_rx_fifo_0_element(&mcan_instance, &rx_element_buffer, rx_fifo_index);
		mcan_rx_fifo_acknowledge(&mcan_instance, 0, rx_fifo_index);
				
		//Read out data
		*id_value = rx_element_buffer.R0.bit.ID >> 18;
		for(int i=0; i<rx_element_buffer.R1.bit.DLC; i++)
			data[i] = rx_element_buffer.data[i];

		return rx_element_buffer.R1.bit.DLC;				
	}
	
	//check extended message fifo
	fifo_status = mcan_rx_get_fifo_status(&mcan_instance, 1);
	if (fifo_status & MCAN_RXF1S_F1FL_Msk)
	{
		struct mcan_rx_element_fifo_1 rx_element_buffer;
		uint32_t rx_fifo_index = (fifo_status >> MCAN_RXF1S_F1GI_Pos ) & 0x3fu;
		
		mcan_get_rx_fifo_1_element(&mcan_instance, &rx_element_buffer, rx_fifo_index);
		mcan_rx_fifo_acknowledge(&mcan_instance, 1, rx_fifo_index);
		
		//Read out data
		*id_value = rx_element_buffer.R0.bit.ID;
		for(int i=0; i<rx_element_buffer.R1.bit.DLC; i++)
		data[i] = rx_element_buffer.data[i];

		return rx_element_buffer.R1.bit.DLC;
	}
	
	//No data ready
	return 0;
}

/* Adds ID filter
 * returns index of added ID or -1 for errror
 */
int mcan_set_rx_filter(uint32_t id_value)
{
	static int standard_filter_count = 0;
	static int extended_filter_count = 0;
	
	if(id_value >= 0x800)
	{	//extended id
		struct mcan_extended_message_filter_element et_filter;

		if(extended_filter_count >= CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM)
			return -1;
			
		mcan_get_extended_message_filter_element_default(&et_filter);
		et_filter.F0.bit.EFID1 = id_value;
		
		mcan_set_rx_extended_filter(&mcan_instance, &et_filter, extended_filter_count);
		
		return extended_filter_count++;
	}
	else
	{	//standard id
		struct mcan_standard_message_filter_element sd_filter;

		if(standard_filter_count >= CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM)
			return -1;

		mcan_get_standard_message_filter_element_default(&sd_filter);
		sd_filter.S0.bit.SFID1 = id_value;

		mcan_set_rx_standard_filter(&mcan_instance, &sd_filter, standard_filter_count);
		
		return standard_filter_count++;
	}
}

/* Master device for CAN test
 * This unit sends a message out the CAN bus then receives the return message and checks that it is valid.
 */
void mcan_test_master(void)
{
	uint8_t msg_cnt = 0;
	
	uint32_t success_count = 0;
	uint32_t failure_count = 0;
	
	//Setup receive
	mcan_set_rx_filter(MCAN_TEST_RECV_ID_STD);
	mcan_set_rx_filter(MCAN_TEST_RECV_ID_EXT);
		
	while(1)
	{
		uint8_t can_tx_message[CONF_MCAN_ELEMENT_DATA_SIZE];
		uint8_t can_rx_message[CONF_MCAN_ELEMENT_DATA_SIZE];
		uint8_t len;

		//Create message
		msg_cnt++;
		len = 2 + rand() % 7;
		
		for(int i=0; i<len; i++)
			can_tx_message[i] = rand();
			
		//Send message
		if(msg_cnt % 2 == 0)
			mcan_send_message(MCAN_TEST_SEND_ID_STD, can_tx_message, len);
		else
			mcan_send_message(MCAN_TEST_SEND_ID_EXT, can_tx_message, len);
				
		//Wait for and Get response
		uint32_t id, timeout = 100;
		while((len = mcan_receive_message(&id, can_rx_message)) == 0 && --timeout > 0)
			vTaskDelay(1);
		
		//Check return message
		int i;
		for(i=0; i<len; i++)
			if((uint8_t)(can_tx_message[i] + 1) != can_rx_message[i])
				break;

		if(i == len)
			success_count++;
		else
			failure_count++;
		//TODO: Do something with test result		
	}
}

/* Slave device for CAN test
 * This unit listens for a CAN message and returns a response.
 */
void mcan_test_slave(void)
{
	//Setup receive
	mcan_set_rx_filter(MCAN_TEST_SEND_ID_STD);
	mcan_set_rx_filter(MCAN_TEST_SEND_ID_EXT);
		
	while(1)
	{
		uint8_t can_tx_message[CONF_MCAN_ELEMENT_DATA_SIZE];
		uint8_t can_rx_message[CONF_MCAN_ELEMENT_DATA_SIZE];
		uint32_t id;
		uint8_t len;
		
		if((len = mcan_receive_message(&id, can_rx_message)) > 0)
		{
			//Change data and send back
			for(int i=0; i<len; i++)
				can_tx_message[i] = can_rx_message[i] + 1;
			
			if(id == MCAN_TEST_SEND_ID_STD)
				mcan_send_message(MCAN_TEST_RECV_ID_STD, can_tx_message, len);
			else
				mcan_send_message(MCAN_TEST_RECV_ID_EXT, can_tx_message, len);
		}
	}		
}
