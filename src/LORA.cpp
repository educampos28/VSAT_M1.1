/*
 * SX1261.cpp
 *
 *  Created on: 20 de jan de 2019
 *      Authors: educampos e Saulo Aislan da Silva Eleutério
 */

#include "LORA.h"
#include "string.h"

#define CHIP_SELECT		GPIO_ResetBits(GPIOB,PIN_MASK(NSS))
#define CHIP_DESELECT	GPIO_SetBits(GPIOB,PIN_MASK(NSS))

LORA::LORA(RadioCallbacks_t *callbacks):SX126x(callbacks){

	this->Local_Address = 0;
	this->Local_Channel = 0;
	Serial = NULL;

	//set pin as output
//	DDRD|= (NRST<<1);

	Pin_Setup();
	GPIO_SetBits(GPIOB,PIN_MASK(NRST)); ///
	GPIO_SetBits(GPIOB,PIN_MASK(NSS));

}

LORA::~LORA() {

}

//When BUSY line is held low the radio is ready to accept a command from the host controller
bool WaitBusy_Low(){

  uint16_t cnt = 0;

  while(BUSY_IS_HIGH)
  {
	  usleep(1000);
	  //avoid infinite loop, the "Counter" was introduced.
	  cnt++;
	  if (cnt>=1000)// considering the _delay_ms, this counter will wait 1s
		  return false; //fail
  }

  return true; //successful

}

void LORA::Normal() {

	GPIO_SetBits(GPIOB,PIN_MASK(NRST));

}

bool LORA::WakeUp() {

	return true;
}

void LORA::PowerSaving() {

	GPIO_ResetBits(GPIOB,PIN_MASK(RXEN));
	GPIO_ResetBits(GPIOB,PIN_MASK(TXEN));

}

void LORA::Sleep() {

}
/* Hardware reset
 *
 * The pin should be held low for more than 50 μs (typically 100 μs) for the Reset to happen
*/
void LORA::Reset() {

	GPIO_ResetBits(GPIOB,PIN_MASK(NRST));
	usleep(200000);
	GPIO_SetBits(GPIOB,PIN_MASK(NRST));
	usleep(1000000);
}

bool LORA::Avaliable(){

	volatile RadioStatus_t status;

	status.Value = GetStatus().Value;

	if(status.Fields.CmdStatus == SX126X_STATUS_DATA_AVAILABLE)
		return true;
	return false;
}

bool LORA::TXDone(void){

	volatile RadioStatus_t status;

	status.Value = GetStatus().Value;

	if(status.Fields.CmdStatus == SX126X_STATUS_TX_DONE)
		return true;
	return false;
}

/*Set pin as output*/
void LORA::Pin_Setup()
{

	// Enable GPIO Peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Pin = PIN_MASK(NRST);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Pin = PIN_MASK(NSS);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure pin in input
	GPIO_InitStructure.GPIO_Pin = PIN_MASK(BUSY);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure pin in input
	GPIO_InitStructure.GPIO_Pin = PIN_MASK(DIO2);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure pin in input
	GPIO_InitStructure.GPIO_Pin = PIN_MASK(TXEN);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Configure pin in input
	GPIO_InitStructure.GPIO_Pin = PIN_MASK(RXEN);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB,PIN_MASK(RXEN));
	GPIO_ResetBits(GPIOB,PIN_MASK(TXEN));

}

bool LORA::Initialize(SPI* Serial,uint16_t Address, uint8_t Channel){

	//Attach the serial port
	this->Serial  = Serial;
	this->Local_Address = Address;
	this->Local_Channel = Channel;

	Pin_Setup();

	Reset();

	while(BUSY_IS_HIGH);

	volatile RadioError_t error;
	volatile RadioStatus_t status;

	status.Value= GetStatus().Value;

	assert(status.Fields.ChipMode==SX126X_STATUS_MODE_STDBY_RC);

	Init(Channel);

	while(BUSY_IS_HIGH);

	assert(GetPacketType()==PACKET_TYPE_LORA);

	error.Value = GetDeviceErrors().Value;
	status.Value = GetStatus().Value;

	//Define where the data payload will be stored with the command
	txBaseAddress=0;
	rxBaseAddress=0;
	SetBufferBaseAddresses(txBaseAddress,rxBaseAddress);

	// Define the modulation parameter according to the chosen protocol
	ModulationParams_t modulationParams;
	modulationParams.PacketType = PACKET_TYPE_LORA;
	modulationParams.Params.LoRa.Bandwidth = LORA_BW_500;
	modulationParams.Params.LoRa.CodingRate = LORA_CR_4_8;
	modulationParams.Params.LoRa.SpreadingFactor = LORA_SF10;
	SetModulationParams(&modulationParams);

	error.Value = GetDeviceErrors().Value;
	status.Value = GetStatus().Value;

//	Define the frame format
	PacketParams_t packetParams;
	packetParams.PacketType = PACKET_TYPE_LORA;
	packetParams.Params.LoRa.PreambleLength = 0x08;
	packetParams.Params.LoRa.PayloadLength = 0x0A;
	packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
	packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
	packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	SetPacketParams(&packetParams);

	error.Value = GetDeviceErrors().Value;
	status.Value = GetStatus().Value;

	assert((error.Value & 0x00FF) == 0);

	return true; //successful
}

//Overwrite the classe function
RadioStatus_t LORA::GetStatus( void )
{
    RadioStatus_t status;

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

	Serial->write_read( ( uint8_t )RADIO_GET_STATUS );
	status.Value = Serial->write_read( 0xFF );
	CHIP_DESELECT;

    return status;
}

void LORA::GetStats( RxCounter_t *pktStats )
{
	uint8_t size = 6;
	uint8_t buffer[size];

	while(BUSY_IS_HIGH);

	//has status
    ReadCommand( RADIO_GET_STATS, buffer, size );

    memcpy( pktStats +2, buffer , sizeof( RxCounter_t ) -2 );

    return;

}

RadioPacketTypes_t LORA::GetPacketType( void ){

	assert(Serial != NULL);

	uint8_t size = 1;
	uint8_t buffer[size];

	RadioPacketTypes_t Radio;

	//has status
    ReadCommand( RADIO_GET_PACKETTYPE, buffer, size );

    Radio = (RadioPacketTypes_t)buffer[0];

    return Radio;
}

void LORA::ResetStats( void )
{
	uint8_t size = 6;
	uint8_t buffer[size];

	WriteCommand( RADIO_RESET_STATS, buffer, size );

    return;

}

uint8_t LORA::ReadReg( uint16_t address )
{
    uint8_t data;

    ReadRegister( address, &data, 1 );
    return data;
}

//------------------------------------------------------------------------------
bool LORA::WriteCommand( RadioCommands_t opcode, uint8_t *buffer, uint8_t size ){

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

	Serial->write_read(opcode);
	Serial->writeBytes(buffer,size);
	CHIP_DESELECT;

	return true;

}

bool LORA::WriteRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{

    assert(Serial != NULL);

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

	Serial->write_read( RADIO_WRITE_REGISTER );
	Serial->write_read( ( address & 0xFF00 ) >> 8 );


	volatile RadioStatus_t status;
	status.Value = Serial->write_read( address & 0x00FF );;

	if (( status.Value == SX126X_STATUS_CMD_TIMEOUT) &&
		( status.Value == SX126X_STATUS_CMD_INVALID) &&
		( status.Value == SX126X_STATUS_CMD_FAILED))
		{
			assert(false);
			return false;
		}


	for( uint16_t i = 0; i < size; i++ )
	{
		Serial->write_read( buffer[i] );
	}
	CHIP_DESELECT;

	return true;

}

bool LORA::ReadRegister( uint16_t address, uint8_t *buffer, uint16_t size )
{

    assert(Serial != NULL);

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

	Serial->write_read( RADIO_READ_REGISTER );
	Serial->write_read( ( address & 0xFF00 ) >> 8 );
	Serial->write_read( address & 0x00FF );

	volatile RadioStatus_t status;
	status.Value = Serial->write_read( 0xFF );

	if (( status.Value == SX126X_STATUS_CMD_TIMEOUT) &&
		( status.Value == SX126X_STATUS_CMD_INVALID) &&
		( status.Value == SX126X_STATUS_CMD_FAILED))
		{
			assert(false);
			return false;
		}

	for( uint16_t i = 0; i < size; i++ )
	{
		buffer[i] = Serial->write_read( 0xFF );
	}
	CHIP_DESELECT;

	return true;

}

void LORA::WriteReg( uint16_t address, uint8_t value )
{
    WriteRegister( address, &value, 1 );
}

bool LORA::WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    assert(Serial != NULL);

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

    Serial->write_read( RADIO_WRITE_BUFFER );


	volatile RadioStatus_t status;
	status.Value = Serial->write_read( offset );

	if (( status.Value == SX126X_STATUS_CMD_TIMEOUT) &&
		( status.Value == SX126X_STATUS_CMD_INVALID) &&
		( status.Value == SX126X_STATUS_CMD_FAILED))
		{
			assert(false);
			return false;
		}

	for( uint16_t i = 0; i < size; i++ )
	{
		Serial->write_read( buffer[i] );

	}
	CHIP_DESELECT;

    return true;
}



bool LORA::ReadCommand( RadioCommands_t opcode, uint8_t *buffer, uint8_t size )
{

    assert(Serial != NULL);

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

	Serial->write_read( ( uint8_t )opcode );

	volatile RadioStatus_t status;
	status.Value = Serial->write_read( 0xFF );

	if (( status.Value == SX126X_STATUS_CMD_TIMEOUT) &&
		( status.Value == SX126X_STATUS_CMD_INVALID) &&
		( status.Value == SX126X_STATUS_CMD_FAILED))
		{
			assert(false);
			return false;
		}

	for( uint8_t i = 0; i < size; i++ )
	{
		 buffer[i] = Serial->write_read( 0xFF );
	}
	CHIP_DESELECT;

	//TODO - Evaluate the status bits to identify any error
	return true;
}

bool LORA::ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{

    assert(Serial != NULL);

	CHIP_SELECT;
	usleep(10);
	while(BUSY_IS_HIGH);

	Serial->write_read( RADIO_READ_BUFFER );
	Serial->write_read( offset );

	volatile RadioStatus_t status;
	status.Value = Serial->write_read( 0xFF );

	if (( status.Value == SX126X_STATUS_CMD_TIMEOUT) &&
		( status.Value == SX126X_STATUS_CMD_INVALID) &&
		( status.Value == SX126X_STATUS_CMD_FAILED))
		{
			assert(false);
			return false;
		}

	for( uint16_t i = 0; i < size; i++ )
	{
		buffer[i] = Serial->write_read( 0xFF );
	}
	CHIP_DESELECT;

	return true;
}

uint8_t LORA::GetDioStatus( void )
{
    //return ( *DIO3 << 3 ) | ( *DIO2 << 2 ) | ( *DIO1 << 1 ) | ( BUSY << 0 );

    return ((GPIO_ReadInputDataBit(GPIOB,PIN_MASK(DIO3))) << 3 |
    		(GPIO_ReadInputDataBit(GPIOB,PIN_MASK(DIO2))) << 2 |
			(GPIO_ReadInputDataBit(GPIOB,PIN_MASK(DIO1))) << 1 |
			(GPIO_ReadInputDataBit(GPIOB,PIN_MASK(BUSY))));
}

uint8_t LORA::GetDeviceType( void )
{

        return( SX1262 );

}

uint8_t LORA::GetFreqSelect( void )
{

   return( MATCHING_FREQ_915 );

}

void LORA::EnableTX(void){

	GPIO_ResetBits(GPIOB,PIN_MASK(RXEN));
	usleep(10);
	GPIO_SetBits(GPIOB,PIN_MASK(TXEN));
	usleep(100000);

}
void LORA::EnableRX(void){

	GPIO_ResetBits(GPIOB,PIN_MASK(TXEN));
	usleep(10);
	GPIO_SetBits(GPIOB,PIN_MASK(RXEN));
	usleep(100000);
}
