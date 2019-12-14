/*
 * SX1262.h
 *
 *  Created on: 29 de Nov de 2019
 *      Authors: Eduardo Lacerda Campos
 *
 *  Driver para acessar a comunicação LoRa por meio do CI SX1262
 *
 */

#ifndef AVIONICA_SOURCE_LORA_H_
#define AVIONICA_SOURCE_LORA_H_


//#include <util/delay.h>
#include "Hard/SPI.h"
//#include <avr/io.h>
#include <assert.h>
#include <string.h>
#include "sx126x.h"

//Pin Out


// Arduino PD2 <--- SX1262 BUSY
// Arduino PD3 <--- SX1262 DIO1
// Arduino PD4 <--- SX1262 DIO2
// Arduino PD5 <--- SX1262 DIO3
// Arduino PD6 ---> SX1262 NRST

// Port numbers: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, ...
#define GPIOx(_N)                 ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))
#define PIN_MASK(_N)              (1 << (_N))
#define RCC_MASKx(_N)             (RCC_APB2Periph_GPIOA << (_N))

#define PD2 0
#define PD3 1
#define PD4 3
#define PD5 4
#define PD6 5

#define DIO0 PD2
#define DIO1 PD3
#define DIO2 PD4
#define DIO3 PD5
#define NRST PD6
#define NSS	 12  //chip select

#define TXEN 6
#define RXEN 7

#define BUSY DIO0
//BUSY line is held low = the radio is ready to accept a command from the host controller
#define BUSY_IS_HIGH (GPIO_ReadInputDataBit(GPIOB,PIN_MASK(BUSY)))

#define DIO2_IS_HIGH (GPIO_ReadInputDataBit(GPIOB,PIN_MASK(DIO2)))

/*!
 * \Provides the frequency of the chip running on the radio and the frequency step *
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define XTAL_FREQ                                   32000000 //32MHz
#define FREQ_DIV                                    33554432
#define FREQ_STEP                                   0.95367431640625 // ( ( double )( XTAL_FREQ / ( double )FREQ_DIV ) )
#define FREQ_ERR                                    0.47683715820312


//MACROS
#define WHILE_COUNT(__X)	\
	{	\
		uint16_t __Counter=0;	\
		while(__X)	\
		{	\
			_delay_ms(1);	\
			__Counter++;	\
			if (__Counter>=1000)	\
				return false;	\
		}	\
	}



class LORA : public SX126x{
public:
	LORA(RadioCallbacks_t *callbacks);
	virtual ~LORA();

	//Channel = 0x0F  -> Default 915 MHz
	bool Initialize(SPI* Serial,uint16_t Address, uint8_t Channel=0x0F);
	void Sleep();
	void PowerSaving();
	bool WakeUp();
	void Normal();
	void Reset();
	bool Avaliable(void);
	bool TXDone(void);

    void EnableTX(void);
    void EnableRX(void);

	RadioStatus_t GetStatus( void );
	void GetStats( RxCounter_t *pktStats );
	void ResetStats( void );
	RadioPacketTypes_t GetPacketType( void );
	bool ReadCommand( RadioCommands_t opcode, uint8_t *buffer, uint8_t size );
	bool ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );
	bool ReadRegister( uint16_t address, uint8_t *buffer, uint16_t size );
	void WriteReg( uint16_t address, uint8_t value );
	bool WriteCommand( RadioCommands_t opcode, uint8_t *buffer, uint8_t size );
	bool WriteRegister( uint16_t address, uint8_t *buffer, uint16_t size );
	bool WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size );

private:
	SPI* Serial;
	uint16_t Local_Address;
	uint8_t Local_Channel;


	uint8_t ReadReg( uint16_t address );
	uint8_t GetDeviceType( void );
	uint8_t GetFreqSelect( void );

	//Hardware dependent functions

	void Pin_Setup();
	uint8_t GetDioStatus( void );

    void AntSwOn( void ){};
    void AntSwOff( void ){};
    void IoIrqInit( DioIrqHandler irqHandler ){ assert(false);};


};



#endif /* AVIONICA_SOURCE_SX1262_H_ */
