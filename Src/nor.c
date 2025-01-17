/*
 * nor.c
 *
 *  Created on: 5 de out de 2023
 *      Author: pablo-jean
 */

#include "nor.h"


#if defined (NOR_DEBUG)
#include <stdarg.h>
#include <stdio.h>
#endif

/*
 * Privates
 */

#if defined (NOR_DEBUG)
// TODO The printf :)
#define NOR_PRINTF(...)			printf(__VA_ARGS__)
#else
#define NOR_PRINTF(...)
#endif

/* If you are in a RTOS Environment, and has more devices into the */
/* Spi Bus, please, implement a threadsafe method (semaphores) */
#if defined (NOR_THREADSAFE)
#define _NOR_LOCK()		// TODO Lock function
#define _NOR_UNLOCK()	// TODO Unlock function
#else
#define _NOR_LOCK()
#define _NOR_UNLOCK()
#endif

#ifndef NOR_EMPTY_CHECK_BUFFER_LEN
#define NOR_EMPTY_CHECK_BUFFER_LEN		64
#endif

#define _SANITY_CHECK(n)			if (n == NULL)	return NOR_INVALID_PARAMS;					\
									if (n->_internal.u16Initialized != NOR_INITIALIZED_FLAG)	\
										return NOR_NOT_INITIALIZED;

/* Enumerates */

enum _nor_sr_select_e{
	_SELECT_SR1,
	_SELECT_SR2,
	_SELECT_SR3,
};

/* Functions */

static void _nor_cs_assert(nor_t *nor){
	nor->config.CsAssert();
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
}

static void _nor_cs_deassert(nor_t *nor){
	nor->config.CsDeassert();
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
}

static void _nor_spi_tx(nor_t *nor, uint8_t *txBuf, uint32_t size){
	nor->config.SpiTxFxn(txBuf, size);
}

static void _nor_spi_rx(nor_t *nor, uint8_t *rxBuf, uint32_t size){
	nor->config.SpiRxFxn(rxBuf, size);
}

static void _nor_delay_us(nor_t *nor, uint32_t us){
	nor->config.DelayUs(us);
}

static uint32_t _nor_ReadID(nor_t *nor)
{
	uint8_t JedecIdCmd = NOR_JEDEC_ID;
	uint32_t ID = 0;

	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &JedecIdCmd, sizeof(JedecIdCmd));
	_nor_spi_rx(nor, (uint8_t*)&ID, 3);
	_nor_cs_deassert(nor);

	return ID;
}

static uint64_t _nor_ReadUniqID(nor_t *nor)
{
	uint8_t UniqueIdCmd = NOR_UNIQUE_ID;
	uint64_t UniqueId = 0;
	uint32_t DummyU32 = (~0UL);

	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &UniqueIdCmd, sizeof(UniqueIdCmd));
	// this if the 4 dummy byte
	_nor_spi_tx(nor, (uint8_t*)&DummyU32, 4);
	_nor_spi_rx(nor, (uint8_t*)&UniqueId, sizeof(UniqueId));
	_nor_cs_deassert(nor);

	return UniqueId;
}

static void _nor_WriteEnable(nor_t *nor)
{
	uint8_t WriteEnCmd = NOR_CMD_WRITE_EN;

	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &WriteEnCmd, sizeof(WriteEnCmd));
	_nor_cs_deassert(nor);
	// TODO Check if a delay was needed here
}

void _nor_WriteDisable(nor_t *nor)
{
	uint8_t WriteDisCmd = NOR_CMD_WRITE_DIS;

	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &WriteDisCmd, sizeof(WriteDisCmd));
	_nor_cs_deassert(nor);
}

uint8_t _nor_ReadStatusRegister(nor_t *nor, enum _nor_sr_select_e SelectSR)
{
	uint8_t status = 0, ReadSRCmd;
	uint8_t *SrUpdateHandler;

	switch (SelectSR){
	case _SELECT_SR1:
		ReadSRCmd = NOR_READ_SR1;
		SrUpdateHandler = &nor->_internal.u8StatusReg1;
		break;
	case _SELECT_SR2:
		ReadSRCmd = NOR_READ_SR2;
		SrUpdateHandler = &nor->_internal.u8StatusReg1;
		break;
	case _SELECT_SR3:
		ReadSRCmd = NOR_READ_SR3;
		SrUpdateHandler = &nor->_internal.u8StatusReg1;
		break;
	default:
		return 0xFF;
	}
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &ReadSRCmd, sizeof(ReadSRCmd));
	_nor_spi_rx(nor, &status, sizeof(status));
	_nor_cs_deassert(nor);

	*SrUpdateHandler = status;

	return status;
}

void _nor_WriteStatusRegister(nor_t *nor, enum _nor_sr_select_e SelectSR, uint8_t data)
{
	uint8_t WriteSR[2];

	switch (SelectSR){
	case _SELECT_SR1:
		WriteSR[0] = NOR_WRITE_SR1;
		nor->_internal.u8StatusReg1 = data;
		break;
	case _SELECT_SR2:
		WriteSR[0] = NOR_WRITE_SR2;
		nor->_internal.u8StatusReg1 = data;
		break;
	case _SELECT_SR3:
		WriteSR[0] = NOR_WRITE_SR3;
		nor->_internal.u8StatusReg1 = data;
		break;
	default:
		return ;
	}
	WriteSR[1] = data;
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, WriteSR, sizeof(WriteSR));
	_nor_cs_deassert(nor);
}

nor_err_e _nor_WaitForWriteEnd(nor_t *nor, uint32_t msTimeout)
{
	uint8_t ReadSr1Cmd = NOR_READ_SR1;

	// multply timeout for 10, we must run a delay of 100us on each iteraction
	msTimeout *= 10;
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, (uint8_t*)&ReadSr1Cmd, sizeof(ReadSr1Cmd));
	do{
		_nor_spi_rx(nor, &nor->_internal.u8StatusReg1, sizeof(uint8_t));
		_nor_delay_us(nor, 100);
		msTimeout -= 100;
	}while ((nor->_internal.u8StatusReg1 & SR1_BUSY_BIT) && (msTimeout > 0));
	_nor_cs_deassert(nor);

	if (msTimeout == 0){
		return NOR_FAIL;
	}
	return NOR_OK;
}

nor_err_e _nor_check_buff_is_empty(uint8_t *pBuffer, uint32_t len){
	uint32_t i;

	for (i=0 ; i<len ; i++){
		if (pBuffer[i] != 0xFF){
			return NOR_REGIONS_IS_NOT_EMPTY;
		}
	}

	return NOR_OK;
}

/*
 * Publics
 */

nor_err_e NOR_Init(nor_t *nor){
	uint8_t ExitPDCmd = NOR_RELEASE_PD;

	if (nor == NULL || nor->config.CsAssert == NULL ||
			nor->config.CsDeassert == NULL || nor->config.DelayUs == NULL ||
			nor->config.SpiRxFxn == NULL || nor->config.SpiTxFxn == NULL){
		NOR_PRINTF("ERROR: Invalid Parameters on %s function\n", __func__);
		return NOR_INVALID_PARAMS;
	}
	if (nor->_internal.u16Initialized == NOR_INITIALIZED_FLAG){
		// the flash instance is already initialized
		NOR_PRINTF("Warning: Flash is already initialized.\n");
		return NOR_OK;
	}
	// we must have sure that the NOR has your CS pin deasserted
	_nor_cs_deassert(nor);
	_nor_delay_us(nor, 100);

	// we are assuming, on startup, that the Flash is on Power Down State
	nor->_internal.u8PdCount = 0;
	nor->pdState = NOR_IN_IDLE;
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &ExitPDCmd, sizeof(ExitPDCmd));
	_nor_cs_deassert(nor);

	nor->info.u32JedecID = _nor_ReadID(nor);
	if (nor->info.u32JedecID == 0x000000 || nor->info.u32JedecID == 0xFFFFFF){
		// invalid Id, I thing we don't has any flash on SPI
		NOR_PRINTF("ERROR: Flash memory bus fault.\n");
		return NOR_NO_MEMORY_FOUND;
	}
	nor->Manufacturer = NOR_IDS_Interpret_Manufacturer(nor->info.u32JedecID);
	nor->Model = NOR_IDS_Interpret_Model(nor->info.u32JedecID);
	if (nor->Model == NOR_MODEL_UNKNOWN){
		NOR_PRINTF("ERROR: The flash memory model wasn't reconignized.\n"
				"You can, yet, start with NOR_Init_wo_ID to ignore the Flash ID.");
		return NOR_UNKNOWN_DEVICE;
	}

	nor->info.u64UniqueId = _nor_ReadUniqID(nor);
	nor->info.u32BlockCount = NOR_IDS_GetQtdBlocks(nor->info.u32JedecID);

	nor->info.u16PageSize = NOR_PAGE_SIZE;
	nor->info.u16SectorSize = NOR_SECTOR_SIZE;
	nor->info.u32BlockSize = NOR_BLOCK_SIZE;
	nor->info.u32SectorCount = nor->info.u32BlockCount * (NOR_BLOCK_SIZE / NOR_SECTOR_SIZE);
	nor->info.u32PageCount = (nor->info.u32SectorCount * nor->info.u16SectorSize) / nor->info.u16PageSize;
	nor->info.u32Size = (nor->info.u32SectorCount * nor->info.u16SectorSize);

	_nor_ReadStatusRegister(nor, _SELECT_SR1);
	_nor_ReadStatusRegister(nor, _SELECT_SR1);
	_nor_ReadStatusRegister(nor, _SELECT_SR1);

	nor->_internal.u16Initialized = NOR_INITIALIZED_FLAG;
	NOR_PRINTF("== Memory Flash NOR Information ==\n");
	NOR_PRINTF(" JEDEC ID     | 0x%06X\n", (uint)nor->info.u32JedecID);
	NOR_PRINTF(" Unique ID    | 0x%lluX\n", (nor->info.u64UniqueId));
	NOR_PRINTF(" Page Size    | %d Bytes\n", nor->info.u16PageSize);
	NOR_PRINTF(" Sector Size  | %d Bytes\n", (uint)nor->info.u16SectorSize);
	NOR_PRINTF(" Block Size   | %d Bytes\n", (uint)nor->info.u32BlockSize);
	NOR_PRINTF(" Page Count   | %d Pages\n", (uint)nor->info.u32PageCount);
	NOR_PRINTF(" Sector Count | %d Sectors\n", (uint)nor->info.u32SectorCount);
	NOR_PRINTF(" Block Count  | %d Blocks\n", (uint)nor->info.u32BlockCount);
	NOR_PRINTF(" Capacity     | %d KB\n", (uint)(nor->info.u32Size/1024));
	NOR_PRINTF(" == NOR Initialization Done ==\n");

	return NOR_OK;
}

nor_err_e NOR_Init_wo_ID(nor_t *nor){
	uint8_t ExitPDCmd = NOR_RELEASE_PD;

	if (nor == NULL || nor->config.CsAssert == NULL ||
			nor->config.CsDeassert == NULL || nor->config.DelayUs == NULL ||
			nor->config.SpiRxFxn == NULL || nor->config.SpiTxFxn == NULL ||
			nor->info.u32BlockCount == 0){
		return NOR_INVALID_PARAMS;
	}
	if (nor->_internal.u16Initialized == NOR_INITIALIZED_FLAG){
		// the flash instance is already initialized
		return NOR_OK;
	}
	// we must have sure that the NOR has your CS pin deasserted
	_nor_cs_deassert(nor);
	_nor_delay_us(nor, 100);

	// we are assuming, on startup, that the Flash is on Power Down State
	nor->_internal.u8PdCount = 0;
	nor->pdState = NOR_IN_IDLE;
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &ExitPDCmd, sizeof(ExitPDCmd));
	_nor_cs_deassert(nor);

	nor->info.u32JedecID = _nor_ReadID(nor);
	nor->info.u64UniqueId = _nor_ReadUniqID(nor);

	nor->info.u16PageSize = NOR_PAGE_SIZE;
	nor->info.u16SectorSize = NOR_SECTOR_SIZE;
	nor->info.u32BlockSize = NOR_BLOCK_SIZE;

	nor->info.u32SectorCount = nor->info.u32BlockCount * (NOR_BLOCK_SIZE / NOR_SECTOR_SIZE);
	nor->info.u32PageCount = (nor->info.u32SectorCount * nor->info.u16SectorSize) / nor->info.u16PageSize;
	nor->info.u32Size = (nor->info.u32SectorCount * nor->info.u16SectorSize);

	_nor_ReadStatusRegister(nor, _SELECT_SR1);
	_nor_ReadStatusRegister(nor, _SELECT_SR2);
	_nor_ReadStatusRegister(nor, _SELECT_SR3);

	nor->_internal.u16Initialized = NOR_INITIALIZED_FLAG;
	NOR_PRINTF("== Memory Flash NOR Information ==\n");
	NOR_PRINTF(" JEDEC ID     | 0x%06X\n", (uint)nor->info.u32JedecID);
	NOR_PRINTF(" Unique ID    | 0x%lluX\n", (nor->info.u64UniqueId));
	NOR_PRINTF(" Page Size    | %d Bytes\n", nor->info.u16PageSize);
	NOR_PRINTF(" Sector Size  | %d Bytes\n", (uint)nor->info.u16SectorSize);
	NOR_PRINTF(" Block Size   | %d Bytes\n", (uint)nor->info.u32BlockSize);
	NOR_PRINTF(" Page Count   | %d Pages\n", (uint)nor->info.u32PageCount);
	NOR_PRINTF(" Sector Count | %d Sectors\n", (uint)nor->info.u32SectorCount);
	NOR_PRINTF(" Block Count  | %d Blocks\n", (uint)nor->info.u32BlockCount);
	NOR_PRINTF(" Capacity     | %d KB\n", (uint)(nor->info.u32Size/1024));
	NOR_PRINTF(" == NOR Initialization Done ==\n");

	return NOR_OK;
}

nor_err_e NOR_ExitPowerDown(nor_t *nor){
	uint8_t ExitPDCmd = NOR_RELEASE_PD;

	_SANITY_CHECK(nor);

	if (nor->_internal.u8PdCount > 0){
		nor->_internal.u8PdCount--;
		if (nor->_internal.u8PdCount == 0){
			NOR_PRINTF("NOR Exiting Deep Power Down\n");
			_nor_cs_assert(nor);
			_nor_spi_tx(nor, &ExitPDCmd, sizeof(ExitPDCmd));
			_nor_cs_deassert(nor);
			nor->pdState = NOR_IN_IDLE;
		}
	}

	return NOR_OK;
}
nor_err_e NOR_EnterPowerDown(nor_t *nor){
	uint8_t DeepPDCmd = NOR_ENTER_PD;

	_SANITY_CHECK(nor);

	if (nor->_internal.u8PdCount == 0){
		NOR_PRINTF("NOR Enter in Deep Power Down\n");
		_nor_cs_assert(nor);
		_nor_spi_tx(nor, &DeepPDCmd, sizeof(DeepPDCmd));
		_nor_cs_deassert(nor);
		nor->pdState = NOR_DEEP_POWER_DOWN;
	}
	nor->_internal.u8PdCount++;

	return NOR_OK;
}

nor_err_e NOR_EraseChip(nor_t *nor){
	uint8_t EraseChipCmd = NOR_CHIP_ERASE;
	nor_err_e err;

	_SANITY_CHECK(nor);

	NOR_PRINTF("Starting Mass Erase\nWait ...\n");
	_nor_WriteEnable(nor);
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, &EraseChipCmd, sizeof(EraseChipCmd));
	_nor_cs_deassert(nor);
	err = _nor_WaitForWriteEnd(nor, NOR_EXPECT_ERASE_CHIP);
	if (err != NOR_OK){
		NOR_PRINTF("ERROR: Failed to erase flash\n");
	}
	NOR_PRINTF("Done!\n");
	return err;
}

nor_err_e NOR_EraseAddress(nor_t *nor, uint32_t Address, nor_erase_method_e method){
	uint8_t EraseChipCmd[4];
	uint32_t expectedTimeoutUs;
	nor_err_e err;

	_SANITY_CHECK(nor);

	switch (method){
	case NOR_ERASE_4K:
		NOR_PRINTF("Erasing 4K bytes on 0x%08X Address\n", (uint)Address);
		EraseChipCmd[0] = NOR_SECTOR_ERASE_4K;
		expectedTimeoutUs = NOR_EXPECT_4K_ERASE_TIME;
		break;
	case NOR_ERASE_32K:
		NOR_PRINTF("Erasing 32K bytes on 0x%08X Address\n", (uint)Address);
		EraseChipCmd[0] = NOR_SECTOR_ERASE_32K;
		expectedTimeoutUs = NOR_EXPECT_32K_ERASE_TIME;
		break;
	case NOR_ERASE_64K:
		NOR_PRINTF("Erasing 64K bytes on 0x%08X Address\n", (uint)Address);
		EraseChipCmd[0] = NOR_SECTOR_ERASE_64K;
		expectedTimeoutUs = NOR_EXPECT_64K_ERASE_TIME;
		break;
	}
	EraseChipCmd[1] = ((Address >> 16) & 0xFF);
	EraseChipCmd[2] = ((Address >> 8) & 0xFF);
	EraseChipCmd[3] = ((Address) & 0xFF);

	_nor_WriteEnable(nor);
	_nor_cs_assert(nor);
	_nor_spi_tx(nor, EraseChipCmd, sizeof(EraseChipCmd));
	_nor_cs_deassert(nor);
	err = _nor_WaitForWriteEnd(nor, expectedTimeoutUs);
	if (err != NOR_OK){
		NOR_PRINTF("ERROR: Failed to erase flash\n");
	}
	NOR_PRINTF("Done!\n");

	return err;
}

nor_err_e NOR_EraseSector(nor_t *nor, uint32_t SectorAddr){
	uint32_t Address;

	_SANITY_CHECK(nor);

	Address = SectorAddr * nor->info.u16SectorSize;
	return NOR_EraseAddress(nor, Address, NOR_ERASE_4K);
}

nor_err_e NOR_EraseBlock(nor_t *nor, uint32_t BlockAddr){
	uint32_t Address;

	_SANITY_CHECK(nor);

	Address = BlockAddr * nor->info.u32BlockSize;
	return NOR_EraseAddress(nor, Address, NOR_ERASE_64K);
}

uint32_t NOR_PageToSector(nor_t *nor, uint32_t PageAddr){
	_SANITY_CHECK(nor);
	return PageAddr * nor->info.u16PageSize / nor->info.u16SectorSize;
}

uint32_t NOR_PageToBlock(nor_t *nor, uint32_t PageAddr){
	_SANITY_CHECK(nor);
	return PageAddr * nor->info.u16PageSize / nor->info.u32BlockSize;
}

uint32_t NOR_SectorToBlock(nor_t *nor, uint32_t SectorAddr){
	_SANITY_CHECK(nor);
	return  SectorAddr * nor->info.u16SectorSize / nor->info.u32BlockSize;
}

uint32_t NOR_SectorToPage(nor_t *nor, uint32_t SectorAddr){
	_SANITY_CHECK(nor);
	return SectorAddr * nor->info.u16SectorSize / nor->info.u16PageSize;
}

uint32_t NOR_BlockToPage(nor_t *nor, uint32_t BlockAddr){
	_SANITY_CHECK(nor);
	return  BlockAddr * nor->info.u32BlockSize / nor->info.u16PageSize;
}

nor_err_e NOR_IsEmptyAddress(nor_t *nor, uint32_t Address, uint32_t NumBytesToCheck){
	uint8_t pBuffer[NOR_EMPTY_CHECK_BUFFER_LEN];

	_SANITY_CHECK(nor);

	NOR_PRINTF("Checking if %d bytes of Address 0x%08X are empty.\n", (uint)NumBytesToCheck, (uint)Address);
	while (NumBytesToCheck > 0){
		NOR_ReadBytes(nor, pBuffer, Address, NOR_EMPTY_CHECK_BUFFER_LEN);
		Address += NOR_EMPTY_CHECK_BUFFER_LEN;
		if (NumBytesToCheck >= NOR_EMPTY_CHECK_BUFFER_LEN){
			NumBytesToCheck -= NOR_EMPTY_CHECK_BUFFER_LEN;
		}
		else{
			NumBytesToCheck = 0;
		}
		if (_nor_check_buff_is_empty(pBuffer, NOR_EMPTY_CHECK_BUFFER_LEN) == NOR_REGIONS_IS_NOT_EMPTY){
			NOR_PRINTF("Warning: Region is NOT empty.\n");
			return NOR_REGIONS_IS_NOT_EMPTY;
		}
	}
	NOR_PRINTF("Region is empty.\n");
	return NOR_OK;
}

nor_err_e NOR_IsEmptyPage(nor_t *nor, uint32_t PageAddr, uint32_t Offset, uint32_t NumBytesToCheck){
	uint32_t ActAddress;

	_SANITY_CHECK(nor);

	ActAddress = (nor->info.u16PageSize * PageAddr) + Offset;
	return NOR_IsEmptyAddress(nor, ActAddress, NumBytesToCheck);
}

nor_err_e NOR_IsEmptySector(nor_t *nor, uint32_t SectorAddr, uint32_t Offset, uint32_t NumBytesToCheck){
	uint32_t ActAddress;

	_SANITY_CHECK(nor);

	ActAddress = (nor->info.u16SectorSize * SectorAddr) + Offset;
	return NOR_IsEmptyAddress(nor, ActAddress, NumBytesToCheck);
}

nor_err_e NOR_IsEmptyBlock(nor_t *nor, uint32_t BlockAddr, uint32_t Offset, uint32_t NumBytesToCheck){
	uint32_t ActAddress;

	_SANITY_CHECK(nor);

	ActAddress = (nor->info.u32BlockSize * BlockAddr) + Offset;
	return NOR_IsEmptyAddress(nor, ActAddress, NumBytesToCheck);
}

nor_err_e NOR_WriteBytes(nor_t *nor, uint8_t *pBuffer, uint32_t WriteAddr, uint32_t NumBytesToWrite){
	uint8_t WriteCmd[4];
	uint32_t _BytesToWrite;

	_SANITY_CHECK(nor);

	if (NumBytesToWrite == 0){
		NOR_PRINTF("ERROR: Invalid parameters on NOR_WriteBytes\n");
		return NOR_INVALID_PARAMS;
	}
	// TODO check if Address is not grater than the Flash size
	NOR_PRINTF("Writing %d bytes into Address %08X.\n", (uint)NumBytesToWrite, (uint)WriteAddr);
	NOR_PRINTF("Buffer to Write into Flash:\n");
	NOR_PRINTF("====================== Values in HEX ========================");
	for (uint32_t i = 0; i < NumBytesToWrite; i++)
	{
		if (i % 16 == 0)
		{
			NOR_PRINTF("\r\n");
			NOR_PRINTF("0x%08X | ", (uint)(WriteAddr + i));
		}
		NOR_PRINTF("%02X ", pBuffer[i]);
	}
	NOR_PRINTF("\n=============================================================\n");
	do{
		if (((WriteAddr%nor->info.u16PageSize)+NumBytesToWrite) > nor->info.u16PageSize){
			_BytesToWrite = nor->info.u16PageSize;
		}
		else{
			_BytesToWrite = NumBytesToWrite;
		}
		_nor_WriteEnable(nor);
		WriteCmd[0] = NOR_PAGE_PROGRAM;
		WriteCmd[1] = ((WriteAddr >> 16) & 0xFF);
		WriteCmd[2] = ((WriteAddr >> 8) & 0xFF);
		WriteCmd[3] = ((WriteAddr) & 0xFF);
		_nor_cs_assert(nor);
		_nor_spi_tx(nor, WriteCmd, sizeof(WriteCmd));
		_nor_spi_tx(nor, pBuffer, _BytesToWrite);
		_nor_cs_deassert(nor);
		_nor_WaitForWriteEnd(nor, NOR_EXPECT_PAGE_PROG_TIME);
		pBuffer += _BytesToWrite;
		NumBytesToWrite -= _BytesToWrite;
	}while (NumBytesToWrite > 0);
	NOR_PRINTF("Write done.!\n");

	return NOR_OK;
}


nor_err_e NOR_WritePage(nor_t *nor, uint8_t *pBuffer, uint32_t PageAddr, uint32_t Offset, uint32_t NumBytesToWrite){
	uint32_t Address;

	_SANITY_CHECK(nor);

	while (Offset >= nor->info.u16PageSize){
		PageAddr++;
		Offset -= nor->info.u16PageSize;
	}

	Address = (PageAddr * nor->info.u16PageSize) + Offset;
	return NOR_WriteBytes(nor, pBuffer, Address, NumBytesToWrite);
}

nor_err_e NOR_WriteSector(nor_t *nor, uint8_t *pBuffer, uint32_t SectorAddr, uint32_t Offset, uint32_t NumBytesToWrite){
	uint32_t Address;

	_SANITY_CHECK(nor);

	while (Offset >= nor->info.u16SectorSize){
		SectorAddr++;
		Offset -= nor->info.u16SectorSize;
	}

	Address = (SectorAddr * nor->info.u16SectorSize) + Offset;
	return NOR_WriteBytes(nor, pBuffer, Address, NumBytesToWrite);
}

nor_err_e NOR_WriteBlock(nor_t *nor, uint8_t *pBuffer, uint32_t BlockAddr, uint32_t Offset, uint32_t NumBytesToWrite){
	uint32_t Address;

	_SANITY_CHECK(nor);

	while (Offset >= nor->info.u32BlockSize){
		BlockAddr++;
		Offset -= nor->info.u32BlockSize;
	}

	Address = (BlockAddr * nor->info.u32BlockSize) + Offset;
	return NOR_WriteBytes(nor, pBuffer, Address, NumBytesToWrite);
}

nor_err_e NOR_ReadBytes(nor_t *nor, uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead){
	uint8_t ReadCmd[5];
	uint32_t _BytesToRead;
	uint8_t *originalBuffer;
	uint32_t originalNumBytes;

	_SANITY_CHECK(nor);

	if (NumByteToRead == 0){
		return NOR_INVALID_PARAMS;
	}
	// TODO check if Address is not grater than the Flash size

	originalBuffer = pBuffer;
	originalNumBytes = NumByteToRead;
	NOR_PRINTF("Reading %d bytes on the Address %08X.\n", (uint)NumByteToRead, (uint)ReadAddr);
	do{
		if (((ReadAddr%nor->info.u16PageSize)+NumByteToRead) > nor->info.u16PageSize){
			_BytesToRead = nor->info.u16PageSize;
		}
		else{
			_BytesToRead = NumByteToRead;
		}
		ReadCmd[0] = NOR_READ_FAST_DATA;
		ReadCmd[1] = ((ReadAddr >> 16) & 0xFF);
		ReadCmd[2] = ((ReadAddr >> 8) & 0xFF);
		ReadCmd[3] = ((ReadAddr) & 0xFF);
		ReadCmd[4] = 0x00;
		_nor_cs_assert(nor);
		_nor_spi_tx(nor, ReadCmd, sizeof(ReadCmd));
		_nor_spi_rx(nor, pBuffer, _BytesToRead);
		_nor_cs_deassert(nor);
		pBuffer += _BytesToRead;
		NumByteToRead -= _BytesToRead;
	}while(NumByteToRead > 0);

	NOR_PRINTF("Buffer readed from NOR:\n");
	NOR_PRINTF("====================== Values in HEX ========================");
	for (uint32_t i = 0; i < originalNumBytes; i++)
	{
		if (i % 16 == 0)
		{
			NOR_PRINTF("\r\n");
			NOR_PRINTF("0x%08X | ", (uint)(ReadAddr + i));
		}
		NOR_PRINTF("%02X ", originalBuffer[i]);
	}
	NOR_PRINTF("\n=============================================================\n");
	NOR_PRINTF("w25qxx ReadBytes done.\n");

	return NOR_OK;
}

nor_err_e NOR_ReadPage(nor_t *nor, uint8_t *pBuffer, uint32_t PageAddr, uint32_t Offset, uint32_t NumByteToRead){
	uint32_t Address;

	_SANITY_CHECK(nor);

	while (Offset >= nor->info.u16PageSize){
		PageAddr++;
		Offset -= nor->info.u16PageSize;
	}

	Address = (PageAddr * nor->info.u16PageSize) + Offset;
	return NOR_ReadBytes(nor, pBuffer, Address, NumByteToRead);
}

nor_err_e NOR_ReadSector(nor_t *nor, uint8_t *pBuffer, uint32_t SectorAddr, uint32_t Offset, uint32_t NumByteToRead){
	uint32_t Address;

	_SANITY_CHECK(nor);

	while (Offset >= nor->info.u16SectorSize){
		SectorAddr++;
		Offset -= nor->info.u16SectorSize;
	}

	Address = (SectorAddr * nor->info.u16SectorSize) + Offset;
	return NOR_ReadBytes(nor, pBuffer, Address, NumByteToRead);
}

nor_err_e NOR_ReadBlock(nor_t *nor, uint8_t *pBuffer, uint32_t BlockAddr, uint32_t Offset, uint32_t NumByteToRead){
	uint32_t Address;

	_SANITY_CHECK(nor);

	while (Offset >= nor->info.u32BlockSize){
		BlockAddr++;
		Offset -= nor->info.u32BlockSize;
	}

	Address = (BlockAddr * nor->info.u32BlockSize) + Offset;
	return NOR_ReadBytes(nor, pBuffer, Address, NumByteToRead);
}
