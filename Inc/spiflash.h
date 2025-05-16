/*
 * spiflash.h
 *
 *  Created on: Apr 28, 2025
 *      Author: sgord
 */
#include "stm32g4xx.h"

// Global flag or function pointer to track flash DMA completion
extern volatile int flash_dma_busy;


#ifndef SPIFLASH_H_
#define SPIFLASH_H_

#define SPIFLASH_PAGE_SIZE (256)
// Helper: Wait until flash is ready
void spi_test(SPI_HandleTypeDef* hspi);
uint32_t flash_read_status(SPI_HandleTypeDef* hspi);
void flash_read_poll(uint32_t address, void *buffer, uint32_t length, SPI_HandleTypeDef* hspi);
void flash_read_dma(uint32_t address, void *buffer, uint32_t length, SPI_HandleTypeDef* hspi);
void flash_page_program_poll(uint32_t address, const void *data, uint32_t length, SPI_HandleTypeDef* hspi);
void flash_page_program_dma(uint32_t address, const void *data, uint32_t length, SPI_HandleTypeDef* hspi);
void flash_page_program_dma_async(uint32_t address, void *data, uint32_t payload_length, SPI_HandleTypeDef* hspi);
void flash_erase_sector(uint32_t address, SPI_HandleTypeDef* hspi);
void flash_erase_chip(SPI_HandleTypeDef* hspi);
uint32_t flash_read_jedec_id(SPI_HandleTypeDef* hspi);
void flash_read_uuid(SPI_HandleTypeDef* hspi);

#endif /* SPIFLASH_H_ */
