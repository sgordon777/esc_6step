/*
 * trace_spiflash.h
 *
 *  Created on: May 2, 2025
 *      Author: sgord
 */
#ifndef TRACE_SPIFLASH_H_
#define TRACE_SPIFLASH_H_

#include "stm32g4xx.h"


//#define PAGE_SIZE 			(256)
#define TRACE_LIB_VER (1)
#define PAGE_SIZE_LOG 		(8)
#define TRACE_ERROR_BASE 	(1<<28)
#define TRACE_ERROR_FORMAT 	(TRACE_ERROR_BASE + 1)
#define TRACE_ERROR_FULL 	(TRACE_ERROR_BASE + 2)
#define TRACE_ERROR_SPIFLASH_FAILED (TRACE_ERROR_BASE + 3)
#define TRACE_ERROR_NOT_ENOUGH_SPACE 	(TRACE_ERROR_BASE + 4)
#define TRACE_ERROR_INVALID_PARAM  (TRACE_ERROR_BASE + 5)
#define TRACE_ERROR_INVALID_RECORD_LEN  (TRACE_ERROR_BASE + 6)

#define FILENAME_MAX_LEN (128)


#define PAGE_EMPTY (0)
#define PAGE_TRACE_HEADER (1)
#define PAGE_OTHER (2)

#define TRACE_STAT_UNINITIALIZED (0)
#define TRACE_STAT_READY (1)
#define TRACE_STAT_STREAMING (2)
#define TRACE_STAT_FINISHING (3)
#define TRACE_STAT_DONE (4)


/*
 *
 *
 * wr,0,28, 85,83,69,74, 69,73,68,83, 85,79,52,68, 78,73,83,82, 4,0,0,0, 0,4,0,0, 0,0,0,0
*/

#define TRACE_ID_1 (0x4a455355)
#define TRACE_ID_2 (0x53444945)
#define TRACE_ID_3 (0x44344f55)
#define TRACE_ID_4 (0x5253494e)

typedef struct {
	uint32_t id1;
	uint32_t id2;
	uint32_t id3;
	uint32_t id4;
	uint32_t trace_file_len_p;
	uint32_t trace_file_len_b;
	uint16_t checksum;
	uint16_t header_len_b;
	uint32_t ver;
	char trace_filename[FILENAME_MAX_LEN];
} trace_header_t;


typedef struct {
	void* ptr;
	int16_t len_b;
} trace_ptr_len_pair_t;

typedef struct {
	// UN INITIALIZED
	uint8_t* write_ptr;					// circular buffer write pointer: data will be read from here
	uint8_t* read_ptr;					// circular buffer read pointer: data will be read from here
	uint32_t flash_start_addr;			// start flash address (header will be written here)
	uint32_t flash_cur_addr;			// current flash pointer
	uint16_t num_tracevals;				// number of tracevals entries
	uint32_t amount_written_b;			// number of bytes written
	uint32_t amount_read_b;				// number of bytes read

	// INITIALIZED
	uint16_t stat;						// state of trace system
	uint8_t* buffer_start;				// start address of circular buffer
	uint16_t buffer_len_b;				// size of circular buffer in bytes
	uint16_t trace_entry_len_b; 		// size of trace file
	uint32_t trace_file_len_b;			// desired size of tracebuffer in bytes
	uint32_t flash_len_b;  				// size of flash in bytes
	trace_ptr_len_pair_t tracevals[];	// variable length of address/size pairs to trace
} trace_object_t;


uint32_t trace_init(trace_object_t* trace_obj, size_t alloc_size, char* filename, SPI_HandleTypeDef* hspi);
void trace(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi);
void trace_end(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi);



#endif /* TRACE_SPIFLASH_H_ */
