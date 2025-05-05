/*
 * trace_spiflash.c
 *
 *  Created on: May 2, 2025
 *      Author: sgord
 */

#include "trace_spiflash.h"
#include "spiflash.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>


void pack_trace_record(trace_object_t* trace_obj);
void flash_write_header(trace_object_t* trace_obj);
void flash_write_page(trace_object_t* trace_obj);
uint32_t analyze_page(trace_header_t* header);



uint32_t analyze_page(trace_header_t* header)
{
	uint8_t* headerb = (uint8_t*)header;
	// look for signiture
	if ( header->id1 == TRACE_ID_1 &&
		 header->id2 == TRACE_ID_2 &&
		 header->id3 == TRACE_ID_3 &&
		 header->id4 == TRACE_ID_4 &&
		 header->record_len_blocks < 65536
		 // TBD check record_len_b and checksum
		 )
	{
		return PAGE_TRACE_HEADER;
	}

	int allff = 1;
	for (int i = 0; i< sizeof(trace_header_t); ++i)
	{
		if (headerb[i] != 0xff)
		{
			allff = 0;
			break;
		}
	}

	if (allff == 1)
		return PAGE_EMPTY;

	return PAGE_OTHER;


}


uint32_t trace_init(trace_object_t* trace_obj, size_t alloc_size, SPI_HandleTypeDef* hspi)
{
	trace_header_t buffer;
	uint32_t addr=0;

	trace_obj->num_tracevals = ( alloc_size - sizeof (trace_obj) ) / sizeof(trace_ptr_len_pair_t);

	trace_obj->stat = TRACE_STAT_UNINITIALIZED;
	// test flash
	if ( flash_read_jedec_id(hspi) != 0xEF4018 )
	{
		printf("flash failed diagnostic\n");
		return TRACE_ERROR_SPIFLASH_FAILED;
	}

	// lengh must be multiple of blocksize
	uint16_t modval = trace_obj->trace_len_b % SPIFLASH_PAGE_SIZE;
	if (modval != 0) trace_obj->trace_len_b = trace_obj->trace_len_b + (SPIFLASH_PAGE_SIZE - modval);
	if (trace_obj->trace_len_b == 0) trace_obj->trace_len_b = SPIFLASH_PAGE_SIZE;

	// record length must be multiple of blocksize
	modval = trace_obj->trace_record_len_b % SPIFLASH_PAGE_SIZE;
	if (modval != 0) trace_obj->trace_record_len_b = trace_obj->trace_record_len_b + (SPIFLASH_PAGE_SIZE - modval);
	if (trace_obj->trace_record_len_b == 0) trace_obj->trace_record_len_b = SPIFLASH_PAGE_SIZE;


	// lengh must be multiple of blocksize
	if (trace_obj->flash_len_b % SPIFLASH_PAGE_SIZE != 0)
	{
		printf("invalid flash size\n");
		return TRACE_ERROR_INVALID_PARAM;
	}

	do
	{
		// find flash start address
		flash_read_dma( addr, &buffer, sizeof(trace_header_t), hspi );
		uint32_t page_type = analyze_page(&buffer);
		if (page_type == PAGE_TRACE_HEADER)
		{
			addr += SPIFLASH_PAGE_SIZE;
		}
		else if (page_type == PAGE_EMPTY)
		{
			break;
		}
		else
		{
			printf("Bad flash format\n");
			return TRACE_ERROR_FORMAT;
		}

	} while (addr < trace_obj->flash_len_b);

	if (addr >= trace_obj->flash_len_b)
	{
		printf("flash is full\n");
		return TRACE_ERROR_FULL;
	}


	// Make sure desired trace size can fit in available flash
	uint32_t flash_avail = trace_obj->flash_len_b - addr;
	if (flash_avail < trace_obj->trace_len_b)
	{
		printf("not enough space on flash\n");
		return TRACE_ERROR_NOT_ENOUGH_SPACE;
	}


	// prepare for tracing
	trace_obj->amount_written_b = 0;
	trace_obj->write_ptr = trace_obj->buffer_start;
	trace_obj->read_ptr = trace_obj->buffer_start;
	trace_obj->flash_cur_addr = addr;
	trace_obj->flash_start_addr = addr;
	trace_obj->flash_cur_addr = addr;

	trace_obj->stat = TRACE_STAT_READY;
	//trace_obj->num_tracevals = sizeof(trace_obj->tracevals) / sizeof(trace_ptr_len_pair_t) ;



	// {TBD} trigger writing of header
	flash_write_header(trace_obj);



	return addr;
}

void pack_trace_record(trace_object_t* trace_obj)
{
	uint8_t* init_write = trace_obj->write_ptr;
	for (int i = 0; i< trace_obj->num_tracevals; ++i)
		memcpy( trace_obj->write_ptr, trace_obj->tracevals[i].ptr, trace_obj->tracevals[i].len_b  );

	trace_obj->write_ptr += trace_obj->trace_record_len_b;
//	assert(trace_obj->write_ptr == init_write + trace_obj->trace_record_len_b );


}


void trace(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{

	// store in circular buffer

	// read trace data and pack into record
	pack_trace_record(trace_obj);
	trace_obj->amount_written_b += trace_obj->trace_record_len_b;


	// If we just finished a page, trigger flash storage
	if ( (trace_obj->amount_written_b > 0) && ( (trace_obj->amount_written_b % SPIFLASH_PAGE_SIZE) == 0) )
	{
		flash_write_page(trace_obj);
	}

	// done?

}

void trace_end(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{
}


void flash_write_header(trace_object_t* trace_obj)
{

}

void flash_write_page(trace_object_t* trace_obj)
{

}


/*
uint32_t trace_find_empty_page
bool trace_init(trace_record_t* rec); // write header
uint32_t trace_cont(rec);  // continue tracing
*/




