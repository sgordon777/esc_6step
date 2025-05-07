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


void pack_trace_entry(trace_object_t* trace_obj);
void flash_write_header(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi);
void flash_write_page(trace_object_t* trace_obj);
void print_trace_object(const trace_object_t *obj);
void print_trace_header(const trace_header_t *hdr);
uint32_t update_circ_ptr(uint32_t* curval, uint32_t inc, uint32_t* start_addr, uint32_t size);


uint32_t analyze_page(trace_header_t* header);



uint32_t analyze_page(trace_header_t* header)
{
	uint8_t* headerb = (uint8_t*)header;
	// look for signiture
	if ( header->id1 == TRACE_ID_1 &&
		 header->id2 == TRACE_ID_2 &&
		 header->id3 == TRACE_ID_3 &&
		 header->id4 == TRACE_ID_4 &&
		 header->trace_file_len_p < 65536 &&
		 header->trace_file_len_p > 0 &&
		 header->trace_file_len_p * SPIFLASH_PAGE_SIZE >= header->trace_file_len_b &&
		 header->trace_file_len_p * (SPIFLASH_PAGE_SIZE-1) < header->trace_file_len_b
		 )
	{
		return PAGE_TRACE_HEADER;
	}

	// TBD this check should look at the entire page, not just the header
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
	trace_header_t header;
	uint32_t addr=0;

	trace_obj->num_tracevals = ( alloc_size - sizeof (trace_obj) ) / sizeof(trace_ptr_len_pair_t);

	trace_obj->stat = TRACE_STAT_UNINITIALIZED;
	// test flash
	if ( flash_read_jedec_id(hspi) != 0xEF4018 )
	{
		printf("flash failed diagnostic\n");
		return TRACE_ERROR_SPIFLASH_FAILED;
	}

	// trace entry lengh must be multiple of pagesize
	uint16_t modval = trace_obj->trace_file_len_b % SPIFLASH_PAGE_SIZE;
	if (modval != 0) trace_obj->trace_file_len_b = trace_obj->trace_file_len_b + (SPIFLASH_PAGE_SIZE - modval);
	if (trace_obj->trace_file_len_b == 0) trace_obj->trace_file_len_b = SPIFLASH_PAGE_SIZE;

	// record length must be multiple of pagesize
	modval = SPIFLASH_PAGE_SIZE % trace_obj->trace_entry_len_b;
	if (modval != 0)
	{
		printf("trace entry len must be an integer divisor of %d\n", SPIFLASH_PAGE_SIZE);
		return TRACE_ERROR_INVALID_PARAM;
	}


	// lengh must be multiple of pagesize
	if (trace_obj->flash_len_b % SPIFLASH_PAGE_SIZE != 0)
	{
		printf("invalid flash size\n");
		return TRACE_ERROR_INVALID_PARAM;
	}

	printf("\nSearching flash for empty page....\n", addr);
	do
	{
		// find flash start address
		flash_read_dma( addr, &header, sizeof(trace_header_t), hspi );
		uint32_t page_type = analyze_page(&header);
		if (page_type == PAGE_TRACE_HEADER)
		{
			printf("--->Found file @%.8X, L=%.8X\n", addr, ( header.trace_file_len_p) * SPIFLASH_PAGE_SIZE);
			addr = addr + ( 1 + header.trace_file_len_p) * SPIFLASH_PAGE_SIZE;
		}
		else if (page_type == PAGE_EMPTY)
		{
			printf("--->Found empty page @%.8X\n", addr);
			break;
		}
		else
		{
			printf("--->Bad flash format\n");
			return TRACE_ERROR_FORMAT;
		}


	} while (addr < trace_obj->flash_len_b);

	if (addr >= trace_obj->flash_len_b)
	{
		printf("--->flash is full\n");
		return TRACE_ERROR_FULL;
	}


	// Make sure desired trace size can fit in available flash
	uint32_t flash_avail = trace_obj->flash_len_b - addr;
	if (flash_avail < trace_obj->trace_file_len_b)
	{
		printf("Error: Not enough space on flash\n");
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
	flash_write_header(trace_obj, hspi);



	return addr;
}

void pack_trace_entry(trace_object_t* trace_obj)
{
	uint8_t* init_write = trace_obj->write_ptr;
	for (int i = 0; i< trace_obj->num_tracevals; ++i)
		memcpy( trace_obj->write_ptr, trace_obj->tracevals[i].ptr, trace_obj->tracevals[i].len_b  );

	trace_obj->write_ptr = update_circ_ptr(trace_obj->write_ptr, trace_obj->trace_entry_len_b, trace_obj->buffer_start, trace_obj->trace_file_len_b);
	trace_obj->amount_written_b += trace_obj->trace_entry_len_b;


}

void trace(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{

	// store in circular buffer

	// read trace data and pack into record
	pack_trace_entry(trace_obj);


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


void flash_write_header(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{
	assert(trace_obj->trace_file_len_b % SPIFLASH_PAGE_SIZE  == 0);
	trace_header_t hdr = {TRACE_ID_1, TRACE_ID_2, TRACE_ID_3, TRACE_ID_4,
			 trace_obj->trace_file_len_b/SPIFLASH_PAGE_SIZE, trace_obj->trace_file_len_b, 0, sizeof(trace_header_t), 1};

	while (flash_dma_busy != 0) {}
	print_trace_header(&hdr);
	print_trace_object(trace_obj);

	flash_page_program_dma_async(trace_obj->flash_cur_addr, &hdr, sizeof(trace_header_t), hspi);

}

void flash_write_page(trace_object_t* trace_obj)
{

}

uint32_t update_circ_ptr(uint32_t* curval, uint32_t inc, uint32_t* start_addr, uint32_t size)
{
	curval += inc;
	if (curval >= (start_addr + size)) curval -= size;
}


void print_trace_header(const trace_header_t *hdr) {
    if (!hdr) return;

    printf("=== Trace Header Report ===\n");
    printf("ID1                       : 0x%08X\n", hdr->id1);
    printf("ID2                       : 0x%08X\n", hdr->id2);
    printf("ID3                       : 0x%08X\n", hdr->id3);
    printf("ID4                       : 0x%08X\n", hdr->id4);
    printf("Tracefile Length (pages)  : %u pages\n", hdr->trace_file_len_p);
    printf("Tracefile Length (blocks) : %u bytes\n",  hdr->trace_file_len_b);
    printf("Checksum                  : 0x%08X\n", hdr->checksum);
    printf("Header Length             : %u bytes\n", hdr->header_len_b);
    printf("Header Version            : %u\n", hdr->header_ver);
    printf("============================\n");
}

void print_trace_object(const trace_object_t *obj) {
    if (!obj) return;

    printf("=== trace_object_t ===\n");
    printf("write_ptr                : %p\n", obj->write_ptr);
    printf("read_ptr                 : %p\n", obj->read_ptr);
    printf("flash_start_addr         : 0x%08X\n", obj->flash_start_addr);
    printf("flash_cur_addr           : 0x%08X\n", obj->flash_cur_addr);
    printf("stat                     : 0x%04X\n", obj->stat);
    printf("num_tracevals            : %u\n", obj->num_tracevals);
    printf("amount_written_b         : %u\n", obj->amount_written_b);
    printf("buffer_start             : %p\n", obj->buffer_start);
    printf("buffer_len_b             : %u\n", obj->buffer_len_b);
    printf("trace_entry_len_b        : %u\n", obj->trace_entry_len_b);
    printf("trace_file_len_b         : %u\n", obj->trace_file_len_b);
    printf("flash_len_b              : %u\n", obj->flash_len_b);

    for (uint16_t i = 0; i < obj->num_tracevals; ++i) {
        printf("tracevals[%u]     : addr=%p, len=%u\n",
               i, obj->tracevals[i].ptr, obj->tracevals[i].len_b);
    }

    printf("=======================\n");
}


/*
uint32_t trace_find_empty_page
bool trace_init(trace_entry_t* rec); // write header
uint32_t trace_cont(rec);  // continue tracing
*/




