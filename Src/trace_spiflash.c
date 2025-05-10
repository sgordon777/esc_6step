/*
 * trace_spiflash.c
 *
 *  Created on: May 2, 2025
 *      Author: sgord
 *
 *
*
 *
 */

#include "trace_spiflash.h"
#include "spiflash.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <string.h>


void pack_trace_entry(trace_object_t* trace_obj);
void flash_write_header(trace_object_t* trace_obj, char* filename, SPI_HandleTypeDef* hspi);
void flash_write_page(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi);
void print_trace_object(const trace_object_t *obj);
void print_trace_header(const trace_header_t *hdr);
void calc_amount_data_space(trace_object_t* trace_obj, uint32_t* data, uint32_t* space);
void update_circ_ptr(uint8_t** curval, uint32_t inc, uint8_t* start_addr, uint32_t size);
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


uint32_t trace_init(trace_object_t* trace_obj, char* filename, SPI_HandleTypeDef* hspi)
{
	trace_header_t header;
	uint32_t addr=0;

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
			char* filename = "not available before version 1";
			if (header.ver >= 1) filename = header.trace_filename;
			printf("--->Found file ver=%d, addr=%.8X, len(b)=%.8X, filename=%s\n", header.ver, addr, ( header.trace_file_len_p) * SPIFLASH_PAGE_SIZE, filename);
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
	trace_obj->amount_read_b = 0;
	trace_obj->write_ptr = trace_obj->buffer_start;
	trace_obj->read_ptr = trace_obj->buffer_start;
	trace_obj->flash_cur_addr = addr;
	trace_obj->flash_start_addr = addr;
	trace_obj->flash_cur_addr = addr;

	// {TBD} trigger writing of header
	flash_write_header(trace_obj, filename, hspi);


	while (flash_dma_busy != 0) {}
	trace_obj->stat = TRACE_STAT_STREAMING;
	//trace_obj->num_tracevals = sizeof(trace_obj->tracevals) / sizeof(trace_ptr_len_pair_t) ;
	HAL_Delay(100);

	return addr;
}

void pack_trace_entry(trace_object_t* trace_obj)
{
	uint8_t* init_write = trace_obj->write_ptr;
	for (int i = 0; i< trace_obj->num_tracevals; ++i)
	{
		memcpy( trace_obj->write_ptr, trace_obj->tracevals[i].ptr, trace_obj->tracevals[i].len_b  );
		update_circ_ptr(&trace_obj->write_ptr, trace_obj->tracevals[i].len_b , trace_obj->buffer_start, trace_obj->buffer_len_b);
	}
	trace_obj->amount_written_b += trace_obj->trace_entry_len_b;
}

void trace(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{
	if ( trace_obj->stat == TRACE_STAT_DONE || trace_obj->stat == TRACE_STAT_UNINITIALIZED)
		return;

	// get data/space
	uint32_t space, data;
	calc_amount_data_space(trace_obj, &data, &space);


	// write entry
	if (trace_obj->stat == TRACE_STAT_STREAMING)
	{
		// out of buffer space?
		assert (space > trace_obj->trace_entry_len_b);

		// read trace data and pack into record
		pack_trace_entry(trace_obj);

		// done?
		if (trace_obj->amount_written_b >= trace_obj->trace_file_len_b)
			trace_obj->stat = TRACE_STAT_FINISHING;

		//printf("stat=%u, bufspc=%u, bufdata=%u, written=%u, read=%u, writeptr=%.8X, readptr=%.8x\n",
		//trace_obj->stat, space, data, trace_obj->amount_written_b, trace_obj->amount_read_b, trace_obj->write_ptr, trace_obj->read_ptr);

	}

	if (trace_obj->amount_read_b >= trace_obj->trace_file_len_b)
	{
		// Finished writing flash
		if ( flash_dma_busy ) return;
		printf("--->Tracing completed\n");
		trace_obj->stat = TRACE_STAT_DONE;
		return;
	}

	// manage flash writing
	if ( flash_dma_busy ) return;
	if (
			(trace_obj->amount_written_b > 0) &&
			((trace_obj->amount_written_b & 0x00000007f) == 0) &&
			(data >= SPIFLASH_PAGE_SIZE)
		)
	{
		//printf("--->write_chunk\n");

		flash_write_page(trace_obj, hspi);
	}


}

void trace_end(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{
}


void flash_write_header(trace_object_t* trace_obj, char* filename, SPI_HandleTypeDef* hspi)
{
	assert(trace_obj->trace_file_len_b % SPIFLASH_PAGE_SIZE  == 0);
	trace_header_t hdr = {TRACE_ID_1, TRACE_ID_2, TRACE_ID_3, TRACE_ID_4,
			 trace_obj->trace_file_len_b/SPIFLASH_PAGE_SIZE, trace_obj->trace_file_len_b, 0, sizeof(trace_header_t), 1};
	hdr.ver = TRACE_LIB_VER;
	strncpy(hdr.trace_filename, filename, FILENAME_MAX_LEN);
	hdr.trace_filename[FILENAME_MAX_LEN-1] = 0; // ensure null termination

	while (flash_dma_busy != 0) {}

	print_trace_header(&hdr);
	print_trace_object(trace_obj);

	flash_page_program_dma_async(trace_obj->flash_cur_addr, &hdr, sizeof(trace_header_t), hspi);
	trace_obj->flash_cur_addr += SPIFLASH_PAGE_SIZE;


}

void flash_write_page(trace_object_t* trace_obj, SPI_HandleTypeDef* hspi)
{
	assert(flash_dma_busy == 0);
	flash_page_program_dma_async(trace_obj->flash_cur_addr, trace_obj->read_ptr, SPIFLASH_PAGE_SIZE, hspi);
	update_circ_ptr(&trace_obj->read_ptr, SPIFLASH_PAGE_SIZE, trace_obj->buffer_start, trace_obj->buffer_len_b );
	trace_obj->flash_cur_addr += SPIFLASH_PAGE_SIZE;
	trace_obj->amount_read_b += SPIFLASH_PAGE_SIZE;
}


//#define CIRC_WRITE(cur, curpinc, startpsz, sz) cur = (curpinc) < (startpsz) ? (curpinc) : (curpinc - sz)
void update_circ_ptr(uint8_t** curval, uint32_t inc, uint8_t* start_addr, uint32_t size)
{
	*curval += inc;
	if (*curval >= (start_addr + size)) *curval -= size;
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
    printf("Header Version            : %u\n", hdr->ver);
    printf("Trace Filename            : %s\n", hdr->trace_filename);
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
    printf("amount_read_b            : %u\n", obj->amount_read_b);
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

void calc_amount_data_space(trace_object_t* trace_obj, uint32_t* data, uint32_t* space)
{
	*data = (trace_obj->write_ptr - trace_obj->read_ptr);
	if (*data & 0x80000000) *data += trace_obj->buffer_len_b;
	*space = trace_obj->buffer_len_b - *data - 1;
}


/*
uint32_t trace_find_empty_page
bool trace_init(trace_entry_t* rec); // write header
uint32_t trace_cont(rec);  // continue tracing
*/




