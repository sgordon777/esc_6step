/*
 * trace_spiflash.c
 *
 *  Created on: May 2, 2025
 *      Author: sgord
 */



/*
uint32_t trace_find_empty_page
bool trace_init(trace_record_t* rec); // write header
uint32_t trace_cont(rec);  // continue tracing
*/


#define TRACE_ID_1 (0x12dac92a)
#define TRACE_ID_2 (0x98caf732)
#define TRACE_ID_3 (0x0c7defa1)
#define TRACE_ID_4 (0xcab7f294)

typedef struct {
	uint32_t id1;
	uint32_t id2;
	uint32_t id3;
	uint32_t id4;
	uint32_t record_len_sectors;
	uint32_t record_len_bytes;
	uint32_t checksum;
} trace_header;

typedef struct {
	uint16_t encpos;
	int16_t analog_val;
	uint8_t commutation_step;
	uint8_t mode;
} trace_record;


