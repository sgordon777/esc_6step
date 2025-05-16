
#include "spiflash.h"
#include <stdio.h>
#include "string.h"
#include "stm32g4xx_ll_gpio.h"

static uint32_t flash_wait_for_ready(SPI_HandleTypeDef* hspi);
static void flash_write_enable(SPI_HandleTypeDef* hspi);


extern SPI_HandleTypeDef hspi3;

#define CPU_CLK (170E6)

#define FLASH_CMD_WRITE_ENABLE   0x06
#define FLASH_CMD_WRITE_DISABLE  0x04
#define FLASH_CMD_READ_DATA       0x03
#define FLASH_CMD_PAGE_PROGRAM    0x02
#define FLASH_CMD_SECTOR_ERASE    0x20
#define FLASH_CMD_READ_STATUS     0x05
#define FLASH_CMD_READ_STATUS2     0x35
#define FLASH_CMD_READ_STATUS3     0x15
#define FLASH_CMD_CHIP_ERASE      0xC7
#define FLASH_CMD_UNIQUE_ID       0x4B
#define FLASH_CMD_JEDEC           0x9F

// Replace with your Flash's WIP (Write In Progress) bit mask
#define FLASH_WIP_BIT             0x01

// Chip Select control macros
#define FLASH_CS_LOW()    LL_GPIO_ResetOutputPin(GPIOB, GPIO_PIN_4)  // <-- Adjust your CS pin
#define FLASH_CS_HIGH()   LL_GPIO_SetOutputPin(GPIOB, GPIO_PIN_4)

// Helper: Wait until flash is ready
uint32_t flash_read_status(SPI_HandleTypeDef* hspi)
{
    uint8_t cmd;
    uint8_t status1;
    uint8_t status2;
	uint8_t status3;
	uint32_t status;

    cmd = FLASH_CMD_READ_STATUS;
    status1 = 0;
    status2 = 0;
	status = 0;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &status1, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    cmd = FLASH_CMD_READ_STATUS2;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &status2, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    cmd = FLASH_CMD_READ_STATUS3;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &status3, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    status = 0;
    status |= ( (uint32_t)(status3) << 16 );
    status |= ( (uint32_t)(status2) << 8 );
    status |= ( (uint32_t)(status1) );

	return (status);


}

// Helper: Wait until flash is ready
static uint32_t flash_wait_for_ready(SPI_HandleTypeDef* hspi)
{
    uint8_t cmd = FLASH_CMD_READ_STATUS;
    uint8_t status = 0;

    uint32_t t0 = DWT->CYCCNT;
    do {
        FLASH_CS_LOW();
        HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(hspi, &status, 1, HAL_MAX_DELAY);
        FLASH_CS_HIGH();
    } while (status & FLASH_WIP_BIT);

    uint32_t tt = DWT->CYCCNT - t0;
    return tt;
}

// Helper: Send Write Enable
static void flash_write_enable(SPI_HandleTypeDef* hspi)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();
}



void flash_erase_chip(SPI_HandleTypeDef* hspi)
{
    flash_write_enable(hspi);
    uint8_t cmd = FLASH_CMD_CHIP_ERASE;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();
    uint32_t tt_ready = flash_wait_for_ready(hspi);
#ifdef SPIFLASH_VERBOSE
    printf("flash_erase_chip: time_waited=%.6fs \n", (float)tt_ready / (float)CPU_CLK);
#endif

}

// ssssssssssssssss
// Public: Read from flash
void flash_read_poll(uint32_t address, void *buffer, uint32_t length, SPI_HandleTypeDef* hspi)
{
	// throughput ~2/5 theoretical maximum, 4Mbps on 10.24Mhz spi link
    uint8_t cmd[4];
    uint8_t dummy[256] = {0}; // dma

#ifdef SPIFLASH_VERBOSE
    uint32_t t0 = DWT->CYCCNT;
#endif
    cmd[0] = FLASH_CMD_READ_DATA;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, buffer, length, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

#ifdef SPIFLASH_VERBOSE
    uint32_t tt = DWT->CYCCNT - t0;
    printf("flash_read: %.6fs, waitready: %.6fs \n",
    		(float)tt / (float)CPU_CLK);
#endif

}

volatile uint8_t flash_dma_done = 0;  // Global or static flag

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi3) {
        flash_dma_done = 1;
    }
}

void flash_read_dma(uint32_t address, void *rx_buffer, uint32_t length, SPI_HandleTypeDef* hspi)
{
	// throughput nearly 75% of theoretical max (7.64Mbps on 10.24Mhz SPI)
#ifdef SPIFLASH_VERBOSE
	uint32_t t0 = DWT->CYCCNT;
#endif

    uint8_t cmd[4] = {
        FLASH_CMD_READ_DATA,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address >> 0)
    };

    uint8_t dummy[length];  // TX dummy buffer to clock in data

    FLASH_CS_LOW();

    // Send command using polling
    HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);

    flash_dma_done = 0;

    // Start DMA read
    HAL_SPI_TransmitReceive_DMA(hspi, dummy, rx_buffer, length);

    // Wait until DMA completes
    while (!flash_dma_done) {
        __NOP(); // Idle or yield
    }

    FLASH_CS_HIGH();

#ifdef SPIFLASH_VERBOSE
    uint32_t tt = DWT->CYCCNT - t0;
    printf("flash_read_DMA, addr=%.8X, elapsed=: %.6fs \n",
    		address, (float)tt / (float)CPU_CLK);
#endif
}

void flash_read_dma_ready(uint32_t address, void *buffer, uint32_t length, SPI_HandleTypeDef *hspi)
{
    uint8_t cmd[4];
    static uint8_t dummy_tx[256]; // make sure this is >= max transfer size
    if (length > sizeof(dummy_tx)) {
        // truncate to safe size or handle error
        length = sizeof(dummy_tx);
    }

    cmd[0] = FLASH_CMD_READ_DATA;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();

    // Send the 4-byte read command over blocking SPI
    HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);

    // Start DMA transfer (dummy bytes TX, real data RX)
    HAL_SPI_TransmitReceive_DMA(hspi, dummy_tx, buffer, length);

    // Wait for DMA to complete
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {
        // optionally add a timeout here
    }

    FLASH_CS_HIGH();
}

// Public: Program one page (up to 256 bytes)
void flash_page_program_poll(uint32_t address, const void *data, uint32_t length, SPI_HandleTypeDef* hspi)
{
	// throughput 3MBb/s, major limiting factor is the ~0.5 ms per page delay
	// using DMA doesn't affect thoughput (but reduces CPU load)
    if (length > SPIFLASH_PAGE_SIZE) return;  // Can't program more than a page
#ifdef SPIFLASH_VERBOSE
    uint32_t t0 = DWT->CYCCNT;
#endif

    flash_write_enable(hspi);

    uint8_t cmd[4];
    cmd[0] = FLASH_CMD_PAGE_PROGRAM;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, (uint8_t *)data, length, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    uint32_t tt_ready = flash_wait_for_ready(hspi);
#ifdef SPIFLASH_VERBOSE
    uint32_t tt = DWT->CYCCNT - t0;
    printf("flash_page_write_poll @0x%.8X total: %.6fs, time_wait=%.6fs \n", address, (float)tt / (float)CPU_CLK, (float)tt_ready / (float)CPU_CLK);
#endif
}


void flash_page_program_dma(uint32_t address, const void *data, uint32_t length, SPI_HandleTypeDef* hspi)
{
#ifdef SPIFLASH_VERBOSE
    uint32_t t0 = DWT->CYCCNT;
#endif
    if (length > SPIFLASH_PAGE_SIZE) return;  // Max 256 bytes per page

    // Command + 24-bit address
    uint8_t cmd[4] = {
        FLASH_CMD_PAGE_PROGRAM,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address >> 0)
    };

    static uint8_t tx_buf[SPIFLASH_PAGE_SIZE + 4];  // static to persist during DMA
    memcpy(tx_buf, cmd, 4);
    memcpy(&tx_buf[4], data, length);

    flash_write_enable(&hspi3);  // Send 0x06 Write Enable

    FLASH_CS_LOW();

    // Start DMA transmit: 4-byte command + address + data payload
    HAL_SPI_Transmit_DMA(hspi, tx_buf, length + 4);

    // Wait for completion
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY);

    FLASH_CS_HIGH();

    // Wait for the write to finish (poll WIP bit)
    uint32_t tt_ready = flash_wait_for_ready(&hspi3);
#ifdef SPIFLASH_VERBOSE
    uint32_t tt = DWT->CYCCNT - t0;
    printf("flash_page_write_dma @0x%.8X total: %.6fs, time_wait=%.6fs \n", address, (float)tt / (float)CPU_CLK, (float)tt_ready / (float)CPU_CLK);
#endif
}


// Global flag or function pointer to track flash DMA completion
volatile int flash_dma_busy = 0;

// You can optionally point this to your own handler
__attribute__((weak)) void flash_dma_done_handler(void)
{
}


// Call this to start the flash program
// length: payload lenth. The array itself is sized length+4
void flash_page_program_dma_async(uint32_t address, void *data, uint32_t payload_length, SPI_HandleTypeDef* hspi)
{
    if (payload_length > SPIFLASH_PAGE_SIZE) return;

    uint8_t* ptr_data = data;
    ptr_data[0] = FLASH_CMD_PAGE_PROGRAM;
    ptr_data[1] = address >> 16;
    ptr_data[2] = address >> 8;
    ptr_data[3] = address;

    flash_write_enable(hspi);  // 21.0529us, 6.4588us after removing wait_ready

    FLASH_CS_LOW();
    flash_dma_busy = 1;  // Mark transfer in progress

    HAL_SPI_Transmit_DMA(hspi, ptr_data, payload_length + 4);   // 3.9 us
}


// Hook into HAL's DMA complete callback
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3 && flash_dma_busy) {
        flash_dma_busy = 0;

        FLASH_CS_HIGH();

        // Optionally wait for flash WIP here, or defer
        flash_wait_for_ready(hspi);

        // Call user-defined handler
        flash_dma_done_handler();
    }
}
/********************************************************************************************************************************/


// Public: Erase 4K sector
void flash_erase_sector(uint32_t address, SPI_HandleTypeDef* hspi)
{
    flash_write_enable(hspi);

    uint8_t cmd[4];
    cmd[0] = FLASH_CMD_SECTOR_ERASE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, cmd, 4, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    uint32_t tt_ready = flash_wait_for_ready(hspi);
    printf("flash_erase_sector#%d: time_waited=%.6fs \n", address, (float)tt_ready / (float)CPU_CLK);
}

uint32_t flash_read_jedec_id(SPI_HandleTypeDef* hspi)
{
    uint8_t cmd = FLASH_CMD_JEDEC;
    uint8_t id[3] = {0};
    uint32_t devid = 0;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, id, 3, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

#ifdef SPIFLASH_VERBOSE
    printf("Flash JEDEC ID: %02X %02X %02X\n", id[0], id[1], id[2]);
#endif

    devid |= ( (uint32_t)(id[0]) << 16 );
    devid |= ( (uint32_t)(id[1]) << 8 );
    devid |= ( (uint32_t)(id[2]) );

    return devid;
}

void flash_read_uuid(SPI_HandleTypeDef* hspi)
{
    uint8_t cmd = FLASH_CMD_UNIQUE_ID;
    uint8_t dummy[4] = {0};
    uint8_t id[8] = {0};

    FLASH_CS_LOW();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, dummy, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, id, 8, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    printf("Flash UUID: %02X %02X %02X %02X %02X %02X %02X %02X\n",
    					id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
}





void spi_test(SPI_HandleTypeDef* hspi)
{

	//
	//printf("eraseing chip\n");
	//flash_erase_chip(&hspi3);

	printf("getting flash SR\n");
	uint32_t flash_stat = flash_read_status(hspi);
	printf ("flash_SR = %.8X\n", flash_stat);
	printf("reading flash jedec ID\n");
	flash_read_jedec_id(hspi);
	printf("reading flash UUID\n");
	flash_read_uuid(hspi);

	// program flash
	printf("programming flash\n");
	//  trace_record_header rec1 = {TRACE_ID_1, TRACE_ID_2, TRACE_ID_3, TRACE_ID_4, 1, 512, 0x7399feed};
	const char* msg = "zyxw vurka wwe abcdef";
	flash_page_program_poll(0x00000600, (const uint8_t*)msg, 256, hspi);

	for (int i=0; i<16; ++i)
	{
		  printf("reading flash %.8X\n", 256*i);
		  uint8_t fbuffer[256];
		  flash_read_dma(256*i, fbuffer, 256, &hspi3);
		  printf ("flash_bytes = %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X %.2X, %.2X, %.2X, %.2X   \n",
					  fbuffer[0], fbuffer[1], fbuffer[2], fbuffer[3], fbuffer[4], fbuffer[5], fbuffer[6], fbuffer[7],
					  fbuffer[8], fbuffer[9], fbuffer[10], fbuffer[11], fbuffer[12], fbuffer[13], fbuffer[14], fbuffer[15],
					  fbuffer[16], fbuffer[17], fbuffer[18], fbuffer[19], fbuffer[20], fbuffer[21], fbuffer[22], fbuffer[23],
					  fbuffer[24], fbuffer[25], fbuffer[26], fbuffer[27], fbuffer[28], fbuffer[29], fbuffer[30], fbuffer[31] );
	}

}

