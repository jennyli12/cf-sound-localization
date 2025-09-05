#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <nrfx_i2s.h>
#include <hal/nrf_i2s.h>
#include <nrfx_grtc.h>
#include <helpers/nrfx_gppi.h>
#include <zephyr/drivers/flash.h>
#include "conn_time_sync.h"

#define SAMPLE_FREQUENCY    8000
#define SAMPLE_BIT_WIDTH    32
#define NUMBER_OF_CHANNELS  1
#define BLOCK_SIZE          4080 
#define TIMEOUT             0

#define FLASH_SECTOR_SIZE   4096
#define FLASH_MAX_SIZE      1024 * 1024
#define FLASH_READ_CHUNK    64 

static const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s20));
static const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r64));

K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, BLOCK_SIZE, 8, 4);

static off_t flash_write_offset = 0;
static uint64_t time_diff = 0;

struct __packed ts_block {
    uint64_t local_ts;
    uint64_t central_ts;
    int32_t samples[BLOCK_SIZE / 4];
};

static struct ts_block flash_write_data;

static int write_to_flash(uint64_t local_ts, uint64_t central_ts, int32_t *samples)
{
    if (flash_write_offset + FLASH_SECTOR_SIZE > FLASH_MAX_SIZE) {
        printk("Flash log area full.\n");
        return -1;
    }

    flash_write_data.local_ts = local_ts;
    flash_write_data.central_ts = central_ts;
    memcpy(flash_write_data.samples, samples, BLOCK_SIZE);

    int ret = flash_write(flash_dev, flash_write_offset, &flash_write_data, FLASH_SECTOR_SIZE);
    if (ret != 0) {
        printk("Flash write failed: %d\n", ret);
        return -1;
    }

    flash_write_offset += FLASH_SECTOR_SIZE;

    return 0;
}

int timed_i2s_flash_init(void)
{
    if (!device_is_ready(flash_dev)) {
		printk("Flash device is not ready\n");
		return -1;
	}

    printk("Erasing flash...\n");

    int ret = flash_erase(flash_dev, flash_write_offset, FLASH_MAX_SIZE);
    if (ret != 0) {
        printk("Flash erase failed: %d\n", ret);
        return -1;
    }

    printk("Erasing flash finished.\n");

    return 0;
}

void timed_i2s_flash_dump(void)
{
    if (!device_is_ready(flash_dev)) {
		printk("Flash device is not ready\n");
		return;
	}

    uint8_t buf[FLASH_READ_CHUNK];
    off_t addr = 0;

    printk("Dumping flash...\n");

    while (addr < FLASH_MAX_SIZE) {
        int ret = flash_read(flash_dev, addr, buf, FLASH_READ_CHUNK);
        if (ret != 0) {
            printk("Flash read failed: %d\n", ret);
            break;
        }

        for (int i = 0; i < FLASH_READ_CHUNK; i++) {
            printk("%02x", buf[i]);
        }

        addr += FLASH_READ_CHUNK;
    }

    printk("\nDumping flash finished.\n");
}

int timed_i2s_init(void)
{
	uint8_t ppi_chan_i2s;

    if (nrfx_gppi_channel_alloc(&ppi_chan_i2s) != NRFX_SUCCESS) {
		printk("Failed allocating PPI chan for I2S\n");
		return -ENOMEM;
	}

    nrfx_gppi_channel_endpoints_setup(ppi_chan_i2s,
        nrf_i2s_event_address_get(NRF_I2S20, NRF_I2S_EVENT_RXPTRUPD),
        nrf_grtc_task_address_get(NRF_GRTC, nrf_grtc_sys_counter_capture_task_get(0)));

	nrfx_gppi_channels_enable(BIT(ppi_chan_i2s));

    struct i2s_config i2s_cfg = {
        .word_size = SAMPLE_BIT_WIDTH,
        .channels = NUMBER_OF_CHANNELS,
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = SAMPLE_FREQUENCY,
        .mem_slab = &i2s_mem_slab,
        .block_size = BLOCK_SIZE,
        .timeout = TIMEOUT
    };

    int ret = i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg);
    if (ret != 0) {
		printk("i2s_configure failed with %d error\n", ret);
		return -1;
    }

    ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error\n", ret);
        return -1;
	}

    return 0;
}

int timed_i2s_read(void)
{
    void *mem_block;
    size_t size;

    int ret = i2s_read(i2s_dev, &mem_block, &size);
    if (ret != 0) {
        return ret;
    }

    uint64_t local_ts = nrf_grtc_sys_counter_cc_get(NRF_GRTC, 0);
    uint64_t central_ts = local_ts + time_diff;
    int32_t *samples = (int32_t *)mem_block;

    printk("local: %llu central: %llu\n", local_ts, central_ts);

    ret = write_to_flash(local_ts, central_ts, samples);

    k_mem_slab_free(&i2s_mem_slab, mem_block);

    return ret;
}

int timed_i2s_drop(void)
{
    int ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
	if (ret != 0) {
		printk("i2s_trigger failed with %d error\n", ret);
        return -1;
	}

    return 0;
}

void timed_i2s_set_time_diff(uint64_t time_diff_us)
{
    time_diff = time_diff_us;
}
