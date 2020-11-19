#include <stdio.h>
#include <driver/periph_ctrl.h>
#include <driver/gpio.h>
#include <soc/spi_periph.h>
#include <esp32/rom/ets_sys.h>
#include "types.h"
#include "util.h"

#define ESP_SPI2 1
#define ESP_SPI3 2

#define SPI3_MOSI 32 /* aka CMD */
#define SPI3_MISO 19 /* aka DATA */
#define SPI3_CLK  33
#define SPI3_CS   34 /* aka ATT */
#define SPI3_ACK  21

#define SPI_LL_RST_MASK (SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST)
#define SPI_LL_UNUSED_INT_MASK  (SPI_INT_EN | SPI_SLV_WR_STA_DONE | SPI_SLV_RD_STA_DONE | SPI_SLV_WR_BUF_DONE | SPI_SLV_RD_BUF_DONE)

static uint32_t pkt_done = 0;

static void IRAM_ATTR spi_rx(void* arg) {
    if (!(GPIO.in1.val & BIT(SPI3_CS - 32))) {
        pkt_done = 0;
        ets_printf("%02X", SPI3.data_buf[0] & 0xFF);
    }
    else {
        if (!pkt_done) {
            ets_printf("\n");
        }
        pkt_done = 1;
    }
#if 0
    if (!(GPIO.in1.val & BIT(SPI3_CS - 32))) {
        GPIO.out_w1tc = BIT(SPI3_ACK);
        ets_delay_us(2);
        GPIO.out_w1ts = BIT(SPI3_ACK);

        SPI3.data_buf[0] = test_data[idx++];
        if (idx >= ARRAY_SIZE(test_data)) {
            idx = 0;
        }
    }
    else {
        SPI3.data_buf[0] = 0xFF;
        idx = 1;
    }
#endif
    SPI3.slave.sync_reset = 1;
    SPI3.slave.sync_reset = 0;

    SPI3.slv_wrbuf_dlen.bit_len = 8 - 1;
    SPI3.slv_rdbuf_dlen.bit_len = 8 - 1;

    SPI3.user.usr_miso = 1;
    SPI3.user.usr_mosi = 1;
    SPI3.slave.trans_done = 0;
    SPI3.cmd.usr = 1;
}

void app_main() {
    /* SPI3 (aka HSPI) */

    gpio_config_t att_conf={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL << SPI3_CS),
    };

    /* ATT is input */
    gpio_config(&att_conf);
#if 0
    /* MISO is output */
    gpio_iomux_in(SPI3_MISO, spi_periph_signal[ESP_SPI3].spiq_in);
    gpio_iomux_out(SPI3_MISO, spi_periph_signal[ESP_SPI3].func, false);

    /* MOSI is input */
    gpio_set_pull_mode(SPI3_MOSI, GPIO_PULLUP_ONLY);
    gpio_iomux_in(SPI3_MOSI, spi_periph_signal[ESP_SPI3].spid_in);
    gpio_iomux_out(SPI3_MOSI, spi_periph_signal[ESP_SPI3].func, false);

    /* CLK is input */
    gpio_set_pull_mode(SPI3_CLK, GPIO_PULLUP_ONLY);
    gpio_iomux_in(SPI3_CLK, spi_periph_signal[ESP_SPI3].spiclk_in);
    gpio_iomux_out(SPI3_CLK, spi_periph_signal[ESP_SPI3].func, false);

    /* CS is input */
    gpio_set_pull_mode(SPI3_CS, GPIO_PULLUP_ONLY);
    gpio_iomux_in(SPI3_CS, spi_periph_signal[ESP_SPI3].spics_in);
    gpio_iomux_out(SPI3_CS, spi_periph_signal[ESP_SPI3].func, false);
#else
    /* MISO is output */
    //gpio_set_direction(SPI3_MISO, GPIO_MODE_INPUT_OUTPUT);
    //gpio_matrix_out(SPI3_MISO, spi_periph_signal[ESP_SPI3].spiq_out, false, false);
    //gpio_matrix_in(SPI3_MISO, spi_periph_signal[ESP_SPI3].spiq_in, false);
    //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_MISO], PIN_FUNC_GPIO);

    /* MOSI is input */
    gpio_set_pull_mode(SPI3_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_MOSI, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_MOSI, spi_periph_signal[ESP_SPI3].spid_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_MOSI], PIN_FUNC_GPIO);

    /* CLK is input */
    gpio_set_pull_mode(SPI3_CLK, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_CLK, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_CLK, spi_periph_signal[ESP_SPI3].spiclk_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_CLK], PIN_FUNC_GPIO);

    /* CS is input */
    gpio_set_pull_mode(SPI3_CS, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_CS, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_CS, spi_periph_signal[ESP_SPI3].spics_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_CS], PIN_FUNC_GPIO);
#endif

    periph_module_enable(PERIPH_VSPI_MODULE);

    //Configure slave
    SPI3.clock.val = 0;
    SPI3.user.val = 0;
    SPI3.ctrl.val = 0;
    SPI3.slave.wr_rd_buf_en = 1; //no sure if needed
    SPI3.user.doutdin = 1; //we only support full duplex
    SPI3.user.sio = 0;
    SPI3.slave.slave_mode = 1;
    SPI3.dma_conf.val |= SPI_LL_RST_MASK;
    SPI3.dma_out_link.start = 0;
    SPI3.dma_in_link.start = 0;
    SPI3.dma_conf.val &= ~SPI_LL_RST_MASK;
    SPI3.slave.sync_reset = 1;
    SPI3.slave.sync_reset = 0;

    //use all 64 bytes of the buffer
    SPI3.user.usr_miso_highpart = 0;
    SPI3.user.usr_mosi_highpart = 0;

    //Disable unneeded ints
    SPI3.slave.val &= ~SPI_LL_UNUSED_INT_MASK;

    /* PS is LSB first */
    SPI3.ctrl.wr_bit_order = 1;
    SPI3.ctrl.rd_bit_order = 1;

    /* Mode 0 */
    SPI3.pin.ck_idle_edge = 0;
    SPI3.user.ck_i_edge = 0;
    SPI3.ctrl2.miso_delay_mode = 1;
    SPI3.ctrl2.miso_delay_num = 0;
    SPI3.ctrl2.mosi_delay_mode = 0;
    SPI3.ctrl2.mosi_delay_num = 0;

    SPI3.slave.sync_reset = 1;
    SPI3.slave.sync_reset = 0;

    SPI3.slv_wrbuf_dlen.bit_len = 8 - 1;
    SPI3.slv_rdbuf_dlen.bit_len = 8 - 1;

    SPI3.user.usr_miso = 1;
    SPI3.user.usr_mosi = 1;

    SPI3.data_buf[0] = 0xFF;
    SPI3.slave.trans_inten = 1;
    SPI3.slave.trans_done = 0;
    SPI3.cmd.usr = 1;

    esp_intr_alloc(spi_periph_signal[ESP_SPI3].irq, 0, spi_rx, NULL, NULL);
}
