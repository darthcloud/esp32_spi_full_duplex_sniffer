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

static spi_dev_t *spi_hw[2] = {&SPI2, &SPI3};
static uint32_t pkt_done = 0;

static void IRAM_ATTR packet_end(void* arg) {
    const uint32_t low_io = GPIO.pcpu_int;
    const uint32_t high_io = GPIO.pcpu_int1.intr;

    if (high_io & BIT(SPI3_CS - 32)) {
        if (!pkt_done) {
            ets_printf("\n");
        }
        pkt_done = 1;
    }
    if (high_io) GPIO.status1_w1tc.intr_st = high_io;
    if (low_io) GPIO.status_w1tc = low_io;
}
static void IRAM_ATTR spi_rx(void* arg) {
    if (!(GPIO.in1.val & BIT(SPI3_CS - 32))) {
        pkt_done = 0;
        ets_delay_us(5);
        ets_printf("%02X ", SPI3.data_buf[0] & 0xFF);
        ets_printf("%02X\n", SPI2.data_buf[0] & 0xFF);
    }
    //else {
    //    if (!pkt_done) {
    //        ets_printf("\n");
    //    }
    //    pkt_done = 1;
    //}
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
    for (uint32_t i = 0; i < ARRAY_SIZE(spi_hw); i++) {
        spi_hw[i]->slave.sync_reset = 1;
        spi_hw[i]->slave.sync_reset = 0;

        spi_hw[i]->slv_wrbuf_dlen.bit_len = 8 - 1;
        spi_hw[i]->slv_rdbuf_dlen.bit_len = 8 - 1;

        spi_hw[i]->user.usr_miso = 1;
        spi_hw[i]->user.usr_mosi = 1;
        spi_hw[i]->slave.trans_done = 0;
        spi_hw[i]->cmd.usr = 1;
    }
}

void app_main() {
    gpio_config_t att_conf={
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL << SPI3_CS),
    };

    /* ATT is input */
    gpio_config(&att_conf);

    /* MISO is input */
    gpio_set_pull_mode(SPI3_MISO, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_MISO, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_MISO, spi_periph_signal[ESP_SPI2].spid_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_MISO], PIN_FUNC_GPIO);

    /* MOSI is input */
    gpio_set_pull_mode(SPI3_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_MOSI, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_MOSI, spi_periph_signal[ESP_SPI3].spid_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_MOSI], PIN_FUNC_GPIO);

    /* CLK is input */
    gpio_set_pull_mode(SPI3_CLK, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_CLK, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_CLK, spi_periph_signal[ESP_SPI2].spiclk_in, false);
    gpio_matrix_in(SPI3_CLK, spi_periph_signal[ESP_SPI3].spiclk_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_CLK], PIN_FUNC_GPIO);

    /* CS is input */
    gpio_set_pull_mode(SPI3_CS, GPIO_PULLUP_ONLY);
    gpio_set_direction(SPI3_CS, GPIO_MODE_INPUT);
    gpio_matrix_in(SPI3_CS, spi_periph_signal[ESP_SPI2].spics_in, false);
    gpio_matrix_in(SPI3_CS, spi_periph_signal[ESP_SPI3].spics_in, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[SPI3_CS], PIN_FUNC_GPIO);

    periph_module_enable(PERIPH_HSPI_MODULE);
    periph_module_enable(PERIPH_VSPI_MODULE);

    //Configure slave
    for (uint32_t i = 0; i < ARRAY_SIZE(spi_hw); i++) {
        spi_hw[i]->clock.val = 0;
        spi_hw[i]->user.val = 0;
        spi_hw[i]->ctrl.val = 0;
        spi_hw[i]->slave.wr_rd_buf_en = 1; //no sure if needed
        spi_hw[i]->user.doutdin = 1; //we only support full duplex
        spi_hw[i]->user.sio = 0;
        spi_hw[i]->slave.slave_mode = 1;
        spi_hw[i]->dma_conf.val |= SPI_LL_RST_MASK;
        spi_hw[i]->dma_out_link.start = 0;
        spi_hw[i]->dma_in_link.start = 0;
        spi_hw[i]->dma_conf.val &= ~SPI_LL_RST_MASK;
        spi_hw[i]->slave.sync_reset = 1;
        spi_hw[i]->slave.sync_reset = 0;

        //use all 64 bytes of the buffer
        spi_hw[i]->user.usr_miso_highpart = 0;
        spi_hw[i]->user.usr_mosi_highpart = 0;

        //Disable unneeded ints
        spi_hw[i]->slave.val &= ~SPI_LL_UNUSED_INT_MASK;

        /* PS is LSB first */
        spi_hw[i]->ctrl.wr_bit_order = 1;
        spi_hw[i]->ctrl.rd_bit_order = 1;

        /* Should be Mode 3, but Mode 1 is what actualy work */
        spi_hw[i]->pin.ck_idle_edge = 1;
        spi_hw[i]->user.ck_i_edge = 1;
        spi_hw[i]->ctrl2.miso_delay_mode = 2;
        spi_hw[i]->ctrl2.miso_delay_num = 0;
        spi_hw[i]->ctrl2.mosi_delay_mode = 0;
        spi_hw[i]->ctrl2.mosi_delay_num = 0;

        spi_hw[i]->slave.sync_reset = 1;
        spi_hw[i]->slave.sync_reset = 0;

        spi_hw[i]->slv_wrbuf_dlen.bit_len = 8 - 1;
        spi_hw[i]->slv_rdbuf_dlen.bit_len = 8 - 1;

        spi_hw[i]->user.usr_miso = 1;
        spi_hw[i]->user.usr_mosi = 1;

        spi_hw[i]->data_buf[0] = 0xFF;
    }
    SPI2.slave.trans_inten = 1;
    SPI2.slave.trans_done = 0;
    SPI2.cmd.usr = 1;
    SPI3.slave.trans_inten = 1;
    SPI3.slave.trans_done = 0;
    SPI3.cmd.usr = 1;

    esp_intr_alloc(spi_periph_signal[ESP_SPI3].irq, 0, spi_rx, NULL, NULL);
    esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 0, packet_end, NULL, NULL);
}
