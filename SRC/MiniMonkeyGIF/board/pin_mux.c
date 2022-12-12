/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v8.0
processor: LPC55S69
package_id: LPC55S69JEV98
mcu_data: ksdk2_0
processor_version: 8.0.3
pin_labels:
- {pin_num: F13, pin_signal: PIO0_28/FC0_SCK/SD1_CMD/CT_INP11/SCT0_OUT7/USB0_OVERCURRENTN/PLU_OUT1/SECURE_GPIO0_28, label: LCD_RST, identifier: LCD_RST}
- {pin_num: G11, pin_signal: PIO1_1/FC3_RXD_SDA_MOSI_DATA/CT_INP3/SCT_GPI5/HS_SPI_SSEL1/USB1_OVERCURRENTN/PLU_OUT4, label: SPI_SSEL, identifier: SPI_SSEL}
- {pin_num: H12, pin_signal: PIO0_26/FC2_RXD_SDA_MOSI_DATA/CLKOUT/CT_INP14/SCT0_OUT5/USB0_IDVALUE/FC0_SCK/HS_SPI_MOSI/SECURE_GPIO0_26, label: SPI_MOSI, identifier: SPI_MOSI}
- {pin_num: G12, pin_signal: PIO1_2/CTIMER0_MAT3/SCT_GPI6/HS_SPI_SCK/USB1_PORTPWRN/PLU_OUT5, label: SPI_SCK, identifier: SPI_SCLK}
- {pin_num: G13, pin_signal: PIO1_3/SCT0_OUT4/HS_SPI_MISO/USB0_PORTPWRN/PLU_OUT6, label: LCD_CMD_DATA, identifier: LCD_CMD_DATA}
- {pin_num: A7, pin_signal: PIO0_5/FC4_RXD_SDA_MOSI_DATA/CTIMER3_MAT0/SCT_GPI5/FC3_RTS_SCL_SSEL1/MCLK/SECURE_GPIO0_5, label: BTN1, identifier: BTN1}
- {pin_num: E6, pin_signal: PIO0_19/FC4_RTS_SCL_SSEL1/UTICK_CAP0/CTIMER0_MAT2/SCT0_OUT2/FC7_TXD_SCL_MISO_WS/PLU_IN4/SECURE_GPIO0_19, label: BTN2, identifier: BTN2}
- {pin_num: E9, pin_signal: PIO0_22/FC6_TXD_SCL_MISO_WS/UTICK_CAP1/CT_INP15/SCT0_OUT3/USB0_VBUS/SD1_D0/PLU_OUT7/SECURE_GPIO0_22, label: BTN3, identifier: BTN3}
- {pin_num: B11, pin_signal: PIO0_2/FC3_TXD_SCL_MISO_WS/CT_INP1/SCT0_OUT0/SCT_GPI2/SECURE_GPIO0_2, label: WS, identifier: WS}
- {pin_num: B7, pin_signal: PIO0_6/FC3_SCK/CT_INP13/CTIMER4_MAT0/SCT_GPI6/SECURE_GPIO0_6, label: SCK, identifier: SCK}
- {pin_num: F8, pin_signal: PIO0_3/FC3_RXD_SDA_MOSI_DATA/CTIMER0_MAT1/SCT0_OUT1/SCT_GPI3/SECURE_GPIO0_3, label: MIC_DAT, identifier: MIC_DAT}
- {pin_num: F12, pin_signal: PIO1_12/FC6_SCK/CTIMER1_MAT1/USB0_PORTPWRN/HS_SPI_SSEL2, label: LCD_BL, identifier: LCD_BL}
- {pin_num: F2, pin_signal: PIO0_10/FC6_SCK/CT_INP10/CTIMER2_MAT0/FC1_TXD_SCL_MISO_WS/SCT0_OUT2/SWO/SECURE_GPIO0_10/ADC0_1, label: SWO, identifier: SWO}
- {pin_num: C12, pin_signal: PIO0_13/FC1_CTS_SDA_SSEL0/UTICK_CAP0/CT_INP0/SCT_GPI0/FC1_RXD_SDA_MOSI_DATA/PLU_IN0/SECURE_GPIO0_13, label: FC1_SDA, identifier: FC1_SDA}
- {pin_num: C13, pin_signal: PIO0_14/FC1_RTS_SCL_SSEL1/UTICK_CAP1/CT_INP1/SCT_GPI1/FC1_TXD_SCL_MISO_WS/PLU_IN1/SECURE_GPIO0_14, label: FC1_SCL, identifier: FC1_SCL}
- {pin_num: H8, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/SD1_D2/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29, label: PIO0_29, identifier: PIO0_29}
- {pin_num: E5, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/SD1_D3/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, label: PIO0_30, identifier: PIO0_31;PIO0_390;PIO0_30}
- {pin_num: H6, pin_signal: PIO1_31/MCLK/SD1_CLK/CTIMER0_MAT2/SCT0_OUT6/PLU_IN0, label: PIO1_31, identifier: PIO1_31}
- {pin_num: L2, pin_signal: PIO0_15/FC6_CTS_SDA_SSEL0/UTICK_CAP2/CT_INP16/SCT0_OUT2/SD0_WR_PRT/SECURE_GPIO0_15/ADC0_2, label: PIO0_15, identifier: PIO0_15}
- {pin_num: J2, pin_signal: PIO0_16/FC4_TXD_SCL_MISO_WS/CLKOUT/CT_INP4/SECURE_GPIO0_16/ADC0_8, label: PIO0_16, identifier: PIO0_16}
- {pin_num: J1, pin_signal: PIO0_23/MCLK/CTIMER1_MAT2/CTIMER3_MAT3/SCT0_OUT4/FC0_CTS_SDA_SSEL0/SD1_D1/SECURE_GPIO0_23/ADC0_0, label: PIO0_23, identifier: PIO0_23}
- {pin_num: B2, pin_signal: PIO1_4/FC0_SCK/SD0_D0/CTIMER2_MAT1/SCT0_OUT0/FREQME_GPIO_CLK_A, label: PIO1_4, identifier: PIO1_4}
- {pin_num: L1, pin_signal: PIO0_31/FC0_CTS_SDA_SSEL0/SD0_D2/CTIMER0_MAT1/SCT0_OUT3/SECURE_GPIO0_31/ADC0_3, label: PIO0_31, identifier: PIO0_31}
- {pin_num: H13, pin_signal: PIO1_19/SCT0_OUT7/CTIMER3_MAT1/SCT_GPI7/FC4_SCK/PLU_OUT1/ACMPVREF, label: BTN2, identifier: BTN2}
- {pin_num: M8, pin_signal: PIO1_22/SD0_CMD/CTIMER2_MAT3/SCT_GPI5/FC4_SSEL3/PLU_OUT4, label: BTN3, identifier: BTN3;BTN32}
- {pin_num: J9, pin_signal: PIO1_17/FC6_RTS_SCL_SSEL1/SCT0_OUT4/SD1_CARD_INT_N/SD1_CARD_DET_N, label: LCD_BL, identifier: LCD_BL}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: H8, peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/SD1_D2/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: E5, peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/SD1_D3/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, identifier: PIO0_30,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: F13, peripheral: GPIO, signal: 'PIO0, 28', pin_signal: PIO0_28/FC0_SCK/SD1_CMD/CT_INP11/SCT0_OUT7/USB0_OVERCURRENTN/PLU_OUT1/SECURE_GPIO0_28, direction: OUTPUT,
    gpio_init_state: 'true', mode: pullUp}
  - {pin_num: G11, peripheral: GPIO, signal: 'PIO1, 1', pin_signal: PIO1_1/FC3_RXD_SDA_MOSI_DATA/CT_INP3/SCT_GPI5/HS_SPI_SSEL1/USB1_OVERCURRENTN/PLU_OUT4, direction: OUTPUT,
    gpio_init_state: 'true'}
  - {pin_num: H12, peripheral: FLEXCOMM8, signal: HS_SPI_MOSI, pin_signal: PIO0_26/FC2_RXD_SDA_MOSI_DATA/CLKOUT/CT_INP14/SCT0_OUT5/USB0_IDVALUE/FC0_SCK/HS_SPI_MOSI/SECURE_GPIO0_26}
  - {pin_num: G12, peripheral: FLEXCOMM8, signal: HS_SPI_SCK, pin_signal: PIO1_2/CTIMER0_MAT3/SCT_GPI6/HS_SPI_SCK/USB1_PORTPWRN/PLU_OUT5}
  - {pin_num: G13, peripheral: GPIO, signal: 'PIO1, 3', pin_signal: PIO1_3/SCT0_OUT4/HS_SPI_MISO/USB0_PORTPWRN/PLU_OUT6, direction: OUTPUT}
  - {pin_num: A7, peripheral: GPIO, signal: 'PIO0, 5', pin_signal: PIO0_5/FC4_RXD_SDA_MOSI_DATA/CTIMER3_MAT0/SCT_GPI5/FC3_RTS_SCL_SSEL1/MCLK/SECURE_GPIO0_5, direction: INPUT,
    mode: pullUp}
  - {pin_num: H13, peripheral: GPIO, signal: 'PIO1, 19', pin_signal: PIO1_19/SCT0_OUT7/CTIMER3_MAT1/SCT_GPI7/FC4_SCK/PLU_OUT1/ACMPVREF, direction: INPUT, mode: pullUp}
  - {pin_num: M8, peripheral: GPIO, signal: 'PIO1, 22', pin_signal: PIO1_22/SD0_CMD/CTIMER2_MAT3/SCT_GPI5/FC4_SSEL3/PLU_OUT4, identifier: BTN3, direction: INPUT,
    mode: pullUp}
  - {pin_num: B11, peripheral: FLEXCOMM3, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_2/FC3_TXD_SCL_MISO_WS/CT_INP1/SCT0_OUT0/SCT_GPI2/SECURE_GPIO0_2}
  - {pin_num: B7, peripheral: FLEXCOMM3, signal: SCK, pin_signal: PIO0_6/FC3_SCK/CT_INP13/CTIMER4_MAT0/SCT_GPI6/SECURE_GPIO0_6}
  - {pin_num: F8, peripheral: FLEXCOMM3, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_3/FC3_RXD_SDA_MOSI_DATA/CTIMER0_MAT1/SCT0_OUT1/SCT_GPI3/SECURE_GPIO0_3}
  - {pin_num: J9, peripheral: GPIO, signal: 'PIO1, 17', pin_signal: PIO1_17/FC6_RTS_SCL_SSEL1/SCT0_OUT4/SD1_CARD_INT_N/SD1_CARD_DET_N, direction: OUTPUT, mode: pullUp}
  - {pin_num: F2, peripheral: SWD, signal: SWO, pin_signal: PIO0_10/FC6_SCK/CT_INP10/CTIMER2_MAT0/FC1_TXD_SCL_MISO_WS/SCT0_OUT2/SWO/SECURE_GPIO0_10/ADC0_1}
  - {pin_num: C12, peripheral: FLEXCOMM1, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_13/FC1_CTS_SDA_SSEL0/UTICK_CAP0/CT_INP0/SCT_GPI0/FC1_RXD_SDA_MOSI_DATA/PLU_IN0/SECURE_GPIO0_13,
    open_drain: enabled}
  - {pin_num: C13, peripheral: FLEXCOMM1, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_14/FC1_RTS_SCL_SSEL1/UTICK_CAP1/CT_INP1/SCT_GPI1/FC1_TXD_SCL_MISO_WS/PLU_IN1/SECURE_GPIO0_14,
    open_drain: enabled}
  - {pin_num: E12, peripheral: SDIF, signal: 'SD0_D, 0', pin_signal: PIO0_24/FC0_RXD_SDA_MOSI_DATA/SD0_D0/CT_INP8/SCT_GPI0/SECURE_GPIO0_24}
  - {pin_num: A11, peripheral: SDIF, signal: 'SD0_D, 1', pin_signal: PIO0_25/FC0_TXD_SCL_MISO_WS/SD0_D1/CT_INP9/SCT_GPI1/SECURE_GPIO0_25}
  - {pin_num: M5, peripheral: SDIF, signal: 'SD0_D, 2', pin_signal: PIO1_5/FC0_RXD_SDA_MOSI_DATA/SD0_D2/CTIMER2_MAT0/SCT_GPI0}
  - {pin_num: H5, peripheral: SDIF, signal: 'SD0_D, 3', pin_signal: PIO1_6/FC0_TXD_SCL_MISO_WS/SD0_D3/CTIMER2_MAT1/SCT_GPI3}
  - {pin_num: A6, peripheral: SDIF, signal: SD0_CLK, pin_signal: PIO1_8/FC0_CTS_SDA_SSEL0/SD0_CLK/SCT0_OUT1/FC4_SSEL2/ADC0_4}
  - {pin_num: C7, peripheral: SDIF, signal: SD0_CMD, pin_signal: PIO1_16/FC6_TXD_SCL_MISO_WS/CTIMER1_MAT3/SD0_CMD}
  - {pin_num: B3, peripheral: SDIF, signal: SD0_CARD_DET, pin_signal: PIO1_13/FC6_RXD_SDA_MOSI_DATA/CT_INP6/USB0_OVERCURRENTN/USB0_FRAME/SD0_CARD_DET_N}
  - {pin_num: H6, peripheral: GPIO, signal: 'PIO1, 31', pin_signal: PIO1_31/MCLK/SD1_CLK/CTIMER0_MAT2/SCT0_OUT6/PLU_IN0, direction: INPUT}
  - {pin_num: L2, peripheral: GPIO, signal: 'PIO0, 15', pin_signal: PIO0_15/FC6_CTS_SDA_SSEL0/UTICK_CAP2/CT_INP16/SCT0_OUT2/SD0_WR_PRT/SECURE_GPIO0_15/ADC0_2}
  - {pin_num: J2, peripheral: GPIO, signal: 'PIO0, 16', pin_signal: PIO0_16/FC4_TXD_SCL_MISO_WS/CLKOUT/CT_INP4/SECURE_GPIO0_16/ADC0_8}
  - {pin_num: J1, peripheral: GPIO, signal: 'PIO0, 23', pin_signal: PIO0_23/MCLK/CTIMER1_MAT2/CTIMER3_MAT3/SCT0_OUT4/FC0_CTS_SDA_SSEL0/SD1_D1/SECURE_GPIO0_23/ADC0_0}
  - {pin_num: B2, peripheral: GPIO, signal: 'PIO1, 4', pin_signal: PIO1_4/FC0_SCK/SD0_D0/CTIMER2_MAT1/SCT0_OUT0/FREQME_GPIO_CLK_A}
  - {pin_num: L1, peripheral: GPIO, signal: 'PIO0, 31', pin_signal: PIO0_31/FC0_CTS_SDA_SSEL0/SD0_D2/CTIMER0_MAT1/SCT0_OUT3/SECURE_GPIO0_31/ADC0_3, direction: INPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void BOARD_InitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);

    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t BTN1_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_5 (pin A7)  */
    GPIO_PinInit(BOARD_INITPINS_BTN1_GPIO, BOARD_INITPINS_BTN1_PORT, BOARD_INITPINS_BTN1_PIN, &BTN1_config);

    gpio_pin_config_t LCD_RST_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO0_28 (pin F13)  */
    GPIO_PinInit(BOARD_INITPINS_LCD_RST_GPIO, BOARD_INITPINS_LCD_RST_PORT, BOARD_INITPINS_LCD_RST_PIN, &LCD_RST_config);

    gpio_pin_config_t PIO0_31_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_31 (pin L1)  */
    GPIO_PinInit(BOARD_INITPINS_PIO0_31_GPIO, BOARD_INITPINS_PIO0_31_PORT, BOARD_INITPINS_PIO0_31_PIN, &PIO0_31_config);

    gpio_pin_config_t SPI_SSEL_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO1_1 (pin G11)  */
    GPIO_PinInit(BOARD_INITPINS_SPI_SSEL_GPIO, BOARD_INITPINS_SPI_SSEL_PORT, BOARD_INITPINS_SPI_SSEL_PIN, &SPI_SSEL_config);

    gpio_pin_config_t LCD_CMD_DATA_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_3 (pin G13)  */
    GPIO_PinInit(BOARD_INITPINS_LCD_CMD_DATA_GPIO, BOARD_INITPINS_LCD_CMD_DATA_PORT, BOARD_INITPINS_LCD_CMD_DATA_PIN, &LCD_CMD_DATA_config);

    gpio_pin_config_t LCD_BL_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_17 (pin J9)  */
    GPIO_PinInit(BOARD_INITPINS_LCD_BL_GPIO, BOARD_INITPINS_LCD_BL_PORT, BOARD_INITPINS_LCD_BL_PIN, &LCD_BL_config);

    gpio_pin_config_t BTN2_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_19 (pin H13)  */
    GPIO_PinInit(BOARD_INITPINS_BTN2_GPIO, BOARD_INITPINS_BTN2_PORT, BOARD_INITPINS_BTN2_PIN, &BTN2_config);

    gpio_pin_config_t BTN3_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_22 (pin M8)  */
    GPIO_PinInit(BOARD_INITPINS_BTN3_GPIO, BOARD_INITPINS_BTN3_PORT, BOARD_INITPINS_BTN3_PIN, &BTN3_config);

    gpio_pin_config_t PIO1_31_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_31 (pin H6)  */
    GPIO_PinInit(BOARD_INITPINS_PIO1_31_GPIO, BOARD_INITPINS_PIO1_31_PORT, BOARD_INITPINS_PIO1_31_PIN, &PIO1_31_config);

    IOCON->PIO[0][10] = ((IOCON->PIO[0][10] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT010 (pin F2) is configured as SWO. */
                         | IOCON_PIO_FUNC(PIO0_10_FUNC_ALT6)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_10_DIGIMODE_DIGITAL));

    IOCON->PIO[0][13] = ((IOCON->PIO[0][13] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_OD_MASK)))

                         /* Selects pin function.
                          * : PORT013 (pin C12) is configured as FC1_RXD_SDA_MOSI_DATA. */
                         | IOCON_PIO_FUNC(PIO0_13_FUNC_ALT5)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_13_DIGIMODE_DIGITAL)

                         /* Controls open-drain mode in standard GPIO mode (EGP = 1).
                          * This bit has no effect in I2C mode (EGP=0).
                          * : Open-drain.
                          * Simulated open-drain output (high drive disabled). */
                         | IOCON_PIO_OD(PIO0_13_OD_OPEN_DRAIN));

    IOCON->PIO[0][14] = ((IOCON->PIO[0][14] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_OD_MASK)))

                         /* Selects pin function.
                          * : PORT014 (pin C13) is configured as FC1_TXD_SCL_MISO_WS. */
                         | IOCON_PIO_FUNC(PIO0_14_FUNC_ALT6)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_14_DIGIMODE_DIGITAL)

                         /* Controls open-drain mode in standard GPIO mode (EGP = 1).
                          * This bit has no effect in I2C mode (EGP=0).
                          * : Open-drain.
                          * Simulated open-drain output (high drive disabled). */
                         | IOCON_PIO_OD(PIO0_14_OD_OPEN_DRAIN));

    if (Chip_GetVersion()==1)
    {
        IOCON->PIO[0][15] = ((IOCON->PIO[0][15] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT015 (pin L2) is configured as PIO0_15. */
                         | IOCON_PIO_FUNC(PIO0_15_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_15_DIGIMODE_DIGITAL));
    }
    else
    {
        IOCON->PIO[0][15] = ((IOCON->PIO[0][15] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT015 (pin L2) is configured as PIO0_15. */
                         | IOCON_PIO_FUNC(PIO0_15_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_15_DIGIMODE_DIGITAL));
    }

    IOCON->PIO[0][16] = ((IOCON->PIO[0][16] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT016 (pin J2) is configured as PIO0_16. */
                         | IOCON_PIO_FUNC(PIO0_16_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_16_DIGIMODE_DIGITAL));

    IOCON->PIO[0][2] = ((IOCON->PIO[0][2] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT02 (pin B11) is configured as FC3_TXD_SCL_MISO_WS. */
                        | IOCON_PIO_FUNC(PIO0_2_FUNC_ALT1)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_2_DIGIMODE_DIGITAL));

    IOCON->PIO[0][23] = ((IOCON->PIO[0][23] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT023 (pin J1) is configured as PIO0_23. */
                         | IOCON_PIO_FUNC(PIO0_23_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_23_DIGIMODE_DIGITAL));

    IOCON->PIO[0][24] = ((IOCON->PIO[0][24] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT024 (pin E12) is configured as SD0_D0. */
                         | IOCON_PIO_FUNC(PIO0_24_FUNC_ALT2)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_24_DIGIMODE_DIGITAL));

    IOCON->PIO[0][25] = ((IOCON->PIO[0][25] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT025 (pin A11) is configured as SD0_D1. */
                         | IOCON_PIO_FUNC(PIO0_25_FUNC_ALT2)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_25_DIGIMODE_DIGITAL));

    IOCON->PIO[0][26] = ((IOCON->PIO[0][26] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT026 (pin H12) is configured as HS_SPI_MOSI. */
                         | IOCON_PIO_FUNC(0x09u)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_26_DIGIMODE_DIGITAL));

    IOCON->PIO[0][28] = ((IOCON->PIO[0][28] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT028 (pin F13) is configured as PIO0_28. */
                         | IOCON_PIO_FUNC(PIO0_28_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO0_28_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_28_DIGIMODE_DIGITAL));

    const uint32_t PIO0_29 = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                              IOCON_PIO_FUNC1 |
                              /* No addition pin function */
                              IOCON_PIO_MODE_INACT |
                              /* Standard mode, output slew rate control is enabled */
                              IOCON_PIO_SLEW_STANDARD |
                              /* Input function is not inverted */
                              IOCON_PIO_INV_DI |
                              /* Enables digital function */
                              IOCON_PIO_DIGITAL_EN |
                              /* Open drain is disabled */
                              IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: H8) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, BOARD_INITPINS_PIO0_29_PORT, BOARD_INITPINS_PIO0_29_PIN, PIO0_29);

    IOCON->PIO[0][3] = ((IOCON->PIO[0][3] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT03 (pin F8) is configured as FC3_RXD_SDA_MOSI_DATA. */
                        | IOCON_PIO_FUNC(PIO0_3_FUNC_ALT1)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_3_DIGIMODE_DIGITAL));

    const uint32_t PIO0_30 = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                              IOCON_PIO_FUNC1 |
                              /* No addition pin function */
                              IOCON_PIO_MODE_INACT |
                              /* Standard mode, output slew rate control is enabled */
                              IOCON_PIO_SLEW_STANDARD |
                              /* Input function is not inverted */
                              IOCON_PIO_INV_DI |
                              /* Enables digital function */
                              IOCON_PIO_DIGITAL_EN |
                              /* Open drain is disabled */
                              IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN30 (coords: E5) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, BOARD_INITPINS_PIO0_30_PORT, BOARD_INITPINS_PIO0_30_PIN, PIO0_30);

    if (Chip_GetVersion()==1)
    {
        IOCON->PIO[0][31] = ((IOCON->PIO[0][31] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT031 (pin L1) is configured as PIO0_31. */
                         | IOCON_PIO_FUNC(PIO0_31_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_31_DIGIMODE_DIGITAL));
    }
    else
    {
        IOCON->PIO[0][31] = ((IOCON->PIO[0][31] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT031 (pin L1) is configured as PIO0_31. */
                         | IOCON_PIO_FUNC(PIO0_31_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_31_DIGIMODE_DIGITAL));
    }

    IOCON->PIO[0][5] = ((IOCON->PIO[0][5] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT05 (pin A7) is configured as PIO0_5. */
                        | IOCON_PIO_FUNC(PIO0_5_FUNC_ALT0)

                        /* Selects function mode (on-chip pull-up/pull-down resistor control).
                         * : Pull-up.
                         * Pull-up resistor enabled. */
                        | IOCON_PIO_MODE(PIO0_5_MODE_PULL_UP)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_5_DIGIMODE_DIGITAL));

    IOCON->PIO[0][6] = ((IOCON->PIO[0][6] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT06 (pin B7) is configured as FC3_SCK. */
                        | IOCON_PIO_FUNC(PIO0_6_FUNC_ALT1)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_6_DIGIMODE_DIGITAL));

    IOCON->PIO[1][1] = ((IOCON->PIO[1][1] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT11 (pin G11) is configured as PIO1_1. */
                        | IOCON_PIO_FUNC(PIO1_1_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_1_DIGIMODE_DIGITAL));

    IOCON->PIO[1][13] = ((IOCON->PIO[1][13] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT113 (pin B3) is configured as SD0_CARD_DET_N. */
                         | IOCON_PIO_FUNC(PIO1_13_FUNC_ALT7)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_13_DIGIMODE_DIGITAL));

    IOCON->PIO[1][16] = ((IOCON->PIO[1][16] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT116 (pin C7) is configured as SD0_CMD. */
                         | IOCON_PIO_FUNC(PIO1_16_FUNC_ALT4)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_16_DIGIMODE_DIGITAL));

    IOCON->PIO[1][17] = ((IOCON->PIO[1][17] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT117 (pin J9) is configured as PIO1_17. */
                         | IOCON_PIO_FUNC(PIO1_17_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO1_17_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_17_DIGIMODE_DIGITAL));

    IOCON->PIO[1][19] = ((IOCON->PIO[1][19] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT119 (pin H13) is configured as PIO1_19. */
                         | IOCON_PIO_FUNC(PIO1_19_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO1_19_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_19_DIGIMODE_DIGITAL));

    IOCON->PIO[1][2] = ((IOCON->PIO[1][2] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT12 (pin G12) is configured as HS_SPI_SCK. */
                        | IOCON_PIO_FUNC(PIO1_2_FUNC_ALT6)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_2_DIGIMODE_DIGITAL));

    IOCON->PIO[1][22] = ((IOCON->PIO[1][22] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT122 (pin M8) is configured as PIO1_22. */
                         | IOCON_PIO_FUNC(PIO1_22_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO1_22_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_22_DIGIMODE_DIGITAL));

    IOCON->PIO[1][3] = ((IOCON->PIO[1][3] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT13 (pin G13) is configured as PIO1_3. */
                        | IOCON_PIO_FUNC(PIO1_3_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_3_DIGIMODE_DIGITAL));

    IOCON->PIO[1][31] = ((IOCON->PIO[1][31] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT131 (pin H6) is configured as PIO1_31. */
                         | IOCON_PIO_FUNC(PIO1_31_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_31_DIGIMODE_DIGITAL));

    IOCON->PIO[1][4] = ((IOCON->PIO[1][4] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT14 (pin B2) is configured as PIO1_4. */
                        | IOCON_PIO_FUNC(PIO1_4_FUNC_ALT0)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_4_DIGIMODE_DIGITAL));

    IOCON->PIO[1][5] = ((IOCON->PIO[1][5] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT15 (pin M5) is configured as SD0_D2. */
                        | IOCON_PIO_FUNC(PIO1_5_FUNC_ALT2)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_5_DIGIMODE_DIGITAL));

    IOCON->PIO[1][6] = ((IOCON->PIO[1][6] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT16 (pin H5) is configured as SD0_D3. */
                        | IOCON_PIO_FUNC(PIO1_6_FUNC_ALT2)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_6_DIGIMODE_DIGITAL));

    IOCON->PIO[1][8] = ((IOCON->PIO[1][8] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT18 (pin A6) is configured as SD0_CLK. */
                        | IOCON_PIO_FUNC(PIO1_8_FUNC_ALT2)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_8_DIGIMODE_DIGITAL));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
