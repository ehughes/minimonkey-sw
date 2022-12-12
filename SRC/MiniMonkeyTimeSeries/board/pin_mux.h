/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Enables digital function */
#define IOCON_PIO_DIGITAL_EN 0x0100u
/*!
 * @brief Selects pin function 1 */
#define IOCON_PIO_FUNC1 0x01u
/*!
 * @brief Input function is not inverted */
#define IOCON_PIO_INV_DI 0x00u
/*!
 * @brief No addition pin function */
#define IOCON_PIO_MODE_INACT 0x00u
/*!
 * @brief Open drain is disabled */
#define IOCON_PIO_OPENDRAIN_DI 0x00u
/*!
 * @brief Standard mode, output slew rate control is enabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_10_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 6. */
#define PIO0_10_FUNC_ALT6 0x06u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_13_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 5. */
#define PIO0_13_FUNC_ALT5 0x05u
/*!
 * @brief
 * Controls open-drain mode in standard GPIO mode (EGP = 1).
 * This bit has no effect in I2C mode (EGP=0).
 * : Open-drain.
 * Simulated open-drain output (high drive disabled).
 */
#define PIO0_13_OD_OPEN_DRAIN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_14_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 6. */
#define PIO0_14_FUNC_ALT6 0x06u
/*!
 * @brief
 * Controls open-drain mode in standard GPIO mode (EGP = 1).
 * This bit has no effect in I2C mode (EGP=0).
 * : Open-drain.
 * Simulated open-drain output (high drive disabled).
 */
#define PIO0_14_OD_OPEN_DRAIN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_15_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_15_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_16_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_16_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_23_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_23_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_24_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 2. */
#define PIO0_24_FUNC_ALT2 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_25_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 2. */
#define PIO0_25_FUNC_ALT2 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_26_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_28_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_28_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_28_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_2_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_2_FUNC_ALT1 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_31_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_31_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_3_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_3_FUNC_ALT1 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_5_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_5_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_5_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_6_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_6_FUNC_ALT1 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_13_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 7. */
#define PIO1_13_FUNC_ALT7 0x07u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_16_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 4. */
#define PIO1_16_FUNC_ALT4 0x04u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_17_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_17_FUNC_ALT0 0x00u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO1_17_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_19_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_19_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_19_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_1_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_1_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_22_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_22_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_22_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_2_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 6. */
#define PIO1_2_FUNC_ALT6 0x06u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_31_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_31_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_3_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_3_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_4_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_4_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_5_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 2. */
#define PIO1_5_FUNC_ALT2 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_6_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 2. */
#define PIO1_6_FUNC_ALT2 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_8_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 2. */
#define PIO1_8_FUNC_ALT2 0x02u

/*! @name PIO0_29 (coord H8), PIO0_29
  @{ */
#define BOARD_INITPINS_PIO0_29_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO0_29_PIN 29U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO0_29_PIN_MASK (1U << 29U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_30 (coord E5), PIO0_30
  @{ */
#define BOARD_INITPINS_PIO0_30_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO0_30_PIN 30U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO0_30_PIN_MASK (1U << 30U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_28 (coord F13), LCD_RST
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LCD_RST_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LCD_RST_GPIO_PIN_MASK (1U << 28U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_LCD_RST_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LCD_RST_PIN 28U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LCD_RST_PIN_MASK (1U << 28U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO1_1 (coord G11), SPI_SSEL
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SPI_SSEL_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_SPI_SSEL_GPIO_PIN_MASK (1U << 1U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_SPI_SSEL_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SPI_SSEL_PIN 1U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SPI_SSEL_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_26 (coord H12), SPI_MOSI
  @{ */
#define BOARD_INITPINS_SPI_MOSI_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SPI_MOSI_PIN 26U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SPI_MOSI_PIN_MASK (1U << 26U)      /*!<@brief PORT pin mask */
                                                          /* @} */

/*! @name PIO1_2 (coord G12), SPI_SCK
  @{ */
#define BOARD_INITPINS_SPI_SCLK_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SPI_SCLK_PIN 2U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SPI_SCLK_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO1_3 (coord G13), LCD_CMD_DATA
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LCD_CMD_DATA_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LCD_CMD_DATA_GPIO_PIN_MASK (1U << 3U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_LCD_CMD_DATA_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LCD_CMD_DATA_PIN 3U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LCD_CMD_DATA_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                                             /* @} */

/*! @name PIO0_5 (coord A7), BTN1
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_BTN1_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_BTN1_GPIO_PIN_MASK (1U << 5U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_BTN1_PORT 0U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_BTN1_PIN 5U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_BTN1_PIN_MASK (1U << 5U)      /*!<@brief PORT pin mask */
                                                     /* @} */

/*! @name PIO1_19 (coord H13), BTN2
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_BTN2_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_BTN2_GPIO_PIN_MASK (1U << 19U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_BTN2_PORT 1U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_BTN2_PIN 19U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_BTN2_PIN_MASK (1U << 19U)      /*!<@brief PORT pin mask */
                                                      /* @} */

/*! @name PIO1_22 (coord M8), BTN3
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_BTN3_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_BTN3_GPIO_PIN_MASK (1U << 22U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_BTN3_PORT 1U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_BTN3_PIN 22U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_BTN3_PIN_MASK (1U << 22U)      /*!<@brief PORT pin mask */
                                                      /* @} */

/*! @name PIO0_2 (coord B11), WS
  @{ */
#define BOARD_INITPINS_WS_PORT 0U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_WS_PIN 2U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_WS_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                                   /* @} */

/*! @name PIO0_6 (coord B7), SCK
  @{ */
#define BOARD_INITPINS_SCK_PORT 0U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SCK_PIN 6U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SCK_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                                    /* @} */

/*! @name PIO0_3 (coord F8), MIC_DAT
  @{ */
#define BOARD_INITPINS_MIC_DAT_PORT 0U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_MIC_DAT_PIN 3U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_MIC_DAT_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PIO1_17 (coord J9), LCD_Bl
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LCD_BL_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LCD_BL_GPIO_PIN_MASK (1U << 17U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_LCD_BL_PORT 1U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LCD_BL_PIN 17U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LCD_BL_PIN_MASK (1U << 17U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PIO0_10 (coord F2), SWO
  @{ */
#define BOARD_INITPINS_SWO_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SWO_PIN 10U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SWO_PIN_MASK (1U << 10U)      /*!<@brief PORT pin mask */
                                                     /* @} */

/*! @name PIO0_13 (coord C12), FC1_SDA
  @{ */
#define BOARD_INITPINS_FC1_SDA_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_FC1_SDA_PIN 13U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_FC1_SDA_PIN_MASK (1U << 13U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_14 (coord C13), FC1_SCL
  @{ */
#define BOARD_INITPINS_FC1_SCL_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_FC1_SCL_PIN 14U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_FC1_SCL_PIN_MASK (1U << 14U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO1_31 (coord H6), PIO1_31
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PIO1_31_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PIO1_31_GPIO_PIN_MASK (1U << 31U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_PIO1_31_PORT 1U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO1_31_PIN 31U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO1_31_PIN_MASK (1U << 31U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_15 (coord L2), PIO0_15
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PIO0_15_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PIO0_15_GPIO_PIN_MASK (1U << 15U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_PIO0_15_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO0_15_PIN 15U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO0_15_PIN_MASK (1U << 15U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_16 (coord J2), PIO0_16
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PIO0_16_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PIO0_16_GPIO_PIN_MASK (1U << 16U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_PIO0_16_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO0_16_PIN 16U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO0_16_PIN_MASK (1U << 16U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO0_23 (coord J1), PIO0_23
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PIO0_23_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PIO0_23_GPIO_PIN_MASK (1U << 23U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_PIO0_23_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO0_23_PIN 23U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO0_23_PIN_MASK (1U << 23U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PIO1_4 (coord B2), PIO1_4
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PIO1_4_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PIO1_4_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_PIO1_4_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO1_4_PIN 4U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO1_4_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                                       /* @} */

/*! @name PIO0_31 (coord L1), PIO0_31
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PIO0_31_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PIO0_31_GPIO_PIN_MASK (1U << 31U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_PIO0_31_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PIO0_31_PIN 31U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PIO0_31_PIN_MASK (1U << 31U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void); /* Function assigned for the Cortex-M33 (Core #0) */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
