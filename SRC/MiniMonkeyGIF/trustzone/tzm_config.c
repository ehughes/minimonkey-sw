/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: TEE v2.0
processor: LPC55S69
package_id: LPC55S69JEV98
mcu_data: ksdk2_0
processor_version: 8.0.3
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "tzm_config.h"

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
tee:
- ahb:
  - regions:
    - relative_region: {memory: PROGRAM_FLASH, security: ns_user, start: '0x00000000', size: '0x000A0000'}
    - relative_region: {memory: BootROM, security: ns_user, start: '0x00000000', size: '0x00020000'}
    - relative_region: {memory: SRAMX, security: ns_user, start: '0x00000000', size: '0x00008000'}
    - relative_region: {memory: SRAM0, security: ns_user, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: SRAM1, security: ns_user, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: SRAM2, security: ns_user, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: SRAM3, security: ns_user, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: SRAM4, security: ns_user, start: '0x00000000', size: '0x00004000'}
    - relative_region: {memory: USB_RAM, security: ns_user, start: '0x00000000', size: '0x00004000'}
  - masters:
    - ns_user:
      - !!seq:id [HASH, MCM33C, MCM33S, PQ, SDIO, SDMA0, SDMA1, USBFSD, USBFSH]
  - peripherals:
    - ns_user:
      - !!seq:id [ADC0, AHB_SECURE_CTRL, ANACTRL, CASPER, CRC_ENGINE, CTIMER0, CTIMER1, CTIMER2, CTIMER3, CTIMER4, DBGMAILBOX, DMA0, DMA1, FLASH, FLEXCOMM0, FLEXCOMM1,
        FLEXCOMM2, FLEXCOMM3, FLEXCOMM4, FLEXCOMM5, FLEXCOMM6, FLEXCOMM7, GINT0, GINT1, GPIO, HASHCRYPT, INPUTMUX, IOCON, MAILBOX, MRT0, OSTIMER, PINT, PLU, PMC,
        POWERQUAD, PRINCE, PUF, RNG, RTC, SCT0, SDIF, SECGPIO, SECPINT, SPI8, SYSCON, SYSCTL, USB0, USBFSH, USBHSD, USBHSH, USBPHY, UTICK0, WWDT]
  - interrupts:
    - masking:
      - Non-masked:
        - !!seq:id [acmp_capt_irq, adc_irq, casper_irq, ctimer0_irq, ctimer1_irq, ctimer2_irq, ctimer3_irq, ctimer4_irq, flexcomm0_irq, flexcomm1_irq, flexcomm2_irq,
          flexcomm3_irq, flexcomm4_irq, flexcomm5_irq, flexcomm6_irq, flexcomm7_irq, global_irq0, global_irq1, lspi_hs_irq, mailbox_irq, mrt_irq, os_event_irq, pin_int4,
          pin_int5, pin_int6, pin_int7, pin_irq0, pin_irq1, pin_irq2, pin_irq3, plu_irq, pq_irq, qddkey_irq, rtc_irq, sct_irq, sdio_irq, sdma0_irq, sdma1_irq, sec_hypervisor_call_irq,
          sec_int0, sec_int1, sec_vio_irq, sha_irq, sys_irq, usb0_irq, usb0_needclk_irq, usb1_irq, usb1_needclk_irq, usb1_utmi_irq, utick_irq]
    - security:
      - Secure:
        - !!seq:id [acmp_capt_irq, adc_irq, casper_irq, ctimer0_irq, ctimer1_irq, ctimer2_irq, ctimer3_irq, ctimer4_irq, flexcomm0_irq, flexcomm1_irq, flexcomm2_irq,
          flexcomm3_irq, flexcomm4_irq, flexcomm5_irq, flexcomm6_irq, flexcomm7_irq, global_irq0, global_irq1, lspi_hs_irq, mailbox_irq, mrt_irq, os_event_irq, pin_int4,
          pin_int5, pin_int6, pin_int7, pin_irq0, pin_irq1, pin_irq2, pin_irq3, plu_irq, pq_irq, qddkey_irq, rtc_irq, sct_irq, sdio_irq, sdma0_irq, sdma1_irq, sec_hypervisor_call_irq,
          sec_int0, sec_int1, sec_vio_irq, sha_irq, sys_irq, usb0_irq, usb0_needclk_irq, usb1_irq, usb1_needclk_irq, usb1_utmi_irq, utick_irq]
  - pins_masks:
    - pio0:
      - Non-masked:
        - !!seq:id ['0', '1', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '2', '20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '3', '30',
          '31', '4', '5', '6', '7', '8', '9']
    - pio1:
      - Non-masked:
        - !!seq:id ['0', '1', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '2', '20', '21', '22', '23', '24', '25', '26', '27', '28', '29', '3', '30',
          '31', '4', '5', '6', '7', '8', '9']
- sau:
  - enabled: 'false'
  - all_non_secure: 'false'
  - generate_code_for_disabled_regions: 'false'
  - regions:
    - region: {index: '0', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '1', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '2', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '3', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '4', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '5', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '6', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '7', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
- global_options:
  - no:
    - !!seq:id [AIRCR_PRIS, AIRCR_BFHFNMINS, AIRCR_SYSRESETREQS, SCR_SLEEPDEEPS, SHCSR_SECUREFAULTENA, NSACR_CP0, NSACR_CP1, NSACR_CP2, NSACR_CP3, NSACR_CP4, NSACR_CP5,
      NSACR_CP6, NSACR_CP7, NSACR_CP10, NSACR_CP11, CPPWR_SU0, CPPWR_SUS0, CPPWR_SU1, CPPWR_SUS1, CPPWR_SU2, CPPWR_SUS2, CPPWR_SU3, CPPWR_SUS3, CPPWR_SU4, CPPWR_SUS4,
      CPPWR_SU5, CPPWR_SUS5, CPPWR_SU6, CPPWR_SUS6, CPPWR_SU7, CPPWR_SUS7, CPPWR_SU10, CPPWR_SUS10, CPPWR_SU11, CPPWR_SUS11, SEC_GPIO_MASK0_LOCK, SEC_GPIO_MASK1_LOCK,
      SEC_CPU1_INT_MASK0_LOCK, SEC_CPU1_INT_MASK1_LOCK, MASTER_SEC_LEVEL_LOCK, CPU0_LOCK_NS_VTOR, CPU0_LOCK_NS_MPU, CPU0_LOCK_S_VTAIRCR, CPU0_LOCK_S_MPU, CPU0_LOCK_SAU,
      CPU0_LOCK_REG_LOCK, CPU1_LOCK_NS_VTOR, CPU1_LOCK_NS_MPU, CPU1_LOCK_REG_LOCK, AHB_MISC_CTRL_REG_ENABLE_SECURE_CHECKING, AHB_MISC_CTRL_REG_ENABLE_S_PRIV_CHECK,
      AHB_MISC_CTRL_REG_ENABLE_NS_PRIV_CHECK, AHB_MISC_CTRL_REG_DISABLE_VIOLATION_ABORT, AHB_MISC_CTRL_REG_DISABLE_SIMPLE_MASTER_STRICT_MODE, AHB_MISC_CTRL_REG_DISABLE_SMART_MASTER_STRICT_MODE,
      AHB_MISC_CTRL_REG_IDAU_ALL_NS, AHB_MISC_CTRL_REG_WRITE_LOCK]
  - c_code:
    - !!seq:id [_output_type_]
- user_memory_regions: []
- mpus:
  - mpu:
    - enabled: 'false'
    - id: 's'
    - priv_default_map: 'false'
    - handler_enabled: 'false'
    - generate_code_for_disabled_regions: 'false'
    - attributes:
      - group: {index: '0', id: '0', memory_type: device, device: nGnRE}
      - group: {index: '1', id: '1', memory_type: device, device: nGnRE}
      - group: {index: '2', id: '2', memory_type: device, device: nGnRE}
      - group: {index: '3', id: '3', memory_type: device, device: nGnRE}
      - group: {index: '4', id: '4', memory_type: device, device: nGnRE}
      - group: {index: '5', id: '5', memory_type: device, device: nGnRE}
      - group: {index: '6', id: '6', memory_type: device, device: nGnRE}
      - group: {index: '7', id: '7', memory_type: device, device: nGnRE}
    - regions:
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '0', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '1', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '2', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '3', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '4', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '5', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '6', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '7', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
  - mpu:
    - enabled: 'false'
    - id: 'ns'
    - priv_default_map: 'false'
    - handler_enabled: 'false'
    - generate_code_for_disabled_regions: 'false'
    - attributes:
      - group: {index: '0', id: '0', memory_type: device, device: nGnRE}
      - group: {index: '1', id: '1', memory_type: device, device: nGnRE}
      - group: {index: '2', id: '2', memory_type: device, device: nGnRE}
      - group: {index: '3', id: '3', memory_type: device, device: nGnRE}
      - group: {index: '4', id: '4', memory_type: device, device: nGnRE}
      - group: {index: '5', id: '5', memory_type: device, device: nGnRE}
      - group: {index: '6', id: '6', memory_type: device, device: nGnRE}
      - group: {index: '7', id: '7', memory_type: device, device: nGnRE}
    - regions:
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '0', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '1', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '2', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '3', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '4', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '5', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '6', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
      - region: {executable: 'false', read_only: 'false', attributes_index: '0', index: '7', enabled: 'false', security: priv, start: '0x00000000', size: '0x00000020'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * BOARD_InitTrustZone function
 **********************************************************************************************************************/
void BOARD_InitTrustZone()
{
    /* SAU configuration */
    
    /* Set SAU Control register: Disable SAU and All Secure */
    SAU->CTRL = 0;
    
    /* Force memory writes before continuing */
    __DSB();
    /* Flush and refill pipeline with updated permissions */
    __ISB();
    
    /* Set SAU Control register: Disable SAU and All Secure */
    SAU->CTRL = ((0U << SAU_CTRL_ALLNS_Pos) & SAU_CTRL_ALLNS_Msk)
        | ((0U << SAU_CTRL_ENABLE_Pos) & SAU_CTRL_ENABLE_Msk);
    
    /* AHB configuration */

    /*--------------------------------------------------------------------
     - AHB Security Level Configurations
     -------------------------------------------------------------------*/
    /* Configuration of AHB Secure Controller
       Possible values for every memory sector or peripheral rule:
        0    Non-secure, User access allowed.
        1    Non-secure, Privileged access allowed.
        2    Secure, User access allowed.
        3    Secure, Privileged access allowed.
    */

    /* Security level configuration of all checkers */
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[2] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[2] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[3] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAMX[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM0[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM0[0].MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM1[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM1[0].MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM2[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM2[0].MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM3[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM3[0].MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM4[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_USB_HS[0].MEM_RULE[0] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL0 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL1 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL2 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL0 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL1 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL2 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL3 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT8_SLAVE0_RULE = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT8_SLAVE1_RULE = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT9_SLAVE0_RULE = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT9_SLAVE1_RULE = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT10[0].SLAVE0_RULE = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT10[0].SLAVE1_RULE = 0;

    /* Security level configuration of masters */
    AHB_SECURE_CTRL->MASTER_SEC_LEVEL = 0x80000000U;
    AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG = 0xBFFFFFFFU;

    /*--------------------------------------------------------------------
     - Pins: Reading GPIO state
     -------------------------------------------------------------------*/
    /* Possible values for every pin:
     *  0b0    Deny
     *  0b1    Allow */
    AHB_SECURE_CTRL->SEC_GPIO_MASK0 = 0xFFFFFFFFU;
    AHB_SECURE_CTRL->SEC_GPIO_MASK1 = 0xFFFFFFFFU;

    /*--------------------------------------------------------------------
     - Interrupts: Interrupt handling by Core1
     -------------------------------------------------------------------*/
    /* Possible values for every interrupt:
     *  0b0    Deny
     *  0b1    Allow */
    AHB_SECURE_CTRL->SEC_CPU_INT_MASK0 = 0xFFFFFFFFU;
    AHB_SECURE_CTRL->SEC_CPU_INT_MASK1 = 0xFFFFFFFFU;

    /*--------------------------------------------------------------------
     - Interrupts: Interrupt security configuration
     -------------------------------------------------------------------*/
    /* Possible values for every interrupt:
     *  0b0    Secure
     *  0b1    Non-secure */
    NVIC->ITNS[0] = 0;
    NVIC->ITNS[1] = 0;

    /* Global Options */
    SCB->AIRCR = (SCB->AIRCR & 0x000009FF7U) | 0x005FA0000U;
    SCB->SCR &= 0x0FFFFFFF7U;
    SCB->SHCSR &= 0x0FFF7FFFFU;
    SCB->NSACR = 0;
    SCnSCB->CPPWR = 0;
    AHB_SECURE_CTRL->SEC_MASK_LOCK = 0x00000AAAU;
    AHB_SECURE_CTRL->MASTER_SEC_LEVEL = (AHB_SECURE_CTRL->MASTER_SEC_LEVEL & 0x03FFFFFFFU) | 0x080000000U;
    AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG = (AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG & 0x03FFFFFFFU) | 0x080000000U;
    AHB_SECURE_CTRL->CPU0_LOCK_REG = 0x800002AAU;
    AHB_SECURE_CTRL->CPU1_LOCK_REG = 0x8000000AU;
    AHB_SECURE_CTRL->MISC_CTRL_REG = (AHB_SECURE_CTRL->MISC_CTRL_REG & 0x0FFFF0003U) | 0x00000AAA8U;
    AHB_SECURE_CTRL->MISC_CTRL_DP_REG = 0x0000AAAAU;
}
