/* User Includes */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

typedef struct
{
    union //0x000
    {
        volatile uint32_t TIDR;

        volatile struct
        {
            uint32_t MINOR                  : 6;
            uint32_t CUSTOM                 : 2;
            uint32_t MAJOR                  : 3;
            uint32_t RTL                    : 5;
            uint32_t MODULE_ID              : 12;
            uint32_t BU                     : 2;
            uint32_t SCHEME                 : 2;
        }TIDR_bit;
    };

    volatile uint8_t RESERVED_0[4]; //0X004
    volatile uint8_t RESERVED_1[4]; //0X008
    volatile uint8_t RESERVED_2[4]; //0X00C

    union //0x010
    {
        volatile uint32_t TIOCP_CFG;

        volatile struct
        {
            uint32_t SOFTRESET              : 1;
            uint32_t EMUFREE                : 1;
            uint32_t IDLEMODE               : 2;
            uint32_t RESERVED_3             : 28;
        }TIOCP_CFG_bit;
    };

    volatile uint8_t RESERVED_4[4]; //0X014
    volatile uint8_t RESERVED_5[4]; //0X018
    volatile uint8_t RESERVED_6[4]; //0X01C

    union //0x020
    {
        volatile uint32_t IRQ_EOI;

        volatile struct
        {
            uint32_t LINE_NUMBER            : 1;
            uint32_t RESERVED_7             : 31;
        }IRQ_EOI_bit;
    };

    union //0x024
    {
        volatile uint32_t IRQSTATUS_RAW;

        volatile struct
        {
            uint32_t MAT_IT_FLAG            : 1;
            uint32_t OVF_IT_FLAG            : 1;
            uint32_t TCAR_IT_FLAG           : 1;
            uint32_t RESERVED_8             : 29;
        }IRQSTATUS_RAW_bit;
    };

    union //0x028
    {
        volatile uint32_t IRQSTATUS;

        volatile struct
        {
            uint32_t MAT_IT_FLAG            : 1;
            uint32_t OVF_IT_FLAG            : 1;
            uint32_t TCAR_IT_FLAG           : 1;
            uint32_t RESERVED_9             : 29;
        }IRQSTATUS_bit;
    };

    union //0x02C
    {
        volatile uint32_t IRQSTATUS_SET;

        volatile struct
        {
            uint32_t MAT_IT_FLAG            : 1;
            uint32_t OVF_IT_FLAG            : 1;
            uint32_t TCAR_IT_FLAG           : 1;
            uint32_t RESERVED_10            : 29;
        }IRQSTATUS_SET_bit;
    };

    union //0x030
    {
        volatile uint32_t IRQSTATUS_CLR;

        volatile struct
        {
            uint32_t MAT_IT_FLAG            : 1;
            uint32_t OVF_IT_FLAG            : 1;
            uint32_t TCAR_IT_FLAG           : 1;
            uint32_t RESERVED_11            : 29;
        }IRQSTATUS_CLR_bit;
    };

    union //0x034
    {
        volatile uint32_t IRQWAKEEN;

        volatile struct
        {
            uint32_t TCAR_WUP_ENA           : 1;
            uint32_t OVF_WUP_ENA            : 1;
            uint32_t MAT_WUP_ENA            : 1;
            uint32_t RESERVED_12            : 29;
        } IRQWAKEEN_bit;
    };

    union //0x38
    {
        volatile uint32_t TCLR;

        volatile struct
        {
            uint32_t ST                     : 1;
            uint32_t AR                     : 2;
            uint32_t PTV                    : 3;
            uint32_t PRE                    : 1;
            uint32_t CE                     : 1;
            uint32_t SCPW                   : 1;
            uint32_t TCM                    : 2;
            uint32_t TRG                    : 2;
            uint32_t PT                     : 1;
            uint32_t CAPT_MODE              : 1;
            uint32_t GPO_CFG                : 1;
            uint32_t RESERVED_13            : 17;
        }TCLR_bit;
    };

    union //0x03c
    {
        volatile uint32_t TCRR;

        volatile struct
        {
            uint32_t TCRR                   : 32;
        }TCRR_bit;
    };

    union //0x040
    {
        volatile uint32_t TLDR;

        volatile struct
        {
            uint32_t TLDR                   : 32;
        }TLDR_bit;
    };

    union //0x044
    {
        volatile uint32_t TTGR;

        volatile struct
        {
            uint32_t TTGR                   : 32;
        }TTGR_bit;
    };

    union //0x48
    {
        volatile uint32_t TWPS;

        volatile struct
        {
            uint32_t W_PEND_TCLR            : 1;
            uint32_t W_PEND_TCRR            : 1;
            uint32_t W_PEND_TLDR            : 1;
            uint32_t W_PEND_TTGR            : 1;
            uint32_t W_PEND_TMAR            : 1;
            uint32_t W_PEND_TPIR            : 1;
            uint32_t W_PEND_TNIR            : 1;
            uint32_t W_PEND_TCVR            : 1;
            uint32_t W_PEND_TOCR            : 1;
            uint32_t W_PEND_TOWR            : 1;
            uint32_t RESERVED_14            : 22;
        }TWPS_bit;
    };

    union //0x04C
    {
        volatile uint32_t TMAR;

        volatile struct
        {
            uint32_t TMAR                   : 32;
        }TMAR_bit;
    };

    union //0x050
    {
        volatile uint32_t TCAR1;

        volatile struct
        {
            uint32_t TCAR1                  : 32;
        }TCAR1_bit;
    };

    union //0x54
    {
        volatile uint32_t TSICR;

        volatile struct
        {
            uint32_t RESERVED_15            : 1;
            uint32_t SFT                    : 1;
            uint32_t POSTED                 : 1;
            uint32_t READ_MODE              : 1;
            uint32_t READ_AFTER_IDLE        : 1;
            uint32_t RESERVED_16            : 27;
        }TSICR_bit;
    };

    union //0x058
    {
        volatile uint32_t TCAR2;

        volatile struct
        {
            uint32_t TCAR2                  : 32;
        }TCAR_bit;
    };

    union //0x05C
    {
        volatile uint32_t TPIR;

        volatile struct
        {
            uint32_t TPIR                   : 32;
        }TPIR_bit;
    };

    union //0x060
    {
        volatile uint32_t TNIR;

        volatile struct
        {
            uint32_t TNIR                   : 32;
        }TNIR_bit;
    };

    union //0x060
    {
        volatile uint32_t TCVR;

        volatile struct
        {
            uint32_t TCVR                   : 32;
        }TCVR_bit;
    };

    union //0x068
    {
        volatile uint32_t TOCR;

        volatile struct
        {
            uint32_t OVF_COUNTER_VALUE      : 24;
            uint32_t RESERVED_17            : 8;
        }TOCR_bit;
    };

    union //0x06C
    {
        volatile uint32_t TOWR;

        volatile struct
        {
            uint32_t OVF_WRAPPING_VALUE     : 24;
            uint32_t RESERVED_18            : 8;
        }TOWR_bit;
    };

} __attribute__ ((packed)) timer_t;

volatile timer_t MCU_TIM3 __attribute__((section(".MCU_TIM3")));

typedef struct timerInstance
{
    uint32_t base_addr;
    uint32_t clksel_addr;

    uint32_t RAT_base_addr;
    uint32_t RAT_clksel_addr;

    struct
    {
        uint32_t inpult_clk;
        uint32_t input_prescaler;
        uint32_t period_ns;
        uint32_t en_oenshot_mode;
        uint32_t en_overflow_irq;
        uint32_t en_dma_trigger;
    } baseParameters;

    struct
    {
        uint32_t irq_number;
        void (*callback_function)(void* args);
        uint32_t is_pulse;
        uint32_t irq_priority;
    } irqParameters;

} TIM_config_t;

/* Main Function */
int main(void)
{

    while(1)
    {

    }

    return 0;
}
