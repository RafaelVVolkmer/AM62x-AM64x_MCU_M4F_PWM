/* User Includes */
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <kernel/dpl/TimerP.h>
#include <drivers/pinmux.h>
#include <kernel/dpl/DebugP.h>

#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* User Defines */
#define MCU_TIM0_CLKSEL_ADRR        (uint32_t)(0x04508068UL)
#define MCU_TIM0_BASE_ADRR          (uint32_t)(0x04800000UL)

#define MCU_TIM1_CLKSEL_ADRR        (uint32_t)(0x04508064UL)
#define MCU_TIM1_BASE_ADRR          (uint32_t)(0x04810000UL)

#define MCU_TIM2_CLKSEL_ADRR        (uint32_t)(0x0450A068UL)
#define MCU_TIM2_BASE_ADRR          (uint32_t)(0x04820000UL)

#define MCU_TIM3_CLKSEL_ADRR        (uint32_t)(0x0450806CUL)
#define MCU_TIM3_BASE_ADRR          (uint32_t)(0x04830000UL)

#define USR_TIMER_IRQ_EOI           (uint8_t)(0x20U)
#define USR_TIMER_IRQ_STATUS_RAW    (uint8_t)(0x24U)
#define USR_TIMER_IRQ_STATUS        (uint8_t)(0x28U)
#define USR_TIMER_IRQ_INT_ENABLE    (uint8_t)(0x2CU)
#define USR_TIMER_IRQ_INT_DISABLE   (uint8_t)(0x30U)
#define USR_TIMER_TCLR              (uint8_t)(0x38U)
#define USR_TIMER_TCRR              (uint8_t)(0x3CU)
#define USR_TIMER_TLDR              (uint8_t)(0x40U)
#define USR_TIMER_TMAR              (uint8_t)(0x4CU)
#define USR_TIMER_TSICR             (uint8_t)(0x54U)
#define USR_TIMER_TWPS              (uint8_t)(0x48U)

#define USR_TIMER_OVF_INT_SHIFT     (uint8_t)(0x1U)
#define USR_TIMER_MATCH_INT_SHIFT   (uint8_t)(0x0U)
#define PWM_TCLR_SCPWM_BIT_MSK      (uint8_t)(0x7U)

#define TIM3_IRQ_PRIORITY           (uint8_t)(4U)

#define MIN_DUTY                    (uint8_t)(0U)
#define MAX_DUTY                    (uint8_t)(100U)
#define MIN_FREQUENCY               (uint8_t)(0U)
#define MAX_FREQUENCY               (uint32_t)(250000U)

#define MCU_TIM0_NVIC_IRQ_PEND      (uint8_t)(4U)
#define MCU_TIM1_NVIC_IRQ_PEND      (uint8_t)(5U)
#define MCU_TIM2_NVIC_IRQ_PEND      (uint8_t)(6U)
#define MCU_TIM3_NVIC_IRQ_PEND      (uint8_t)(7U)

#define MCU_IRQ_VECTOR_OFFSET       (uint8_t)(16U)

#define SOC_PARTITION               (uint8_t)(0U)

#define MCU_TIMER_IO3               (uint8_t)(1U)

#define TIMER_TSICR_POSTED_MASK     (uint8_t)(0x00000004U)

#define TIMER_TSICR_POSTED_SHIFT    (uint8_t)(2U)

#define NS_SCALE                    (uint64_t)(1000000000UL)
#define PERCENT_SCALE               (uint64_t)(100U)
#define PRESCALER_VALUE             (uint64_t)(1U)

#define OVERFLOW_VALUE              (uint64_t)(0xFFFFFFFFUL)
#define MAX_COUNT_VALUE             (uint64_t)(0xFFFFFFFDUL)
#define PERIOD_OFFSET               (uint64_t)(1U)
#define HALF_DIVIDE                 (uint64_t)(2U)

#define TIM3_INPUT_CLOCK            (uint32_t)(25000000UL)
#define TIM3_PRESCALER_VALUE        (uint8_t)(1U)
#define TIM3_START_PERIOD           (uint8_t)(0U)

#define LOW_POLARITY                (uint8_t)(0U)
#define HIGH_POLARITY               (uint8_t)(1U)

#define CLEAR                       (uint8_t)(0U)
#define SET                         (uint8_t)(1U)

#define ENABLE                      (uint8_t)(0U)
#define DISABLE                     (uint8_t)(1U)

//User Enums
typedef enum FastIOdiag
{
    FAST_IO_SUCESS              =  0,
    FAST_IO_MATH_ERROR          = - (ERANGE)
} FastIO_status_t;

enum clockSources
{
    HFOSC0_CLKOUT               = 0b000u,
    MCU_SYSCLK0                 = 0b001u,
    CLK_12M_RC                  = 0b010u,
    MCU_PLL0_HSDIV3_CLKOUT      = 0b011u,
    MCU_EXT_REFCLK0             = 0b100u,
    DEVICE_CLKOUT_32K           = 0b101u,
    CPSW_GENF0                  = 0b110u,
    CLK_32K_RC                  = 0b111u
};

enum domainId
{
   MAIN_SOC_DOMAIN_ID           = 0u,
   MCU_SOC_DOMAIN_ID            = 1u,
   WKUP_SOC_DOMAIN_ID_WKUP      = 2u
};

enum TWPS_bitField
{
    W_PEND_TCRL = (1u << 0u),
    W_PEND_TCRR = (1u << 1u),
    W_PEND_TLDR = (1u << 2u),
    W_PEND_TTGR = (1u << 3u),
    W_PEND_TMAR = (1u << 4u),
    W_PEND_TPIR = (1u << 5u),
    W_PEND_TNIR = (1u << 6u),
    W_PEND_TCVR = (1u << 7u),
    W_PEND_TOCR = (1u << 8u),
    W_PEND_TOWR = (1u << 9u)
};

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

/* User Function Prototypes */
void TimerP_isr0(void *args);

static void FastIO_configTimer(TIM_config_t* TIM_config);

static void MCU_TIM3_pinmux(void);

static FastIO_status_t FastIO_timerControl(uint32_t timer_addr);
static inline void TIM_waitForWriteOnRegister(uint32_t timer_addr, uint32_t register_bit);
static inline uint32_t TIM_getWritingPendingRegister(uint32_t baseAddr);
static FastIO_status_t FastIO_configSignal(TIM_config_t *timer_instance, uint32_t frequency, uint32_t duty_cycle);
static void FastIO_setSignalRegisters(uint32_t timer_addr, uint32_t TCRR_value, uint32_t TLDR_value, uint32_t TMAR_value);
static inline void FatIO_signalPolarity(uint32_t base_addr, bool set);


TIM_config_t MCU_TIM3_instance = 
{
    .base_addr                  = MCU_TIM3_BASE_ADRR,
    .clksel_addr                = MCU_TIM3_CLKSEL_ADRR,

    .RAT_base_addr              = 0x0u,
    .RAT_clksel_addr            = 0x0u,

    .baseParameters = 
    {
            .inpult_clk             = TIM3_INPUT_CLOCK,
            .input_prescaler        = TIM3_PRESCALER_VALUE,
            .period_ns              = TIM3_START_PERIOD,
            .en_oenshot_mode        = DISABLE,
            .en_overflow_irq        = ENABLE,
            .en_dma_trigger         = DISABLE
    },
        
    .irqParameters = 
    {
            .irq_number             = (MCU_IRQ_VECTOR_OFFSET + MCU_TIM3_NVIC_IRQ_PEND),
            .callback_function      = TimerP_isr0,
            .is_pulse               = DISABLE,
            .irq_priority           = TIM3_IRQ_PRIORITY
    }
};


void TimerP_isr0(void *args)
{
    uint32_t value = 0x00;

    /* clear status for overflow interrupt */
    value = CSL_REG32_RD(MCU_TIM3_instance.RAT_base_addr + USR_TIMER_IRQ_STATUS);

    if(value & (0x1U << USR_TIMER_OVF_INT_SHIFT) )
    {
        value = (0x1U << USR_TIMER_OVF_INT_SHIFT);
        CSL_REG32_WR((MCU_TIM3_instance.RAT_base_addr + USR_TIMER_IRQ_STATUS), value);
    }
    else
    {
        value = (0x1U << USR_TIMER_MATCH_INT_SHIFT);
        CSL_REG32_WR((MCU_TIM3_instance.RAT_base_addr + USR_TIMER_IRQ_STATUS), value);
    }

    HwiP_clearInt(CSLR_MCU_M4FSS0_CORE0_NVIC_MCU_TIMER2_INTR_PEND_0 + 16U);
}

/* Main Function */
int main(void)
{
    /* System Init */
    System_init();
    Board_init();

    Drivers_open();
    Board_driversOpen();

    // Traduzir os endereços uma vez e armazená-los na estrutura
    MCU_TIM3_instance.RAT_base_addr      = (uint32_t)AddrTranslateP_getLocalAddr(MCU_TIM3_instance.base_addr);
    MCU_TIM3_instance.RAT_clksel_addr    = (uint32_t)AddrTranslateP_getLocalAddr(MCU_TIM3_instance.clksel_addr);

    /* User Code */
    MCU_TIM3_pinmux();

    FastIO_configTimer(&MCU_TIM3_instance);

    FastIO_timerControl(MCU_TIM3_instance.RAT_base_addr);

    FastIO_configSignal(&MCU_TIM3_instance, 250000, 50);
 
    TimerP_start(MCU_TIM3_instance.RAT_base_addr);

    /* Main Loop */
    while(1)
    {

    }

    /* System Denit */
    Board_driversClose();
    Drivers_close();

    Board_deinit();
    System_deinit();

    return 0;
}

/* TIM3 Pinmux */
static void MCU_TIM3_pinmux(void)
{
    Pinmux_PerCfg_t Timer_pin_Cfg[] =
    {
       {
           PIN_MCU_MCAN1_RX,
           ( PIN_MODE(MCU_TIMER_IO3)  | PIN_PULL_DISABLE )
       },

       {PINMUX_END, PINMUX_END}
    };

    Pinmux_config(Timer_pin_Cfg, PINMUX_DOMAIN_ID_MCU);
}

static void FastIO_configTimer(TIM_config_t *timer_instance)
{
    HwiP_Params timerHwiParams;
    HwiP_Object gtimerHwiObject;
    TimerP_Params timerParams;

    SOC_controlModuleUnlockMMR(MCU_SOC_DOMAIN_ID, SOC_PARTITION);
    *(volatile uint32_t*)timer_instance->RAT_clksel_addr = HFOSC0_CLKOUT;
    SOC_controlModuleLockMMR(MCU_SOC_DOMAIN_ID, SOC_PARTITION);

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler          = timer_instance->baseParameters.input_prescaler;
    timerParams.inputClkHz              = timer_instance->baseParameters.inpult_clk;
    timerParams.periodInNsec            = timer_instance->baseParameters.period_ns;
    timerParams.oneshotMode             = timer_instance->baseParameters.en_oenshot_mode;
    timerParams.enableOverflowInt       = timer_instance->baseParameters.en_overflow_irq;
    timerParams.enableDmaTrigger        = timer_instance->baseParameters.en_dma_trigger;
    TimerP_setup(timer_instance->RAT_base_addr, &timerParams);

    HwiP_Params_init(&timerHwiParams);
    timerHwiParams.intNum               = timer_instance->irqParameters.irq_number;
    timerHwiParams.callback             = timer_instance->irqParameters.callback_function;
    timerHwiParams.isPulse              = timer_instance->irqParameters.is_pulse;
    timerHwiParams.priority             = timer_instance->irqParameters.irq_priority;
    HwiP_construct(&gtimerHwiObject, &timerHwiParams);
}

/* Timers Control Function */
static FastIO_status_t FastIO_timerControl(uint32_t timer_addr)
{
    FastIO_status_t status_out  = FAST_IO_SUCESS;

    uint32_t TCLR_val_write     = CLEAR;

    TCLR_val_write = 
    ( 
        (0 << 14u) |  // PWM OUTPUT: MCU_TIMER03 -> MCU_TIMER_IO3 (D4 | PIN 11)
        (1 << 12u) |  // Toggle on a event
        (1 << 11u) |  // Event on MATCH and OVERFLOW
        (0 << 10u) |  // Event on MATCH and OVERFLOW
        (1 << 7u)  |  // Default value at PWM pin is high
        (1 << 6u)  |  // Enable Compare Feature
        (1 << 5u)  |  // Disable Prescaler
        (1 << 1u)  |  // Auto Reload mode
        (0 << 0u)     // Stop Timer
    );

    CSL_REG32_WR((timer_addr + USR_TIMER_TCLR), TCLR_val_write);

    TIM_waitForWriteOnRegister(timer_addr,  W_PEND_TCRL);

    return status_out;
}


static FastIO_status_t FastIO_configSignal(TIM_config_t *timer_instance, uint32_t frequency, uint32_t duty_cycle)
{
    FastIO_status_t status_out      = FAST_IO_SUCESS;

    volatile uint64_t period_in_ns           = CLEAR;
    volatile uint64_t counter_ticks          = CLEAR;
    volatile uint64_t absolute_clk           = CLEAR;
    volatile uint64_t duty_ticks             = CLEAR;

    volatile uint32_t TCRR_value             = CLEAR;
    volatile uint32_t TLDR_value             = CLEAR;
    volatile uint32_t TMAR_value             = CLEAR;

    switch(duty_cycle)
    {
        case 0:

            TimerP_stop(timer_instance->RAT_base_addr);

            FatIO_signalPolarity(timer_instance->RAT_base_addr, LOW_POLARITY);

                break;

        case 100:

            TimerP_stop(timer_instance->RAT_base_addr);

            FatIO_signalPolarity(timer_instance->RAT_base_addr, HIGH_POLARITY);

                break;

        default:

            period_in_ns    =( ( NS_SCALE / frequency ) / 2 );

            absolute_clk    = ( 
                                timer_instance->baseParameters.inpult_clk 
                                                    / 
                                timer_instance->baseParameters.input_prescaler 
                              );

            counter_ticks   = ( ( absolute_clk * period_in_ns ) / NS_SCALE );

            duty_ticks      = ( counter_ticks - ( ( counter_ticks * duty_cycle ) / PERCENT_SCALE ) );

            TCRR_value      = ( OVERFLOW_VALUE - counter_ticks - PERIOD_OFFSET );
            TLDR_value      = ( TCRR_value );
            TMAR_value      = ( OVERFLOW_VALUE - duty_ticks - PERIOD_OFFSET );

            FatIO_signalPolarity(timer_instance->RAT_base_addr, LOW_POLARITY);

                break;
    }

    if (TCRR_value >= MAX_COUNT_VALUE || TLDR_value >= MAX_COUNT_VALUE || TMAR_value >= MAX_COUNT_VALUE)
    {
        status_out = FAST_IO_MATH_ERROR;

        FastIO_setSignalRegisters(timer_instance->RAT_base_addr, CLEAR, CLEAR, CLEAR);
    }
    else
    {
        status_out = FAST_IO_SUCESS;

        FastIO_setSignalRegisters(timer_instance->RAT_base_addr, TCRR_value, TLDR_value, TMAR_value);
    }

    return status_out;
}

static inline void TIM_waitForWriteOnRegister(uint32_t timer_addr, uint32_t register_bit)
{
    if ( HW_RD_FIELD32(timer_addr + USR_TIMER_TSICR, TIMER_TSICR_POSTED) != FALSE )
    {
        while( (TIM_getWritingPendingRegister(timer_addr) & register_bit) != FALSE )
        {
            /* Do nothing - Busy wait */
        }
    }
}

static inline uint32_t TIM_getWritingPendingRegister(uint32_t timer_addr)
{
    return ( HW_RD_REG32(timer_addr + USR_TIMER_TWPS) );
}

static inline void FatIO_signalPolarity(uint32_t base_addr, bool set)
{
    uint32_t TCLR_value = CSL_REG32_RD(base_addr + USR_TIMER_TCLR);

    TCLR_value = set ? 
    (TCLR_value |  (1u << PWM_TCLR_SCPWM_BIT_MSK)) : 
    (TCLR_value & ~(1u << PWM_TCLR_SCPWM_BIT_MSK));

    CSL_REG32_WR(base_addr + USR_TIMER_TCLR, TCLR_value);
    TIM_waitForWriteOnRegister(base_addr, W_PEND_TCRL);
}

static void FastIO_setSignalRegisters(uint32_t timer_addr, uint32_t TCRR_value, uint32_t TLDR_value, uint32_t TMAR_value)
{
    
    TimerP_stop(timer_addr);

    /* 
    *   - SET (TCRR) REGISTER -
    * 
    *   COUNTER FIRST VALUE : counter start in TCRR. stop in overflow, when counter reach 0xFFFFFFFF.
    */
    CSL_REG32_WR((timer_addr + USR_TIMER_TCRR), TCRR_value);
    TIM_waitForWriteOnRegister(timer_addr, W_PEND_TCRR);

    /* 
    *   - SET (TLDR) REGISTER -
    *
    *   RELOAD VALUE: first value of counter (TCRR) after overflow event.
    */
    CSL_REG32_WR((timer_addr + USR_TIMER_TLDR), TLDR_value);
    TIM_waitForWriteOnRegister(timer_addr, W_PEND_TLDR);
    
    /* 
    *   - SET (TMAR) REGISTER -
    *
    *   COUNTER COMPARE VALUE : generate a event when counter reach this value.
    */
    CSL_REG32_WR((timer_addr + USR_TIMER_TMAR), TMAR_value);
    TIM_waitForWriteOnRegister(timer_addr, W_PEND_TMAR);
}
