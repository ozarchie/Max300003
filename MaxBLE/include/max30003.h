#pragma once

#define WREG 0x00
#define RREG 0x01

#define EINT_STATUS         1 << 23
#define RTOR_STATUS         1 << 10
#define RTOR_REG_OFFSET     10
#define RTOR_LSB_RES        0.0078125f
#define FIFO_OVF            0x7
#define FIFO_VALID_SAMPLE   0x0
#define FIFO_FAST_SAMPLE    0x1
#define ETAG_BITS           0x7

#define   STATUS          0x01
#define   EN_INT          0x02
#define   EN_INT2         0x03
#define   MNGR_INT        0x04
#define   MNGR_DYN        0x05
#define   SW_RST          0x08
#define   SYNCH           0x09
#define   FIFO_RST        0x0A
#define   INFO            0x0F
#define   CNFG_GEN        0x10
#define   CNFG_CAL        0x12
#define   CNFG_EMUX       0x14
#define   CNFG_ECG        0x15
#define   CNFG_RTOR1      0x1D
#define   CNFG_RTOR2      0x1E
#define   ECG_FIFO_BURST  0x20
#define   ECG_FIFO        0x21
#define   RTOR            0x25
#define   NO_OP           0x7F

// MAX30003 Configuration
    ///Status register bits
#define    bm_loff_nl    (1 << 0)          // DC leads off details
#define    bm_loff_nh    (1 << 1)          // DC leads off details
#define    bm_loff_pl    (1 << 2)          // DC leads off details
#define    bm_loff_ph    (1 << 3)          // DC leads off details

#define    bm_pllint     (1 << 8)           // PLL Unlocked Interrupt
#define    bm_samp       (1 << 9)           // Sample Synchronization Interrupt
#define    bm_rrint      (1 << 10)          // R2R detector event Interrupt
#define    bm_lonint     (1 >> 11)          // ULP leads on Interrupt

#define    bm_dcloffint  (1 << 20)          // DC leads off Interrupt
#define    bm_fstint     (1 << 21)          // ECG FIFO Fast Recovery Mode Interrupt
#define    bm_eovf       (1 << 22)          // ECG FIFO Overflow Interrupt
#define    bm_eint       (1 << 23)          // ECG FIFO Threshold Interrupt

// MAX30003 Configuration
// General config register setting
#define    bm_en_ulp_lon  (3 << 22)          // 2 Disable ULP Lead-on detection
#define    bm_fmstr       (3 << 20)          // 2 Master clock frequency 32768Hz
#define    bm_en_ecg      (1 << 19)          // 1 Enable ECG channel

#define    bm_en_dcloff   (3 << 12)          // 2 Disable DC lead-off detection
#define    bm_ipol        (1 << 11)          // 1 DC lead-off polarity = Pullup
#define    bm_image       (7 << 8)           // 3 DC lead-off magnitude = disable
#define    bm_vth         (3 << 6)           // 2 DC lead-off threshold = 300mV

#define    bm_en_rbias    (3 << 4)           // 2 Disable resistive bias
#define    bm_rbias       (3 << 2)           // 2 Resistive bias = 50M
#define    bm_rbiasp      (1 << 1)           // 1 Enable resistive bias on positive input
#define    bm_rbiasn      (1 << 0)           // 1 Enable resistive bias on negative input

// ECG Config register setting          -> 0x005000     // <d23-d22> 00:500sps, 01:250sps, 10:125sps; DHPF = .5Hz, DLPF = 40Hz

#define    bm_dlpf  1                   // Digital LPF cutoff = 40Hz
#define    bm_dhpf  1                   // Digital HPF cutoff = 0.5Hz
#define    bm_gain  3                   // ECG gain = 160V/V
#define    bm_rate  2                   // Sample rate = 128 sps

#define PIN_NUM_FCLK        GPIO_NUM_13          // MAX30003 Clock
#define MAX30003_INTB_PIN   GPIO_NUM_27          // MAX30003 IRQ
#define MAX30003_INT2B_PIN  GPIO_NUM_26          // MAX30003 Unused?

#define MSGMAX              20

void start_max30003(void);
void max30003_start_timer(void);
void MAX30003_ReadID(void);
void max30003_initchip(int pin_miso, int pin_mosi, int pin_sck, int pin_cs );
uint8_t* max30003_read_send_data(void );
float pnn_ff(unsigned int queue_array[MSGMAX]);
float mean(unsigned int queue_array[MSGMAX]);
float sdnn_ff(unsigned int queue_array[]);
int arraymax(unsigned int array[]);
int arraymin(unsigned int array[]);
