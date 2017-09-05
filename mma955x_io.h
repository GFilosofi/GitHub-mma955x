/******************************************************************************
 * Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
 * Author: Gabriele Filosofi <gabrielef@cosmed.it>
 * File Name		: mma_955x.h
 * Description		: data structure related to MMA955X.
 ******************************************************************************/
#ifndef MMA_955X_H_
#define MMA_955X_H_

/******************************************************************************
*		Slave Port Command Interpreter (CI)
*******************************************************************************/
/* CI Commands */
#define CMD_RD_FIX			0x00	/* read version from ROM CI */
#define CMD_RD_CFG			0x10	/* read cfg */
#define CMD_WR_CFG			0x20	/* write cfg */
#define CMD_RD_STS			0x30	/* read status */

/* CI Error Codes */
#define MCI_ERROR_NONE			0x00
#define MCI_ERROR_PARAM			0x04
#define MCI_INVALID_COUNT		0x19
#define MCI_ERROR_COMMAND		0x1C
#define MCI_ERROR_INVALID_LENGTH	0x21
#define MCI_ERROR_FIFO_BUSY		0x22
#define MCI_ERROR_FIFO_ALLOCATED	0x23
#define MCI_ERROR_FIFO_OVERSIZE		0x24

/******************************************************************************
* 			Application Identifiers
*******************************************************************************/
typedef enum appid_tag { //org: algid
//Common to all MMA955x devices:
	VERSION_APPID			= 0x00,	/* Version, Always active */ //org: NO_APP_APPID
	SCHED_APPID			= 0x01,
	CI_APPID			= 0x02,	/* Reserved, 488 Hz */
	GPIO_APPID			= 0x03,	/* GPIO, 488 Hz */
	MBOX_APPID			= 0x04,	/* Mailbox, 488 Hz */
	TEST_VECTOR_CAPTURE_APPID	= 0x05,	/* Reserved */
	AFE_APPID			= 0x06,	/* Analog Front-End, 488 Hz */ //org: XYZ_DATA_APPID
//Gestures (available only for MMA9551):
	PL_APPID			= 0x07,	/* Portrait/Landscape app., 122 Hz. Detects the device orientation */
	HIGH_G_DETECTION_APPID		= 0x08,	/* High-G, 244 Hz. Performs motion and drop detection */
	LOW_G_DETECTION_APPID		= 0x09,	/* Low-G, 244 Hz. Performs motion and drop detection */
	TAP_DETECTION_APPID		= 0x0a,	/* Tap detection, 488 Hz */ //org: PULSE_DETECTION_APPID
	TILT_SENSING_APPID		= 0x0b,	/* Tilt, 122 Hz. Detects the device’s tilt, relative to each of its axes */
//Common to all MMA955x devices:
	FRAME_COUNTER_APPID		= 0x0e,	/* 488 Hz */
	DATA_FIFO_APPID			= 0x0f,	/* Data FIFO, 488 Hz */
	EVENT_FIFO_APPID		= 0x10,	/* Event Queue, 488 Hz */
	STATUS_REG_APPID		= 0x11,	/* Status Register, 488 Hz */
	SLEEP_WAKE_APPID		= 0x12,	/* Sleep/Wake, 488 Hz */
//Pedometer (available only for MMA9553):
	RESERVED_APPID			= 0x14,	/* Reserved for Pedometer internal functions */
	PEDOM_APPID			= 0x15,	/* Pedometer */
//Common to all MMA955x devices:
	RST_CLR_SUSP_SYSID		= 0x17,	/* Reset/suspend/Clear, 488 Hz */
	MBOXCFG_APPID			= 0x18,	/* Mailbox Configuration, 488 Hz */ //org: MBOX_INTO_CFG_APPID
//Motion (available only for MMA9550):
	BASIC_APPID			= 0x19,	/* mma9550, 1-400 Hz */
	HUB_APPID			= 0x1a,	/* mma9550, 1-400 Hz  */
//....available range for other applications.....up to 0x1f
	MAX_APPID			= 32,
	NO_APP_APPID_INDX		= 0xff
} appid_t;

/******************************************************************************
*				Device Version
*******************************************************************************/
typedef struct {
	uint32_t id;
	uint8_t	rom_major;
	uint8_t rom_minor;
	uint8_t fw_major;
	uint8_t fw_minor;
	uint8_t hw_major;
	uint8_t hw_minor;
	uint8_t build_major;
	uint8_t build_minor;
} mmaVer_t;

/******************************************************************************
*				Scheduler
*******************************************************************************/
#define USER_INT_SFD		0	/* index of INTERRUPT SFD configuration */
#define USER_INT_AFE_COCO	1	/* index of INTERRUPT AFE COCO bit configuration */
#define USER_INT_IRQ		2	/* index of INTERRUPT IRQ configuration */
#define USER_INT_TPMOV		3	/* index of INTERRUPT on TPM overflow */
#define USER_INT_TPMCH0		4	/* index of INTERRUPT on TPM match CH0 register */
#define USER_INT_TPMCH1		5	/* index of INTERRUPT on TPM match CH1 register */
#define USER_INT_MTIMOV		6	/* index of INTERRUPT on MTIM register overflow */
#define USER_INT_PDBA		7	/* index of INTERRUPT on PDB match modulo value */
#define USER_INT_PDBB		8	/* index of INTERRUPT on PDB match modulo value */
#define USER_INT_MSTIIC		9	/* index of INTERRUPT MASTER I2C configuration */
#define NUMS_OF_INTERRUPTS	10	/* number of interrupts */

typedef enum {
	NEVER    =  0, // never execute app
	INACTIVE =  1, // execute app only during low activity
	ACTIVE   =  2, // execute app only during high activity
	ALWAYS   =  3  // execute app during high and low activity
} activity_t;

typedef enum {
	TASK488HZ  =  23, // start frame digital (AFE)
	TASK244HZ  =  22, // TPMOV, tpm counter overflow
	TASK122HZ  =  21, // TPMCH0, tpm channel 0
	TASK61HZ   =  20, // TPMCH1, tpm channel 1
	TASK30HZ   =  19, // TPMCH1, tpm channel 1
	TASK15HZ   =  18, // TPMCH1, tpm channel 1
	TASK7HZ    =  17, // PDBA
	TASK3HZ    =  16, // PDBB
	PRIORITY15 =  15,
	PRIORITY14 =  14,
	PRIORITY13 =  13,
	PRIORITY12 =  12,
	PRIORITY11 =  11,
	PRIORITY10 =  10,
	PRIORITY9  =  9,
	PRIORITY8  =  8,
	PRIORITY7  =  7,
	PRIORITY6  =  6,
	PRIORITY5  =  5,
	PRIORITY4  =  4,
	PRIORITY3  =  3,
	PRIORITY2  =  2,
	PRIORITY1  =  1,
	PRIORITY0  =  0, // MTIM counter overflow
	IDLE	   = -1  // idle
} priority_t;

typedef union {
	uint32_t word;
	struct {
		uint8_t	priority_idle	:1;
		uint8_t	priority0	:1;
		uint8_t	priority1	:1;
		uint8_t	priority2	:1;
		uint8_t	priority3	:1;
		uint8_t	priority4	:1;
		uint8_t	priority5	:1;
		uint8_t	priority6	:1;
		uint8_t	priority7	:1;
		uint8_t	priority8	:1;
		uint8_t	priority9	:1;
		uint8_t	priority10	:1;
		uint8_t	priority11	:1;
		uint8_t	priority12	:1;
		uint8_t	priority13	:1;
		uint8_t	priority14	:1;
		uint8_t	priority15	:1;
		uint8_t	priority16	:1;
		uint8_t	priority17	:1;
		uint8_t	priority18	:1;
		uint8_t	priority19	:1;
		uint8_t	priority20	:1;
		uint8_t	priority21	:1;
		uint8_t	priority22	:1;
		uint8_t	priority23	:1;
		uint8_t	rsvd		:7;
	}bits;
}task_status_t;

typedef union {
	uint8_t byte;
	struct {
		priority_t priority	:6; // which task to execute by the scheduler
		activity_t activity	:2; // execute during high and/or low activity
	} bits;
} sched_parms_t;

typedef struct {
	task_status_t timeoutStatus;
} mmaSchedSts_t;

typedef struct {
	task_status_t reqToStart;
	task_status_t interrupt[NUMS_OF_INTERRUPTS];
	sched_parms_t schedParms[MAX_APPID];
} mmaSchedCfg_t;

typedef struct {
	mmaSchedSts_t sts;
	mmaSchedCfg_t cfg;
} mmaSched_t;

/******************************************************************************
*				AFE (Analog Front End)
*******************************************************************************/
#define AFE_STAGE_0		0	/* index of stage 0 anti-aliased output */
#define AFE_STAGE_1		1	/* index of stage 1 anti-aliased output */
#define AFE_STAGE_0_ABS		2	/* index of stage 0 absolute value output */
#define AFE_STAGE_0_GM		3	/* index of stage 0 g-mode dependent resolution output */
#define AFE_STAGE_0_LPF		4	/* index of stage 0 configurable lowpass filter output */
#define AFE_STAGE_0_HPF		5	/* index of stage 0 configurable highpass filter output */
#define FRONTEND_OUTPUTS	6	/* number of outputs */

#define FRONTEND_X		0	/* index to x axis */
#define FRONTEND_Y		1	/* index to y axis */
#define FRONTEND_Z		2	/* index to z axis */
#define FRONTEND_AXES		3	/* number of axes */

#define AFE_CSR_FS_BIT		6	/* bit offset of Full Scale */
#define FS_2G			1	/* index to 2g Full scale */
#define FS_4G			2	/* index to 4g Full scale */
#define FS_8G			3	/* index to 8g Full scale */

#define CM_16bit		0	/* index to 16 bit conversion */
#define CM_14bit		1	/* index to 14 bit conversion */
#define CM_12bit		2	/* index to 12 bit conversion */
#define CM_10bit		3	/* index to 10 bit conversion */

#define ExtADC			1	/* index to use external signal to ADC conversion */
#define NExtADC			0	/* index to not use external signal to ADC conversion */

#define Temp			1	/* index to use temperature conversion */
#define NTemp			0	/* index to not use temperature conversion */

#define FRONTEND_CONFIG_HPF		0	/* index to highpass filter in array of configurable filters */
#define FRONTEND_CONFIG_LPF		1	/* index to lowpass filter in array of configurable filters */
#define FRONTEND_CONFIG_TEMP_LPF	2	/* index of lowpass filter for temp output in array of configurable filters*/
#define FRONTEND_CONFIG_EIC_LPF		3	/* index of lowpass filter for external input conversion in array of configurable filters*/
#define FRONTEND_CONFIG_FILTERS_PARAMS	4	/* number of configurable filters of parameter structure*/

typedef union {
	uint16_t hword;
	struct {
		uint8_t rsvd1;		/* Reserved */
		uint8_t rsvd2:	2;	/* Reserved */
		uint8_t cm:	2;	/* Conversion mode */
		uint8_t temp:	1;	/* Temperature as input */
		uint8_t extAdc:	1;	/* External ADC as input */
		uint8_t fs:	2;	/* Full scale selection */
	} bits;
} csr_t;

typedef struct {
	short acc_out[FRONTEND_OUTPUTS][FRONTEND_AXES];
 	short outputTemp;
 	short outputEIC;
 	uint16_t frameCounter;
} mmaAfeSts_t;

typedef struct {
 	csr_t csr;
	uint16_t user_offset[FRONTEND_AXES];
	uint8_t config_k[FRONTEND_CONFIG_FILTERS_PARAMS];
	uint8_t sfd_rate;
} mmaAfeCfg_t;

typedef struct {
	mmaAfeSts_t sts;
	mmaAfeCfg_t cfg;
} mmaAfe_t;

/*******************************************************************************
* 				Sleep-Wake management
*******************************************************************************/
#define SLEEPWAKE_CFG_SNCEN_MASK	0x01

typedef union{
	uint8_t Byte;
	struct {
		uint8_t stop_nc_en  	:1;
		uint8_t fle_en      	:1;
		uint8_t chnge_sched 	:1;
		uint8_t irq_int_en  	:1;
		uint8_t stop_disable	:1;
		uint8_t unused      	:3;
	} bits;
} cfg_t;

typedef union {
	uint8_t mode;
	struct {
		uint8_t MODE		:2;
		uint8_t	rsvd		:6;
	} bits;
} mmaWkSlpSts_t;

typedef struct {
	uint16_t sensitThr;
	uint16_t dozeThr;
	uint8_t longTimeOff;
	uint8_t ShortTimeOff;
	cfg_t cfg;
} mmaWkSlpCfg_t;

typedef struct {
	mmaWkSlpSts_t sts;
	mmaWkSlpCfg_t cfg;
} mmaWkSlp_t;

/*******************************************************************************
*                             Reset Suspend Clear
*******************************************************************************/
#define RESET		0	/* index of RESET Register configuration */
#define SUSPEND		1	/* index of SUSPEND Register configuration */
#define CLEAR		2	/* index of CLEAR Register configuration */
#define NUMS_OF_RSC	3	/* number of Status Registers */

typedef union {
	uint32_t word;
	struct {
		uint8_t userApp0		:1;
		uint8_t userApp1		:1;
		uint8_t UserApp2		:1;
		uint8_t UserApp3		:1;
		uint8_t UserApp4		:1;
		uint8_t UserApp5		:1;
		uint8_t rsvd1			:2;

		uint8_t EventFIFO		:1;
		uint8_t rsvd2			:1;
		uint8_t AutoWakeSleep		:1;
		uint8_t UserApp6		:1;
		uint8_t UserApp7		:1;
		uint8_t	UserApp8		:1;
		uint8_t	UserApp9		:1;
		uint8_t RSC			:1;

		uint8_t HG			:1;
		uint8_t LG			:1;
		uint8_t TapDetect		:1;
		uint8_t TiltSensing		:1;
		uint8_t UserApp10		:1;
		uint8_t UserApp11		:1;
		uint8_t FrameCounter		:1;
		uint8_t FIFO			:1;

		uint8_t GeneralRst		:1;
		uint8_t Scheduler		:1;
		uint8_t CI			:1;
		uint8_t GPIO			:1;
		uint8_t MBOX			:1;
		uint8_t rsvd			:1;
		uint8_t AFE			:1;
		uint8_t PL			:1;
	}bits;
} rsc_t;

typedef struct {
	rsc_t sr[NUMS_OF_RSC];
} mmaRscCfg_t;

/*******************************************************************************
*                                  Status
*******************************************************************************/
#define SR_0		0	/* index of Status Register configuration */
#define SR_1		1
#define SR_2		2
#define SR_3		3
#define SR_4		4
#define SR_5		5
#define SR_6		6
#define SR_7		7
#define NUMS_OF_SR	8	/* number of Status Registers */

typedef struct {
	uint8_t Byte;
	union{
		uint8_t rsvd			:6;
		uint8_t dataReady		:1;
		uint8_t commandComplete		:1;
	} bits;
} msb_t;

typedef struct {
	uint8_t byte;
	union {
		uint8_t PL_BAFRO		:2;
		uint8_t PL_LAPO			:3;
		uint8_t rsvd1			:1;
		uint8_t PL_ZtiltLockout		:1;
		uint8_t	rsvd2			:1;
	} bits;
} lsb_t;

typedef struct {
	msb_t msb;
	lsb_t lsb;
} mmaSrSts_t;

typedef struct {
    uint8_t appid;
    uint8_t output_bit_id;
} app_t;

typedef struct {
	app_t sr[NUMS_OF_SR];
} mmaSrCfg_t;

typedef struct {
	mmaSrSts_t sts;
	mmaSrCfg_t cfg;
} mmaSr_t;

/******************************************************************************
*				     MBox
*******************************************************************************/
#define MBOX4		0
#define MBOX5		1
#define MBOX6		2
#define MBOX7		3
#define MBOX8		4
#define MBOX9		5
#define MBOX10		6
#define MBOX11		7
#define MBOX12		8
#define MBOX13		9
#define MBOX14		10
#define MBOX15		11
#define MBOX16		12
#define MBOX17		13
#define MBOX18		14
#define MBOX19		15
#define MBOX20		16
#define MBOX21		17
#define MBOX22		18
#define MBOX23		19
#define MBOX24		20
#define MBOX25		21
#define MBOX26		22
#define MBOX27		23
#define MBOX28		24
#define MBOX29		25
#define MBOX30		26
#define MBOX31		27
#define NUMS_OF_MBOX	28	/* number of mailboxes */

typedef struct {
    uint8_t appid;
    uint8_t byte_id;
} mboxApp_t;

typedef struct {
	mboxApp_t mbox[NUMS_OF_MBOX];
}mmaMboxSts_t;

typedef struct {
	uint8_t byte;
	union{
		uint8_t rsvd		:1;
		uint8_t WAIT		:1;
		uint8_t UPDMODES	:2;
		uint8_t LEGACY		:1;
		uint8_t INT_O_FRAME_EN	:1;
		uint8_t INT_O_POL	:1;
		uint8_t INT_O_EN	:1;
	} bits;
}mmaMboxCfg_t;

typedef struct {
	mmaMboxSts_t sts;
	mmaMboxCfg_t cfg;
} mmaMbox_t;

/******************************************************************************
*				  GPIO
*******************************************************************************/
#define GPIO6		0	/* index of GPIO6 */
#define GPIO7		1	/* index of GPIO7 */
#define GPIO8		2	/* index of GPIO8 */
#define GPIO9		3	/* index of GPIO9 */
#define NUMS_OF_GPIO	4	/* number of LG/HG */

typedef struct {
	uint8_t appid;
	uint8_t sr_bitnum;
}gpioApp_t;

typedef union {
	uint8_t byte;
	struct {
		uint8_t gpio8_bit	:1;

		uint8_t gpio9_bit 	:1;
		uint8_t Rsvd		:6;
	} bits;
} polMsb_t;

typedef union {
	uint8_t byte;
	struct {
		uint8_t rsvd		:6;
		uint8_t gpio6_bit	:1;
		uint8_t gpio7_bit	:1;
	} bits;
} polLsb_t;

typedef struct {
	gpioApp_t app[NUMS_OF_GPIO];
	polMsb_t gpio_polMSB;
	polLsb_t gpio_polLSB;
} mmaGpioSts_t;

/******************************************************************************
*				   FIFO
*******************************************************************************/
typedef struct {
	uint8_t wmrk_flag	:1;
	uint8_t empty_flag	:1;
	uint8_t ovf_flag	:1;
	uint8_t reserved	:3;
	uint8_t on_going_pop	:1;
	uint8_t on_going_push	:1;
} fifoStsBits_t;

typedef union {
	uint8_t byte;
	fifoStsBits_t bits;
} fifoStsByte_t;

typedef struct {
	uint16_t records_number;
	uint8_t entry_size;
	fifoStsByte_t status;
} mmaFifoSts_t;

typedef struct {
	uint8_t mode		:2;
	uint8_t ch1_sz		:2;
	uint8_t rsvd		:4;
} configBits_t;

typedef union {
	configBits_t bits;
	uint8_t byte;
} config_t;

typedef struct {
	config_t config;
	uint8_t rsvd1;
	uint8_t rsvd2;
	uint8_t rsvd3;
	uint16_t sz;
	uint8_t ch1_app_id;
	uint8_t rsvd4;
	uint16_t wmrk;
} mmaDataFifoCfg_t;

typedef struct {
	uint16_t sz;
	uint16_t wmrk;
	uint16_t time_out;
} mmaEventFifoSts_t;

typedef struct dataFifo_struct_tag
{
	mmaFifoSts_t sts;
	mmaDataFifoCfg_t cfg;
} mmaDataFifo_t;

typedef struct {
	mmaFifoSts_t sts;
	mmaEventFifoSts_t cfg;
} mmaEventFifo_t;

/******************************************************************************
*				Basic Motion
*******************************************************************************/
typedef struct {
	uint8_t gpioState;	/* GPIO7 output value */
	uint8_t dataCnt;	/* number of raw data, roll over to 0 at event_cnt */
	uint8_t ZlockCnt;	/* Portrait-landscape Zlock count, roll over to 0 at event_cnt */
	uint8_t PLmixCnt;	/* Portrait-landscape count, roll over to 0 at event_cnt */
	uint8_t eventCnt;	/* Event count, roll over to 0 at event_cnt */
	uint8_t dummy;		/* gf30-07-2012 addef for 16-bit alignment issue */
	short acc_X;		/* raw x axis accelerometer */
	short acc_Y;		/* raw y axis accelerometer */
	short acc_Z;		/* raw z axis accelerometer */
	short facc_X;		/* filtered x axis accelerometer */
	short facc_Y;		/* filtered y axis accelerometer */
	short facc_Z;		/* filtered Z axis accelerometer */
} mmaBasicSts_t;

typedef struct {
	uint8_t eventCnt;	/* event_cnt, accessible from MBox */
	uint8_t cfg;		/* 1: event = g-force (Z-axis) > 0.8g; 2: event = Zlock; 3: event = Zlock & Portrait_landscape */
} mmaBasicCfg_t;

typedef struct {
	mmaBasicSts_t sts;
	mmaBasicCfg_t cfg;
} mmaBasic_t;

/******************************************************************************
*                            Gestures - HIGH G/LOW G
*******************************************************************************/
#define LG_SELCT	0	/* index of LG configuration */
#define HG_SELCT	1	/* index of HG configuration */
#define NUMS_OF_LHG	2	/* number of LG/HG */

typedef struct {
	uint8_t lgx_enable		:1;
	uint8_t lgy_enable		:1;
	uint8_t lgz_enable		:1;
	uint8_t lgAndOr_enable		:1;
	uint8_t DBCNT			:1;
	uint8_t rsvd			:3;
} lhg_cfg_bits_t;

typedef union {
	uint8_t byte;
	lhg_cfg_bits_t bits;
} lhg_cfg_t;

typedef struct {
    uint16_t lhg_thresh;
    uint8_t lhg_cnt_min;
    lhg_cfg_t lhg_cfg;
} lhg_app_t;

typedef union {
	uint8_t byte;
	struct {
		uint8_t	rsvd1		:3;
		uint8_t	FIFOLGE		:1;
		uint8_t	rsvd2		:3;
		uint8_t	FIFOHGE		:1;
	} bits;
} lhg_fifo_t;

typedef struct {
	lhg_app_t lhg_app[NUMS_OF_LHG];
	uint8_t low_high_g_K_LP;
	lhg_fifo_t lhg_fifo_mask;
} mmaLhgCfg_t;

typedef union
{
	uint8_t byte;
	struct {
		uint8_t LGX		:1;
		uint8_t LGY		:1;
		uint8_t LGZ		:1;
		uint8_t LGE		:1;
		uint8_t HGX		:1;
		uint8_t HGY		:1;
		uint8_t HGZ		:1;
		uint8_t HGE		:1;
	} bits;
} mmaLhgSts_t;

typedef struct {
	mmaLhgSts_t sts;
	mmaLhgCfg_t cfg;
} mmaLhg_t;


/******************************************************************************
*                        Gestures - PORTRAIT LANDSCAPE
*******************************************************************************/
typedef union {
	uint8_t	byte;
	struct {
		uint8_t GOFF		:3;
		uint8_t DBCNTM		:1;
		uint8_t Rsvd1		:1;
		uint8_t BKFR_EN		:1;
		uint8_t PL_EN		:1;
		uint8_t PLFDE		:1;
	} bits;
} pl_cfg_t;

typedef struct{
	uint8_t  threshold_tilt;
	uint8_t  portrait_angle;
	uint8_t  landscape_angle;
	uint8_t  debounce_count;
	uint8_t  hysteresis_LO;
	uint8_t  hysteresis_BAFRO;
	pl_cfg_t cfg;
} mmaPlCfg_t;

typedef union{
	uint8_t byte;
	struct {
		uint8_t BAFRO		:2;
		uint8_t LAPO		:3;
		uint8_t Rsvd1		:1;
		uint8_t Lock		:1;
		uint8_t Evnt 		:1;
	} bits;
} mmaPlSts_t;

typedef struct {
	mmaPlSts_t sts;
	mmaPlCfg_t cfg;
} mmaPl_t;

/******************************************************************************
*                             Gestures - TAP DTAP
*******************************************************************************/
#define TAP_SELCT			0		/* index of TAP configuration */
#define DTAP_SELECT			1		/* index of DTAP configuration */
#define NUMS_OF_TAP			2		/* number of TAP */

typedef union
{
	uint8_t byte;
	struct{
		uint8_t  X_enable	:1;
		uint8_t  Y_enable	:1;
		uint8_t  Z_enable	:1;
		uint8_t  rsvd		:5;
	} bits;
} tap_axls_enable_t;

typedef struct
{
	  int16_t tap_thresh;
	  uint8_t tap_on_min;
	  uint8_t tap_on_max;
	  uint8_t tap_tap_min_time;
	  uint8_t tap_K_HP;
	  uint8_t tap_K_LP;
	  tap_axls_enable_t tap_axis_enable;
	  uint8_t tap_events_mask;
} mmaTapCfg_t;

typedef union{
	uint8_t byte;
	struct {
		uint8_t	XEv		:1;
		uint8_t	XDir		:1;
		uint8_t	YEv		:1;
		uint8_t	YDir		:1;
		uint8_t	ZEv		:1;
		uint8_t	ZDir		:1;
		uint8_t	Rsvd		:1;
		uint8_t	TAP		:1;
	 }bits;
}TAP_DT_out_t;

typedef struct {
	TAP_DT_out_t TAP_DT_out[NUMS_OF_TAP];
} mmaTapSts_t;

typedef struct {
	mmaTapSts_t sts;
	mmaTapCfg_t cfg;
} mmaTap_t;

/******************************************************************************
*                             Gestures - TILT
*******************************************************************************/
#define XZ_PLANE		0	/* index of XZ PLANE configuration */
#define YZ_PLANE		1	/* index of YZ PLANE configuration */
#define XY_PLANE		2	/* index of XY PLANE configuration */
#define NUM_OF_PLANES		3	/* number of PLANES */

typedef union {
	uint8_t byte;
	struct{
		uint8_t AngThreshold    :4;
		uint8_t XZEn            :1;
		uint8_t YZEn            :1;
		uint8_t XYEn            :1;
		uint8_t rsvd            :1;
	} bits;
} tilt_cfg1_t;

typedef union {
	uint8_t byte;
	struct {
		uint8_t rsvd		:7;
		uint8_t change_quad_evnt:1;
	} bits;
}tilt_evnts_mask_t;

typedef struct {
	uint8_t tilt_K_LP;
	tilt_cfg1_t tilt_cfg1;
	tilt_evnts_mask_t tilt_events_mask;
} mmaTiltCfg_t;

typedef union{
	uint8_t	byte;
	struct {
		uint8_t	ANGLE		:7;
		uint8_t ANGFLG		:1;
	} bits;
} tilt_ang_t;

typedef union {
	uint8_t byte;
	struct {
		uint8_t	xy_quad 	:2;
		uint8_t	yz_quad 	:2;
		uint8_t	xz_quad 	:2;
		uint8_t	Rsvd  		:1;
		uint8_t	quadflag	:1;
	} bits;
} tilt_quad_t;

typedef struct {
	tilt_ang_t tilt_ang[NUM_OF_PLANES];
	tilt_quad_t tilt_xz_yz_quad;
} mmaTiltSts_t;

typedef struct {
	mmaTiltSts_t sts;
	mmaTiltCfg_t cfg;
} mmaTilt_t;

/******************************************************************************
*				Pedometer
*******************************************************************************/
#define PEDOMETER_STS_MRGFL_MASK		0x8000
#define PEDOMETER_STS_SUSPCHG_MASK		0x4000
#define PEDOMETER_STS_STEPCHG_MASK		0x2000
#define PEDOMETER_STS_ACTCHG_MASK		0x1000
#define PEDOMETER_STS_SUSP_MASK			0x0800
#define PEDOMETER_STS_ACTIVITY_MASK		0x0700

#define PEDOMETER_CFG_CONFIG_INIT_MASK		0x80
#define PEDOMETER_CFG_CONFIG_ACTDBCNTM_MASK	0x40
#define PEDOMETER_CFG_CONFIG_SLPDBCNTM_MASK	0x20
#define PEDOMETER_CFG_GENDER_MASK		0x80
#define PEDOMETER_CFG_GENDER_FEMALE		0
#define PEDOMETER_CFG_GENDER_MALE		1

/* activity levels */
enum {
	PEDOMETER_UNKNOWN = 0,
	PEDOMETER_REST,
	PEDOMETER_WALKING,
	PEDOMETER_JOGGING,
	PEDOMETER_RUNNING,
};

typedef struct {
	uint16_t status;
	uint16_t stepCnt;
	uint16_t distance;
	uint16_t speed;
	uint16_t calories;
	uint16_t sleepCnt;
} mmaPedSts_t;

typedef struct {
	uint16_t sleepMin;
	uint16_t sleepMax;
	uint16_t sleepThd;
	uint16_t configStepLen;
	uint16_t heightWeight;
	uint16_t filter;
	uint16_t speedPeriod;
	uint16_t actThd;
} mmaPedCfg_t;

typedef struct {
	mmaPedSts_t sts;
	mmaPedCfg_t cfg;
} mmaPed_t;

/******************************************************************************
*				Hub (K5MB specific)
*******************************************************************************/
#define HUB_AMB_BAROMETER_MODE	0
#define HUB_AMB_ALTIMETER_MODE	1

/* Gyroscope Full-Scale or ranges */
#define HUB_GYR_RANGE_250DPS	0	/* range is +-250dps; sensitivity is 8.75 m°/LSB */
#define HUB_GYR_RANGE_500DPS	1	/* range is +-500dps; sensitivity is 17.5 m°/LSB (default) */
#define HUB_GYR_RANGE_2000DPS	2	/* range is +-2000dps; sensitivity is 70 m°/LSB */

#define HUB_GYR_RANGE_MASK_BIT	4

typedef struct { /* to do */
	uint8_t gyroAddr;	/* I2C address of the gyroscope */
	uint8_t ambAddr;	/* I2C address of the P/T ambient sensors */
	short gyro_Ox;		/* x-axis angular rate Ox */
	short gyro_Oy;		/* y-axis angular rate Oy */
	short gyro_Oz;		/* z-axis angular rate Oz */
	int amb_P;		/* ambient pressure */
	short amb_T;		/* ambient temperature */
	uint16_t acc_mod;	/* acceleration vector modulus (sqrt of Gx^2 + Gy^2 + Gz^2) */
} mmaHubSts_t;

typedef struct {
	uint8_t gyroFlag;	/* set to 1 to re-init gyroscope with current config values */
	uint8_t gyroCfg;	/* to set some configuration of gyroscope */
	uint8_t ambFlag;	/* set to 1 to re-init P/T ambient sensors with current config values */
	uint8_t ambCfg;		/* to set some configuration of the P/T ambient sensors */
	char ambP_ofs;		/* resolution: 4 Pa/LSB */
	char ambH_ofs;		/* resolution: 1 m/LSB */
	char ambT_ofs;		/* resolution: 0.0625 °C/LSB */
	uint8_t dummy;
} mmaHubCfg_t;

typedef struct {
	mmaHubSts_t sts;
	mmaHubCfg_t cfg;
} mmaHub_t;

#endif /* MMA_955X_H_ */
