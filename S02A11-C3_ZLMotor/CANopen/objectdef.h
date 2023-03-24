#ifndef __objectdef_h__
#define __objectdef_h__

/* New define, if SDO_MAX_LENGTH_TRANSFERT is exceeded allocate data buffer dynamically */
#define SDO_MAX_LENGTH_TRANSFER 					32
#define SDO_MAX_SIMULTANEOUS_TRANSFERS 		5
#define NMT_MAX_NODE_ID 	                5
#define SDO_TIMEOUT_MS 		                3000U
/* 一次最多传输sdo块数量 */
#define SDO_BLOCK_SIZE  16

/* PDO trans define */
#define PDO_MAX_LENGTH_TRANSFER 					4

/* EMCY trans define */
#define EMCY_MAX_ERRORS 8

#define REPEAT_SDO_MAX_SIMULTANEOUS_TRANSFERS_TIMES(repeat)\
repeat repeat repeat repeat repeat

#define REPEAT_EMCY_MAX_ERRORS_TIMES(repeat)\
repeat repeat repeat repeat repeat repeat repeat repeat

/* CANopen Status Init */
#define s_transfer_Initializer {\
		0,          /* nodeId */\
		0,          /* wohami */\
		SDO_RESET,  /* state */\
		0,          /* toggle */\
		0,          /* abortCode */\
		0,          /* index */\
		0,          /* subIndex */\
		0,          /* count */\
		0,          /* offset */\
		{0},        /* data (static use, so that all the table is initialize at 0)*/\
		0,          /* peerCRCsupport */\
		0,          /* blksize */\
		0,          /* ackseq */\
		0,          /* objsize */\
		0,          /* lastblockoffset */\
		0,          /* seqno */\
		0,          /* endfield */\
		RXSTEP_INIT,/* rxstep */\
		{0},        /* tmpData */\
		0,          /*  */\
		-1,         /*  */\
		NULL        /*  */\
	  },

#define ERROR_DATA_INITIALIZER \
	{\
	0, /* errCode */\
	0, /* errRegMask */\
	0 /* active */\
	},

#define MSG_DATA_INITIALIZER \
  {\
    0,      /* StdId */\
    0,      /* ExtId */\
    0,      /* IDE */\
    0,      /* RTR */\
    0,      /* DLC */\
    {       /* Data[8] */\
      0,          \
      0,          \
      0,          \
      0,          \
      0,          \
      0,          \
      0,          \
      0,          \
    }\
  },

typedef struct S_QUICK_INDEX_STRUCT
{
    unsigned short int  SDO_SVR;      //SDO服务器参数ID
    unsigned short int  SDO_CLT;      //SDO客户端参数ID
    unsigned short int  PDO_RCV;      //RPDO通信参数ID
    unsigned short int  PDO_RCV_MAP;  //RPDO映射参数ID
    unsigned short int  PDO_TRS;      //TPDO通信参数ID
    unsigned short int  PDO_TRS_MAP;  //TPDO映射参数ID
} Quick_Index_Status;


#endif
