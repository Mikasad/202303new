#ifndef __CANOPEN_SDO_H__
#define __CANOPEN_SDO_H__

#include "canopen_od.h"
#include "canopen_objacces.h"
#include "canopen_def.h"
#include "canopen_nmt.h"
#include "canopen_timer.h"
#include "bsp_can.h"
#include "timerscfg.h"


/* MACRO DECLARE BEGIN */

#define GET_SDO_CS_CODE(m)        (m>>5)

#define getSDOcs(byte)            (byte >> 5)
#define getSDOn2(byte)            ((byte >> 2) & 3)
#define getSDOn3(byte)            ((byte >> 1) & 7)
#define getSDOe(byte)             ((byte >> 1) & 1)
#define getSDOs(byte)             (byte & 1)
#define getSDOc(byte)             (byte & 1)
#define getSDOt(byte)             ((byte >> 4) & 1)
#define getSDOindex(byte1, byte2) (((unsigned short int)byte2 << 8) | ((unsigned short int)byte1))
#define getSDOsubIndex(byte3)     (byte3)
#define getSDOblockSC(byte)       (byte & 3)
/* SDO PARA DEFINE*/
#define SDO_MAX_LENGTH_TRANSFER 				  32
#define SDO_MAX_SIMULTANEOUS_TRANSFERS 		5
#define SDO_TIMEOUT_MS 		                3000U
#define MAX_NB_TIMER 			                8
/** Status of the SDO transmission
 */
#define SDO_RESET                         0x0      /* Transmission not started. Init state. */
#define SDO_FINISHED                      0x1      /* data are available */                          
#define	SDO_ABORTED_RCV                   0x80     /* Received an abort message. Data not available */
#define	SDO_ABORTED_INTERNAL              0x85     /* Aborted but not because of an abort message. */
#define	SDO_DOWNLOAD_IN_PROGRESS          0x2 
#define	SDO_UPLOAD_IN_PROGRESS            0x3   
#define	SDO_BLOCK_DOWNLOAD_IN_PROGRESS    0x4 
#define	SDO_BLOCK_UPLOAD_IN_PROGRESS      0x5

/* MACRO DECLARE END */


unsigned char SendSDO(CO_Data *d,unsigned char whoami,unsigned char CliServNbr,unsigned char *data);
unsigned char SDO_Err_Record(CO_Data *d,unsigned char whoami,unsigned char CliServNbr,unsigned int Index,unsigned char subIndex,unsigned int EndCode);
void SDOTimeoutAlarm(CO_Data *d, unsigned int id);
void resetSDO(CO_Data *d);
unsigned int SDOlineToObjdict(CO_Data *d, unsigned char line);
unsigned int objdictToSDOline(CO_Data *d, unsigned char line);
unsigned char lineToSDO(CO_Data *d, unsigned char line, unsigned int nbBytes, unsigned char *data);
unsigned char SDOtoLine(CO_Data *d, unsigned char line, unsigned int nbBytes, unsigned char *data);
unsigned char failedSDO(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned short int index, unsigned char subIndex, unsigned int abortCode);
void resetSDOline(CO_Data *d, unsigned char line);
unsigned char initSDOline(CO_Data *d, unsigned char line, unsigned char CliServNbr, unsigned short int index, unsigned char subIndex, unsigned char state);
unsigned char getSDOfreeLine(CO_Data *d, unsigned char whoami, unsigned char *line);
unsigned char getSDOlineOnUse(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned char *line);
unsigned char getSDOlineToClose(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned char *line);
unsigned char closeSDOtransfer(CO_Data *d, unsigned char nodeId, unsigned char whoami);
unsigned char getSDOlineRestBytes(CO_Data *d, unsigned char line, unsigned int *nbBytes);
unsigned char setSDOlineRestBytes(CO_Data *d, unsigned char line, unsigned int nbBytes);
unsigned char SDO_Email_Handler(CO_Data *d,CanRxMsgTypeDef *m);

#endif
