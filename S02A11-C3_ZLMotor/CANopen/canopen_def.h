#ifndef __CANOPEN_DEF_H__
#define __CANOPEN_DEF_H__

#define UNS16_LE(v)  (v)
#define UNS32_LE(v)  (v)

/*OD_Macro_Define*/
#define SLAVE_NODE_NUM_MAX         0x7F
#define SLAVE_NODE_NUM_MIN         0x00
#define OD_ADDR_MAX                0xFFFF
#define OD_ADDR_MIN                0x0000
#define OD_NUM                     0
#define OD_FIRST_NUM               0
#define OD_LAST_NUM                OD_NUM
#define OBJ_ADD_NUM                0
#define CAN_ID_BIT                 0x7FF //11bit

#define BIT_VALID        1
#define BIT_INVALID      0
#define RTR_VALID        2
#define RTR_INVALID      0

#define STORE_PARA_SIGNATURE   0x65766173 //"e""v""a""s"
//Permison CanBandrate(Kbps)
#define CAN_BANDRATE_50K    50
#define CAN_BANDRATE_100K   100
#define CAN_BANDRATE_125K   125
#define CAN_BANDRATE_200K   200
#define CAN_BANDRATE_250K   250
#define CAN_BANDRATE_500K   500
#define CAN_BANDRATE_1000K  1000
//STD/EX
#define EX_CANID          0x1FFFFFFF
#define STD_CANID         0x000007FF
//LIMIT
#define LIMIT_COUNTER    0xFFFF
//Addr Index range
#define NO_USED_ADDR                    0x0000 //不使用
#define STATIC_DATA_TYPE_BEGIN          0x0001 //静态数据类型
#define STATIC_DATA_TYPE_END            0x001F
#define COMPLEX_DATA_TYPE_BEGIN         0x0020 //复杂数据类型
#define COMPLEX_DATA_TYPE_END           0x003F
#define MANUFACTURER_COMPLEX_BEGIN      0x0040 //制造商定义的复杂数据类型
#define MANUFACTURER_COMPLEX_END        0x005F
#define DEVICE_STANDARD_STATIC_BEGIN    0x0060 //设备规范定义的静态数据类型
#define DEVICE_STANDARD_STATIC_END      0x007F
#define DEVICE_STANDARD_COMPLEX_BEGIN   0x0080 //设备规范定义的复杂数据类型
#define DEVICE_STANDARD_COMPLEX_END     0x009F
#define RESERVED1_BEGIN                 0x00A0 //保留
#define RESERVED1_END                   0x0FFF
#define COMUNICATION_STANDARD_BEGIN     0x1000 //通讯规范区域(DS301)
#define COMUNICATION_STANDARD_END       0x1FFF
#define DEVICE_MANUFACTURER_BEGIN       0x2000 //设备制造商相关区域
#define DEVICE_MANUFACTURER_END         0x5FFF
#define STANDARD_DEVICE_BEGIN           0x6000 //标准化的设备规范区域
#define STANDARD_DEVICE_END             0x9FFF
#define PORT_STANDARD_BEGIN             0xA000 //接口规范说明区域
#define PORT_STANDARD_END               0xBFFF
#define RESERVED2_BEGIN                 0xC000 //保留
#define RESERVED2_END                   0xFFFF

#define PDO_MAP_BEGIN                   0x1A00 //PDO映射参数地址
#define PDO_MAP_END                     0x1A03 //
//Access Type Define
#define RW     0x00
#define WO     0x01
#define RO     0x02
#define CONST  0x04       //只读且为常量，该值可以在NMT初始化态更改，其余状态不可变更
#define TO_BE_SAVE  0x08
#define DCF_TO_SEND 0x10

#define SIZE_UINT8      1
#define SIZE_UINT16     2
#define SIZE_UINT32     4
#define SIZE_INTEGER8   1
#define SIZE_INTEGER16  2
#define SIZE_INTEGER32  4
#define SIZE_LONG       4
#define SIZE_FLOAT      4
//dataType define
#define BOLLEAN             0x00
#define NIL                 0x01
#define UNSIGNED8           0x02
#define UNSIGNED16          0x03
#define UNSIGNED24          0x04
#define UNSIGNED32          0x05
#define UNSIGNED40          0x06
#define UNSIGNED48          0x07
#define UNSIGNED56          0x08
#define INTEGER8            0x09
#define INTEGER16           0x0A
#define INTEGER24           0x0B
#define INTEGER32           0x0C
#define INTEGER40           0x0D
#define INTEGER48           0x0E
#define INTEGER56           0x0F
#define INTEGER64           0x10
#define REAL32              0x11
#define REAL364             0x12
#define STRUCT_OF           0x13
#define UNSIGNED8_STRING    0x14
#define UNSIGNED16_STRING   0x15

/*SDO_Define begin*/
#define SDO_COMUNICATION_PARA_INDEX   0x1200
//SDO命令字CCS(byte0高3位)  主站->从站
#define INIT_DOWNLOAD_REQUEST         0x1   //初始化下载         
#define SEGMENT_DOWNLOAD_REQUEST      0x0   //分段下载
#define INIT_UPLOAD_REQUEST           0x2   //初始化上传
#define SEGMENT_UPLOAD_REQUEST        0x3   //分段上传
#define ABORT_TRANSER_REQUEST         0x4   //终止码传输
//SDO命令字SCS(byte0高3位)  从站->主站
#define INIT_DOWNLOAD_RESPOND         0x3
#define SEGMENT_DOWNLOAD              0x1
#define INIT_UPLOAD                   0x2
#define SEGMENT_UPLOAD                0x0
#define ABORT_TRANSER                 0x4
//Trans Type ExDefine
#define TYPE_INIT_DOWNLOAD_REQ        0x1
#define TYPE_SEGMENT_DOWNLOAD_REQ     0x0
#define TYPE_INIT_UPLOAD_REQ          0x2
#define TYPE_SEGMENT_UPLOAD_REQ       0x3
#define TYPE_ABORT_TRANSER_REQ        0x4
#define TYPE_SLAVE_ABORT_ACTIVE       0x5
//SDO SERVER_COB-ID
#define VALID_BIT_SET     0x80000000
#define VALID_BIT_CLR     0x7FFFFFFF
#define DYN_BIT_SET       0x40000000
#define DYN_BIT_CLR       0xBFFFFFFF
#define FRAME_BIT_SET     0x20000000
#define FRAME_BIT_CLR     0xDFFFFFFF

//SDO_cmd_bit
#define TRIG_BIT_SET     0x10 //触发位
#define TRIG_BIT_CLR     0xEF

#define E_BIT_SET        0x02 //传输类型 0-正常传送 1-加速传送
#define E_BIT_CLR        0xFD

#define S_BIT_SET        0x01 //是否指明数据长度 0-长度未指明 1-长度指明(表示长度大于4Byte，需分段传送)
#define S_BIT_CLR        0xFE

#define C_BIT_SET        0x01//是否有后续分段数据
#define C_BIT_CLR        0xFE
//SDO MessType
#define MessType_RSDO		 0x02
#define MessType_TSDO		 0x01

//节点在SDO传输过程中的状态
#define SDO_SERVER       0x01
#define SDO_CLIENT       0x02
#define SDO_UNKNOWN      0x03

//SDO trans lenth
#define NORMAL_LEN_SDO   4   //SDO通用数据长度
#define SEGMENT_LEN_MAX_SDO 7 //分段传输单次最大容量
//SDO中止代码(未列出的中止代码应保留)
/* SDO Define end */

/*OD_Define访问对象字典状态*/
#define NO_RESULT                    0x0
#define OD_SUCCESSFUL 	             0x00000000
#define OD_READ_NOT_ALLOWED          0x06010001
#define OD_WRITE_NOT_ALLOWED         0x06010002
#define OD_NO_SUCH_OBJECT            0x06020000
#define OD_NOT_MAPPABLE              0x06040041
#define OD_LENGTH_DATA_INVALID       0x06070010
#define OD_NO_SUCH_SUBINDEX 	       0x06090011
#define OD_VALUE_RANGE_EXCEEDED      0x06090030 /* Value range test result */
#define OD_VALUE_TOO_LOW             0x06090031 /* Value range test result */
#define OD_VALUE_TOO_HIGH            0x06090032 /* Value range test result */
/* Others SDO abort codes
 */
#define SDOABT_TOGGLE_NOT_ALTERNED   0x05030000
#define SDOABT_TIMED_OUT             0x05040000
#define SDOABT_OUT_OF_MEMORY         0x05040005 /* Size data exceed SDO_MAX_LENGTH_TRANSFER */
#define SDOABT_GENERAL_ERROR         0x08000000 /* Error size of SDO message */
#define SDOABT_LOCAL_CTRL_ERROR      0x08000021



#define DEVICE_END_CODE_SUPPORT  //Support

#ifdef DEVICE_END_CODE_SUPPORT

#define INVER_BIT_NOT_CHANGE                  0x05030000   //翻转位未变化
#define SERVER_CLIENT_CMD_UNKNOWN             0x05040001   //SDO命令字错误
#define WRITE_ONLY_READ                       0x06010002   //试图写入只读对象
#define NOT_EXIST_IN_OD                       0x06020000   //对象不存在
#define CANNOT_MAP_PDO                        0x06040041   //对象不能被映射进PDO(修改PDO映射参数时)
#define DATATYPE_NOMATCH_SERVICE_LEN_NOMATCH  0x06070010   //数据类型错误
#define SUB_INDEX_NOT_EXIST                   0x06090011   //子索引不存在
#define WRITE_PARA_TOOHIGH_DOWNLOAD_ONLY      0x06090031   //写入参数值太大(仅下载)
#define WRITE_PARA_TOOLOW_DOWNLOAD_ONLY       0x06090032   //写入参数值太小(仅下载)
#define GENERAL_ERROR                         0x08000000   //常规错误
#define ACCESS_FAILED_DUE_TO_HARDWARE         0x06060000   //硬件错误导致的访问失败
#define LENGTH_NUMBER_BEYOND_PDO_LEN          0x06040042   //
#define GEERAL_PARA_INCOMPATIBILITY_REASON    0x06040043   //常规参数不兼容额定原因
#else
#define INVER_BIT_NOT_CHANGE                  0x05030000   //翻转位未变化
#define SDO_AGREEMENT_OUTTIME                 0x05040000   //SDO协议超时
#define SERVER_CLIENT_CMD_UNKNOWN             0x05040001   //客户端/服务器命令无效或未知
#define INVALID_BLOCK_SIZE                    0x05040002   //无效的块大小
#define INVALID_SERIAL_NUMBER                 0x05040003   //无效的序列号
#define CRC_ERR                               0x05040004   //CRC错误(仅块错误)
#define OUT_OF_MEMORY                         0x05040005   //内存不足
#define ACCESS_UNSUPPORTEED                   0x06010000   //不支持的访问对象
#define READ_ONLY_WRITE                       0x06010001   //试图读取只写对象
#define WRITE_ONLY_READ                       0x06010002   //试图写入只读对象
#define NOT_EXIST_IN_OD                       0x06020000   //对象不存在与OD
#define CANNOT_MAP_PDO                        0x06040041   //对象不能被映射进PDO
#define LENGTH_NUMBER_BEYOND_PDO_LEN          0x06040042   //对象长度和数量超出PDO长度
#define GEERAL_PARA_INCOMPATIBILITY_REASON    0x06040043   //常规参数不兼容额定原因
#define DEVICE_INTERNAL_INCOMPATIBILITY       0x06040047   //设备内部不兼容
#define DATATYPE_NOMATCH_SERVICE_LEN_NOMATCH  0x06070010   //数据类型不匹配，服务长度参数不匹配
#define DATATYPE_NOMATCH_SERVICE_LEN_TOOHIGH  0x06070012   //数据类型不匹配，服务长度参数太大
#define DATATYPE_NOMATCH_SERVICE_LEN_TOOLOW   0x06070013   //数据类型不匹配，服务长度参数太小
#define SUB_INDEX_NOT_EXIST                   0x06090011   //子索引不存在
#define INVALID_PARA_DOWNLOAD_ONLY            0x06090030   //无效的参数值(仅下载)
#define WRITE_PARA_TOOHIGH_DOWNLOAD_ONLY      0x06090031   //写入参数值太高(仅下载)
#define WRITE_PARA_TOOLOW_DOWNLOAD_ONLY       0x06090032   //写入参数值太低(仅下载)
#define MAXIMUM_VALUE_LESS_THAN_MINIMUM       0x06090036   //最大值小于最小值
#define RESOURCE_NOT_ACCESS_SDO_CONNECT       0x060A0023   //资源不可用：SDO连接
#define GENERAL_ERROR                         0x08000000   //常规错误
#define DATA_CANNOT_TRANS_STORED_TO_APP       0x08000020   //数据不能被传输或保存到应用程序
#define DATA_CANNOT_TRANS_STORED_BECAUSE_LOCAL_CTR 0x08000021   //数据不能被传输或保存到应用程序，由于本地的控制
#define DATA_CANNOT_TRANS_STORED_BECAUSE_PRESENT_DEVICE_STATE 0x08000022   //数据不能被传输或保存到应用程序，因为当前设备状态
#define OD_DYNAMIC_GENERATION_FAIL_OR_OD_NOTEXIST 0x08000023   //对象字典的动态生成失败或无对象字典的存在(例如对象字典是从文件生成，而由于文件的出错误生成失败)
#define NO_DATA_AVAILABLE                     0x080000241   //无可用数据

#endif
/*PDO_Define*/
#define PDO_COMUNICATION_PARA_INDEX   0x1800  //通讯参数索引
#define PDO_MAPPING_PARA_INDEX        0x1A00  //映射参数索引
//允许PDO映射
#define PDO_ALLOW 1
#define PDO_BAN   0
//TPDO/RPDO标识符
#define PDO_EXIST_BIT_SET       0x7FFFFFFF
#define PDO_EXIST_BIT_CLR       0x80000000
#define PDO_EXIST_BIT_DEFLT     PDO_EXIST_BIT_SET

#define PDO_ALLOW_RTR_BIT_SET   0xBFFFFFFF
#define PDO_ALLOW_RTR_BIT_CLR   0x40000000
#define PDO_ALLOW_RTR_DEFLT     PDO_ALLOW_RTR_BIT_SET

#define PDO_20A_BIT_SET         0xDFFFFFFF
#define PDO_20B_BIT_SET         0x20000000
#define PDO_ID_STANDARD_DEFLT   PDO_20A_BIT_SET
//PDO传输类型
#define SYNC_NOCYCLE      0     //同步 非循环[收到一个同步信号后就进行一次PDO传输]
#define SYNC_CYCLE_BEGIN  1     //同步 循环起始[收到N个同步信号后进行一次PDO传输]
#define SYNC_CYCLE_END    240   //同步 循环结束
#define RESERVED_BEGIN    241   //保留起始
#define RESERVED_END      251   //保留结束
#define SYNC_RTR_ONLY     252   //同步 仅远程传送
#define ASYNC_TR_ONLY     253   //异步 仅远程传送[收到一个远程帧后进行一次PDO传输]
#define ASYNC_TRIGGER_A   254   //异步 制造商特定事件
#define ASYNC_TRIGGER_B   255   //异步 设备子协议特定事件

#define PDO_MAP_NUMBER_MAX 0x8 //0x40
//PDO MAP PARA QUICK INDEX
#define MAP_INDEX_OD_ADDR 0     //映射参数-OD地址
#define MAP_INDEX_OD_SUB  1     //映射参数-OD地址对应下标
//PDO trans lenth
#define PDO_TRANSLEN_MAX  8

/* constantes used in the different state machines */
/* ----------------------------------------------- */
/* Must not be modified */
#define state1  0x01
#define state2  0x02
#define state3  0x03
#define state4  0x04
#define state5  0x05
#define state6  0x06
#define state7  0x07
#define state8  0x08
#define state9  0x09
#define state10 0x0A
#define state11 0x0B

/*DS402*/
//Motor Type
#define NON_STANDERD_MOTOR            0x0000
#define PHASE_MODULATED_DC_MOTOR      0x0001
#define FRAQ_CONTROLLED_DC_MOTOR      0x0002
#define PM_SYNC_MOTOR                 0x0003
#define FC_SYNC_MOTOR                 0x0004
#define SWITCH_RELUCTANCE_MOTOR       0x0005
#define WOUND_ROTOR_INDUCTION_MOTOR   0x0006
#define SQUIREL_CAGE_INDUCTION_MOTOR  0x0007
#define STEPPER_MOTOR                 0x0008
#define MICRO_STEP_STEPPER_MOTOR      0x0009
#define SINUSOIDAL_PM_BL_MOTOR        0x000A
#define TRAPEZOIDAL_PM_BL_MOTOR       0x000B
#define MOTOR_TYPE_RESERVEDS          0x000C
//Supported Driver mode
#define DRIVER_PP          (1<<0)
#define DRIVER_VL          (1<<1)
#define DRIVER_PV          (1<<2)
#define DRIVER_TQ          (1<<2)
#define DRIVER_RESERVED_0  (0<<3)
#define DRIVER_HM          (1<<4)
#define DRIVER_IP          (1<<5)
#define DRIVER_RESERVED_1  ((0x00<<7)|(0<<6))
#define DRIVER_MANUFACTUREER_SPECIFIC  (0x0006<<16)
//Motion profile type
#define LINEAR_RAMP                 0x0000
#define SIN2_RAMP                   0x0001
#define JERK_FREE_RAMP              0x0002
#define JERK_LIMITED_RAMP           0x0003

#endif
/* This is End */
