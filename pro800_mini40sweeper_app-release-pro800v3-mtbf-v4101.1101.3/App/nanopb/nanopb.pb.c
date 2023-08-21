/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.6 at Tue Oct 25 15:14:41 2016. */

#include "nanopb.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t HeaderMessage_fields[2] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, HeaderMessage, id, id, 0),
    PB_LAST_FIELD
};

const pb_field_t CommandMessage_fields[5] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, CommandMessage, id, id, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, CommandMessage, speed, id, 0),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, CommandMessage, angle, speed, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, CommandMessage, cmd, angle, 0),
    PB_LAST_FIELD
};

const pb_field_t SpeedDataMessage_fields[8] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, SpeedDataMessage, id, id, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, SpeedDataMessage, L_cnt, id, 0),
    PB_FIELD(  3, UINT32  , REQUIRED, STATIC  , OTHER, SpeedDataMessage, R_cnt, L_cnt, 0),
    PB_FIELD(  4, INT32   , REQUIRED, STATIC  , OTHER, SpeedDataMessage, L_delta_cnt, R_cnt, 0),
    PB_FIELD(  5, INT32   , REQUIRED, STATIC  , OTHER, SpeedDataMessage, R_delta_cnt, L_delta_cnt, 0),
    PB_FIELD(  6, INT32   , REQUIRED, STATIC  , OTHER, SpeedDataMessage, speed, R_delta_cnt, 0),
    PB_FIELD(  7, INT32   , REQUIRED, STATIC  , OTHER, SpeedDataMessage, angle, speed, 0),
    PB_LAST_FIELD
};

const pb_field_t StatusDataMessage_fields[5] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, StatusDataMessage, id, id, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, StatusDataMessage, hd_status, id, 0),
    PB_FIELD(  3, UINT32  , REQUIRED, STATIC  , OTHER, StatusDataMessage, battery, hd_status, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, StatusDataMessage, charger, battery, 0),
    PB_LAST_FIELD
};

const pb_field_t SafeDataMessage_fields[6] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, SafeDataMessage, id, id, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, SafeDataMessage, ultrasonic0_3, id, 0),
    PB_FIELD(  3, UINT32  , REQUIRED, STATIC  , OTHER, SafeDataMessage, ultrasonic4_7, ultrasonic0_3, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, SafeDataMessage, ultrasonic8_11, ultrasonic4_7, 0),
    PB_FIELD(  5, UINT32  , REQUIRED, STATIC  , OTHER, SafeDataMessage, ultrasonic12_15, ultrasonic8_11, 0),
    PB_LAST_FIELD
};


/* @@protoc_insertion_point(eof) */
