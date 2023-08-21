/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.7 at Tue Sep 11 10:43:55 2018. */

#ifndef PB_SERVICE_PB_H_INCLUDED
#define PB_SERVICE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _AnalogAntiCollisionData {
    uint32_t id;
    pb_callback_t data;
    pb_callback_t trigger_data;
/* @@protoc_insertion_point(struct:AnalogAntiCollisionData) */
} AnalogAntiCollisionData;

typedef struct _AntiDropData {
    uint32_t id;
    pb_callback_t data;
    pb_callback_t trigger_data;
/* @@protoc_insertion_point(struct:AntiDropData) */
} AntiDropData;

typedef struct _BatteryData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:BatteryData) */
} BatteryData;

typedef struct _CANBusCommand {
    uint32_t id;
    uint32_t rtr;
    uint32_t ide;
    uint32_t can_id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:CANBusCommand) */
} CANBusCommand;

typedef struct _CANBusStatusData {
    uint32_t id;
    uint32_t rtr;
    uint32_t ide;
    uint32_t can_id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:CANBusStatusData) */
} CANBusStatusData;

typedef struct _CameraPTZCommand {
    uint32_t id;
    uint32_t command_id;
    uint32_t data_1;
    uint32_t data_2;
/* @@protoc_insertion_point(struct:CameraPTZCommand) */
} CameraPTZCommand;

typedef struct _ClearErrorCommand {
    uint32_t id;
    pb_callback_t bits;
/* @@protoc_insertion_point(struct:ClearErrorCommand) */
} ClearErrorCommand;

typedef struct _ConfigCommand {
    uint32_t id;
    pb_callback_t config;
/* @@protoc_insertion_point(struct:ConfigCommand) */
} ConfigCommand;

typedef struct _DeviceCommand {
    uint32_t id;
    pb_callback_t command;
/* @@protoc_insertion_point(struct:DeviceCommand) */
} DeviceCommand;

typedef struct _DeviceData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:DeviceData) */
} DeviceData;

typedef struct _DriverConfigCommand {
    uint32_t id;
    pb_callback_t config;
/* @@protoc_insertion_point(struct:DriverConfigCommand) */
} DriverConfigCommand;

typedef struct _HaideDeviceCommand {
    uint32_t id;
    pb_callback_t command;
/* @@protoc_insertion_point(struct:HaideDeviceCommand) */
} HaideDeviceCommand;

typedef struct _HaideDeviceData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:HaideDeviceData) */
} HaideDeviceData;

typedef struct _HaideMotionData {
    uint32_t id;
    int32_t speed;
    int32_t angle;
/* @@protoc_insertion_point(struct:HaideMotionData) */
} HaideMotionData;

typedef struct _HardwareDriverData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:HardwareDriverData) */
} HardwareDriverData;

typedef struct _HeaderMessage {
    uint32_t id;
/* @@protoc_insertion_point(struct:HeaderMessage) */
} HeaderMessage;

typedef struct _HealthData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:HealthData) */
} HealthData;

typedef struct _IMUData {
    uint32_t id;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
    bool has_angular_velocity_x;
    int32_t angular_velocity_x;
    bool has_angular_velocity_y;
    int32_t angular_velocity_y;
    bool has_angular_velocity_z;
    int32_t angular_velocity_z;
    bool has_linear_acceleration_x;
    int32_t linear_acceleration_x;
    bool has_linear_acceleration_y;
    int32_t linear_acceleration_y;
    bool has_linear_acceleration_z;
    int32_t linear_acceleration_z;
    bool has_magnetic_field_x;
    int32_t magnetic_field_x;
    bool has_magnetic_field_y;
    int32_t magnetic_field_y;
    bool has_magnetic_field_z;
    int32_t magnetic_field_z;
/* @@protoc_insertion_point(struct:IMUData) */
} IMUData;

typedef struct _MCUDebugData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:MCUDebugData) */
} MCUDebugData;

typedef struct _MCUUpgradeCommand {
    uint32_t id;
    int32_t time;
    uint32_t num;
    uint32_t check_code;
    pb_callback_t command;
/* @@protoc_insertion_point(struct:MCUUpgradeCommand) */
} MCUUpgradeCommand;

typedef struct _MCUUpgradeStatusData {
    uint32_t id;
    uint32_t status;
/* @@protoc_insertion_point(struct:MCUUpgradeStatusData) */
} MCUUpgradeStatusData;

typedef struct _ManualModeDeviceCommand {
    uint32_t id;
    pb_callback_t command;
/* @@protoc_insertion_point(struct:ManualModeDeviceCommand) */
} ManualModeDeviceCommand;

typedef struct _MotionControlMotorCommand {
    uint32_t id;
    pb_callback_t command;
/* @@protoc_insertion_point(struct:MotionControlMotorCommand) */
} MotionControlMotorCommand;

typedef struct _MotionControlMotorCommand_MotorCommandPair {
    uint32_t id;
    int32_t command;
/* @@protoc_insertion_point(struct:MotionControlMotorCommand_MotorCommandPair) */
} MotionControlMotorCommand_MotorCommandPair;

typedef struct _MotionPositionCammand {
    uint32_t id;
    int32_t left_position;
    int32_t right_position;
    int32_t brake_cmd;
/* @@protoc_insertion_point(struct:MotionPositionCammand) */
} MotionPositionCammand;

typedef struct _MotorDriverConfigCommand {
    uint32_t id;
    pb_callback_t config;
/* @@protoc_insertion_point(struct:MotorDriverConfigCommand) */
} MotorDriverConfigCommand;

typedef struct _MotorDriverConfigData {
    uint32_t id;
    pb_callback_t config;
/* @@protoc_insertion_point(struct:MotorDriverConfigData) */
} MotorDriverConfigData;

typedef struct _OdomRecorderData {
    uint32_t id;
    uint32_t odom_x;
    uint32_t odom_y;
    uint32_t odom_w;
    uint32_t dx;
    uint32_t x;
    uint32_t y;
    uint32_t w;
/* @@protoc_insertion_point(struct:OdomRecorderData) */
} OdomRecorderData;

typedef struct _OdometryData {
    uint32_t id;
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta_count;
    int32_t right_delta_count;
/* @@protoc_insertion_point(struct:OdometryData) */
} OdometryData;

typedef struct _OtherDeviceData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:OtherDeviceData) */
} OtherDeviceData;

typedef struct _DriverData {
    uint32_t id;
    pb_callback_t data;
} DriverData;

typedef struct _ProtectorData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:ProtectorData) */
} ProtectorData;

typedef struct _PhantasMcuTypeData {
    uint32_t id;
    pb_callback_t mcu_types;
/* @@protoc_insertion_point(struct:PhantasMcuTypeData) */
} PhantasMcuTypeData;

typedef struct _PhantasMcuUpgradeCommand {
    uint32_t id;
    uint32_t target;
    uint32_t command_type;
    uint32_t total_num;
    uint32_t total_size;
/* @@protoc_insertion_point(struct:PhantasMcuUpgradeCommand) */
} PhantasMcuUpgradeCommand;

typedef struct _PhantasMcuUpgradeCommandStatusData {
    uint32_t id;
    uint32_t status;
    uint32_t command_type;
    uint32_t target;
/* @@protoc_insertion_point(struct:PhantasMcuUpgradeCommandStatusData) */
} PhantasMcuUpgradeCommandStatusData;

typedef struct _PhantasMcuUpgradePackage {
    uint32_t id;
    uint32_t target;
    uint32_t sequence;
    uint32_t crc;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:PhantasMcuUpgradePackage) */
} PhantasMcuUpgradePackage;

typedef struct _PhantasMcuUpgradePackageStatusData {
    uint32_t id;
    uint32_t status;
    uint32_t target;
    uint32_t sequence;
/* @@protoc_insertion_point(struct:PhantasMcuUpgradePackageStatusData) */
} PhantasMcuUpgradePackageStatusData;

typedef struct _RFIDData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:RFIDData) */
} RFIDData;

typedef struct _RFIDData_RFIDStruct {
    uint32_t id;
    uint32_t data;
    uint32_t strength;
/* @@protoc_insertion_point(struct:RFIDData_RFIDStruct) */
} RFIDData_RFIDStruct;

typedef struct _RemoteCommand433 {
    uint32_t id;
    pb_callback_t command;
/* @@protoc_insertion_point(struct:RemoteCommand433) */
} RemoteCommand433;

typedef struct _RemoteData433 {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:RemoteData433) */
} RemoteData433;

typedef struct _SaberIMUData {
    uint32_t id;
    uint32_t acc_x;
    uint32_t acc_y;
    uint32_t acc_z;
    uint32_t gyro_x;
    uint32_t gyro_y;
    uint32_t gyro_z;
    uint32_t roll;
    uint32_t pitch;
    uint32_t yaw;
/* @@protoc_insertion_point(struct:SaberIMUData) */
} SaberIMUData;

typedef struct _SaveMotorDriverConfigCommand {
    uint32_t id;
/* @@protoc_insertion_point(struct:SaveMotorDriverConfigCommand) */
} SaveMotorDriverConfigCommand;

typedef struct _ShaoTestData {
    uint32_t id;
    int32_t angular_velocity;
    int32_t linear_velocity;
/* @@protoc_insertion_point(struct:ShaoTestData) */
} ShaoTestData;

typedef struct _ShutdownCommand {
    uint32_t id;
    uint32_t type;
/* @@protoc_insertion_point(struct:ShutdownCommand) */
} ShutdownCommand;

typedef struct _UInt32Pair {
    uint32_t key;
    uint32_t value;
/* @@protoc_insertion_point(struct:UInt32Pair) */
} UInt32Pair;

typedef struct _UWBData {
    uint32_t id;
    uint32_t x;
    uint32_t y;
    uint32_t z;
/* @@protoc_insertion_point(struct:UWBData) */
} UWBData;

typedef struct _UWBDistanceData {
    uint32_t id;
    uint32_t tag_num;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:UWBDistanceData) */
} UWBDistanceData;

typedef struct _UWBDistanceData_UWBDistanceStruct {
    uint32_t id;
    uint32_t data;
/* @@protoc_insertion_point(struct:UWBDistanceData_UWBDistanceStruct) */
} UWBDistanceData_UWBDistanceStruct;

typedef struct _UltrasonicData {
    uint32_t id;
    pb_callback_t data;
/* @@protoc_insertion_point(struct:UltrasonicData) */
} UltrasonicData;

typedef struct _VerifyMessage {
    uint32_t id;
    uint32_t data;
/* @@protoc_insertion_point(struct:VerifyMessage) */
} VerifyMessage;

typedef struct _VersionData {
    uint32_t id;
    pb_callback_t version;
/* @@protoc_insertion_point(struct:VersionData) */
} VersionData;

typedef struct _VersionData_VersionStruct {
    uint32_t board;
    uint32_t type;
    uint32_t major;
    uint32_t minor;
    uint32_t build;
/* @@protoc_insertion_point(struct:VersionData_VersionStruct) */
} VersionData_VersionStruct;

/* Default values for struct fields */

/* Initializer values for message structs */
#define HeaderMessage_init_default               {0}
#define UInt32Pair_init_default                  {0, 0}
#define VerifyMessage_init_default               {0, 0}
#define ShutdownCommand_init_default             {0, 0}
#define ConfigCommand_init_default               {0, {{NULL}, NULL}}
#define DriverConfigCommand_init_default         {0, {{NULL}, NULL}}
#define MotionControlMotorCommand_init_default   {0, {{NULL}, NULL}}
#define MotionControlMotorCommand_MotorCommandPair_init_default {0, 0}
#define DeviceCommand_init_default               {0, {{NULL}, NULL}}
#define ManualModeDeviceCommand_init_default     {0, {{NULL}, NULL}}
#define RemoteCommand433_init_default            {0, {{NULL}, NULL}}
#define ClearErrorCommand_init_default           {0, {{NULL}, NULL}}
#define SaveMotorDriverConfigCommand_init_default {0}
#define MotorDriverConfigCommand_init_default    {0, {{NULL}, NULL}}
#define MCUUpgradeCommand_init_default           {0, 0, 0, 0, {{NULL}, NULL}}
#define CameraPTZCommand_init_default            {0, 0, 0, 0}
#define MCUUpgradeStatusData_init_default        {0, 0}
#define VersionData_init_default                 {0, {{NULL}, NULL}}
#define VersionData_VersionStruct_init_default   {0, 0, 0, 0, 0}
#define HealthData_init_default                  {0, {{NULL}, NULL}}
#define DeviceData_init_default                  {0, {{NULL}, NULL}}
#define OdometryData_init_default                {0, 0, 0, 0, 0}
#define IMUData_init_default                     {0, 0, 0, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define SaberIMUData_init_default                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define UltrasonicData_init_default              {0, {{NULL}, NULL}}
#define ProtectorData_init_default               {0, {{NULL}, NULL}}
#define AnalogAntiCollisionData_init_default     {0, {{NULL}, NULL}, {{NULL}, NULL}}
#define AntiDropData_init_default                {0, {{NULL}, NULL}, {{NULL}, NULL}}
#define UWBData_init_default                     {0, 0, 0, 0}
#define UWBDistanceData_init_default             {0, 0, {{NULL}, NULL}}
#define UWBDistanceData_UWBDistanceStruct_init_default {0, 0}
#define RFIDData_init_default                    {0, {{NULL}, NULL}}
#define RFIDData_RFIDStruct_init_default         {0, 0, 0}
#define RemoteData433_init_default               {0, {{NULL}, NULL}}
#define MotorDriverConfigData_init_default       {0, {{NULL}, NULL}}
#define DriverData_init_zero                     {0, {{NULL}, NULL}}
#define HardwareDriverData_init_default          {0, {{NULL}, NULL}}
#define OdomRecorderData_init_default            {0, 0, 0, 0, 0, 0, 0, 0}
#define MCUDebugData_init_default                {0, {{NULL}, NULL}}
#define BatteryData_init_default                 {0, {{NULL}, NULL}}
#define ShaoTestData_init_default                {0, 0, 0}
#define MotionPositionCammand_init_default       {0, 0, 0, 0}
#define OtherDeviceData_init_default             {0, {{NULL}, NULL}}
#define HaideDeviceCommand_init_default          {0, {{NULL}, NULL}}
#define HaideMotionData_init_default             {0, 0, 0}
#define HaideDeviceData_init_default             {0, {{NULL}, NULL}}

#define CANBusCommand_init_default               {0, 0, 0, 0, {{NULL}, NULL}}
#define CANBusStatusData_init_default            {0, 0, 0, 0, {{NULL}, NULL}}
#define PhantasMcuUpgradeCommand_init_default    {0, 0, 0, 0, 0}
#define PhantasMcuUpgradePackage_init_default    {0, 0, 0, 0, {{NULL}, NULL}}
#define PhantasMcuUpgradeCommandStatusData_init_default {0, 0, 0, 0}
#define PhantasMcuUpgradePackageStatusData_init_default {0, 0, 0, 0}
#define PhantasMcuTypeData_init_default          {0, {{NULL}, NULL}}

#define HeaderMessage_init_zero                  {0}
#define UInt32Pair_init_zero                     {0, 0}
#define VerifyMessage_init_zero                  {0, 0}
#define ShutdownCommand_init_zero                {0, 0}
#define ConfigCommand_init_zero                  {0, {{NULL}, NULL}}
#define DriverConfigCommand_init_zero            {0, {{NULL}, NULL}}
#define MotionControlMotorCommand_init_zero      {0, {{NULL}, NULL}}
#define MotionControlMotorCommand_MotorCommandPair_init_zero {0, 0}
#define DeviceCommand_init_zero                  {0, {{NULL}, NULL}}
#define ManualModeDeviceCommand_init_zero        {0, {{NULL}, NULL}}
#define RemoteCommand433_init_zero               {0, {{NULL}, NULL}}
#define ClearErrorCommand_init_zero              {0, {{NULL}, NULL}}
#define SaveMotorDriverConfigCommand_init_zero   {0}
#define MotorDriverConfigCommand_init_zero       {0, {{NULL}, NULL}}
#define MCUUpgradeCommand_init_zero              {0, 0, 0, 0, {{NULL}, NULL}}
#define CameraPTZCommand_init_zero               {0, 0, 0, 0}
#define MCUUpgradeStatusData_init_zero           {0, 0}
#define VersionData_init_zero                    {0, {{NULL}, NULL}}
#define VersionData_VersionStruct_init_zero      {0, 0, 0, 0, 0}
#define HealthData_init_zero                     {0, {{NULL}, NULL}}
#define DeviceData_init_zero                     {0, {{NULL}, NULL}}
#define OdometryData_init_zero                   {0, 0, 0, 0, 0}
#define IMUData_init_zero                        {0, 0, 0, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define SaberIMUData_init_zero                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define UltrasonicData_init_zero                 {0, {{NULL}, NULL}}
#define ProtectorData_init_zero                  {0, {{NULL}, NULL}}
#define AnalogAntiCollisionData_init_zero        {0, {{NULL}, NULL}, {{NULL}, NULL}}
#define AntiDropData_init_zero                   {0, {{NULL}, NULL}, {{NULL}, NULL}}
#define UWBData_init_zero                        {0, 0, 0, 0}
#define UWBDistanceData_init_zero                {0, 0, {{NULL}, NULL}}
#define UWBDistanceData_UWBDistanceStruct_init_zero {0, 0}
#define RFIDData_init_zero                       {0, {{NULL}, NULL}}
#define RFIDData_RFIDStruct_init_zero            {0, 0, 0}
#define RemoteData433_init_zero                  {0, {{NULL}, NULL}}
#define MotorDriverConfigData_init_zero          {0, {{NULL}, NULL}}
#define HardwareDriverData_init_zero             {0, {{NULL}, NULL}}
#define OdomRecorderData_init_zero               {0, 0, 0, 0, 0, 0, 0, 0}
#define MCUDebugData_init_zero                   {0, {{NULL}, NULL}}
#define BatteryData_init_zero                    {0, {{NULL}, NULL}}
#define ShaoTestData_init_zero                   {0, 0, 0}
#define MotionPositionCammand_init_zero          {0, 0, 0, 0}
#define OtherDeviceData_init_zero                {0, {{NULL}, NULL}}
#define HaideDeviceCommand_init_zero             {0, {{NULL}, NULL}}
#define HaideMotionData_init_zero                {0, 0, 0}
#define HaideDeviceData_init_zero                {0, {{NULL}, NULL}}

#define CANBusCommand_init_zero                  {0, 0, 0, 0, {{NULL}, NULL}}
#define CANBusStatusData_init_zero               {0, 0, 0, 0, {{NULL}, NULL}}
#define PhantasMcuUpgradeCommand_init_zero       {0, 0, 0, 0, 0}
#define PhantasMcuUpgradePackage_init_zero       {0, 0, 0, 0, {{NULL}, NULL}}
#define PhantasMcuUpgradeCommandStatusData_init_zero {0, 0, 0, 0}
#define PhantasMcuUpgradePackageStatusData_init_zero {0, 0, 0, 0}
#define PhantasMcuTypeData_init_zero             {0, {{NULL}, NULL}}

/* Field tags (for use in manual encoding/decoding) */
#define AnalogAntiCollisionData_id_tag           1
#define AnalogAntiCollisionData_data_tag         2
#define AnalogAntiCollisionData_trigger_data_tag 3
#define AntiDropData_id_tag                      1
#define AntiDropData_data_tag                    2
#define AntiDropData_trigger_data_tag            3
#define BatteryData_id_tag                       1

#define CANBusCommand_id_tag                     1
#define CANBusCommand_rtr_tag                    2
#define CANBusCommand_ide_tag                    3
#define CANBusCommand_can_id_tag                 4
#define CANBusCommand_data_tag                   5
#define CANBusStatusData_id_tag                  1
#define CANBusStatusData_rtr_tag                 2
#define CANBusStatusData_ide_tag                 3
#define CANBusStatusData_can_id_tag              4
#define CANBusStatusData_data_tag                5

#define BatteryData_data_tag                     2
#define CameraPTZCommand_id_tag                  1
#define CameraPTZCommand_command_id_tag          2
#define CameraPTZCommand_data_1_tag              3
#define CameraPTZCommand_data_2_tag              4
#define ClearErrorCommand_id_tag                 1
#define ClearErrorCommand_bits_tag               2
#define ConfigCommand_id_tag                     1
#define ConfigCommand_config_tag                 2
#define DeviceCommand_id_tag                     1
#define DeviceCommand_command_tag                2
#define DeviceData_id_tag                        1
#define DeviceData_data_tag                      2
#define DriverConfigCommand_id_tag               1
#define DriverConfigCommand_config_tag           2
#define HaideDeviceCommand_id_tag                1
#define HaideDeviceCommand_command_tag           2
#define HaideDeviceData_id_tag                   1
#define HaideDeviceData_data_tag                 2
#define HaideMotionData_id_tag                   1
#define HaideMotionData_speed_tag                2
#define HaideMotionData_angle_tag                3
#define HardwareDriverData_id_tag                1
#define HardwareDriverData_data_tag              2
#define HeaderMessage_id_tag                     1
#define HealthData_id_tag                        1
#define HealthData_data_tag                      2
#define IMUData_id_tag                           1
#define IMUData_roll_tag                         2
#define IMUData_pitch_tag                        3
#define IMUData_yaw_tag                          4
#define IMUData_angular_velocity_x_tag           5
#define IMUData_angular_velocity_y_tag           6
#define IMUData_angular_velocity_z_tag           7
#define IMUData_linear_acceleration_x_tag        8
#define IMUData_linear_acceleration_y_tag        9
#define IMUData_linear_acceleration_z_tag        10
#define IMUData_magnetic_field_x_tag             11
#define IMUData_magnetic_field_y_tag             12
#define IMUData_magnetic_field_z_tag             13
#define MCUDebugData_id_tag                      1
#define MCUDebugData_data_tag                    2
#define MCUUpgradeCommand_id_tag                 1
#define MCUUpgradeCommand_time_tag               2
#define MCUUpgradeCommand_num_tag                3
#define MCUUpgradeCommand_check_code_tag         4
#define MCUUpgradeCommand_command_tag            5
#define MCUUpgradeStatusData_id_tag              1
#define MCUUpgradeStatusData_status_tag          2
#define ManualModeDeviceCommand_id_tag           1
#define ManualModeDeviceCommand_command_tag      2
#define MotionControlMotorCommand_id_tag         1
#define MotionControlMotorCommand_command_tag    2
#define MotionControlMotorCommand_MotorCommandPair_id_tag 1
#define MotionControlMotorCommand_MotorCommandPair_command_tag 2
#define MotionPositionCammand_id_tag             1
#define MotionPositionCammand_left_position_tag  2
#define MotionPositionCammand_right_position_tag 3
#define MotionPositionCammand_brake_cmd_tag      4
#define MotorDriverConfigCommand_id_tag          1
#define MotorDriverConfigCommand_config_tag      2
#define MotorDriverConfigData_id_tag             1
#define MotorDriverConfigData_config_tag         2
#define OdomRecorderData_id_tag                  1
#define OdomRecorderData_odom_x_tag              2
#define OdomRecorderData_odom_y_tag              3
#define OdomRecorderData_odom_w_tag              4
#define OdomRecorderData_dx_tag                  5
#define OdomRecorderData_x_tag                   6
#define OdomRecorderData_y_tag                   7
#define OdomRecorderData_w_tag                   8
#define OdometryData_id_tag                      1
#define OdometryData_left_count_tag              2
#define OdometryData_right_count_tag             3
#define OdometryData_left_delta_count_tag        4
#define OdometryData_right_delta_count_tag       5
#define OtherDeviceData_id_tag                   1
#define OtherDeviceData_data_tag                 2
#define ProtectorData_id_tag                     1
#define ProtectorData_data_tag                   2
#define RFIDData_id_tag                          1
#define RFIDData_data_tag                        2
#define RFIDData_RFIDStruct_id_tag               1
#define RFIDData_RFIDStruct_data_tag             2
#define RFIDData_RFIDStruct_strength_tag         3
#define RemoteCommand433_id_tag                  1
#define RemoteCommand433_command_tag             2
#define RemoteData433_id_tag                     1
#define RemoteData433_data_tag                   2
#define SaberIMUData_id_tag                      1
#define SaberIMUData_acc_x_tag                   2
#define SaberIMUData_acc_y_tag                   3
#define SaberIMUData_acc_z_tag                   4
#define SaberIMUData_gyro_x_tag                  5
#define SaberIMUData_gyro_y_tag                  6
#define SaberIMUData_gyro_z_tag                  7
#define SaberIMUData_roll_tag                    8
#define SaberIMUData_pitch_tag                   9
#define SaberIMUData_yaw_tag                     10
#define SaveMotorDriverConfigCommand_id_tag      1
#define ShaoTestData_id_tag                      1
#define ShaoTestData_angular_velocity_tag        2
#define ShaoTestData_linear_velocity_tag         3
#define ShutdownCommand_id_tag                   1
#define ShutdownCommand_type_tag                 2
#define UInt32Pair_key_tag                       1
#define UInt32Pair_value_tag                     2
#define UWBData_id_tag                           1
#define UWBData_x_tag                            2
#define UWBData_y_tag                            3
#define UWBData_z_tag                            4
#define UWBDistanceData_id_tag                   1
#define UWBDistanceData_tag_num_tag              2
#define UWBDistanceData_data_tag                 3
#define UWBDistanceData_UWBDistanceStruct_id_tag 1
#define UWBDistanceData_UWBDistanceStruct_data_tag 2
#define UltrasonicData_id_tag                    1
#define UltrasonicData_data_tag                  2
#define VerifyMessage_id_tag                     1
#define VerifyMessage_data_tag                   2
#define VersionData_id_tag                       1
#define VersionData_version_tag                  2
#define VersionData_VersionStruct_board_tag      1
#define VersionData_VersionStruct_type_tag       2
#define VersionData_VersionStruct_major_tag      3
#define VersionData_VersionStruct_minor_tag      4
#define VersionData_VersionStruct_build_tag      5

/* Struct field encoding specification for nanopb */
extern const pb_field_t HeaderMessage_fields[2];
extern const pb_field_t UInt32Pair_fields[3];
extern const pb_field_t VerifyMessage_fields[3];
extern const pb_field_t ShutdownCommand_fields[3];
extern const pb_field_t ConfigCommand_fields[3];
extern const pb_field_t DriverConfigCommand_fields[3];
extern const pb_field_t MotionControlMotorCommand_fields[3];
extern const pb_field_t MotionControlMotorCommand_MotorCommandPair_fields[3];
extern const pb_field_t DeviceCommand_fields[3];
extern const pb_field_t ManualModeDeviceCommand_fields[3];
extern const pb_field_t RemoteCommand433_fields[3];
extern const pb_field_t ClearErrorCommand_fields[3];
extern const pb_field_t SaveMotorDriverConfigCommand_fields[2];
extern const pb_field_t MotorDriverConfigCommand_fields[3];
extern const pb_field_t MCUUpgradeCommand_fields[6];
extern const pb_field_t CameraPTZCommand_fields[5];
extern const pb_field_t MCUUpgradeStatusData_fields[3];
extern const pb_field_t VersionData_fields[3];
extern const pb_field_t VersionData_VersionStruct_fields[6];
extern const pb_field_t HealthData_fields[3];
extern const pb_field_t DeviceData_fields[3];
extern const pb_field_t OdometryData_fields[6];
extern const pb_field_t IMUData_fields[14];
extern const pb_field_t SaberIMUData_fields[11];
extern const pb_field_t UltrasonicData_fields[3];
extern const pb_field_t ProtectorData_fields[3];
extern const pb_field_t AnalogAntiCollisionData_fields[4];
extern const pb_field_t AntiDropData_fields[4];
extern const pb_field_t UWBData_fields[5];
extern const pb_field_t UWBDistanceData_fields[4];
extern const pb_field_t UWBDistanceData_UWBDistanceStruct_fields[3];
extern const pb_field_t RFIDData_fields[3];
extern const pb_field_t RFIDData_RFIDStruct_fields[4];
extern const pb_field_t RemoteData433_fields[3];
extern const pb_field_t MotorDriverConfigData_fields[3];
extern const pb_field_t HardwareDriverData_fields[3];
extern const pb_field_t OdomRecorderData_fields[9];
extern const pb_field_t MCUDebugData_fields[3];
extern const pb_field_t BatteryData_fields[3];
extern const pb_field_t ShaoTestData_fields[4];
extern const pb_field_t MotionPositionCammand_fields[5];
extern const pb_field_t OtherDeviceData_fields[3];
extern const pb_field_t DriverData_fields[3];
extern const pb_field_t HaideDeviceCommand_fields[3];
extern const pb_field_t HaideMotionData_fields[4];
extern const pb_field_t HaideDeviceData_fields[3];

extern const pb_field_t CANBusCommand_fields[6];
extern const pb_field_t CANBusStatusData_fields[6];
extern const pb_field_t PhantasMcuUpgradeCommand_fields[6];
extern const pb_field_t PhantasMcuUpgradePackage_fields[6];
extern const pb_field_t PhantasMcuUpgradeCommandStatusData_fields[5];
extern const pb_field_t PhantasMcuUpgradePackageStatusData_fields[5];
extern const pb_field_t PhantasMcuTypeData_fields[3];

/* Maximum encoded size of messages (where known) */
#define HeaderMessage_size                       6
#define UInt32Pair_size                          12
#define VerifyMessage_size                       12
#define ShutdownCommand_size                     12
/* ConfigCommand_size depends on runtime parameters */
/* DriverConfigCommand_size depends on runtime parameters */
/* MotionControlMotorCommand_size depends on runtime parameters */
#define MotionControlMotorCommand_MotorCommandPair_size 17
/* DeviceCommand_size depends on runtime parameters */
/* ManualModeDeviceCommand_size depends on runtime parameters */
/* RemoteCommand433_size depends on runtime parameters */
/* ClearErrorCommand_size depends on runtime parameters */
#define SaveMotorDriverConfigCommand_size        6
/* MotorDriverConfigCommand_size depends on runtime parameters */
/* MCUUpgradeCommand_size depends on runtime parameters */
#define CameraPTZCommand_size                    24
#define MCUUpgradeStatusData_size                12
/* VersionData_size depends on runtime parameters */
#define VersionData_VersionStruct_size           30
/* HealthData_size depends on runtime parameters */
/* DeviceData_size depends on runtime parameters */
#define OdometryData_size                        50
#define IMUData_size                             138
#define SaberIMUData_size                        60
/* UltrasonicData_size depends on runtime parameters */
/* ProtectorData_size depends on runtime parameters */
/* AnalogAntiCollisionData_size depends on runtime parameters */
/* AntiDropData_size depends on runtime parameters */
#define UWBData_size                             24
/* UWBDistanceData_size depends on runtime parameters */
#define UWBDistanceData_UWBDistanceStruct_size   12
/* RFIDData_size depends on runtime parameters */
#define RFIDData_RFIDStruct_size                 18
/* RemoteData433_size depends on runtime parameters */
/* MotorDriverConfigData_size depends on runtime parameters */
/* HardwareDriverData_size depends on runtime parameters */
#define OdomRecorderData_size                    48
/* MCUDebugData_size depends on runtime parameters */
/* BatteryData_size depends on runtime parameters */
#define ShaoTestData_size                        28
#define MotionPositionCammand_size               39
/* OtherDeviceData_size depends on runtime parameters */
/* HaideDeviceCommand_size depends on runtime parameters */
#define HaideMotionData_size                     28
/* HaideDeviceData_size depends on runtime parameters */
/* OtherSensorData_size depends on runtime parameters */
/* CANBusCommand_size depends on runtime parameters */
/* CANBusStatusData_size depends on runtime parameters */
#define PhantasMcuUpgradeCommand_size            30
/* PhantasMcuUpgradePackage_size depends on runtime parameters */
#define PhantasMcuUpgradeCommandStatusData_size  24
#define PhantasMcuUpgradePackageStatusData_size  24
/* PhantasMcuTypeData_size depends on runtime parameters */
/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define SERVICE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif