/*
 * imu.h
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#ifndef ICAP_INC_IMU_H_
#define ICAP_INC_IMU_H_

#include <stdbool.h>
#include <stdint.h>

/* i400 IMU Packet Definitions */
#define IMU_PACKET_SIZE       44      /* i400 Navigation packet size (0xA2) */
#define IMU_SYNC_BYTE         0x0E    /* i400 packet sync byte */
#define IMU_MSG_ID_NAV        0xA2    /* i400 Navigation message ID */
#define IMU_MSG_ID_CTRL       0xA1    /* i400 Control message ID (ignored) */

/* i400 Raw Navigation Packet Structure (0xA2) */
#pragma pack(push, 1)
typedef struct {
    uint8_t sync;                   /* 0x0E */
    uint8_t msg_id;                 /* 0xA2 */
    int16_t angular_rate_x;         /* LSB: 2^-11 rad/s */
    int16_t angular_rate_y;
    int16_t angular_rate_z;
    int16_t linear_accel_x;         /* LSB: (2^-5) * 0.3048 m/s² */
    int16_t linear_accel_y;
    int16_t linear_accel_z;
    uint16_t status_word;
    int32_t delta_angle_x;          /* LSB: 2^-33 rad */
    int32_t delta_angle_y;
    int32_t delta_angle_z;
    int32_t delta_velocity_x;       /* LSB: (2^-27) * 0.3048 m/s */
    int32_t delta_velocity_y;
    int32_t delta_velocity_z;
    uint16_t checksum;
} imu_raw_packet_t;
#pragma pack(pop)

/* i400 IMU Decoded Data Structure */
typedef struct {
    /* Angular Rate (rad/s) */
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    
    /* Linear Acceleration (m/s²) */
    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;
    
    /* Status Word */
    uint16_t status_word;
    
    /* Delta Angle (rad) */
    float delta_angle_x;
    float delta_angle_y;
    float delta_angle_z;
    
    /* Delta Velocity (m/s) */
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    
    /* Timestamp (optional, can be filled by caller) */
    uint32_t timestamp;
} IMU_Data_t;

/* Function Prototypes */

/**
 * @brief Calculate i400 packet checksum (sum of first 42 bytes)
 * @param data: Pointer to packet data
 * @param length: Packet length (should be IMU_PACKET_SIZE)
 * @return Calculated checksum
 */
uint16_t IMU_CalculateChecksum(uint8_t *data, uint16_t length);

/**
 * @brief Parse i400 Navigation packet (0xA2) and decode data
 * @param packet_data: Pointer to raw packet data (44 bytes)
 * @param imu_data: Pointer to IMU_Data_t structure to fill
 * @return true if packet is valid and parsed successfully, false otherwise
 */
bool IMU_ParsePacket(uint8_t *packet_data, IMU_Data_t *imu_data);

/**
 * @brief Check if a message ID is a valid IMU message type
 * @param msg_id: Message ID byte
 * @return true if valid (0xA1 or 0xA2), false otherwise
 */
bool IMU_IsValidMessageID(uint8_t msg_id);

/**
 * @brief Check if a message ID is a Control packet (should be ignored)
 * @param msg_id: Message ID byte
 * @return true if Control packet (0xA1), false otherwise
 */
bool IMU_IsControlPacket(uint8_t msg_id);

/**
 * @brief Check if a message ID is a Navigation packet (should be processed)
 * @param msg_id: Message ID byte
 * @return true if Navigation packet (0xA2), false otherwise
 */
bool IMU_IsNavigationPacket(uint8_t msg_id);

#endif /* ICAP_INC_IMU_H_ */
