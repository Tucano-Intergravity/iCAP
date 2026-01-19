/*
 * imu.c
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#include "imu.h"
#include <stddef.h>

/* Calculate i400 packet checksum (16-bit Unsigned Summation)
 * Honeywell HGuide i400 IMU HGNSI Protocol
 * 
 * Calculation range: bytes 0 to length-3 (checksum field excluded)
 * This sums from 0x0E (first byte) to the byte before checksum field
 * Same logic as Verify_i400_Checksum but returns the calculated value
 * 
 * @param data: Pointer to packet data
 * @param length: Total packet length (including checksum, typically 44 bytes)
 * @return Calculated checksum (uint16_t)
 */
uint16_t IMU_CalculateChecksum(uint8_t *data, uint16_t length)
{
    if (data == NULL || length < IMU_PACKET_SIZE) {
        return 0;
    }
    
    /* Initialize 16-bit sum to 0 */
    uint16_t sum = 0;
    
    /* Sum 21 words (42 bytes) using Little Endian word formation (Method 3) */
    for (uint16_t i = 0; i < (IMU_PACKET_SIZE - 2); i += 2) {
        uint16_t word = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
        sum += word;
    }
    
    return sum;
}

/* Parse i400 Navigation packet (0xA2) and decode data */
bool IMU_ParsePacket(uint8_t *packet_data, IMU_Data_t *imu_data)
{
    if (packet_data == NULL || imu_data == NULL) {
        return false;
    }
    
    imu_raw_packet_t *packet = (imu_raw_packet_t *)packet_data;
    
    /* Verify sync byte and message ID */
    if (packet->sync != IMU_SYNC_BYTE || packet->msg_id != IMU_MSG_ID_NAV) {
        return false;
    }
    
    /* Verify checksum (16-bit Unsigned Summation from 0x0E to byte before checksum) */
    /* Note: This is kept for module integrity, but UARTComm.c already verifies before calling */
    uint16_t calculated_checksum = IMU_CalculateChecksum(packet_data, IMU_PACKET_SIZE);
    uint16_t received_checksum = packet_data[IMU_PACKET_SIZE - 2] | (packet_data[IMU_PACKET_SIZE - 1] << 8);  /* Little endian */
    
    if (calculated_checksum != received_checksum) {
        return false;
    }
    
    /* Packet is valid - decode data */
    
    /* Decode Angular Rate (rad/s) - LSB: 2^-11 rad/s */
    const float ANGULAR_RATE_LSB = 1.0f / (1 << 11);  /* 2^-11 */
    imu_data->angular_rate_x = (float)packet->angular_rate_x * ANGULAR_RATE_LSB;
    imu_data->angular_rate_y = (float)packet->angular_rate_y * ANGULAR_RATE_LSB;
    imu_data->angular_rate_z = (float)packet->angular_rate_z * ANGULAR_RATE_LSB;
    
    /* Decode Linear Acceleration (m/s²) - LSB: (2^-5) * 0.3048 m/s² */
    const float LINEAR_ACCEL_LSB = (1.0f / (1 << 5)) * 0.3048f;  /* (2^-5) * 0.3048 */
    imu_data->linear_accel_x = (float)packet->linear_accel_x * LINEAR_ACCEL_LSB;
    imu_data->linear_accel_y = (float)packet->linear_accel_y * LINEAR_ACCEL_LSB;
    imu_data->linear_accel_z = (float)packet->linear_accel_z * LINEAR_ACCEL_LSB;
    
    /* Status Word */
    imu_data->status_word = packet->status_word;
    
    /* Decode Delta Angle (rad) - LSB: 2^-33 rad */
    const float DELTA_ANGLE_LSB = 1.0f / (1ULL << 33);  /* 2^-33 */
    imu_data->delta_angle_x = (float)packet->delta_angle_x * DELTA_ANGLE_LSB;
    imu_data->delta_angle_y = (float)packet->delta_angle_y * DELTA_ANGLE_LSB;
    imu_data->delta_angle_z = (float)packet->delta_angle_z * DELTA_ANGLE_LSB;
    
    /* Decode Delta Velocity (m/s) - LSB: (2^-27) * 0.3048 m/s */
    const float DELTA_VELOCITY_LSB = (1.0f / (1ULL << 27)) * 0.3048f;  /* (2^-27) * 0.3048 */
    imu_data->delta_velocity_x = (float)packet->delta_velocity_x * DELTA_VELOCITY_LSB;
    imu_data->delta_velocity_y = (float)packet->delta_velocity_y * DELTA_VELOCITY_LSB;
    imu_data->delta_velocity_z = (float)packet->delta_velocity_z * DELTA_VELOCITY_LSB;
    
    /* Timestamp is not provided by packet, leave it to caller to fill if needed */
    imu_data->timestamp = 0;
    
    return true;
}

/* Check if a message ID is a valid IMU message type */
bool IMU_IsValidMessageID(uint8_t msg_id)
{
    return (msg_id == IMU_MSG_ID_CTRL || msg_id == IMU_MSG_ID_NAV);
}

/* Check if a message ID is a Control packet (should be ignored) */
bool IMU_IsControlPacket(uint8_t msg_id)
{
    return (msg_id == IMU_MSG_ID_CTRL);
}

/* Check if a message ID is a Navigation packet (should be processed) */
bool IMU_IsNavigationPacket(uint8_t msg_id)
{
    return (msg_id == IMU_MSG_ID_NAV);
}
