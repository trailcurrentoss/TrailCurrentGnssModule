#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "globals.h"
#include "driver/twai.h"

#define CAN_RX 13
#define CAN_TX 15
#define POLLING_RATE_MS 100
static bool driver_installed = false;
#define CAN_SEND_MESSAGE_LATLON_IDENTIFIER 0x009;
#define CAN_SEND_MESSAGE_DATETIME_IDENTIFIER 0x006;
#define CAN_SEND_MESSAGE_SATNUM_SPEED_COURSE_GNSSMODE_IDENTIFIER 0x007;
#define CAN_SEND_MESSAGE_ALTITUDE_IDENTIFIER 0x008;

namespace canHelper
{
    void setup()
    {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NO_ACK);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Look in the api-reference for other speed sets.
        // THIS IS A FILTER THAT ENSURES TRANSMIT ONLY
        twai_filter_config_t f_config = {
            .acceptance_code = 0xFFFFFFFF, // Base ID = 0
            .acceptance_mask = 0x00000000, // Match exactly against the mask
            .single_filter = true          // Single filter mode
        };
        // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        //  Install TWAI driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
        {
            debugln("Driver installed");
        }
        else
        {
            debugln("Failed to install driver");
            return;
        }

        // Start TWAI driver
        if (twai_start() == ESP_OK)
        {
            debugln("Driver started");
        }
        else
        {
            debugln("Failed to start driver");
            return;
        }

        // Note: Receive queue clearing not implemented as this is transmit-only device
        // The CAN filter is configured to reject all incoming messages to prevent queue buildup

        // TWAI driver is now successfully installed and started
        driver_installed = true;
    }

    uint8_t latBytes[4];
    uint8_t lonBytes[4];
    uint8_t latLonByteAry[8];
    uint8_t dateTimeByteAry[8];
    uint8_t courseOverGroundByteAry[2];
    uint8_t speedOverGroundByteAry[2];
    uint8_t altitudeByteAry[4];
    // Endcode a latitude or longitude float value into a 4-byte array with the first byte as sign
    void encodeLatOrLonValue(float f, uint8_t out[4])
    {
        // Sign byte
        out[0] = (f < 0) ? 1 : 0;
        if (f < 0)
            f = -f;

        // Scale and round
        uint32_t scaled = (uint32_t)(f * 10000.0f + 0.5f);

        // Store 24-bit integer
        out[1] = (scaled >> 16) & 0xFF;
        out[2] = (scaled >> 8) & 0xFF;
        out[3] = scaled & 0xFF;
    }
    // Decode a latitude or longitude float value from a 4-byte array with the first byte as sign
    float decodeLatOrLonValue(const uint8_t in[4])
    {
        uint32_t scaled =
            ((uint32_t)in[1] << 16) |
            ((uint32_t)in[2] << 8) |
            (uint32_t)in[3];

        float f = scaled / 10000.0f;
        if (in[0] == 1)
            f = -f;
        debuglnf(f, 5);
        return f;
    }
    // Method that can be used to decode date and time data from a CAN message
    void decodeDateTimeData(const uint8_t in[8], uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &hour, uint8_t &minute, uint8_t &second)
    {
        year = ((uint16_t)in[0] << 8) | (uint16_t)in[1];
        month = in[2];
        day = in[3];
        hour = in[4];
        minute = in[5];
        second = in[6];
    }
    // Method that can be used to decode altitude data from a CAN message
    void decodeAltitudeData(const uint8_t in[4])
    {
        double altitude = 0.0;
        uint32_t scaled =
            ((uint32_t)in[0] << 24) |
            ((uint32_t)in[1] << 16) |
            ((uint32_t)in[2] << 8) |
            (uint32_t)in[3];
        altitude = scaled / 100.0;
    }

    void formatDateTimeData(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
    {
        // Implementation for formatting date and time data into CAN message
        dateTimeByteAry[0] = (year >> 8) & 0xFF;
        dateTimeByteAry[1] = year & 0xFF;
        dateTimeByteAry[2] = month;
        dateTimeByteAry[3] = day;
        dateTimeByteAry[4] = hour;
        dateTimeByteAry[5] = minute;
        dateTimeByteAry[6] = second;
        dateTimeByteAry[7] = 0; // Reserved for future use or padding
    }

    void formatLatLongData(float latitude, float longitude)
    {
        encodeLatOrLonValue(latitude, canHelper::latBytes);
        encodeLatOrLonValue(longitude, canHelper::lonBytes);
        latLonByteAry[0] = latBytes[0];
        latLonByteAry[1] = latBytes[1];
        latLonByteAry[2] = latBytes[2];
        latLonByteAry[3] = latBytes[3];
        latLonByteAry[4] = lonBytes[0];
        latLonByteAry[5] = lonBytes[1];
        latLonByteAry[6] = lonBytes[2];
        latLonByteAry[7] = lonBytes[3];
    }

    void formatCourseOverGroundData(double courseOverGround)
    {
        uint16_t scaled = (uint16_t)(courseOverGround * 10.0 + 0.5);
        courseOverGroundByteAry[0] = (scaled >> 8) & 0xFF;
        courseOverGroundByteAry[1] = scaled & 0xFF;
    }

    void formatSpeedOverGroundData(double speedOverGround)
    {
        uint16_t scaled = (uint16_t)(speedOverGround * 100.0);
        speedOverGroundByteAry[0] = (scaled >> 8) & 0xFF;
        speedOverGroundByteAry[1] = scaled & 0xFF;
    }

    void formatAltitudeData(double altitude)
    {
        uint32_t scaled = (uint32_t)(altitude * 100.0);
        altitudeByteAry[0] = (scaled >> 24) & 0xFF;
        altitudeByteAry[1] = (scaled >> 16) & 0xFF;
        altitudeByteAry[2] = (scaled >> 8) & 0xFF;
        altitudeByteAry[3] = scaled & 0xFF;
        decodeAltitudeData(altitudeByteAry);
    }

    void sendDateTimeCanMessage(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
    {
        if (!driver_installed)
        {
            debugln("TWAI driver not installed, cannot send message");
            return;
        }

        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_DATETIME_IDENTIFIER;
        message.data_length_code = 7;
        message.data[0] = (year >> 8) & 0xFF;
        message.data[1] = year & 0xFF;
        message.data[2] = month;
        message.data[3] = day;
        message.data[4] = hour;
        message.data[5] = minute;
        message.data[6] = second;
        message.flags = TWAI_MSG_FLAG_NONE;

        // Transmit message
        if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            // debugln("Date/Time CAN message sent");
        }
        else
        {
            debugln("Failed to send Date/Time CAN message");
        }
    }

    void sendLatLonCanMessage(float latitude, float longitude)
    {
        if (!driver_installed)
        {
            debugln("TWAI driver not installed, cannot send message");
            return;
        }

        formatLatLongData(latitude, longitude);

        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_LATLON_IDENTIFIER;
        message.data_length_code = 8;
        memcpy(message.data, latLonByteAry, 8);
        message.flags = TWAI_MSG_FLAG_NONE;

        // Transmit message
        if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            debugln("Lat/Lon CAN message sent");
        }
        else
        {
            debugln("Failed to send Lat/Lon CAN message");
        }
    }

    void sendSatSpeedCourseAndModeMessage(uint8_t numSatUsed, double speedOverGround, double courseOverGround, uint8_t gnssMode)
    {
        if (!driver_installed)
        {
            debugln("TWAI driver not installed, cannot send message");
            return;
        }

        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_SATNUM_SPEED_COURSE_GNSSMODE_IDENTIFIER;
        message.data_length_code = 6;
        message.data[0] = numSatUsed;
        
        // Format and send speed over ground (scaled by 100)
        uint16_t speedScaled = (uint16_t)(speedOverGround * 100.0);
        message.data[1] = (speedScaled >> 8) & 0xFF;
        message.data[2] = speedScaled & 0xFF;
        
        // Format and send course over ground (scaled by 10)
        uint16_t courseScaled = (uint16_t)(courseOverGround * 10.0 + 0.5);
        message.data[3] = (courseScaled >> 8) & 0xFF;
        message.data[4] = courseScaled & 0xFF;
        // Debug output to verify course over ground values (minimal)
        debuglnf(courseOverGround, 1);
        
        message.data[5] = gnssMode;
        message.flags = TWAI_MSG_FLAG_NONE;

        // Transmit message
        if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            debugln("Sat/Speed/Course/GNSSMode CAN message sent");
        }
        else
        {
            debugln("Failed to send Sat/Speed/Course/GNSSMode CAN message");
        }
    }

    void sendAltitudeDate(double altitude)
    {
        if (!driver_installed)
        {
            debugln("TWAI driver not installed, cannot send message");
            return;
        }

        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_ALTITUDE_IDENTIFIER;
        message.data_length_code = 4;
        message.data[0] = altitudeByteAry[0];
        message.data[1] = altitudeByteAry[1];
        message.data[2] = altitudeByteAry[2];
        message.data[3] = altitudeByteAry[3];
        message.flags = TWAI_MSG_FLAG_NONE;

        // Transmit message
        if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            // debugln("Course Over Ground CAN message sent");
        }
        else
        {
            // debugln("Failed to send Course Over Ground CAN message");
        }
    }

    void sendGnssData(
        uint16_t &currentYear,
        uint8_t &currentMonth,
        uint8_t &currentDay,
        uint8_t &currentHour,
        uint8_t &currentMinute,
        uint8_t &currentSecond,
        float &currentLatitude,
        float &currentLongitude,
        double &currentAltitude,
        uint8_t &currentNumSatUsed,
        double &currentSpeedOverGround,
        double &currentCourseOverGround,
        uint8_t &currentGnssMode)
    {

        formatLatLongData(currentLatitude, currentLongitude);
        formatDateTimeData(currentYear, currentMonth, currentDay, currentHour, currentMinute, currentSecond);
        formatCourseOverGroundData(currentCourseOverGround);
        formatSpeedOverGroundData(currentSpeedOverGround);
        formatAltitudeData(currentAltitude);

        sendSatSpeedCourseAndModeMessage(currentNumSatUsed, currentSpeedOverGround, currentCourseOverGround, currentGnssMode);
        sendLatLonCanMessage(currentLatitude, currentLongitude);
        sendDateTimeCanMessage(currentYear, currentMonth, currentDay, currentHour, currentMinute, currentSecond);
        sendSatSpeedCourseAndModeMessage(currentNumSatUsed, currentSpeedOverGround, currentCourseOverGround, currentGnssMode);
        sendAltitudeDate(currentAltitude);
        return;
    }
}