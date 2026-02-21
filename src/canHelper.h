#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "globals.h"
#include <TwaiTaskBased.h>
#include <OtaUpdate.h>
#include <Preferences.h>

#define CAN_RX 13
#define CAN_TX 15

// Forward declaration for OTA handler
extern OtaUpdate otaUpdate;

// WiFi credential reception state (CAN ID 0x01 protocol)
static bool wifiConfigInProgress = false;
static uint8_t wifiSsidBuffer[33];       // Max 32 chars + null
static uint8_t wifiPasswordBuffer[64];   // Max 63 chars + null
static uint8_t wifiSsidLen = 0;
static uint8_t wifiPasswordLen = 0;
static uint8_t wifiSsidReceived = 0;
static uint8_t wifiPasswordReceived = 0;

#define CAN_SEND_MESSAGE_LATLON_IDENTIFIER 0x009
#define CAN_SEND_MESSAGE_DATETIME_IDENTIFIER 0x006
#define CAN_SEND_MESSAGE_SATNUM_SPEED_COURSE_GNSSMODE_IDENTIFIER 0x007
#define CAN_SEND_MESSAGE_ALTITUDE_IDENTIFIER 0x008

namespace canHelper
{
    void saveWifiCredentials(const char* ssid, const char* password) {
        Preferences prefs;
        prefs.begin("wifi", false);  // read-write
        prefs.putString("ssid", ssid);
        prefs.putString("password", password);
        prefs.end();
        debugf("[WiFi] Credentials saved to NVS (SSID: %s)\n", ssid);
    }

    void handleWifiConfigMessage(const twai_message_t &message) {
        uint8_t msgType = message.data[0];

        switch (msgType) {
            case 0x01: {  // Start message
                wifiSsidLen = message.data[1];
                wifiPasswordLen = message.data[2];
                wifiSsidReceived = 0;
                wifiPasswordReceived = 0;
                memset(wifiSsidBuffer, 0, sizeof(wifiSsidBuffer));
                memset(wifiPasswordBuffer, 0, sizeof(wifiPasswordBuffer));
                wifiConfigInProgress = true;
                debugg("[WiFi] Config start: SSID len=%d, Password len=%d\n", wifiSsidLen, wifiPasswordLen);
                break;
            }

            case 0x02: {  // SSID chunk
                if (!wifiConfigInProgress) break;
                uint8_t dataBytes = message.data_length_code - 2;
                uint8_t remaining = wifiSsidLen - wifiSsidReceived;
                if (dataBytes > remaining) dataBytes = remaining;
                if (wifiSsidReceived + dataBytes <= 32) {
                    memcpy(wifiSsidBuffer + wifiSsidReceived, &message.data[2], dataBytes);
                    wifiSsidReceived += dataBytes;
                }
                break;
            }

            case 0x03: {  // Password chunk
                if (!wifiConfigInProgress) break;
                uint8_t dataBytes = message.data_length_code - 2;
                uint8_t remaining = wifiPasswordLen - wifiPasswordReceived;
                if (dataBytes > remaining) dataBytes = remaining;
                if (wifiPasswordReceived + dataBytes <= 63) {
                    memcpy(wifiPasswordBuffer + wifiPasswordReceived, &message.data[2], dataBytes);
                    wifiPasswordReceived += dataBytes;
                }
                break;
            }

            case 0x04: {  // End message with checksum
                if (!wifiConfigInProgress) break;
                wifiConfigInProgress = false;

                uint8_t checksum = 0;
                for (uint8_t i = 0; i < wifiSsidReceived; i++) checksum ^= wifiSsidBuffer[i];
                for (uint8_t i = 0; i < wifiPasswordReceived; i++) checksum ^= wifiPasswordBuffer[i];

                if (checksum == message.data[1] && wifiSsidReceived == wifiSsidLen && wifiPasswordReceived == wifiPasswordLen) {
                    wifiSsidBuffer[wifiSsidReceived] = '\0';
                    wifiPasswordBuffer[wifiPasswordReceived] = '\0';
                    saveWifiCredentials((const char*)wifiSsidBuffer, (const char*)wifiPasswordBuffer);
                } else {
                    debugf("[WiFi] Config failed: checksum %s\n",
                           (checksum == message.data[1]) ? "OK" : "MISMATCH");
                    debugg("[WiFi]   SSID %d/%d bytes\n", wifiSsidReceived, wifiSsidLen);
                    debugg("[WiFi]   Password %d/%d bytes\n", wifiPasswordReceived, wifiPasswordLen);
                }
                break;
            }
        }
    }

    static void handle_rx_message(const twai_message_t &message) {
        // OTA trigger message (ID 0x0)
        if (message.identifier == 0x0) {
            debugln("[OTA] CAN trigger received");

            String currentHostName = otaUpdate.getHostName();

            char targetHostName[14];
            sprintf(targetHostName, "esp32-%02X%02X%02X",
                    message.data[0], message.data[1], message.data[2]);

            debugf("[OTA] Target hostname:  '%s'\n", targetHostName);
            debugf("[OTA] Current hostname: '%s'\n", currentHostName.c_str());

            if (currentHostName.equals(targetHostName)) {
                debugln("[OTA] Hostname matched - reading WiFi credentials from NVS");
                Preferences prefs;
                prefs.begin("wifi", true);  // read-only
                String ssid = prefs.getString("ssid", "");
                String password = prefs.getString("password", "");
                prefs.end();

                if (ssid.length() > 0 && password.length() > 0) {
                    debugf("[OTA] Using stored WiFi credentials (SSID: %s)\n", ssid.c_str());
                    OtaUpdate ota(180000, ssid.c_str(), password.c_str());
                    ota.waitForOta();
                    debugln("[OTA] OTA mode exited - resuming normal operation");
                } else {
                    debugln("[OTA] ERROR: No WiFi credentials in NVS - cannot start OTA");
                }
            } else {
                debugln("[OTA] Hostname mismatch - ignoring OTA trigger");
            }
            return;
        }

        // WiFi credential configuration message (ID 0x01)
        if (message.identifier == 0x01) {
            handleWifiConfigMessage(message);
            return;
        }
    }

    static void handle_tx_result(bool success) {
        debugf("[CAN] TX %s\n", success ? "OK" : "FAILED");
    }

    void setupCan()
    {
        if (TwaiTaskBased::begin((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, 500000, TWAI_MODE_NO_ACK)) {
            debugln("[CAN] Driver initialized");
        } else {
            debugln("[CAN] Failed to initialize driver");
            return;
        }

        TwaiTaskBased::onReceive(handle_rx_message);
        TwaiTaskBased::onTransmit(handle_tx_result);
        debugln("[CAN] RX/TX callbacks registered");
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
        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_DATETIME_IDENTIFIER;
        message.extd = false;
        message.rtr = false;
        message.data_length_code = 7;
        message.data[0] = (year >> 8) & 0xFF;
        message.data[1] = year & 0xFF;
        message.data[2] = month;
        message.data[3] = day;
        message.data[4] = hour;
        message.data[5] = minute;
        message.data[6] = second;

        TwaiTaskBased::send(message, pdMS_TO_TICKS(10));
    }

    void sendLatLonCanMessage(float latitude, float longitude)
    {
        formatLatLongData(latitude, longitude);

        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_LATLON_IDENTIFIER;
        message.extd = false;
        message.rtr = false;
        message.data_length_code = 8;
        memcpy(message.data, latLonByteAry, 8);

        TwaiTaskBased::send(message, pdMS_TO_TICKS(10));
    }

    void sendSatSpeedCourseAndModeMessage(uint8_t numSatUsed, double speedOverGround, double courseOverGround, uint8_t gnssMode)
    {
        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_SATNUM_SPEED_COURSE_GNSSMODE_IDENTIFIER;
        message.extd = false;
        message.rtr = false;
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

        TwaiTaskBased::send(message, pdMS_TO_TICKS(10));
    }

    void sendAltitudeDate(double altitude)
    {
        twai_message_t message;
        message.identifier = CAN_SEND_MESSAGE_ALTITUDE_IDENTIFIER;
        message.extd = false;
        message.rtr = false;
        message.data_length_code = 4;
        message.data[0] = altitudeByteAry[0];
        message.data[1] = altitudeByteAry[1];
        message.data[2] = altitudeByteAry[2];
        message.data[3] = altitudeByteAry[3];

        TwaiTaskBased::send(message, pdMS_TO_TICKS(10));
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
