/*
 *@brief: Library for AS608 module using in esp32-idf 
 *@author: minhnt29
 *@Note : Retranslate from Fingerprint.h
 *@license : None
 */

#ifndef _FINGERPRINT_H
#define _FINGERPRINT_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"


/***************************************************************************************************************************************************/
/*                                                         PROJECT DEFINITIONS                                                                     */
/***************************************************************************************************************************************************/
#define AS608_UART_PORT UART_NUM_1
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)


/***************************************************************************************************************************************************/
/*                                                         DEFINE COMMAND AND ENUMS                                                                */
/***************************************************************************************************************************************************/
#define FINGERPRINT_OK                     0x00 //!< Command execution is complete
#define FINGERPRINT_PACKETRECIEVEERR       0x01 //!< Error when receiving data package
#define FINGERPRINT_NOFINGER               0x02 //!< No finger on the sensor
#define FINGERPRINT_IMAGEFAIL              0x03 //!< Failed to enroll the finger
#define FINGERPRINT_IMAGEMESS              0x06 //!< Failed to generate character file due to overly disorderly
                                                //!< fingerprint image
#define FINGERPRINT_FEATUREFAIL            0x07 //!< Failed to generate character file due to the lack of character point
                                                //!< or small fingerprint image
#define FINGERPRINT_NOMATCH                0x08 //!< Finger doesn't match
#define FINGERPRINT_NOTFOUND               0x09 //!< Failed to find matching finger
#define FINGERPRINT_ENROLLMISMATCH         0x0A //!< Failed to combine the character files
#define FINGERPRINT_BADLOCATION            0x0B //!< Addressed PageID is beyond the finger library
#define FINGERPRINT_DBRANGEFAIL            0x0C //!< Error when reading template from library or invalid template
#define FINGERPRINT_UPLOADFEATUREFAIL      0x0D //!< Error when uploading template
#define FINGERPRINT_PACKETRESPONSEFAIL     0x0E //!< Module failed to receive the following data packages
#define FINGERPRINT_UPLOADFAIL             0x0F //!< Error when uploading image
#define FINGERPRINT_DELETEFAIL             0x10 //!< Failed to delete the template
#define FINGERPRINT_DBCLEARFAIL            0x11 //!< Failed to clear finger library
#define FINGERPRINT_PASSFAIL               0x13 //!< Find whether the fingerprint passed or failed
#define FINGERPRINT_INVALIDIMAGE           0x15 //!< Failed to generate image because of lac of valid primary image
#define FINGERPRINT_FLASHERR               0x18 //!< Error when writing flash
#define FINGERPRINT_INVALIDREG             0x1A //!< Invalid register number
#define FINGERPRINT_ADDRCODE               0x20 //!< Address code
#define FINGERPRINT_PASSVERIFY             0x21 //!< Verify the fingerprint passed
#define FINGERPRINT_STARTCODE              0xEF01 //!< Fixed falue of EF01H; High byte transferred first

#define FINGERPRINT_COMMANDPACKET          0x1 //!< Command packet
#define FINGERPRINT_DATAPACKET             0x2 //!< Data packet, must follow command packet or acknowledge packet
#define FINGERPRINT_ACKPACKET              0x7 //!< Acknowledge packet
#define FINGERPRINT_ENDDATAPACKET          0x8 //!< End of data packet

#define FINGERPRINT_TIMEOUT                0xFF //!< Timeout was reached
#define FINGERPRINT_BADPACKET              0xFE //!< Bad packet was sent

#define FINGERPRINT_GETIMAGE               0x01 //!< Collect finger image
#define FINGERPRINT_IMAGE2TZ               0x02 //!< Generate character file from image
#define FINGERPRINT_SEARCH                 0x04 //!< Search for fingerprint in slot
#define FINGERPRINT_REGMODEL               0x05 //!< Combine character files and generate template
#define FINGERPRINT_STORE                  0x06 //!< Store template
#define FINGERPRINT_LOAD                   0x07 //!< Read/load template
#define FINGERPRINT_UPLOAD                 0x08 //!< Upload template
#define FINGERPRINT_DELETE                 0x0C //!< Delete templates
#define FINGERPRINT_EMPTY                  0x0D //!< Empty library
#define FINGERPRINT_READSYSPARAM           0x0F //!< Read system parameters
#define FINGERPRINT_SETPASSWORD            0x12 //!< Sets passwords
#define FINGERPRINT_VERIFYPASSWORD         0x13 //!< Verifies the password
#define FINGERPRINT_HISPEEDSEARCH          0x1B //!< Asks the sensor to search for a matching fingerprint template to the
                                                //!< last model generated
#define FINGERPRINT_TEMPLATECOUNT          0x1D //!< Read finger template numbers
#define FINGERPRINT_AURALEDCONFIG          0x35 //!< Aura LED control
#define FINGERPRINT_LEDON                  0x50 //!< Turn on the onboard LED
#define FINGERPRINT_LEDOFF                 0x51 //!< Turn off the onboard LED

#define FINGERPRINT_LED_BREATHING          0x01 //!< Breathing light
#define FINGERPRINT_LED_FLASHING           0x02 //!< Flashing light
#define FINGERPRINT_LED_ON                 0x03 //!< Always on
#define FINGERPRINT_LED_OFF                0x04 //!< Always off
#define FINGERPRINT_LED_GRADUAL_ON         0x05 //!< Gradually on
#define FINGERPRINT_LED_GRADUAL_OFF        0x06 //!< Gradually off
#define FINGERPRINT_LED_RED                0x01 //!< Red LED
#define FINGERPRINT_LED_BLUE               0x02 //!< Blue LED
#define FINGERPRINT_LED_PURPLE             0x03 //!< Purple LEDpassword

#define FINGERPRINT_REG_ADDR_ERROR         0x1A //!< shows register address error
#define FINGERPRINT_WRITE_REG              0x0E //!< Write system register instruction

#define FINGERPRINT_BAUD_REG_ADDR          0x4  //!< BAUDRATE register address
#define FINGERPRINT_BAUDRATE_9600          0x1  //!< UART baud 9600
#define FINGERPRINT_BAUDRATE_19200         0x2  //!< UART baud 19200
#define FINGERPRINT_BAUDRATE_28800         0x3  //!< UART baud 28800
#define FINGERPRINT_BAUDRATE_38400         0x4  //!< UART baud 38400
#define FINGERPRINT_BAUDRATE_48000         0x5  //!< UART baud 48000
#define FINGERPRINT_BAUDRATE_57600         0x6  //!< UART baud 57600
#define FINGERPRINT_BAUDRATE_67200         0x7  //!< UART baud 67200
#define FINGERPRINT_BAUDRATE_76800         0x8  //!< UART baud 76800
#define FINGERPRINT_BAUDRATE_86400         0x9  //!< UART baud 86400
#define FINGERPRINT_BAUDRATE_96000         0xA  //!< UART baud 96000
#define FINGERPRINT_BAUDRATE_105600        0xB  //!< UART baud 105600
#define FINGERPRINT_BAUDRATE_115200        0xC  //!< UART baud 115200

#define FINGERPRINT_SECURITY_REG_ADDR      0x5  //!< Security level register address
// The safety level is 1 The highest rate of false recognition , The rejection
// rate is the lowest . The safety level is 5 The lowest tate of false
// recognition, The rejection rate is the highest .
#define FINGERPRINT_SECURITY_LEVEL_1       0X1 //!< Security level 1
#define FINGERPRINT_SECURITY_LEVEL_2       0X2 //!< Security level 2
#define FINGERPRINT_SECURITY_LEVEL_3       0X3 //!< Security level 3
#define FINGERPRINT_SECURITY_LEVEL_4       0X4 //!< Security level 4
#define FINGERPRINT_SECURITY_LEVEL_5       0X5 //!< Security level 5

#define FINGERPRINT_PACKET_REG_ADDR        0x6 //!< Packet size register address
#define FINGERPRINT_PACKET_SIZE_32         0X0 //!< Packet size is 32 Byte
#define FINGERPRINT_PACKET_SIZE_64         0X1 //!< Packet size is 64 Byte
#define FINGERPRINT_PACKET_SIZE_128        0X2 //!< Packet size is 128 Byte
#define FINGERPRINT_PACKET_SIZE_256        0X3 //!< Packet size is 256 Byte

//#define FINGERPRINT_DEBUG

#define DEFAULTTIMEOUT                     1000 //!< UART reading timeout in milliseconds

typedef enum as608_err_t{
  AS608_OK = 0,
  AS608_ERROR = -1
}as608_err_t;

/***************************************************************************************************************************************************/
/*                                                         HELPER INFO STRUCTURE                                                                   */
/***************************************************************************************************************************************************/
///! Helper structure to craft UART packets
typedef struct  {
  uint16_t start_code;   ///< "Wakeup" code for packet detection
  uint8_t address[4];    ///< 32-bit Fingerprint sensor address
  uint8_t type;          ///< Type of packet
  uint16_t length;       ///< Length of packet
  uint8_t data[64];      ///< Data contains 1 byte Scripts
                         //                 4 bytes data packet optinal
                         //                 2 bytes checksum = type + length + script + data (sum of values per byte)
}Fingerprint_Packet;

///! Helper structure for finger print information 
typedef struct {
   /// The matching location that is set by fingerFastSearch()
  uint16_t fingerID;
  /// The confidence of the fingerFastSearch() match, higher numbers are more
  /// confidents
  uint16_t confidence;
  /// The number of stored templates in the sensor, set by getTemplateCount()
  uint16_t templateCount;

  uint16_t status_reg;           ///< The status register (set by getParameters)
  uint16_t system_id;            ///< The system identifier (set by getParameters)
  uint16_t capacity;             ///< The fingerprint capacity (set by getParameters)
  uint16_t security_level;        ///< The security level (set by getParameters)
  uint32_t device_addr;  ///< The device address (set by getParameters)
  uint16_t packet_len;           ///< The max packet length (set by getParameters)
  uint16_t baud_rate;         ///< The UART baud rate (set by getParameters)
  uint32_t thePassword;
  uint32_t theAddress;
  uint8_t recvPacket[20];
}Fingureprint;
    

/***************************************************************************************************************************************************/
/*                                                         HARDWARE FUNCTION                                                                       */
/***************************************************************************************************************************************************/
void as608Init(Fingureprint *fp, uint32_t password);
Fingerprint_Packet packetInit( uint8_t type, uint16_t length, uint8_t *data);

as608_err_t begin(uint32_t baudrate);

bool verifyPassword(Fingureprint *fp);
uint8_t getParameters(Fingureprint *fp);

uint8_t getImage(void);
uint8_t image2Tz(uint8_t slot);
uint8_t createModel(void);

uint8_t emptyDatabase(void);
uint8_t storeModel(uint16_t id);
uint8_t loadModel(uint16_t id);
uint8_t getModel(void);
uint8_t deleteModel(uint16_t id);
uint8_t fingerFastSearch(Fingureprint *fp);
uint8_t fingerSearch(Fingureprint *fp, uint8_t slot);
uint8_t getTemplateCount(Fingureprint *fp);
uint8_t setPassword(uint32_t password);
uint8_t LEDcontrol(bool on);
uint8_t LEDcontrol_2(uint8_t control, uint8_t speed, uint8_t coloridx, uint8_t count);
uint8_t setBaudRate(uint8_t baudrate);
uint8_t setSecurityLevel(uint8_t level);
uint8_t setPacketSize(uint8_t size);

as608_err_t writeStructuredPacket(Fingerprint_Packet *p);
uint8_t getStructuredPacket(Fingerprint_Packet *p, uint16_t timeout);
uint8_t checkPassword(Fingureprint *fp);
uint8_t writeRegister(uint8_t regAdd, uint8_t value);

/***************************************************************************************************************************************************/
/*                                                         USER APPLICATION FUNCTION                                                               */
/***************************************************************************************************************************************************/
as608_err_t enrollUser(uint16_t userID);
uint8_t searchUser(Fingureprint *fp);

#endif
