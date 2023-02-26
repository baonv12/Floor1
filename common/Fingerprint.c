/*
 *@brief: Library for AS608 module using in esp32-idf 
 *@author: minhnt29
 *@Note : Retranslate from Adafruit_Fingerprint.h
 *@license : None
 */

#include "Fingerprint.h"
#include "i2c-lcd.h"

static const int RX_BUF_SIZE = 256;

/*!
 * @brief Gets the command packet
 */
#define GET_CMD_PACKET(...)                                                        \
  uint8_t data[] = {__VA_ARGS__};                                                  \
  Fingerprint_Packet packet = packetInit(FINGERPRINT_COMMANDPACKET, sizeof(data),  \
                                     data);                                        \
  writeStructuredPacket(&packet);                                                  \
  if (getStructuredPacket(&packet, DEFAULTTIMEOUT) != FINGERPRINT_OK)                              \
    return FINGERPRINT_PACKETRECIEVEERR;                                           \
  if (packet.type != FINGERPRINT_ACKPACKET)                                        \
    return FINGERPRINT_PACKETRECIEVEERR;                                           

/*!
 * @brief Sends the command packet
 */
#define SEND_CMD_PACKET(...)                                                   \
  GET_CMD_PACKET(__VA_ARGS__);                                                 \
  return packet.data[0];


/**************************************************************************/
/*!
      @brief   Create a new UART packet
      @param   type Command, data, ack type packet
      @param   length Size of payload
      @param   data Pointer to bytes of size length we will memcopy into the
      internal buffer
*/
/**************************************************************************/
Fingerprint_Packet packetInit(uint8_t type, uint16_t length, uint8_t *data) {
    Fingerprint_Packet fpp;
    fpp.start_code = FINGERPRINT_STARTCODE;
    fpp.type = type;
    fpp.length = length;
    fpp.address[0] = 0xFF;
    fpp.address[1] = 0xFF;
    fpp.address[2] = 0xFF;
    fpp.address[3] = 0xFF;
    if (length < 64)
      memcpy(fpp.data, data, length);
    else
      memcpy(fpp.data, data, 64);
    return fpp;
}
/**************************************************************************/
/*!
    @brief  Instantiates sensor with UART
    @param  hs Pointer to HardwareSerial object
    @param  password 32-bit integer password (default is 0)

*/
/**************************************************************************/
void as608Init(Fingureprint *fp, uint32_t password){
  //Setup params for fingure print
  fp->thePassword = password;
  fp->theAddress = 0xFFFFFFFF;
}
/**************************************************************************/
/*!
    @brief  Initializes serial interface and baud rate
    @param  baudrate Sensor's UART baud rate (usually 57600, 9600 or 115200)
*/
/**************************************************************************/
as608_err_t begin(uint32_t baudrate) {
  vTaskDelay(1000 / portTICK_PERIOD_MS); // one second delay to let the sensor 'boot up'

  //Init UART parameters
  const uart_config_t uart_config = {
    .baud_rate = baudrate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
  };
  // We won't use a buffer for sending data.
  if(uart_driver_install(AS608_UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0) != ESP_OK){
    return AS608_ERROR;
  }
  if(uart_param_config(AS608_UART_PORT, &uart_config) != ESP_OK){
    return AS608_ERROR;
  }
  if(uart_set_pin(AS608_UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK){
    return AS608_ERROR;
  }
  return AS608_OK;
}

/**************************************************************************/
/*!
    @brief  Verifies the sensors' access password (default password is
   0x0000000). A good way to also check if the sensors is active and responding
    @returns True if password is correct
*/
/**************************************************************************/
bool verifyPassword(Fingureprint *fp) {
  return checkPassword(fp) == FINGERPRINT_OK;
}

uint8_t checkPassword(Fingureprint *fp) {
  GET_CMD_PACKET(FINGERPRINT_VERIFYPASSWORD, (uint8_t)(fp->thePassword >> 24), (uint8_t)(fp->thePassword >> 16), (uint8_t)(fp->thePassword >> 8), (uint8_t)(fp->thePassword & 0xFF));
  if (packet.data[0] == FINGERPRINT_OK)
    return FINGERPRINT_OK;
  else
    return FINGERPRINT_PACKETRECIEVEERR;
}

/**************************************************************************/
/*!
    @brief  Get the sensors parameters, fills in the member variables
    status_reg, system_id, capacity, security_level, device_addr, packet_len
    and baud_rate
    @returns True if password is correct
*/
/**************************************************************************/
uint8_t getParameters(Fingureprint *fp) {
  GET_CMD_PACKET(FINGERPRINT_READSYSPARAM);

  fp->status_reg = ((uint16_t)packet.data[1] << 8) | packet.data[2];
  fp->system_id = ((uint16_t)packet.data[3] << 8) | packet.data[4];
  fp->capacity = ((uint16_t)packet.data[5] << 8) | packet.data[6];
  fp->security_level = ((uint16_t)packet.data[7] << 8) | packet.data[8];
  fp->device_addr = ((uint32_t)packet.data[9] << 24) |
                    ((uint32_t)packet.data[10] << 16) |
                    ((uint32_t)packet.data[11] << 8) | (uint32_t)packet.data[12];
  fp->packet_len = ((uint16_t)packet.data[13] << 8) | packet.data[14];
  if (fp->packet_len == 0) {
    fp->packet_len = 32;
  } else if (fp->packet_len == 1) {
    fp->packet_len = 64;
  } else if (fp->packet_len == 2) {
    fp->packet_len = 128;
  } else if (fp->packet_len == 3) {
    fp->packet_len = 256;
  }
  fp->baud_rate = (((uint16_t)packet.data[15] << 8) | packet.data[16]) * 9600;

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take an image of the finger pressed on surface
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_NOFINGER</code> if no finger detected
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_IMAGEFAIL</code> on imaging error
*/
/**************************************************************************/
uint8_t getImage(void) {
  SEND_CMD_PACKET(FINGERPRINT_GETIMAGE);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to convert image to feature template
    @param slot Location to place feature template (put one in 1 and another in
   2 for verification to create model)
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_IMAGEMESS</code> if image is too messy
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_FEATUREFAIL</code> on failure to identify
   fingerprint features
    @returns <code>FINGERPRINT_INVALIDIMAGE</code> on failure to identify
   fingerprint features
*/
uint8_t image2Tz(uint8_t slot) {
  SEND_CMD_PACKET(FINGERPRINT_IMAGE2TZ, slot);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take two print feature template and create a
   model
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
*/
uint8_t createModel(void) {
  SEND_CMD_PACKET(FINGERPRINT_REGMODEL);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to store the calculated model for later matching
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t storeModel(uint16_t location) {
  SEND_CMD_PACKET(FINGERPRINT_STORE, 0x01, (uint8_t)(location >> 8),
                  (uint8_t)(location & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to load a fingerprint model from flash into buffer 1
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t loadModel(uint16_t location) {
  SEND_CMD_PACKET(FINGERPRINT_LOAD, 0x01, (uint8_t)(location >> 8),
                  (uint8_t)(location & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to transfer 256-byte fingerprint template from the
   buffer to the UART
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t getModel(void) {
  SEND_CMD_PACKET(FINGERPRINT_UPLOAD, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete a model in memory
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t deleteModel(uint16_t location) {
  SEND_CMD_PACKET(FINGERPRINT_DELETE, (uint8_t)(location >> 8),
                  (uint8_t)(location & 0xFF), 0x00, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete ALL models in memory
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written
   to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t emptyDatabase(void) {
  SEND_CMD_PACKET(FINGERPRINT_EMPTY);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot 1 fingerprint features to
   match saved templates. The matching location is stored in <b>fingerID</b> and
   the matching confidence in <b>confidence</b>
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t fingerFastSearch(Fingureprint *fp) {
  // high speed search of slot #1 starting at page 0x0000 and page #0x00A3
  GET_CMD_PACKET(FINGERPRINT_HISPEEDSEARCH, 0x01, 0x00, 0x00, 0x00, 0xA3);
  fp->fingerID = 0xFFFF;
  fp->confidence = 0xFFFF;

  fp->fingerID = packet.data[1];
  fp->fingerID <<= 8;
  fp->fingerID |= packet.data[2];

  fp->confidence = packet.data[3];
  fp->confidence <<= 8;
  fp->confidence |= packet.data[4];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Control the built in LED
    @param on True if you want LED on, False to turn LED off
    @returns <code>FINGERPRINT_OK</code> on success
*/
/**************************************************************************/
uint8_t LEDcontrol(bool on) {
  if (on) {
    SEND_CMD_PACKET(FINGERPRINT_LEDON);
  } else {
    SEND_CMD_PACKET(FINGERPRINT_LEDOFF);
  }
}

/**************************************************************************/
/*!
    @brief   Control the built in Aura LED (if exists). Check datasheet/manual
    for different colors and control codes available
    @param control The control code (e.g. breathing, full on)
    @param speed How fast to go through the breathing/blinking cycles
    @param coloridx What color to light the indicator
    @param count How many repeats of blinks/breathing cycles
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t LEDcontrol_2(uint8_t control, uint8_t speed,
                                         uint8_t coloridx, uint8_t count) {
  SEND_CMD_PACKET(FINGERPRINT_AURALEDCONFIG, control, speed, coloridx, count);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot fingerprint features to
   match saved templates. The matching location is stored in <b>fingerID</b> and
   the matching confidence in <b>confidence</b>
   @param slot The slot to use for the print search, defaults to 1
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t fingerSearch(Fingureprint *fp, uint8_t slot) {
  // search of slot starting thru the capacity
  GET_CMD_PACKET(FINGERPRINT_SEARCH, slot, 0x00, 0x00, (uint8_t)(fp->capacity >> 8),
                 (uint8_t)(fp->capacity & 0xFF));

  fp->fingerID = 0xFFFF;
  fp->confidence = 0xFFFF;

  fp->fingerID = packet.data[1];
  fp->fingerID <<= 8;
  fp->fingerID |= packet.data[2];

  fp->confidence = packet.data[3];
  fp->confidence <<= 8;
  fp->confidence |= packet.data[4];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Ask the sensor for the number of templates stored in memory. The
   number is stored in <b>templateCount</b> on success.
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t getTemplateCount(Fingureprint *fp) {
  GET_CMD_PACKET(FINGERPRINT_TEMPLATECOUNT);

  fp->templateCount = packet.data[1];
  fp->templateCount <<= 8;
  fp->templateCount |= packet.data[2];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Set the password on the sensor (future communication will require
   password verification so don't forget it!!!)
    @param   password 32-bit password code
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t setPassword(uint32_t password) {
  SEND_CMD_PACKET(FINGERPRINT_SETPASSWORD, (uint8_t)(password >> 24),
                  (uint8_t)(password >> 16), (uint8_t)(password >> 8),
                  (uint8_t)(password & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Writing module registers
    @param   regAdd 8-bit address of register
    @param   value 8-bit value will write to register
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ADDRESS_ERROR</code> on register address error
*/
/**************************************************************************/
uint8_t writeRegister(uint8_t regAdd, uint8_t value) {

  SEND_CMD_PACKET(FINGERPRINT_WRITE_REG, regAdd, value);
}

/**************************************************************************/
/*!
    @brief   Change UART baudrate
    @param   baudrate 8-bit Uart baudrate
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t setBaudRate(uint8_t baudrate) {

  return (writeRegister(FINGERPRINT_BAUD_REG_ADDR, baudrate));
}

/**************************************************************************/
/*!
    @brief   Change security level
    @param   level 8-bit security level
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t setSecurityLevel(uint8_t level) {

  return (writeRegister(FINGERPRINT_SECURITY_REG_ADDR, level));
}

/**************************************************************************/
/*!
    @brief   Change packet size
    @param   size 8-bit packet size
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t setPacketSize(uint8_t size) {

  return (writeRegister(FINGERPRINT_PACKET_REG_ADDR, size));
}
/**************************************************************************/
/*!
    @brief   Helper function to process a packet and send it over UART to the
   sensor
    @param   packet A structure containing the bytes to transmit
*/
/**************************************************************************/

as608_err_t writeStructuredPacket(Fingerprint_Packet *packet) {
  uint16_t temp;
  uint8_t byte_temp;
  
  //Send Start code
  temp = packet->start_code;
  byte_temp = (uint8_t)(temp >> 8);
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  byte_temp = (uint8_t)(temp & 0xFF);
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  //Send Address
  byte_temp = packet->address[0];
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  byte_temp = packet->address[1];
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  byte_temp = packet->address[2];
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  byte_temp = packet->address[3];
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  //Send Packet ID
  byte_temp = packet->type;
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  //Send length
  //@Note that must plus length with size of checksum =2 
  packet->length = packet->length +2;
  temp = packet->length;
  byte_temp = (uint8_t)(temp >> 8);
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  byte_temp = (uint8_t)(temp & 0xFF);
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  //Send data payload
  temp = (packet->length >> 8) + (packet->length & 0xFF) + packet->type; 
  for (uint8_t i = 0; i < packet->length - 2; i++)
  {
    byte_temp = packet->data[i];
    if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
    {
      return AS608_ERROR;
    }
    temp += byte_temp; 
  }
  //Send Checksum data
  byte_temp = (uint8_t)(temp >> 8);
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }
  byte_temp = (uint8_t)(temp & 0xFF);
  if(uart_write_bytes(AS608_UART_PORT, &byte_temp, 1) != 1)
  {
    return AS608_ERROR;
  }

  return AS608_OK;
}

/**************************************************************************/
/*!
    @brief   Helper function to receive data over UART from the sensor and
   process it into a packet
    @param   packet A structure containing the bytes received
    @param   timeout how many milliseconds we're willing to wait
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_TIMEOUT</code> or
   <code>FINGERPRINT_BADPACKET</code> on failure
*/
/**************************************************************************/
uint8_t getStructuredPacket(Fingerprint_Packet *packet, uint16_t timeout) {
  uint8_t byte;
  uint16_t idx = 0;
  
  
  while (true) {
    if (uart_read_bytes(AS608_UART_PORT, &byte, 1, timeout / portTICK_PERIOD_MS) == -1)
    {
      return FINGERPRINT_TIMEOUT;
    }
    switch (idx) {
    case 0:
      if (byte != (FINGERPRINT_STARTCODE >> 8))
        continue;
      packet->start_code = (uint16_t)byte << 8;
      break;
    case 1:
      packet->start_code |= byte;
      if (packet->start_code != FINGERPRINT_STARTCODE)
        return FINGERPRINT_BADPACKET;
      break;
    case 2:
    case 3:
    case 4:
    case 5:
      packet->address[idx - 2] = byte;
      break;
    case 6:
      packet->type = byte;
      break;
    case 7:
      packet->length = (uint16_t)byte << 8;
      break;
    case 8:
      packet->length |= byte;
      break;
    default:
      packet->data[idx - 9] = byte;
      //Stop condition
      if ((idx - 8) == packet->length) {
        return FINGERPRINT_OK;
      }
      break;
    }
    idx++;
    if ((idx + 9) >= sizeof(packet->data)) {
      return FINGERPRINT_BADPACKET;
    }
  }
  // Shouldn't get here so...
  return FINGERPRINT_BADPACKET;
}

/***************************************************************************************************************************************************/
/*                                                         USER APPLICATION FUNCTION                                                               */
/***************************************************************************************************************************************************/
extern uint8_t enrol_time;
as608_err_t enrollUser(uint16_t userID)
{
  int p = -1;
  ESP_LOGI("Fingerprint.c", "Waiting for valid finger to enroll as #%d", userID ); 
  while (p != FINGERPRINT_OK) {
    if(!enrol_time) return AS608_ERROR;
    p = getImage();
    switch (p) {
    case FINGERPRINT_OK:
        ESP_LOGI("Fingerprint.c", "Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      ESP_LOGI("Fingerprint.c", ".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      ESP_LOGI("Fingerprint.c", "Communication error");
      return AS608_ERROR;
    case FINGERPRINT_IMAGEFAIL:
      ESP_LOGI("Fingerprint.c", "Imaging error");
      return AS608_ERROR;
    default:
      ESP_LOGI("Fingerprint.c", "Unknown error");
      return AS608_ERROR;
    }
  }

  // OK success!

  p = image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      ESP_LOGI("Fingerprint.c", "Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      ESP_LOGI("Fingerprint.c", "Image too messy");
      return AS608_ERROR;
    case FINGERPRINT_PACKETRECIEVEERR:
      ESP_LOGI("Fingerprint.c", "Communication error");
      return AS608_ERROR;
    case FINGERPRINT_FEATUREFAIL:
      ESP_LOGI("Fingerprint.c", "Could not find fingerprint features");
      return AS608_ERROR;
    case FINGERPRINT_INVALIDIMAGE:
      ESP_LOGI("Fingerprint.c", "Could not find fingerprint features");
      return AS608_ERROR;
    default:
      ESP_LOGI("Fingerprint.c", "Unknown error");
      return AS608_ERROR;
  }

  ESP_LOGI("Fingerprint.c", "Remove finger");
  lcd_clear();
  lcd_put_cur(0, 2);
  lcd_send_string("Remove finger");

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = getImage();
  }
  ESP_LOGI("Fingerprint.c", "ID: %d", userID ); 
  p = -1;
  ESP_LOGI("Fingerprint.c", "Place same finger again");
  lcd_clear();
  lcd_put_cur(0, 3);
  lcd_send_string("Place same");
  lcd_put_cur(1, 2);
  lcd_send_string("finger again");

  while (p != FINGERPRINT_OK) {
    p = getImage();
    switch (p) {
    case FINGERPRINT_OK:
      ESP_LOGI("Fingerprint.c", "Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      ESP_LOGI("Fingerprint.c", ".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      ESP_LOGI("Fingerprint.c", "Communication error");
      return AS608_ERROR;
    case FINGERPRINT_IMAGEFAIL:
      ESP_LOGI("Fingerprint.c", "Imaging error");
      return AS608_ERROR;
    default:
      ESP_LOGI("Fingerprint.c", "Unknown error");
      return AS608_ERROR;
    }
  }

  // OK success!
    p = image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      ESP_LOGI("Fingerprint.c", "Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      ESP_LOGI("Fingerprint.c", "Image too messy");
      return AS608_ERROR;
    case FINGERPRINT_PACKETRECIEVEERR:
      ESP_LOGI("Fingerprint.c", "Communication error");
      return AS608_ERROR;
    case FINGERPRINT_FEATUREFAIL:
      ESP_LOGI("Fingerprint.c", "Could not find fingerprint features");
      return AS608_ERROR;
    case FINGERPRINT_INVALIDIMAGE:
      ESP_LOGI("Fingerprint.c", "Could not find fingerprint features");
      return AS608_ERROR;
    default:
      ESP_LOGI("Fingerprint.c", "Unknown error");
      return AS608_ERROR;
  }

  // OK converted!
  ESP_LOGI("Fingerprint.c", "Creating model for #%d", userID);  

  p = createModel();
  if (p == FINGERPRINT_OK) {
    ESP_LOGI("Fingerprint.c", "Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    ESP_LOGI("Fingerprint.c", "Communication error");
    return AS608_ERROR;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    ESP_LOGI("Fingerprint.c", "Fingerprints did not match");
    return AS608_ERROR;
  } else {
    ESP_LOGI("Fingerprint.c", "Unknown error");
    return AS608_ERROR;
  }

  ESP_LOGI("Fingerprint.c", "ID %d", userID ); 
  p = storeModel(userID);
  if (p == FINGERPRINT_OK) {
    ESP_LOGI("Fingerprint.c", "Stored!");
    lcd_clear();
    lcd_put_cur(0, 2);
    lcd_send_string("Stored!");

  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    ESP_LOGI("Fingerprint.c", "Communication error");
    return AS608_ERROR;
  } else if (p == FINGERPRINT_BADLOCATION) {
    ESP_LOGI("Fingerprint.c", "Could not store in that location");
    return AS608_ERROR;
  } else if (p == FINGERPRINT_FLASHERR) {
    ESP_LOGI("Fingerprint.c", "Error writing to flash");
    return AS608_ERROR;
  } else {
    ESP_LOGI("Fingerprint.c", "Unknown error");
    return AS608_ERROR;
  }

  return AS608_OK;
}

uint8_t searchUser(Fingureprint *fp)
{ 
  uint8_t p = getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = image2Tz(1);
  if (p != FINGERPRINT_OK)  return -1;

  p = fingerFastSearch(fp);
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  ESP_LOGI("Fingerprint.c", "Found ID #"); 
  ESP_LOGI("Fingerprint.c", "%d", fp->fingerID);
  ESP_LOGI("Fingerprint.c", " with confidence of "); 
  ESP_LOGI("Fingerprint.c", "%d", fp->confidence);
  return fp->fingerID;
}
