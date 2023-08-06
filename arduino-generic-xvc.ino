
/*- Symbolic Constants ------------------------------------------------------*/
/*! Open Drain output type                                                   */
#define OPENDRAIN       INPUT

#define SERIAL_PORT SerialUSB

#define SERIAL_DEBUG Serial.println

/*! Message buffer size                                                      */
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO)
#define DATA_BUF_SIZE   632
#elif defined(ARDUINO_AVR_MEGA)
#define DATA_BUF_SIZE   1468
#elif defined(ARDUINO_SAM_DUE)
#define DATA_BUF_SIZE   16*1024
#else
#define DATA_BUF_SIZE   128
#endif
#define DATA_VEC_LEN    (DATA_BUF_SIZE * 8)

/*! Pin definitions for JTAG connection
 *  @{                                                                       */
#define TCK_PIN         2
#define TMS_PIN         3
#define TDI_PIN         4
#define TDO_PIN         5
/*! @}                                                                       */

/*! Status LED                                                               */
#define LED_PIN         13

/*! Macro for min/max limit                                                  */
#define limit(x, l, h)  ((x < l) ? l : ((x > h) ? h : x))


/*- Type definitions --------------------------------------------------------*/
/*!***************************************************************************
 * @brief
 * State coding for JTAG interface handler FSM
 *****************************************************************************/
typedef enum {
  /*! Start transmission                                                     */
  EN_JTAG_STATE_START,

  /*! Generate TCK rising edge                                               */
  EN_JTAG_STATE_RISINGEDGE,

  /*! Generate TCK falling edge                                              */
  EN_JTAG_STATE_FALLINGEDGE,

  /*! Transfer complete                                                      */
  EN_JTAG_STATE_DONE
} teJtagState;


/*! TMS data vector                                                          */
volatile uint8_t aucDataTMS[DATA_BUF_SIZE];

/*! TDI/TDO data vector                                                      */
volatile uint8_t aucDataTDx[DATA_BUF_SIZE];

/*! JTAG vector length (bits)                                                */
volatile uint16_t uiDataLen;

/*! Bit index in both JTAG data vectors                                      */
volatile uint16_t uiDataIndex;

/*! Interface handler state variable                                         */
volatile teJtagState eState;

/*! Active period                                                            */
uint32_t period_act = 1;

/*- Local Functions ---------------------------------------------------------*/
/*!***************************************************************************
 * @brief
 * Emulate Open Drain output
 * 
 * @param[in] pin   Same as pin for digitalWrite
 * @param[in] value "HIGH" will turn pin High-Z, "LOW" will pull pin low
 *****************************************************************************/
static void digitalWrite_OD(uint8_t pin, uint8_t value)
{
  pinMode(pin, (value == LOW) ? OUTPUT : OPENDRAIN);
  digitalWrite(pin, value);
}


/*!***************************************************************************
 * @brief
 * Read bit value from multibyte vector 
 *
 * @param[in] *vector   Bit vector
 * @param[in] bit       Bit index (0-indexed)
 * @return    bool      Bit value
 *****************************************************************************/
static bool getBitFromVector(const uint8_t* vector, uint16_t bit)
{
  bool value = vector[bit >> 3] & (1 << (bit & 0x7));
  return value;
}



/*!***************************************************************************
 * @brief
 * Set bit value in multibyte vector
 * 
 * @param[inout] *vector  Bit vector
 * @param[in] bit         Bit index (0-indexed)
 * @param[in] value       New bit value
 *****************************************************************************/
static void setBitInVector(uint8_t* vector, uint16_t bit, bool value)
{
  if (value)
  {
    vector[bit >> 3] |= 1 << (bit & 0x7);
  }
  else
  {
    vector[bit >> 3] &= ~(1 << (bit & 0x7));
  }
}

/*!***************************************************************************
 * @brief
 * Setup routine
 *****************************************************************************/
void setup()
{
  Serial.begin(115200);
  
  /* Start serial port at 115200 baud, no parity, 8 bit words and 1 stop bit */
  SERIAL_PORT.begin(115200);

  /* Initialise digital GPIO pins                                            */
  pinMode(TMS_PIN, OPENDRAIN);
  pinMode(TCK_PIN, OPENDRAIN);
  pinMode(TDO_PIN, OPENDRAIN);
  pinMode(TDI_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  /* Initialise state machine                                                */
  eState = EN_JTAG_STATE_DONE;
  uiDataLen = 0;
  uiDataIndex = 0;
  memset((uint8_t*)aucDataTMS, 0, sizeof(aucDataTMS));
  memset((uint8_t*)aucDataTDx, 0, sizeof(aucDataTDx));
}

/*!***************************************************************************
 * @brief
 * Background program loop
 *****************************************************************************/
void loop()
{
  SERIAL_DEBUG("Hello");
  String cmd = SERIAL_PORT.readStringUntil(':');
  if (cmd.compareTo("getinfo") == 0)
  {
    /* Get protocol version and max. vector length                           */
    SERIAL_PORT.print("xvcServer_v1.0:");
    SERIAL_PORT.print(DATA_BUF_SIZE, DEC);
    SERIAL_PORT.print("\n");
  }
  else if (cmd.compareTo("settck") == 0)
  {
    /* Set TCK period (in ns)                                                */
    uint32_t period_req;
    SERIAL_PORT.readBytes((uint8_t*)&period_req, 4);

    /* Period must be twice as fast for rising and falling edges             */
    period_act = (period_req / 2000UL) * 500UL;
    limit(period_act, 1, 1000);
    
    SERIAL_PORT.write((const uint8_t*)&period_act, 4);
  }
  else if (cmd.compareTo("shift") == 0)
  {
    /* Shift data in/out of the device                                       */
    uint32_t num;
    uint32_t num_bytes;
    SERIAL_PORT.readBytes((uint8_t*)&num, 4);

    num_bytes = (num + 7) >> 3;  /* (n + 7) / 8 */
    if (num_bytes <= DATA_BUF_SIZE)
    {
      /* Prepare data and start transfer                                     */
      SERIAL_PORT.readBytes((uint8_t*)aucDataTMS, num_bytes);
      SERIAL_PORT.readBytes((uint8_t*)aucDataTDx, num_bytes);
      memset((uint8_t*)aucDataTDx + num_bytes, 0, DATA_BUF_SIZE - num_bytes - 1);
      uiDataLen = num;
      eState = EN_JTAG_STATE_START;

      /* Wait until transfer is complete                                     */
      while (eState != EN_JTAG_STATE_DONE) {
        /* JTAG interface handler FSM                                              */
        switch (eState)
        {
          case EN_JTAG_STATE_START:
            /* Start new transfer
            * Set up initial TDI/TMS/TCK values                                   */
            uiDataIndex = 0;
            digitalWrite_OD(TCK_PIN, LOW);
            digitalWrite(LED_PIN, HIGH);
            digitalWrite_OD(TMS_PIN, getBitFromVector((const uint8_t*)aucDataTMS, uiDataIndex));
            digitalWrite_OD(TDI_PIN, getBitFromVector((const uint8_t*)aucDataTDx, uiDataIndex));
            eState = EN_JTAG_STATE_RISINGEDGE;
            break;

          case EN_JTAG_STATE_RISINGEDGE:
            /* Generate rising edge on TCK
            * Shift in TDO                                                        */
            if (uiDataIndex < uiDataLen)
            {
              setBitInVector((uint8_t*)aucDataTDx, uiDataIndex, digitalRead(TDO_PIN));
              digitalWrite_OD(TCK_PIN, HIGH);
              eState = EN_JTAG_STATE_FALLINGEDGE;
            }
            else
            {
              /* Shifted in last bit - transfer complete                           */
              digitalWrite(LED_PIN, LOW);
              eState = EN_JTAG_STATE_DONE;
            }
            break;

          case EN_JTAG_STATE_FALLINGEDGE:
            /* Generate falling edge on TCK
            * Set up new TMS/TDI                                                  */
            digitalWrite_OD(TCK_PIN, LOW);
            ++uiDataIndex;
            if (uiDataIndex < uiDataLen)
            {
              digitalWrite_OD(TMS_PIN, getBitFromVector((const uint8_t*)aucDataTMS, uiDataIndex));
              digitalWrite_OD(TDI_PIN, getBitFromVector((const uint8_t*)aucDataTDx, uiDataIndex));
            }
            eState = EN_JTAG_STATE_RISINGEDGE;
            break;

          case EN_JTAG_STATE_DONE:
          default:
            ;
        }
        //volatile int i = 0;
        //for(i = 0; i < 100; i++);
        delayMicroseconds(period_act);
      }
      /* Reply with TDO vector                                               */
      SERIAL_PORT.write((const char*)aucDataTDx, num_bytes);
    }
    else
    {
      /* Error - shift out zero-bytes                                        */
      do {
        SERIAL_PORT.write((uint8_t)'\0');
        --num_bytes;
      } while (num_bytes > 0);
    }
  }
}
