#define I2C_BUFFER_LENGTH 256
#include "LoRaWan_APP.h"
#include "vl53l8cx_api.h"

enum state_type {
  IDLE,        
  PERSON_ENTERS, 
  PERSON_ENTERED,
  PERSON_LEAVES,
  PERSON_LEAVED,
  AT_DOOR
};

state_type State = IDLE;
VL53L8CX_ResultsData 	Results;	

int16_t thresholdDistance = 1500;
uint8_t numOfPersons = 0;
bool isPersonDetected = false;

uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x96, 0x0D };
uint8_t appEui[] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45 };
uint8_t appKey[] = { 0x64, 0x41, 0xFB, 0xEC, 0x45, 0x2A, 0xE9, 0xE5, 0x0A, 0xDC, 0x50, 0xAA, 0x53, 0x1D, 0xC9, 0x3F };

uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 0;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port, uint8_t num )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
    appDataSize = 1;
    appData[0] = num;
}

void sendDataToActility() {
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
      #if(LORAWAN_DEVEUI_AUTO)
            LoRaWAN.generateDeveuiByChipID();
      #endif
      LoRaWAN.init(loraWanClass,loraWanRegion);
      //both set join DR and DR when ADR off 
      LoRaWAN.setDefaultDR(3);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      if(isPersonDetected == true) {
        isPersonDetected = false;
        prepareTxFrame( appPort, numOfPersons );
        LoRaWAN.send();
        deviceState = DEVICE_STATE_SLEEP;
      }
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass);
      if(isPersonDetected == true) {
        deviceState = DEVICE_STATE_SEND;
      }
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

// parameter is 4 for 4x4 or 8 for 8x8
uint16_t measureAverageLeftDistance(int8_t type) {
  int16_t leftDist = 0;
  uint32_t sum = 0;
  for (int y = 0; y < type; y++) {
    for (int x = 0; x < type/2; x++) {
        sum += (uint16_t)Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * (type * y + x)];
    }
  }
  leftDist = (uint16_t)(sum / ((type*type) / 2));
  return leftDist;
}

uint16_t measureAverageRightDistance(int8_t type) {
  int16_t rightDist = 0;
  uint32_t sum = 0;
  for (int y = 0; y < type; y++) {
    for (int x = type/2; x < type; x++) {
        sum += (uint16_t)Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * (type * y + x)];
    }
  }
  rightDist = (uint16_t)(sum / ((type*type) / 2));
  return rightDist;
}

int ToF_4x4(void)
{

	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53L8CX_Configuration 	Dev;			/* Sensor configuration */
//	VL53L8CX_ResultsData 	Results;		/* Results data from VL53L8CX */


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	Reset_Sensor(&(Dev.platform));

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	status = vl53l8cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		Serial.print("VL53L8CX not detected at requested address\n");
		return status;
	}
	Serial.print("VL53L8CX detect OK!\n");

	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(&Dev);
	if(status)
	{
		Serial.print("VL53L8CX ULD Loading failed\n");
		return status;
	}

	Serial.println("VL53L8CX ULD ready !");

  status = vl53l8cx_set_ranging_frequency_hz(&Dev, 60);

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l8cx_start_ranging(&Dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A1
		 * (INT) when a new measurement is ready */
 
		status = vl53l8cx_check_data_ready(&Dev, &isReady);

		if(isReady)
		{
			vl53l8cx_get_ranging_data(&Dev, &Results);

         
      /*  DISPLAY RESULTS IN 16 POINTS  */
      /*
      Serial.println("\n******************************\n");
			for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {  
          char buf[100];
          sprintf(buf, "%4d\t", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * (4 * y + x)]);
          Serial.print(buf);
        }
        Serial.print("\n");  
      }
      Serial.println("\n******************************\n");
      */
      
			sendDataToActility();

      int16_t leftDistance = measureAverageLeftDistance(4);
      int16_t rightDistance = measureAverageRightDistance(4);

      switch(State) {
        case IDLE:
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_LEAVES;
          else if(rightDistance < thresholdDistance)
            State = PERSON_ENTERS;
          else
            State = IDLE;

          break;
        case PERSON_ENTERS: 
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_ENTERED;
          else if(rightDistance < thresholdDistance)
            State = PERSON_ENTERS;
          else
            State = IDLE;

          break;
        case PERSON_LEAVES: 
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_LEAVES;
          else if(rightDistance < thresholdDistance)
            State = PERSON_LEAVED;
          else
            State = IDLE;

          break;
        case AT_DOOR: 
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_ENTERED;
          else if(rightDistance < thresholdDistance)
            State = PERSON_LEAVED;
          else
            State = IDLE;

          break;
        case PERSON_ENTERED:  
        
          if(leftDistance > thresholdDistance && rightDistance > thresholdDistance) {
            numOfPersons += 1;
            char buffer[200];

            Serial.println("\n------------------------------\n");
            Serial.print("Left distance: ");
            Serial.print(leftDistance);
            Serial.print("\tRight distance: ");
            Serial.println(rightDistance);

            sprintf(buffer, "\nNumber of persons is %d", numOfPersons);
            Serial.print(buffer);
            Serial.println ("\n------------------------------\n");
            isPersonDetected = true;

            State = IDLE;
          }
          
          break;
        case PERSON_LEAVED: 

          if(leftDistance > thresholdDistance && rightDistance > thresholdDistance) {
            if(numOfPersons > 0) {
              numOfPersons -= 1;
              char buffer[200];

              Serial.println("\n------------------------------\n");
              Serial.print("Left distance: ");
              Serial.print(leftDistance);
              Serial.print("\tRight distance: ");
              Serial.println(rightDistance);

              sprintf(buffer, "\nNumber of persons is %d", numOfPersons);
              Serial.print(buffer);
              Serial.println ("\n------------------------------\n");
              isPersonDetected = true;
            }
            State = IDLE;
          }

          break;
        default:
          break;
      } 
		}
	}

	status = vl53l8cx_stop_ranging(&Dev);
	Serial.print("End of ULD demo\n");
	return status;
}

int ToF_8x8(void)
{

	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	uint32_t 				integration_time_ms;
	VL53L8CX_Configuration 	Dev;			/* Sensor configuration */
	//VL53L8CX_ResultsData 	Results;		/* Results data from VL53L8CX */

	
	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	* from the default one (filled with 0x20 for this example).
	*/
	//status = vl53l8cx_set_i2c_address(&Dev, 0x20);
	

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	status = vl53l8cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		Serial.print("VL53L8CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(&Dev);
	if(status)
	{
		Serial.print("VL53L8CX ULD Loading failed\n");
		return status;
	}

	Serial.println("VL53L8CX ULD ready ! ");
			

	/*********************************/
	/*        Set some params        */
	/*********************************/

	/* Set resolution in 8x8. WARNING : As others settings depend to this
	 * one, it must be the first to use.
	 */
	status = vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
	if(status)
	{
    Serial.print("vl53l8cx_set_resolution failed, status ");
    Serial.println(status);
		return status;
	}

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
	if(status)
	{
    Serial.print("vl53l8cx_set_ranging_frequency_hz failed, status ");
    Serial.println(status);
		return status;
	}

	/* Set target order to closest */
	status = vl53l8cx_set_target_order(&Dev, VL53L8CX_TARGET_ORDER_CLOSEST);
	if(status)
	{
    Serial.print("vl53l8cx_set_target_order failed, status ");
    Serial.println(status);
		return status;
	}

	/* Get current integration time */
	status = vl53l8cx_get_integration_time_ms(&Dev, &integration_time_ms);
	if(status)
	{
    Serial.print("vl53l8cx_get_integration_time_ms failed, status ");
    Serial.println(status);
		return status;
	}
  Serial.print("Current integration time is : ");
  Serial.print((int)integration_time_ms);
  Serial.println("ms");


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l8cx_start_ranging(&Dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A1
		 * (INT) when a new measurement is ready */
 
		status = vl53l8cx_check_data_ready(&Dev, &isReady);

		if(isReady)
		{
			vl53l8cx_get_ranging_data(&Dev, &Results);

			/*  DISPLAY RESULTS IN 64 POINTS  */
      /*
      Serial.println("\n******************************\n");
			for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {  
          char buf[100];
          sprintf(buf, "%4d\t", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * (8 * y + x)]);
          Serial.print(buf);
        }
        Serial.print("\n");  
      }
      Serial.println("\n******************************\n");
      */
      
			sendDataToActility();

      int16_t leftDistance = measureAverageLeftDistance(8);
      int16_t rightDistance = measureAverageRightDistance(8);

      switch(State) {
        case IDLE:
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_LEAVES;
          else if(rightDistance < thresholdDistance)
            State = PERSON_ENTERS;
          else
            State = IDLE;

          break;
        case PERSON_ENTERS: 
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_ENTERED;
          else if(rightDistance < thresholdDistance)
            State = PERSON_ENTERS;
          else
            State = IDLE;

          break;
        case PERSON_LEAVES: 
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_LEAVES;
          else if(rightDistance < thresholdDistance)
            State = PERSON_LEAVED;
          else
            State = IDLE;

          break;
        case AT_DOOR: 
        
          if(leftDistance < thresholdDistance && rightDistance < thresholdDistance)
            State = AT_DOOR;
          else if(leftDistance < thresholdDistance)
            State = PERSON_ENTERED;
          else if(rightDistance < thresholdDistance)
            State = PERSON_LEAVED;
          else
            State = IDLE;

          break;
        case PERSON_ENTERED:  
        
          if(leftDistance > thresholdDistance && rightDistance > thresholdDistance) {
            numOfPersons += 1;
            char buffer[200];

            Serial.println("\n------------------------------\n");
            Serial.print("Left distance: ");
            Serial.print(leftDistance);
            Serial.print("\tRight distance: ");
            Serial.println(rightDistance);

            sprintf(buffer, "\nNumber of persons is %d", numOfPersons);
            Serial.print(buffer);
            Serial.println ("\n------------------------------\n");
            isPersonDetected = true;

            State = IDLE;
          }
          
          break;
        case PERSON_LEAVED: 

          if(leftDistance > thresholdDistance && rightDistance > thresholdDistance) {
            if(numOfPersons > 0) {
              numOfPersons -= 1;
              char buffer[200];

              Serial.println("\n------------------------------\n");
              Serial.print("Left distance: ");
              Serial.print(leftDistance);
              Serial.print("\tRight distance: ");
              Serial.println(rightDistance);
              
              sprintf(buffer, "\nNumber of persons is %d", numOfPersons);
              Serial.print(buffer);
              Serial.println ("\n------------------------------\n");
              isPersonDetected = true;
            }
            State = IDLE;
          }

          break;
        default:
          break;
      } 
		}
	}

	status = vl53l8cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}

void setup() 
{
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.println("ToF test....");
  ToF_4x4();


  //ToF_8x8();
}

void loop() 
{
}
