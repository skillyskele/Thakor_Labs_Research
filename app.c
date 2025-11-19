/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "sl_bt_api.h"
#include "sl_main_init.h"
#include "app_assert.h"
#include "app.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_iadc.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_ldma.h"
#include "em_letimer.h"
#include "gatt_db.h"
#include "sl_status.h"
#include "common_config.h"
#include "circular_buffer.h"
#include <stdio.h>

// Set CLK_ADC to 10MHz
#define CLK_SRC_ADC_FREQ          20000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ              10000000 // CLK_ADC - 10MHz max in normal mode

#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortAPin6; // ecg input pin
#define IADC_INPUT_1_PORT_PIN     iadcPosInputPortAPin7; // eeg input pin

#define IADC_INPUT_0_BUS          ABUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0
#define IADC_INPUT_1_BUS          ABUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0

// LDMA transfer complete GPIO toggle port/pin
#define LDMA_OUTPUT_0_PORT        gpioPortD
#define LDMA_OUTPUT_0_PIN         2

// Desired LETIMER frequency in Hz
#define LETIMER_FREQ              1000

// LETIMER GPIO toggle port/pin (toggled in EM2; requires port A/B GPIO)
#define LETIMER_OUTPUT_0_PORT     gpioPortA
#define LETIMER_OUTPUT_0_PIN      5

// Use specified LDMA/PRS channel
#define IADC_LDMA_CH              0
#define PRS_CHANNEL               0

// number of samples per buffer, and stuff like that, is all defined in common_config.h


/* This example enters EM2 in the infinite while loop; Setting this define to 1
 * enables debug connectivity in the EMU_CTRL register, which will consume about
 * 0.5uA additional supply current */
#define EM2DEBUG                  1

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/

/// Globally declared LDMA link descriptor
LDMA_Descriptor_t sampleLoop[2]; // sampleLoop0 will link to sampleLoop1, which loops back around for ping pong buffer functionality

// buffer to store IADC samples
struct {
    uint32_t buffer_A[SAMPLES_PER_BUFFER+1]; // one more space added for a packet ID!
    uint32_t buffer_B[SAMPLES_PER_BUFFER+1];
    volatile bool buffer_A_full;
    volatile bool buffer_B_full;
    uint32_t* dma_buffer;  // For writing process (dictated by interrupts)
    uint32_t* send_buffer; // For BLE Process (dictated by OS)
} ping_pong;


/**************************************************************************//**
 * @brief  GPIO Initializer
 *****************************************************************************/
void initGPIO (void)
{
  // Enable GPIO clock branch
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure LDMA/LETIMER as outputs
  GPIO_PinModeSet(LDMA_OUTPUT_0_PORT, LDMA_OUTPUT_0_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(LETIMER_OUTPUT_0_PORT, LETIMER_OUTPUT_0_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  PRS Initializer
 *****************************************************************************/
void initPRS (void)
{
  // Use LETIMER0 as async PRS to trigger IADC in EM2
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Set up PRS LETIMER and IADC as producer and consumer respectively */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0, PRS_LETIMER0_CH0);
  PRS_ConnectConsumer(PRS_CHANNEL, prsTypeAsync, prsConsumerIADC0_SCANTRIGGER);
}

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT; // Scan Table

  // Enable IADC0 clock branch
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  /*
   * These two settings are modified from the defaults to reduce the
   * IADC current.  In low-frequency use cases, such as this example,
   * iadcWarmupNormal shuts down the IADC between conversions, which
   * reduces current at the expense of requiring 5 microseconds of
   * warm-up time before a conversion can begin.
   *
   * In cases where a PRS event triggers scan conversions, enabling
   * iadcClkSuspend0 gates off the ADC_CLK until the PRS trigger event
   * occurs and again upon the completion of the channel conversions
   * specified in the scan table.
   */
  init.warmup = iadcWarmupNormal;
  init.iadcClkSuspend0 = true;

  /*
   * Configuration 0 is used by both scan and single conversions by
   * default.  Use internal bandgap as the reference and specify the
   * reference voltage in mV.
   *
   * Resolution is not configurable directly but is based on the
   * selected oversampling ratio (osrHighSpeed), which defaults to
   * 2x and generates 12-bit results.
   */
  initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
  initAllConfigs.configs[0].vRef = 1210;
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;// I believe if I wanted to get 16 bit resolution, I'd make OSR=32, and it'll automatically adjust
  initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;

  /*
   * CLK_SRC_ADC must be prescaled by some value greater than 1 to
   * derive the intended CLK_ADC frequency.
   *
   * Based on the default 2x oversampling rate (OSRHS)...
   *
   * conversion time = ((4 * OSRHS) + 2) / fCLK_ADC
   *
   * ...which results in a maximum sampling rate of 833 ksps with the
   * 2-clock input multiplexer switching time is included.
   */
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

  /*
   * Trigger conversions on the PRS0 rising edge input (PRS0 is not a
   * specific channel but simply the dedicated trigger input for scan
   * conversions; PRS1 serves the same purpose for single conversions).
   *
   * Set the SCANFIFODVL flag when there are 2 entries in the scan
   * FIFO.  Note that in this example, the interrupt associated with
   * the SCANFIFODVL flag in the IADC_IF register is not used.
   *
   * Enable DMA wake-up to save the results when the specified FIFO
   * level is hit.
   *
   * Allow a scan conversion sequence to start as soon as there is a
   * trigger event.
   */
  initScan.triggerSelect = iadcTriggerSelPrs0PosEdge;
  initScan.dataValidLevel = iadcFifoCfgDvl1; // should be lvl # channels, but is 1 because we use 1 channel for now
  initScan.fifoDmaWakeup = true;
  initScan.start = true;

  /*
   * Configure entries in scan table.  CH0 is single-ended from
   * input 0; CH1 is single-ended from input 1.
   */
  initScanTable.entries[0].posInput = IADC_INPUT_0_PORT_PIN;
  initScanTable.entries[0].negInput = iadcNegInputGnd;
  initScanTable.entries[0].includeInScan = true;

  /** we'll just stick with one channel for now
  initScanTable.entries[1].posInput = IADC_INPUT_1_PORT_PIN;
  initScanTable.entries[1].negInput = iadcNegInputGnd;
  initScanTable.entries[1].includeInScan = true;
  */

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initScan(IADC0, &initScan, &initScanTable);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  //GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
}

/**************************************************************************//**
 * @brief Clock initialization
 *****************************************************************************/
void initClock(void)
{
  CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

  // Select LFXO for the LETIMER
  CMU_LFXOInit(&lfxoInit);
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);
}

/**************************************************************************//**
 * @brief LETIMER initialization
 *****************************************************************************/
void initLetimer(void)
{
  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  // Enable LETIMER0 clock tree
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Calculate the top value (frequency) based on clock source
  uint32_t topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / LETIMER_FREQ;

  // Reload top on underflow, pulse output, and run in free mode
  letimerInit.comp0Top = true;
  letimerInit.topValue = topValue;
  letimerInit.ufoa0 = letimerUFOAPulse;
  letimerInit.repMode = letimerRepeatFree;

  // Enable LETIMER0 output0
  GPIO->LETIMERROUTE.ROUTEEN = GPIO_LETIMER_ROUTEEN_OUT0PEN;
  GPIO->LETIMERROUTE.OUT0ROUTE = \
      (LETIMER_OUTPUT_0_PORT << _GPIO_LETIMER_OUT0ROUTE_PORT_SHIFT) \
      | (LETIMER_OUTPUT_0_PIN << _GPIO_LETIMER_OUT0ROUTE_PIN_SHIFT);

  // Initialize LETIMER
  LETIMER_Init(LETIMER0, &letimerInit);
}

/**************************************************************************//**
 * @brief
 *   LDMA Initializer
 *
 * @param[in] buffer
 *   pointer to the array where ADC data will be stored.
 * @param[in] size
 *   size of the array
 *****************************************************************************/
void initLDMA(uint32_t *buffer0, uint32_t *buffer1, uint32_t size)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // Configure LDMA for transfer from IADC to memory
  // LDMA will loop continuously
  LDMA_TransferCfg_t transferCfg =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);

  // Set up descriptors for ping pong buffers
  //descriptor1 = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SCANFIFODATA, buffer, size, 0);

  sampleLoop[0] = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(
          &IADC0->SCANFIFODATA, buffer0, size, 1);   // Link to next (+1)

  sampleLoop[1] = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(
          &IADC0->SCANFIFODATA, buffer1, size, -1);  // Link back around (-1)

  sampleLoop[0].xfer.xferCnt = size - 1;  // Transfer count
  sampleLoop[0].xfer.doneIfs = 1;         // Interrupt when done

  sampleLoop[1].xfer.xferCnt = size - 1;  // Transfer count
  sampleLoop[1].xfer.doneIfs = 1;         // Interrupt when done



  // Initialize LDMA with default configuration
  LDMA_Init(&init);

  // Start transfer, LDMA will sample the IADC NUM_SAMPLES time, and then interrupt
  LDMA_StartTransfer(IADC_LDMA_CH, &transferCfg, &sampleLoop[0]);
}

volatile bool blueToothNotif = false;

uint32_t* toggleBuffer(uint32_t* current_buffer) {
    if (current_buffer == ping_pong.buffer_A) {
        return ping_pong.buffer_B;
    } else {
        return ping_pong.buffer_A;
    }
}


/**************************************************************************//**
 * @brief  LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{

    LDMA_IntClear(LDMA_IF_DONE0);

    ping_pong.dma_buffer = toggleBuffer(ping_pong.dma_buffer);

    blueToothNotif = true;

    // PERHAPS ENQUEUE THE RESULT TO THE BLUETOOTH QUEUE before setting bluetoothnotif true

  // Toggle LED0 to notify that transfers are complete
  GPIO_PinOutToggle(LDMA_OUTPUT_0_PORT, LDMA_OUTPUT_0_PIN);
}



// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Application Init.
void app_init(void)
{
  // this is called once during start up
  // Initialize GPIO
    initGPIO();

    // Initialize PRS
    initPRS();

    // Initialize the IADC
    initIADC();

    ping_pong.dma_buffer = ping_pong.buffer_A; // we'll start with buffer A
    ping_pong.send_buffer = ping_pong.buffer_A; // we'll have BLE start from sending buffer A as well, though it won't activate until the notif flag has been set

    // Initialize LDMA
    initLDMA(ping_pong.buffer_A, ping_pong.buffer_B, NUM_SAMPLES);

    // Initialize LFXO
    initClock();

    // Initialize the LETIMER
    initLetimer();

    // Initialize our BLE Packet history!
    circular_buffer_init(BLE_PACKET_QUEUE_SIZE);

  #ifdef EM2DEBUG
  #if (EM2DEBUG == 1)
    // Enable debug connectivity in EM2
    EMU->CTRL_SET = EMU_CTRL_EM2DBGEN;
  #endif
  #endif

}


static uint32_t packet_id = 0;  // Global packet counter

sl_status_t sendPacket() {
// #define gattdb_iadc_result                    27
  sl_status_t sc;

  ping_pong.send_buffer[NUM_SAMPLES] = packet_id;

  // Little note on casting the active buffer pointer to uint8_t
  // As uint32_t*: reads 4 bytes at a time as integers
  // As uint8_t*: reads 1 byte at a time as bytes
  // Same memory, different interpretation!
  // const means "i won't modify the DATA the pointer points to, i promise to only read it as a stream of data for bluetooth packet sending"
  // const is a promise to read only and not modify made by the program
  sc = sl_bt_gatt_server_notify_all(gattdb_iadc_result, gattdb_iadc_result_len, (const uint8_t*) ping_pong.send_buffer); // PERHAPS EMPTY OUT QUEUE OR SOMETHING INSTEAD...

  if (sc == SL_STATUS_OK) {
      packet_id++; //naturally wraps around from FFFFFFFF to 0
  }

  circular_buffer_enqueue(ping_pong.send_buffer); // purely for watching the past packets that have been sent

  ping_pong.send_buffer = toggleBuffer(ping_pong.send_buffer); // after sending, move to the next buffer to send


  if (sc != SL_STATUS_OK) { return sc; }

  return sc;
}

// Application Process Action.
void app_process_action(void)
{
  if (app_is_process_required()) {

  }

  if (blueToothNotif) {
      sendPacket();
      blueToothNotif = false;
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the default weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      //printf("Device connected!\n"); // start sampling after connection...need timestamps and packet IDs
      //LETIMER_CounterSet(LETIMER0, LETIMER_CompareGet(LETIMER0, 0));  // Reset to top
      //LETIMER_Enable(LETIMER0, true);
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      //LETIMER_Enable(LETIMER0, false);
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable); // stop sampling after connection
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
