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
#include "ring_buffer.h"
#include "compression.h"
#include "dwt.h"
#include "test_signal_data.h"
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
#define LETIMER_FREQ              5000

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
LDMA_Descriptor_t descriptor;

// buffer to store IADC samples

uint32_t scanBuffer[NUM_SAMPLES];

static rbd_t _rbd = 0;
static rbd_t sampleQidx = 0;
static SampleSlotType sampleQueue[SAMPLE_Q_SIZE];

//static rbd_t compressedQidx;
static bool compress_trigger;
static COMPRESSION_TYPE compressionTemp[COMPRESS_AT_A_TIME];
static uint8_t curIdx;

static uint8_t samples_lost = 0;


wave_object_t wave_pool[MAX_WAVE_OBJECTS];
int wave_pool_used[MAX_WAVE_OBJECTS] = {0};


wt_object_t wt_pool[MAX_WT_OBJECTS];
int wt_pool_used[MAX_WT_OBJECTS] = {0};

wave_object wave;
wt_object wave_transform;

uint8_t* outgoing_data_ptr;
uint32_t outgoing_total_bytes;
uint32_t outgoing_bytes_sent;
uint8_t  outgoing_packet_id;
bool     tx_in_progress = false;


volatile ble_transfer_state_t transfer_state = BLE_TRANSFER_IDLE;


static int test_idx = 0; // used to index into test_signal which is defined in test_signal_data.c


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
  letimerInit.enable = false;

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

void initLDMA(uint32_t *buffer, uint32_t size)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // Configure LDMA for transfer from IADC to memory
  // LDMA will loop continuously
  LDMA_TransferCfg_t transferCfg =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);

  // Set up descriptors for dual buffer transfer
  descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SCANFIFODATA, buffer, size, 0);

  // Loop of NUM_SAMPLES, run continuously
  descriptor.xfer.decLoopCnt = 0;
  descriptor.xfer.xferCnt = NUM_SAMPLES - 1; // 1 less than desired transfer count

  // Interrupt upon transfer complete
  descriptor.xfer.doneIfs = 1;
  descriptor.xfer.ignoreSrec = 0;

  // Initialize LDMA with default configuration
  LDMA_Init(&init);

  // Start transfer, LDMA will sample the IADC NUM_SAMPLES time, and then interrupt
  LDMA_StartTransfer(IADC_LDMA_CH, &transferCfg, &descriptor);
}

volatile bool blueToothNotif = false;

//volatile uint32_t irq_times[1000];
//volatile uint32_t irq_idx = 0;


/**************************************************************************//**
 * @brief  LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{

    LDMA_IntClear(LDMA_IF_DONE0);
//    uint32_t now = LETIMER_CounterGet(LETIMER0);
//    if (irq_idx < 1000) irq_times[irq_idx++] = now;

    SAMPLE_TYPE iadcResults[NUM_SAMPLES];

    for (uint32_t i = 0; i < NUM_SAMPLES; i++) {
        //iadcResults[i] = (SAMPLE_TYPE)(scanBuffer[i] & 0xFFF); // mask the bottom 12 bits for the right iadc value

        iadcResults[i] = test_signal[test_idx]; // COMMENT THIS OUT
        test_idx = (test_idx + 1) % 1000;        // AND UNCOMMENT SCANBUFFER CODE FOR NORMAL OPERATION
    }
    int err = ring_buffer_put(sampleQidx, iadcResults); // throw it into the sample queue!

    if (err) {
      samples_lost++; // count if we lose any samples due to buffer being full
    }

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


    // Initialize LDMA
    initLDMA(scanBuffer, NUM_SAMPLES);

    // Initialize LFXO
    initClock();

    // Initialize the LETIMER
    initLetimer();

    // Initialize the buffers.
    rb_attr_t attr1 = {
        .s_elem = NUM_SAMPLES * sizeof(SAMPLE_TYPE), // i want to make one slot of sampleQueue, NUMSAMPLES amount of SAMPLETYPE, or 60 uint32_t.
        .n_elem = SAMPLE_Q_SIZE,
        .buffer = sampleQueue,
    };
    sampleQidx = 0;
    ring_buffer_init(&_rbd, &attr1); // make sampleQueue






  #ifdef EM2DEBUG
  #if (EM2DEBUG == 1)
    // Enable debug connectivity in EM2
    EMU->CTRL_SET = EMU_CTRL_EM2DBGEN;
  #endif
  #endif

}

/**** BLUETOOTH PROTOCOL CODE ****/

typedef struct __attribute__((packed)) {
    COEFFICIENT_TYPE quant;
    // int original_signal_length; // it's possible to use something smaller than int to store the original signal length.
                                // What is the original signal length? It's the wave_transform->outlength, which represents the length of the sparse representation coefficients, whose calculation can be found in wavedec.c
                                // Is type must be large enough to represent the max output length. For example, for a signal chunk of 6000, we expect
    uint8_t num_levels;
    int book_keeping[NUM_LEVELS]; // it's better to send the book keeping vector for python to reconstruct with!
} StartHeader; // size is NUM_LEVELS amount of int + one float.

typedef struct __attribute__((packed)) {
    uint8_t packet_id;
    uint8_t flags;
} PacketHeader;

#define BUFFER_MEMBER_SIZE sizeof(CodewordEntry) // it's 8 bytes
#define PACKET_HEADER_SIZE sizeof(PacketHeader) // 3 bytes
#define MAX_BYTES_PER_NOTIFICATION (gattdb_iadc_result_len - 1 - PACKET_HEADER_SIZE)

#define PACKET_TYPE_START 0x01
#define PACKET_TYPE_DATA  0x02


void start_ble_transfer(uint8_t* data, uint32_t total_bytes, COEFFICIENT_TYPE* quant, int* book_keeping)
{
    if (tx_in_progress) return;
    outgoing_data_ptr = data;
    outgoing_total_bytes = total_bytes;
    outgoing_bytes_sent = 0;
    outgoing_packet_id = 0;
    tx_in_progress = true;
    transfer_state = BLE_TRANSFER_SENDING_HEADER;
    send_start_packet(quant, book_keeping);
}
void send_start_packet(COEFFICIENT_TYPE* quant, int* book_keeping) {
  // fill up the start_header
    StartHeader start_header;
    start_header.quant = *quant;
    start_header.num_levels = NUM_LEVELS;
    for (int i = 0; i < NUM_LEVELS; i++) {
        start_header.book_keeping[i] = book_keeping[i];
    }

    // create the start_packet and ZERO IT OUT
    uint8_t start_packet[1 + sizeof(StartHeader)];
    memset(start_packet, 0, sizeof(start_packet));

    start_packet[0] = PACKET_TYPE_START; // first byte always tells what type of packet it is

    memcpy(start_packet + 1, &start_header, sizeof(StartHeader));

    // send it off
    volatile sl_status_t sc = sl_bt_gatt_server_notify_all(gattdb_iadc_result, sizeof(start_packet), start_packet);

    if (sc != SL_STATUS_OK) {
        // retry?? no clue
    } else {
        // what to say??
    }


}


/**
 * The purpose of the sendPacket() function is to greedily send as much data per packet
 * until all the compressed values have been relayed
 */

sl_status_t send_next_notification() {
    sl_status_t sc = SL_STATUS_OK;


    // Send the StartHeader once at the very start
    uint32_t remaining = outgoing_total_bytes - outgoing_bytes_sent;
    uint32_t cur_bytes_sent = (remaining > MAX_BYTES_PER_NOTIFICATION) ? MAX_BYTES_PER_NOTIFICATION : remaining;

    PacketHeader header;
    header.packet_id = outgoing_packet_id;
    header.flags = 0;
    if (outgoing_bytes_sent == 0) header.flags |= 0x01; // start
    if ((outgoing_bytes_sent + cur_bytes_sent) >= outgoing_total_bytes) header.flags |= 0x02; // end

    // make data packet and ZERO IT OUT
    uint8_t packet[1 + PACKET_HEADER_SIZE + cur_bytes_sent];

    memset(packet, 0, sizeof(packet));

    packet[0] = PACKET_TYPE_DATA;

    memcpy(packet + 1, &header, PACKET_HEADER_SIZE);
    memcpy(packet + PACKET_HEADER_SIZE, outgoing_data_ptr + outgoing_bytes_sent, cur_bytes_sent);

    sc = sl_bt_gatt_server_notify_all(gattdb_iadc_result, sizeof(packet), packet);
    if (sc == SL_STATUS_OK) {
         outgoing_bytes_sent += cur_bytes_sent;
         outgoing_packet_id++;
     } else {
         // could handle errors here
     }


  return sc;
}

// Application Process Action.
void app_process_action(void)
{
  if (app_is_process_required()) {

  }

//  if(blueToothNotif) {
//      blueToothNotif = false;
//      sendPacket();
//  }

  if ((ring_buffer_length(sampleQidx) >= COMPRESSION_THRESHOLD) && !tx_in_progress) {

          int i = 0;

          while ((i < COMPRESSION_THRESHOLD) && !(_ring_buffer_empty(&_rb[sampleQidx]))) {
              SampleSlotType sampleQElem;
              ring_buffer_get(sampleQidx, &sampleQElem);

              for (int j = 0; j < NUM_SAMPLES; j++) {
                  compressionTemp[curIdx*NUM_SAMPLES + j] = (COMPRESSION_TYPE) sampleQElem.samples[j];
              }

              i++;

              curIdx++;

          }
          //blueToothNotif = true;
          curIdx = 0;

          COEFFICIENT_TYPE quant;
          int num_nnz;
          CodewordEntry codeword_results[MAX_CODEWORDS];
          int compressed_signal_length;


          compress(wave, wave_transform, COMPRESSION_RATIO, compressionTemp, COMPRESS_AT_A_TIME, NUM_LEVELS,
                   NUM_CHANNELS, codeword_results, &num_nnz, &quant, &compressed_signal_length);

          start_ble_transfer((uint8_t*)codeword_results, (uint32_t) num_nnz*sizeof(COEFFICIENT_TYPE), &quant, wave_transform->length);



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

      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      LETIMER_Enable(LETIMER0, false);
      //wave_free(wave);
      //wt_free(wave_transform);

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

    case sl_bt_evt_gatt_server_characteristic_status_id:
         if (evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
             evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_notification) {
             wave = wave_init("db4");
             wave_transform = wt_init(wave, "dwt", COMPRESS_AT_A_TIME, NUM_LEVELS);
             LETIMER_CounterSet(LETIMER0, LETIMER_CompareGet(LETIMER0, 0));  // Reset to top
             LETIMER_Enable(LETIMER0, true);
         }
         break;

    case sl_bt_evt_gatt_server_notification_tx_completed_id:
        if (transfer_state == BLE_TRANSFER_SENDING_HEADER) {
            transfer_state = BLE_TRANSFER_SENDING_DATA;
            send_next_notification(); // begin normal data transfer
        } else if (transfer_state == BLE_TRANSFER_SENDING_DATA) {
            if (outgoing_bytes_sent < outgoing_total_bytes) {
                send_next_notification();
            } else {
                transfer_state = BLE_TRANSFER_IDLE;
                tx_in_progress = false;
            }
        }

        break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
