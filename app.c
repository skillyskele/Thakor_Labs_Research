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
static SampleSlotType sampleQueue[SAMPLE_Q_SIZE]; // just a continuous array of uint32_t
static uint8_t sampleCount = 0;

//static rbd_t compressedQidx;
static bool compress_trigger;
//static COMPRESSION_TYPE compressedQueue[COMPRESSED_Q_SIZE];
static COMPRESSION_TYPE compressionTemp[COMPRESSED_BUFFER_SIZE];
static uint8_t curIdx;

static uint8_t samples_lost = 0;


wave_object_t wave_pool[MAX_WAVE_OBJECTS];
int wave_pool_used[MAX_WAVE_OBJECTS] = {0}; // Track usage


wt_object_t wt_pool[MAX_WT_OBJECTS];
int wt_pool_used[MAX_WT_OBJECTS] = {0}; // Track usage

wave_object wave;
wt_object wave_transform;

static SAMPLE_TYPE test_signal[160] = {
    2243, 3007, 2949, 3061, 3075, 2934, 2969, 2826, 2710, 2617, 2594, 1121,
    83, 4084, 4085, 4082, 4079, 4085, 4088, 4080, 4082, 4080, 4084, 4082,
    4075, 130, 74, 251, 264, 580, 814, 1058, 1169, 1375, 1413, 1150,
    1500, 1860, 1697, 1852, 2036, 1992, 2093, 2149, 2231, 2262, 2258, 2282,
    2347, 2323, 2318, 2279, 2247, 2430, 2340, 2269, 2356, 2388, 2374, 2494,
    2537, 2523, 2559, 2819, 2539, 2451, 2816, 2779, 2842, 2827, 2803, 2757,
    2748, 2817, 2761, 2874, 2882, 2826, 2915, 3014, 2960, 2949, 2967, 2975,
    2895, 2826, 2926, 2851, 2774, 2885, 2849, 2883, 2715, 2597, 2486, 2635,
    2676, 2651, 2311, 2493, 2429, 2510, 2528, 2464, 2526, 2429, 2536, 2431,
    2506, 2254, 2288, 2436, 2351, 2218, 2335, 2270, 2226, 2195, 2220, 2320,
    2268, 2195, 2231, 2058, 2071, 2175, 2171, 2063, 2078, 1996, 2050, 2075,
    2151, 2192, 2109, 2134, 2198, 2180, 2176, 2186, 2169, 2082, 1974, 1994,
    1911, 1964, 1992, 2124, 2153, 2038, 2208, 2145, 2108, 2229, 2209, 2122,
    2009, 2064, 2154, 2068
};
static int test_idx = 0;


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
        iadcResults[i] = test_signal[test_idx]; // TEST INPUT
        test_idx = (test_idx + 1) % 160;
    }
    int err = ring_buffer_put(sampleQidx, iadcResults); // put sizeof(iadcResults) bytes into sampleQueue[head]

    if (err) {
      samples_lost++;
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

typedef struct __attribute__((packed)) {
    COEFFICIENT_TYPE quant;
    int original_signal_length;
} StartHeader;

typedef struct __attribute__((packed)) {
    uint16_t packet_id;
    uint8_t flags;
} PacketHeader;

// we get 244 bytes per bluetooth packet
// #define PACKET_ID_SIZE sizeof(uint16_t) //34 bytes 16 * uint16_t, 1 uint16_t
#define BUFFER_MEMBER_SIZE sizeof(CodewordEntry) // it's 6 bytes
#define PACKET_HEADER_SIZE sizeof(PacketHeader) // 3 bytes
#define MAX_SAMPLES_PER_PAYLOAD ((gattdb_iadc_result_len - 1 - PACKET_HEADER_SIZE) / BUFFER_MEMBER_SIZE) // should be 31

#define PACKET_TYPE_START 0x01
#define PACKET_TYPE_DATA  0x02


sl_status_t sendPacket(CodewordEntry* codeword_results, int* num_nnz, COEFFICIENT_TYPE* quant, int* compressed_signal_length) {
  sl_status_t sc = SL_STATUS_OK;
  // find the number of samples that can fit
  int bufferIdx = 0;
  uint16_t local_packet_id = 0; // local ID for the current compressed chunk, its type must match max # elements in compressionTemp

  // Send the StartHeader once at the very start
  StartHeader start_header;
  start_header.quant = *quant;
  start_header.original_signal_length = *compressed_signal_length; // we're compressing COMPRESSED_BUFFER_SIZE at a time and sending it off

  uint8_t start_packet[1 + sizeof(StartHeader)];
  start_packet[0] = PACKET_TYPE_START;
  memcpy(start_packet + 1, &start_header, sizeof(StartHeader));
  sc = sl_bt_gatt_server_notify_all(gattdb_iadc_result, sizeof(start_packet), start_packet);
  if (sc != SL_STATUS_OK) {
      return sc;
  }



  while (bufferIdx < *num_nnz){ // 11 is the max.
    int samples_that_can_fit = *num_nnz - bufferIdx;
    int samples_used = (samples_that_can_fit > MAX_SAMPLES_PER_PAYLOAD) ? MAX_SAMPLES_PER_PAYLOAD : samples_that_can_fit;


    PacketHeader header;
    header.packet_id = local_packet_id++;
    header.flags = 0;
    if (bufferIdx == 0) header.flags |= 0x01; // start
    if ((bufferIdx + samples_used) >= *num_nnz) header.flags |= 0x02; // end


    uint8_t packet[1 + sizeof(PacketHeader) + samples_used * BUFFER_MEMBER_SIZE];
    packet[0] = PACKET_TYPE_DATA;

    memcpy(packet + 1, &header, sizeof(PacketHeader));

    memcpy(packet + 1 + sizeof(PacketHeader), &codeword_results[bufferIdx], samples_used * BUFFER_MEMBER_SIZE);


    sl_status_t sc = sl_bt_gatt_server_notify_all(gattdb_iadc_result, sizeof(packet), packet);
    if (sc != SL_STATUS_OK) {
        break;
    }


    bufferIdx += samples_used;
  }

  uint8_t packet[sizeof(int) + sizeof(COEFFICIENT_TYPE) + sizeof(CodewordEntry)];
  memcpy(packet, num_nnz, sizeof(int));
  memcpy(packet + sizeof(int), quant, sizeof(COEFFICIENT_TYPE));
  memcpy(packet + sizeof(int) + sizeof(COEFFICIENT_TYPE), codeword_results, sizeof(CodewordEntry));

  sc = sl_bt_gatt_server_notify_all(gattdb_iadc_result, sizeof(packet), packet);


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

  if (ring_buffer_length(sampleQidx) >= COMPRESSION_THRESHOLD) {

          int i = 0;

          while ((i < COMPRESSION_THRESHOLD) && !(_ring_buffer_empty(&_rb[sampleQidx]))) { // so N_COMPRESSION should define how many uint32 slots we want to grab out of sampleQidx
              SampleSlotType sampleQElem;
              ring_buffer_get(sampleQidx, &sampleQElem); // compressionTemp is of type uint32_t[], so it should realign each individual sample

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


          compress(wave, wave_transform, COMPRESSION_RATIO, compressionTemp, COMPRESSED_BUFFER_SIZE, NUM_LEVELS,
                   NUM_CHANNELS, codeword_results, &num_nnz, &quant, &compressed_signal_length);

          // chat should it be &compressionTemp[0] or what?
          sendPacket(codeword_results, &num_nnz, &quant, &compressed_signal_length); // might have to take in a NUM_CHANNELS parameter in the future but not for now!

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
      wave = wave_init("db4");
      wave_transform = wt_init(wave, "dwt", COMPRESSED_BUFFER_SIZE, NUM_LEVELS);
      LETIMER_CounterSet(LETIMER0, LETIMER_CompareGet(LETIMER0, 0));  // Reset to top
      LETIMER_Enable(LETIMER0, true);
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

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
