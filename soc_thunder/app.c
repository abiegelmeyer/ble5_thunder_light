/***************************************************************************//**
 * @file
 * @brief Application code
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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

#include <stdbool.h>
#include <stdio.h>
#include "em_common.h"
#include "sl_status.h"

#include "sl_btmesh.h"
#include "sl_bluetooth.h"
#include "app.h"

#include "gatt_db.h"

#include "app_assert.h"

/* Buttons and LEDs headers */
//#include "app_button_press.h"
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"
//#include "sl_simple_led.h"
//#include "sl_simple_led_instances.h"

#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT

#ifdef SL_CATALOG_APP_LOG_PRESENT
#include "app_log.h"
#endif // SL_CATALOG_APP_LOG_PRESENT

#ifdef SL_CATALOG_BTMESH_WSTK_LCD_PRESENT
#include "sl_btmesh_wstk_lcd.h"
#endif // SL_CATALOG_BTMESH_WSTK_LCD_PRESENT

/* Switch app headers */
#include "sl_simple_timer.h"
#include "sl_btmesh_factory_reset.h"
//#include "sl_btmesh_lighting_client.h"
//#include "sl_btmesh_ctl_client.h"
//#include "sl_btmesh_scene_client.h"
//#include "sl_btmesh_provisioning_decorator.h"


#include "my_model_def.h"

my_model_t my_model = {
    .elem_index = PRIMARY_ELEMENT,
    .vendor_id = MY_VENDOR_ID,
    .model_id = MY_MODEL_SERVER_ID,
    .publish = 1,
    .opcodes_len = NUMBER_OF_OPCODES,
    .opcodes_data[0] = temperature_get,
    .opcodes_data[1] = temperature_status,
    .opcodes_data[2] = unit_get,
    .opcodes_data[3] = unit_set,
    .opcodes_data[4] = unit_set_unack,
    .opcodes_data[5] = unit_status,
    .opcodes_data[6] = update_interval_get,
    .opcodes_data[7] = update_interval_set,
    .opcodes_data[8] = update_interval_set_unack,
    .opcodes_data[9] = update_interval_status };

#ifdef PROV_LOCALLY
static uint16_t uni_addr = 0;

static aes_key_128 enc_key = {
  .data = "\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03\x03"
};
#endif /* #ifdef PROV_LOCALLY */

#ifdef PROV_LOCALLY
#include "self_test.h"
/* The default settings of the network and the node */
#define NET_KEY_IDX                 0
#define APP_KEY_IDX                 0
#define IVI                         0
#define DEFAULT_TTL                 5
/* #define ELEMENT_ID                  0 */
#endif /* #ifdef PROV_LOCALLY */



/// High Priority
#define HIGH_PRIORITY                  0
/// No Timer Options
#define NO_FLAGS                       0
/// Callback has no parameters
#define NO_CALLBACK_DATA               (void *)NULL
/// Timeout for Blinking LED during provisioning
#define APP_LED_BLINKING_TIMEOUT       250
/// Connection uninitialized
#define UNINITIALIZED_CONNECTION       0xFF
/// Advertising Provisioning Bearer
#define PB_ADV                         0x1
/// GATT Provisioning Bearer
#define PB_GATT                        0x2
/// Increase step of physical values (lightness, color temperature)
#define INCREASE                       10
/// Decrease step of physical values (lightness, color temperature)
#define DECREASE                       (-10)
/// Length of the display name buffer
#define NAME_BUF_LEN                   20
/// Length of boot error message buffer
#define BOOT_ERR_MSG_BUF_LEN           30
/// Used button index
#define BUTTON_PRESS_BUTTON_0          0

#ifndef SL_CATALOG_APP_LOG_PRESENT
#define app_log(...)
#endif // SL_CATALOG_APP_LOG_PRESENT

#ifdef SL_CATALOG_BTMESH_WSTK_LCD_PRESENT
#define lcd_print(...) sl_btmesh_LCD_write(__VA_ARGS__)
#else
#define lcd_print(...)
#endif // SL_CATALOG_BTMESH_WSTK_LCD_PRESENT

/// periodic timer handle
static sl_simple_timer_t app_led_blinking_timer;

/// periodic timer callback
static void app_led_blinking_timer_cb(sl_simple_timer_t *handle, void *data);
// Handling of boot event
static void handle_boot_event(void);
// Handling of le connection events
static void handle_le_connection_events(sl_bt_msg_t *evt);
// Set device name in the GATT database
static void set_device_name(bd_addr *addr);

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/// number of active Bluetooth connections
static uint8_t num_connections = 0;

static bool init_done = false;

/***************************************************************************//**
 * Change buttons to LEDs in case of shared pin
 *
 ******************************************************************************/
void change_buttons_to_leds(void)
{
  //app_button_press_disable();
  // Disable button and enable led
  //sl_simple_button_disable(&sl_button_btn0);
  //sl_simple_led_init(sl_led_led0.context);
  // Disable button and enable led
#ifndef SINGLE_BUTTON
  //sl_simple_button_disable(&sl_button_btn1);
#endif // SINGLE_BUTTON
#ifndef SINGLE_LED
  //sl_simple_led_init(sl_led_led1.context);
#endif // SINGLE_LED
}

/***************************************************************************//**
 * Change LEDs to buttons in case of shared pin
 *
 ******************************************************************************/
void change_leds_to_buttons(void)
{
  // Enable buttons
  sl_simple_button_enable(&sl_button_btn0);
#ifndef SINGLE_BUTTON
  sl_simple_button_enable(&sl_button_btn1);
#endif // SINGLE_BUTTON
  // Wait
  sl_sleeptimer_delay_millisecond(1);
  // Enable button presses
  //app_button_press_enable();
}

/***************************************************************************//**
 * Application Init.
 ******************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  app_log("BT mesh Switch initialized\r\n");
  // Ensure right init order in case of shared pin for enabling buttons
  change_buttons_to_leds();
  // Change LEDs to buttons in case of shared pin
  change_leds_to_buttons();



}

/***************************************************************************//**
 * Application Process Action.
 ******************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/***************************************************************************//**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] addr  Pointer to Bluetooth address.
 ******************************************************************************/
static void set_device_name(bd_addr *addr)
{
  char name[NAME_BUF_LEN];
  sl_status_t result;

  // Create unique device name using the last two bytes of the Bluetooth address
  snprintf(name, NAME_BUF_LEN, "switch node %02x:%02x",
           addr->addr[1], addr->addr[0]);



  uni_addr = ((addr->addr[1] << 8) | addr->addr[0]) & 0x7FFF;



  app_log("Device name: '%s'\r\n", name);

  result = sl_bt_gatt_server_write_attribute_value(gattdb_device_name,
                                                   0,
                                                   strlen(name),
                                                   (uint8_t *)name);
  if (result) {
    app_log("sl_bt_gatt_server_write_attribute_value() failed, code %lx\r\n",
            result);
  }

  // Show device name on the LCD
  lcd_print(name, SL_BTMESH_WSTK_LCD_ROW_NAME_CFG_VAL);
}

/***************************************************************************//**
 * Handles button press and does a factory reset
 *
 * @return true if there is no button press
 ******************************************************************************/
bool handle_reset_conditions(void)
{
  // If PB0 is held down then do full factory reset
  if (sl_simple_button_get_state(&sl_button_btn0)
      == SL_SIMPLE_BUTTON_PRESSED) {
    // Disable button presses
    //app_button_press_disable();
    // Full factory reset
    sl_btmesh_initiate_full_reset();
    return false;
  }

#ifndef SINGLE_BUTTON
  // If PB1 is held down then do node factory reset
  if (sl_simple_button_get_state(&sl_button_btn1)
      == SL_SIMPLE_BUTTON_PRESSED) {
    // Disable button presses
    //app_button_press_disable();
    // Node factory reset
    sl_btmesh_initiate_node_reset();
    return false;
  }
#endif // SL_CATALOG_BTN1_PRESENT
  return true;
}

/***************************************************************************//**
 * Handling of boot event.
 * If needed it performs factory reset. In other case it sets device name
 * and initialize mesh node.
 ******************************************************************************/
static void handle_boot_event(void)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  char buf[BOOT_ERR_MSG_BUF_LEN];
  // Check reset conditions and continue if not reset.
  if (handle_reset_conditions()) {
    sc = sl_bt_system_get_identity_address(&address, &address_type);
    app_assert_status_f(sc, "Failed to get Bluetooth address\n");
    set_device_name(&address);
    // Initialize Mesh stack in Node operation mode, wait for initialized event
    sc = sl_btmesh_node_init();
    if (sc) {
      snprintf(buf, BOOT_ERR_MSG_BUF_LEN, "init failed (0x%lx)", sc);
      lcd_print(buf, SL_BTMESH_WSTK_LCD_ROW_STATUS_CFG_VAL);
    }
  }
}

/***************************************************************************//**
 *  Handling of le connection events.
 *  It handles:
 *   - le_connection_opened
 *   - le_connection_parameters
 *   - le_connection_closed
 *
 *  @param[in] evt  Pointer to incoming connection event.
 ******************************************************************************/
static void handle_le_connection_events(sl_bt_msg_t *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_connection_opened_id:
      num_connections++;
      lcd_print("connected", SL_BTMESH_WSTK_LCD_ROW_CONNECTION_CFG_VAL);
      app_log("Connected\r\n");
      break;

    case sl_bt_evt_connection_closed_id:
      if (num_connections > 0) {
        if (--num_connections == 0) {
          lcd_print("", SL_BTMESH_WSTK_LCD_ROW_CONNECTION_CFG_VAL);
          app_log("Disconnected\r\n");
        }
      }
      break;

    default:
      break;
  }
}

/***************************************************************************//**
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt    Pointer to incoming event.
 ******************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      handle_boot_event();
      break;

    case sl_bt_evt_connection_opened_id:
    case sl_bt_evt_connection_parameters_id:
    case sl_bt_evt_connection_closed_id:
      handle_le_connection_events(evt);
      break;

    default:
      break;
  }
}

/***************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Pointer to incoming event from the Bluetooth Mesh stack.
 ******************************************************************************/
uint32_t contarx=0;
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_btmesh_evt_node_initialized_id:



      sc = sl_btmesh_vendor_model_init(my_model.elem_index,
                                        my_model.vendor_id,
                                        my_model.model_id,
                                        my_model.publish,
                                        my_model.opcodes_len,
                                        my_model.opcodes_data);

      app_log("vendor model init: %u\r\n", sc);


      /* Provision itself if not provisioned yet */
      if (evt->data.evt_node_initialized.provisioned) {

          app_log("Node provisioned already\r\n");

      } else {
        app_log("Node unprovisioned\r\n");
      }

      if (!(evt->data.evt_node_initialized.provisioned)) {

#ifdef PROV_LOCALLY
        /* Derive the unicast address from the LSB 2 bytes from the BD_ADDR */


          app_log("Unicast Address = 0x%04X\n", uni_addr);
          app_log("Provisioning itself.\n");

        sc = sl_btmesh_node_set_provisioning_data(enc_key,
                                                          enc_key,
                                                          NET_KEY_IDX,
                                                          IVI,
                                                          uni_addr,
                                                          0);


        for(uint32_t cont=100000; cont; cont--);  /* delay */
        break;
#else
        // Enable ADV and GATT provisioning bearer
        app_log("Sending unprovisioned device beacon\r\n");
        sc = sl_btmesh_node_start_unprov_beaconing(PB_ADV | PB_GATT);

        app_assert_status_f(sc, "Failed to start unprovisioned beaconing\n");



#endif /* #ifdef PROV_LOCALLY */

      }



#ifdef PROV_LOCALLY
      uint16_t appkey_index;
      uint16_t pub_address;
      uint8_t ttl;
      uint8_t period;
      uint8_t retrans;
      uint8_t credentials;
      /* Set the publication and subscription */
      sc =  sl_btmesh_test_get_local_model_pub(my_model.elem_index,
                                                  my_model.vendor_id,
                                                  my_model.model_id,
                                                  &appkey_index,
                                                  &pub_address,
                                                  &ttl,
                                                  &period,
                                                  &retrans,
                                                  &credentials);


      //if (!sc && pub_address == SERVER_PUB_ADDR) {
      if (!sc && pub_address == SERVER_SUB_ADDR) {
          app_log("Configuration done already.\n");
      } else {
          app_log("Pub setting result = 0x%04X, pub setting address = 0x%04X\n",
             sc,
             pub_address);
          app_log("Add local app key ...\n");

          sc = sl_btmesh_test_add_local_key(1,
                                       enc_key,
                                       APP_KEY_IDX,
                                       NET_KEY_IDX);
        app_log("Bind local app key ...\n");

        sc = sl_btmesh_test_bind_local_model_app(my_model.elem_index,
                                                         APP_KEY_IDX,
                                                         my_model.vendor_id,
                                                         my_model.model_id);
        app_log("Set local model pub ...\n");


        /* ----- SERVER ----- */
        sc = sl_btmesh_test_set_local_model_pub(my_model.elem_index,
                                                        APP_KEY_IDX,
                                                        my_model.vendor_id,
                                                        my_model.model_id,
                                                        SERVER_SUB_ADDR,
                                                        DEFAULT_TTL,
                                                        0, 0, 0);




        app_log("Add local model sub ...\n");
        sc = sl_btmesh_test_add_local_model_sub(my_model.elem_index,
                                                        my_model.vendor_id,
                                                        my_model.model_id,
                                                        SERVER_PUB_ADDR);

        app_log("Add local model sub ...: %u\r\n", sc);

        app_log("Set relay ...\n");
        sc = sl_btmesh_test_set_relay(1, 0, 0);
        app_log("Set Network tx state.\n");
        sc = sl_btmesh_test_set_nettx(2, 4);
        /* LOGD("addlocal model sub ...\n"); */
        /* SE_CALL(gecko_cmd_mesh_test_add_local_model_sub(0, my_model.vendor_id, MODEL_ID, DUT_SUB_ADDR)); */
      }
#endif /* #ifdef PROV_LOCALLY */




      break;

    case sl_btmesh_evt_vendor_model_receive_id:
      contarx++;
      app_log("Vendor Model RECEBIDO!!! ...: %u\r\n", contarx);

      if(evt->data.evt_vendor_model_receive.opcode == temperature_status)
        {
          app_log("Data[0]= : %u\r\n", evt->data.evt_vendor_model_receive.payload.data[0]);
          app_log("Data[1]= : %u\r\n", evt->data.evt_vendor_model_receive.payload.data[1]);
          app_log("Data[2]= : %u\r\n", evt->data.evt_vendor_model_receive.payload.data[2]);
          app_log("Data[3]= : %u\r\n", evt->data.evt_vendor_model_receive.payload.data[3]);

        }

      break;


    default:
      break;
  }
}

/*******************************************************************************
 * Callbacks
 ******************************************************************************/

/***************************************************************************//**
 * Button press Callbacks
 ******************************************************************************/
#include "sl_btmesh_model_specification_defs.h"
#include "sl_btmesh_generic_model_capi_types.h"
#include "sl_btmesh_dcd.h"
#include "sl_btmesh_lib.h"
uint8_t bufsend[8];
void app_button_press_cb(uint8_t button, uint8_t duration)
{

  struct mesh_generic_request req;
    static sl_status_t sc;
    static uint8_t kindtest=1;
    uint16_t  countmsg;

    const uint8_t *msg_buf;

  // Selecting action by duration
  switch (duration) {
    case 0:
      // Handling of button press less than 0.25s
      if (button == BUTTON_PRESS_BUTTON_0) {
        //sl_btmesh_change_lightness(DECREASE);



          countmsg++;

          bufsend[0] = 0x11;
          bufsend[1] = 0x22;
          bufsend[2] = 0x33;
          bufsend[3] = 0x44;
          bufsend[4] = 4;
          bufsend[5] = 5;



          sl_status_t ret=0;



          /*ret = sl_btmesh_vendor_model_init(my_model.elem_index,
                                            my_model.vendor_id,
                                            my_model.model_id,
                                            my_model.publish,
                                            my_model.opcodes_len,
                                            my_model.opcodes_data);

          app_log("vendor model init: %u\r\n", ret);*/

          /* biegelmeyer */
          uint8_t opcode = 0, length = 0, *data = NULL;
          opcode = temperature_status;
          length = TEMP_DATA_LENGTH;
          data = bufsend;



          sc = sl_btmesh_vendor_model_set_publication(my_model.elem_index,
                                                      my_model.vendor_id,
                                                      my_model.model_id,
                                                      opcode, 1,
                                                      length,
                                                      data);
          if (sc)
            {
              app_log("Set publication error = 0x%04X\r\n", sc);

            }
          else
            {
              app_log("Set publication done. Publishing...\r\n");
              sc = sl_btmesh_vendor_model_publish(my_model.elem_index, my_model.vendor_id, my_model.model_id);
              if (sc)
                {
                  app_log("Publish error = 0x%04X\r\n", sc);
                }
              else
                {
                  app_log("Publish done.\r\n");
                }
            }





          //log("\n Enviado Property!! %u\r\n\n", sc);
          app_log("\n Enviado Property!! %u\r\n\n", sc);

          if (sc == SL_STATUS_OK)
            {
              sc = 2;
            }








      } else {
        sl_btmesh_change_lightness(INCREASE);
      }
      break;
//    case APP_BUTTON_PRESS_DURATION_MEDIUM:
//      // Handling of button press greater than 0.25s and less than 1s
//      if (button == BUTTON_PRESS_BUTTON_0) {
//        sl_btmesh_change_temperature(DECREASE);
//      } else {
//        sl_btmesh_change_temperature(INCREASE);
//      }
//      break;
//    case APP_BUTTON_PRESS_DURATION_LONG:
//      // Handling of button press greater than 1s and less than 5s
//#ifdef SINGLE_BUTTON
//      sl_btmesh_change_switch_position(SL_BTMESH_LIGHTING_CLIENT_TOGGLE);
//#else
//      if (button == BUTTON_PRESS_BUTTON_0) {
//        sl_btmesh_change_switch_position(SL_BTMESH_LIGHTING_CLIENT_OFF);
//      } else {
//        sl_btmesh_change_switch_position(SL_BTMESH_LIGHTING_CLIENT_ON);
//      }
//#endif
//      break;
//    case APP_BUTTON_PRESS_DURATION_VERYLONG:
//      // Handling of button press greater than 5s
//      if (button == BUTTON_PRESS_BUTTON_0) {
//        sl_btmesh_select_scene(1);
//      } else {
//        sl_btmesh_select_scene(2);
//      }
//      break;
    default:
      break;
  }
}

/*******************************************************************************
 * Provisioning Decorator Callbacks
 ******************************************************************************/
// Called when the Provisioning starts
void sl_btmesh_on_node_provisioning_started(uint16_t result)
{
  // Change buttons to LEDs in case of shared pin
  change_buttons_to_leds();

  sl_status_t sc = sl_simple_timer_start(&app_led_blinking_timer,
                                         APP_LED_BLINKING_TIMEOUT,
                                         app_led_blinking_timer_cb,
                                         NO_CALLBACK_DATA,
                                         true);

  app_assert_status_f(sc, "Failed to start periodic timer\n");

#if defined(SL_CATALOG_BTMESH_WSTK_LCD_PRESENT) || defined(SL_CATALOG_APP_LOG_PRESENT)
  app_show_btmesh_node_provisioning_started(result);
#else
  (void)result;
#endif // defined(SL_CATALOG_BTMESH_WSTK_LCD_PRESENT) || defined(SL_CATALOG_APP_LOG_PRESENT)
}

// Called when the Provisioning finishes successfully
void sl_btmesh_on_node_provisioned(uint16_t address,
                                   uint32_t iv_index)
{
  sl_status_t sc = sl_simple_timer_stop(&app_led_blinking_timer);
  app_assert_status_f(sc, "Failed to stop periodic timer\n");
  // Turn off LED
  init_done = true;
 // sl_simple_led_turn_off(sl_led_led0.context);
#ifndef SINGLE_LED
  //sl_simple_led_turn_off(sl_led_led1.context);
#endif // SINGLE_LED
  change_leds_to_buttons();

#if defined(SL_CATALOG_BTMESH_WSTK_LCD_PRESENT) || defined(SL_CATALOG_APP_LOG_PRESENT)
  app_show_btmesh_node_provisioned(address, iv_index);
#else
  (void)address;
  (void)iv_index;
#endif // defined(SL_CATALOG_BTMESH_WSTK_LCD_PRESENT) || defined(SL_CATALOG_APP_LOG_PRESENT)
}

/***************************************************************************//**
 * Timer Callbacks
 ******************************************************************************/
static void app_led_blinking_timer_cb(sl_simple_timer_t *handle, void *data)
{
  (void)data;
  (void)handle;
  if (!init_done) {
    // Toggle LEDs
    //sl_simple_led_toggle(sl_led_led0.context);
#ifndef SINGLE_LED
    //sl_simple_led_toggle(sl_led_led1.context);
#endif // SINGLE_LED
  }
}
