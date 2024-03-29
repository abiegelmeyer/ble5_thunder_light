/******************************************************************************
 *
 * @file   sl_btmesh_dcd.c
 * @brief  BLE Mesh Device Composition Data and memory configuration
 *
 *  Autogenerated file, do not edit
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
// *INDENT-OFF*

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "sl_btmesh_memory_config.h"
#include "sl_bluetooth_config.h"
#include "sl_btmesh_dcd.h"
#include "sl_btmesh_config.h"
#include "sl_btmesh.h"
#define U16TOA(A) ((A) & 0xFF), ((A) >> 8)

const uint8_t __mesh_dcd[] = {
  U16TOA(0x02ff), /* Company ID */
  U16TOA(0x0003), /* Product ID */
  U16TOA(0x0221), /* Version Number */
  U16TOA(SL_BTMESH_CONFIG_RPL_SIZE), /* Capacity of Replay Protection List */
  U16TOA(SL_BTMESH_FEATURE_BITMASK), /* Features Bitmask */
  /* Main */
    U16TOA(0x0000), /* Location */
    0x11, /* Number of SIG Models = 17 */
    0x01, /* Number of Vendor Models = 1 */
    /* SIG Models */
      U16TOA(0x0000), /* Configuration Server */
      U16TOA(0x0002), /* Health Server */
      U16TOA(0x1303), /* Light CTL Server */
      U16TOA(0x1304), /* Light CTL Setup Server */
      U16TOA(0x1000), /* Generic OnOff Server */
      U16TOA(0x1002), /* Generic Level Server */
      U16TOA(0x1004), /* Generic Default Transition Time Server */
      U16TOA(0x1006), /* Generic Power OnOff Server */
      U16TOA(0x1007), /* Generic Power OnOff Setup Server */
      U16TOA(0x1300), /* Light Lightness Server */
      U16TOA(0x1301), /* Light Lightness Setup Server */
      U16TOA(0x1203), /* Scene Server */
      U16TOA(0x1204), /* Scene Setup Server */
      U16TOA(0x1206), /* Scheduler Server */
      U16TOA(0x1207), /* Scheduler Setup Server */
      U16TOA(0x1200), /* Time Server */
      U16TOA(0x1201), /* Time Setup Server */
    /* Vendor Models */
      U16TOA(0x02ff), U16TOA(0xabcd), /* Vendor Model 0 */
  /* Temperature */
    U16TOA(0x0000), /* Location */
    0x02, /* Number of SIG Models = 2 */
    0x00, /* Number of Vendor Models = 0 */
    /* SIG Models */
      U16TOA(0x1002), /* Generic Level Server */
      U16TOA(0x1306), /* Light CTL Temperature Server */
  /* Light LC */
    U16TOA(0x0000), /* Location */
    0x03, /* Number of SIG Models = 3 */
    0x00, /* Number of Vendor Models = 0 */
    /* SIG Models */
      U16TOA(0x1000), /* Generic OnOff Server */
      U16TOA(0x130f), /* Light LC Server */
      U16TOA(0x1310), /* Light LC Setup Server */
};

const size_t __mesh_dcd_len = sizeof(__mesh_dcd);
const uint8_t *__mesh_dcd_ptr = __mesh_dcd;

const mesh_memory_config_t __mesh_memory_config = {
  .max_elements = SL_BTMESH_CONFIG_MAX_ELEMENTS,
  .max_models = SL_BTMESH_CONFIG_MAX_MODELS,
  .max_net_keys = SL_BTMESH_CONFIG_MAX_NETKEYS,
  .max_appkeys = SL_BTMESH_CONFIG_MAX_APPKEYS,
  .max_friendships = SL_BTMESH_CONFIG_MAX_FRIENDSHIPS,
  .max_app_binds = SL_BTMESH_CONFIG_MAX_APP_BINDS,
  .max_subscriptions = SL_BTMESH_CONFIG_MAX_SUBSCRIPTIONS,
  .max_foundation_model_commands = SL_BTMESH_CONFIG_MAX_FOUNDATION_CLIENT_CMDS,
  .net_cache_size = SL_BTMESH_CONFIG_NET_CACHE_SIZE,
  .replay_size = SL_BTMESH_CONFIG_RPL_SIZE,
  .max_send_segs = SL_BTMESH_CONFIG_MAX_SEND_SEGS,
  .max_recv_segs = SL_BTMESH_CONFIG_MAX_RECV_SEGS,
  .max_virtual_addresses = SL_BTMESH_CONFIG_MAX_VAS,
  .max_provision_sessions = SL_BTMESH_CONFIG_MAX_PROV_SESSIONS,
  .max_provision_bearers = SL_BTMESH_CONFIG_MAX_PROV_BEARERS,
  .max_gatt_connections = SL_BTMESH_CONFIG_MAX_GATT_CONNECTIONS,
  .gatt_txqueue_size = SL_BTMESH_CONFIG_GATT_TXQ_SIZE,
  .provisioner_max_ddb_entries = SL_BTMESH_CONFIG_MAX_PROVISIONED_DEVICES,
  .provisioner_max_node_net_keys = SL_BTMESH_CONFIG_MAX_PROVISIONED_DEVICE_NETKEYS,
  .provisioner_max_node_app_keys = SL_BTMESH_CONFIG_MAX_PROVISIONED_DEVICE_APPKEYS,
  .pstore_write_interval_elem_seq = (1 << (SL_BTMESH_CONFIG_SEQNUM_WRITE_INTERVAL_EXP)),
  .friend_max_total_cache = SL_BTMESH_CONFIG_FRIEND_MAX_TOTAL_CACHE,
  .friend_max_single_cache = SL_BTMESH_CONFIG_FRIEND_MAX_SINGLE_CACHE,
  .friend_max_subs_list = SL_BTMESH_CONFIG_FRIEND_MAX_SUBS_LIST,
  .app_send_max_queue = SL_BTMESH_CONFIG_APP_TXQ_SIZE,
  .proxy_max_access_control_list_entries = SL_BTMESH_CONFIG_MAX_PROXY_ACCESS_CONTROL_LIST_ENTRIES
};

#ifdef __cplusplus
}
#endif
// *INDENT-ON*
