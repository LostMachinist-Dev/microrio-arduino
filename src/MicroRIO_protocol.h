// @generated from spec/protocol.toml by `cargo xtask gen-header` - do not edit.
//
// Wire format shared with the Rust driver station. Little-endian, no padding.
//
// Header layout (12 bytes):
//   uint16 magic | uint8 version | uint8 msg_type
//   uint16 node_id | uint32 schema_hash | uint16 seq

#ifndef MICRORIO_PROTOCOL_H
#define MICRORIO_PROTOCOL_H

#include <stdint.h>

// Magic bytes "MR".
#define MICRORIO_MAGIC 0x4D52u
// Rejected on mismatch, never negotiated.
#define MICRORIO_PROTO_VERSION 2u
#define MICRORIO_HEADER_LEN 12u

// Limits and defaults.
#define MICRORIO_STR_LEN 32u
#define MICRORIO_MAX_NAME_LEN 24u
#define MICRORIO_DEFAULT_MAX_FRAME 1400u
#define MICRORIO_MIN_MAX_FRAME 128u
#define MICRORIO_DEFAULT_RATE_HZ 50u
#define MICRORIO_DEFAULT_WATCHDOG_MS 200u
#define MICRORIO_DEFAULT_HISTORY 8u

// Fixed body widths before any payload. See spec/protocol.toml.
#define MICRORIO_CONTROL_FIXED 12u
#define MICRORIO_TELEM_FIXED 13u
#define MICRORIO_RECORD_FIXED 4u

// Message types.
#define MICRORIO_MSG_SCHEMA_REQ 1u
#define MICRORIO_MSG_SCHEMA 2u
#define MICRORIO_MSG_CONTROL 3u
#define MICRORIO_MSG_TELEM 4u

// Channel types. Bools are bitpacked, so their width is not a byte count.
#define MICRORIO_TYPE_BOOL 0u  // 0 bytes on the wire
#define MICRORIO_TYPE_I32 1u  // 4 bytes on the wire
#define MICRORIO_TYPE_F32 2u  // 4 bytes on the wire
#define MICRORIO_TYPE_STR 3u  // 32 bytes on the wire
#define MICRORIO_TYPE_CMD 4u  // 2 bytes on the wire

// Direction. Telemetry is displayed by the GUI; control is populated by it.
#define MICRORIO_DIR_TELEMETRY 0u
#define MICRORIO_DIR_CONTROL 1u

// Operating state. Outputs are live only in ENABLED.
#define MICRORIO_STATE_DISABLED 0u
#define MICRORIO_STATE_ENABLED 1u
#define MICRORIO_STATE_ESTOP 2u
#define MICRORIO_STATE_FAULT 3u

// Control flag bits, station to node.
#define MICRORIO_CFLAG_VIEW_ONLY 0x01u

// Telemetry flag bits.
#define MICRORIO_FLAG_WATCHDOG_TRIPPED 0x01u
#define MICRORIO_FLAG_NEVER_LINKED 0x02u
#define MICRORIO_FLAG_SCHEMA_DIRTY 0x04u
#define MICRORIO_FLAG_STATION_CONFLICT 0x08u

// FNV-1a 32-bit, over dir, type, slot and name of each channel in order.
#define MICRORIO_FNV_OFFSET 0x811C9DC5ul
#define MICRORIO_FNV_PRIME  0x01000193ul

#endif // MICRORIO_PROTOCOL_H
