# VESC CAN Protocol: Retrieving Internal values

This document outlines how to retrieve internal values like `fault_code`, `id`, and `iq` from a VESC motor controller over a CAN bus using a custom microcontroller.

## 1. The Core Limitation
VESCs **do not** broadcast `fault_code`, `id`, or `iq` automatically in their default periodic CAN messages (such as `CAN_PACKET_STATUS`).

To get these values, your external microcontroller must actively **poll** (request) the data from the target VESC by sending specific "COMM" commands encapsulated inside CAN frames.

---

## 2. Constructing the CAN ID
The VESC uses a Custom 29-bit Extended CAN ID to encode both the "packet type" (command) and the "destination address" (node ID).

The format used in C is: 
```c
(8 << 8) | Target_VESC_ID
```

### What does this structure mean?
This structure is a common bitwise operation used to pack multiple pieces of data into a single identifier. 

1. **`8` (The Command):** This relates to `CAN_PACKET_PROCESS_SHORT_BUFFER`. It tells the VESC to treat the CAN payload as a standard USB/UART VESC command. Binary: `0000 0000 0000 1000`.
2. **`<< 8` (Bitwise Left Shift):** This shifts the `8` to the left by 8 binary places. It moves the command identifier into the upper bits, effectively making room at the bottom for the target ID.
   * Shifted binary: `0000 1000 0000 0000`
3. **`Target_VESC_ID`:** This is the 1-byte node ID of the motor controller you want to address (e.g., node `43`, or `0010 1011` in binary).
4. **`|` (Bitwise OR):** This merges the shifted command with the Target ID, snapping the 8-bit Target ID directly into the bottom 8 zeroed-out slots created by the shift.
   * Merged binary: `0000 1000 0010 1011`

By doing this, the receiving VESC can easily reverse the process (using `& 0xFF` for the ID and `>> 8` for the command) to unpack the message header.

---

## 3. Two Methods for Polling Data

### Method A: Request Everything (`COMM_GET_VALUES`)
This is the simplest method, but it utilizes the most CAN bandwidth because it returns the entire monolithic `mc_values` structure.

* **What to send in the CAN Frame:** 
  * **Extended CAN ID:** `(8 << 8) | Target_VESC_ID`
  * **Payload (3 bytes):** `[Sender_ID, Routing_Flag, COMM_Command]`
    * Byte 0: `0x00` (Your local Node ID, e.g., 0)
    * Byte 1: `0x00` (Routing Flag: 0 = Process & Reply)
    * Byte 2: `0x04` (The hex value for `COMM_GET_VALUES`)

### Method B: Request Specific Data (`COMM_GET_VALUES_SELECTIVE`)
This is the optimized method. You send a 32-bit mask specifying exactly which fields you want, and the VESC replies *only* with those data points.

If you specifically want `id`, `iq`, and `fault_code`:
* `id` is Bit 4 (`0x00000010`)
* `iq` is Bit 5 (`0x00000020`)
* `fault_code` is Bit 15 (`0x00008000`)
* **Combined Mask:** `0x00008030`

* **What to send in the CAN Frame:**
  * **Extended CAN ID:** `(8 << 8) | Target_VESC_ID`
  * **Payload (7 bytes total):** `[Sender_ID, Routing_Flag, COMM_Command, Mask(4 bytes)]`
    * Byte 0: `0x00` (Your local Node ID, e.g., 0)
    * Byte 1: `0x00` (Routing Flag: 0 = Process & Reply)
    * Byte 2: `0x32` (`COMM_GET_VALUES_SELECTIVE` command)
    * Bytes 3-6: The 32-bit mask `[0x00, 0x00, 0x80, 0x30]`

---

## 4. Processing the Reply
Because these replies are often larger than the 8-byte payload limit of standard CAN frames, the VESC chunks its response into multiple CAN frames using the packet ID `CAN_PACKET_FILL_RX_BUFFER` (ID `5`).

Your receiving microcontroller must:
1. Listen on the bus for messages with the ID: `(5 << 8) | Your_Local_Node_ID`
2. Stitch the bytes from those multiple frames together into an array.
3. Parse the data types (floats, integers, bytes) out of the array according to the standard VESC structural formatting.