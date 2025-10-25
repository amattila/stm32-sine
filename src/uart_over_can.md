# UART over CAN – Minimal Tunnel Spec & Reference Implementation

**Context**: STM32 (inverter side) ⟷ **CAN bus** (shared with other traffic) ⟷ ESP32 (display side, TWAI).  
Goal: Forward a UART byte stream over CAN **without** CANopen/SDO. The link must coexist with other CAN traffic safely and predictably.

---

## 1) Design (General)

### Goals
- **Transparent-enough** UART tunnel (binary safe).
- **Tiny footprint** (no dynamic alloc, minimal state).
- **Coexists** with heavy CAN traffic (dedicated IDs + HW filters).
- **Robust**: resync on losses/duplication, bounded buffers, backpressure optional.

### Link Topology
- **Two CAN IDs** (standard 11-bit):
  - **ESP32 → STM32**: `0x701`
  - **STM32 → ESP32**: `0x700`
- Reasoning: high numeric IDs give way to safety‑critical frames in CAN arbitration.

### Hardware Filtering
- STM32 accepts **only** `0x701`.
- ESP32/TWAI accepts **only** `0x700`.
- Everything else is dropped by hardware → minimal CPU overhead.

### Frame Format (Std CAN, DLC = 2..8)
```
byte0: SEQ           // 0..255, increments per sent frame (wrap allowed)
byte1: LEN_FLAGS     // b7=LAST (1=last chunk), b0..3=LEN (0..6 data bytes in this frame)
byte2..(1+LEN): DATA // up to 6 bytes (CAN max 8 bytes - 2 byte header = 6 bytes data)
```
- Split long UART payloads into chunks of ≤6 bytes.
- **LAST=1** marks the end of a reassembled message.
- **Resync rule**: If an unexpected `SEQ` arrives, **reset reassembly** and start from that frame.

> **Tip**: Treat every UART “write() call” as one logical message. If you stream indefinitely, insert periodic LAST=1 boundaries (e.g., after N bytes or idle timeout).

### Timing & Throughput
- Works with **Classical CAN** (8-byte frame, 6-byte data payload after 2-byte header). CAN-FD optional (64 B frame → max data=62 B).
- Typical sustained throughput on 500 kbit/s CAN with other traffic: **~18–35 kB/s** (6 bytes per frame vs 7).
- Respect a small **inter-frame gap** (1–2 ms) when the bus is busy to avoid starving others.

### Error Handling
- **Out-of-order/dup**: reassembly resets on SEQ mismatch.
- **Overflow**: if RX buffer would overflow, drop the partial message (reset state).
- **Optional ACK**: Not required; CAN already provides frame-level ACK. For application ACK/NACK, add a control message on the reverse ID (e.g., `LEN=0, LAST=1, FLAG_ACK`).

### UART Baud Rate
- 115200 8N1 works if bus load allows; 9600–57600 safer for noisy/busy systems.
- Backpressure: throttle UART reads when CAN TX queue is near full.

---

## 2) STM32 Side (part of **stm32-sine**)

### Summary
- Uses libopencm3 CAN hardware abstraction
- Implements CanCallback interface for message handling
- Buffers received UART data until complete message (LAST=1)
- Chunks outgoing UART data into CAN frames with SEQ + LEN_FLAGS format

### UART over CAN Class Interface
```cpp
class UartOverCan : public CanCallback
{
public:
    UartOverCan(CanHardware* can);
    void Init();
    void SendUartData(const uint8_t* data, uint32_t length);
    int GetUartData(uint8_t* buffer, uint32_t maxLength);

    // CanCallback interface
    bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
    void HandleClear();

private:
    CanHardware* m_can;
    uint8_t m_rxBuffer[512];
    uint32_t m_rxBufferPos;
    uint8_t m_expectedSeq;
};
```

### CAN Message Handling
```cpp
bool UartOverCan::HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc)
{
    if (canId == UART_CAN_RX_ID && dlc >= 2)
    {
        uint8_t* canData = (uint8_t*)data;
        uint8_t seq = canData[0];
        uint8_t lf = canData[1];
        uint8_t n = lf & 0x0F;
        bool last = (lf & 0x80) != 0;

        // Validate frame (max 6 bytes data + 2 bytes header = 8 bytes total)
        if (n > 6 || (2 + n) > dlc) {
            m_rxBufferPos = 0; // Reset on invalid frame
            return true;
        }

        // Resync logic: reset if unexpected sequence
        if (m_rxBufferPos == 0) m_expectedSeq = seq;
        if (seq != m_expectedSeq) {
            m_rxBufferPos = 0; // Resync
            m_expectedSeq = seq;
        }
        m_expectedSeq++;

        // Check buffer overflow
        if (m_rxBufferPos + n > sizeof(m_rxBuffer)) {
            m_rxBufferPos = 0; // Reset on overflow
            return true;
        }

        // Copy data
        memcpy(&m_rxBuffer[m_rxBufferPos], &canData[2], n);
        m_rxBufferPos += n;

        // If this is the last frame, message is complete
        if (last) {
            // Data is ready for reading via GetUartData()
        }
    }
    return true; // Continue processing other callbacks
}
```

### UART Data Transmission
```cpp
void UartOverCan::SendUartData(const uint8_t* data, uint32_t length)
{
    static uint8_t seq = 0;
    uint32_t sent = 0;

    while (sent < length)
    {
        uint8_t chunkSize = (length - sent) > 6 ? 6 : (uint8_t)(length - sent);
        bool isLast = (sent + chunkSize >= length);
        uint8_t canData[8] = {0};

        // SEQ + LEN_FLAGS format
        canData[0] = seq++;
        canData[1] = (chunkSize & 0x0F) | (isLast ? 0x80 : 0x00);
        memcpy(&canData[2], &data[sent], chunkSize);

        m_can->Send(UART_CAN_TX_ID, canData, 2 + chunkSize);
        sent += chunkSize;
    }
}
```

### Integration with stm32-sine
- Create `UartOverCan` instance with CAN hardware reference
- Call `Init()` after CAN setup
- In 100ms task, feed received UART data to terminal:
```cpp
if (terminal != NULL && uartOverCan != NULL) {
    uint8_t buffer[32];
    int received = uartOverCan->GetUartData(buffer, sizeof(buffer));
    for (int i = 0; i < received; i++) {
        terminal->PutChar(buffer[i]);
    }
}
```

---

## 3) ESP32 Side (ESP-IDF TWAI)

### Summary
- Configure TWAI @ same bitrate; filter **only** `0x700`.
- RX handler reconstructs frames and calls application callback. TX function chunks data to ≤6 bytes, sends to `0x701`.
- **No physical UART** - uses callback-based architecture for OpenInverter terminal protocol.

### TWAI Config (in can_link.c)
```c
// Hardware filter: accept only UART over CAN RX ID (0x700)
twai_filter_config_t f_config = {
    .acceptance_code = (0x700 << 21),  // 11-bit ID at MSBs (bits 28:18)
    .acceptance_mask = (0x7FF << 21),  // exact match (all 11 bits must match)
    .single_filter = true
};

twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
g_config.rx_queue_len = 30;
g_config.tx_queue_len = 30;
g_config.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF;

twai_driver_install(&g_config, &t_config, &f_config);
twai_start();
```

### RX Reassembly → Callback (uart_over_can.c)
```c
#define UART_CAN_RX_BUFFER_SIZE 512
#define UART_CAN_MAX_DATA_PER_FRAME 6  // CAN max 8 bytes - 2 byte header
static uint8_t s_rx_buffer[UART_CAN_RX_BUFFER_SIZE];
static size_t s_rx_pos = 0;
static uint8_t s_expected_seq = 0;
static uart_can_rx_callback_t s_rx_callback = NULL;

// CAN RX handler (called by can_link for each CAN message)
static void uart_can_rx_handler(uint32_t can_id, const uint8_t *data, uint8_t len) {
    // Only handle UART_CAN_RX_ID (0x700)
    if (can_id != 0x700 || len < 2) return;
    
    uint8_t seq = data[0];
    uint8_t len_flags = data[1];
    uint8_t payload_len = len_flags & 0x0F;
    bool is_last = (len_flags & 0x80) != 0;
    
    // Validate payload length (max 6 bytes data + 2 bytes header = 8 bytes)
    if (payload_len > UART_CAN_MAX_DATA_PER_FRAME || (2 + payload_len) > len) {
        s_rx_pos = 0;
        return;
    }
    
    // Resync logic: if buffer empty, accept any SEQ as start
    if (s_rx_pos == 0) {
        s_expected_seq = seq;
    }
    
    // Check SEQ - if mismatch, resync from this frame
    if (seq != s_expected_seq) {
        s_rx_pos = 0;
        s_expected_seq = seq;
    }
    s_expected_seq = (s_expected_seq + 1) & 0xFF;
    
    // Check buffer overflow
    if (s_rx_pos + payload_len > UART_CAN_RX_BUFFER_SIZE) {
        s_rx_pos = 0;
        return;
    }
    
    // Copy payload to buffer
    if (payload_len > 0) {
        memcpy(&s_rx_buffer[s_rx_pos], &data[2], payload_len);
        s_rx_pos += payload_len;
    }
    
    // If LAST flag set, message is complete
    if (is_last) {
        // Call application callback with complete message
        if (s_rx_callback && s_rx_pos > 0) {
            s_rx_callback(s_rx_buffer, s_rx_pos);
        }
        s_rx_pos = 0;  // Reset for next message
    }
}

// Public API for registering callback
void uart_can_register_rx_callback(uart_can_rx_callback_t callback) {
    s_rx_callback = callback;
}
```

### TX: Data → CAN (uart_over_can.c)
```c
#define UART_CAN_TX_ID          0x701
#define UART_CAN_MAX_DATA_PER_FRAME  6  // CAN max 8 bytes - 2 byte header
static uint8_t s_tx_seq = 0;

// Public API for sending data via UART over CAN
esp_err_t uart_can_send(const uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    
    size_t sent = 0;
    
    while (sent < len) {
        // Calculate chunk size (max 6 bytes per frame)
        uint8_t chunk_size = (len - sent) > UART_CAN_MAX_DATA_PER_FRAME ? 
                             UART_CAN_MAX_DATA_PER_FRAME : (uint8_t)(len - sent);
        
        bool is_last = (sent + chunk_size >= len);
        
        // Build CAN frame
        uint8_t frame[2 + UART_CAN_MAX_DATA_PER_FRAME];
        frame[0] = s_tx_seq++;  // SEQ wraps automatically at 256
        frame[1] = (chunk_size & 0x0F) | (is_last ? 0x80 : 0x00);
        memcpy(&frame[2], &data[sent], chunk_size);
        
        uint8_t dlc = 2 + chunk_size;
        
        // Send CAN frame via can_link
        esp_err_t err = can_link_send(UART_CAN_TX_ID, frame, dlc, timeout_ms);
        if (err != ESP_OK) {
            return err;
        }
        
        sent += chunk_size;
    }
    
    return ESP_OK;
}

// Convenience wrapper for strings
esp_err_t uart_can_send_string(const char *str, uint32_t timeout_ms) {
    if (!str) return ESP_ERR_INVALID_ARG;
    return uart_can_send((const uint8_t *)str, strlen(str), timeout_ms);
}
```

### Integration Example (app_tasks.c)
```c
#include "uart_over_can.h"
#include "openinverter_proto.h"

void io_task(void *arg) {
    // Initialize CAN link
    can_link_init(500);  // 500 kbps
    can_link_start();
    
    // Initialize UART over CAN tunnel
    uart_can_init();
    
    // Initialize OpenInverter protocol (uses UART over CAN)
    oi_proto_init(1);
    oi_proto_register_sdo_handler();  // Registers UART RX callback
    
    // Protocol is now ready - send commands via uart_can_send()
    // or use high-level oi_proto_* functions
}
```

### Notes
- **No physical UART needed** - all data flows through CAN bus
- **Callback-based architecture** - application receives complete messages via callback
- **OpenInverter terminal protocol** - uses text commands (get, set, save, etc.)
- If bus is **very busy**, add small delays between frames to lower bus occupancy
- For **CAN-FD**, increase chunk size (up to 62 bytes data after 2-byte header) + adjust DLC mapping

---

## 4) OpenInverter Terminal Protocol Integration

The ESP32 implementation uses UART over CAN to communicate with OpenInverter using **text-based terminal commands**:

### Command Format
```
get <param>\r\n          → Read parameter value
set <param> <value>\r\n  → Write parameter value
save\r\n                 → Save parameters to flash
load\r\n                 → Load parameters from flash
defaults\r\n             → Load default parameters
reset\r\n                → Reboot device
json\r\n                 → Get JSON parameter database
serial\r\n               → Get serial number
errors\r\n               → Get error log
```

### Response Format
```
<param>=<value> <unit>\r\n    → For get commands
OK\r\n                        → For successful operations
Error: <message>\r\n          → For errors
```

### Example Session
```
ESP32 → STM32: "get udc\r\n"
STM32 → ESP32: "udc=350.5 V\r\n"

ESP32 → STM32: "set opmode 1\r\n"
STM32 → ESP32: "OK\r\n"

ESP32 → STM32: "save\r\n"
STM32 → ESP32: "OK\r\n"
```

---

## 5) Testing & Bring-up

1. **CAN bus setup**
   - Connect STM32 & ESP32 with CAN transceivers + 120 Ω termination at both ends
   - Verify both devices can see CAN frames (use CAN analyzer if available)

2. **Filter verification**
   - Spam bus with unrelated IDs and confirm ESP32 only processes 0x700
   - Check TWAI alerts for RX queue overflows

3. **Terminal protocol test**
   - Send `serial\r\n` command from ESP32
   - Verify STM32 responds with serial number
   - Try `get` and `set` commands

4. **Loss/dup resilience**
   - Drop random frames (disable TX momentarily)
   - Verify reassembly resets gracefully with SEQ mismatch

5. **Performance test**
   - Measure throughput with large JSON download
   - Should achieve ~20-40 kB/s on 500 kbit/s CAN bus

---

## 6) Configuration Cheatsheet

- **IDs**: STM32→ESP32 `0x700`, ESP32→STM32 `0x701`
- **Payload**: up to 6 bytes data/frame (Classical CAN: 8 bytes total - 2 byte header)
- **Flags**: `LEN_FLAGS = (LEN & 0x0F) | (LAST<<7)`, where LEN ∈ [0, 6]
- **Resync**: reset on unexpected SEQ
- **Buffers**: RX 512 B (tune to your message sizes)
- **Backpressure**: add small inter-frame delay or throttle data rate

---

## 7) Security / Safety Notes
- High IDs lower priority; adjust if you must preempt non-critical traffic.
- Validate maximum message length at application boundary (avoid unlimited streams).
- Consider signing/CRC if bus integrity is a concern beyond CAN-level ACK.

---

## 8) Firmware Update Over CAN

### Overview
Since UART over CAN provides a transparent UART tunnel, **firmware updates work identically** to standard UART flashing. The ESP32 acts as a gateway, forwarding binary firmware data from the web interface to the STM32 bootloader via CAN bus.

### Process Flow
1. **Web Interface** uploads firmware file to ESP32
2. **ESP32** chunks the binary data and sends via UART over CAN frames
3. **STM32 Bootloader** receives data as normal UART stream and writes to flash
4. **ESP32** monitors progress and reports status back to web interface

### Bootloader Requirements
The STM32 bootloader must support UART-based firmware updates (most STM32 bootloaders do). No special CAN-aware bootloader is required - the UART over CAN tunnel makes the CAN bus appear as a standard UART interface to the bootloader.

### Implementation Notes
- **Binary Data Handling**: The protocol handles binary data transparently - no text encoding/decoding needed
- **Error Recovery**: CAN's built-in error detection helps ensure reliable firmware transfer
- **Progress Reporting**: ESP32 can relay bootloader status messages back through the tunnel
- **Timeout Handling**: Implement appropriate timeouts for large firmware transfers

### Example Command Flow
```
Web → ESP32: "Enter bootloader mode"
ESP32 → CAN → STM32: UART command to enter bootloader
STM32 → CAN → ESP32: "Ready for firmware"
ESP32 → CAN → STM32: Binary firmware data chunks
STM32 → CAN → ESP32: Progress updates ("25%", "50%", etc.)
STM32 → CAN → ESP32: "Update complete, rebooting"
ESP32 → Web: Success status
```

### Safety Considerations
- **Verify Checksums**: Always validate firmware integrity before and after transfer
- **Backup Recovery**: Ensure fallback mechanism if update fails
- **Bus Arbitration**: Firmware updates may require higher priority CAN frames during critical phases
- **Power Stability**: Ensure stable power during flash operations to prevent corruption

---

## 9) License
This specification and reference snippets are provided under the **MIT License**. Use at your own risk.
