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
byte1: LEN_FLAGS     // b7=LAST (1=last chunk), b0..3=LEN (0..7 data bytes in this frame)
byte2..(1+LEN): DATA // up to 7 bytes
```
- Split long UART payloads into chunks of ≤7B.
- **LAST=1** marks the end of a reassembled message.
- **Resync rule**: If an unexpected `SEQ` arrives, **reset reassembly** and start from that frame.

> **Tip**: Treat every UART “write() call” as one logical message. If you stream indefinitely, insert periodic LAST=1 boundaries (e.g., after N bytes or idle timeout).

### Timing & Throughput
- Works with **Classical CAN** (8-byte payload). CAN-FD optional (64 B payload → set max chunk=61 B).
- Typical sustained throughput on 500 kbit/s CAN with other traffic: **~20–40 kB/s**.
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

        // Validate frame
        if (n > 7 || (2 + n) > dlc) {
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
        uint8_t chunkSize = (length - sent) > 7 ? 7 : (uint8_t)(length - sent);
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
- RX task reconstructs frames and writes to UART. TX task reads UART, chunks to ≤7B, sends to `0x701`.

### TWAI Config
```c
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = {
    .acceptance_code = (0x700 << 21),       // 11-bit ID at MSBs
    .acceptance_mask = (0x7FF << 21),       // exact match
    .single_filter   = true
};
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_RX, GPIO_NUM_TX, TWAI_MODE_NORMAL);
// init & start
twai_driver_install(&g_config, &t_config, &f_config);
twai_start();
```

### RX Reassembly → UART
```c
static uint8_t rx_buf[RX_BUFFER_SIZE];
static size_t  rx_len = 0;
static uint8_t expect_seq = 0;

void twai_rx_task(void *arg){
    while (1) {
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
        if (m.rtr || m.extd || m.identifier != 0x700 || m.data_length_code < 2) continue;

        uint8_t *d = m.data;
        uint8_t seq = d[0];
        uint8_t lf  = d[1];
        uint8_t n   = lf & 0x0F;
        bool last   = (lf & 0x80) != 0;
        if (n > 7 || (2 + n) > m.data_length_code) { rx_len = 0; continue; }

        if (rx_len == 0) expect_seq = seq;
        if (seq != expect_seq) { rx_len = 0; expect_seq = seq; }
        expect_seq++;

        if ((rx_len + n) > sizeof(rx_buf)) { rx_len = 0; continue; }
        memcpy(&rx_buf[rx_len], &d[2], n);
        rx_len += n;

        if (last) {
            uart_write_bytes(UART_NUM_1, (const char*)rx_buf, rx_len);
            rx_len = 0;
        }
    }
}
```

### UART → CAN (TWAI TX)
```c
static void twai_send_chunk(const uint8_t *p, uint8_t len, bool last){
    static uint8_t seq = 0;
    twai_message_t m = {0};
    m.identifier = 0x701;
    m.data_length_code = 2 + len;
    m.data[0] = seq++;
    m.data[1] = (len & 0x0F) | (last ? 0x80 : 0x00);
    memcpy(&m.data[2], p, len);
    twai_transmit(&m, pdMS_TO_TICKS(50));
}

void uart_to_twai_task(void *arg){
    uint8_t buf[UART_TX_CHUNK_MAX];
    while (1) {
        int n = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0) {
            // If you want to group into messages, collect more here; SIMPLE: send each chunk as a frame
            twai_send_chunk(buf, (uint8_t)n, true);
            // optional: vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
```

### Notes
- Pin assign: use proper GPIOs for TWAI RX/TX and set matching UART pins.
- If bus is **very busy**, add small delays between frames to lower bus occupancy.
- If you need **CAN-FD**, switch to an FD-capable transceiver/driver and increase chunk size (up to 61 B payload) + adjust DLC mapping.

---

## 4) Testing & Bring-up

1. **Loopback smoke test**
   - Start ESP32 TWAI in `TWAI_MODE_NO_ACK` loopback; send/recv local frame, verify chunk logic.
2. **Point-to-point on bench**
   - Connect STM32 & ESP32 with a proper CAN transceiver each + 120 Ω termination at both ends.
3. **Filter sanity**
   - Spam the bus with unrelated IDs and confirm neither side sees them (only your IDs pass). 
4. **Loss/dup resilience**
   - Drop random frames (disable TX momentarily). Verify reassembly resets gracefully.
5. **UART echo**
   - Tie ESP32 UART RX to TX; send from STM32 → CAN → ESP32 → echo back to STM32. Confirm byte‑exact roundtrip.

---

## 5) Configuration Cheatsheet

- **IDs**: STM32→ESP32 `0x700`, ESP32→STM32 `0x701`
- **Payload**: up to 7 B/frame (Classical CAN)
- **Flags**: `LEN_FLAGS = (LEN & 0x0F) | (LAST<<7)`
- **Resync**: reset on unexpected SEQ
- **Buffers**: RX 512 B (tune to your message sizes)
- **Backpressure**: add small inter-frame delay or throttle UART

---

## 6) Security / Safety Notes
- High IDs lower priority; adjust if you must preempt non-critical traffic.
- Validate maximum message length at application boundary (avoid unlimited streams).
- Consider signing/CRC if bus integrity is a concern beyond CAN-level ACK.

---

## 7) Firmware Update Over CAN

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

## 8) License
This specification and reference snippets are provided under the **MIT License**. Use at your own risk.
