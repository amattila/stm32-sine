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
- Configure CAN (HAL) with a single 11-bit **accept-all mask** but **centered at `0x701`** so only that ID passes.
- ISR collects frames, reassembles into a buffer, writes to UART when **LAST=1**.
- TX path reads UART, chunks into ≤7B, sets **LAST** on the final chunk, and transmits to `0x700`.

### Compile-Time Parameters
```c
#define CAN_BAUD             500000        // adjust to your bus
#define CAN_ID_TX            0x700         // STM32 -> ESP32
#define CAN_ID_RX            0x701         // ESP32 -> STM32
#define UART_TX_CHUNK_MAX    7
#define RX_BUFFER_SIZE       512
#define INTERFRAME_DELAY_US  1000          // 1 ms, tune as needed
```

### CAN Filter (HAL)
```c
static void CAN_ConfigFilter_Only701(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef f = {0};
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = (CAN_ID_RX << 5);
    f.FilterIdLow  = 0;
    f.FilterMaskIdHigh = (0x7FF << 5);  // exact match
    f.FilterMaskIdLow  = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &f);
}
```

### RX Reassembly ISR → UART TX
```c
static uint8_t  rx_buf[RX_BUFFER_SIZE];
static size_t   rx_len = 0;
static uint8_t  expect_seq = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef hdr; 
    uint8_t d[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) != HAL_OK) return;
    if (hdr.IDE != CAN_ID_STD || hdr.StdId != CAN_ID_RX || hdr.DLC < 2) return;

    const uint8_t seq = d[0];
    const uint8_t lf  = d[1];
    const uint8_t n   = lf & 0x0F;
    const bool last   = (lf & 0x80) != 0;
    if (n > 7u || (2u + n) > hdr.DLC) { rx_len = 0; return; }

    if (rx_len == 0) expect_seq = seq;           // establish sync
    if (seq != expect_seq) { rx_len = 0; expect_seq = seq; }  // resync
    expect_seq++;

    if ((rx_len + n) > sizeof(rx_buf)) { rx_len = 0; return; }
    memcpy(&rx_buf[rx_len], &d[2], n);
    rx_len += n;

    if (last) {
        // send to local UART (blocking or ring-buffered)
        HAL_UART_Transmit(&huartX, rx_buf, rx_len, 50);
        rx_len = 0;
    }
}
```

### UART → CAN TX (Chunking)
```c
static void can_send_chunk(const uint8_t *p, uint8_t len, uint8_t last) {
    static uint8_t seq = 0;
    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId = CAN_ID_TX; tx.IDE = CAN_ID_STD; tx.RTR = CAN_RTR_DATA; 
    tx.DLC = 2 + len;
    uint8_t d[8];
    d[0] = seq++;
    d[1] = (len & 0x0F) | (last ? 0x80 : 0x00);
    memcpy(&d[2], p, len);
    uint32_t mb;
    while (HAL_CAN_AddTxMessage(&hcan, &tx, d, &mb) != HAL_OK) { /* retry/backoff */ }
}

void uart_to_can_send(const uint8_t *p, size_t len) {
    while (len) {
        uint8_t chunk = (len > UART_TX_CHUNK_MAX) ? UART_TX_CHUNK_MAX : (uint8_t)len;
        size_t rem = len - chunk;
        can_send_chunk(p, chunk, rem == 0);
        p   += chunk;
        len -= chunk;
        // optional throttle for busy buses
        // delay_us(INTERFRAME_DELAY_US);
    }
}
```

### Integration Hooks (stm32-sine)
- Call `CAN_ConfigFilter_Only701()` after CAN init.
- Enable interrupts: `HAL_CAN_Start()` + `HAL_CAN_ActivateNotification(..., CAN_IT_RX_FIFO0_MSG_PENDING)`.
- In UART RX IRQ or task, gather bytes (one logical “message”) then call `uart_to_can_send()`.

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

## 7) License
This specification and reference snippets are provided under the **MIT License**. Use at your own risk.
