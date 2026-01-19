# UART 통신 모듈 설정 가이드

## ⚠️ 중요 사항
**CERTUS와 RPI가 둘 다 UART5로 설정되어 있습니다. 이는 물리적으로 불가능합니다.**
실제 하드웨어 연결에 맞게 수정이 필요합니다.

## CubeMX 설정 절차

### 1. UART 인스턴스 활성화
다음 UART들을 활성화하세요:
- **USART1** (Umbilical)
- **UART4** (IMU)
- **UART5** (RPI / CERTUS - 하나만 선택)
- **UART7** (Iridium)
- **UART8** (GPS)

### 2. 각 UART 기본 설정
각 UART에 대해:
- **Mode**: Asynchronous
- **Baudrate**: 필요에 따라 설정 (기본값 예: 115200 또는 9600)
- **Word Length**: 8 Bits
- **Parity**: None
- **Stop Bits**: 1
- **Data Direction**: Receive and Transmit

### 3. DMA 설정 (중요!)

각 UART에 대해 **2개의 DMA Request**를 추가:

#### RX DMA (수신):
- **DMA Request**: UARTx RX
- **Channel**: 적절한 DMA Channel 선택
- **Direction**: Peripheral to Memory
- **Priority**: Medium
- **Mode**: **Circular** ⚠️ (반드시 Circular!)
- **Increment Address**: Memory에만 체크
- **Data Width**: Byte to Byte

#### TX DMA (송신):
- **DMA Request**: UARTx TX
- **Channel**: 적절한 DMA Channel 선택
- **Direction**: Memory to Peripheral
- **Priority**: Medium
- **Mode**: **Normal**
- **Increment Address**: Memory에만 체크
- **Data Width**: Byte to Byte

### 4. NVIC 인터럽트 설정

각 UART에 대해:
- **USARTx global interrupt**: ✅ Enabled
- **USARTx interrupt**: ✅ Enabled

**USARTx IRQn** 우선순위 설정:
- **Preemption Priority**: 적절히 설정 (예: 5)
- **Sub Priority**: 0

### 5. IDLE 인터럽트 활성화

**UART Parameter Settings**에서:
- **Receiver timeout interrupt**: ✅ Enabled (가능한 경우)
- 또는 코드에서 `HAL_UARTEx_ReceiveToIdle_DMA()` 사용

### 6. 핀 할당

각 UART의 TX/RX 핀을 할당:
- **USART1**: PA9 (TX), PA10 (RX) 또는 다른 사용 가능한 핀
- **UART4**: PC10 (TX), PC11 (RX) 또는 다른 사용 가능한 핀
- **UART5**: PC12 (TX), PD2 (RX) 또는 다른 사용 가능한 핀
- **UART7**: PE7 (TX), PE8 (RX) 또는 다른 사용 가능한 핀
- **UART8**: PE0 (TX), PE1 (RX) 또는 다른 사용 가능한 핀

> ⚠️ **주의**: 실제 하드웨어 설계에 맞게 핀을 선택하세요!

### 7. 코드 생성 후 확인사항

CubeMX에서 코드 생성 후 다음 파일들이 자동 생성됩니다:
- `CM7/Core/Src/usart.c` (또는 uart.c) - UART 초기화 함수
- `CM7/Core/Src/dma.c` - DMA 초기화 함수
- `CM7/Core/Src/stm32h7xx_it.c` - 인터럽트 핸들러

생성된 함수들:
- `MX_USART1_UART_Init()`
- `MX_UART4_Init()`
- `MX_UART5_Init()`
- `MX_UART7_Init()`
- `MX_UART8_Init()`

### 8. main.c에서 초기화 추가

`main.c`의 `MX_GPIO_Init()` 호출 후에 추가:

```c
/* USER CODE BEGIN 2 */

/* Initialize UART instances */
MX_USART1_UART_Init();
MX_UART4_Init();
MX_UART5_Init();
MX_UART7_Init();
MX_UART8_Init();

/* Initialize UART Communication Module */
UART_Init(UART_UMBILICAL, 115200);  /* 예시: Umbilical 115200 baud */
UART_Init(UART_IMU, 115200);
UART_Init(UART_RPI, 115200);
UART_Init(UART_IRIDIUM, 115200);
UART_Init(UART_GPS, 9600);
// UART_Init(UART_CERTUS, 115200);  /* UART5 충돌 확인 후 활성화 */

/* USER CODE END 2 */
```

## 사용 예시

```c
#include "UARTComm.h"
#include "iCAP_global.h"

// 송신 예시
uint8_t data[] = "Hello World\r\n";
UART_Status_t status = UART_Transmit(UART_UMBILICAL, data, strlen(data));

// 수신 예시 (non-blocking)
uint8_t buffer[256];
uint16_t received = UART_Receive(UART_UMBILICAL, buffer, sizeof(buffer));

// 수신 데이터 확인
uint16_t available = UART_GetAvailableBytes(UART_UMBILICAL);
if (available > 0) {
    uint16_t read = UART_Receive(UART_UMBILICAL, buffer, available);
    // 데이터 처리...
}
```

## 문제 해결

### DMA가 동작하지 않는 경우
1. DMA 채널이 올바르게 할당되었는지 확인
2. DMA 모드가 RX는 Circular, TX는 Normal인지 확인
3. NVIC에서 DMA 인터럽트가 활성화되었는지 확인

### 데이터가 수신되지 않는 경우
1. IDLE 인터럽트가 활성화되었는지 확인
2. `HAL_UARTEx_ReceiveToIdle_DMA()` 사용 여부 확인
3. Ring buffer 크기가 충분한지 확인

### 전송이 완료되지 않는 경우
1. `UART_IsTransmitComplete()`로 상태 확인
2. TX DMA가 Normal 모드인지 확인
3. 버퍼 크기 확인 (TX_BUFFER_SIZE = 256)

## 매핑 요약

| 논리적 채널 | 물리적 UART | 용도 |
|------------|------------|------|
| UART_UMBILICAL | USART1 | Umbilical 통신 |
| UART_RPI | UART5 | Raspberry Pi 통신 |
| UART_GPS | UART8 | GPS 수신 |
| UART_IMU | UART4 | IMU 통신 |
| UART_IRIDIUM | UART7 | Iridium 통신 |
| UART_CERTUS | UART5 ⚠️ | CERTUS 통신 (RPI와 충돌!) |

