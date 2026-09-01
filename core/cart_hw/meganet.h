#ifndef GPGX_MEGANET_H
#define GPGX_MEGANET_H

#include <stdint.h>

enum
{
  MEGANET_CART_NONE = 0,
  MEGANET_CART_MEGANET = 1,
  MEGANET_CART_TELEBRADESCO = 2
};

enum
{
  MEGANET_LINK_DOWN = 0,
  MEGANET_LINK_CONNECTING = 1,
  MEGANET_LINK_UP = 2,
  MEGANET_LINK_FAILED = 3,

  /*
   * Host-only control states used by the BizHawk physical-modem backend.
   * These are never exposed to the cartridge as modem result codes.
   */
  MEGANET_LINK_PHYSICAL_PASSTHROUGH_ENABLE = 4,
  MEGANET_LINK_PHYSICAL_PASSTHROUGH_DISABLE = 5
};

enum
{
  MEGANET_HOST_EVENT_NONE = 0,
  MEGANET_HOST_EVENT_CONNECT_REQUEST = 1,
  MEGANET_HOST_EVENT_DISCONNECT_REQUEST = 2
};

enum
{
  MEGANET_TRACE_NONE = 0,
  MEGANET_TRACE_UART_READ = 1,
  MEGANET_TRACE_UART_WRITE = 2,
  MEGANET_TRACE_RX_ENQUEUE = 3,
  MEGANET_TRACE_AT_COMMAND = 4,
  MEGANET_TRACE_HOST_EVENT = 5
};

typedef struct
{
  uint32_t type;
  uint32_t address;
  uint32_t value;
  uint32_t aux;
} meganet_trace_record_t;

typedef struct
{
  uint32_t active;
  uint32_t cart_type;
  uint32_t link_state;
  uint32_t rx_queued;
  uint32_t tx_queued;
  uint32_t uart_lcr;
  uint32_t uart_mcr;
  uint32_t numeric_mode;
  uint32_t echo_enabled;
} meganet_host_status_t;

int meganet_get_cart_type(void);
int meganet_is_cart(void);
int meganet_is_meganet_cart(void);
int meganet_is_telebradesco_cart(void);

void meganet_modem_reset(void);
void meganet_modem_install(void);

/* Non-blocking deterministic host contract. */
int meganet_host_pop_event(void);
int meganet_host_pop_trace(meganet_trace_record_t *record);
int meganet_host_read_tx(uint8_t *buffer, int capacity);
int meganet_host_write_rx(const uint8_t *buffer, int length);
int meganet_host_get_rx_space(void);
void meganet_host_set_link_state(int state);
void meganet_host_disconnect(void);
void meganet_host_reset(int report_no_carrier);
void meganet_host_get_status(meganet_host_status_t *status);

#endif /* GPGX_MEGANET_H */
