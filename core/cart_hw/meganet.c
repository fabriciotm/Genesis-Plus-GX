/***************************************************************************************
 * Mega Net / Telebradesco RC224ATF device for the BizHawk GPGX Waterbox core.
 *
 * The cartridge sees the RC224ATF parallel host interface, compatible with a 16C450,
 * at $200001-$20000D. Hayes command processing remains in this deterministic C core.
 * TCP, serial ports, files, wall-clock time and threads are owned by the BizHawk host.
 *
 * Host exchange is made only through bounded in-memory queues and explicit link events.
 ***************************************************************************************/

#include "shared.h"
#include "meganet.h"

#include <string.h>

#define MEGANET_ADDRESS_MASK        0x00ffffffU
#define MEGANET_PAGE_BASE           0x00200000U
#define RC224ATF_REG_RBR_THR_DLL    0x00200001U
#define RC224ATF_REG_IER_DLM        0x00200003U
#define RC224ATF_REG_IIR            0x00200005U
#define RC224ATF_REG_LCR            0x00200007U
#define RC224ATF_REG_MCR            0x00200009U
#define RC224ATF_REG_LSR            0x0020000bU
#define RC224ATF_REG_MSR            0x0020000dU

#define RC224ATF_LCR_DLAB           0x80U
#define RC224ATF_LSR_DR             0x01U
#define RC224ATF_LSR_THRE           0x20U
#define RC224ATF_LSR_TEMT           0x40U
#define RC224ATF_MSR_CTS            0x10U
#define RC224ATF_MSR_DSR            0x20U
#define RC224ATF_MSR_DCD            0x80U

#define MEGANET_QUEUE_SIZE          65536U
#define MEGANET_QUEUE_MASK          (MEGANET_QUEUE_SIZE - 1U)
#define MEGANET_EVENT_QUEUE_SIZE    16U
#define MEGANET_EVENT_QUEUE_MASK    (MEGANET_EVENT_QUEUE_SIZE - 1U)
#define MEGANET_TRACE_QUEUE_SIZE    4096U
#define MEGANET_TRACE_QUEUE_MASK    (MEGANET_TRACE_QUEUE_SIZE - 1U)
#define MEGANET_AT_BUFFER_SIZE      256U
#define MEGANET_TX_BUSY_POLLS       2U

typedef struct
{
  uint8 data[MEGANET_QUEUE_SIZE];
  uint32 head;
  uint32 tail;
} meganet_byte_queue_t;

typedef struct
{
  uint8 data[MEGANET_EVENT_QUEUE_SIZE];
  uint32 head;
  uint32 tail;
} meganet_event_queue_t;

typedef struct
{
  meganet_trace_record_t data[MEGANET_TRACE_QUEUE_SIZE];
  uint32 head;
  uint32 tail;
} meganet_trace_queue_t;

typedef struct
{
  uint8 rbr;
  uint8 ier;
  uint8 iir;
  uint8 lcr;
  uint8 mcr;
  uint8 dll;
  uint8 dlm;
} rc224atf_host_if_state_t;

static rc224atf_host_if_state_t rc224atf_host_if;
static meganet_byte_queue_t meganet_rx_queue;
static meganet_byte_queue_t meganet_tx_queue;
static meganet_event_queue_t meganet_event_queue;
static meganet_trace_queue_t meganet_trace_queue;

static uint32 meganet_uart_read_count[7];
static uint32 meganet_uart_last_read[7];

static char meganet_at_buffer[MEGANET_AT_BUFFER_SIZE];
static uint32 meganet_at_len;
static uint32 meganet_link_state;
static uint32 meganet_modem_connected;
static uint32 meganet_physical_passthrough;
static uint32 meganet_tx_busy_polls;
static uint32 meganet_echo_enabled;
static uint32 meganet_echo_enable_when_rx_empty;
static uint32 meganet_numeric_mode;
static uint32 meganet_transport_last_dial_failed;

/*
 * Telebradesco login-frame compatibility state.
 *
 * With Data Cartao present, the ROM expands the MEGABRA login frame to
 * 40 bytes.  Its final ETX byte occupies 0xFF20EC, which is immediately
 * reused as the first byte ('A') of the dial-command buffer.  The original
 * RC224ATF path accepts the intended end of frame; the emulation normalizes
 * only this exact frame signature before forwarding it to the host.
 */
static uint32 meganet_tele_login_prefix_index;
static uint32 meganet_tele_login_frame_active;
static uint32 meganet_tele_login_frame_count;
static uint8 meganet_tele_login_previous_byte;
static void telebradesco_login_filter_reset(void);
static uint32 queue_count(const meganet_byte_queue_t *queue)
{
  return (queue->tail - queue->head) & MEGANET_QUEUE_MASK;
}

static uint32 queue_space(const meganet_byte_queue_t *queue)
{
  return MEGANET_QUEUE_MASK - queue_count(queue);
}

static int queue_push(meganet_byte_queue_t *queue, uint8 data)
{
  uint32 next = (queue->tail + 1U) & MEGANET_QUEUE_MASK;

  if (next == queue->head)
  {
    return 0;
  }

  queue->data[queue->tail] = data;
  queue->tail = next;
  return 1;
}

static int queue_pop(meganet_byte_queue_t *queue, uint8 *data)
{
  if (queue->head == queue->tail)
  {
    return 0;
  }

  *data = queue->data[queue->head];
  queue->head = (queue->head + 1U) & MEGANET_QUEUE_MASK;
  return 1;
}


static void trace_push(uint32 type, uint32 address, uint32 value, uint32 aux)
{
  uint32 next;
  meganet_trace_record_t *record;

  if (type == MEGANET_TRACE_NONE)
  {
    return;
  }

  next = (meganet_trace_queue.tail + 1U) & MEGANET_TRACE_QUEUE_MASK;
  if (next == meganet_trace_queue.head)
  {
    meganet_trace_queue.head =
      (meganet_trace_queue.head + 1U) & MEGANET_TRACE_QUEUE_MASK;
  }

  record = &meganet_trace_queue.data[meganet_trace_queue.tail];
  record->type = type;
  record->address = address;
  record->value = value;
  record->aux = aux;
  meganet_trace_queue.tail = next;
}

static void trace_uart_read(
  uint32 address,
  uint32 value,
  int force)
{
  uint32 index;
  uint32 count;

  if (meganet_modem_connected ||
      (address < RC224ATF_REG_RBR_THR_DLL) ||
      (address > RC224ATF_REG_MSR))
  {
    return;
  }

  index = (address - RC224ATF_REG_RBR_THR_DLL) >> 1;
  count = ++meganet_uart_read_count[index];

  if (force ||
      (value != meganet_uart_last_read[index]) ||
      (count <= 16U) ||
      ((count & 0xffU) == 0U))
  {
    trace_push(MEGANET_TRACE_UART_READ, address, value, count);
  }

  meganet_uart_last_read[index] = value;
}

static void event_push(uint8 event_type)
{
  uint32 next;

  if (event_type == MEGANET_HOST_EVENT_NONE)
  {
    return;
  }

  next = (meganet_event_queue.tail + 1U) & MEGANET_EVENT_QUEUE_MASK;
  if (next == meganet_event_queue.head)
  {
    /* Keep the most recent control request if the host stopped pumping. */
    meganet_event_queue.head = (meganet_event_queue.head + 1U) & MEGANET_EVENT_QUEUE_MASK;
  }

  meganet_event_queue.data[meganet_event_queue.tail] = event_type;
  meganet_event_queue.tail = next;
  trace_push(MEGANET_TRACE_HOST_EVENT, 0U, event_type, meganet_link_state);
}

static int meganet_header_has_token(const char *token)
{
  return (strstr(rominfo.international, token) != NULL) ||
         (strstr(rominfo.domestic, token) != NULL);
}

int meganet_get_cart_type(void)
{
  if (meganet_header_has_token("MEGA NET") ||
      meganet_header_has_token("MEGANET"))
  {
    return MEGANET_CART_MEGANET;
  }

  if (meganet_header_has_token("TELEBRADESCO") ||
      meganet_header_has_token("TELE BRADESCO") ||
      meganet_header_has_token("CART. BRADESCO") ||
      meganet_header_has_token("BRADESCO"))
  {
    return MEGANET_CART_TELEBRADESCO;
  }

  return MEGANET_CART_NONE;
}

int meganet_is_cart(void)
{
  return meganet_get_cart_type() != MEGANET_CART_NONE;
}

int meganet_is_meganet_cart(void)
{
  return meganet_get_cart_type() == MEGANET_CART_MEGANET;
}

int meganet_is_telebradesco_cart(void)
{
  return meganet_get_cart_type() == MEGANET_CART_TELEBRADESCO;
}

static uint32 meganet_normalize_address(uint32 address)
{
  address &= MEGANET_ADDRESS_MASK;

  if (address < 0x00010000U)
  {
    address |= MEGANET_PAGE_BASE;
  }

  return address;
}

static int meganet_is_rc224atf_register(uint32 address)
{
  address = meganet_normalize_address(address);

  return (address >= RC224ATF_REG_RBR_THR_DLL) &&
         (address <= RC224ATF_REG_MSR) &&
         ((address & 1U) != 0U);
}

static void rx_enqueue_byte(uint8 data)
{
  if (queue_push(&meganet_rx_queue, data))
  {
    trace_push(
      MEGANET_TRACE_RX_ENQUEUE,
      RC224ATF_REG_RBR_THR_DLL,
      data,
      queue_count(&meganet_rx_queue));
  }
}

static void rx_enqueue_string(const char *text)
{
  while (*text != '\0')
  {
    rx_enqueue_byte((uint8)*text++);
  }
}

static uint8 rx_dequeue_byte(void)
{
  uint8 data = 0x00U;

  if (queue_pop(&meganet_rx_queue, &data))
  {
    if ((queue_count(&meganet_rx_queue) == 0U) && meganet_echo_enable_when_rx_empty)
    {
      meganet_echo_enable_when_rx_empty = 0U;
      meganet_echo_enabled = 1U;

      /*
       * After NO DIALTONE, the Telebradesco ROM retries from ATH1.
       * Release the failed-dial latch only after the complete result code
       * has been consumed, so ATH1 receives its normal echo and OK.
       */
      if (meganet_link_state == MEGANET_LINK_FAILED)
      {
        meganet_transport_last_dial_failed = 0U;
      }
    }
  }

  rc224atf_host_if.rbr = data;
  return data;
}

static uint32 at_command_length_without_crlf(const char *command)
{
  uint32 length = 0U;

  while ((command[length] != '\0') &&
         (command[length] != '\r') &&
         (command[length] != '\n'))
  {
    length++;
  }

  return length;
}

static int at_command_equals(const char *command, const char *expected)
{
  uint32 i = 0U;
  uint32 length = at_command_length_without_crlf(command);

  while (expected[i] != '\0')
  {
    if ((i >= length) || (command[i] != expected[i]))
    {
      return 0;
    }
    i++;
  }

  return i == length;
}

static int at_command_contains(const char *command, const char *token)
{
  uint32 command_length = at_command_length_without_crlf(command);
  uint32 token_length = 0U;
  uint32 i;
  uint32 j;

  while (token[token_length] != '\0')
  {
    token_length++;
  }

  if ((token_length == 0U) || (token_length > command_length))
  {
    return 0;
  }

  for (i = 0U; i <= command_length - token_length; i++)
  {
    for (j = 0U; j < token_length; j++)
    {
      if (command[i + j] != token[j])
      {
        break;
      }
    }

    if (j == token_length)
    {
      return 1;
    }
  }

  return 0;
}

static int at_command_is_dial(const char *command)
{
  uint32 length = at_command_length_without_crlf(command);
  uint32 i;

  if ((length < 3U) || (command[0] != 'A') || (command[1] != 'T'))
  {
    return 0;
  }

  for (i = 2U; i < length; i++)
  {
    if (command[i] == 'D')
    {
      if ((i + 1U) >= length)
      {
        return 1;
      }

      switch (command[i + 1U])
      {
        case 'T':
        case 'P':
        case ',':
        case 'W':
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
          return 1;
        default:
          break;
      }
    }
  }

  return 0;
}

static int at_command_is_answer(const char *command)
{
  uint32 length = at_command_length_without_crlf(command);
  return (length >= 3U) &&
         (command[0] == 'A') &&
         (command[1] == 'T') &&
         (command[2] == 'A');
}

static void enqueue_ok_response(void)
{
  if (meganet_is_telebradesco_cart() || meganet_numeric_mode)
  {
    rx_enqueue_string("0\r");
  }
  else
  {
    rx_enqueue_string("0\r\n");
  }
}

static void enqueue_connect_response(void)
{
  if (meganet_is_telebradesco_cart() || meganet_numeric_mode)
  {
    rx_enqueue_string("1\r");
  }
  else
  {
    rx_enqueue_string("1\r\n");
  }
}

static void enqueue_no_carrier_response(void)
{
  if (meganet_numeric_mode)
  {
    rx_enqueue_string("3\r");
  }
  else
  {
    rx_enqueue_string("NO CARRIER\r\n");
  }
}

static void enqueue_no_dialtone_response(void)
{
  if (meganet_numeric_mode)
  {
    rx_enqueue_string("6\r");
  }
  else
  {
    rx_enqueue_string("NO DIALTONE\r\n");
  }
}

static void transport_connected(void)
{
  telebradesco_login_filter_reset();
  meganet_transport_last_dial_failed = 0U;
  meganet_link_state = MEGANET_LINK_UP;
  meganet_modem_connected = 1U;
  enqueue_connect_response();

  /* Compatibility stimulus confirmed by the working Telebradesco build. */
  if (meganet_is_telebradesco_cart())
  {
    rx_enqueue_string("login:\r");
  }
}

static void transport_failed(void)
{
  telebradesco_login_filter_reset();
  meganet_transport_last_dial_failed = 1U;
  meganet_link_state = MEGANET_LINK_FAILED;
  meganet_modem_connected = 0U;
  meganet_echo_enable_when_rx_empty =
    meganet_is_telebradesco_cart() ? 1U : 0U;
  meganet_echo_enabled = 0U;
  meganet_tx_queue.head = meganet_tx_queue.tail = 0U;

  /*
   * MEGANET_LINK_FAILED is reported only while the host is attempting the
   * initial TCP connection. Model that condition as absence of dial tone.
   *
   * Telebradesco retries after consuming result code 6. Keep echo disabled
   * while 6 CR is pending, then rx_dequeue_byte() restores echo and clears
   * the failed-dial latch when the result queue becomes empty.
   *
   * A socket closed after MEGANET_LINK_UP continues through
   * meganet_host_disconnect() and reports NO CARRIER.
   */
  enqueue_no_dialtone_response();
}

static void handle_at_command(const char *command)
{
  trace_push(
    MEGANET_TRACE_AT_COMMAND,
    0U,
    at_command_length_without_crlf(command),
    meganet_numeric_mode | (meganet_echo_enabled << 8));

  if (at_command_equals(command, "+++"))
  {
    if (!meganet_transport_last_dial_failed)
    {
      enqueue_ok_response();
    }
    return;
  }

  if (at_command_equals(command, "AT&F"))
  {
    meganet_modem_connected = 0U;
    meganet_link_state = MEGANET_LINK_DOWN;
    meganet_transport_last_dial_failed = 0U;
    return;
  }

  if (at_command_is_dial(command) || at_command_is_answer(command))
  {
    if (at_command_contains(command, "V0"))
    {
      meganet_numeric_mode = 1U;
    }

    meganet_transport_last_dial_failed = 0U;
    meganet_modem_connected = 0U;
    meganet_link_state = MEGANET_LINK_CONNECTING;
    event_push(MEGANET_HOST_EVENT_CONNECT_REQUEST);
    return;
  }

  if ((command[0] == 'A') && (command[1] == 'T') && (command[2] == 'H'))
  {
    if (meganet_link_state != MEGANET_LINK_DOWN)
    {
      event_push(MEGANET_HOST_EVENT_DISCONNECT_REQUEST);
    }

    meganet_modem_connected = 0U;
    meganet_link_state = MEGANET_LINK_DOWN;
    meganet_tx_queue.head = meganet_tx_queue.tail = 0U;

    if (meganet_transport_last_dial_failed)
    {
      meganet_echo_enable_when_rx_empty = 0U;
      meganet_echo_enabled = 0U;
      return;
    }

    enqueue_ok_response();
    meganet_echo_enable_when_rx_empty = meganet_numeric_mode ? 0U : 1U;
    return;
  }

  if (at_command_contains(command, "E0"))
  {
    meganet_echo_enabled = 0U;
    meganet_echo_enable_when_rx_empty = 0U;
  }
  else if (at_command_contains(command, "E1"))
  {
    meganet_echo_enabled = 1U;
    meganet_echo_enable_when_rx_empty = 0U;
  }

  {
    if (at_command_contains(command, "V0"))
    {
      meganet_numeric_mode = 1U;
    }
    else if (at_command_contains(command, "V1"))
    {
      meganet_numeric_mode = 0U;
    }

    /*
     * The Telebradesco bootstrap parser calls its two-byte receive helper
     * twice. It discards the first pair and compares the first byte of the
     * second pair with ASCII '0'. The command terminator echo is consumed by
     * the transmit helper, so the response itself must be:
     *
     *   CR LF | 0 CR
     *
     * Do not append a final LF: it would remain queued before the echo of ATH1.
     */
    if (meganet_is_telebradesco_cart() &&
        at_command_equals(command, "AT&FV0B0+FCLASS=1;+FF=1"))
    {
      rx_enqueue_string("\r\n0\r");
    }
    else
    {
      enqueue_ok_response();
    }
  }
}

static void telebradesco_login_filter_reset(void)
{
  meganet_tele_login_prefix_index = 0U;
  meganet_tele_login_frame_active = 0U;
  meganet_tele_login_frame_count = 0U;
  meganet_tele_login_previous_byte = 0U;
}

static int telebradesco_filter_connected_tx(uint8 *data)
{
  static const uint8 login_prefix[] =
  {
    0xffU, 0x03U, 0x43U, 0xb2U, 0xa2U,
    0xe2U, 0x82U, 0x42U, 0x4aU, 0x82U
  };

  uint8 value;

  if (data == NULL)
  {
    return 0;
  }

  value = *data;

  if (!meganet_is_telebradesco_cart())
  {
    return 1;
  }

  if (!meganet_tele_login_frame_active)
  {
    if (value == login_prefix[meganet_tele_login_prefix_index])
    {
      meganet_tele_login_prefix_index++;

      if (meganet_tele_login_prefix_index ==
          (uint32)(sizeof(login_prefix) / sizeof(login_prefix[0])))
      {
        meganet_tele_login_prefix_index = 0U;
        meganet_tele_login_frame_active = 1U;
        meganet_tele_login_frame_count =
          (uint32)(sizeof(login_prefix) / sizeof(login_prefix[0]));
        meganet_tele_login_previous_byte = value;
      }
    }
    else
    {
      meganet_tele_login_prefix_index =
        (value == login_prefix[0]) ? 1U : 0U;
    }

    return 1;
  }

  meganet_tele_login_frame_count++;

  /*
   * With Data Cartao, the ROM appends four NUL bytes after the four date
   * digits.  They are outside the observed 36-byte MEGABRA login format
   * and make the server reject the otherwise complete frame.  Suppress
   * only those exact positions when they are NUL; the no-card frame reaches
   * DLE/ETX at positions 35/36 and is therefore unchanged.
   */
  if ((meganet_tele_login_frame_count >= 35U) &&
      (meganet_tele_login_frame_count <= 38U) &&
      (value == 0x00U))
  {
    return 0;
  }

  /*
   * Bit-reversed DLE/ETX is 08 C0 on this UART path.  In the 40-byte
   * Data Cartao form, the ROM sends 08 82 because the ETX source byte
   * was overwritten by ASCII 'A'.  Correct only that exact position.
   */
  if ((meganet_tele_login_frame_count == 40U) &&
      (meganet_tele_login_previous_byte == 0x08U) &&
      (value == 0x82U))
  {
    value = 0xc0U;
  }

  if ((meganet_tele_login_previous_byte == 0x08U) &&
      (value == 0xc0U))
  {
    *data = value;
    telebradesco_login_filter_reset();
    return 1;
  }

  meganet_tele_login_previous_byte = value;
  *data = value;

  if (meganet_tele_login_frame_count >= 64U)
  {
    telebradesco_login_filter_reset();
  }

  return 1;
}

static void modem_tx_byte(uint8 data)
{
  /*
   * Telebradesco uses the physical modem as a real Fax Class 1 device.
   * In passthrough mode the ROM, not this deterministic compatibility
   * parser, owns the Hayes/Fax command state machine. Forward every THR
   * byte verbatim from power-on/command mode through the connected phase.
   */
  if (meganet_physical_passthrough)
  {
    (void)queue_push(&meganet_tx_queue, data);
    return;
  }

  if (meganet_modem_connected)
  {
    if (!telebradesco_filter_connected_tx(&data))
    {
      return;
    }

    (void)queue_push(&meganet_tx_queue, data);
    return;
  }

  if (meganet_echo_enabled)
  {
    rx_enqueue_byte(data);
  }

  if (data == 0x0dU)
  {
    if (meganet_at_len < MEGANET_AT_BUFFER_SIZE - 1U)
    {
      meganet_at_buffer[meganet_at_len++] = '\r';
    }

    meganet_at_buffer[meganet_at_len] = '\0';
    handle_at_command(meganet_at_buffer);
    meganet_at_len = 0U;
    meganet_at_buffer[0] = '\0';
    return;
  }

  if (data == 0x0aU)
  {
    return;
  }

  if (meganet_at_len < MEGANET_AT_BUFFER_SIZE - 1U)
  {
    meganet_at_buffer[meganet_at_len++] = (char)data;
    meganet_at_buffer[meganet_at_len] = '\0';
  }
}

static uint32 rc224atf_host_if_read(uint32 address)
{
  uint32 value;
  uint32 queued_before;

  address = meganet_normalize_address(address);

  switch (address)
  {
    case RC224ATF_REG_RBR_THR_DLL:
      queued_before = queue_count(&meganet_rx_queue);
      value = (rc224atf_host_if.lcr & RC224ATF_LCR_DLAB) ?
        rc224atf_host_if.dll : rx_dequeue_byte();
      trace_uart_read(address, value, queued_before != 0U);
      return value;

    case RC224ATF_REG_IER_DLM:
      value = (rc224atf_host_if.lcr & RC224ATF_LCR_DLAB) ?
        rc224atf_host_if.dlm : rc224atf_host_if.ier;
      trace_uart_read(address, value, 0);
      return value;

    case RC224ATF_REG_IIR:
      value = rc224atf_host_if.iir;
      trace_uart_read(address, value, 0);
      return value;

    case RC224ATF_REG_LCR:
      value = rc224atf_host_if.lcr;
      trace_uart_read(address, value, 0);
      return value;

    case RC224ATF_REG_MCR:
      value = rc224atf_host_if.mcr;
      trace_uart_read(address, value, 0);
      return value;

    case RC224ATF_REG_LSR:
      value = 0U;

      if (meganet_tx_busy_polls != 0U)
      {
        meganet_tx_busy_polls--;
      }
      else
      {
        value |= RC224ATF_LSR_THRE | RC224ATF_LSR_TEMT;
      }

      if (queue_count(&meganet_rx_queue) != 0U)
      {
        value |= RC224ATF_LSR_DR;
      }

      trace_uart_read(address, value, 0);
      return value;

    case RC224ATF_REG_MSR:
      value = RC224ATF_MSR_CTS | RC224ATF_MSR_DSR |
        (meganet_modem_connected ? RC224ATF_MSR_DCD : 0U);
      trace_uart_read(address, value, 0);
      return value;

    default:
      return 0xffU;
  }
}

static void rc224atf_host_if_write(uint32 address, uint32 data)
{
  uint8 value = (uint8)data;

  address = meganet_normalize_address(address);

  if (!meganet_modem_connected ||
      (address != RC224ATF_REG_RBR_THR_DLL) ||
      (rc224atf_host_if.lcr & RC224ATF_LCR_DLAB))
  {
    trace_push(
      MEGANET_TRACE_UART_WRITE,
      address,
      value,
      rc224atf_host_if.lcr);
  }

  switch (address)
  {
    case RC224ATF_REG_RBR_THR_DLL:
      if (rc224atf_host_if.lcr & RC224ATF_LCR_DLAB)
      {
        rc224atf_host_if.dll = value;
      }
      else
      {
        meganet_tx_busy_polls = MEGANET_TX_BUSY_POLLS;
        modem_tx_byte(value);
      }
      return;

    case RC224ATF_REG_IER_DLM:
      if (rc224atf_host_if.lcr & RC224ATF_LCR_DLAB)
      {
        rc224atf_host_if.dlm = value;
      }
      else
      {
        rc224atf_host_if.ier = value;
      }
      return;

    case RC224ATF_REG_IIR:
      /* IIR is read-only. Preserve the historical bit-1 write as an RX clear. */
      if (value & 0x02U)
      {
        meganet_rx_queue.head = meganet_rx_queue.tail = 0U;
      }
      return;

    case RC224ATF_REG_LCR:
      rc224atf_host_if.lcr = value;
      return;

    case RC224ATF_REG_MCR:
      rc224atf_host_if.mcr = value;
      return;

    default:
      return;
  }
}

static uint32 meganet_rom_read8(uint32 address)
{
  address = meganet_normalize_address(address);
  return READ_BYTE(cart.rom, address & cart.mask);
}

static uint32 meganet_rom_read16(uint32 address)
{
  address = meganet_normalize_address(address);
  return *(uint16 *)(cart.rom + (address & cart.mask));
}

static uint32 meganet_read8(uint32 address)
{
  uint32 normalized = meganet_normalize_address(address);

  if (meganet_is_rc224atf_register(normalized))
  {
    return rc224atf_host_if_read(normalized);
  }

  return meganet_rom_read8(normalized);
}

static uint32 meganet_read16(uint32 address)
{
  if (meganet_is_rc224atf_register(address) ||
      meganet_is_rc224atf_register(address + 1U))
  {
    return (meganet_read8(address) << 8) | meganet_read8(address + 1U);
  }

  return meganet_rom_read16(address);
}

static void meganet_write8(uint32 address, uint32 data)
{
  uint32 normalized = meganet_normalize_address(address);

  if (meganet_is_rc224atf_register(normalized))
  {
    rc224atf_host_if_write(normalized, data);
    return;
  }

  m68k_unused_8_w(normalized, data);
}

static void meganet_write16(uint32 address, uint32 data)
{
  if (meganet_is_rc224atf_register(address) ||
      meganet_is_rc224atf_register(address + 1U))
  {
    meganet_write8(address, data >> 8);
    meganet_write8(address + 1U, data & 0xffU);
    return;
  }

  m68k_unused_16_w(address, data);
}

int meganet_host_pop_event(void)
{
  uint8 event_type;

  if (meganet_event_queue.head == meganet_event_queue.tail)
  {
    return MEGANET_HOST_EVENT_NONE;
  }

  event_type = meganet_event_queue.data[meganet_event_queue.head];
  meganet_event_queue.head =
    (meganet_event_queue.head + 1U) & MEGANET_EVENT_QUEUE_MASK;
  return event_type;
}

int meganet_host_pop_trace(meganet_trace_record_t *record)
{
  if ((record == NULL) ||
      (meganet_trace_queue.head == meganet_trace_queue.tail))
  {
    return 0;
  }

  *record = meganet_trace_queue.data[meganet_trace_queue.head];
  meganet_trace_queue.head =
    (meganet_trace_queue.head + 1U) & MEGANET_TRACE_QUEUE_MASK;
  return 1;
}

int meganet_host_read_tx(uint8_t *buffer, int capacity)
{
  int count = 0;
  uint8 data;

  if ((buffer == NULL) || (capacity <= 0))
  {
    return 0;
  }

  while ((count < capacity) && queue_pop(&meganet_tx_queue, &data))
  {
    buffer[count++] = data;
  }

  return count;
}

int meganet_host_write_rx(const uint8_t *buffer, int length)
{
  int count = 0;

  if ((buffer == NULL) || (length <= 0) ||
      (!meganet_modem_connected && !meganet_physical_passthrough))
  {
    return 0;
  }

  while ((count < length) && queue_push(&meganet_rx_queue, buffer[count]))
  {
    count++;
  }

  return count;
}

int meganet_host_get_rx_space(void)
{
  return (int)queue_space(&meganet_rx_queue);
}

void meganet_host_set_link_state(int state)
{
  if (state == MEGANET_LINK_PHYSICAL_PASSTHROUGH_ENABLE)
  {
    /*
     * Only Telebradesco needs transparent physical UART/Hayes forwarding.
     * MegaNet keeps the validated 0045i high-level Hayes/data-mode path.
     */
    if (meganet_is_telebradesco_cart())
    {
      meganet_physical_passthrough = 1U;
      meganet_link_state = MEGANET_LINK_DOWN;
      meganet_modem_connected = 0U;
      meganet_transport_last_dial_failed = 0U;
      meganet_echo_enable_when_rx_empty = 0U;
      meganet_echo_enabled = 0U;
      meganet_at_len = 0U;
      meganet_at_buffer[0] = '\0';
      telebradesco_login_filter_reset();

      /*
       * Drop compatibility-parser residue before the real modem takes over.
       * From this point onward RX/TX contain only physical modem bytes.
       */
      meganet_rx_queue.head = meganet_rx_queue.tail = 0U;
      meganet_tx_queue.head = meganet_tx_queue.tail = 0U;
      meganet_event_queue.head = meganet_event_queue.tail = 0U;
    }
    return;
  }

  if (state == MEGANET_LINK_PHYSICAL_PASSTHROUGH_DISABLE)
  {
    meganet_physical_passthrough = 0U;
    meganet_link_state = MEGANET_LINK_DOWN;
    meganet_modem_connected = 0U;
    meganet_transport_last_dial_failed = 0U;
    meganet_echo_enable_when_rx_empty = 0U;
    meganet_at_len = 0U;
    meganet_at_buffer[0] = '\0';
    telebradesco_login_filter_reset();
    meganet_rx_queue.head = meganet_rx_queue.tail = 0U;
    meganet_tx_queue.head = meganet_tx_queue.tail = 0U;
    meganet_event_queue.head = meganet_event_queue.tail = 0U;
    return;
  }

  /*
   * While transparent passthrough is active, UP/DOWN are only the physical
   * carrier indication used to drive MSR.DCD and diagnostics.  Do not call
   * transport_connected()/transport_failed(): those functions synthesize
   * CONNECT/login:/NO DIALTONE bytes which must come from the real modem.
   */
  if (meganet_physical_passthrough)
  {
    switch (state)
    {
      case MEGANET_LINK_UP:
        meganet_link_state = MEGANET_LINK_UP;
        meganet_modem_connected = 1U;
        break;

      case MEGANET_LINK_CONNECTING:
        meganet_link_state = MEGANET_LINK_CONNECTING;
        meganet_modem_connected = 0U;
        break;

      case MEGANET_LINK_FAILED:
        meganet_link_state = MEGANET_LINK_FAILED;
        meganet_modem_connected = 0U;
        break;

      case MEGANET_LINK_DOWN:
      default:
        meganet_link_state = MEGANET_LINK_DOWN;
        meganet_modem_connected = 0U;
        break;
    }
    return;
  }

  switch (state)
  {
    case MEGANET_LINK_CONNECTING:
      meganet_link_state = MEGANET_LINK_CONNECTING;
      meganet_modem_connected = 0U;
      break;

    case MEGANET_LINK_UP:
      if (meganet_link_state == MEGANET_LINK_CONNECTING)
      {
        transport_connected();
      }
      break;

    case MEGANET_LINK_FAILED:
      if (meganet_link_state == MEGANET_LINK_CONNECTING)
      {
        transport_failed();
      }
      break;

    case MEGANET_LINK_DOWN:
    default:
      telebradesco_login_filter_reset();
      meganet_link_state = MEGANET_LINK_DOWN;
      meganet_modem_connected = 0U;
      break;
  }
}

void meganet_host_disconnect(void)
{
  uint32 was_active = meganet_modem_connected ||
    (meganet_link_state == MEGANET_LINK_CONNECTING);

  if (meganet_physical_passthrough)
  {
    /*
     * The physical modem is authoritative in passthrough mode. Never append
     * a synthetic NO CARRIER to bytes already delivered by the COM port.
     */
    meganet_link_state = MEGANET_LINK_DOWN;
    meganet_modem_connected = 0U;
    return;
  }

  telebradesco_login_filter_reset();
  meganet_link_state = MEGANET_LINK_DOWN;
  meganet_modem_connected = 0U;
  meganet_tx_queue.head = meganet_tx_queue.tail = 0U;

  if (was_active)
  {
    enqueue_no_carrier_response();
  }
}

void meganet_host_reset(int report_no_carrier)
{
  uint32 was_active = meganet_modem_connected ||
    (meganet_link_state == MEGANET_LINK_CONNECTING);
  uint32 was_physical_passthrough = meganet_physical_passthrough;

  meganet_event_queue.head = meganet_event_queue.tail = 0U;
  meganet_tx_queue.head = meganet_tx_queue.tail = 0U;
  telebradesco_login_filter_reset();
  meganet_link_state = MEGANET_LINK_DOWN;
  meganet_modem_connected = 0U;

  /*
   * Physical passthrough is a host configuration, not volatile modem state.
   * Preserve it across host/core resets.  Only the explicit
   * MEGANET_LINK_PHYSICAL_PASSTHROUGH_DISABLE control state may turn it off.
   *
   * BizHawk can configure/open the COM before meganet_modem_install()/reset
   * runs. Clearing this flag here would leave the COM open while silently
   * returning the native core to its synthetic Hayes parser.
   */

  if (was_active || was_physical_passthrough)
  {
    /* Bytes restored from a live remote stream cannot be replayed safely. */
    meganet_rx_queue.head = meganet_rx_queue.tail = 0U;
    meganet_at_len = 0U;
    meganet_at_buffer[0] = '\0';

    if (report_no_carrier && !was_physical_passthrough)
    {
      enqueue_no_carrier_response();
    }
  }
}

void meganet_host_get_status(meganet_host_status_t *status)
{
  if (status == NULL)
  {
    return;
  }

  status->active = meganet_is_cart() ? 1U : 0U;
  status->cart_type = (uint32_t)meganet_get_cart_type();
  status->link_state = meganet_link_state;
  status->rx_queued = queue_count(&meganet_rx_queue);
  status->tx_queued = queue_count(&meganet_tx_queue);
  status->uart_lcr = rc224atf_host_if.lcr;
  status->uart_mcr = rc224atf_host_if.mcr;
  status->numeric_mode = meganet_numeric_mode;
  status->echo_enabled = meganet_echo_enabled;
}

void meganet_modem_reset(void)
{
  memset(&rc224atf_host_if, 0, sizeof(rc224atf_host_if));
  memset(&meganet_rx_queue, 0, sizeof(meganet_rx_queue));
  memset(&meganet_tx_queue, 0, sizeof(meganet_tx_queue));
  memset(&meganet_event_queue, 0, sizeof(meganet_event_queue));
  memset(&meganet_trace_queue, 0, sizeof(meganet_trace_queue));
  memset(meganet_at_buffer, 0, sizeof(meganet_at_buffer));

  rc224atf_host_if.iir = 0x01U;
  meganet_at_len = 0U;
  meganet_link_state = MEGANET_LINK_DOWN;
  meganet_modem_connected = 0U;

  /*
   * Do not clear meganet_physical_passthrough here.
   * The host can enable Telebradesco raw UART/Fax Class 1 passthrough before
   * this reset is reached during core/cart initialization. Reset the emulated
   * modem registers and queues, but retain the selected transport mode.
   */
  meganet_tx_busy_polls = 0U;
  meganet_echo_enabled = meganet_is_telebradesco_cart() ? 1U : 0U;
  meganet_echo_enable_when_rx_empty = 0U;
  meganet_numeric_mode = 0U;
  meganet_transport_last_dial_failed = 0U;
  telebradesco_login_filter_reset();
  memset(meganet_uart_read_count, 0, sizeof(meganet_uart_read_count));
  memset(meganet_uart_last_read, 0xff, sizeof(meganet_uart_last_read));
}

void meganet_modem_install(void)
{
  if (!meganet_is_cart())
  {
    return;
  }

  meganet_modem_reset();

  /* Keep a valid ROM base for direct reads and md_cart_context_save(). */
  m68k.memory_map[0x20].base = cart.rom + ((0x20U << 16) & cart.mask);
  m68k.memory_map[0x20].read8 = meganet_read8;
  m68k.memory_map[0x20].read16 = meganet_read16;
  m68k.memory_map[0x20].write8 = meganet_write8;
  m68k.memory_map[0x20].write16 = meganet_write16;

  zbank_memory_map[0x20].read = meganet_read8;
  zbank_memory_map[0x20].write = meganet_write8;
}
