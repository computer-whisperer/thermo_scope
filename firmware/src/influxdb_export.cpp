//
// Created by christian on 9/14/23.
//
#include "influxdb_export.hpp"
#include "influxdb_export.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "cyw43.h"
#include "power.h"
#include "uzlib.h"
#include "defl_static.h"

std::string influxdb_url = "kalogon-influxdb.cjbal.com";

enum InfluxDBConnectionState influxdb_connection_state = InfluxDBConnectionState_DISCONNECTED;

struct tcp_pcb* influxdb_pcb = nullptr;

constexpr bool use_uzlib = true;

static void tcp_close_con() {
  tcp_close(influxdb_pcb);
  influxdb_connection_state = InfluxDBConnectionState_DISCONNECTED;
}

static uint32_t influx_tx_buffer_pos = 0;
constexpr size_t influx_tx_text_buffer_size = 1<<10;
static char influx_tx_buffer[influx_tx_text_buffer_size];
static char influx_tx_gzip_buffer[influx_tx_text_buffer_size];
static bool influx_buffer_ready = true;

static err_t tcpRecvCallback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  if (p == NULL) {
    tcp_close_con();
  } else {
    tcp_recved(tpcb, p->len);
    pbuf_free(p);
  }
  return 0;
}


static void tcpErrorHandler(void *arg, err_t tpcb)
{
}


uint32_t last_compress_us = 0;
uint32_t last_crc_us = 0;
uint32_t last_send_us = 0;
absolute_time_t last_segment_start = nil_time;

static err_t tcpSendCallback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  influx_buffer_ready = true;
  last_send_us = absolute_time_diff_us(last_segment_start,  get_absolute_time());
  influx_tx_buffer_pos = 0;
  return ERR_OK;
}

static err_t connectCallback(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  if (err)
  {
    return err;
  }
  else
  {
    influxdb_connection_state = InfluxDBConnectionState_CONNECTED;
    influx_buffer_ready = true;
    influx_tx_buffer_pos = 0;
    return ERR_OK;
  }
}

static void dns_resolved_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
  if (ipaddr == NULL)
  {
    influxdb_connection_state = InfluxDBConnectionState_DISCONNECTED;
    return;
  }
  influxdb_pcb = tcp_new();
  tcp_err(influxdb_pcb, tcpErrorHandler);
  tcp_recv(influxdb_pcb, tcpRecvCallback);
  tcp_sent(influxdb_pcb, tcpSendCallback);
  tcp_connect(influxdb_pcb, ipaddr, 80, connectCallback);
}

void influxdb_try_connect() {
  if (influxdb_connection_state != InfluxDBConnectionState_DISCONNECTED)
  {
    return;
  }

  ip_addr_t addr;
  err_t ret = dns_gethostbyname_addrtype(influxdb_url.c_str(), &addr, dns_resolved_cb, nullptr, LWIP_DNS_ADDRTYPE_IPV6_IPV4);
  influxdb_connection_state = InfluxDBConnectionState_CONNECTING;
  if (ret == ERR_OK) {
    dns_resolved_cb(influxdb_url.c_str(), &addr, nullptr);
  }
  else if (ret == ERR_INPROGRESS) {
    // Callback should be called once resolved or failed
  }
  else {
    influxdb_connection_state = InfluxDBConnectionState_DISCONNECTED;
  }
}

static std::string header_text = "POST /api/v2/write?org=Kalogon&bucket=thermo_scope "
                                 "HTTP/1.1\r\n"
                                 "Connection: keep-alive\r\n"
                                 "Keep-Alive: timeout=60, max=0\r\n"
                                 "Content-Type: text/plain; charset=utf-8\r\n"
                                 "Host: kalogon-influxdb.cjbal.com\r\n"
                                 "Authorization: Token 0G8WAfZ9bgdcMVd7iFvXyACM71bd6PT9pGKnmiRBPDHW7m5eTvX2oV7JFsyO2yKoQQWImKF3_RYsNG70uIwNug==\r\n";

constexpr uint32_t uzlib_hash_bits = 8;
static uzlib_hash_entry_t uzlib_hashtable[1<<uzlib_hash_bits];

struct uzlib_comp uzlib_comp_data = {};

static void influxdb_send_packet(const char* body, uint32_t body_len)
{
  last_segment_start = get_absolute_time();
  const char * buffer_to_use = body;
  uint32_t content_len = body_len;

  if (use_uzlib) {
    uzlib_comp_data.outlen = 0;
    buffer_to_use = (const char *)influx_tx_gzip_buffer;
    content_len = 0;
    influx_tx_gzip_buffer[content_len++] = 0x1f; // ID1
    influx_tx_gzip_buffer[content_len++] = 0x8b; // ID2
    influx_tx_gzip_buffer[content_len++] = 0x08; // CM
    influx_tx_gzip_buffer[content_len++] = 0x00; // FLG
    influx_tx_gzip_buffer[content_len++] = 0x00; // MTIME
    influx_tx_gzip_buffer[content_len++] = 0x00;
    influx_tx_gzip_buffer[content_len++] = 0x00;
    influx_tx_gzip_buffer[content_len++] = 0x00;
    influx_tx_gzip_buffer[content_len++] = 0x04; // XFL
    influx_tx_gzip_buffer[content_len++] = 0xFF; // OS
    uzlib_comp_data.dict_size = 32768;
    uzlib_comp_data.hash_bits = uzlib_hash_bits;
    uzlib_comp_data.hash_table = uzlib_hashtable;
    memset(uzlib_hashtable, 0, sizeof(uzlib_hashtable));
    zlib_start_block(&uzlib_comp_data);
    uzlib_compress(&uzlib_comp_data, reinterpret_cast<const uint8_t *>(body), body_len);
    zlib_finish_block(&uzlib_comp_data);

    last_compress_us = absolute_time_diff_us(last_segment_start,  get_absolute_time());
    last_segment_start = get_absolute_time();

    memcpy(influx_tx_gzip_buffer + content_len, uzlib_comp_data.outbuf, uzlib_comp_data.outlen);
    content_len += uzlib_comp_data.outlen;
    uint32_t crc = ~uzlib_crc32(body, body_len, ~0);

    last_crc_us = absolute_time_diff_us(last_segment_start,  get_absolute_time());
    last_segment_start = get_absolute_time();

    memcpy(influx_tx_gzip_buffer + content_len, &crc, 4);
    content_len += 4;
    memcpy(influx_tx_gzip_buffer + content_len, &body_len, 4);
    content_len += 4;
  }

  tcp_write(influxdb_pcb, header_text.c_str(), header_text.length(), 0);

  std::string remaining_header = "Content-Length: " + std::to_string(content_len) + "\r\n\r\n";
  if (use_uzlib) {
    remaining_header = "Content-Encoding: gzip\r\n" + remaining_header;
  }
  tcp_write(influxdb_pcb, remaining_header.c_str(), remaining_header.length(), TCP_WRITE_FLAG_COPY);
  tcp_write(influxdb_pcb, buffer_to_use, content_len, 0);

  tcp_output(influxdb_pcb);
}

bool influxdb_can_push_point()
{
  return (influxdb_connection_state == InfluxDBConnectionState_CONNECTED && influx_buffer_ready && power_wifi_power_state);
}

#define APPEND(...) influx_tx_buffer_pos += snprintf(&influx_tx_buffer[influx_tx_buffer_pos], sizeof(influx_tx_buffer) - influx_tx_buffer_pos, __VA_ARGS__)


void influxdb_push_point(DataChannel::DataPoint &point, DataChannel *channel) {
  if (!influx_buffer_ready)
  {
    return;
  }
  // Assume there is enough space in tx buffer;
  APPEND("thermo_scope %s=%.16f %lld\n", channel->channel_name.c_str(), point.value, point.timestamp_us * 1000);

  if (influx_tx_buffer_pos > (sizeof(influx_tx_buffer) - 100))
  {
    influxdb_send_packet(influx_tx_buffer, influx_tx_buffer_pos);
    influx_buffer_ready = false;
  }
}

