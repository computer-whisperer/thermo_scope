//
// Created by christian on 9/14/23.
//
#include "influxdb_export.hpp"
#include "influxdb_export.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "cyw43.h"

std::string influxdb_url = "kalogon-influxdb.cjbal.com";

enum InfluxDBConnectionState influxdb_connection_state = InfluxDBConnectionState_DISCONNECTED;

struct tcp_pcb* influxdb_pcb = nullptr;

static void tcp_close_con() {
  tcp_close(influxdb_pcb);
  influxdb_connection_state = InfluxDBConnectionState_DISCONNECTED;
}

static uint32_t influx_tx_buffer_pos = 0;
static char influx_tx_buffer[1024];
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

static err_t tcpSendCallback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  influx_buffer_ready = true;
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
                                 "Host: kalogon-influxdb.cjbal.com\r\n"
                                 "Authorization: Token 0G8WAfZ9bgdcMVd7iFvXyACM71bd6PT9pGKnmiRBPDHW7m5eTvX2oV7JFsyO2yKoQQWImKF3_RYsNG70uIwNug==\r\n";

static void influxdb_send_packet(const char* body, uint32_t body_len)
{


  tcp_write(influxdb_pcb, header_text.c_str(), header_text.length(), 0);

  std::string content_length_hdr = "Content-Length: " + std::to_string(body_len) + "\r\n\r\n";

  tcp_write(influxdb_pcb, content_length_hdr.c_str(), content_length_hdr.length(), TCP_WRITE_FLAG_COPY);

  tcp_write(influxdb_pcb, body, body_len, 0);

  tcp_output(influxdb_pcb);
}

bool influxdb_can_push_point()
{
  return (influxdb_connection_state == InfluxDBConnectionState_CONNECTED && influx_buffer_ready);
}

#define APPEND(...) influx_tx_buffer_pos += snprintf(&influx_tx_buffer[influx_tx_buffer_pos], sizeof(influx_tx_buffer) - influx_tx_buffer_pos, __VA_ARGS__)

void influxdb_push_point(DataChannel::DataPoint &point, DataChannel *channel) {
  if (!influx_buffer_ready)
  {
    return;
  }
  // Assume there is enough space in tx buffer;
  APPEND("thermo_scope %s=%.16f %lld\n", channel->channel_name.c_str(), point.value, point.timestamp);

  if (influx_tx_buffer_pos > (sizeof(influx_tx_buffer) - 100))
  {
    influxdb_send_packet(influx_tx_buffer, influx_tx_buffer_pos);
    influx_buffer_ready = false;
  }
}

