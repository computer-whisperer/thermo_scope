//
// Created by christian on 9/14/23.
//

#ifndef THERMO_SCOPE_INFLUXDB_EXPORT_H
#define THERMO_SCOPE_INFLUXDB_EXPORT_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

enum InfluxDBConnectionState {
  InfluxDBConnectionState_INVALID,
  InfluxDBConnectionState_DISCONNECTED,
  InfluxDBConnectionState_CONNECTED,
  InfluxDBConnectionState_CONNECTING,
};

extern enum InfluxDBConnectionState influxdb_connection_state;

EXTERNC void influxdb_try_connect();

#endif //THERMO_SCOPE_INFLUXDB_EXPORT_H
