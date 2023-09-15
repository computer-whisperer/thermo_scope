//
// Created by christian on 9/14/23.
//

#ifndef THERMO_SCOPE_INFLUXDB_EXPORT_HPP
#define THERMO_SCOPE_INFLUXDB_EXPORT_HPP
#include "data_collection.hpp"



bool influxdb_can_push_point();

void influxdb_push_point(DataChannel::DataPoint &point, DataChannel* channel);


#endif //THERMO_SCOPE_INFLUXDB_EXPORT_HPP
