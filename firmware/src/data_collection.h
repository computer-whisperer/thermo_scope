#include <sys/cdefs.h>
//
// Created by christian on 9/6/23.
//

#ifndef THERMO_SCOPE_DATA_COLLECTION_H
#define THERMO_SCOPE_DATA_COLLECTION_H
#include <stdint.h>

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void data_collection();

EXTERNC void data_collection_core0_process_samples();

EXTERNC void* data_collection_get_default_channel_pointer();

EXTERNC void* data_collection_get_channel_pointer(char* name);

EXTERNC uint32_t data_collection_update_channel_names(lv_obj_t * dropdown, uint32_t current_rev);

EXTERNC uint32_t data_collection_get_channel_chart_data(void* channel_ref, double* buffer, uint32_t buffer_size);

#undef EXTERNC

#endif //THERMO_SCOPE_DATA_COLLECTION_H
