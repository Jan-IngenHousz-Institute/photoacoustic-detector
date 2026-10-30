/*
 * protocol.h - the JSON line protocol (see README): commands in, messages out.
 */
#pragma once
#include <stdint.h>

void proto_handle_line(const char *line);
void proto_send_id(void);
void proto_send_status(void);
void proto_send_data(uint32_t seq, uint64_t t_us);
void proto_send_burst(uint32_t seq, const int32_t *frames);
void proto_log(const char *level, const char *fmt, ...);
void proto_error(const char *fmt, ...);
/* self-test result list helpers (selftest.c) */
void proto_selftest_begin(void);
void proto_selftest_add(const char *name, const char *status, const char *detail);
void proto_selftest_end(void);
