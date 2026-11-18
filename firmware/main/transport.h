#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

bool transport_open(struct uxrCustomTransport *t);
bool transport_close(struct uxrCustomTransport *t);
size_t transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err);
size_t transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err);
