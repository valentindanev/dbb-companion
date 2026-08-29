#ifndef DBB_NETLOG_H
#define DBB_NETLOG_H

/*
 * Wi-Fi and heap breadcrumbs for SYSTEM.LOG.
 *
 * WHY THIS IS NOT JUST A CALL TO db_sonar_log_log_system_event():
 * that path ends in db_log_appendf(), which takes the logging mutex with
 * portMAX_DELAY and then writes to flash. The same mutex is held network-paced
 * for the whole of a log download. Calling it from the Wi-Fi event task would
 * therefore let a log download stall Wi-Fi itself - including the download.
 * So the event task only ever formats a short string into a queue and returns;
 * a normal task drains it later.
 *
 * Written after 22-08-2026, where the Deeper sonar's link died at 21:55:00 and
 * never recovered, and NOTHING on the boat recorded it: the Wi-Fi handlers in
 * main.c log to the serial console only, which is gone at power-down. The
 * cause had to be inferred from the phone's log and from reading the source.
 *
 * The heap line exists for the same reason - the owner's first question was
 * whether memory had been exhausted, and there was no way to answer it.
 * esp_get_free_heap_size() alone would NOT have answered it either: it counts
 * PSRAM, and Wi-Fi allocates from internal DMA-capable RAM, so internal free
 * and the largest internal block are the numbers that actually matter.
 */

#include <stdbool.h>
#include <stdint.h>

/** Create the queue. Safe to call more than once. Call before Wi-Fi starts. */
void db_netlog_init(void);

/**
 * Queue one breadcrumb. NEVER blocks and never touches the filesystem, so it
 * is safe from the Wi-Fi event handler. Format a single space-separated run of
 * key=value pairs, e.g. "event=wifi_sta_disconnect reason=%d".
 * Overflow is counted and reported on the next drained line rather than
 * blocking or silently vanishing.
 */
void db_netlog_note(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * Drain queued breadcrumbs into SYSTEM.LOG and emit the periodic heap line.
 * Must be called from an ordinary task - it can block on the logging mutex.
 */
void db_netlog_tick(void);

#endif /* DBB_NETLOG_H */
