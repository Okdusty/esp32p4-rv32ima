/* SPDX-License-Identifier: BSD-3-Clause */
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "sdkconfig.h"
#include "soc/cache_reg.h"
#include "soc/soc.h"
#include "host_cache_profile.h"
#include "port.h"
#include "virtio_blk_bridge.h"

#if !CONFIG_RV32_CACHE_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
#error "Hardware cache sampling requires exclusive counter ownership"
#endif

#define L1_ENABLE (CACHE_L1_IBUS0_CNT_ENA | CACHE_L1_IBUS1_CNT_ENA | \
		   CACHE_L1_DBUS0_CNT_ENA | CACHE_L1_DBUS1_CNT_ENA)
#define L2_ENABLE (CACHE_L2_IBUS0_CNT_ENA | CACHE_L2_IBUS1_CNT_ENA | \
		   CACHE_L2_DBUS0_CNT_ENA | CACHE_L2_DBUS1_CNT_ENA)
#define L1_CLEAR (CACHE_L1_IBUS0_CNT_CLR | CACHE_L1_IBUS1_CNT_CLR | \
		  CACHE_L1_DBUS0_CNT_CLR | CACHE_L1_DBUS1_CNT_CLR)
#define L2_CLEAR (CACHE_L2_IBUS0_CNT_CLR | CACHE_L2_IBUS1_CNT_CLR | \
		  CACHE_L2_DBUS0_CNT_CLR | CACHE_L2_DBUS1_CNT_CLR)

struct cache_bus_sample {
	uint32_t hits;
	uint32_t misses;
	uint32_t conflicts;
	uint32_t reads;
	uint32_t writes;
};

/* Small fixed storage avoids adding printf/snapshot pressure to app_main's
 * stack. This module is normally cached code, not another noflash object. */
static struct cache_bus_sample samples[2][4];
static struct virtio_blk_bridge_perf_stats block_previous;
static uint64_t window_start;
static uint64_t previous_report_us;

static void counters_stop(void)
{
	/* Only counter enables change. Freeze both levels before reading any
	 * values; the two register writes are not a single atomic snapshot. */
	REG_CLR_BIT(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, L1_ENABLE);
	REG_CLR_BIT(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, L2_ENABLE);
}

static void counters_start(void)
{
	/* CLR fields are write-trigger bits in the matching ESP-IDF register
	 * definitions. Preserve other, unowned fields in these registers. */
	REG_SET_BIT(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, L1_CLEAR);
	REG_SET_BIT(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, L2_CLEAR);
	REG_SET_BIT(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, L1_ENABLE);
	REG_SET_BIT(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, L2_ENABLE);
	window_start = GetTimeMicroseconds();
}

void HostCacheProfileStart(void)
{
	counters_stop();
	previous_report_us = 0u;
	virtio_blk_bridge_perf_snapshot(&block_previous);
	counters_start();
}

static void counters_read(void)
{
	/* Use each documented register by name: instruction buses do not have
	 * write-back counters and data-bus register strides differ from I-bus. */
#define READ_BUS(level, bus, index) do { \
	struct cache_bus_sample *s = &samples[(level) - 1u][index]; \
	s->hits = REG_READ(CACHE_L##level##_##bus##_ACS_HIT_CNT_REG); \
	s->misses = REG_READ(CACHE_L##level##_##bus##_ACS_MISS_CNT_REG); \
	s->conflicts = REG_READ(CACHE_L##level##_##bus##_ACS_CONFLICT_CNT_REG); \
	s->reads = REG_READ(CACHE_L##level##_##bus##_ACS_NXTLVL_RD_CNT_REG); \
	s->writes = 0u; \
} while (0)
#define READ_DATA_BUS(level, bus, index) do { \
	READ_BUS(level, bus, index); \
	samples[(level) - 1u][index].writes = \
		REG_READ(CACHE_L##level##_##bus##_ACS_NXTLVL_WR_CNT_REG); \
} while (0)
	READ_BUS(1, IBUS0, 0);
	READ_BUS(1, IBUS1, 1);
	READ_DATA_BUS(1, DBUS0, 2);
	READ_DATA_BUS(1, DBUS1, 3);
	READ_BUS(2, IBUS0, 0);
	READ_BUS(2, IBUS1, 1);
	READ_DATA_BUS(2, DBUS0, 2);
	READ_DATA_BUS(2, DBUS1, 3);
#undef READ_DATA_BUS
#undef READ_BUS
}

void HostCacheProfileReport(void)
{
	static const char *const buses[] = { "I0", "I1", "D0", "D1" };
	const uint64_t entered = GetTimeMicroseconds();
	uint64_t window_end;
	struct virtio_blk_bridge_perf_stats block;

	counters_stop();
	window_end = GetTimeMicroseconds();
	counters_read();
	virtio_blk_bridge_perf_snapshot(&block);
	printf("\n[Cache perf] start_us=%" PRIu64 " end_us=%" PRIu64
	       " span_us=%" PRIu64 " previous_report_us=%" PRIu64
	       " late=%u; fields=hit,miss,conflict,next-r,next-w\n",
	       window_start, window_end, window_end - window_start,
	       previous_report_us,
	       (unsigned int)(window_end - window_start > 2000000u));
	for (unsigned int bus = 0; bus < 4u; bus++) {
		const struct cache_bus_sample *l1 = &samples[0][bus];
		const struct cache_bus_sample *l2 = &samples[1][bus];

		printf("[Cache perf] %s L1=%" PRIu32 ",%" PRIu32 ",%" PRIu32
		       ",%" PRIu32 ",%" PRIu32 " L2=%" PRIu32 ",%" PRIu32
		       ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", buses[bus],
		       l1->hits, l1->misses, l1->conflicts, l1->reads, l1->writes,
		       l2->hits, l2->misses, l2->conflicts, l2->reads, l2->writes);
	}
	printf("[Cache perf] SD request=%" PRIu32 " hit=%" PRIu32
	       " miss=%" PRIu32 " read_bytes=%" PRIu32
	       " write_bytes=%" PRIu32 "\n",
	       block.requests - block_previous.requests,
	       block.cache_hits - block_previous.cache_hits,
	       block.cache_misses - block_previous.cache_misses,
	       block.read_bytes - block_previous.read_bytes,
	       block.write_bytes - block_previous.write_bytes);
	/* SD snapshots are independent atomic fields, not a transactional device
	 * snapshot. Rebase after reporting so its interval approximately matches
	 * the next cache window. The 32-bit hardware counters can overflow if a
	 * sampling gap is unusually long; retain raw span/late data in the log. */
	virtio_blk_bridge_perf_snapshot(&block_previous);
	counters_start();
	previous_report_us = GetTimeMicroseconds() - entered;
}
