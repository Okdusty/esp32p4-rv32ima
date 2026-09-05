/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef HOST_CACHE_PROFILE_H
#define HOST_CACHE_PROFILE_H

/* One owner on the emulator task; separate from per-instruction profiling.
 * Start may be called again after a guest reboot. Neither function changes
 * cache capacity, line size, prefetch, contents, or interrupt configuration. */
void HostCacheProfileStart(void);
void HostCacheProfileReport(void);

#endif
