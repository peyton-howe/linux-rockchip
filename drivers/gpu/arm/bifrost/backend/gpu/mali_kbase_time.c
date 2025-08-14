// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2014-2024 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <mali_kbase.h>
#include <mali_kbase_hwaccess_time.h>
#if MALI_USE_CSF
#include <linux/gcd.h>
#include <csf/mali_kbase_csf_timeout.h>
#endif
#include <device/mali_kbase_device.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <mali_kbase_config_defaults.h>
#include <linux/version_compat_defs.h>
#include <asm/arch_timer.h>
#include <linux/mali_hw_access.h>

struct kbase_timeout_info {
	char *selector_str;
	u64 timeout_cycles;
};

#if MALI_USE_CSF

#define GPU_TIMESTAMP_OFFSET_INVALID S64_MAX

static struct kbase_timeout_info timeout_info[KBASE_TIMEOUT_SELECTOR_COUNT] = {
	[CSF_FIRMWARE_TIMEOUT] = { "CSF_FIRMWARE_TIMEOUT", MIN(CSF_FIRMWARE_TIMEOUT_CYCLES,
							       CSF_FIRMWARE_PING_TIMEOUT_CYCLES) },
	[CSF_PM_TIMEOUT] = { "CSF_PM_TIMEOUT", CSF_PM_TIMEOUT_CYCLES },
	[CSF_GPU_RESET_TIMEOUT] = { "CSF_GPU_RESET_TIMEOUT", CSF_GPU_RESET_TIMEOUT_CYCLES },
	[CSF_CSG_TERM_TIMEOUT] = { "CSF_CSG_TERM_TIMEOUT", CSF_CSG_TERM_TIMEOUT_CYCLES },
	[CSF_FIRMWARE_BOOT_TIMEOUT] = { "CSF_FIRMWARE_BOOT_TIMEOUT",
					CSF_FIRMWARE_BOOT_TIMEOUT_CYCLES },
	[CSF_FIRMWARE_PING_TIMEOUT] = { "CSF_FIRMWARE_PING_TIMEOUT",
					CSF_FIRMWARE_PING_TIMEOUT_CYCLES },
	[CSF_SCHED_PROTM_PROGRESS_TIMEOUT] = { "CSF_SCHED_PROTM_PROGRESS_TIMEOUT",
					       DEFAULT_PROGRESS_TIMEOUT_CYCLES },
	[MMU_AS_INACTIVE_WAIT_TIMEOUT] = { "MMU_AS_INACTIVE_WAIT_TIMEOUT",
					   MMU_AS_INACTIVE_WAIT_TIMEOUT_CYCLES },
	[KCPU_FENCE_SIGNAL_TIMEOUT] = { "KCPU_FENCE_SIGNAL_TIMEOUT",
					KCPU_FENCE_SIGNAL_TIMEOUT_CYCLES },
	[KBASE_PRFCNT_ACTIVE_TIMEOUT] = { "KBASE_PRFCNT_ACTIVE_TIMEOUT",
					  KBASE_PRFCNT_ACTIVE_TIMEOUT_CYCLES },
	[KBASE_CLEAN_CACHE_TIMEOUT] = { "KBASE_CLEAN_CACHE_TIMEOUT",
					KBASE_CLEAN_CACHE_TIMEOUT_CYCLES },
	[KBASE_AS_INACTIVE_TIMEOUT] = { "KBASE_AS_INACTIVE_TIMEOUT",
					KBASE_AS_INACTIVE_TIMEOUT_CYCLES },
	[IPA_INACTIVE_TIMEOUT] = { "IPA_INACTIVE_TIMEOUT", IPA_INACTIVE_TIMEOUT_CYCLES },
	[CSF_FIRMWARE_STOP_TIMEOUT] = { "CSF_FIRMWARE_STOP_TIMEOUT",
					CSF_FIRMWARE_STOP_TIMEOUT_CYCLES },
};
#else
static struct kbase_timeout_info timeout_info[KBASE_TIMEOUT_SELECTOR_COUNT] = {
	[MMU_AS_INACTIVE_WAIT_TIMEOUT] = { "MMU_AS_INACTIVE_WAIT_TIMEOUT",
					   MMU_AS_INACTIVE_WAIT_TIMEOUT_CYCLES },
	[JM_DEFAULT_JS_FREE_TIMEOUT] = { "JM_DEFAULT_JS_FREE_TIMEOUT",
					 JM_DEFAULT_JS_FREE_TIMEOUT_CYCLES },
	[KBASE_PRFCNT_ACTIVE_TIMEOUT] = { "KBASE_PRFCNT_ACTIVE_TIMEOUT",
					  KBASE_PRFCNT_ACTIVE_TIMEOUT_CYCLES },
	[KBASE_CLEAN_CACHE_TIMEOUT] = { "KBASE_CLEAN_CACHE_TIMEOUT",
					KBASE_CLEAN_CACHE_TIMEOUT_CYCLES },
	[KBASE_AS_INACTIVE_TIMEOUT] = { "KBASE_AS_INACTIVE_TIMEOUT",
					KBASE_AS_INACTIVE_TIMEOUT_CYCLES },
};
#endif

#if MALI_USE_CSF
void kbase_backend_invalidate_gpu_timestamp_offset(struct kbase_device *kbdev)
{
	kbdev->backend_time.gpu_timestamp_offset = GPU_TIMESTAMP_OFFSET_INVALID;
}
KBASE_EXPORT_TEST_API(kbase_backend_invalidate_gpu_timestamp_offset);

/**
 * kbase_backend_compute_gpu_ts_offset() - Compute GPU TS offset.
 *
 * @kbdev:	Kbase device.
 *
 * This function compute the value of GPU and CPU TS offset:
 *   - set to zero current TIMESTAMP_OFFSET register
 *   - read CPU TS and convert it to ticks
 *   - read GPU TS
 *   - calculate diff between CPU and GPU ticks
 *   - cache the diff as the GPU TS offset
 *
 * To reduce delays, preemption must be disabled during reads of both CPU and GPU TS
 * this function require access to GPU register to be enabled
 */
static inline void kbase_backend_compute_gpu_ts_offset(struct kbase_device *kbdev)
{
	s64 cpu_ts_ticks = 0;
	s64 gpu_ts_ticks = 0;

	if (kbdev->backend_time.gpu_timestamp_offset != GPU_TIMESTAMP_OFFSET_INVALID)
		return;

	kbase_reg_write64(kbdev, GPU_CONTROL_ENUM(TIMESTAMP_OFFSET), 0);

	gpu_ts_ticks = kbase_reg_read64_coherent(kbdev, GPU_CONTROL_ENUM(TIMESTAMP));
	cpu_ts_ticks = ktime_get_raw_ns();
	cpu_ts_ticks = div64_u64(cpu_ts_ticks * kbdev->backend_time.divisor,
				 kbdev->backend_time.multiplier);
	kbdev->backend_time.gpu_timestamp_offset = cpu_ts_ticks - gpu_ts_ticks;
}

void kbase_backend_update_gpu_timestamp_offset(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->pm.lock);

	kbase_backend_compute_gpu_ts_offset(kbdev);

	dev_dbg(kbdev->dev, "Setting GPU timestamp offset register to %lld (%lld ns)",
		kbdev->backend_time.gpu_timestamp_offset,
		div64_s64(kbdev->backend_time.gpu_timestamp_offset *
				  (s64)kbdev->backend_time.multiplier,
			  (s64)kbdev->backend_time.divisor));
	kbase_reg_write64(kbdev, GPU_CONTROL_ENUM(TIMESTAMP_OFFSET),
			  kbdev->backend_time.gpu_timestamp_offset);
}
#if MALI_UNIT_TEST
u64 kbase_backend_read_gpu_timestamp_offset_reg(struct kbase_device *kbdev)
{
	return kbase_reg_read64_coherent(kbdev, GPU_CONTROL_ENUM(TIMESTAMP_OFFSET));
}
KBASE_EXPORT_TEST_API(kbase_backend_read_gpu_timestamp_offset_reg);
#endif
#endif

void kbase_backend_get_gpu_time_norequest(struct kbase_device *kbdev, u64 *cycle_counter,
					  u64 *system_time, struct timespec64 *ts)
{
	u32 hi1, hi2;

	if (cycle_counter)
		*cycle_counter = kbase_backend_get_cycle_cnt(kbdev);

	if (system_time) {
		/* Read hi, lo, hi to ensure a coherent u64 */
		do {
			hi1 = kbase_reg_read(kbdev,
					     GPU_CONTROL_REG(TIMESTAMP_HI));
			*system_time = kbase_reg_read(kbdev,
					     GPU_CONTROL_REG(TIMESTAMP_LO));
			hi2 = kbase_reg_read(kbdev,
					     GPU_CONTROL_REG(TIMESTAMP_HI));
		} while (hi1 != hi2);
		*system_time |= (((u64) hi1) << 32);
	}

	/* Record the CPU's idea of current time */
	if (ts != NULL)
#if (KERNEL_VERSION(4, 17, 0) > LINUX_VERSION_CODE)
		*ts = ktime_to_timespec64(ktime_get_raw());
#else
		ktime_get_raw_ts64(ts);
#endif
}
KBASE_EXPORT_TEST_API(kbase_backend_get_gpu_time_norequest);

#if !MALI_USE_CSF
/**
 * timedwait_cycle_count_active() - Timed wait till CYCLE_COUNT_ACTIVE is active
 *
 * @kbdev: Kbase device
 *
 * Return: true if CYCLE_COUNT_ACTIVE is active within the timeout.
 */
static bool timedwait_cycle_count_active(struct kbase_device *kbdev)
{
#if IS_ENABLED(CONFIG_MALI_BIFROST_NO_MALI)
	return true;
#else
	bool success = false;
	const unsigned int timeout = 100;
	const unsigned long remaining = jiffies + msecs_to_jiffies(timeout);

	while (time_is_after_jiffies(remaining)) {
		if ((kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_STATUS)) &
		     GPU_STATUS_CYCLE_COUNT_ACTIVE)) {
			success = true;
			break;
		}
	}
	return success;
#endif
}
#endif

void kbase_backend_get_gpu_time(struct kbase_device *kbdev, u64 *cycle_counter,
				u64 *system_time, struct timespec64 *ts)
{
#if !MALI_USE_CSF
	kbase_pm_request_gpu_cycle_counter(kbdev);
	WARN_ONCE(kbdev->pm.backend.l2_state != KBASE_L2_ON,
		  "L2 not powered up");
	WARN_ONCE((!timedwait_cycle_count_active(kbdev)),
		  "Timed out on CYCLE_COUNT_ACTIVE");
#endif
	kbase_backend_get_gpu_time_norequest(kbdev, cycle_counter, system_time,
					     ts);
#if !MALI_USE_CSF
	kbase_pm_release_gpu_cycle_counter(kbdev);
#endif
}
KBASE_EXPORT_TEST_API(kbase_backend_get_gpu_time);

unsigned int kbase_get_timeout_ms(struct kbase_device *kbdev,
				  enum kbase_timeout_selector selector)
{
	u64 freq_khz = kbdev->lowest_gpu_freq_khz;

	if (!freq_khz) {
		dev_dbg(kbdev->dev,
			"Lowest frequency uninitialized! Using reference frequency for scaling");
		return DEFAULT_REF_TIMEOUT_FREQ_KHZ;
	}

	return freq_khz;
}

void kbase_device_set_timeout_ms(struct kbase_device *kbdev, enum kbase_timeout_selector selector,
				 unsigned int timeout_ms)
{
	char *selector_str;

	if (unlikely(selector >= KBASE_TIMEOUT_SELECTOR_COUNT)) {
		selector = KBASE_DEFAULT_TIMEOUT;
		selector_str = timeout_info[selector].selector_str;
		dev_warn(kbdev->dev,
			 "Unknown timeout selector passed, falling back to default: %s\n",
			 timeout_info[selector].selector_str);
	}
	selector_str = timeout_info[selector].selector_str;

#if MALI_USE_CSF
	if (IS_ENABLED(CONFIG_MALI_REAL_HW) && !IS_ENABLED(CONFIG_MALI_IS_FPGA) &&
	    unlikely(timeout_ms >= MAX_TIMEOUT_MS)) {
		dev_warn(kbdev->dev, "%s is capped from %dms to %dms\n",
			 timeout_info[selector].selector_str, timeout_ms, MAX_TIMEOUT_MS);
		timeout_ms = MAX_TIMEOUT_MS;
	}
#endif

	kbdev->backend_time.device_scaled_timeouts[selector] = timeout_ms;
	dev_dbg(kbdev->dev, "\t%-35s: %ums\n", selector_str, timeout_ms);
}

void kbase_device_set_timeout(struct kbase_device *kbdev, enum kbase_timeout_selector selector,
			      u64 timeout_cycles, u32 cycle_multiplier)
{
	u64 final_cycles;
	u64 timeout;
	u64 freq_khz = kbase_device_get_scaling_frequency(kbdev);

	if (unlikely(selector >= KBASE_TIMEOUT_SELECTOR_COUNT)) {
		selector = KBASE_DEFAULT_TIMEOUT;
		dev_warn(kbdev->dev,
			 "Unknown timeout selector passed, falling back to default: %s\n",
			 timeout_info[selector].selector_str);
	}

	/* If the multiplication overflows, we will have unsigned wrap-around, and so might
	 * end up with a shorter timeout. In those cases, we then want to have the largest
	 * timeout possible that will not run into these issues. Note that this will not
	 * wait for U64_MAX/frequency ms, as it will be clamped to a max of UINT_MAX
	 * milliseconds by subsequent steps.
	 */
	if (check_mul_overflow(timeout_cycles, (u64)cycle_multiplier, &final_cycles))
		final_cycles = U64_MAX;

	/* Timeout calculation:
	 * dividing number of cycles by freq in KHz automatically gives value
	 * in milliseconds. nr_cycles will have to be multiplied by 1e3 to
	 * get result in microseconds, and 1e6 to get result in nanoseconds.
	 */

	u64 timeout, nr_cycles = 0;
	u64 freq_khz;

	/* Only for debug messages, safe default in case it's mis-maintained */
	const char *selector_str = "(unknown)";

	if (!kbdev->lowest_gpu_freq_khz) {
		dev_dbg(kbdev->dev,
			"Lowest frequency uninitialized! Using reference frequency for scaling");
		freq_khz = DEFAULT_REF_TIMEOUT_FREQ_KHZ;
	} else {
		freq_khz = kbdev->lowest_gpu_freq_khz;
	}

	switch (selector) {
	case MMU_AS_INACTIVE_WAIT_TIMEOUT:
		selector_str = "MMU_AS_INACTIVE_WAIT_TIMEOUT";
		nr_cycles = MMU_AS_INACTIVE_WAIT_TIMEOUT_CYCLES;
		break;
	case KBASE_TIMEOUT_SELECTOR_COUNT:
	default:
#if !MALI_USE_CSF
		WARN(1, "Invalid timeout selector used! Using default value");
		nr_cycles = JM_DEFAULT_TIMEOUT_CYCLES;
		break;
	case JM_DEFAULT_JS_FREE_TIMEOUT:
		selector_str = "JM_DEFAULT_JS_FREE_TIMEOUT";
		nr_cycles = JM_DEFAULT_JS_FREE_TIMEOUT_CYCLES;
		break;
#else
		/* Use Firmware timeout if invalid selection */
		WARN(1,
		     "Invalid timeout selector used! Using CSF Firmware timeout");
		fallthrough;
	case CSF_FIRMWARE_TIMEOUT:
		selector_str = "CSF_FIRMWARE_TIMEOUT";
		/* Any FW timeout cannot be longer than the FW ping interval, after which
		 * the firmware_aliveness_monitor will be triggered and may restart
		 * the GPU if the FW is unresponsive.
		 */
		nr_cycles = min(CSF_FIRMWARE_PING_TIMEOUT_CYCLES, CSF_FIRMWARE_TIMEOUT_CYCLES);

		if (nr_cycles == CSF_FIRMWARE_PING_TIMEOUT_CYCLES)
			dev_warn(kbdev->dev, "Capping %s to CSF_FIRMWARE_PING_TIMEOUT\n",
				 selector_str);
		break;
	case CSF_PM_TIMEOUT:
		selector_str = "CSF_PM_TIMEOUT";
		nr_cycles = CSF_PM_TIMEOUT_CYCLES;
		break;
	case CSF_GPU_RESET_TIMEOUT:
		selector_str = "CSF_GPU_RESET_TIMEOUT";
		nr_cycles = CSF_GPU_RESET_TIMEOUT_CYCLES;
		break;
	case CSF_CSG_SUSPEND_TIMEOUT:
		selector_str = "CSF_CSG_SUSPEND_TIMEOUT";
		nr_cycles = CSF_CSG_SUSPEND_TIMEOUT_CYCLES;
		break;
	case CSF_FIRMWARE_BOOT_TIMEOUT:
		selector_str = "CSF_FIRMWARE_BOOT_TIMEOUT";
		nr_cycles = CSF_FIRMWARE_BOOT_TIMEOUT_CYCLES;
		break;
	case CSF_FIRMWARE_PING_TIMEOUT:
		selector_str = "CSF_FIRMWARE_PING_TIMEOUT";
		nr_cycles = CSF_FIRMWARE_PING_TIMEOUT_CYCLES;
		break;
	case CSF_SCHED_PROTM_PROGRESS_TIMEOUT:
		selector_str = "CSF_SCHED_PROTM_PROGRESS_TIMEOUT";
		nr_cycles = kbase_csf_timeout_get(kbdev);
		break;
#endif
	}

	timeout = div_u64(nr_cycles, freq_khz);
	if (WARN(timeout > UINT_MAX,
		 "Capping excessive timeout %llums for %s at freq %llukHz to UINT_MAX ms",
		 (unsigned long long)timeout, selector_str, (unsigned long long)freq_khz))
		timeout = UINT_MAX;
	return (unsigned int)timeout;
}
KBASE_EXPORT_TEST_API(kbase_get_timeout_ms);

u64 kbase_backend_get_cycle_cnt(struct kbase_device *kbdev)
{
	u32 hi1, hi2, lo;

	/* Read hi, lo, hi to ensure a coherent u64 */
	do {
		hi1 = kbase_reg_read(kbdev,
					GPU_CONTROL_REG(CYCLE_COUNT_HI));
		lo = kbase_reg_read(kbdev,
					GPU_CONTROL_REG(CYCLE_COUNT_LO));
		hi2 = kbase_reg_read(kbdev,
					GPU_CONTROL_REG(CYCLE_COUNT_HI));
	} while (hi1 != hi2);

	return lo | (((u64) hi1) << 32);
}

#if MALI_USE_CSF
u64 __maybe_unused kbase_backend_time_convert_gpu_to_cpu(struct kbase_device *kbdev, u64 gpu_ts)
{
	if (WARN_ON(!kbdev))
		return 0;

	return div64_u64(gpu_ts * kbdev->backend_time.multiplier, kbdev->backend_time.divisor);
}
KBASE_EXPORT_TEST_API(kbase_backend_time_convert_gpu_to_cpu);
#endif

u64 kbase_arch_timer_get_cntfrq(struct kbase_device *kbdev)
{
	u64 freq = mali_arch_timer_get_cntfrq();

	dev_dbg(kbdev->dev, "System Timer Freq = %lluHz", freq);

	return freq;
}

int kbase_backend_time_init(struct kbase_device *kbdev)
{
#if MALI_USE_CSF
	u64 freq;
	u64 common_factor;

	kbase_pm_register_access_enable(kbdev);
	freq = kbase_arch_timer_get_cntfrq(kbdev);

	if (!freq) {
		dev_warn(kbdev->dev, "arch_timer_get_rate() is zero!");
		return -EINVAL;
	}

	common_factor = gcd(NSEC_PER_SEC, freq);

	kbdev->backend_time.multiplier = div64_u64(NSEC_PER_SEC, common_factor);
	kbdev->backend_time.divisor = div64_u64(freq, common_factor);

	if (!kbdev->backend_time.divisor) {
		dev_warn(kbdev->dev, "CPU to GPU divisor is zero!");
		return -EINVAL;
	}

	kbase_backend_invalidate_gpu_timestamp_offset(
		kbdev); /* force computation of GPU Timestamp offset */
#endif

	return 0;
}
