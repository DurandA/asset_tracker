/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <sys/util.h>
#include <drivers/gps.h>
#include <modem/lte_lc.h>

#include "ui.h"
#include "gps_controller.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(gps_control, CONFIG_ASSET_TRACKER_LOG_LEVEL);

/* Structure to hold GPS work information */
static struct device *gps_dev;
static atomic_t gps_is_enabled;
static atomic_t gps_is_active;
static struct k_work_q *app_work_q;
static struct k_delayed_work start_work;
static struct k_delayed_work stop_work;
static int gps_reporting_interval_seconds;

#ifdef CONFIG_SUPL_CLIENT_LIB
static enum gps_agps_type type_lookup_socket2gps[] = {
	[NRF_GNSS_AGPS_UTC_PARAMETERS]	= GPS_AGPS_UTC_PARAMETERS,
	[NRF_GNSS_AGPS_EPHEMERIDES]	= GPS_AGPS_EPHEMERIDES,
	[NRF_GNSS_AGPS_ALMANAC]		= GPS_AGPS_ALMANAC,
	[NRF_GNSS_AGPS_KLOBUCHAR_IONOSPHERIC_CORRECTION]
					= GPS_AGPS_KLOBUCHAR_CORRECTION,
	[NRF_GNSS_AGPS_NEQUICK_IONOSPHERIC_CORRECTION]
					= GPS_AGPS_NEQUICK_CORRECTION,
	[NRF_GNSS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS]
					= GPS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS,
	[NRF_GNSS_AGPS_LOCATION]	= GPS_AGPS_LOCATION,
	[NRF_GNSS_AGPS_INTEGRITY]	= GPS_AGPS_INTEGRITY,
};
#endif /* CONFIG_SUPL_CLIENT_LIB */

static void start(struct k_work *work)
{
	ARG_UNUSED(work);
	int err;
	struct gps_config gps_cfg = {
		.nav_mode = GPS_NAV_MODE_PERIODIC,
		.power_mode = GPS_POWER_MODE_DISABLED,
		.timeout = CONFIG_GPS_CONTROL_FIX_TRY_TIME,
		.interval = CONFIG_GPS_CONTROL_FIX_TRY_TIME +
			gps_reporting_interval_seconds,
		.priority = true,
	};

	if (gps_dev == NULL) {
		LOG_ERR("GPS controller is not initialized properly");
		return;
	}

#ifdef CONFIG_GPS_CONTROL_PSM_ENABLE_ON_START
	LOG_INF("Enabling PSM");

	err = lte_lc_psm_req(true);
	if (err) {
		LOG_ERR("PSM request failed, error: %d", err);
	} else {
		LOG_INF("PSM enabled");
	}
#endif /* CONFIG_GPS_CONTROL_PSM_ENABLE_ON_START */

	err = gps_start(gps_dev, &gps_cfg);
	if (err) {
		LOG_ERR("Failed to enable GPS, error: %d", err);
		return;
	}

	atomic_set(&gps_is_enabled, 1);
	gps_control_set_active(true);
	ui_led_set_pattern(UI_LED_GPS_SEARCHING);

	LOG_INF("GPS started successfully. Searching for satellites ");
	LOG_INF("to get position fix. This may take several minutes.");
	LOG_INF("The device will attempt to get a fix for %d seconds, ",
		CONFIG_GPS_CONTROL_FIX_TRY_TIME);
	LOG_INF("before the GPS is stopped. It's restarted every %d seconds",
		gps_reporting_interval_seconds);
#if !defined(CONFIG_GPS_SIM)
#if IS_ENABLED(CONFIG_GPS_START_ON_MOTION)
	LOG_INF("or as soon as %d seconds later when movement occurs.",
		CONFIG_GPS_CONTROL_FIX_CHECK_INTERVAL);
#endif
#endif
}

static void stop(struct k_work *work)
{
	ARG_UNUSED(work);
	int err;

	if (gps_dev == NULL) {
		LOG_ERR("GPS controller is not initialized");
		return;
	}

#ifdef CONFIG_GPS_CONTROL_PSM_DISABLE_ON_STOP
	LOG_INF("Disabling PSM");

	err = lte_lc_psm_req(false);
	if (err) {
		LOG_ERR("PSM mode could not be disabled, error: %d",
			err);
	}
#endif /* CONFIG_GPS_CONTROL_PSM_DISABLE_ON_STOP */

	err = gps_stop(gps_dev);
	if (err) {
		LOG_ERR("Failed to disable GPS, error: %d", err);
		return;
	}

	atomic_set(&gps_is_enabled, 0);
	gps_control_set_active(false);
	LOG_INF("GPS operation was stopped");
}

bool gps_control_is_enabled(void)
{
	return atomic_get(&gps_is_enabled);
}

bool gps_control_is_active(void)
{
	return atomic_get(&gps_is_active);
}

bool gps_control_set_active(bool active)
{
	return atomic_set(&gps_is_active, active ? 1 : 0);
}

void gps_control_start(uint32_t delay_ms)
{
	k_delayed_work_submit_to_queue(app_work_q, &start_work,
				       K_MSEC(delay_ms));
}

void gps_control_stop(uint32_t delay_ms)
{
	k_delayed_work_submit_to_queue(app_work_q, &stop_work,
				       K_MSEC(delay_ms));
}

int gps_control_get_gps_reporting_interval(void)
{
	return gps_reporting_interval_seconds;
}

/** @brief Configures and starts the GPS device. */
int gps_control_init(struct k_work_q *work_q, gps_event_handler_t handler)
{
	int err;
	static bool is_init;

	if (is_init) {
		return -EALREADY;
	}

	if ((work_q == NULL) || (handler == NULL)) {
		return -EINVAL;
	}

	app_work_q = work_q;

	gps_dev = device_get_binding(CONFIG_GPS_DEV_NAME);
	if (gps_dev == NULL) {
		LOG_ERR("Could not get %s device",
			log_strdup(CONFIG_GPS_DEV_NAME));
		return -ENODEV;
	}

	err = gps_init(gps_dev, handler);
	if (err) {
		LOG_ERR("Could not initialize GPS, error: %d", err);
		return err;
	}

	k_delayed_work_init(&start_work, start);
	k_delayed_work_init(&stop_work, stop);

#if !defined(CONFIG_GPS_SIM)
	gps_reporting_interval_seconds =
		IS_ENABLED(CONFIG_GPS_START_ON_MOTION) ?
		CONFIG_GPS_CONTROL_FIX_CHECK_OVERDUE :
		CONFIG_GPS_CONTROL_FIX_CHECK_INTERVAL;
#else
	gps_reporting_interval_seconds = CONFIG_GPS_CONTROL_FIX_CHECK_INTERVAL;
#endif

	LOG_INF("GPS initialized");

	is_init = true;

	return err;
}

#ifdef CONFIG_SUPL_CLIENT_LIB
static inline enum gps_agps_type type_socket2gps(
	nrf_gnss_agps_data_type_t type)
{
	return type_lookup_socket2gps[type];
}

int inject_agps_type(void *agps,
		     size_t agps_size,
		     nrf_gnss_agps_data_type_t type,
		     void *user_data)
{
	ARG_UNUSED(user_data);
	return gps_agps_write(gps_dev, type_socket2gps(type), agps,
				agps_size);
}
#endif /* CONFIG_SUPL_CLIENT_LIB */
