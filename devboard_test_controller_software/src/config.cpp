#include "config.h"

static Preferences prefs;

void configLoad(AppConfig &cfg) {
    cfg = defaultConfig();
    prefs.begin("ebike", true); // read-only

    cfg.assist_ratio      = prefs.getFloat("assist",    cfg.assist_ratio);
    cfg.max_motor_current = prefs.getFloat("maxcur",    cfg.max_motor_current);
    cfg.min_cadence_rpm   = prefs.getFloat("mincad",    cfg.min_cadence_rpm);
    cfg.current_ramp_rate = prefs.getFloat("ramp",      cfg.current_ramp_rate);
    cfg.assist_start_nm = prefs.getFloat("astartnm", cfg.assist_start_nm);
    cfg.assist_full_nm  = prefs.getFloat("afullnm",  cfg.assist_full_nm);

    cfg.torque_min_v   = prefs.getFloat("tqminv",  cfg.torque_min_v);
    cfg.torque_max_v   = prefs.getFloat("tqmaxv",  cfg.torque_max_v);
    cfg.torque_max_nm  = prefs.getFloat("tqmaxnm", cfg.torque_max_nm);
    cfg.divider_ratio  = prefs.getFloat("divr",    cfg.divider_ratio);
    cfg.torque_ema_alpha = prefs.getFloat("emaa",   cfg.torque_ema_alpha);
    cfg.cadence_ppr    = prefs.getUShort("ppr",     cfg.cadence_ppr);
    cfg.flux_linkage   = prefs.getFloat("flnk",    cfg.flux_linkage);
    cfg.vesc_baud      = prefs.getULong("vescbaud", cfg.vesc_baud);

    String ssid = prefs.getString("ssid", cfg.wifi_ssid);
    String pass = prefs.getString("wpass", cfg.wifi_pass);
    String ap   = prefs.getString("apass", cfg.ap_pass);
    strlcpy(cfg.wifi_ssid, ssid.c_str(), sizeof(cfg.wifi_ssid));
    strlcpy(cfg.wifi_pass, pass.c_str(), sizeof(cfg.wifi_pass));
    strlcpy(cfg.ap_pass,   ap.c_str(),   sizeof(cfg.ap_pass));

    cfg.ble_bridge_enabled = prefs.getBool("ble", cfg.ble_bridge_enabled);
    cfg.torque_assist_enabled = prefs.getBool("tqassist", cfg.torque_assist_enabled);

    prefs.end();
}

void configSave(const AppConfig &cfg) {
    prefs.begin("ebike", false); // read-write

    prefs.putFloat("assist",    cfg.assist_ratio);
    prefs.putFloat("maxcur",    cfg.max_motor_current);
    prefs.putFloat("mincad",    cfg.min_cadence_rpm);
    prefs.putFloat("ramp",      cfg.current_ramp_rate);
    prefs.putFloat("astartnm",  cfg.assist_start_nm);
    prefs.putFloat("afullnm",   cfg.assist_full_nm);

    prefs.putFloat("tqminv",    cfg.torque_min_v);
    prefs.putFloat("tqmaxv",    cfg.torque_max_v);
    prefs.putFloat("tqmaxnm",  cfg.torque_max_nm);
    prefs.putFloat("divr",      cfg.divider_ratio);
    prefs.putFloat("emaa",      cfg.torque_ema_alpha);
    prefs.putUShort("ppr",      cfg.cadence_ppr);
    prefs.putFloat("flnk",      cfg.flux_linkage);
    prefs.putULong("vescbaud",  cfg.vesc_baud);

    prefs.putString("ssid",     cfg.wifi_ssid);
    prefs.putString("wpass",    cfg.wifi_pass);
    prefs.putString("apass",    cfg.ap_pass);

    prefs.putBool("ble",        cfg.ble_bridge_enabled);
    prefs.putBool("tqassist",   cfg.torque_assist_enabled);

    prefs.end();
}
