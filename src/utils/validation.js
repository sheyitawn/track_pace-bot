export function validateSplits(splits) {
    if (!splits.length) return "Please add at least one split.";
    const sorted = [...splits].sort((a, b) => a.start - b.start);
    for (let i = 0; i < sorted.length; i++) {
        const s = sorted[i];
        if (s.end <= s.start) return `Split ${i + 1}: end must be > start.`;
        if (s.time <= 0) return `Split ${i + 1}: time must be > 0.`;
        if (i > 0 && s.start !== sorted[i - 1].end) {
            return `Split ${i + 1}: start (${s.start}) must equal previous end (${sorted[i - 1].end}).`;
        }
    }
    const total = sorted[sorted.length - 1].end - sorted[0].start;
    if (total <= 0) return "Total distance must be positive.";
    return null;
}

export function nearestOdd135(n) {
  const v = Math.max(1, Math.min(5, Math.round(Number(n) || 1)));
  if (v <= 2) return 1;
  if (v <= 4) return 3;
  return 5;
}

function inRange(name, v, lo, hi, errors, msg) {
  const num = Number(v);
  if (!Number.isFinite(num) || num < lo || num > hi) {
    errors[name] = msg || `${name} must be in [${lo}, ${hi}]`;
  }
}

export function validateConfig(cfg) {
  const e = {};

  // booleans
  if (typeof cfg.white_line !== "boolean") {
    e.white_line = "white_line must be true or false";
  }

  // proportional gain
  inRange("Kp", cfg.Kp, 0, 2.5, e);

  // thresholds
  inRange("error_slowdown_thresh", cfg.error_slowdown_thresh, 0, 6500, e);
  inRange("lost_confidence_sum", cfg.lost_confidence_sum, 0, 14000, e);

  // microseconds ranges
  ["us_min","us_neu","us_max","steer_min_us","steer_max_us","throttle_min","throttle_base","throttle_max"]
    .forEach(k => inRange(k, cfg[k], 800, 2200, e));

  // structural relationships
  if (cfg.us_neu < cfg.us_min || cfg.us_neu > cfg.us_max)
    e.us_neu = "Neutral must lie between us_min and us_max";
  if (cfg.steer_min_us < cfg.us_min) e.steer_min_us = "steer_min_us >= us_min";
  if (cfg.steer_max_us > cfg.us_max) e.steer_max_us = "steer_max_us <= us_max";
  if (cfg.steer_min_us > cfg.steer_max_us) e.steer_min_us = "steer_min_us <= steer_max_us";
  if (cfg.throttle_min < cfg.us_min) e.throttle_min = "throttle_min >= us_min";
  if (cfg.throttle_max > cfg.us_max) e.throttle_max = "throttle_max <= us_max";
  if (cfg.throttle_base < cfg.throttle_min || cfg.throttle_base > cfg.throttle_max)
    e.throttle_base = "throttle_base between throttle_min and throttle_max";

  // timing
  inRange("esc_arm_ms", cfg.esc_arm_ms, 0, 10000, e);
  inRange("qtr_timeout_us", cfg.qtr_timeout_us, 500, 12000, e);
  inRange("calibrate_ms", cfg.calibrate_ms, 500, 10000, e);

  inRange("qtr_gamma", cfg.qtr_gamma, 0.3, 2.5, e);
  const ns = nearestOdd135(cfg.qtr_smooth_n);
  if (Number(cfg.qtr_smooth_n) !== ns) {
    e.qtr_smooth_n = "Must be 1, 3, or 5 (odd). It will auto-correct on blur.";
  }
  inRange("min_line_contrast", cfg.min_line_contrast, 20, 500, e);
  inRange("deadband_error", cfg.deadband_error, 0, 1500, e);

  return e;
}
