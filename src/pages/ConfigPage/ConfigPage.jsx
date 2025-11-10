import React, { useEffect, useState } from "react";
import { deviceApi } from "../../api/deviceApi";

function Field({ label, children }) {
  return (
    <>
      <label className="label">{label}</label>
      <div>{children}</div>
    </>
  );
}

// fallbacks 
const DEFAULTS = {
  qtr_gamma: 0.75,         // 0.3 .. 2.5
  qtr_smooth_n: 3,         // 1, 3, 5 (odd only)
  min_line_contrast: 120,  // 20 .. 500
  deadband_error: 300      // 0 .. 1500
};

function nearestOdd135(n) {
  const v = Math.max(1, Math.min(5, Math.round(Number(n) || 1)));
  if (v <= 2) return 1;
  if (v <= 4) return 3;
  return 5;
}

export default function ConfigPage() {
  const baseUrl = deviceApi.getBaseUrl();
  const [cfg, setCfg] = useState(null);
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState("");

  const load = async () => {
    if (!baseUrl) { setCfg(null); return; }
    setBusy(true);
    setMsg("");
    try {
      const c = await deviceApi.getConfig();
      setCfg({ ...DEFAULTS, ...c });
    } catch (e) {
      setMsg(e.message || String(e));
    } finally {
      setBusy(false);
    }
  };

  useEffect(() => { load(); }, []); 

  const save = async () => {
    if (!cfg) return;
    setBusy(true);
    setMsg("");
    try {
      const c = await deviceApi.updateConfig(cfg);
      setCfg({ ...DEFAULTS, ...c });
      setMsg("Saved.");
    } catch (e) {
      setMsg(e.message || String(e));
    } finally {
      setBusy(false);
    }
  };

  const reset = async () => {
    if (!window.confirm("Factory reset config?")) return;
    setBusy(true);
    setMsg("");
    try {
      const c = await deviceApi.resetConfig();
      setCfg({ ...DEFAULTS, ...c });
      setMsg("Config reset to defaults.");
    } catch (e) {
      setMsg(e.message || String(e));
    } finally {
      setBusy(false);
    }
  };

  const refresh = async () => {
    try {
      setBusy(true);
      setMsg("Refreshing connection…");
      if (cfg) await deviceApi.updateConfig(cfg);
      await deviceApi.refreshConnection();
      setMsg("Reconnected.");
    } catch (e) {
      setMsg(e.message || String(e));
    } finally {
      setBusy(false);
    }
  };

  const on = (k, v) => setCfg((old) => ({ ...old, [k]: v }));

  const toFloat = (e, k) => on(k, parseFloat(e.target.value || 0));
  const toInt   = (e, k) => on(k, parseInt(e.target.value || 0, 10));

  return (
    <div className="container">
      <header className="nav">
        <h1>Track Pace-Bot – Config</h1>
        <div className="row">
          <a href="/" className="badge">← Back</a>
        </div>
      </header>

      <div className="card" style={{ padding: 16 }}>
        {!baseUrl && (
          <div className="row" style={{ marginBottom: 12 }}>
            <span className="badge warn">Not connected</span>
            <span className="label">Connect on the Home page first.</span>
          </div>
        )}

        <div className="row" style={{ gap: 8, marginBottom: 8 }}>
          <button onClick={load} disabled={busy || !baseUrl}>Reload</button>
          <button className="primary" onClick={save} disabled={busy || !cfg}>Save</button>
          <button onClick={refresh} disabled={busy || !baseUrl}>Refresh (save + reconnect)</button>
          <button className="danger" onClick={reset} disabled={busy || !baseUrl}>Factory Reset</button>
          {busy && <span className="badge">Working…</span>}
          {!!msg && <span className="badge">{msg}</span>}
        </div>

        {!cfg ? (
          <div className="label">Load config to edit.</div>
        ) : (
          <>
            {/* Core line-following */}
            <div className="grid" style={{ gridTemplateColumns: "220px 1fr", gap: 12 }}>
              <Field label="White Line">
                <input
                  type="checkbox"
                  checked={!!cfg.white_line}
                  onChange={e => on("white_line", e.target.checked)}
                />
              </Field>

              <Field label="Kp (µs per 1000)">
                <input
                  type="number"
                  step="0.01"
                  value={cfg.Kp}
                  onChange={e => toFloat(e, "Kp")}
                />
              </Field>

              <Field label="Error Slowdown Thresh">
                <input
                  type="number"
                  value={cfg.error_slowdown_thresh}
                  onChange={e => toInt(e, "error_slowdown_thresh")}
                />
              </Field>

              <Field label="Lost Confidence Sum">
                <input
                  type="number"
                  value={cfg.lost_confidence_sum}
                  onChange={e => toInt(e, "lost_confidence_sum")}
                />
              </Field>
            </div>

            {/* NEW: QTR / Line Tracking section */}
            <div className="grid" style={{ gridTemplateColumns: "220px 1fr", gap: 12, marginTop: 12 }}>
              <Field label="QTR Gamma">
                <input
                  type="number"
                  step="0.01"
                  min="0.3"
                  max="2.5"
                  value={cfg.qtr_gamma}
                  onChange={e => toFloat(e, "qtr_gamma")}
                />
              </Field>

              <Field label="QTR Smooth N">
                <input
                  type="number"
                  step="1"
                  min="1"
                  max="5"
                  value={cfg.qtr_smooth_n}
                  onChange={e => toInt(e, "qtr_smooth_n")}
                  onBlur={() => on("qtr_smooth_n", nearestOdd135(cfg.qtr_smooth_n))}
                />
              </Field>

              <Field label="Min Line Contrast">
                <input
                  type="number"
                  step="1"
                  min="20"
                  max="500"
                  value={cfg.min_line_contrast}
                  onChange={e => toInt(e, "min_line_contrast")}
                />
              </Field>

              <Field label="Deadband Error">
                <input
                  type="number"
                  step="1"
                  min="0"
                  max="1500"
                  value={cfg.deadband_error}
                  onChange={e => toInt(e, "deadband_error")}
                />
              </Field>
            </div>

            {/* Servo / ESC */}
            <div className="grid" style={{ gridTemplateColumns: "220px 1fr", gap: 12, marginTop: 12 }}>
              <Field label="Steer Min (µs)">
                <input
                  type="number"
                  value={cfg.steer_min_us}
                  onChange={e => toInt(e, "steer_min_us")}
                />
              </Field>

              <Field label="Steer Max (µs)">
                <input
                  type="number"
                  value={cfg.steer_max_us}
                  onChange={e => toInt(e, "steer_max_us")}
                />
              </Field>

              <Field label="Throttle Min (µs)">
                <input
                  type="number"
                  value={cfg.throttle_min}
                  onChange={e => toInt(e, "throttle_min")}
                />
              </Field>

              <Field label="Throttle Base (µs)">
                <input
                  type="number"
                  value={cfg.throttle_base}
                  onChange={e => toInt(e, "throttle_base")}
                />
              </Field>

              <Field label="Throttle Max (µs)">
                <input
                  type="number"
                  value={cfg.throttle_max}
                  onChange={e => toInt(e, "throttle_max")}
                />
              </Field>
            </div>

            {/* Timing / Calibration */}
            <div className="grid" style={{ gridTemplateColumns: "220px 1fr", gap: 12, marginTop: 12 }}>
              <Field label="ESC Arm (ms)">
                <input
                  type="number"
                  value={cfg.esc_arm_ms}
                  onChange={e => toInt(e, "esc_arm_ms")}
                />
              </Field>

              <Field label="QTR Timeout (µs)">
                <input
                  type="number"
                  value={cfg.qtr_timeout_us}
                  onChange={e => toInt(e, "qtr_timeout_us")}
                />
              </Field>

              <Field label="Calibrate (ms)">
                <input
                  type="number"
                  value={cfg.calibrate_ms}
                  onChange={e => toInt(e, "calibrate_ms")}
                />
              </Field>
            </div>
          </>
        )}
      </div>
    </div>
  );
}
