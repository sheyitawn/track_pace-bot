import React from "react";
import "./telemetrypanel.css";

export default function TelemetryPanel({ telemetry, connected, lastTs }) {
  const now = Date.now();
  const staleMs = lastTs ? (now - lastTs) : null;
  const stale = connected && lastTs && staleMs > 2000;

  if (!connected) {
    return <div className="telemetry-root"><div className="muted">Disconnected.</div></div>;
  }
  if (!telemetry) {
    return <div className="telemetry-root"><div className="muted">
      Connected, awaiting telemetry…
    </div></div>;
  }

  const fields = [
    ["Mode", telemetry.mode],
    ["Speed (m/s)", safeNum(telemetry.speed_mps, 2)],
    ["Distance (m)", safeNum(telemetry.distance_m, 1)],
    ["Line error (cm)", safeNum(telemetry.line_error_cm, 1, "—")],
    ["Yaw (deg)", safeNum(telemetry.imu_yaw_deg, 1)],
    ["QTR quality", safeNum(telemetry.qtr_quality, 0, "—")],
    ["Battery (V)", safeNum(telemetry.battery_v, 2)]
  ];

  return (
    <div className="telemetry-root">
      <div className="row">
        <h3>Telemetry</h3>
        <span className={`badge ${stale ? "warn" : "ok"}`}>
          {stale ? `No frames ${Math.round(staleMs/1000)}s` : "Live"}
        </span>
      </div>
      <table className="table">
        <tbody>
          {fields.map(([k,v]) => (
            <tr key={k}><th>{k}</th><td>{v}</td></tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

function safeNum(v, dp=2, fallback="0.00") {
  if (v === null || v === undefined || Number.isNaN(v)) return fallback;
  if (typeof v !== "number") return String(v);
  return v.toFixed(dp);
}
