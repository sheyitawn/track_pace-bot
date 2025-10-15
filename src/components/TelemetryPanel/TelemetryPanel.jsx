import React from "react";
import "./telemetrypanel.css";

export default function TelemetryPanel({ telemetry }) {
  if (!telemetry) {
    return <div className="telemetry-root"><div className="muted">
      No telemetry yet… Connect to the device.
    </div></div>;
  }
  const fields = [
    ["Mode", telemetry.mode],
    ["Speed (m/s)", telemetry.speed_mps?.toFixed(2)],
    ["Distance (m)", telemetry.distance_m?.toFixed(1)],
    ["Line error (cm)", telemetry.line_error_cm != null ? telemetry.line_error_cm.toFixed(1) : "—"],
    ["Yaw (deg)", telemetry.imu_yaw_deg?.toFixed(1)],
    ["QTR quality", telemetry.qtr_quality != null ? telemetry.qtr_quality.toFixed(0) : "—"],
    ["Battery (V)", telemetry.battery_v?.toFixed(2)]
  ];
  return (
    <div className="telemetry-root">
       <h3>Telemetry</h3>
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
