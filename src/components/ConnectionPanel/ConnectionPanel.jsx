import React, { useState } from "react";
import "./connectionpanel.css";
import { deviceApi } from "../../api/deviceApi";

export default function ConnectionPanel({ connected, onConnect, onDisconnect }) {
  // const [deviceUrl, setDeviceUrl] = useState("http://pacebot.local");
  const [deviceUrl, setDeviceUrl] = useState("http://192.168.1.207/");
  const [showWifi, setShowWifi] = useState(false);

  // AP provisioning state
  const [apHost, setApHost] = useState("http://192.168.4.1");
  const [ssidList, setSsidList] = useState([]);
  const [ssid, setSsid] = useState("");
  const [pwd, setPwd] = useState("");
  const [scanBusy, setScanBusy] = useState(false);
  const [provBusy, setProvBusy] = useState(false);

  async function scan() {
    setScanBusy(true);
    try {
      const controller = new AbortController();
      const t = setTimeout(() => controller.abort(), 8000);
      const res = await fetch(`${apHost.replace(/\/$/,"")}/wifi/scan`, { signal: controller.signal });
      clearTimeout(t);

      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const nets = await res.json();
      nets.sort((a,b)=> (b.rssi ?? 0) - (a.rssi ?? 0));
      setSsidList(nets);
      if (nets.length && !ssid) setSsid(nets[0].ssid);
      if (!nets.length) alert("No networks found. Try again closer to your router.");
    } catch (e) {
      alert("Scan failed. Ensure you’re connected to the ESP AP.\n" + e.message);
    } finally {
      setScanBusy(false);
    }
  }

  async function provision() {
    try {
      if (!ssid) return alert("Pick an SSID first.");
      setProvBusy(true);
      const info = await deviceApi.apProvision(apHost, ssid, pwd);
      alert((info.ok ? "Provisioned OK\n" : "Provision failed\n") +
            `Host: ${info.host}\nIP: ${info.ip}\n\nTip: On Windows, mDNS 'pacebot.local' often needs Bonjour.\nIf it doesn't resolve, use the IP above in the Device URL.`);
    } catch (e) {
      alert("Provision failed: " + e);
    } finally { setProvBusy(false); }
  }

  const doConnect = async () => onConnect(deviceUrl);

  return (
    <div className="connection">
      <div className="row">
        <span className={`badge ${connected ? "ok" : "danger"}`}>
          {connected ? "Connected" : "Disconnected"}
        </span>
        <span className="label">Transport: WebSocket (auto from HTTP base)</span>
      </div>

      <button className="disclosure" onClick={()=>setShowWifi(s => !s)}>
        <span className="arrow">{showWifi ? "▾" : "▸"}</span>
        Wi-Fi Setup
      </button>

      {showWifi && (
        <div className="wifi-block">
          <div className="row">
            <label className="label">AP host</label>
            <input value={apHost} onChange={e=>setApHost(e.target.value)} placeholder="http://192.168.4.1" />
            <button onClick={scan} disabled={scanBusy}>
              {scanBusy ? "Scanning…" : "Scan"}
            </button>
          </div>

          <div className="row">
            <label className="label">SSID</label>
            <select value={ssid} onChange={e=>setSsid(e.target.value)}>
              <option value="">Select…</option>
              {ssidList.map((n,i)=>(
                <option key={i} value={n.ssid}>
                  {n.ssid} {typeof n.rssi==="number" ? `(${n.rssi} dBm)` : ""}
                </option>
              ))}
            </select>
            <input placeholder="Password" type="password" value={pwd} onChange={e=>setPwd(e.target.value)} />
            <button onClick={provision} disabled={provBusy}>
              {provBusy ? "Provisioning…" : "Provision"}
            </button>
          </div>
        </div>
      )}

      <div className="row">
        <input
          style={{minWidth: 280}}
          value={deviceUrl}
          onChange={e=>setDeviceUrl(e.target.value)}
          placeholder="http://pacebot.local"
        />
        {!connected ? (
          <button className="primary" onClick={doConnect}>Connect</button>
        ) : (
          <button className="danger" onClick={onDisconnect}>Disconnect</button>
        )}
      </div>

      <div className="hint">
        1. Join the ESP AP and provision venue Wi-Fi (arrow). 2. Connect using HTTP base (e.g. http://pacebot.local). Telemetry WS is derived automatically.
      </div>
    </div>
  );
}
