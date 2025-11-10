import { openTelemetrySocket, closeTelemetrySocket, sendJSON } from "./wsTransport";

let _baseUrl = null;   // "http://192.168.4.1", "http://pacebot.local"
let _unsub = null;
let _telemetrySubs = new Set();

const trimBase = (u) => (u || "").replace(/\/+$/,"");
const originOr = (baseUrl) => trimBase(baseUrl || _baseUrl);

async function http(path, opts = {}, baseUrl) {
  const origin = originOr(baseUrl);
  if (!origin) throw new Error("Not connected");
  const url = `${origin}${path}`;
  const res = await fetch(url, {
    // Only set JSON header when we send a body
    headers: opts.body ? { "Content-Type": "application/json" } : undefined,
    ...opts,
  });
  if (!res.ok) throw new Error(`${opts.method || "GET"} ${path} ${res.status}`);
  return res;
}

function notifyTelemetry(msg) {
  _telemetrySubs.forEach((cb) => { try { cb(msg); } catch {} });
}


export async function getConfig(baseUrl) {
  const r = await http("/config", { method: "GET" }, baseUrl);
  return r.json();
}

export async function postConfig(baseUrl, cfg) {
  const r = await http("/config", {
    method: "POST",
    body: JSON.stringify(cfg) // do not filter keys; firmware merges/clamps safely
  }, baseUrl);
  return r.json();
}

export async function resetConfig(baseUrl) {
  const r = await http("/config/reset", { method: "POST" }, baseUrl);
  return r.json();
}


export const deviceApi = {
  getBaseUrl() { return _baseUrl; },

  async connect(url) {
    _baseUrl = trimBase(url); // HTTP base
    _unsub?.();
    _unsub = openTelemetrySocket(_baseUrl, (j) => notifyTelemetry(j));
    return true;
  },

  async disconnect() {
    _unsub?.();
    _unsub = null;
    closeTelemetrySocket();
    return true;
  },

  async refreshConnection() {
    const url = _baseUrl;
    if (!url) throw new Error("No base URL to refresh");
    await this.disconnect();
    await this.connect(url);
    return true;
  },

  //  telemetry subscription
  subscribeTelemetry(cb) {
    _telemetrySubs.add(cb);
    return () => _telemetrySubs.delete(cb);
  },

  //  commands
  async setMode(mode) { sendJSON({ cmd: "set_mode", value: mode }); },
  async uploadSplits(splits) { sendJSON({ cmd: "upload_splits", splits }); },

  // teleop (normalized x,y ∈ [-1,1])
  teleop(x, y)        { sendJSON({ cmd: "teleop", x, y }); },
  teleopNeutral()     { sendJSON({ cmd: "teleop", x: 0, y: 0 }); },

  async apProvision(apBaseUrl, ssid, password) {
    const base = trimBase(apBaseUrl);
    const res = await fetch(`${base}/wifi/connect`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ssid, password }),
    });
    if (!res.ok) throw new Error(`POST /wifi/connect ${res.status}`);
    return res.json(); // {ok, ip, host, ws}
  },

  // status
  async getStatus() { const r = await http("/wifi/status"); return r.json(); },

  // config endpoints
  async getConfig()         { return getConfig(); },
  async updateConfig(cfg)   { return postConfig(undefined, cfg); },
  async resetConfig()       { return resetConfig(); },
};
