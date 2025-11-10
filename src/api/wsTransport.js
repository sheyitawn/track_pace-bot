let ws = null;
let urlHost = null; // "192.168.4.1", "pacebot.local"
let onMessageCB = null;

export function openTelemetrySocket(baseUrl, onMessage) {
  closeTelemetrySocket();

  const host = baseUrl.replace(/^https?:\/\//, "").replace(/\/+$/, "");
  urlHost = host;
  onMessageCB = onMessage;

  // WS is on port 81, no path
  const wsUrl = `ws://${host.split("/")[0].split(":")[0]}:81`;
  ws = new WebSocket(wsUrl);

  ws.onopen = () => { };
  ws.onclose = () => {  };
  ws.onerror = () => {  };
  ws.onmessage = (e) => {
    try {
      const j = JSON.parse(e.data);
      if (onMessageCB) onMessageCB(j);
    } catch (err) {
    }
  };
  return () => closeTelemetrySocket();
}

export function closeTelemetrySocket() {
  if (ws) {
    try { ws.close(); } catch {}
    ws = null;
  }
  onMessageCB = null;
  urlHost = null;
}

export function sendJSON(obj) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}
