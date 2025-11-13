let ws = null;
let onMessageCB = null;

function hostFromBaseUrl(baseUrl) {
  try {
    const u = new URL(baseUrl.match(/^https?:\/\//) ? baseUrl : `http://${baseUrl}`);
    return u.hostname; // strips protocol, port, path
  } catch {
    return baseUrl.replace(/^https?:\/\//, "").replace(/\/+$/, "");
  }
}

export function openTelemetrySocket(baseUrl, onMessage) {
  closeTelemetrySocket();
  onMessageCB = onMessage;

  const host = hostFromBaseUrl(baseUrl);
  const wsUrl = `ws://${host}:81`; // firmware listens on :81

  return new Promise((resolve, reject) => {
    let settled = false;
    ws = new WebSocket(wsUrl);

    ws.onopen = () => { settled = true; resolve(() => closeTelemetrySocket()); };
    ws.onerror = () => { if (!settled) { settled = true; reject(new Error("WebSocket error")); } };
    ws.onclose = () => { if (!settled) { settled = true; reject(new Error("WebSocket closed")); } };
    ws.onmessage = (e) => {
      try { const j = JSON.parse(e.data); onMessageCB && onMessageCB(j); } catch {}
    };
  });
}

export function closeTelemetrySocket() {
  if (ws) { try { ws.close(); } catch {} ws = null; }
  onMessageCB = null;
}

export function sendJSON(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}
