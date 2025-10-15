import React from "react";
import "./connectionpanel.css";

export default function ConnectionPanel({ connected, onConnect, onDisconnect }) {
  return (
    <div className="connection">
      <div className="row">
        <span className={`badge ${connected ? "ok" : "danger"}`}>
          {connected ? "Connected" : "Disconnected"}
        </span>
        <span className="label">Transport: mock (swap to WebSocket later)</span>
      </div>
      <div className="row">
        {!connected ? (
          <button className="primary" onClick={onConnect}>Connect</button>
        ) : (
          <button className="danger" onClick={onDisconnect}>Disconnect</button>
        )}
      </div>
      <div className="hint">
        When hardware is ready, mock rc will be replaced with WebSocket.
      </div>
    </div>
  );
}
