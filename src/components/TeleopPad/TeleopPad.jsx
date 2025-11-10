import React, { useRef, useState } from "react";
import "./teleoppad.css";
import { deviceApi } from "../../api/deviceApi";

export default function TeleopPad({ connected, mode, onSetMode }) {
  const [pressed, setPressed] = useState(null); // 'up','down','left','right','stop'
  const sendingRef = useRef(false);

  const requireTeleopMode = async () => {
    if (mode !== "teleop") {
      await onSetMode("teleop");
    }
  };

  const send = (x, y) => {
    if (!connected) return;
    deviceApi.teleop(x, y);
  };
  const neutral = () => {
    if (!connected) return;
    deviceApi.teleopNeutral();
  };

  const handlePress = async (dir) => {
    if (!connected) return;
    setPressed(dir);
    await requireTeleopMode();
    sendingRef.current = true;

    let x = 0, y = 0;
    if (dir === "up") y = 1;
    if (dir === "down") y = -1;
    if (dir === "left") x = -1;
    if (dir === "right") x = 1;
    if (dir === "stop") { neutral(); return; }

    send(x, y);
  };

  const handleRelease = () => {
    if (!connected) return;
    setPressed(null);
    sendingRef.current = false;
    neutral(); // fail-safe: neutral on release
  };

  return (
    <div className="teleop-root">
      <div className="row" style={{ justifyContent: "space-between" }}>
        <h3 style={{ margin: 0 }}>Teleop</h3>
        <span className="badge">{connected ? (mode === "teleop" ? "teleop" : "idle") : "offline"}</span>
      </div>

      <div className="pad">
        <button
          className={`pad-btn up ${pressed==="up" ? "active":""}`}
          disabled={!connected}
          onPointerDown={() => handlePress("up")}
          onPointerUp={handleRelease}
          onPointerLeave={handleRelease}
        >▲</button>

        <div className="mid-row">
          <button
            className={`pad-btn left ${pressed==="left" ? "active":""}`}
            disabled={!connected}
            onPointerDown={() => handlePress("left")}
            onPointerUp={handleRelease}
            onPointerLeave={handleRelease}
          >◀</button>

          <button
            className={`pad-btn center ${pressed==="stop" ? "active":""}`}
            disabled={!connected}
            onPointerDown={() => handlePress("stop")}
            onPointerUp={handleRelease}
            onPointerLeave={handleRelease}
          >■</button>

          <button
            className={`pad-btn right ${pressed==="right" ? "active":""}`}
            disabled={!connected}
            onPointerDown={() => handlePress("right")}
            onPointerUp={handleRelease}
            onPointerLeave={handleRelease}
          >▶</button>
        </div>

        <button
          className={`pad-btn down ${pressed==="down" ? "active":""}`}
          disabled={!connected}
          onPointerDown={() => handlePress("down")}
          onPointerUp={handleRelease}
          onPointerLeave={handleRelease}
        >▼</button>
      </div>

      <div className="label" style={{ marginTop: 8 }}>
        Tip: Hold arrows to nudge; release sends neutral. Press ■ to stop.
      </div>
    </div>
  );
}
