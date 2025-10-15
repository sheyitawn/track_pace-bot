import React, { useRef, useState } from "react";
import "./trackmapper.css";
import { deviceApi } from "../../api/deviceApi";


export default function TrackMapper({ connected }) {
  const [points, setPoints] = useState([]);
  const canvasRef = useRef();

  const addPoint = (e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const x = (e.clientX - rect.left);
    const y = (e.clientY - rect.top);
    setPoints(p => [...p, { x, y }]);
  };

  const reset = () => setPoints([]);

  const save = async () => {
    if (!connected) { alert("Connect first."); return; }
    await deviceApi.saveTrackMap(points);
    alert(`Saved ${points.length} points to device (mock).`);
  };

  return (
    <div>
      <div className="row">
        <h3>Reference Track Mapper (mock)</h3>
        <button onClick={reset}>Clear</button>
        <button className="primary" onClick={save}>Save to Device</button>
      </div>
      <div className="label">Click to add points. This is a placeholder for a real mapping pass.</div>
      <div className="canvas-wrap">
        <canvas
          ref={canvasRef}
          className="canvas"
          width="760"
          height="300"
          onClick={addPoint}
        />
        {points.map((p, i) => (
          <div
            key={i}
            className="pt"
            style={{ left: p.x - 3, top: p.y - 3 }}
            title={`${i}: (${p.x.toFixed(0)}, ${p.y.toFixed(0)})`}
          />
        ))}
      </div>
    </div>
  );
}
