import React, { useState } from "react";
import { nanoid } from "nanoid";
import "./spliteditor.css";
import { validateSplits } from "../../utils/validation";

export default function SplitEditor({ splits, setSplits, onUpload }) {
  const [error, setError] = useState(null);

  const addSplit = () => {
    setSplits([...splits, { id: nanoid(6), start: 0, end: 0, time: 0 }]);
  };
  const update = (id, field, val) => {
    setSplits(splits.map(s => s.id === id ? { ...s, [field]: val } : s));
  };
  const remove = (id) => setSplits(splits.filter(s => s.id !== id));

  const upload = () => {
    const err = validateSplits(splits);
    if (err) { setError(err); return; }
    setError(null);
    onUpload();
  };

  const totalTime = splits.reduce((a,b) => a + Number(b.time||0), 0);

  return (
    <div>
      <div className="row">
        <h3>Split Profiles</h3>
        <span className="badge">total: {totalTime.toFixed(2)} s</span>
        <button onClick={addSplit}>Add Split</button>
        <button className="primary" onClick={upload}>Upload to Device</button>
      </div>

      {error && <div className="error">{error}</div>}

      <table className="table">
        <thead>
          <tr>
            <th>#</th><th>Start (m)</th><th>End (m)</th><th>Time (s)</th><th>Target m/s</th><th></th>
          </tr>
        </thead>
        <tbody>
          {splits.map((s, i) => {
            const dist = Number(s.end) - Number(s.start);
            const v = dist > 0 && s.time > 0 ? (dist / s.time) : 0;
            return (
              <tr key={s.id}>
                <td>{i+1}</td>
                <td><input type="number" min="0" max="1000"
                  value={s.start} onChange={e => update(s.id,"start", Number(e.target.value))}/></td>
                <td><input type="number" min="0" max="1000"
                  value={s.end} onChange={e => update(s.id,"end", Number(e.target.value))}/></td>
                <td><input type="number" step="0.01" min="0"
                  value={s.time} onChange={e => update(s.id,"time", Number(e.target.value))}/></td>
                <td>{v.toFixed(2)}</td>
                <td><button onClick={() => remove(s.id)}>Delete</button></td>
              </tr>
            );
          })}
        </tbody>
      </table>

      <div className="row tips">
        <span className="label">Tip:</span>
        <span>For a single target (e.g., 100 m in 10.50 s), use one row: start 0, end 100, time 10.50.</span>
      </div>
    </div>
  );
}
