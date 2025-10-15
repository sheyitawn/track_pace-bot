import React from "react";
import "./controlbar.css";

export default function ControlBar({
  connected, mode, onStartFollowTrack, onStartPaceAthlete, onStartMapTrack, onStop
}) {
  return (
    <div className="controlbar">
      <div className="row">
        <button disabled={!connected || mode!=="idle"} onClick={onStartFollowTrack}>
          Follow Track
        </button>
        <button disabled={!connected || mode!=="idle"} onClick={onStartPaceAthlete}>
          Pace Athlete
        </button>
        <button disabled={!connected || mode!=="idle"} onClick={onStartMapTrack}>
          Map Track
        </button>
        <button className="danger" disabled={!connected || mode==="idle"} onClick={onStop}>
          Stop
        </button>
      </div>
      <div className="row">
        <span className="label">Mode:</span>
        <span className="badge">{mode}</span>
      </div>
      <div className="row">
        {/* <label className="label">Ease-In</label> */}
        <p className="help">
          Use the controller to nudge the car onto the line, then press <em>Follow Track</em>.
        </p>
      </div>
    </div>
  );
}
