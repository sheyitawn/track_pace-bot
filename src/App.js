import React, { useEffect, useState } from "react";
import ConnectionPanel from "./components/ConnectionPanel/ConnectionPanel";
import ControlBar from "./components/ControlBar/ControlBar";
import SplitEditor from "./components/SplitEditor/SplitEditor";
import TelemetryPanel from "./components/TelemetryPanel/TelemetryPanel";
import TrackMapper from "./components/TrackMapper/TrackMapper";
import { deviceApi } from "./api/deviceApi";
import Footer from "./components/Footer/Footer";

export default function App() {
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState(null);
  const [splits, setSplits] = useState([
    { id: "s1", start: 0, end: 30, time: 3.6 },
    { id: "s2", start: 30, end: 70, time: 4.2 },
    { id: "s3", start: 70, end: 100, time: 3.1 }
  ]);
  const [mode, setMode] = useState("idle"); // idle | follow_track | pace_athlete | map_track

  // subscribe to telemetry
  useEffect(() => {
    if (!connected) return;
    const unsub = deviceApi.subscribeTelemetry((t) => setTelemetry(t));
    return () => unsub && unsub();
  }, [connected]);

  const handleConnect = async () => {
    const ok = await deviceApi.connect();
    setConnected(ok);
  };

  const handleDisconnect = async () => {
    await deviceApi.disconnect();
    setConnected(false);
  };

  const handleUploadSplits = async () => {
    await deviceApi.uploadSplits(splits);
    alert("Split profile uploaded to device (temp).");
  };

  const handleStart = async (nextMode) => {
    setMode(nextMode);
    await deviceApi.setMode(nextMode);
  };

  const handleStop = async () => {
    setMode("idle");
    await deviceApi.setMode("idle");
  };

  return (
    <div className="container">
      <header className="nav">
        <h1>Track Pace-Bot</h1>
        <div className="build-pill"> UI (temp info)</div>
      </header>

      <div className="grid">
        <section className="card">
          <ConnectionPanel
            connected={connected}
            onConnect={handleConnect}
            onDisconnect={handleDisconnect}
          />
        </section>

        <section className="card">
          <ControlBar
            connected={connected}
            mode={mode}
            onStartFollowTrack={() => handleStart("follow_track")}
            onStartPaceAthlete={() => handleStart("pace_athlete")}
            onStartMapTrack={() => handleStart("map_track")}
            onStop={handleStop}
          />
        </section>

        <section className="card span-2">
          <SplitEditor
            splits={splits}
            setSplits={setSplits}
            onUpload={handleUploadSplits}
          />
        </section>

        <section className="card tall">
          <TelemetryPanel telemetry={telemetry} />
        </section>

        <section className="card span-2">
          <TrackMapper connected={connected} />
        </section>
      </div>
            <div className="footer">
                <small>
                  NOTE: This UI is running with mock data. This will be replaced when final parts arrive.
                  </small>
              </div>
            <Footer />
    </div>
  );
}
