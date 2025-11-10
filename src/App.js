import React, { useEffect, useState } from "react";
import { BrowserRouter, Routes, Route, Link } from "react-router-dom";
import ConnectionPanel from "./components/ConnectionPanel/ConnectionPanel";
import ControlBar from "./components/ControlBar/ControlBar";
import SplitEditor from "./components/SplitEditor/SplitEditor";
import TelemetryPanel from "./components/TelemetryPanel/TelemetryPanel";
import TrackMapper from "./components/TrackMapper/TrackMapper";
import Footer from "./components/Footer/Footer";
import ConfigPage from "./pages/ConfigPage/ConfigPage";
import { deviceApi } from "./api/deviceApi";
import TeleopPad from "./components/TeleopPad/TeleopPad";

function Home() {
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState(null);
  const [lastTs, setLastTs] = useState(null);
  const [splits, setSplits] = useState([
    { id: "s1", start: 0,   end: 30,  time: 3.6 },
    { id: "s2", start: 30,  end: 70,  time: 4.2 },
    { id: "s3", start: 70,  end: 100, time: 3.1 },
  ]);
  const [mode, setMode] = useState("idle");

  useEffect(() => {
    if (!connected) return;
    const unsub = deviceApi.subscribeTelemetry((t) => {
      setTelemetry(t);
      if (t?.ts) setLastTs(Date.now());
      if (t?.mode && t.mode !== mode) setMode(t.mode);
    });
    return () => unsub && unsub();
  }, [connected]);

  const handleConnect = async (url) => {
    const ok = await deviceApi.connect(url);
    setConnected(ok);
    setTelemetry(null);
    setLastTs(null);
  };
  const handleDisconnect = async () => {
    await deviceApi.disconnect();
    setConnected(false);
    setTelemetry(null);
    setLastTs(null);
  };

  const handleUploadSplits = async () => { await deviceApi.uploadSplits(splits); };
  const handleStart = async (nextMode) => { setMode(nextMode); await deviceApi.setMode(nextMode); };
  const handleStop = async () => { setMode("idle"); await deviceApi.setMode("idle"); };

  const setModeFromChild = async (m) => { setMode(m); await deviceApi.setMode(m); };

  return (
    <div className="container">
      <header className="nav">
        <h1>Track Pace-Bot</h1>
        <div className="row">
          <Link to="/config" className="badge">Config</Link>
          <div className="build-pill">WS live</div>
        </div>
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

        {/* ADDED: TeleopPad block */}
        <section className="card">
          <TeleopPad connected={connected} mode={mode} onSetMode={setModeFromChild} />
        </section>

        <section className="card tall">
          <TelemetryPanel telemetry={telemetry} connected={connected} lastTs={lastTs} />
        </section>

        <section className="card span-2">
          <SplitEditor splits={splits} setSplits={setSplits} onUpload={handleUploadSplits} />
        </section>

        <section className="card span-2">
          <TrackMapper connected={connected} />
        </section>
      </div>

      <Footer />
    </div>
  );
}

export default function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/config" element={<ConfigPage />} />
        <Route path="/" element={<Home />} />
      </Routes>
    </BrowserRouter>
  );
}
