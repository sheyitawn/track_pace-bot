//  mock rc producing telemetry at ~20 Hz.
let interval;
let subs = new Set();
let mode = "idle";

export async function connect() {
  // simulate connection delay
  await new Promise((r) => setTimeout(r, 400));
  // start telemetry loop
  let t = 0;
  clearInterval(interval);
  interval = setInterval(() => {
    t += 0.05;
    const speed = mode === "idle" ? 0 : Math.max(0, 3 + 2 * Math.sin(t));
    const payload = {
      ts: Date.now(),
      mode,
      speed_mps: speed,
      distance_m: Math.max(0, speed * t * 0.2),
      line_error_cm: mode === "follow_track" ? (Math.sin(t * 0.7) * 6) : null,
      imu_yaw_deg: (t * 10) % 360,
      battery_v: 7.6 + 0.1 * Math.sin(t / 5),
      qtr_quality: mode === "follow_track" ? 85 + 10 * Math.sin(t) : 0
    };
    subs.forEach((fn) => fn(payload));
  }, 50);
  return true;
}
export async function disconnect() {
  clearInterval(interval);
  subs.clear();
}
export function onTelemetry(fn) {
  subs.add(fn);
  return () => subs.delete(fn);
}
export async function setMode(next) {
  mode = next;
}
export async function uploadSplits(splits) {
  // mock ack
  return { ok: true, count: splits.length };
}
export async function saveTrackMap(points) {
  return { ok: true, points: points.length };
}
