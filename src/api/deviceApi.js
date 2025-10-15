// will be switched later to real WebSocket/HTTP/BLE transport.
import * as mock from "./mockTransport";

export const deviceApi = {
  connect: () => mock.connect(),
  disconnect: () => mock.disconnect(),
  subscribeTelemetry: (fn) => mock.onTelemetry(fn),
  setMode: (m) => mock.setMode(m),
  uploadSplits: (s) => mock.uploadSplits(s),
  saveTrackMap: (p) => mock.saveTrackMap(p)
};
