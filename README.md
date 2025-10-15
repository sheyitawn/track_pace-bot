# Track Pace-Bot Control Panel
A web interface for configuring and monitoring the **Track Pace-Bot**. This is a high-speed RC car that autonomously follows a 100 m track and paces an athlete.

### Features
- Connect via WebSocket (ESP32-S3)
- Live telemetry (speed, distance, line error, IMU, battery)
- Split-time profile editor & upload
- Mode control: Follow Track / Pace Athlete / Map Track
- Reference track mapper

### Structure
React → deviceApi → wsTransport → ESP32 firmware

```bash
npm install
npm start
```

### Quick Use
1. Connect device
2. Add split profiles → Upload
3. Press Follow Track
4. Watch telemetry update live


© 2025 [sheyitawn.dev](https://sheyitawn.dev)