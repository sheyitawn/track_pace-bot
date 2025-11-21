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
1. Power Device
2. Connect to 'PaceBot-XXXX' Wifi AP. (pass: 'setup1234')
3. Click Scan in the Connection Panel (under WIFI Setup)
4. Select and Fill out wifi SSID & Password for WIFI you want to connect to
2. Click Provision
3. Click Connect. You should now be connected to the pace-bot, and see telemetry data coming through

### Config Page
### Modes
### Teleoperation
### Split Profiles


© 2025 [sheyitawn.dev](https://sheyitawn.dev)