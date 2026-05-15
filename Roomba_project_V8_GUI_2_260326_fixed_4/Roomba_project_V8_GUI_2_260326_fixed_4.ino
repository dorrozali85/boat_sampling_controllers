// ESP32 Master Navigation (version 28.3 / GUI v8.4)
//fixed reports from arduino in gui

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <LittleFS.h>
#define RXD2 16  // RX to Arduino TX (via divider)
#define TXD2 17  // TX to Arduino RX

// -------------------- Motor Pins --------------------

// -------------------- States --------------------
bool stopMode = true;         // Start in STOP mode
bool manualMode = false;
bool autonomousMode = false;
bool stuckDetected = false;   // External obstacle detection flag
bool escArmed = false;        // Track ESC arming state
bool sampling = false;        // Sampling state
String arduinoState = "Ready"; // Tracks status of Arduino Uno platform

// -------------------- Parameters --------------------
float Motor_Power = 80.0;     // Base speed (Mini-me default)
float Turn_Power = 30.0;      // Max turn adjustment (Mini-me default)
float Reverse_Power = 60.0;   // Tunable reverse power (0-255), default 60
unsigned long STUCK_STOP_TIME_1 = 400;     // Stuck stopping time 1 (ms)
unsigned long STUCK_REVERSE_TIME = 2000;   // Stuck reversing time (ms)
unsigned long STUCK_STOP_TIME_2 = 400;     // Stuck stopping time 2 (ms)
unsigned long STUCK_FINAL_STOP_TIME = 400; // Stuck final stop time (ms)
float Motor_Offset = 1.0;     // Right motor power multiplier
unsigned long lastFwdPrint = 0;

// New tunable parameters
unsigned long debounce_ms = 30;        // Debounce time (ms)
unsigned long forward_lock_ms = 2000;  // Forward lock time (ms)
unsigned long sample_interval_ms = 60000; // Sample interval (ms, default 1 min)

// -------------------- Magnetometer Heading Navigation --------------------
unsigned long min_turn_angle = 30;        // Min random turn angle (degrees) — GUI: MA:
unsigned long max_turn_angle = 90;        // Max random turn angle (degrees) — GUI: XA:
float turn_tolerance_deg = 5.0;           // Tolerance for turn complete (degrees) — GUI: TT:
float heading_kp = 1.5;                   // Closed-loop forward P-gain — GUI: KP:
unsigned long turn_timeout_ms = 8000;     // Abort turn-to-heading timeout (ms) — GUI: TM: (in seconds)
float targetHeading = 0;                  // Current commanded heading (0–360°)
float currentHeading = 0;                 // Latest magnetometer reading (0–360°)

// -------------------- Stuck-Recovery Sample-Timer Pause --------------------
unsigned long stuckPauseStart = 0;        // millis() at stuck recovery entry; on exit shifts sampleStart by elapsed

// -------------------- Post-Sample Heading Alignment --------------------
bool aligningAfterSample = false;         // true = boat is realigning to targetHeading after a sample completes
unsigned long alignStart = 0;             // millis() at align start; for turn_timeout_ms abort

// -------------------- Sampling Platform Parameters (relayed to Arduino) --------------------
float    sample_depth_m  = 2.0;    // Sample depth (metres, 0–2)
unsigned long flush_time_ms = 30000;  // Flush pump duration (ms, default 30s)
unsigned long fill_time_ms  = 30000;  // Bottle-fill pump duration (ms, default 30s)

// -------------------- No-Sampler Mode --------------------
bool isWaterSamplerOnBoard = true;     // true = use Arduino Uno, false = simulate sampling
unsigned long skipSampleDelay = 10000; // Simulated sample hold time (ms, default 10s)
unsigned long skipSampleStart = 0;     // Internal: timestamp for non-blocking skip
unsigned long reverse_after_sample_duration = 7; // Reverse-motor duration at sample start in SECONDS (GUI: RSD:)
bool sampleReverseHandled = false;     // Internal: tracks reverse phase for non-blocking skip path

// -------------------- CSV Mission Logger (LittleFS) --------------------
// Records full boat state to /log_NNN.csv at 2 Hz for the duration of each
// autonomous mission. Sequence number persists in /log_counter.txt so
// filenames never collide across reboots.
File          logFile;                                 // file handle stays open for whole mission
bool          loggerActive = false;                    // true between autonomous start and stop
unsigned long autoStartMs  = 0;                        // millis() at autonomous mode entry — drives Runtime_sec
unsigned long lastLogMs    = 0;                        // last row timestamp — drives 500 ms rate gate
String        logBuffer    = "";                       // batches up to LOG_FLUSH_BYTES of CSV before file.print()
uint32_t      logSeqNum    = 0;                        // current/last log file number
const size_t  LOG_FLUSH_BYTES   = 512;                 // ~6 rows
const char*   LOG_COUNTER_PATH  = "/log_counter.txt";  // single int — persistent log sequence number
const char*   LOG_CSV_HEADER    =
    "Timestamp_ms,Runtime_sec,Mode,NavState,Stuck,SampleCount,ArduinoState,"
    "TargetHeading,ActualHeading,HeadingError,LeftPower,RightPower,"
    "SampleIntervalRemaining_ms";

// -------------------- PWM Channels --------------------
const int pwmPinA = 4;         // Servo pin for left motor
const int pwmPinB = 14;        // Servo pin for right motor
const int reverseAPin = 18;    // GPIO18 - free, PWM-capable
const int reverseBPin = 19;    // GPIO19 - free, PWM-capable

const int MAX_SAFE_PULSE = 1500; // Cap max throttle pulse
Servo escA;                    // Servo object for left motor
Servo escB;                    // Servo object for right motor
Servo escAReverse;             // Reverse enable left motor (yellow wire)
Servo escBReverse;             // Reverse enable right motor (yellow wire)

// -------------------- Wi-Fi Setup --------------------
const char* ssid = "ESP32_Robot_HTML";
const char* password = "12345678";
WebServer server(80);

// -------------------- Magnetometer --------------------
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// -------------------- Stuck Recovery --------------------
unsigned long stuckStart = 0;
int stuckStep = 0;

// -------------------- Status Variables --------------------
int lastLeft = 0;
int lastRight = 0;
// (lastHeading replaced by currentHeading in heading-nav section)

// -------------------- Forward Declarations --------------------
void stopCar();
void moveForward();
void moveForwardAutonomous();
void moveBackward();
void turnLeft();
void turnRight();
void readHeading();
float headingError(float target, float current);
void computeStuckTurnTarget(int switchHit);

// -------------------- HTML Content --------------------
const char* html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Boat SRV-01 - Control GUI</title>

<style>
:root {
  --bg: #eaf0f8;
  --surface: #ffffff;
  --surface2: #f0f5fb;
  --navy: #152a48;
  --navy2: #1e3d6a;
  --navy3: #2a5298;
  --border: rgba(21,42,72,0.18);
  --border2: rgba(21,42,72,0.32);
  --btn-blue: #1a4f8a;
  --btn-blue-h: #133d6e;
  --btn-teal: #0a6e54;
  --btn-teal-h: #085542;
  --btn-amber: #8a5c00;
  --btn-amber-h: #6e4800;
  --btn-red: #b81c1c;
  --btn-red-h: #961616;
  --val-blue: #1a4f8a;
  --val-teal: #0a6e54;
  --val-amber: #8a5c00;
  --val-red: #b81c1c;
  --txt: #152a48;
  --txt2: #4a6a8a;
  --sans: 'Trebuchet MS', 'Arial Narrow', Arial, sans-serif;
  --mono: 'Courier New', 'Lucida Console', monospace;
  --mw: 460px;
}
*{box-sizing:border-box;margin:0;padding:0;}
body{font-family:var(--sans);background:var(--bg);color:var(--txt);min-height:100vh;display:flex;flex-direction:column;align-items:center;}

.topbar{
  width:100%;background:var(--navy);padding:10px 20px;
  display:flex;justify-content:space-between;align-items:center;
  position:sticky;top:0;z-index:100;border-bottom:3px solid var(--navy3);
}
.vessel-tag{font-family:var(--mono);font-size:13px;color:#7ab0e0;letter-spacing:3px;}
.topbar-btns{display:flex;gap:8px;}
.btn-stop,.btn-arm,.btn-refresh{
  font-family:var(--sans);font-size:16px;font-weight:700;
  letter-spacing:1px;padding:10px 20px;border-radius:5px;border:none;
  cursor:pointer;transition:background 0.15s;color:#fff;
}
.btn-stop{background:var(--btn-red);}  .btn-stop:hover{background:var(--btn-red-h);}
.btn-arm{background:var(--btn-red);}   .btn-arm:hover{background:var(--btn-red-h);}
.btn-refresh{background:var(--btn-blue);}.btn-refresh:hover{background:var(--btn-blue-h);}

.page-header{width:100%;max-width:var(--mw);padding:16px 20px 4px;display:flex;align-items:baseline;gap:14px;}
.page-title{font-family:var(--sans);font-size:24px;font-weight:700;color:var(--navy);letter-spacing:1px;}
.page-sub{font-family:var(--mono);font-size:12px;color:var(--txt2);letter-spacing:2px;}

.tabnav{width:100%;max-width:var(--mw);display:flex;gap:4px;padding:10px 20px;}
.tabnav button{
  flex:1;font-family:var(--sans);font-size:16px;font-weight:700;
  letter-spacing:1px;padding:12px 8px;
  background:var(--surface);color:var(--navy2);
  border:1.5px solid var(--border2);border-radius:6px;
  cursor:pointer;transition:all 0.15s;
}
.tabnav button.active{background:var(--navy);color:#fff;border-color:var(--navy);}
.tabnav button:hover:not(.active){background:var(--surface2);border-color:var(--navy2);}

.tab{display:none;width:100%;max-width:var(--mw);padding:0 20px 32px;}
.tab.active{display:block;}

.sec-label{
  font-family:var(--mono);font-size:11px;letter-spacing:3px;color:var(--txt2);
  text-transform:uppercase;margin:14px 0 10px;
  padding-bottom:6px;border-bottom:1.5px solid var(--border2);
}

.joystick-wrap{display:flex;justify-content:center;margin-bottom:14px;}
canvas{border-radius:50%;border:2px solid var(--border2);background:var(--surface);touch-action:none;display:block;}

.full-btn{
  display:block;width:100%;font-family:var(--sans);font-size:18px;font-weight:700;
  letter-spacing:2px;padding:14px;background:var(--btn-blue);color:#fff;
  border:none;border-radius:6px;cursor:pointer;margin-bottom:14px;transition:background 0.15s;
}
.full-btn:hover{background:var(--btn-blue-h);}

.btn-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:8px;}
.btn-grid.two{grid-template-columns:1fr 1fr;}
.abtn{
  font-family:var(--sans);font-size:15px;font-weight:700;letter-spacing:0.5px;
  padding:14px 6px;background:var(--btn-blue);color:#fff;
  border:none;border-radius:6px;cursor:pointer;transition:background 0.15s;text-align:center;
}
.abtn:hover{background:var(--btn-blue-h);}
.abtn.green{background:var(--btn-teal);}  .abtn.green:hover{background:var(--btn-teal-h);}
.abtn.amber{background:var(--btn-amber);} .abtn.amber:hover{background:var(--btn-amber-h);}

.tt-grid{display:grid;grid-template-columns:repeat(6,1fr);gap:6px;margin-bottom:14px;}
.tbtn{
  font-family:var(--mono);font-size:18px;font-weight:700;padding:14px 0;
  background:var(--btn-amber);color:#fff;border:none;border-radius:6px;
  cursor:pointer;text-align:center;transition:background 0.15s;
}
.tbtn:hover{background:var(--btn-amber-h);}

.status-card{background:var(--surface);border:1.5px solid var(--border2);border-radius:8px;overflow:hidden;margin-bottom:14px;}
.status-card-head{font-family:var(--mono);font-size:11px;letter-spacing:3px;color:var(--navy);background:var(--surface2);padding:10px 16px;border-bottom:1.5px solid var(--border2);}
.stat-grid{display:grid;grid-template-columns:1fr 1fr;}
.stat-cell{padding:12px 16px;border-right:1px solid var(--border);border-bottom:1px solid var(--border);}
.stat-cell:nth-child(2n){border-right:none;}
.stat-cell.wide{grid-column:1/-1;border-right:none;}
.stat-lbl{font-family:var(--mono);font-size:10px;letter-spacing:2px;color:var(--txt2);margin-bottom:4px;text-transform:uppercase;}
.stat-val{font-family:var(--mono);font-size:20px;color:var(--val-blue);}
.stat-val.teal{color:var(--val-teal);}
.stat-val.amber{color:var(--val-amber);}
.stat-val.red{color:var(--val-red);}

.auto-btns{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px;}
.auto-btns button{
  font-family:var(--sans);font-size:17px;font-weight:700;letter-spacing:1px;
  padding:16px;border:none;border-radius:6px;color:#fff;cursor:pointer;transition:background 0.15s;
}
.abtn-auto{background:var(--btn-teal);}  .abtn-auto:hover{background:var(--btn-teal-h);}
.abtn-stuck{background:var(--btn-amber);}.abtn-stuck:hover{background:var(--btn-amber-h);}

.mission-card{
  background:var(--surface);border:1.5px solid var(--border2);
  border-radius:8px;overflow:hidden;margin-bottom:14px;
}
.mission-card-head{
  font-family:var(--mono);font-size:11px;letter-spacing:3px;color:var(--navy);
  background:var(--surface2);padding:10px 16px;border-bottom:1.5px solid var(--border2);
}
.mission-grid{display:grid;grid-template-columns:1fr 1fr;gap:0;}
.mission-cell{
  padding:12px 16px;border-right:1px solid var(--border);border-bottom:1px solid var(--border);
}
.mission-cell:nth-child(2n){border-right:none;}
.mission-cell.wide{grid-column:1/-1;border-right:none;}
.mission-cell:last-child{border-bottom:none;}
.mission-cell:nth-last-child(2){border-bottom:none;}
.mc-label{font-family:var(--mono);font-size:10px;letter-spacing:2px;color:var(--txt2);margin-bottom:6px;text-transform:uppercase;}
.mc-select,.mc-input{
  font-family:var(--mono);font-size:17px;color:var(--val-blue);
  background:var(--bg);border:1.5px solid var(--border2);border-radius:5px;
  width:100%;padding:8px 10px;cursor:pointer;
}
.mc-select:focus,.mc-input:focus{outline:none;border-color:var(--navy3);}

.timers-card{
  background:var(--navy);border:1.5px solid var(--navy2);
  border-radius:8px;overflow:hidden;margin-bottom:14px;
}
.timers-card-head{
  font-family:var(--mono);font-size:11px;letter-spacing:3px;color:#7ab0e0;
  background:rgba(255,255,255,0.06);padding:10px 16px;border-bottom:1px solid rgba(255,255,255,0.1);
}
.timers-grid{display:grid;grid-template-columns:1fr 1fr;gap:0;}
.timer-cell{padding:16px 16px 14px;border-right:1px solid rgba(255,255,255,0.08);}
.timer-cell:last-child{border-right:none;}
.timer-label{font-family:var(--mono);font-size:10px;letter-spacing:2px;color:#7ab0e0;margin-bottom:6px;text-transform:uppercase;}
.timer-value{font-family:var(--mono);font-size:28px;font-weight:normal;color:#f0f5fb;letter-spacing:1px;line-height:1;}
.timer-value.sampling{color:#34c789;font-size:19px;letter-spacing:0;margin-top:5px;}
.timer-value.stopped{color:#7ab0e0;}
.timer-sub{font-family:var(--mono);font-size:10px;color:#7ab0e0;margin-top:5px;letter-spacing:1px;}

.param-row{
  display:flex;align-items:center;gap:8px;padding:10px 14px;
  background:var(--surface);border:1.5px solid var(--border);border-radius:6px;margin-bottom:7px;
}
.pname{font-family:var(--sans);font-size:15px;font-weight:600;color:var(--txt);flex:1;min-width:0;}
.padj{
  font-family:var(--mono);font-size:20px;font-weight:bold;width:38px;height:38px;
  background:var(--surface2);color:var(--navy);border:1.5px solid var(--border2);border-radius:5px;
  cursor:pointer;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:background 0.12s;
}
.padj:hover{background:var(--bg);color:var(--btn-blue);}
.pinp{
  font-family:var(--mono);font-size:17px;width:80px;height:38px;text-align:center;
  background:var(--bg);color:var(--val-blue);border:1.5px solid var(--border2);border-radius:5px;flex-shrink:0;
}
.pset{
  font-family:var(--sans);font-size:14px;font-weight:700;letter-spacing:1px;padding:0 16px;height:38px;
  background:var(--btn-blue);color:#fff;border:none;border-radius:5px;cursor:pointer;flex-shrink:0;transition:background 0.12s;
}
.pset:hover{background:var(--btn-blue-h);}

@media(min-width:660px){
  :root{--mw:700px;}
  .topbar,.tabnav,.tab,.page-header{max-width:700px;}
  .manual-layout{display:grid;grid-template-columns:1fr 1fr;gap:18px;align-items:start;}
  canvas{width:220px;height:220px;}
  .stat-val{font-size:22px;}
  .abtn{font-size:16px;padding:16px 8px;}
  .full-btn{font-size:19px;}
  .timer-value{font-size:30px;}
  .mc-select,.mc-input{font-size:18px;}
}
@media(min-width:1020px){
  :root{--mw:980px;}
  .topbar,.tabnav,.tab,.page-header{max-width:980px;}
  .manual-layout{grid-template-columns:440px 1fr;gap:28px;}
  .auto-layout{display:grid;grid-template-columns:1fr 1fr;gap:24px;align-items:start;}
  .auto-left{grid-column:1;}
  .auto-right{grid-column:2;}
  .params-layout{display:grid;grid-template-columns:1fr 1fr;gap:20px;align-items:start;}
  canvas{width:240px;height:240px;}
  .stat-val{font-size:24px;}
  .abtn{font-size:17px;padding:17px 8px;}
  .full-btn{font-size:20px;padding:16px;}
  .tbtn{font-size:20px;padding:16px 0;}
  .tabnav button{font-size:17px;padding:14px;}
  .pname{font-size:16px;}
  .timer-value{font-size:34px;}
}
</style>
</head>
<body>

<div class="topbar">
  <div class="vessel-tag">&#9679; SRV-01 &nbsp;|&nbsp; CONTROL</div>
  <div class="topbar-btns">
    <button class="btn-stop" onclick="globalStop()">STOP</button>
    <button class="btn-arm" id="armButton" onclick="armEsc()">ARM</button>
    <button class="btn-refresh" onclick="refresh()">&#8635; Refresh</button>
  </div>
</div>

<div class="page-header">
  <div class="page-title">Boat Real Scale GUI</div>
  <div class="page-sub">ESP32 MASTER v26</div>
</div>

<div class="tabnav">
  <button onclick="openTab('manual')" class="active">Manual</button>
  <button onclick="openTab('auto')">Autonomous</button>
  <button onclick="openTab('params')">Parameters</button>
</div>

<div id="manual" class="tab active">
  <div class="manual-layout">
    <div class="manual-left">
      <div class="sec-label">Helm</div>
      <div class="joystick-wrap">
        <canvas id="joystick" width="200" height="200"></canvas>
      </div>
      <button class="full-btn" onclick="engageManual()">&#9654;&ensp;Engage Manual Mode</button>

      <div class="sec-label">Sampling Operations</div>
      <div class="btn-grid" style="margin-bottom:8px;">
        <button class="abtn green" onclick="sendCommand('A')">Manual Sample</button>
        <button class="abtn green" onclick="sendCommand('Q')">Pump On</button>
        <button class="abtn" onclick="sendCommand('I')">Return Zero</button>
      </div>
      <div class="btn-grid two" style="margin-bottom:8px;">
        <button class="abtn" onclick="sendCommand('D')">Sample Depth</button>
        <button class="abtn" onclick="sendCommand('H')">Depth Up</button>
      </div>
      <div class="btn-grid two" style="margin-bottom:14px;">
        <button class="abtn" onclick="sendCommand('G')">Depth Down</button>
      </div>

      <div class="sec-label">Turntable Position</div>
      <div class="tt-grid">
        <button class="tbtn" onclick="sendCommand('0')">0</button>
        <button class="tbtn" onclick="sendCommand('1')">1</button>
        <button class="tbtn" onclick="sendCommand('2')">2</button>
        <button class="tbtn" onclick="sendCommand('3')">3</button>
        <button class="tbtn" onclick="sendCommand('4')">4</button>
        <button class="tbtn" onclick="sendCommand('5')">5</button>
      </div>
    </div>

    <div class="manual-right">
      <div class="sec-label">Telemetry</div>
      <div class="status-card">
        <div class="status-card-head">System Status</div>
        <div class="stat-grid">
          <div class="stat-cell"><div class="stat-lbl">Mode</div><div class="stat-val" id="manual-mode">--</div></div>
          <div class="stat-cell"><div class="stat-lbl">Heading</div><div class="stat-val amber" id="manual-angle">0&deg;</div></div>
          <div class="stat-cell"><div class="stat-lbl">Target</div><div class="stat-val amber" id="manual-target">--</div></div>
          <div class="stat-cell"><div class="stat-lbl">Left Motor</div><div class="stat-val" id="manual-left">0</div></div>
          <div class="stat-cell"><div class="stat-lbl">Right Motor</div><div class="stat-val" id="manual-right">0</div></div>
          <div class="stat-cell"><div class="stat-lbl">ESP32 State</div><div class="stat-val" id="manual-state">N/A</div></div>
          <div class="stat-cell"><div class="stat-lbl">Stuck</div><div class="stat-val" id="manual-stuck">NO</div></div>
          <div class="stat-cell"><div class="stat-lbl">Switch Hit</div><div class="stat-val" id="manual-switch-hit">--</div></div>
          <div class="stat-cell"><div class="stat-lbl">Samples</div><div class="stat-val teal" id="manual-sample-count">0</div></div>
          <div class="stat-cell wide"><div class="stat-lbl">Arduino State</div><div class="stat-val teal" id="manual-arduino-state">Ready</div></div>
        </div>
      </div>
    </div>
  </div>
</div>

<div id="auto" class="tab">
  <div class="auto-layout">
    <div class="auto-left">

      <div class="sec-label">Mission Parameters</div>
      <div class="mission-card">
        <div class="mission-card-head">Pre-Mission Setup</div>
        <div class="mission-grid">

          <div class="mission-cell">
            <div class="mc-label">No. of Samples</div>
            <select class="mc-select" id="missionSamples"
              onchange="setParam('NS', this.value)">
              <option value="1">1 sample</option>
              <option value="2">2 samples</option>
              <option value="3">3 samples</option>
              <option value="4">4 samples</option>
              <option value="5" selected>5 samples</option>
            </select>
          </div>

          <div class="mission-cell">
            <div class="mc-label">Sample Depth</div>
            <select class="mc-select" id="missionDepth"
              onchange="sendSamplingParam('SD', this.value)">
              <option value="0">0.0 m (surface)</option>
              <option value="0.2">0.2 m</option>
              <option value="0.5">0.5 m</option>
              <option value="1">1.0 m</option>
              <option value="1.5">1.5 m</option>
              <option value="2" selected>2.0 m</option>
            </select>
          </div>

          <div class="mission-cell">
            <div class="mc-label">Flush Time (s)</div>
            <div style="display:flex;align-items:center;gap:6px;">
              <input class="pinp" type="number" id="flushTime" min="0" max="60" value="30"
                style="flex:1;width:auto;font-size:17px;height:38px;">
              <button class="pset" onclick="sendSamplingParam('FT', document.getElementById('flushTime').value)"
                style="height:38px;font-size:13px;">Set</button>
            </div>
          </div>

          <div class="mission-cell">
            <div class="mc-label">Fill Time (s)</div>
            <div style="display:flex;align-items:center;gap:6px;">
              <input class="pinp" type="number" id="fillTime" min="0" max="60" value="30"
                style="flex:1;width:auto;font-size:17px;height:38px;">
              <button class="pset" onclick="sendSamplingParam('BT', document.getElementById('fillTime').value)"
                style="height:38px;font-size:13px;">Set</button>
            </div>
          </div>

          <div class="mission-cell wide">
            <div class="mc-label">Sample Interval (min)</div>
            <div style="display:flex;align-items:center;gap:8px;">
              <button class="padj" onclick="adjustInterval(-1)">-</button>
              <input class="pinp" type="number" id="sampleIntervalAuto" min="1" max="60" value="3"
                style="flex:1;width:auto;font-size:19px;height:42px;"
                oninput="syncInterval('auto')">
              <button class="padj" onclick="adjustInterval(1)">+</button>
              <button class="pset" onclick="commitInterval()" style="height:42px;font-size:15px;">Set</button>
            </div>
          </div>

        </div>
      </div>

      <div class="auto-btns">
        <button class="abtn-auto" onclick="startAuto()">&#9654;&ensp;Start Auto Mode</button>
        <button class="abtn-stuck" onclick="sendCommand('CL')">&#11013;&ensp;Stuck Left</button>
        <button class="abtn-stuck" onclick="sendCommand('CR')">Stuck Right&ensp;&#10145;</button>
      </div>

    </div>

    <div class="auto-right">

      <div class="sec-label">Mission Timers</div>
      <div class="timers-card">
        <div class="timers-card-head">Live Mission Clock</div>
        <div class="timers-grid">
          <div class="timer-cell">
            <div class="timer-label">Elapsed Time</div>
            <div class="timer-value stopped" id="timer-elapsed">00:00:00</div>
            <div class="timer-sub" id="timer-elapsed-sub">HH : MM : SS</div>
          </div>
          <div class="timer-cell">
            <div class="timer-label">Next Sample In</div>
            <div class="timer-value stopped" id="timer-countdown">--:--</div>
            <div class="timer-sub" id="timer-countdown-sub">MM : SS</div>
          </div>
        </div>
      </div>

      <div class="sec-label">Telemetry</div>
      <div class="status-card">
        <div class="status-card-head">System Status</div>
        <div class="stat-grid">
          <div class="stat-cell"><div class="stat-lbl">Mode</div><div class="stat-val" id="auto-mode">--</div></div>
          <div class="stat-cell"><div class="stat-lbl">Heading</div><div class="stat-val amber" id="auto-angle">0&deg;</div></div>
          <div class="stat-cell"><div class="stat-lbl">Target</div><div class="stat-val amber" id="auto-target">--</div></div>
          <div class="stat-cell"><div class="stat-lbl">Left Motor</div><div class="stat-val" id="auto-left">0</div></div>
          <div class="stat-cell"><div class="stat-lbl">Right Motor</div><div class="stat-val" id="auto-right">0</div></div>
          <div class="stat-cell"><div class="stat-lbl">ESP32 State</div><div class="stat-val" id="auto-state">N/A</div></div>
          <div class="stat-cell"><div class="stat-lbl">Stuck</div><div class="stat-val" id="auto-stuck">NO</div></div>
          <div class="stat-cell"><div class="stat-lbl">Switch Hit</div><div class="stat-val" id="auto-switch-hit">--</div></div>
          <div class="stat-cell"><div class="stat-lbl">Samples</div><div class="stat-val teal" id="auto-sample-count">0</div></div>
          <div class="stat-cell wide"><div class="stat-lbl">Arduino State</div><div class="stat-val teal" id="auto-arduino-state">Ready</div></div>
        </div>
      </div>

    </div>
  </div>
</div>

<div id="params" class="tab">
  <div class="params-layout">
    <div>
      <div class="sec-label">Drive</div>
      <div class="param-row"><span class="pname">Motor Power</span><button class="padj" onclick="updateParam('motorPower',-10)">-</button><input class="pinp" type="number" id="motorPower" min="0" max="255" value="80"><button class="padj" onclick="updateParam('motorPower',10)">+</button><button class="pset" onclick="setParam('S',document.getElementById('motorPower').value)">Set</button></div>
      <div class="param-row"><span class="pname">Reverse Power</span><button class="padj" onclick="updateParam('reversePower',-5)">-</button><input class="pinp" type="number" id="reversePower" min="0" max="255" value="60"><button class="padj" onclick="updateParam('reversePower',5)">+</button><button class="pset" onclick="setParam('R',document.getElementById('reversePower').value)">Set</button></div>
      <div class="param-row"><span class="pname">Turn Power</span><button class="padj" onclick="updateParam('turnPower',-5)">-</button><input class="pinp" type="number" id="turnPower" min="0" max="255" value="30"><button class="padj" onclick="updateParam('turnPower',5)">+</button><button class="pset" onclick="setParam('P',document.getElementById('turnPower').value)">Set</button></div>
      <div class="param-row"><span class="pname">Motor Offset</span><button class="padj" onclick="updateParam('motorOffset',-0.1)">-</button><input class="pinp" type="number" id="motorOffset" min="0.5" max="2.0" step="0.1" value="1.0"><button class="padj" onclick="updateParam('motorOffset',0.1)">+</button><button class="pset" onclick="setParam('X',document.getElementById('motorOffset').value)">Set</button></div>

      <div class="sec-label">Stuck Recovery</div>
      <div class="param-row"><span class="pname">Stop Time 1 (ms)</span><button class="padj" onclick="updateParam('stuckStop1',-10)">-</button><input class="pinp" type="number" id="stuckStop1" min="50" max="1000" value="400"><button class="padj" onclick="updateParam('stuckStop1',10)">+</button><button class="pset" onclick="setParam('K',document.getElementById('stuckStop1').value)">Set</button></div>
      <div class="param-row"><span class="pname">Reverse Time (ms)</span><button class="padj" onclick="updateParam('stuckReverse',-10)">-</button><input class="pinp" type="number" id="stuckReverse" min="50" max="1000" value="2000"><button class="padj" onclick="updateParam('stuckReverse',10)">+</button><button class="pset" onclick="setParam('D',document.getElementById('stuckReverse').value)">Set</button></div>
      <div class="param-row"><span class="pname">Stop Time 2 (ms)</span><button class="padj" onclick="updateParam('stuckStop2',-10)">-</button><input class="pinp" type="number" id="stuckStop2" min="50" max="1000" value="400"><button class="padj" onclick="updateParam('stuckStop2',10)">+</button><button class="pset" onclick="setParam('B',document.getElementById('stuckStop2').value)">Set</button></div>
      <div class="param-row"><span class="pname">Final Stop (ms)</span><button class="padj" onclick="updateParam('stuckFinal',-10)">-</button><input class="pinp" type="number" id="stuckFinal" min="50" max="1000" value="400"><button class="padj" onclick="updateParam('stuckFinal',10)">+</button><button class="pset" onclick="setParam('O',document.getElementById('stuckFinal').value)">Set</button></div>
    </div>
    <div>
      <div class="sec-label">Navigation Timing</div>
      <div class="param-row"><span class="pname">Debounce (ms)</span><button class="padj" onclick="updateParam('debounce',-1)">-</button><input class="pinp" type="number" id="debounce" min="0" max="50" value="30"><button class="padj" onclick="updateParam('debounce',1)">+</button><button class="pset" onclick="setParam('E',document.getElementById('debounce').value)">Set</button></div>
      <div class="param-row"><span class="pname">Min Turn Angle (&deg;)</span><button class="padj" onclick="updateParam('minTurnAngle',-5)">-</button><input class="pinp" type="number" id="minTurnAngle" min="5" max="180" value="30"><button class="padj" onclick="updateParam('minTurnAngle',5)">+</button><button class="pset" onclick="setParam('MA',document.getElementById('minTurnAngle').value)">Set</button></div>
      <div class="param-row"><span class="pname">Max Turn Angle (&deg;)</span><button class="padj" onclick="updateParam('maxTurnAngle',-5)">-</button><input class="pinp" type="number" id="maxTurnAngle" min="5" max="180" value="90"><button class="padj" onclick="updateParam('maxTurnAngle',5)">+</button><button class="pset" onclick="setParam('XA',document.getElementById('maxTurnAngle').value)">Set</button></div>
      <div class="param-row"><span class="pname">Fwd Lock (ms)</span><button class="padj" onclick="updateParam('forwardLock',-100)">-</button><input class="pinp" type="number" id="forwardLock" min="0" max="10000" value="2000"><button class="padj" onclick="updateParam('forwardLock',100)">+</button><button class="pset" onclick="setParam('F',document.getElementById('forwardLock').value)">Set</button></div>

      <div class="sec-label">Heading Control</div>
      <div class="param-row"><span class="pname">Tolerance (&deg;)</span><button class="padj" onclick="updateParam('headingTol',-1)">-</button><input class="pinp" type="number" id="headingTol" min="1" max="30" value="5"><button class="padj" onclick="updateParam('headingTol',1)">+</button><button class="pset" onclick="setParam('TT',document.getElementById('headingTol').value)">Set</button></div>
      <div class="param-row"><span class="pname">Kp Gain</span><button class="padj" onclick="updateParam('headingKp',-0.1)">-</button><input class="pinp" type="number" id="headingKp" min="0.1" max="10" step="0.1" value="1.5"><button class="padj" onclick="updateParam('headingKp',0.1)">+</button><button class="pset" onclick="setParam('KP',document.getElementById('headingKp').value)">Set</button></div>
      <div class="param-row"><span class="pname">Turn Timeout (s)</span><button class="padj" onclick="updateParam('turnTimeout',-1)">-</button><input class="pinp" type="number" id="turnTimeout" min="1" max="30" value="8"><button class="padj" onclick="updateParam('turnTimeout',1)">+</button><button class="pset" onclick="setParam('TM',document.getElementById('turnTimeout').value)">Set</button></div>

      <div class="sec-label">Sampling</div>
      <div class="param-row">
        <span class="pname">Sample Interval (min)</span>
        <button class="padj" onclick="adjustInterval(-1)">-</button>
        <input class="pinp" type="number" id="sampleIntervalParams" min="1" max="60" value="3" oninput="syncInterval('params')">
        <button class="padj" onclick="adjustInterval(1)">+</button>
        <button class="pset" onclick="commitInterval()">Set</button>
      </div>

      <div class="sec-label">Water Sampler</div>
      <div class="param-row">
        <span class="pname">Sampler On Board</span>
        <button class="pset" id="wsOn"  onclick="toggleSampler(true)">ON</button>
        <button class="pset" id="wsOff" onclick="toggleSampler(false)">OFF</button>
      </div>
      <div class="param-row">
        <span class="pname">Skip Delay (s)</span>
        <button class="padj" onclick="updateParam('skipDelay',-1)">-</button>
        <input class="pinp" type="number" id="skipDelay" min="1" max="120" value="10">
        <button class="padj" onclick="updateParam('skipDelay',1)">+</button>
        <button class="pset" onclick="setParam('SST',document.getElementById('skipDelay').value)">Set</button>
      </div>
      <div class="param-row">
        <span class="pname">Reverse Duration (s)</span>
        <button class="padj" onclick="updateParam('rsdDelay',-1)">-</button>
        <input class="pinp" type="number" id="rsdDelay" min="0" max="30" value="7">
        <button class="padj" onclick="updateParam('rsdDelay',1)">+</button>
        <button class="pset" onclick="setParam('RSD',document.getElementById('rsdDelay').value)">Set</button>
      </div>
    </div>
  </div>
</div>

<script>
let autoRunning = false;
let elapsedAccum = 0;
let elapsedRunStart = 0;
let cdTotalSecs = 0;
let cdPaused = false;
let cdRunning = false;
let lastArduinoState = '';
let timerTick = null;

function openTab(n) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.tabnav button').forEach((b, i) =>
    b.classList.toggle('active', ['manual','auto','params'][i] === n));
  document.getElementById(n).classList.add('active');
  refresh();
}

function sendCommand(cmd) {
  fetch('/' + cmd).catch(() => {});
  refresh();
}

function globalStop() {
  sendCommand('S');
  stopTimers();
}

function engageManual() {
  sendCommand('M');
  stopTimers();
}

function armEsc() { sendCommand('Z'); }

function startAuto() {
  sendCommand('T');
  startTimers();
}

function startTimers() {
  autoRunning = true;
  elapsedAccum = 0;
  elapsedRunStart = Date.now();
  const intervalMins = parseInt(document.getElementById('sampleIntervalAuto').value) || 3;
  cdTotalSecs = intervalMins * 60;
  cdPaused = false;
  cdRunning = true;
  lastArduinoState = '';
  if (timerTick) clearInterval(timerTick);
  timerTick = setInterval(tickTimers, 1000);
  tickTimers();
}

function stopTimers() {
  if (autoRunning) {
    elapsedAccum += Date.now() - elapsedRunStart;
  }
  autoRunning = false;
  cdRunning = false;
  cdPaused = false;
  if (timerTick) { clearInterval(timerTick); timerTick = null; }
  renderElapsed(false);
  renderCountdown(false);
}

function tickTimers() {
  renderElapsed(true);
  if (!cdPaused && cdRunning) {
    if (cdTotalSecs > 0) cdTotalSecs--;
  }
  renderCountdown(true);
}

function renderElapsed(running) {
  let totalMs = elapsedAccum;
  if (running && autoRunning) totalMs += Date.now() - elapsedRunStart;
  const totalSecs = Math.floor(totalMs / 1000);
  const h = Math.floor(totalSecs / 3600);
  const m = Math.floor((totalSecs % 3600) / 60);
  const s = totalSecs % 60;
  const el = document.getElementById('timer-elapsed');
  el.textContent = pad(h) + ':' + pad(m) + ':' + pad(s);
  el.className = 'timer-value' + (running ? '' : ' stopped');
}

function renderCountdown(running) {
  const el = document.getElementById('timer-countdown');
  const sub = document.getElementById('timer-countdown-sub');
  if (!cdRunning) {
    el.textContent = '--:--';
    el.className = 'timer-value stopped';
    sub.textContent = 'MM : SS';
    return;
  }
  if (cdPaused) {
    el.textContent = 'SAMPLING';
    el.className = 'timer-value sampling';
    sub.textContent = 'Water collection';
    return;
  }
  const m = Math.floor(cdTotalSecs / 60);
  const s = cdTotalSecs % 60;
  el.textContent = pad(m) + ':' + pad(s);
  el.className = 'timer-value' + (running ? '' : ' stopped');
  sub.textContent = 'MM : SS';
}

function pad(n) { return String(n).padStart(2, '0'); }

function handleArduinoStateTransition(newState) {
  if (newState === lastArduinoState) return;
  const isSampling = newState === 'Auto Sampling...';
  const sampleDone = (newState === 'Sample Done');
  if (isSampling && cdRunning) cdPaused = true;
  if (sampleDone && cdRunning) {
    cdPaused = false;
    const intervalMins = parseInt(document.getElementById('sampleIntervalAuto').value) || 3;
    cdTotalSecs = intervalMins * 60;
  }
  lastArduinoState = newState;
}

function parseResponse(text) {
  const d = {mode:'--',left:0,right:0,angle:0,target:0,stuck:false,state:'N/A',switch_hit:'--',arduino_state:'Ready',sample_count:0};
  text.split('\n').forEach(l => {
    if (l.includes('MODE:'))         d.mode          = l.split('MODE:')[1].trim();
    if (l.includes('LEFT:'))         d.left          = parseInt(l.split('LEFT:')[1].trim()) || 0;
    if (l.includes('RIGHT:'))        d.right         = parseInt(l.split('RIGHT:')[1].trim()) || 0;
    if (l.includes('ANGLE:'))        d.angle         = parseInt(l.split('ANGLE:')[1].trim()) || 0;
    if (l.includes('TARGET:'))       d.target        = parseInt(l.split('TARGET:')[1].trim()) || 0;
    if (l.includes('STUCK:'))        d.stuck         = l.split('STUCK:')[1].trim().toLowerCase() === 'true';
    if (l.includes('STATE:'))        d.state         = l.split('STATE:')[1].trim();
    if (l.includes('SWITCH_HIT:'))   d.switch_hit    = l.split('SWITCH_HIT:')[1].trim();
    if (l.includes('ARDUINO:'))      d.arduino_state = l.split('ARDUINO:')[1].trim();
    if (l.includes('SAMPLE_COUNT:')) d.sample_count  = parseInt(l.split('SAMPLE_COUNT:')[1].trim()) || 0;
  });
  return d;
}

function refresh() {
  fetch('/status').then(r => r.text()).then(text => {
    const d = parseResponse(text);
    const p = document.getElementById('manual').classList.contains('active') ? 'manual' : 'auto';
    document.getElementById(p + '-mode').innerText          = d.mode;
    document.getElementById(p + '-left').innerText          = d.left;
    document.getElementById(p + '-right').innerText         = d.right;
    document.getElementById(p + '-angle').innerText         = d.angle + '\u00b0';
    document.getElementById(p + '-target').innerText        = d.target + '\u00b0';
    const se = document.getElementById(p + '-stuck');
    se.innerText   = d.stuck ? 'YES' : 'NO';
    se.className   = 'stat-val' + (d.stuck ? ' red' : '');
    document.getElementById(p + '-state').innerText         = d.state;
    document.getElementById(p + '-switch-hit').innerText    = d.switch_hit;
    document.getElementById(p + '-arduino-state').innerText = d.arduino_state;
    document.getElementById(p + '-sample-count').innerText  = d.sample_count;
    handleArduinoStateTransition(d.arduino_state);
  }).catch(() => {});
}

function syncInterval(source) {
  const val = document.getElementById(
    source === 'auto' ? 'sampleIntervalAuto' : 'sampleIntervalParams'
  ).value;
  const clamped = Math.max(1, Math.min(60, parseInt(val) || 1));
  document.getElementById('sampleIntervalAuto').value   = clamped;
  document.getElementById('sampleIntervalParams').value = clamped;
}

function adjustInterval(delta) {
  const inp = document.getElementById('sampleIntervalAuto');
  let v = parseInt(inp.value) + delta;
  v = Math.max(1, Math.min(60, v));
  document.getElementById('sampleIntervalAuto').value   = v;
  document.getElementById('sampleIntervalParams').value = v;
}

function commitInterval() {
  syncInterval('auto');
  const v = document.getElementById('sampleIntervalAuto').value;
  setParam('Y', v);
}

function sendSamplingParam(letter, value) {
  if (lastArduinoState === 'Ready' || lastArduinoState === 'Sample Done') {
    setParam(letter, value);
  } else {
    alert('Parameter not sent - Arduino must be in idle state (Ready or Sample Done).\nCurrent state: ' + lastArduinoState);
  }
}

function setParam(l, v) {
  fetch('/' + l + ':' + v).catch(() => {});
  localStorage.setItem(l, v);
}

function toggleSampler(on) {
  fetch('/WS:' + (on ? '1' : '0')).catch(() => {});
  localStorage.setItem('WS', on ? '1' : '0');
  document.getElementById('wsOn').style.background  = on  ? '#4CAF50' : '';
  document.getElementById('wsOff').style.background = !on ? '#f44336' : '';
}

function updateParam(id, d) {
  const i = document.getElementById(id);
  let v = parseFloat(i.value) + d;
  v = Math.max(parseFloat(i.min), Math.min(parseFloat(i.max), v));
  i.value = v.toFixed(i.step < 1 ? 1 : 0);
}

function loadParams() {
  const m = {
    motorPower:'S', reversePower:'R', turnPower:'P', motorOffset:'X',
    stuckStop1:'K', stuckReverse:'D', stuckStop2:'B', stuckFinal:'O',
    debounce:'E', minTurnAngle:'MA', maxTurnAngle:'XA', forwardLock:'F', sampleIntervalParams:'Y',
    flushTime:'FT', fillTime:'BT',
    headingTol:'TT', headingKp:'KP', turnTimeout:'TM'
  };
  Object.entries(m).forEach(([id, k]) => {
    const s = localStorage.getItem(k);
    if (s && document.getElementById(id)) document.getElementById(id).value = s;
  });
  const storedY  = localStorage.getItem('Y');
  if (storedY)  document.getElementById('sampleIntervalAuto').value = storedY;
  const storedNS = localStorage.getItem('NS');
  if (storedNS) document.getElementById('missionSamples').value = storedNS;
  const storedSD = localStorage.getItem('SD');
  if (storedSD) document.getElementById('missionDepth').value = storedSD;
  const storedWS  = localStorage.getItem('WS');
  if (storedWS !== null) toggleSampler(storedWS === '1');
  const storedSST = localStorage.getItem('SST');
  if (storedSST && document.getElementById('skipDelay'))
    document.getElementById('skipDelay').value = storedSST;
  const storedRSD = localStorage.getItem('RSD');
  if (storedRSD && document.getElementById('rsdDelay'))
    document.getElementById('rsdDelay').value = storedRSD;
}

const canvas = document.getElementById('joystick');
const ctx    = canvas.getContext('2d');
const R = 88, cx = canvas.width / 2, cy = canvas.height / 2;
let drag = false, curCmd = null, cmdInt = null;

function drawJoystick(x, y) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI);
  ctx.fillStyle = '#f0f5fb'; ctx.fill();
  ctx.strokeStyle = 'rgba(21,42,72,0.25)'; ctx.lineWidth = 1.5; ctx.stroke();
  ctx.strokeStyle = 'rgba(21,42,72,0.1)'; ctx.lineWidth = 1;
  [R * 0.33, R * 0.66].forEach(r => { ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI); ctx.stroke(); });
  ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();
  ctx.beginPath(); ctx.arc(x, y, R / 3.2, 0, 2 * Math.PI);
  ctx.fillStyle = 'rgba(26,79,138,0.15)'; ctx.fill();
  ctx.strokeStyle = '#1a4f8a'; ctx.lineWidth = 2; ctx.stroke();
  ctx.beginPath(); ctx.arc(x, y, 5, 0, 2 * Math.PI);
  ctx.fillStyle = '#8a5c00'; ctx.fill();
}

function getCmd(x, y) {
  const dx = x - cx, dy = y - cy, dist = Math.sqrt(dx * dx + dy * dy);
  if (dist < R * 0.28 || dist > R) return null;
  const a = Math.atan2(dy, dx) * 180 / Math.PI;
  if (a >= -135 && a < -45) return 'F';
  if (a >= 45  && a < 135)  return 'B';
  if (a >= 135 || a < -135) return 'L';
  return 'R';
}

function startCmd(cmd) {
  if (cmdInt) clearInterval(cmdInt);
  curCmd = cmd;
  if (cmd) { sendCommand(cmd); cmdInt = setInterval(() => sendCommand(cmd), 200); }
}

function handleMove(e) {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width, scaleY = canvas.height / rect.height;
  let x, y;
  if (e.type.startsWith('touch')) {
    x = (e.touches[0].clientX - rect.left) * scaleX;
    y = (e.touches[0].clientY - rect.top)  * scaleY;
  } else {
    x = (e.clientX - rect.left) * scaleX;
    y = (e.clientY - rect.top)  * scaleY;
  }
  const dx = x - cx, dy = y - cy, dist = Math.sqrt(dx * dx + dy * dy);
  if (dist > R) { const s = R / dist; x = cx + dx * s; y = cy + dy * s; }
  drawJoystick(x, y);
  const cmd = getCmd(x, y);
  if (cmd !== curCmd) startCmd(cmd);
}

function handleEnd() { drag = false; startCmd(null); drawJoystick(cx, cy); }

canvas.addEventListener('mousedown',  e => { drag = true; handleMove(e); });
canvas.addEventListener('mousemove',  e => { if (drag) handleMove(e); });
canvas.addEventListener('mouseup',    handleEnd);
canvas.addEventListener('touchstart', e => { drag = true; handleMove(e); }, { passive: false });
canvas.addEventListener('touchmove',  handleMove, { passive: false });
canvas.addEventListener('touchend',   handleEnd);

drawJoystick(cx, cy);
window.addEventListener('load', loadParams);
setInterval(refresh, 500);
</script>
</body>
</html>
)rawliteral";

// -------------------- Microswitches --------------------
const int leftSwitch1 = 13;
const int frontSwitch1 = 25;
const int rightSwitch1 = 26;

// -------------------- LEDs --------------------
const int ledRed = 27;
const int ledGreen = 12;
const int ledBlue = 33;
const int ledWhite = 32;

// -------------------- Forward Lock --------------------
unsigned long forwardLockStart = 0;

// -------------------- Debounce --------------------
unsigned long lastDebounceTime = 0;

// -------------------- Turn Vars (replaced by heading-nav, see top of file) --------------------

// -------------------- Sampling Vars --------------------
unsigned long sampleStart = 0;
int sample_count = 0;
int sample_max = 5;

// -------------------- Functions --------------------
void performAutoSample() {
    // 1. Safety Limit Check
    if (sample_count >= sample_max) {
        Serial.println("Max samples performed, Skipping sampling, exiting autonomous mode");
        autonomousMode = false;
        stopMode = true;
        stopCar();
        return;
    }

    // 2. Clear Buffer & Send Command
    while(Serial2.available()) { Serial2.read(); }

    Serial.println("Commanding water collection " + String(sample_count + 1) + "/5");
    Serial2.print(":M1\n");

    // Update Arduino State immediately
    arduinoState = "Auto Sampling...";

    // 3. The "Smart Wait" Loop
    String cmd = "";
    bool taskDone = false;
    unsigned long waitStart = millis();
    const unsigned long TIMEOUT_MS = 200000; // 200 Seconds — covers full cycle:
                                              // flush(30s) + depth(~67s) + pumps(30s)
                                              // + turns(~10s) + 63s safety margin

    // Reverse motors briefly to kill forward momentum
    Serial.println("Reversing motors for " + String(reverse_after_sample_duration) + "s at sample start");
    moveBackward();
    bool reverseHandled = false;

    Serial.println("Waiting for Arduino response (:X1)...");

    // Loop until Timeout OR Stop button is pressed
    while ((millis() - waitStart < TIMEOUT_MS) && !stopMode) {

        // CRITICAL: Keep GUI alive and responsive
        server.handleClient();

        // End reverse phase after configured duration
        if (!reverseHandled &&
            (millis() - waitStart) >= (reverse_after_sample_duration * 1000UL)) {
            stopCar();
            reverseHandled = true;
            Serial.println("Reverse phase complete");
        }

        if (Serial2.available()) {
            char c = Serial2.read();
            cmd += c;

            // Check for Newline (End of Command)
            if (c == '\n') {
                cmd.trim();

                if (cmd == ":X1") {  // Success Ack
                    Serial.println("Arduino: Received :X1 (Task Done)");
                    arduinoState = "Sample Done";
                    taskDone = true;
                    break;
                }
                else if (cmd == ":FULL") {
                    Serial.println("Arduino: Memory Full");
                    arduinoState = "Memory Full";
                }
                // --- Live status updates from Arduino state machine ---
                else if (cmd == "Starting auto sample cycle")         arduinoState = "Starting cycle";
                else if (cmd == "Turning to home position")           arduinoState = "Turning to home";
                else if (cmd == "Drum lowering to sample depth")      arduinoState = "Lowering drum";
                else if (cmd == "Drum at depth - settling before flush") arduinoState = "Drum at depth";
                else if (cmd == "Flushing at depth")                  arduinoState = "Flushing hose";
                else if (cmd == "Flush done - settling")              arduinoState = "Flush settling";
                else if (cmd.startsWith("Turning to bottle"))         arduinoState = "Turning to bottle";
                else if (cmd == "At bottle - settling before sample") arduinoState = "At bottle";
                else if (cmd == "Sampling bottle")                    arduinoState = "Sampling bottle";
                else if (cmd == "Sample done - settling")             arduinoState = "Sample settling";
                else if (cmd == "Returning turntable to home")        arduinoState = "Returning home";
                else if (cmd == "Turntable home - retracting drum")   arduinoState = "Retracting drum";
                else if (cmd == "Cycle complete - sent :X1")          arduinoState = "Cycle complete";

                cmd = "";
            }
        }
    }

    // 4. Handle Result
    if (taskDone) {
        sample_count++;
        sampleStart = millis();
        sampling = false;
        aligningAfterSample = true;     // Realign to targetHeading before resuming forward
        alignStart = millis();
    } else {
        if (stopMode) {
             Serial.println("Sampling Aborted by User (STOP).");
             arduinoState = "Aborted";
        } else {
             Serial.println("ERROR: Sampling Timeout! Did not receive :X1");
             arduinoState = "Timeout Error";
        }
        autonomousMode = false;
        stopCar();
        stopMode = true;
    }
}

void handleStuckNonBlocking() {
    unsigned long now = millis();
    if (stuckStep == 0) {
        Serial.println("Stuck stopping");
        stopCar();
        stuckStart = now;
        stuckPauseStart = now;          // Pause sample-interval clock for full stuck duration
        stuckStep = 1;
    } else if (stuckStep == 1 && now - stuckStart >= STUCK_STOP_TIME_1) {
        Serial.println("Stuck reversing");
        moveBackward();
        stuckStart = now;
        stuckStep = 2;
    } else if (stuckStep == 2 && now - stuckStart >= STUCK_REVERSE_TIME) {
        Serial.println("Stuck stopping");
        stopCar();
        stuckStart = now;
        stuckStep = 3;
    } else if (stuckStep == 3 && now - stuckStart >= STUCK_STOP_TIME_2) {
        Serial.println("Stuck turning to heading " + String(targetHeading, 1) + "°");
        stuckStart = now;
        stuckStep = 4;
    } else if (stuckStep == 4) {
        // Heading-controlled turn (closed-loop on magnetometer)
        readHeading();
        float err = headingError(targetHeading, currentHeading);
        bool reached  = fabs(err) < turn_tolerance_deg;
        bool timedOut = (now - stuckStart) >= turn_timeout_ms;
        if (reached || timedOut) {
            stopCar();
            stuckStart = now;
            stuckStep = 5;
            Serial.println(reached ? "Heading reached" : "Turn TIMEOUT");
            Serial.println("  err=" + String(err, 1) + "° actual=" + String(currentHeading, 1) + "°");
        } else {
            if (err > 0) turnRight();
            else         turnLeft();
        }
    } else if (stuckStep == 5 && now - stuckStart >= STUCK_FINAL_STOP_TIME) {
        stuckDetected = false;
        stuckStep = 0;  // Done, back to forward drive
        forwardLockStart = millis();
        if (stuckPauseStart > 0) {
            // Resume sample timer: shift sampleStart forward by the duration we were stuck.
            unsigned long pausedFor = now - stuckPauseStart;
            sampleStart += pausedFor;
            Serial.println("Sample timer paused during stuck for " + String(pausedFor / 1000) + "s");
            stuckPauseStart = 0;
        }
        Serial.println("Stuck ended");
    }
}

void setMotorPower(int leftPower, int rightPower) {
    leftPower  = constrain(leftPower,  -255, 255);
    rightPower = constrain(rightPower, -255, 255);
    rightPower = (int)(rightPower * Motor_Offset);
    rightPower = constrain(rightPower, -255, 255);

    int pulseL = 1105 + (leftPower  * (MAX_SAFE_PULSE - 1105) / 255);
    int pulseR = 1105 + (rightPower * (MAX_SAFE_PULSE - 1105) / 255);
    pulseL = constrain(pulseL, 1105, MAX_SAFE_PULSE);
    pulseR = constrain(pulseR, 1105, MAX_SAFE_PULSE);

    escA.writeMicroseconds(pulseL);
    escB.writeMicroseconds(pulseR);

    lastLeft  = leftPower;
    lastRight = rightPower;
}

void stopCar() {
    setMotorPower(0, 0);
}

void moveForwardAutonomous() {
    if (!escArmed) return;
    // Closed-loop P-controller: hold targetHeading by biasing motor differential.
    readHeading();
    float err        = headingError(targetHeading, currentHeading);
    float correction = heading_kp * err;  // err > 0 → need to turn right
    int leftPwr  = constrain((int)(Motor_Power + correction), 0, 255);
    int rightPwr = constrain((int)(Motor_Power - correction), 0, 255);
    escAReverse.writeMicroseconds(1200);
    escBReverse.writeMicroseconds(1200);
    setMotorPower(leftPwr, rightPwr);
    if (millis() - lastFwdPrint >= 2000) {
        Serial.println("AutoFwd target=" + String(targetHeading, 1) +
                       "° actual=" + String(currentHeading, 1) +
                       "° err=" + String(err, 1) +
                       "° L=" + String(leftPwr) + " R=" + String(rightPwr));
        lastFwdPrint = millis();
    }
}

void moveForward() {
    if (!escArmed) return;
    Serial.println("                FWD");
    escAReverse.writeMicroseconds(1200);
    escBReverse.writeMicroseconds(1200);
    setMotorPower(Motor_Power, Motor_Power);
}

void moveBackward() {
    if (!escArmed) return;
    escAReverse.writeMicroseconds(1900);
    escBReverse.writeMicroseconds(1900);
    setMotorPower(Reverse_Power, Reverse_Power);
    Serial.println("                  BWD");
}

void turnLeft() {
    if (!escArmed) return;
    Serial.println("                Left");
    escAReverse.writeMicroseconds(1900);
    escBReverse.writeMicroseconds(1200);
    setMotorPower(Motor_Power - Turn_Power, Motor_Power);
}

void turnRight() {
    if (!escArmed) return;
    Serial.println("               Right");
    escAReverse.writeMicroseconds(1200);
    escBReverse.writeMicroseconds(1900);
    setMotorPower(Motor_Power, Motor_Power - Turn_Power);
}

// -------------------- Command Handling --------------------
void handleCommand(char cmd) {
    if (cmd == 'S') {
        stopCar();
        stopMode = true;
        manualMode = false;
        autonomousMode = false;
        sampling = false;
        skipSampleStart = 0;
        sampleReverseHandled = false;
        aligningAfterSample = false;
        alignStart = 0;
        stuckPauseStart = 0;
        Serial2.print(":C1\n");  // Task 2
        return;
    }
    if (cmd == 'Z') {
        Serial.println("              Arming ESCs...");
        escArmed = false;
        escA.writeMicroseconds(1100);
        escB.writeMicroseconds(1100);
        escAReverse.writeMicroseconds(1100);
        escBReverse.writeMicroseconds(1100);
        //delay(2000);
        escArmed = true;
        Serial.println("                ESCs armed.");
        return;
    }
    if (stopMode) {
        if (cmd == 'M') {
            manualMode = true;
            autonomousMode = false;
            stopMode = false;
        } else if (cmd == 'T') {
            autonomousMode = true;
            manualMode = false;
            stopMode = false;
            sample_count = 0;
            sampleStart = millis();
            forwardLockStart = millis(); // Initialize so forward lock is active immediately
            sampling = false;
            // Capture current compass reading as locked target heading
            readHeading();
            targetHeading = currentHeading;
            Serial.println("Autonomous mode started — locked target heading = " + String(targetHeading, 1) + "°");
        }
    } else if (manualMode) {
        if (cmd == 'F') moveForward();
        else if (cmd == 'B') moveBackward();
        else if (cmd == 'L') turnLeft();
        else if (cmd == 'R') turnRight();
        else if (cmd == 'A') {
            Serial2.print(":M1\n");
            arduinoState = "Manual Sampling...";
        }
        else if (cmd == 'Q') Serial2.print(":M2\n");
        else if (cmd == 'I') Serial2.print(":B1\n");
        else if (cmd == 'D') Serial2.print(":D1\n");
        else if (cmd == 'G') Serial2.print(":E1\n");
        else if (cmd == 'H') Serial2.print(":F1\n");
        else if (cmd == '0') Serial2.print(":P0\n");
        else if (cmd == '1') Serial2.print(":P1\n");
        else if (cmd == '2') Serial2.print(":P2\n");
        else if (cmd == '3') Serial2.print(":P3\n");
        else if (cmd == '4') Serial2.print(":P4\n");
        else if (cmd == '5') Serial2.print(":P5\n");
    } else if (autonomousMode) {
        // Manual stuck triggers are now CL / CR (handled in handleCommandPath()).
        // The bare 'C' command has been removed in favour of explicit left/right triggers.
    }
}

// -------------------- HTTP Handlers --------------------
void handleRoot() {
    // Chunked send — required for iOS/mobile browsers with large HTML payloads
    // Single server.send() causes iOS to time out or hang on responses > ~4KB
    String htmlStr = String(html);
    size_t len = htmlStr.length();
    const size_t chunkSize = 2048;
    server.setContentLength(len);
    server.sendHeader("Content-Type", "text/html");
    server.sendHeader("Connection", "close");
    server.send(200);
    size_t offset = 0;
    while (offset < len) {
        size_t toSend = min(chunkSize, len - offset);
        server.sendContent(htmlStr.substring(offset, offset + toSend));
        offset += toSend;
    }
}

String getCurrentMode() {
    if (stopMode)      return "STOP";
    if (manualMode)    return "MANUAL";
    if (autonomousMode) return "AUTONOMOUS";
    return "Unknown";
}

String getState() {
    // Mirrors loop() priority: stuck > sampling > align > forward
    if (stuckDetected) {
        if (stuckStep == 1) return "STOP1";
        if (stuckStep == 2) return "REVERSE";
        if (stuckStep == 3) return "STOP2";
        if (stuckStep == 4) return "TURN→" + String((int)targetHeading) + "°";
        if (stuckStep == 5) return "FINAL STOP";
        return "STUCK";  // step 0 — transient, before first handleStuckNonBlocking() iteration
    }
    if (sampling) return "WATER SAMPLE";
    if (aligningAfterSample) return "ALIGN→" + String((int)targetHeading) + "°";
    if (millis() - forwardLockStart < forward_lock_ms) return "FORWARD LOCK " + String(forward_lock_ms) + "ms";
    return "FORWARD";
}

int checkSwitches() {
    unsigned long now = millis();
    if (now - lastDebounceTime < debounce_ms) return 0;

    if (digitalRead(leftSwitch1)  == LOW) return 1;
    if (digitalRead(frontSwitch1) == LOW) return 3;
    if (digitalRead(rightSwitch1) == LOW) return 5;

    return 0;
}

// -------------------- Magnetometer Heading Helpers --------------------
void readHeading() {
    sensors_event_t event;
    mag.getEvent(&event);
    float h = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    h += 4.0;  // Magnetic declination East
    if (h < 0)   h += 360;
    if (h >= 360) h -= 360;
    currentHeading = h;
}

float headingError(float target, float current) {
    float diff = target - current;
    while (diff > 180)  diff -= 360;
    while (diff < -180) diff += 360;
    return diff;  // -180..+180; positive = need to turn right (CW)
}

// Computes new target heading for stuck recovery turn.
// Right obstacle (5)  → subtract angle (turn LEFT, CCW)
// Left  obstacle (1)  → add angle      (turn RIGHT, CW)
// Front obstacle (3)  → add angle      (always turn RIGHT)
void computeStuckTurnTarget(int switchHit) {
    long randomAngle = random((long)min_turn_angle, (long)max_turn_angle + 1);
    float prevTarget = targetHeading;
    if (switchHit == 5) {
        targetHeading = targetHeading - (float)randomAngle;
    } else {  // switchHit == 1 or 3
        targetHeading = targetHeading + (float)randomAngle;
    }
    while (targetHeading >= 360) targetHeading -= 360;
    while (targetHeading < 0)    targetHeading += 360;
    Serial.println("Stuck switch=" + String(switchHit) + " angle=" + String(randomAngle)
                   + "° prev=" + String(prevTarget, 1) + "° new=" + String(targetHeading, 1) + "°");
}

void updateLED() {
    digitalWrite(ledRed,   LOW);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue,  LOW);
    digitalWrite(ledWhite, LOW);
    if (stopMode)      digitalWrite(ledWhite, HIGH);
    else if (manualMode) digitalWrite(ledGreen, HIGH);
    else if (autonomousMode) {
        if (stuckDetected) digitalWrite(ledRed,  HIGH);
        else               digitalWrite(ledBlue, HIGH);
    }
}

void handleStatus() {
    readHeading();

    String status = "";
    status += "MODE:"         + getCurrentMode()                      + "\n";
    status += "LEFT:"         + String(lastLeft)                      + "\n";
    status += "RIGHT:"        + String(lastRight)                     + "\n";
    status += "ANGLE:"        + String((int)currentHeading)           + "\n";
    status += "TARGET:"       + String((int)targetHeading)            + "\n";
    status += "STUCK:"        + String(stuckDetected ? "True":"False")+ "\n";
    status += "STATE:"        + getState()                            + "\n";
    status += "SWITCH_HIT:"   + String(checkSwitches())               + "\n";
    status += "ARDUINO:"      + arduinoState                          + "\n";
    status += "SAMPLE_COUNT:" + String(sample_count)                  + "\n";
    server.send(200, "text/plain", status);
}

// =====================================================================
//  CSV Mission Logger — LittleFS, append-only, 2 Hz, non-blocking
//  - File handle stays open for the whole autonomous mission
//  - Rows batched in a small String buffer; flushed every ~6 rows
//  - Sequence number persists across reboots via /log_counter.txt
//  - Entry/exit edges detected against autonomousMode flag inside
//    updateLogger() — no coupling to T / S / mode-exit handlers
// =====================================================================

// Read persistent log sequence number. Returns 0 if file missing.
uint32_t readLogCounter() {
    if (!LittleFS.exists(LOG_COUNTER_PATH)) return 0;
    File f = LittleFS.open(LOG_COUNTER_PATH, "r");
    if (!f) return 0;
    uint32_t n = (uint32_t)f.parseInt();
    f.close();
    return n;
}

// Persist current log sequence number BEFORE opening the file, so a
// crash mid-mission never causes filename reuse after reboot.
void writeLogCounter(uint32_t n) {
    File f = LittleFS.open(LOG_COUNTER_PATH, "w");
    if (!f) {
        Serial.println("LOG: failed to write counter");
        return;
    }
    f.printf("%u\n", (unsigned)n);
    f.close();
}

// "/log_NNN.csv" — zero-padded to 3 digits. Grows naturally past 999.
String makeLogPath(uint32_t n) {
    char buf[24];
    snprintf(buf, sizeof(buf), "/log_%03u.csv", (unsigned)n);
    return String(buf);
}

// Commit RAM buffer to flash. Cheap if buffer is empty.
void flushLogBuffer() {
    if (!logFile || logBuffer.length() == 0) return;
    logFile.print(logBuffer);
    logFile.flush();      // forces commit to flash — survives power loss from here
    logBuffer = "";
}

// Open a new log file at the next sequence number and write the header.
void startNewLog() {
    logSeqNum = readLogCounter() + 1;
    writeLogCounter(logSeqNum);              // persist BEFORE open
    String path = makeLogPath(logSeqNum);
    logFile = LittleFS.open(path, "w");
    if (!logFile) {
        Serial.println("LOG: failed to open " + path);
        return;
    }
    logFile.println(LOG_CSV_HEADER);
    logFile.flush();
    logBuffer = "";
    Serial.println("LOG: started " + path);
}

// Mission-end cleanup. Flush remaining buffer + close so FS is consistent.
void stopLog() {
    flushLogBuffer();
    if (logFile) {
        logFile.close();
        Serial.println("LOG: closed " + makeLogPath(logSeqNum));
    }
}

// Build one CSV row and append to buffer; flush if buffer is full.
// snprintf into a small stack buffer — no heap churn per row.
void writeLogRow() {
    if (!logFile) return;

    unsigned long now = millis();
    float runtimeSec  = (float)(now - autoStartMs) / 1000.0f;
    float err         = headingError(targetHeading, currentHeading);
    long  sampleRem   = (long)sample_interval_ms - (long)(now - sampleStart);
    if (sampleRem < 0) sampleRem = 0;

    char row[220];
    // ArduinoState wrapped in "..." in case a future state string contains a comma.
    snprintf(row, sizeof(row),
        "%lu,%.3f,%s,%s,%s,%d,\"%s\",%.1f,%.1f,%.1f,%d,%d,%ld\n",
        now,
        runtimeSec,
        getCurrentMode().c_str(),
        getState().c_str(),
        stuckDetected ? "true" : "false",
        sample_count,
        arduinoState.c_str(),
        targetHeading,
        currentHeading,
        err,
        lastLeft,
        lastRight,
        sampleRem);

    logBuffer += row;
    if (logBuffer.length() >= LOG_FLUSH_BYTES) flushLogBuffer();
}

// Single entry point called from loop(). Edge-detects autonomousMode
// transitions and gates the 2 Hz sample rate.
void updateLogger() {
    // Edge UP — autonomous just started
    if (autonomousMode && !loggerActive) {
        startNewLog();
        if (logFile) {
            loggerActive = true;
            autoStartMs  = millis();
            lastLogMs    = 0;       // force first row immediately
        }
        return;
    }
    // Edge DOWN — autonomous just ended (STOP, sample limit, mode change, etc)
    if (!autonomousMode && loggerActive) {
        stopLog();
        loggerActive = false;
        return;
    }
    // Steady-state — 2 Hz row write
    if (loggerActive && (millis() - lastLogMs >= 500)) {
        writeLogRow();
        lastLogMs = millis();
    }
}

// -------- HTTP: GET /logs → simple HTML index page --------
void handleLogList() {
    String out =
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>SRV-01 Logs</title><style>body{font-family:sans-serif;background:#0c1116;color:#e0e0e0;padding:20px}"
        "a{color:#4cb}li{margin:6px 0}</style></head><body><h2>SRV-01 Mission Logs</h2>";
    out += "<p>LittleFS: " + String(LittleFS.usedBytes()) + " / "
         + String(LittleFS.totalBytes()) + " bytes used</p><ul>";
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f) {
        String name = f.name();
        // LittleFS may return name with or without leading "/"
        String base = name.startsWith("/") ? name.substring(1) : name;
        if (base.startsWith("log_") && base.endsWith(".csv")) {
            out += "<li><a href='/" + base + "'>" + base + "</a>  ("
                 + String(f.size()) + " bytes)</li>";
        }
        f = root.openNextFile();
    }
    out += "</ul><p><a href='/'>&larr; Back to control</a></p></body></html>";
    server.send(200, "text/html", out);
}

void handleCommandPath() {
    String msg = server.uri().substring(1);
    msg.trim();

    // -------- Serve log files: /log_NNN.csv --------
    if (msg.startsWith("log_") && msg.endsWith(".csv")) {
        String path = "/" + msg;
        if (!LittleFS.exists(path)) { server.send(404, "text/plain", "Not found"); return; }
        File f = LittleFS.open(path, "r");
        if (!f) { server.send(500, "text/plain", "Open failed"); return; }
        server.streamFile(f, "text/csv");
        f.close();
        return;
    }

    if (msg.length() == 1) {
        handleCommand(msg[0]);
    } else {
        if (msg.startsWith("S:")) {
            Motor_Power = constrain(msg.substring(2).toFloat(), 0, 255);
            Serial.println("Motor Power: " + String(Motor_Power));
        } else if (msg.startsWith("P:")) {
            Turn_Power = constrain(msg.substring(2).toFloat(), 0, 255);
            Serial.println("Turn Power: " + String(Turn_Power));
        } else if (msg.startsWith("X:")) {
            Motor_Offset = constrain(msg.substring(2).toFloat(), 0.5, 2.0);
            Serial.println("Motor Offset: " + String(Motor_Offset));
        } else if (msg.startsWith("K:")) {
            STUCK_STOP_TIME_1 = constrain(msg.substring(2).toInt(), 50, 1000);
            Serial.println("STUCK_STOP_TIME_1: " + String(STUCK_STOP_TIME_1));
        } else if (msg.startsWith("D:")) {
            STUCK_REVERSE_TIME = constrain(msg.substring(2).toInt(), 50, 1000);
            Serial.println("STUCK_REVERSE_TIME: " + String(STUCK_REVERSE_TIME));
        } else if (msg.startsWith("B:")) {
            STUCK_STOP_TIME_2 = constrain(msg.substring(2).toInt(), 50, 1000);
            Serial.println("STUCK_STOP_TIME_2: " + String(STUCK_STOP_TIME_2));
        } else if (msg.startsWith("O:")) {
            STUCK_FINAL_STOP_TIME = constrain(msg.substring(2).toInt(), 50, 1000);
            Serial.println("STUCK_FINAL_STOP_TIME: " + String(STUCK_FINAL_STOP_TIME));
        } else if (msg.startsWith("E:")) {
            debounce_ms = constrain(msg.substring(2).toInt(), 0, 50);
            Serial.println("Debounce: " + String(debounce_ms));
        } else if (msg.startsWith("MA:")) {
            min_turn_angle = (unsigned long)constrain(msg.substring(3).toInt(), 5, 180);
            Serial.println("Min Turn Angle: " + String(min_turn_angle) + "°");
        } else if (msg.startsWith("XA:")) {
            max_turn_angle = (unsigned long)constrain(msg.substring(3).toInt(), 5, 180);
            Serial.println("Max Turn Angle: " + String(max_turn_angle) + "°");
        } else if (msg.startsWith("TT:")) {
            turn_tolerance_deg = constrain(msg.substring(3).toFloat(), 1.0, 30.0);
            Serial.println("Turn Tolerance: " + String(turn_tolerance_deg, 1) + "°");
        } else if (msg.startsWith("KP:")) {
            heading_kp = constrain(msg.substring(3).toFloat(), 0.1, 10.0);
            Serial.println("Heading Kp: " + String(heading_kp, 2));
        } else if (msg.startsWith("TM:")) {
            turn_timeout_ms = (unsigned long)constrain(msg.substring(3).toInt(), 1, 30) * 1000;
            Serial.println("Turn Timeout: " + String(turn_timeout_ms / 1000) + "s");
        } else if (msg.startsWith("F:")) {
            forward_lock_ms = msg.substring(2).toInt();
            Serial.println("Forward Lock: " + String(forward_lock_ms));
        } else if (msg.startsWith("Y:")) {
            sample_interval_ms = msg.substring(2).toInt() * 60000;
            Serial.println("Sample Interval: " + String(sample_interval_ms / 60000) + " min");
        } else if (msg.startsWith("R:")) {
            Reverse_Power = constrain(msg.substring(2).toFloat(), 0, 255);
            Serial.println("Reverse Power: " + String(Reverse_Power));
        } else if (msg.startsWith("NS:")) {
            sample_max = constrain(msg.substring(3).toInt(), 1, 5);
            Serial.println("Sample max: " + String(sample_max));
        } else if (msg.startsWith("SD:")) {
            sample_depth_m = constrain(msg.substring(3).toFloat(), 0.0, 2.0);
            Serial.println("Sample depth: " + String(sample_depth_m, 1) + "m");
            Serial2.print(":SD" + String(sample_depth_m, 1) + "\n");
        } else if (msg.startsWith("FT:")) {
            flush_time_ms = (unsigned long)constrain(msg.substring(3).toInt(), 0, 60) * 1000;
            Serial.println("Flush time: " + String(flush_time_ms / 1000) + "s");
            Serial2.print(":FT" + String(flush_time_ms / 1000) + "\n");
        } else if (msg.startsWith("BT:")) {
            fill_time_ms = (unsigned long)constrain(msg.substring(3).toInt(), 0, 60) * 1000;
            Serial.println("Fill time: " + String(fill_time_ms / 1000) + "s");
            Serial2.print(":BT" + String(fill_time_ms / 1000) + "\n");
        } else if (msg.startsWith("WS:")) {
            isWaterSamplerOnBoard = (msg.substring(3).toInt() == 1);
            Serial.println("Water Sampler on board: " + String(isWaterSamplerOnBoard ? "YES" : "NO"));
        } else if (msg.startsWith("SST:")) {
            skipSampleDelay = (unsigned long)constrain(msg.substring(4).toInt(), 1, 120) * 1000;
            Serial.println("Skip Sample Delay: " + String(skipSampleDelay / 1000) + "s");
        } else if (msg.startsWith("RSD:")) {
            reverse_after_sample_duration = (unsigned long)constrain(msg.substring(4).toInt(), 0, 30);
            Serial.println("Reverse after sample: " + String(reverse_after_sample_duration) + "s");
        } else if (msg == "CL") {
            // Manual stuck trigger — boat turns LEFT
            if (autonomousMode) {
                stuckDetected = true;
                computeStuckTurnTarget(5);  // 5 = right-switch semantics → boat turns LEFT (CCW)
                Serial.println("Manual stuck trigger: LEFT");
            }
        } else if (msg == "CR") {
            // Manual stuck trigger — boat turns RIGHT
            if (autonomousMode) {
                stuckDetected = true;
                computeStuckTurnTarget(1);  // 1 = left-switch semantics → boat turns RIGHT (CW)
                Serial.println("Manual stuck trigger: RIGHT");
            }
        }
    }
    server.send(200, "text/plain", "OK");
}

// -------------------- Setup --------------------
void setup() {
    Serial.begin(115200);
    escA.attach(pwmPinA, 1000, 2000);
    escB.attach(pwmPinB, 1000, 2000);
    escAReverse.attach(reverseAPin, 1100, 2000);
    escAReverse.writeMicroseconds(1100); // Default: reverse OFF
    escBReverse.attach(reverseBPin, 1100, 2000);
    escBReverse.writeMicroseconds(1100); // Default: reverse OFF

    pinMode(leftSwitch1,  INPUT_PULLUP);
    pinMode(frontSwitch1, INPUT_PULLUP);
    pinMode(rightSwitch1, INPUT_PULLUP);

    pinMode(ledRed,   OUTPUT);
    pinMode(ledGreen, OUTPUT);
    pinMode(ledBlue,  OUTPUT);
    pinMode(ledWhite, OUTPUT);

    stopCar();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.println("Wi-Fi AP Started. IP: " + WiFi.softAPIP().toString());
    Wire.begin();

    // LittleFS for mission logging — auto-format on mount failure.
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount FAILED — logger disabled");
    } else {
        Serial.println("LittleFS mounted. Total=" + String(LittleFS.totalBytes())
                       + " Used=" + String(LittleFS.usedBytes()));
    }

    if (!mag.begin()) {
        Serial.println("Magnetometer initialization failed!");
        while (1);
    }
    Serial.println("Magnetometer initialized.");

    server.on("/",       HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/logs",   HTTP_GET, handleLogList);
    server.onNotFound(handleCommandPath);
    server.begin();
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART to Arduino
}

// -------------------- Non-blocking Serial2 Reader --------------------
// Listens for Arduino responses in manual mode.
// In autonomous mode this is skipped — performAutoSample() owns Serial2 directly.
void readSerial2NonBlocking() {
    static String s2buf = "";
    while (Serial2.available()) {
        char c = Serial2.read();
        s2buf += c;
        if (c == '\n') {
            s2buf.trim();
            if (s2buf == ":X1")   arduinoState = "Sample Done";
            if (s2buf == ":FULL") arduinoState = "Memory Full";
            s2buf = "";
        }
    }
}

// -------------------- Main Loop --------------------
void loop() {
    server.handleClient();

    // Update LED
    updateLED();

    // Mission logger — non-blocking, 2 Hz, only active in autonomous mode.
    // Edge-detects autonomousMode transitions internally; no coupling to
    // motor/sample logic below.
    updateLogger();

    if (stopMode) {
        stopCar();
    } else if (manualMode) {
        if (millis() - lastFwdPrint >= 2000) {
            Serial.println("Manual mode active");
            lastFwdPrint = millis();
        }
        readSerial2NonBlocking(); // Catch :X1 / :FULL from Arduino in manual mode
    } else if (autonomousMode) {
        if (stuckDetected) {
            Serial.println("Stuck detected");
            handleStuckNonBlocking();
        } else if (sampling) {
            // NOTE: do NOT stopCar() here. Both branches below begin with moveBackward()
            // for `reverse_after_sample_duration` seconds (kills forward momentum BEFORE
            // the sample wait). A pre-stop here was overriding the no-sampler reverse on
            // every loop iteration and clobbering the real-Uno reverse on first entry.
            if (isWaterSamplerOnBoard) {
                performAutoSample();                          // calls moveBackward() internally
            } else {
                // Non-blocking simulated sample
                if (skipSampleStart == 0) {
                    if (sample_count >= sample_max) {         // mirror safety check in performAutoSample()
                        Serial.println("Max samples reached, exiting autonomous mode");
                        autonomousMode = false; stopMode = true; sampling = false;
                    } else {
                        moveBackward();                           // Start reverse phase
                        sampleReverseHandled = false;
                        skipSampleStart = millis();
                        arduinoState = "Auto Sampling...";        // GUI countdown pauses on this state
                        Serial.println("No sampler - reversing " + String(reverse_after_sample_duration)
                            + "s then simulating sample "
                            + String(sample_count + 1) + "/" + String(sample_max));
                    }
                } else {
                    // End reverse phase after configured duration
                    if (!sampleReverseHandled &&
                        (millis() - skipSampleStart) >= (reverse_after_sample_duration * 1000UL)) {
                        stopCar();
                        sampleReverseHandled = true;
                        Serial.println("Reverse phase complete (skip mode)");
                    }
                    // Self-ack when total delay elapsed
                    if ((millis() - skipSampleStart) >= skipSampleDelay) {
                        sample_count++;
                        sampleStart          = millis();
                        sampling             = false;
                        aligningAfterSample  = true;             // Realign to targetHeading before forward
                        alignStart           = millis();
                        skipSampleStart      = 0;
                        sampleReverseHandled = false;
                        arduinoState         = "Sample Done";  // GUI countdown resets on this state
                        Serial.println("Simulated sample done: "
                            + String(sample_count) + "/" + String(sample_max));
                        if (sample_count >= sample_max) {
                            Serial.println("All simulated samples complete, stopping");
                            autonomousMode = false; stopMode = true; stopCar();
                        }
                    }
                }   // end else (timing checks)
            }   // end else (!isWaterSamplerOnBoard)
        } else if (aligningAfterSample) {
            // Post-sample alignment: turn under closed-loop heading control until error
            // is within tolerance, then resume forward. Boat may have rotated during the
            // sample wait — this re-acquires the locked targetHeading before forward.
            readHeading();
            float err     = headingError(targetHeading, currentHeading);
            bool reached  = fabs(err) < turn_tolerance_deg;
            bool timedOut = (millis() - alignStart) >= turn_timeout_ms;
            if (reached || timedOut) {
                stopCar();
                aligningAfterSample = false;
                forwardLockStart    = millis();   // grace period before microswitches re-engage
                Serial.println(reached ? "Post-sample aligned" : "Post-sample align TIMEOUT");
                Serial.println("  err=" + String(err, 1) + "° actual=" + String(currentHeading, 1) + "°");
            } else {
                if (err > 0) turnRight();
                else         turnLeft();
            }
        } else {
            moveForwardAutonomous();
            int switchHit = checkSwitches();
            if (switchHit > 0 && millis() - forwardLockStart >= forward_lock_ms) {
                stuckDetected = true;
                computeStuckTurnTarget(switchHit);
                lastDebounceTime = millis();
            } else if (millis() - sampleStart >= sample_interval_ms) {
                sampling = true;
            }
        }
    }
}
