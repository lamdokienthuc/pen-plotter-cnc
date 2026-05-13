/* ================================================================
   ESP32 Pen Plotter Host v2.1
   FIX: Race condition upload — file giờ chạy được
   
   Thay đổi chính so với v2.0:
   - Dùng File handle global, mở 1 lần ở chunk đầu, đóng ở chunk cuối
   - Trigger job CHỈ khi (index + len >= total) — body đã ghi xong
   - Bỏ gFileReady (không cần nữa)
   - sendLineToPico có flush input buffer trước khi gửi
   ================================================================ */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

/* ================================================================
   CẤU HÌNH
   ================================================================ */
const char* WIFI_SSID     = "mywifi";
const char* WIFI_PASSWORD = "11111111";

#define PICO_SERIAL     Serial2
#define PICO_RX         16
#define PICO_TX         17
#define PICO_BAUD       115200
#define PICO_TIMEOUT_MS 120000

#define GCODE_FILE      "/job.gcode"
#define FW_VERSION      "v2.1"

/* ================================================================
   TRẠNG THÁI
   ================================================================ */
volatile float gX        = 0.0f;
volatile float gY        = 0.0f;
volatile bool  gRunning  = false;
volatile bool  gStopFlag = false;
String         gStatus   = "Idle";

SemaphoreHandle_t picoMutex;
QueueHandle_t     jobQueue;
QueueHandle_t     cmdQueue;

struct CmdJob { char line[192]; };

/* File handle global cho upload — KHÔNG mở/đóng mỗi chunk */
File uploadFile;
volatile bool uploadInProgress = false;

/* ================================================================
   WEB UI (giữ nguyên như v2.0)
   ================================================================ */
const char UI[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="vi"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Máy Vẽ</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0f0f0f;color:#ccc;padding:12px;max-width:480px;margin:auto}
h1{color:#00e676;font-size:15px;letter-spacing:3px;margin-bottom:12px;text-align:center}
.card{background:#181818;border:1px solid #252525;border-radius:10px;padding:12px;margin-bottom:10px}
.card h2{font-size:10px;color:#555;text-transform:uppercase;letter-spacing:2px;margin-bottom:10px}
.coords{display:flex;gap:8px}
.coord{flex:1;background:#0f0f0f;border:1px solid #252525;border-radius:6px;padding:8px;text-align:center}
.coord .v{font-size:22px;color:#00e676;font-weight:bold}
.coord .l{font-size:10px;color:#444;margin-top:2px}
#stbadge{display:inline-block;padding:3px 14px;border-radius:20px;font-size:11px;font-weight:bold;margin-top:8px}
.idle{background:#0d2e1a;color:#00e676}
.run {background:#2e1f00;color:#ffb300}
.err {background:#2e0d0d;color:#ff5252}
select{background:#0f0f0f;border:1px solid #2a2a2a;border-radius:6px;color:#ccc;padding:5px 8px;font-family:monospace;font-size:12px}
.jog-wrap{display:flex;flex-direction:column;align-items:center;gap:5px}
.jog-row{display:flex;gap:5px}
.jb{width:54px;height:54px;background:#1a1a1a;border:1px solid #2a2a2a;border-radius:8px;color:#ccc;font-size:22px;cursor:pointer;touch-action:manipulation;user-select:none;-webkit-user-select:none}
.jb:active{background:#003320;border-color:#00e676}
.opts{display:flex;gap:8px;margin-bottom:10px;align-items:center;flex-wrap:wrap}
.opts label{font-size:11px;color:#555}
.br{display:flex;gap:6px;flex-wrap:wrap;margin-top:8px}
.btn{padding:7px 14px;border:none;border-radius:6px;font-family:monospace;font-size:12px;font-weight:bold;cursor:pointer}
.btn:active{opacity:.7}
.g{background:#00e676;color:#000}.r{background:#ff1744;color:#fff}.b{background:#2979ff;color:#fff}.gr{background:#222;color:#888;border:1px solid #333}
textarea{width:100%;height:90px;background:#0f0f0f;border:1px solid #252525;border-radius:6px;color:#00e676;font-family:monospace;font-size:12px;padding:7px;resize:vertical}
input[type=file]{font-size:11px;color:#555;width:100%;margin-bottom:6px}
.pb-wrap{height:4px;background:#1a1a1a;border-radius:2px;margin-bottom:8px;overflow:hidden}
.pb{height:100%;width:0;background:#00e676;border-radius:2px;transition:width .4s}
#log{height:100px;overflow-y:auto;font-size:11px;background:#0a0a0a;border:1px solid #1a1a1a;border-radius:6px;padding:6px;line-height:1.6}
#log p{margin:0;color:#444}
#log .ok{color:#00e676}#log .er{color:#ff5252}#log .in{color:#ffb300}#log .db{color:#555}
.chip{font-size:10px;padding:2px 8px;border-radius:10px;margin-left:6px}
.esp{background:#1a2a3a;color:#448aff}.pico{background:#2a1a3a;color:#b388ff}
.ver{font-size:10px;color:#333;text-align:center;margin-top:4px}
</style></head><body>
<h1>&#9998; MÁY VẼ <span class="chip esp">ESP32</span><span class="chip pico">PICO</span></h1>
<div class="ver" id="ver">--</div>
<div class="card"><h2>Vị trí thực (từ Pico)</h2><div class="coords">
<div class="coord"><div class="v" id="vx">0.00</div><div class="l">X mm</div></div>
<div class="coord"><div class="v" id="vy">0.00</div><div class="l">Y mm</div></div></div>
<div style="text-align:center"><span id="stbadge" class="idle">Idle</span></div></div>
<div class="card"><h2>Jog thủ công</h2><div class="opts">
<label>Bước</label><select id="step"><option value="0.1">0.1 mm</option><option value="1" selected>1 mm</option><option value="10">10 mm</option><option value="50">50 mm</option></select>
<label>Tốc độ</label><select id="spd"><option value="1000">1000/min</option><option value="3000" selected>3000/min</option><option value="6000">6000/min</option></select></div>
<div class="jog-wrap"><div class="jog-row"><div style="width:54px"></div><button class="jb" onmousedown="jS(0,1)" onmouseup="jE()" onmouseleave="jE()" ontouchstart="jS(0,1)" ontouchend="jE()">↑</button></div>
<div class="jog-row"><button class="jb" onmousedown="jS(-1,0)" onmouseup="jE()" onmouseleave="jE()" ontouchstart="jS(-1,0)" ontouchend="jE()">←</button><button class="jb" style="font-size:12px;color:#2a2a2a;cursor:default">●</button><button class="jb" onmousedown="jS(1,0)" onmouseup="jE()" onmouseleave="jE()" ontouchstart="jS(1,0)" ontouchend="jE()">→</button></div>
<div class="jog-row"><div style="width:54px"></div><button class="jb" onmousedown="jS(0,-1)" onmouseup="jE()" onmouseleave="jE()" ontouchstart="jS(0,-1)" ontouchend="jE()">↓</button></div></div>
<div class="br"><button class="btn b" onclick="cmd('G28')">&#8962; Home</button><button class="btn gr" onclick="cmd('G92 X0 Y0')">Set Zero</button><button class="btn gr" onclick="cmd('M18')">Tắt motor</button><button class="btn r" onclick="doStop()">&#9632; STOP</button></div></div>
<div class="card"><h2>Upload file G-code</h2><input type="file" id="gf" accept=".gcode,.nc,.txt"><div class="pb-wrap"><div class="pb" id="pb"></div></div>
<div class="br"><button class="btn g" onclick="uploadFile()">&#9654; Upload &amp; Chạy</button><button class="btn gr" onclick="clearFile()">&#128465; Xóa</button><button class="btn r" onclick="doStop()">&#9632; STOP</button></div></div>
<div class="card"><h2>G-code trực tiếp</h2><textarea id="gc" placeholder="G1 X100 Y100 F6000&#10;G28"></textarea><div class="br"><button class="btn g" onclick="runGcode()">&#9654; Chạy</button><button class="btn gr" onclick="document.getElementById('gc').value=''">Xóa</button></div></div>
<div class="card"><h2>Log</h2><div id="log"></div></div>
<script>
let ws,jt=null;
function initWS(){ws=new WebSocket('ws://'+location.hostname+'/ws');ws.onopen=()=>lg('WS kết nối','ok');ws.onclose=()=>{lg('Mất kết nối...','er');setTimeout(initWS,2500)};ws.onmessage=e=>{try{const d=JSON.parse(e.data);if(d.x!==undefined)document.getElementById('vx').textContent=parseFloat(d.x).toFixed(2);if(d.y!==undefined)document.getElementById('vy').textContent=parseFloat(d.y).toFixed(2);if(d.ver)document.getElementById('ver').textContent='Firmware '+d.ver;if(d.status){const b=document.getElementById('stbadge');b.textContent=d.status;b.className=d.ready?'idle':(d.status.includes('Error')||d.status.includes('Timeout')||d.status.includes('Stop')?'err':'run');}}catch(ex){}};}initWS();
function lg(m,c=''){const el=document.getElementById('log');const p=document.createElement('p');if(c)p.className=c;p.textContent=new Date().toLocaleTimeString('vi')+' '+m;el.appendChild(p);el.scrollTop=el.scrollHeight;while(el.children.length>120)el.removeChild(el.firstChild);}
function post(url,body,cb){fetch(url,{method:'POST',body}).then(r=>r.text()).then(t=>{if(cb)cb(t);}).catch(e=>lg('POST '+url+' lỗi','er'));}
function cmd(g){if(!g.trim())return;lg('> '+g,'in');post('/cmd',g);}
function doStop(){fetch('/stop',{method:'POST'});lg('⬛ STOP','er');}
function clearFile(){post('/clear','',t=>lg('🗑 '+t,'ok'));}
function runGcode(){const g=document.getElementById('gc').value.trim();if(!g)return;lg('Gửi '+g.split('\n').length+' dòng','in');post('/gcode',g,()=>lg('Đang chạy','ok'));}
function uploadFile(){const f=document.getElementById('gf').files[0];if(!f){alert('Chọn file!');return;}const pb=document.getElementById('pb');pb.style.width='10%';lg('Upload: '+f.name+' ('+(f.size/1024).toFixed(1)+' KB)','in');const r=new FileReader();r.onload=e=>{pb.style.width='50%';post('/gcode',e.target.result,()=>{pb.style.width='100%';lg('Đang chạy: '+f.name,'ok');setTimeout(()=>pb.style.width='0%',2000);});};r.readAsText(f);}
let jogInterval=null;function jS(dx,dy){if(jogInterval)return;doJ(dx,dy);jogInterval=setInterval(()=>doJ(dx,dy),350);}function jE(){clearInterval(jogInterval);jogInterval=null;}
function doJ(dx,dy){const s=parseFloat(document.getElementById('step').value);const f=document.getElementById('spd').value;cmd('G91 G1 X'+(dx*s).toFixed(3)+' Y'+(dy*s).toFixed(3)+' F'+f+'\nG90');}
</script></body></html>)HTML";

/* ================================================================
   WEBSOCKET
   ================================================================ */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void broadcastStatus(const String& status) {
  gStatus = status;
  String j = "{\"status\":\"" + status   + "\""
             ",\"x\":"        + String(gX, 2) +
             ",\"y\":"        + String(gY, 2) +
             ",\"ready\":"    + (gRunning ? "false" : "true") +
             ",\"ver\":\""    + FW_VERSION + "\"}";
  ws.textAll(j);
}

/* ================================================================
   GIAO TIẾP PICO
   ================================================================ */
bool sendLineToPico(const String& rawLine) {
  if (gStopFlag) return false;

  String line = rawLine;
  line.trim();
  int sc = line.indexOf(';');
  if (sc >= 0) line = line.substring(0, sc);
  line.trim();
  if (line.length() == 0) return true;

  if (xSemaphoreTake(picoMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    Serial.println("[ESP] picoMutex timeout!");
    return false;
  }

  /* Xả buffer cũ trước khi gửi */
  while (PICO_SERIAL.available()) PICO_SERIAL.read();

  PICO_SERIAL.println(line);
  Serial.println("[->] " + line);

  unsigned long t0 = millis();
  bool gotOk = false;

  while (millis() - t0 < PICO_TIMEOUT_MS) {
    if (gStopFlag) break;

    if (PICO_SERIAL.available()) {
      String reply = PICO_SERIAL.readStringUntil('\n');
      reply.trim();
      if (reply.length() == 0) continue;

      Serial.println("[<-] " + reply);

      if (reply.startsWith("ok")) {
        int xi = reply.indexOf("X:");
        int yi = reply.indexOf(" Y:");
        if (xi >= 0 && yi > xi) {
          gX = reply.substring(xi + 2, yi).toFloat();
          gY = reply.substring(yi + 3).toFloat();
        }
        gotOk = true;
        break;
      } else if (reply.startsWith("error")) {
        xSemaphoreGive(picoMutex);
        broadcastStatus("Error: " + reply.substring(0, 30));
        return false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  xSemaphoreGive(picoMutex);

  if (!gotOk) {
    broadcastStatus("Timeout!");
    Serial.println("[ESP] Timeout: " + line);
    return false;
  }

  broadcastStatus(line.length() > 22 ? line.substring(0, 22) : line);
  return true;
}

/* ================================================================
   GCODE TASK
   ================================================================ */
void gcodeTask(void* param) {
  int sig;
  for (;;) {
    xQueueReceive(jobQueue, &sig, portMAX_DELAY);

    /* Verify file tồn tại và không rỗng */
    if (!SPIFFS.exists(GCODE_FILE)) {
      broadcastStatus("Error: no file");
      Serial.println("[gcode] File not found");
      continue;
    }

    File file = SPIFFS.open(GCODE_FILE, FILE_READ);
    if (!file) {
      broadcastStatus("Error: open fail");
      continue;
    }

    int total = file.size();
    if (total == 0) {
      file.close();
      broadcastStatus("Error: empty file");
      Serial.println("[gcode] Empty file");
      continue;
    }

    Serial.printf("[gcode] Job started, size=%d bytes\n", total);
    gStopFlag = false;
    gRunning  = true;
    broadcastStatus("Running");

    int pos     = 0;
    int lineNum = 0;
    int sentNum = 0;

    while (file.available() && !gStopFlag) {
      String line = file.readStringUntil('\n');
      pos += line.length() + 1;
      line.trim();
      lineNum++;

      if (line.length() == 0) continue;
      char first = line.charAt(0);
      if (first == ';' || first == '(') continue;

      if (!sendLineToPico(line)) {
        Serial.printf("[gcode] Abort at line %d\n", lineNum);
        break;
      }
      sentNum++;

      if (lineNum % 10 == 0 && total > 0) {
        int pct = (int)((float)pos / total * 100);
        broadcastStatus("Running " + String(pct) + "%");
      }

      vTaskDelay(1);
    }

    file.close();

    if (!gStopFlag) {
      sendLineToPico("G28");
      broadcastStatus("Idle");
      Serial.printf("[gcode] Done. %d/%d lines sent\n", sentNum, lineNum);
    } else {
      broadcastStatus("Stopped");
      Serial.println("[gcode] Stopped by user");
    }

    gRunning = false;
  }
}

/* ================================================================
   CMD TASK
   ================================================================ */
void cmdTask(void* param) {
  CmdJob job;
  for (;;) {
    if (xQueueReceive(cmdQueue, &job, portMAX_DELAY) != pdTRUE) continue;
    if (gRunning) continue;

    gRunning = true;

    String block = String(job.line);
    int start = 0;
    while (start < (int)block.length()) {
      int nl = block.indexOf('\n', start);
      String ln = (nl < 0) ? block.substring(start)
                           : block.substring(start, nl);
      ln.trim();
      if (ln.length() > 0) sendLineToPico(ln);
      start = (nl < 0) ? block.length() : nl + 1;
    }

    gRunning = false;
    broadcastStatus("Idle");
  }
}

/* ================================================================
   SERVER ROUTES
   ================================================================ */
void setupServer() {

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", UI);
  });

  /* ================================================================
     UPLOAD G-CODE — Đã sửa race condition
     - Chunk đầu (index==0): xóa file cũ, mở file handle
     - Mỗi chunk: write vào handle (KHÔNG đóng)
     - Chunk cuối (index+len >= total): đóng file, trigger job
     - onRequest chỉ trả "ok" cho HTTP, không trigger gì
     ================================================================ */
  server.on("/gcode", HTTP_POST,
    /* onRequest — body đã nhận xong toàn bộ, trigger job ở đây */
    [](AsyncWebServerRequest* req) {
      /* Đảm bảo file đóng (phòng trường hợp onBody chưa kịp đóng) */
      if (uploadFile) {
        uploadFile.close();
      }
      uploadInProgress = false;

      /* Verify file có tồn tại và > 0 byte */
      File chk = SPIFFS.open(GCODE_FILE, FILE_READ);
      size_t sz = chk ? chk.size() : 0;
      if (chk) chk.close();

      if (sz == 0) {
        req->send(400, "text/plain", "empty file");
        Serial.println("[ESP] Upload failed: empty");
        return;
      }

      Serial.printf("[ESP] Upload complete: %u bytes — queuing job\n", (unsigned)sz);
      req->send(200, "text/plain", "ok");

      int sig = 1;
      xQueueSend(jobQueue, &sig, 0);
    },
    NULL,
    /* onBody — nhận từng chunk */
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len,
       size_t index, size_t total) {

      if (index == 0) {
        /* Bắt đầu upload mới */
        if (gRunning) {
          Serial.println("[ESP] Upload rejected: job running");
          return;
        }
        SPIFFS.remove(GCODE_FILE);
        uploadFile = SPIFFS.open(GCODE_FILE, FILE_WRITE);
        if (!uploadFile) {
          Serial.println("[ESP] Cannot open file for write!");
          return;
        }
        uploadInProgress = true;
        Serial.printf("[ESP] Upload start: %u bytes total\n", (unsigned)total);
      }

      if (uploadFile && uploadInProgress) {
        size_t w = uploadFile.write(data, len);
        if (w != len) {
          Serial.printf("[ESP] Write short: %u/%u\n", (unsigned)w, (unsigned)len);
        }
      }

      if (index + len >= total) {
        /* Chunk cuối — flush + đóng file */
        if (uploadFile) {
          uploadFile.flush();
          uploadFile.close();
        }
        Serial.printf("[ESP] All chunks written: %u bytes\n", (unsigned)total);
      }
    }
  );

  /* Lệnh đơn */
  server.on("/cmd", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      req->send(200, "text/plain", "ok");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len,
       size_t index, size_t total) {
      if (gRunning) return;
      CmdJob job;
      int cpLen = min((int)len, 191);
      memcpy(job.line, data, cpLen);
      job.line[cpLen] = '\0';
      xQueueSend(cmdQueue, &job, 0);
    }
  );

  /* STOP */
  server.on("/stop", HTTP_POST, [](AsyncWebServerRequest* req) {
    gStopFlag = true;
    if (xSemaphoreTake(picoMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
      PICO_SERIAL.println("STOP");
      xSemaphoreGive(picoMutex);
    }
    req->send(200, "text/plain", "ok");
    Serial.println("[ESP] STOP");
  });

  /* Clear */
  server.on("/clear", HTTP_POST, [](AsyncWebServerRequest* req) {
    SPIFFS.remove(GCODE_FILE);
    req->send(200, "text/plain", "File cleared");
    Serial.println("[ESP] File cleared");
  });

  /* Info */
  server.on("/info", HTTP_GET, [](AsyncWebServerRequest* req) {
    size_t fileSize = 0;
    if (SPIFFS.exists(GCODE_FILE)) {
      File f = SPIFFS.open(GCODE_FILE, FILE_READ);
      if (f) { fileSize = f.size(); f.close(); }
    }
    String s = "{\"fw\":\"" + String(FW_VERSION) + "\""
               ",\"file_size\":"   + String(fileSize) +
               ",\"spiffs_free\":" + String(SPIFFS.totalBytes() - SPIFFS.usedBytes()) +
               ",\"heap\":"        + String(ESP.getFreeHeap()) +
               ",\"running\":"     + (gRunning ? "true" : "false") +
               ",\"uptime\":"      + String(millis() / 1000) + "}";
    req->send(200, "application/json", s);
  });

  /* WebSocket */
  ws.onEvent([](AsyncWebSocket* srv, AsyncWebSocketClient* client,
                AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      broadcastStatus(gStatus);
      Serial.printf("[WS] Client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  });

  server.addHandler(&ws);
  server.begin();
  Serial.println("[ESP] WebServer started");
}

/* ================================================================
   SETUP
   ================================================================ */
void setup() {
  Serial.begin(115200);
  PICO_SERIAL.begin(PICO_BAUD, SERIAL_8N1, PICO_RX, PICO_TX);

  Serial.println("\nESP32 Pen Plotter " + String(FW_VERSION) + " booting...");

  if (!SPIFFS.begin(true)) {
    Serial.println("[ESP] SPIFFS failed");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  SPIFFS.remove(GCODE_FILE);
  Serial.println("[ESP] SPIFFS OK, stale file cleared");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[ESP] Connecting WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[ESP] IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[ESP] WiFi failed");
  }

  picoMutex = xSemaphoreCreateMutex();
  jobQueue  = xQueueCreate(1, sizeof(int));
  cmdQueue  = xQueueCreate(4, sizeof(CmdJob));

  if (!picoMutex || !jobQueue || !cmdQueue) {
    Serial.println("[ESP] RTOS resource failed");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xTaskCreatePinnedToCore(gcodeTask, "gcode", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(cmdTask,   "cmd",   4096, NULL, 1, NULL, 0);

  setupServer();
  broadcastStatus("Idle");
  Serial.println("[ESP] Boot complete — " + String(FW_VERSION));
}

void loop() {
  ws.cleanupClients();
  vTaskDelay(pdMS_TO_TICKS(100));
}
