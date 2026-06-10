// Web interface HTML/CSS/JS для управления роботом джойстиком

const char* web_interface = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Control</title>
  <style>
    body { font-family: Arial; margin: 0; padding: 20px; background: #1a1a1a; color: #fff; }
    .container { max-width: 800px; margin: 0 auto; }
    h1 { text-align: center; color: #00ff00; }

    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin: 20px 0; }

    .panel {
      background: #2a2a2a;
      border: 2px solid #00ff00;
      border-radius: 10px;
      padding: 20px;
    }

    .joystick-container {
      width: 300px;
      height: 300px;
      background: #111;
      border: 3px solid #00ff00;
      border-radius: 150px;
      position: relative;
      margin: 0 auto;
      touch-action: none;
    }

    .joystick-bg { width: 100%; height: 100%; border-radius: 150px; }

    .joystick-dot {
      width: 60px;
      height: 60px;
      background: #00ff00;
      border-radius: 50%;
      position: absolute;
      top: 120px;
      left: 120px;
      cursor: grab;
      box-shadow: 0 0 20px #00ff00;
    }

    .joystick-dot:active { cursor: grabbing; }

    .sensor-display {
      background: #111;
      border: 2px solid #00ff00;
      border-radius: 8px;
      padding: 15px;
      margin: 10px 0;
      font-family: monospace;
      font-size: 14px;
    }

    .sensor-row {
      display: flex;
      justify-content: space-between;
      margin: 8px 0;
      padding: 5px;
      background: #1a1a1a;
      border-radius: 4px;
    }

    .sensor-label { color: #00ff00; font-weight: bold; }
    .sensor-value { color: #ffff00; }

    .line-sensors {
      display: flex;
      gap: 5px;
      margin: 10px 0;
      justify-content: center;
    }

    .sensor-bar {
      width: 30px;
      height: 100px;
      background: #333;
      border: 1px solid #00ff00;
      border-radius: 4px;
      position: relative;
      overflow: hidden;
    }

    .sensor-fill {
      width: 100%;
      height: 100%;
      background: linear-gradient(180deg, #00ff00 0%, #ffff00 50%, #ff6600 100%);
      position: absolute;
      bottom: 0;
      transition: height 0.1s;
    }

    .control-buttons {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin: 15px 0;
    }

    button {
      padding: 15px;
      font-size: 16px;
      font-weight: bold;
      border: 2px solid #00ff00;
      background: #1a1a1a;
      color: #00ff00;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.3s;
    }

    button:hover { background: #00ff00; color: #000; box-shadow: 0 0 20px #00ff00; }
    button:active { transform: scale(0.95); }
    button.disabled { opacity: 0.5; cursor: not-allowed; }

    .status {
      text-align: center;
      font-weight: bold;
      padding: 10px;
      margin: 10px 0;
      border-radius: 8px;
      background: #2a2a2a;
      border: 2px solid #00ff00;
    }

    .status.idle { color: #ffff00; }
    .status.running { color: #00ff00; }
    .status.stopped { color: #ff0000; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🤖 Robot Control</h1>

    <div class="grid">
      <!-- Управление -->
      <div class="panel">
        <h2>Управление</h2>
        <div class="joystick-container">
          <div class="joystick-dot" id="joystick"></div>
        </div>
        <div style="margin-top: 20px;">
          <div class="control-buttons">
            <button onclick="startStraight()">▶ Прямо</button>
            <button onclick="startLine()">〰 Линия</button>
            <button onclick="stopRobot()">⏹ Стоп</button>
            <button onclick="getStatus()">📊 Статус</button>
          </div>
        </div>

        <!-- Управление захватом -->
        <div style="margin-top: 20px;">
          <h3 style="color: #00ff00; margin-top: 0;">🤖 Захват</h3>
          <div class="control-buttons">
            <button onclick="gripperOpen()" style="background: #0099ff;">✋ Открыть</button>
            <button onclick="gripperClose()" style="background: #ff6600;">✌️ Закрыть</button>
          </div>
        </div>

        <div class="status idle" id="statusDisplay">IDLE</div>
      </div>

      <!-- Датчики -->
      <div class="panel">
        <h2>Датчики</h2>

        <!-- Датчик расстояния -->
        <div class="sensor-display">
          <div class="sensor-label">📏 Расстояние (мм)</div>
          <div class="sensor-row">
            <span class="sensor-label">Дистанция:</span>
            <span class="sensor-value" id="distance">--</span>
          </div>
        </div>

        <!-- Датчик линии -->
        <div class="sensor-display">
          <div class="sensor-label">🔍 Датчик линии</div>
          <div class="line-sensors" id="lineSensors">
            <div class="sensor-bar"><div class="sensor-fill" id="sensor0" style="height: 0%"></div></div>
            <div class="sensor-bar"><div class="sensor-fill" id="sensor1" style="height: 0%"></div></div>
            <div class="sensor-bar"><div class="sensor-fill" id="sensor2" style="height: 0%"></div></div>
            <div class="sensor-bar"><div class="sensor-fill" id="sensor3" style="height: 0%"></div></div>
            <div class="sensor-bar"><div class="sensor-fill" id="sensor4" style="height: 0%"></div></div>
          </div>
          <div class="sensor-row">
            <span class="sensor-label">Ошибка:</span>
            <span class="sensor-value" id="lineError">0</span>
          </div>
          <div class="sensor-row">
            <span class="sensor-label">На линии:</span>
            <span class="sensor-value" id="onLine">НЕТ</span>
          </div>
        </div>

        <!-- Основные параметры -->
        <div class="sensor-display">
          <div class="sensor-label">⚙ Параметры</div>
          <div class="sensor-row">
            <span class="sensor-label">Угол:</span>
            <span class="sensor-value" id="angle">0.0°</span>
          </div>
          <div class="sensor-row">
            <span class="sensor-label">На уклоне:</span>
            <span class="sensor-value" id="onSlope">НЕТ</span>
          </div>
          <div class="sensor-row">
            <span class="sensor-label">Throttle:</span>
            <span class="sensor-value" id="throttle">0</span>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const joystick = document.getElementById('joystick');
    const container = joystick.parentElement;
    let isPressed = false;

    // Джойстик логика
    function handleJoystickMove(e) {
      if (!isPressed) return;

      const rect = container.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      const radius = centerY - 30;

      let x = e.touches ? e.touches[0].clientX - rect.left : e.clientX - rect.left;
      let y = e.touches ? e.touches[0].clientY - rect.top : e.clientY - rect.top;

      x -= centerX;
      y -= centerY;

      const distance = Math.sqrt(x * x + y * y);
      if (distance > radius) {
        const angle = Math.atan2(y, x);
        x = Math.cos(angle) * radius;
        y = Math.sin(angle) * radius;
      }

      joystick.style.left = (centerX + x - 30) + 'px';
      joystick.style.top = (centerY + y - 30) + 'px';

      // Отправить команду на робот
      const throttle = Math.round(-y / radius * 200);
      const steering = Math.round(x / radius * 100);
      sendJoystickCommand(throttle, steering);
    }

    joystick.addEventListener('mousedown', () => isPressed = true);
    joystick.addEventListener('touchstart', () => isPressed = true);
    document.addEventListener('mouseup', resetJoystick);
    document.addEventListener('touchend', resetJoystick);
    document.addEventListener('mousemove', handleJoystickMove);
    document.addEventListener('touchmove', handleJoystickMove);

    function resetJoystick() {
      isPressed = false;
      joystick.style.left = '120px';
      joystick.style.top = '120px';
      sendJoystickCommand(0, 0);
    }

    // API команды
    async function sendJoystickCommand(throttle, steering) {
      try {
        const response = await fetch(`/api/joystick?throttle=${throttle}&steering=${steering}`);
        updateDisplay(await response.json());
      } catch (e) {
        console.log('Connection error:', e);
      }
    }

    async function startStraight() {
      try {
        const response = await fetch('/api/command?cmd=straight');
        updateDisplay(await response.json());
      } catch (e) { console.log('Error:', e); }
    }

    async function startLine() {
      try {
        const response = await fetch('/api/command?cmd=line');
        updateDisplay(await response.json());
      } catch (e) { console.log('Error:', e); }
    }

    async function stopRobot() {
      try {
        const response = await fetch('/api/command?cmd=stop');
        updateDisplay(await response.json());
      } catch (e) { console.log('Error:', e); }
    }

    async function gripperOpen() {
      try {
        const response = await fetch('/api/command?cmd=gripper_open');
        updateDisplay(await response.json());
      } catch (e) { console.log('Error:', e); }
    }

    async function gripperClose() {
      try {
        const response = await fetch('/api/command?cmd=gripper_close');
        updateDisplay(await response.json());
      } catch (e) { console.log('Error:', e); }
    }

    async function getStatus() {
      try {
        const response = await fetch('/api/status');
        updateDisplay(await response.json());
      } catch (e) { console.log('Error:', e); }
    }

    function updateDisplay(data) {
      if (!data) return;

      // Обновить статус
      const statusDiv = document.getElementById('statusDisplay');
      const states = ['IDLE', 'STRAIGHT', 'LINE', 'STOPPED'];
      statusDiv.textContent = states[data.state] || 'UNKNOWN';
      statusDiv.className = 'status ' + (data.state === 0 ? 'idle' : data.state === 3 ? 'stopped' : 'running');

      // Датчик расстояния
      document.getElementById('distance').textContent = data.distance || '--';

      // Датчик линии
      if (data.sensors) {
        for (let i = 0; i < 5; i++) {
          const fill = document.getElementById('sensor' + i);
          if (fill) fill.style.height = ((data.sensors[i] || 0) / 200 * 100) + '%';
        }
      }

      // Параметры
      document.getElementById('lineError').textContent = data.lineError || '0';
      document.getElementById('onLine').textContent = (data.distance < 150) ? 'ДА' : 'НЕТ';
      document.getElementById('angle').textContent = (data.angle || 0).toFixed(1) + '°';
      document.getElementById('onSlope').textContent = data.slope ? 'ДА' : 'НЕТ';
      document.getElementById('throttle').textContent = data.throttle || '0';
    }

    // Обновлять данные каждую секунду
    setInterval(getStatus, 1000);
  </script>
</body>
</html>
)rawliteral";
