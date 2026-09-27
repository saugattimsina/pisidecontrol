document.addEventListener('DOMContentLoaded', () => {
    // Clock
    setInterval(() => {
        document.getElementById('clock').textContent = new Date().toLocaleTimeString('en-US', { hour12: false });
    }, 1000);

    // Asset Selection
    let currentDroneId = "1"; // Default to C-1
    const assetItems = document.querySelectorAll('.asset-item');
    const telDroneIdDisplay = document.getElementById('telemetry-drone-id');
    const cmdDroneIdDisplay = document.getElementById('command-drone-id');

    function selectAsset(id, name) {
        currentDroneId = id;
        telDroneIdDisplay.textContent = name;
        cmdDroneIdDisplay.textContent = name;
        
        assetItems.forEach(item => {
            if (item.getAttribute('data-id') === id) {
                item.classList.add('active');
            } else {
                item.classList.remove('active');
            }
        });
        
        // Clear telemetry when switching
        removeSkeletons();
        telArmed.textContent = '--';
        telMode.textContent = '--';
        telAlt.textContent = '--';
        telBaro.textContent = '--';
        telSats.textContent = '--';
        telLoc.textContent = '--';
        telGpsLock.textContent = 'WAITING';
        telGpsLock.className = 'tel-val text-red';

        // Auto fetch status
        sendCommand('status');
    }

    assetItems.forEach(item => {
        item.addEventListener('click', () => {
            selectAsset(item.getAttribute('data-id'), item.querySelector('.asset-name').textContent);
        });
    });

    // Telemetry Elements
    const telArmed = document.getElementById('tel-armed');
    const telMode = document.getElementById('tel-mode');
    const telAlt = document.getElementById('tel-alt');
    const telBaro = document.getElementById('tel-baro');
    const telSats = document.getElementById('tel-sats');
    const telLoc = document.getElementById('tel-loc');
    const telGpsLock = document.getElementById('tel-gps-lock');
    
    // Log & Loader
    const terminal = document.getElementById('terminal-log');
    const spinner = document.getElementById('loading-spinner');

    // Action Buttons Mapping
    document.querySelectorAll('.cmd-btn[data-cmd]').forEach(btn => {
        btn.addEventListener('click', () => {
            sendCommand(btn.getAttribute('data-cmd'));
        });
    });

    // Dynamic Inputs
    document.getElementById('btn-takeoff')?.addEventListener('click', () => {
        const alt = document.getElementById('takeoff-alt').value || "20";
        sendCommand(`t ${alt}`);
    });

    document.getElementById('btn-down')?.addEventListener('click', () => {
        const m = document.getElementById('down-meters').value || "1";
        sendCommand(`d ${m}`);
    });

    document.getElementById('btn-area')?.addEventListener('click', () => {
        const px = document.getElementById('area-px').value || "3000";
        sendCommand(`area ${px}`);
    });

    document.getElementById('btn-set-param')?.addEventListener('click', () => {
        const param = document.getElementById('tune-param').value.trim();
        const val = document.getElementById('tune-val').value.trim();
        if (param && val !== "") {
            sendCommand(`set ${param} ${val}`);
        } else {
            log('ERR: LIVE TUNE REQ PARAM AND VALUE', 'error');
        }
    });

    // Logging
    function log(message, type) {
        const entry = document.createElement('div');
        entry.className = `log-entry ${type}`;
        const time = new Date().toLocaleTimeString([], {hour12:false});
        entry.textContent = `[${time}] ${message}`;
        terminal.appendChild(entry);
        terminal.scrollTop = terminal.scrollHeight;
    }

    // Clear Logs
    document.querySelector('.col-logs .small-btn').addEventListener('click', () => {
        terminal.innerHTML = '<div class="log-entry system">LOG CLEARED.</div>';
    });

    function removeSkeletons() {
        // Not using skeletons in tactical UI, just standard dashes
    }

    function parseTelemetry(rawStr) {
        if (!rawStr.includes("[Drone ")) return;
        
        const payload = rawStr.split('] ')[1];
        if (!payload) return;

        const parts = payload.split('|');
        const data = {};
        parts.forEach(p => {
            const [k, v] = p.split(':');
            if(k && v !== undefined) data[k] = v;
        });

        // Update UI
        if (data.A) {
            const isArmed = data.A === 'Y';
            telArmed.textContent = isArmed ? 'ARMED' : 'DISARMED';
            telArmed.className = `tel-val ${isArmed ? 'text-red' : 'text-green'}`;
        }
        if (data.M) telMode.textContent = data.M;
        
        if (data.Alt) telAlt.textContent = data.Alt;
        if (data.Baro) telBaro.textContent = data.Baro;
        
        if (data.Sats) {
            telSats.textContent = data.Sats;
            if (parseInt(data.Sats) > 5) {
                telGpsLock.textContent = 'LOCKED';
                telGpsLock.className = 'tel-val text-green';
            }
        }
        if (data.Loc) telLoc.textContent = data.Loc;
    }

    async function sendCommand(commandAction) {
        if (!currentDroneId) return;
        const fullCmd = `${currentDroneId}-${commandAction}`;
        
        spinner.classList.remove('hidden');
        document.querySelectorAll('.cmd-btn').forEach(b => b.disabled = true);
        
        log(`TX // ${fullCmd}`, 'tx');

        try {
            const response = await fetch('/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: fullCmd })
            });

            const data = await response.json();

            if (response.ok) {
                log(`RX // ${data.response}`, 'rx');
                parseTelemetry(data.response);
            } else if (response.status === 408) {
                log(`ERR // TIMEOUT - NO RESPONSE FROM C-${currentDroneId}`, 'error');
            } else {
                log(`ERR // ${data.error || 'UNKNOWN SERVER ERROR'}`, 'error');
            }
        } catch (error) {
            log(`ERR // OFFLINE - FAILED TO CONNECT TO RELAY`, 'error');
        } finally {
            spinner.classList.add('hidden');
            document.querySelectorAll('.cmd-btn').forEach(b => b.disabled = false);
        }
    }

    // Init
    selectAsset("1", "C-1");
});
