document.addEventListener('DOMContentLoaded', () => {
    // Clock
    setInterval(() => {
        document.getElementById('clock').textContent = new Date().toLocaleTimeString('en-US', { hour12: false });
    }, 1000);

    // Asset Selection
    let currentDroneId = "1";
    const drones = ["1", "2", "3"]; // The three assets on our roster
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
        
        // Clear telemetry when switching, the bg loop will fill it right back up
        telArmed.textContent = '--';
        telMode.textContent = '--';
        telAlt.textContent = '--';
        telBaro.textContent = '--';
        telSats.textContent = '--';
        telLoc.textContent = '--';
        telGpsLock.textContent = 'WAITING';
        telGpsLock.className = 'tel-val text-red';
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

    // Logging function (only for manual actions, not background polling!)
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
            } else {
                telGpsLock.textContent = 'WAITING';
                telGpsLock.className = 'tel-val text-red';
            }
        }
        if (data.Loc) telLoc.textContent = data.Loc;
        
        // If your C++ string sends battery like "Bat:95", we parse it here:
        if (data.Bat) {
            // Future expansion: hook this up to the visual battery progress bar
        }
    }

    // MANUAL COMMAND SENDER
    let isTransmittingManual = false;
    async function sendUserCommand(commandAction) {
        if (!currentDroneId) return;
        
        isTransmittingManual = true; // PAUSE BACKGROUND POLLING
        const fullCmd = `${currentDroneId}-${commandAction}`;
        
        spinner.classList.remove('hidden');
        // Never disable the red LAND buttons while waiting for a reply.
        document.querySelectorAll('.cmd-btn:not(.btn-danger)').forEach(b => b.disabled = true);
        
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
            } else if (response.status === 409) {
                log(`SKIPPED // ${data.message}`, 'error');
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
            
            // Wait a moment after manual command before resuming polls to let radio breathe
            setTimeout(() => { isTransmittingManual = false; }, 500); 
        }
    }

    // BACKGROUND LINK CHECK: ping the selected drone every 4 s.
    // Pings are LOW priority on the server: they are skipped (409) whenever a
    // real command is using the radio, and never delay one.
    const PING_INTERVAL_MS = 4000;
    const OFFLINE_AFTER_MISSES = 2;
    const missCount = {};
    let isPolling = false;

    function setBadge(droneId, text, cls) {
        const badge = document.querySelector(`.asset-item[data-id="${droneId}"] .asset-status`);
        if (badge) {
            badge.textContent = text;
            badge.className = `asset-status ${cls}`;
        }
    }

    async function pingSelectedDrone() {
        if (isPolling || isTransmittingManual || !currentDroneId) {
            setTimeout(pingSelectedDrone, PING_INTERVAL_MS);
            return;
        }
        isPolling = true;
        const droneId = currentDroneId;

        try {
            const response = await fetch('/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: `${droneId}-ping` })
            });
            const data = await response.json();

            if (response.status === 409) {
                // Skipped because a real command had the radio - not a failure.
            } else if (response.ok && data.response && data.response.includes('PONG')) {
                missCount[droneId] = 0;
                const armed = data.response.includes('A:Y');
                const rtt = data.rtt_ms ? ` ${(data.rtt_ms / 1000).toFixed(1)}s` : '';
                setBadge(droneId, (armed ? 'FLYING' : 'ONLINE') + rtt,
                         armed ? 'status-flying' : 'status-standby');
                if (droneId === currentDroneId) parseTelemetry(data.response);
            } else {
                missCount[droneId] = (missCount[droneId] || 0) + 1;
                if (missCount[droneId] >= OFFLINE_AFTER_MISSES) {
                    setBadge(droneId, 'OFFLINE', 'status-offline');
                }
            }
        } catch (error) {
            setBadge(droneId, 'OFFLINE', 'status-offline');
        }

        isPolling = false;
        setTimeout(pingSelectedDrone, PING_INTERVAL_MS);
    }

    // Button Bindings
    document.querySelectorAll('.cmd-btn[data-cmd]').forEach(btn => {
        btn.addEventListener('click', () => {
            sendUserCommand(btn.getAttribute('data-cmd'));
        });
    });

    // Emergency broadcast: every drone on the channel lands (no replies awaited).
    document.getElementById('btn-land-all')?.addEventListener('click', async () => {
        log('TX // all-land (BROADCAST)', 'tx');
        try {
            const response = await fetch('/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: 'all-land' })
            });
            const data = await response.json();
            log(response.ok ? `RX // ${data.response}` : `ERR // ${data.error || 'BROADCAST FAILED'}`,
                response.ok ? 'rx' : 'error');
        } catch (error) {
            log('ERR // OFFLINE - FAILED TO CONNECT TO RELAY', 'error');
        }
    });

    document.getElementById('btn-takeoff')?.addEventListener('click', () => {
        // Max 65 m: matches the drone's alt_safe_max (fence is 70 m).
        const MAX_TAKEOFF_M = 65;
        const alt = Math.min(Number(document.getElementById('takeoff-alt').value) || 5, MAX_TAKEOFF_M);
        sendUserCommand(`t ${alt}`);
    });

    document.getElementById('btn-down')?.addEventListener('click', () => {
        const m = document.getElementById('down-meters').value || "1";
        sendUserCommand(`d ${m}`);
    });

    document.getElementById('btn-area')?.addEventListener('click', () => {
        const px = document.getElementById('area-px').value || "3000";
        sendUserCommand(`area ${px}`);
    });

    document.getElementById('btn-set-param')?.addEventListener('click', () => {
        const param = document.getElementById('tune-param').value.trim();
        const val = document.getElementById('tune-val').value.trim();
        if (param && val !== "") {
            sendUserCommand(`set ${param} ${val}`);
        } else {
            log('ERR: LIVE TUNE REQ PARAM AND VALUE', 'error');
        }
    });

    // Init
    selectAsset("1", "C-1");
    // Start the endless background loop for the main drone
    setTimeout(pingSelectedDrone, 1000);
});
