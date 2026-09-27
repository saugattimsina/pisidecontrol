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
            
            // Wait a moment after manual command before resuming polls to let radio breathe
            setTimeout(() => { isTransmittingManual = false; }, 500); 
        }
    }

    // BACKGROUND AUTO-POLLING LOOP
    let isPolling = false;
    async function pollMainDrone() {
        if (isPolling) return;
        
        // If the user just clicked a manual command (like ARM), skip this polling cycle
        // so we don't jam the LoRa module with status requests!
        if (isTransmittingManual) {
            setTimeout(pollMainDrone, 1000);
            return;
        }
        
        isPolling = true;

        const droneId = currentDroneId; // Only poll the currently selected main drone
        
        if (!droneId) {
            isPolling = false;
            setTimeout(pollMainDrone, 1000);
            return;
        }

        try {
            // Send the status request quietly in the background
            const response = await fetch('/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd: `${droneId}-status` })
            });

            const data = await response.json();
            
            // Locate this drone in the left Asset Roster panel
            const assetItem = document.querySelector(`.asset-item[data-id="${droneId}"]`);
            const statusBadge = assetItem?.querySelector('.asset-status');

            if (response.ok && data.response) {
                const payloadStr = data.response.split('] ')[1];
                if (payloadStr) {
                    // Check armed state to decide if flying or standby
                    const isArmed = payloadStr.includes('A:Y');
                    if (statusBadge) {
                        statusBadge.textContent = isArmed ? 'FLYING' : 'STANDBY';
                        statusBadge.className = `asset-status ${isArmed ? 'status-flying' : 'status-standby'}`;
                    }
                    
                    // Because we only poll the current drone, we can safely update the dashboard
                    parseTelemetry(data.response);
                }
            } else {
                // Timeout or error: Mark offline
                if (statusBadge) {
                    statusBadge.textContent = 'OFFLINE';
                    statusBadge.className = 'asset-status status-offline';
                }
            }
        } catch (error) {
            // Network failure
            const assetItem = document.querySelector(`.asset-item[data-id="${droneId}"]`);
            if (assetItem) {
                const statusBadge = assetItem.querySelector('.asset-status');
                statusBadge.textContent = 'OFFLINE';
                statusBadge.className = 'asset-status status-offline';
            }
        }
        
        isPolling = false;
        // Wait before querying again. The drone radio is half-duplex and each poll
        // makes it transmit twice (ACK + telemetry), so polling too fast drops commands.
        setTimeout(pollMainDrone, 2500);
    }

    // Button Bindings
    document.querySelectorAll('.cmd-btn[data-cmd]').forEach(btn => {
        btn.addEventListener('click', () => {
            sendUserCommand(btn.getAttribute('data-cmd'));
        });
    });

    document.getElementById('btn-takeoff')?.addEventListener('click', () => {
        const alt = document.getElementById('takeoff-alt').value || "20";
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
    setTimeout(pollMainDrone, 1000);
});
