/**
 * Sawyer Control Cockpit - main.js
 * High-performance robot control with Virtual Joysticks & Direct Sync
 */

window.onload = function () {
    console.log("Sawyer Cockpit initializing...");

    if (typeof ROSLIB === 'undefined') {
        alert("错误: 无法加载 ROS 核心库。请检查网络连接。");
        return;
    }

    // UI Helper
    function safeUI(id, action) {
        var el = document.getElementById(id);
        if (el) action(el);
        else console.warn("Missing UI element: " + id);
    }

    function updateLog(msg) {
        safeUI('debug-log', function(el) {
            el.innerHTML = `[${new Date().toLocaleTimeString()}] ${msg}<br>${el.innerHTML}`;
        });
    }

    // 1. ROS Setup
    var hostname = window.location.hostname;
    updateLog("Attempting connection to: " + hostname);
    
    var ros = new ROSLIB.Ros({ url: 'ws://' + hostname + ':9090' });

    var connectionTimeout = setTimeout(function() {
        if (!ros.isConnected) {
            updateLog("Connection Timeout. Check if rosbridge is running.");
            alert("连接超时。请检查机器人端 rosbridge 是否启动。");
        }
    }, 10000);

    ros.on('connection', function() {
        clearTimeout(connectionTimeout);
        updateLog("Connected to ROS Master.");
        safeUI('connection-status', function(el) { 
            el.innerText = '系统在线'; 
            el.className = 'status-badge online'; 
        });
        // Always set video URLs so feeds show when cameras are active
        refreshVideos();
    });

    ros.on('error', function(err) {
        updateLog("ROS Error: Check 9090 port.");
        safeUI('connection-status', function(el) { el.innerText = '连接异常'; el.className = 'status-badge'; });
    });

    // 2. Video Streams
    function refreshVideos() {
        updateLog("Loading video streams...");
        safeUI('video-head', function(el) { el.src = 'http://' + hostname + ':8080/stream?topic=/io/internal_camera/head_camera/image_rect_color&quality=50'; });
        safeUI('video-hand', function(el) { el.src = 'http://' + hostname + ':8080/stream?topic=/io/internal_camera/right_hand_camera/image_raw&quality=50'; });
    }

    // 3. Topics & Services
    var jointTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_joints', messageType: 'sensor_msgs/JointState' });
    var stateSub = new ROSLIB.Topic({ ros: ros, name: '/sawyer_state', messageType: 'sensor_msgs/JointState' });
    var headTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_head', messageType: 'sensor_msgs/JointState' });
    var gripperTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_gripper', messageType: 'std_msgs/Bool' });
    var speedTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_speed', messageType: 'std_msgs/Float32' });
    var targetNumTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_target_number', messageType: 'std_msgs/Int32' });

    var graspSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_grasp', serviceType: 'std_srvs/SetBool' });
    var followSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_follow', serviceType: 'std_srvs/SetBool' });
    var pointSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_point', serviceType: 'std_srvs/SetBool' });
    var numGraspSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_number_grasp', serviceType: 'std_srvs/SetBool' });
    var stopSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_stop', serviceType: 'std_srvs/Trigger' });
    var headCamSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_head_cam', serviceType: 'std_srvs/SetBool' });
    var handCamSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_hand_cam', serviceType: 'std_srvs/SetBool' });
    var resetSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_reset', serviceType: 'std_srvs/Trigger' });

    // 3.1 Emergency Stop
    safeUI('btn-estop', function(btn) {
        btn.onclick = function() {
            stopSrv.callService(new ROSLIB.ServiceRequest({}), function(res) {
                updateLog("!!! EMERGENCY STOP EXECUTED !!!");
                alert("紧急停止已触发！所有进程已关闭，机械臂已锁定。");
            });
        };
    });

    // 3.2 One-click Reset
    safeUI('btn-reset', function(btn) {
        btn.onclick = function() {
            updateLog("Sending reset command to Sawyer...");
            resetSrv.callService(new ROSLIB.ServiceRequest({}), function(res) {
                if (res.success) {
                    updateLog("Sawyer reset sequence complete.");
                    alert("复位完成：头部已显示表情，关节已平滑慢速复位！");
                } else {
                    updateLog("Sawyer reset failed: " + res.message);
                    alert("复位失败：" + res.message);
                }
            });
        };
    });

    // 4. Joint Mapping & State
    var jointNames = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
    var currentAngles = [0, -0.5, 0, 1.5, 0, 1.5, 0]; // Default start
    var isDragging = [false, false, false, false, false, false, false];

    // Listen to real robot state
    stateSub.subscribe(function(msg) {
        msg.name.forEach(function(name, i) {
            var idx = jointNames.indexOf(name);
            if (idx !== -1) {
                currentAngles[idx] = msg.position[i];
                if (!isDragging[idx]) {
                    safeUI('slip-j' + idx, function(slider) { slider.value = currentAngles[idx]; });
                    safeUI('val-j' + idx, function(label) { label.innerText = currentAngles[idx].toFixed(2); });
                }
            }
        });
    });

    function sendSingleJoint(idx, value) {
        var msg = new ROSLIB.Message({
            name: [jointNames[idx]],
            position: [value]
        });
        jointTopic.publish(msg);
    }

    // Initialize Active Sliders (J4-J6)
    [4, 5, 6].forEach(function(index) {
        safeUI('slip-j' + index, function(slider) {
            slider.onmousedown = function() { isDragging[index] = true; };
            slider.ontouchstart = function() { isDragging[index] = true; };
            slider.oninput = function() {
                var v = parseFloat(this.value);
                safeUI('val-j' + index, function(label) { label.innerText = v.toFixed(2); });
                sendSingleJoint(index, v);
            };
            slider.onmouseup = function() { isDragging[index] = false; };
            slider.ontouchend = function() { isDragging[index] = false; };
        });
    });

    // Speed & Head
    safeUI('slip-speed', function(slider) {
        slider.oninput = function() {
            var v = parseFloat(this.value);
            safeUI('val-speed', function(label) { label.innerText = v.toFixed(2); });
            speedTopic.publish(new ROSLIB.Message({ data: v }));
        };
    });

    safeUI('slip-head', function(slider) {
        slider.oninput = function() {
            var v = parseFloat(this.value);
            safeUI('val-head', function(label) { label.innerText = v.toFixed(2); });
            headTopic.publish(new ROSLIB.Message({ name: ['head_pan'], position: [v] }));
        };
    });

    // Gripper
    var gripperOpen = true;
    safeUI('btn-gripper', function(btn) {
        btn.onclick = function() {
            gripperOpen = !gripperOpen;
            btn.innerHTML = gripperOpen ? '<span>🗜️ 打开夹爪</span>' : '<span>🗜️ 关闭夹爪</span>';
            gripperTopic.publish(new ROSLIB.Message({ data: gripperOpen }));
        };
    });

    // 5. Virtual Joystick Implementation
    class VirtualJoystick {
        constructor(containerId, onMove) {
            this.container = document.getElementById(containerId);
            if (!this.container) return;

            // Create DOM
            this.container.innerHTML = `
                <div class="joystick-base"></div>
                <div class="joystick-handle"></div>
            `;
            this.handle = this.container.querySelector('.joystick-handle');
            
            this.active = false;
            this.centerX = 0;
            this.centerY = 0;
            this.maxDist = 70; // Half of zone approx

            // Events
            this.container.addEventListener('mousedown', this.start.bind(this));
            this.container.addEventListener('touchstart', this.start.bind(this), { passive: false });
            window.addEventListener('mousemove', this.move.bind(this));
            window.addEventListener('touchmove', this.move.bind(this), { passive: false });
            window.addEventListener('mouseup', this.end.bind(this));
            window.addEventListener('touchend', this.end.bind(this));

            this.onMove = onMove;
            this.timer = null;
            this.currentX = 0;
            this.currentY = 0;
        }

        start(e) {
            this.active = true;
            let rect = this.container.getBoundingClientRect();
            this.centerX = rect.left + rect.width / 2;
            this.centerY = rect.top + rect.height / 2;
            this.move(e);
            
            // Start publishing loop (smooth movement)
            if (this.timer) clearInterval(this.timer);
            this.timer = setInterval(() => {
                if (this.active) this.onMove(this.currentX, this.currentY);
            }, 50); // 20Hz
        }

        move(e) {
            if (!this.active) return;
            if (e.preventDefault) e.preventDefault();

            let clientX = e.touches ? e.touches[0].clientX : e.clientX;
            let clientY = e.touches ? e.touches[0].clientY : e.clientY;

            let dx = clientX - this.centerX;
            let dy = clientY - this.centerY;
            let dist = Math.sqrt(dx*dx + dy*dy);

            if (dist > this.maxDist) {
                dx = (dx / dist) * this.maxDist;
                dy = (dy / dist) * this.maxDist;
            }

            this.currentX = dx / this.maxDist; // -1 to 1
            this.currentY = -dy / this.maxDist; // -1 to 1 (inverted Y)

            this.handle.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
        }

        end() {
            if (!this.active) return;
            this.active = false;
            this.handle.style.transform = `translate(-50%, -50%)`;
            if (this.timer) clearInterval(this.timer);
            this.currentX = 0;
            this.currentY = 0;
        }
    }

    // Initialize Joysticks
    // Joystick 1: J0 (Base) & J1 (Shoulder)
    new VirtualJoystick('joystick-base', (x, y) => {
        if (Math.abs(x) > 0.1) {
            currentAngles[0] -= x * 0.05; // Base rotation step
            sendSingleJoint(0, currentAngles[0]);
        }
        if (Math.abs(y) > 0.1) {
            currentAngles[1] -= y * 0.05; // Shoulder pitch step
            sendSingleJoint(1, currentAngles[1]);
        }
    });

    // Joystick 2: J2 (Elbow Rotation) & J3 (Elbow Bend)
    new VirtualJoystick('joystick-elbow', (x, y) => {
        if (Math.abs(x) > 0.1) {
            currentAngles[2] += x * 0.05;
            sendSingleJoint(2, currentAngles[2]);
        }
        if (Math.abs(y) > 0.1) {
            currentAngles[3] -= y * 0.05;
            sendSingleJoint(3, currentAngles[3]);
        }
    });

    // 6. Mode Toggles
    function bindMode(id, srv, text) {
        safeUI(id, function(btn) {
            var active = false;
            btn.onclick = function() {
                active = !active;
                srv.callService(new ROSLIB.ServiceRequest({ data: active }), function(res) {
                    if(res.success) {
                        btn.classList.toggle('active', active);
                        updateLog(`${text} ${active ? 'Enabled' : 'Disabled'}`);
                    }
                });
            };
        });
    }

    bindMode('btn-grasp', graspSrv, "Visual Grasping");
    bindMode('btn-follow', followSrv, "Arm Following");
    bindMode('btn-point', pointSrv, "Point to Grasp");

    // Number Grasping Logic
    safeUI('btn-num-grasp', function(btn) {
        var active = false;
        btn.onclick = function() {
            active = !active;
            
            if (active) {
                // Publish target number first
                var selectEl = document.getElementById('select-number');
                var num = parseInt(selectEl.value);
                targetNumTopic.publish(new ROSLIB.Message({ data: num }));
                updateLog("Target number " + num + " sent.");
            }

            // Then start/stop the service
            numGraspSrv.callService(new ROSLIB.ServiceRequest({ data: active }), function(res) {
                if(res.success) {
                    btn.classList.toggle('active', active);
                    if (active) {
                        btn.innerHTML = '<span>⏹ 停止抓取</span>';
                    } else {
                        btn.innerHTML = '<span>🔢 启动数字抓取</span>';
                    }
                    updateLog(`Number Grasping ${active ? 'Started' : 'Stopped'}`);
                }
            });
        };
    });

    // 7. Camera Toggle Buttons
    function bindCam(btnId, srv, label) {
        safeUI(btnId, function(btn) {
            var active = false;
            btn.onclick = function() {
                active = !active;
                srv.callService(new ROSLIB.ServiceRequest({ data: active }), function(res) {
                    if (res.success) {
                        btn.classList.toggle('active', active);
                        updateLog(label + (active ? ' ON' : ' OFF'));
                    } else {
                        active = !active;
                        updateLog(label + ' failed: ' + res.message);
                    }
                });
            };
        });
    }

    bindCam('btn-head-cam', headCamSrv, 'Head Camera');
    bindCam('btn-hand-cam', handCamSrv, 'Hand Camera');

    // 8. Teach & Replay Logic
    var teachCmdTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_teach_cmd', messageType: 'std_msgs/String' });

    function sendTeachCmd(cmdObj) {
        var msg = new ROSLIB.Message({ data: JSON.stringify(cmdObj) });
        teachCmdTopic.publish(msg);
    }

    function fetchActionsList() {
        fetch('/api/actions')
            .then(response => response.json())
            .then(data => {
                var listContainer = document.getElementById('teach-actions-list');
                if (data.length === 0) {
                    listContainer.innerHTML = '<div style="padding: 10px; color: var(--text-secondary); text-align: center;">暂无保存的动作</div>';
                    return;
                }
                var html = '';
                data.forEach(action => {
                    html += `
                        <div class="teach-item" style="display: flex; justify-content: space-between; align-items: center;">
                            <div>
                                <input type="checkbox" value="${action}" class="action-checkbox">
                                <span>${action}</span>
                            </div>
                            <span class="delete-action-btn" data-name="${action}" style="cursor: pointer; color: var(--danger); font-size: 14px; padding: 0 5px;" title="删除动作">🗑️</span>
                        </div>
                    `;
                });
                listContainer.innerHTML = html;

                // Bind delete buttons
                var delBtns = listContainer.querySelectorAll('.delete-action-btn');
                delBtns.forEach(btn => {
                    btn.onclick = function(e) {
                        e.stopPropagation();
                        var name = this.getAttribute('data-name');
                        if (confirm('确定要删除动作 "' + name + '" 吗？此操作无法恢复。')) {
                            sendTeachCmd({ cmd: 'action_delete', name: name });
                            updateLog('Deleted action: ' + name);
                            setTimeout(fetchActionsList, 500);
                        }
                    };
                });
            })
            .catch(err => {
                console.error("Failed to fetch actions list:", err);
                document.getElementById('teach-actions-list').innerHTML = '<div style="padding: 10px; color: var(--danger); text-align: center;">加载失败</div>';
            });
    }

    // Initial fetch
    fetchActionsList();

    // Record Controls
    safeUI('btn-record-start', function(btn) {
        btn.onclick = function() {
            sendTeachCmd({ cmd: 'record_start' });
            document.getElementById('record-controls-idle').style.display = 'none';
            document.getElementById('record-controls-active').style.display = 'grid';
            btn.classList.add('recording-active');
            updateLog("Started recording trajectory. Drag the arm manually.");
        };
    });

    safeUI('btn-record-save', function(btn) {
        btn.onclick = function() {
            var name = prompt("请输入要保存的动作名称 (仅限英文/数字/下划线):", "my_action_" + Math.floor(Math.random()*1000));
            if (name) {
                // simple sanitize
                name = name.replace(/[^a-zA-Z0-9_]/g, '_');
                sendTeachCmd({ cmd: 'record_save', name: name });
                updateLog("Saving trajectory as: " + name);
                setTimeout(fetchActionsList, 1500); // refresh list after a short delay
            } else {
                sendTeachCmd({ cmd: 'record_cancel' });
                updateLog("Recording cancelled (no name provided).");
            }
            document.getElementById('record-controls-active').style.display = 'none';
            document.getElementById('record-controls-idle').style.display = 'grid';
            document.getElementById('btn-record-start').classList.remove('recording-active');
        };
    });

    safeUI('btn-record-cancel', function(btn) {
        btn.onclick = function() {
            sendTeachCmd({ cmd: 'record_cancel' });
            updateLog("Recording cancelled.");
            document.getElementById('record-controls-active').style.display = 'none';
            document.getElementById('record-controls-idle').style.display = 'grid';
            document.getElementById('btn-record-start').classList.remove('recording-active');
        };
    });

    // Playback Controls
    safeUI('btn-play-start', function(btn) {
        btn.onclick = function() {
            var checkboxes = document.querySelectorAll('.action-checkbox:checked');
            var selectedActions = Array.from(checkboxes).map(cb => cb.value);
            
            if (selectedActions.length === 0) {
                alert("请至少选择一个保存的动作。");
                return;
            }

            var loop = document.getElementById('chk-loop-action').checked;
            var seq = document.getElementById('chk-seq-action').checked;

            // If sequence is not checked but multiple selected, we still send them all but they play one by one?
            // The backend iterates over the array. If they only wanted one, they should check one.
            if (!seq && selectedActions.length > 1) {
                selectedActions = [selectedActions[0]]; // Only play the first one if seq is off
                updateLog("Sequence mode off. Only playing the first selected action.");
            }

            sendTeachCmd({ cmd: 'play_start', actions: selectedActions, loop: loop });
            updateLog("Started playback: " + selectedActions.join(", ") + " (Loop: " + loop + ")");
        };
    });

    safeUI('btn-play-stop', function(btn) {
        btn.onclick = function() {
            sendTeachCmd({ cmd: 'play_stop' });
            updateLog("Playback stopped.");
        };
    });

};
