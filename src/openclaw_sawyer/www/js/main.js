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

    var graspSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_grasp', serviceType: 'std_srvs/SetBool' });
    var followSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_follow', serviceType: 'std_srvs/SetBool' });
    var pointSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_point', serviceType: 'std_srvs/SetBool' });
    var stopSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_stop', serviceType: 'std_srvs/Trigger' });
    var headCamSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_head_cam', serviceType: 'std_srvs/SetBool' });
    var handCamSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_hand_cam', serviceType: 'std_srvs/SetBool' });

    // 3.1 Emergency Stop
    safeUI('btn-estop', function(btn) {
        btn.onclick = function() {
            stopSrv.callService(new ROSLIB.ServiceRequest({}), function(res) {
                updateLog("!!! EMERGENCY STOP EXECUTED !!!");
                alert("紧急停止已触发！所有进程已关闭，机械臂已锁定。");
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
            currentAngles[0] += x * 0.05; // Base rotation step
            sendSingleJoint(0, currentAngles[0]);
        }
        if (Math.abs(y) > 0.1) {
            currentAngles[1] += y * 0.05; // Shoulder pitch step
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
            currentAngles[3] += y * 0.05;
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
};
