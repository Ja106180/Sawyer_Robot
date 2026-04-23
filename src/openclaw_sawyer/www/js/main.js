window.onload = function () {
    console.log("Sawyer Control System (Sync & Smooth Mode) initializing...");

    function safeUI(id, action) {
        var el = document.getElementById(id);
        if (el) action(el);
    }

    // 1. ROS Connection
    var hostname = window.location.hostname;
    var ros = new ROSLIB.Ros({ url: 'ws://' + hostname + ':9090' });
    
    ros.on('connection', function() {
        safeUI('connection-status', function(el) { el.innerText = '在线'; el.className = 'status-badge online'; });
    });
    ros.on('error', function() {
        safeUI('connection-status', function(el) { el.innerText = '连接错误'; el.className = 'status-badge'; });
    });
    ros.on('close', function() {
        safeUI('connection-status', function(el) { el.innerText = '离线'; el.className = 'status-badge'; });
    });

    // 2. Video Feeds (Fixed to always request)
    function refreshVideos() {
        safeUI('video-head', function(el) { el.src = 'http://' + hostname + ':8080/stream?topic=/io/internal_camera/head_camera/image_rect_color&quality=50'; });
        safeUI('video-hand', function(el) { el.src = 'http://' + hostname + ':8080/stream?topic=/io/internal_camera/right_hand_camera/image_rect_color&quality=50'; });
    }
    refreshVideos();

    // 3. Topics & Services
    var jointTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_joints', messageType: 'sensor_msgs/JointState' });
    var stateSub = new ROSLIB.Topic({ ros: ros, name: '/sawyer_state', messageType: 'sensor_msgs/JointState' });
    var headTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_head', messageType: 'sensor_msgs/JointState' });
    var gripperTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_gripper', messageType: 'std_msgs/Bool' });

    var graspSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_grasp', serviceType: 'std_srvs/SetBool' });
    var followSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_follow', serviceType: 'std_srvs/SetBool' });

    // 4. Joint Control Logic (Anti-Twitch & State Sync)
    var jointNames = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
    var isDragging = [false, false, false, false, false, false, false];

    // Listen to real robot state to sync sliders
    stateSub.subscribe(function(msg) {
        msg.name.forEach(function(name, i) {
            var idx = jointNames.indexOf(name);
            if (idx !== -1 && !isDragging[idx]) {
                var val = msg.position[i];
                safeUI('slip-j' + idx, function(slider) { slider.value = val; });
                safeUI('val-j' + idx, function(label) { label.innerText = val.toFixed(2); });
            }
        });
    });

    // Only send the CHANGED joint to prevent multiple joints twitching
    function sendSingleJoint(idx, value) {
        var msg = new ROSLIB.Message({
            name: [jointNames[idx]],
            position: [value]
        });
        jointTopic.publish(msg);
    }

    jointNames.forEach(function(name, index) {
        var sliderId = 'slip-j' + index;
        safeUI(sliderId, function(slider) {
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

    // 5. Head Control
    var isHeadDragging = false;
    safeUI('slip-head', function(slider) {
        slider.onmousedown = function() { isHeadDragging = true; };
        slider.oninput = function() {
            var v = parseFloat(this.value);
            safeUI('val-head', function(label) { label.innerText = v.toFixed(2); });
            headTopic.publish(new ROSLIB.Message({ name: ['head_pan'], position: [v] }));
        };
        slider.onmouseup = function() { isHeadDragging = false; };
    });

    // 6. Gripper
    var gripperOpen = true;
    safeUI('btn-gripper', function(btn) {
        btn.onclick = function() {
            gripperOpen = !gripperOpen;
            btn.innerText = gripperOpen ? '关闭 夹爪' : '打开 夹爪';
            gripperTopic.publish(new ROSLIB.Message({ data: gripperOpen }));
        };
    });

    // 7. Mode Toggles
    safeUI('btn-grasp', function(btn) {
        var active = false;
        btn.onclick = function() {
            active = !active;
            graspSrv.callService(new ROSLIB.ServiceRequest({ data: active }), function(res) {
                if(res.success) {
                    btn.innerText = active ? '关闭 视觉抓取' : '开启 视觉抓取';
                    btn.classList.toggle('active', active);
                    // Reset videos just in case
                    setTimeout(refreshVideos, 1000);
                }
            });
        };
    });

    safeUI('btn-follow', function(btn) {
        var active = false;
        btn.onclick = function() {
            active = !active;
            followSrv.callService(new ROSLIB.ServiceRequest({ data: active }), function(res) {
                if(res.success) {
                    btn.innerText = active ? '关闭 手臂跟随' : '开启 手臂跟随';
                    btn.classList.toggle('active', active);
                }
            });
        };
    });
};
