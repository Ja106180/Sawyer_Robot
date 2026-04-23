window.onload = function () {
    console.log("Sawyer Control System (UCAR-Matched Style) initializing...");

    function safeUI(id, action) {
        var el = document.getElementById(id);
        if (el) action(el);
    }

    // 1. ROS Connection (UCAR Style)
    var hostname = window.location.hostname;
    var ros = new ROSLIB.Ros({ url: 'ws://' + hostname + ':9090' });
    
    ros.on('connection', function() {
        safeUI('connection-status', function(el) { el.innerText = '在线'; el.className = 'status-badge online'; });
        console.log("Connected to ROSBridge.");
    });
    ros.on('error', function() {
        safeUI('connection-status', function(el) { el.innerText = '连接错误'; el.className = 'status-badge'; });
    });
    ros.on('close', function() {
        safeUI('connection-status', function(el) { el.innerText = '离线'; el.className = 'status-badge'; });
    });

    // 2. Video Feeds (UCAR Style - Simplified)
    safeUI('video-head', function(el) { el.src = 'http://' + hostname + ':8080/stream?topic=/io/internal_camera/head_camera/image_rect_color'; });
    safeUI('video-hand', function(el) { el.src = 'http://' + hostname + ':8080/stream?topic=/io/internal_camera/right_hand_camera/image_rect_color'; });

    // 3. Topics & Services
    var jointTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_joints', messageType: 'sensor_msgs/JointState' });
    var headTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_head', messageType: 'sensor_msgs/JointState' });
    var gripperTopic = new ROSLIB.Topic({ ros: ros, name: '/sawyer_gripper', messageType: 'std_msgs/Bool' });

    var graspSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_grasp', serviceType: 'std_srvs/SetBool' });
    var followSrv = new ROSLIB.Service({ ros: ros, name: '/sawyer_follow', serviceType: 'std_srvs/SetBool' });

    // 4. Joint Control Logic
    var jointNames = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
    var jointValues = [0, -0.5, 0, 1.5, 0, 1.5, 0];

    function sendJoints() {
        var msg = new ROSLIB.Message({ name: jointNames, position: jointValues });
        jointTopic.publish(msg);
    }

    jointNames.forEach(function(name, index) {
        var sliderId = 'slip-j' + index;
        safeUI(sliderId, function(slider) {
            slider.oninput = function() {
                var v = parseFloat(this.value);
                safeUI('val-j' + index, function(label) { label.innerText = v.toFixed(2); });
                jointValues[index] = v;
                sendJoints();
            };
        });
    });

    // 5. Head Control Logic
    safeUI('slip-head', function(slider) {
        slider.oninput = function() {
            var v = parseFloat(this.value);
            safeUI('val-head', function(label) { label.innerText = v.toFixed(2); });
            headTopic.publish(new ROSLIB.Message({ name: ['head_pan'], position: [v] }));
        };
    });

    // 6. Gripper Control Logic
    var gripperOpen = true;
    safeUI('btn-gripper', function(btn) {
        btn.onclick = function() {
            gripperOpen = !gripperOpen;
            btn.innerText = gripperOpen ? '关闭 夹爪' : '打开 夹爪';
            gripperTopic.publish(new ROSLIB.Message({ data: gripperOpen }));
        };
    });

    // 7. Mode Toggles Logic
    var graspActive = false;
    safeUI('btn-grasp', function(btn) {
        btn.onclick = function() {
            graspActive = !graspActive;
            graspSrv.callService(new ROSLIB.ServiceRequest({ data: graspActive }), function(res) {
                if(res.success) {
                    btn.innerText = graspActive ? '关闭 视觉抓取' : '开启 视觉抓取';
                    btn.style.backgroundColor = graspActive ? '#22c55e' : '';
                }
            });
        };
    });

    var followActive = false;
    safeUI('btn-follow', function(btn) {
        btn.onclick = function() {
            followActive = !followActive;
            followSrv.callService(new ROSLIB.ServiceRequest({ data: followActive }), function(res) {
                if(res.success) {
                    btn.innerText = followActive ? '关闭 手臂跟随' : '开启 手臂跟随';
                    btn.style.backgroundColor = followActive ? '#22c55e' : '';
                }
            });
        };
    });
};
