window.onload = function () {
    const hostname = window.location.hostname;
    const ros = new ROSLIB.Ros({ url: `ws://${hostname}:9090` });

    const statusBadge = document.getElementById('connection-status');
    const debugLog = document.getElementById('debug-log');

    function log(msg) {
        const div = document.createElement('div');
        div.innerText = `[${new Date().toLocaleTimeString()}] ${msg}`;
        debugLog.appendChild(div);
        debugLog.scrollTop = debugLog.scrollHeight;
        console.log(msg);
    }

    ros.on('connection', () => {
        statusBadge.innerText = 'ROS 在线';
        statusBadge.className = 'status-badge online';
        log('成功连接到 ROSBridge');
    });

    ros.on('error', (error) => {
        statusBadge.innerText = '连接错误';
        statusBadge.className = 'status-badge';
        log('ROS 连接错误: ' + JSON.stringify(error));
    });

    ros.on('close', () => {
        statusBadge.innerText = 'ROS 离线';
        statusBadge.className = 'status-badge';
        log('ROS 连接已关闭');
    });

    // 1. Video Feeds
    log('正在请求视频流...');
    const headUrl = `http://${hostname}:8080/stream?topic=/io/internal_camera/head_camera/image_rect_color&type=mjpeg&quality=30`;
    const handUrl = `http://${hostname}:8080/stream?topic=/io/internal_camera/right_hand_camera/image_rect_color&type=mjpeg&quality=30`;
    
    document.getElementById('video-head').src = headUrl;
    document.getElementById('video-hand').src = handUrl;

    // 2. 3D Visualization
    const viewer = new ROS3D.Viewer({
        divID: 'viz-container',
        width: document.getElementById('viz-container').offsetWidth,
        height: document.getElementById('viz-container').offsetHeight,
        antialias: true,
        background: '#000000',
        cameraPose: { x: 3, y: 3, z: 3 }
    });

    const tfClient = new ROSLIB.TFClient({
        ros: ros,
        angularThres: 0.01,
        transThres: 0.01,
        rate: 10.0,
        fixedFrame: 'base'
    });

    try {
        new ROS3D.UrdfClient({
            ros: ros,
            tfClient: tfClient,
            path: `http://${hostname}:8000/`,
            rootObject: viewer.scene,
            loader: ROS3D.COLLADA_LOADER
        });
        log('3D 模型加载中...');
    } catch(e) {
        log('3D 模型加载失败');
    }

    // 3. Mode Toggles (GLOBAL NAMES)
    const graspSrv = new ROSLIB.Service({
        ros: ros,
        name: '/sawyer_skill_server/set_grasp_mode',
        serviceType: 'std_srvs/SetBool'
    });
    const followSrv = new ROSLIB.Service({
        ros: ros,
        name: '/sawyer_skill_server/set_follow_mode',
        serviceType: 'std_srvs/SetBool'
    });

    let graspActive = false;
    document.getElementById('btn-grasp').onclick = function() {
        graspActive = !graspActive;
        const btn = this;
        log(`发送抓取请求: ${graspActive}`);
        graspSrv.callService(new ROSLIB.ServiceRequest({ data: graspActive }), (result) => {
            log(`抓取响应: ${result.message}`);
            if (result.success) {
                btn.innerText = graspActive ? '关闭 视觉抓取' : '开启 视觉抓取';
                btn.classList.toggle('active', graspActive);
            } else {
                graspActive = !graspActive;
                alert(result.message);
            }
        });
    };

    let followActive = false;
    document.getElementById('btn-follow').onclick = function() {
        followActive = !followActive;
        const btn = this;
        log(`发送跟随请求: ${followActive}`);
        followSrv.callService(new ROSLIB.ServiceRequest({ data: followActive }), (result) => {
            log(`跟随响应: ${result.message}`);
            if (result.success) {
                btn.innerText = followActive ? '关闭 手臂跟随' : '开启 手臂跟随';
                btn.classList.toggle('active', followActive);
            } else {
                followActive = !followActive;
                alert(result.message);
            }
        });
    };

    // 4. Joint Sliders (GLOBAL NAMES)
    const jointTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sawyer_skill_server/cmd_joints',
        messageType: 'sensor_msgs/JointState'
    });

    const jointNames = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
    const jointValues = [0, -0.5, 0, 1.5, 0, 1.5, 0];
    
    function updateJoints() {
        const msg = new ROSLIB.Message({
            header: { stamp: { secs: 0, nsecs: 0 }, frame_id: '' },
            name: jointNames,
            position: jointValues
        });
        log('发布关节指令...');
        jointTopic.publish(msg);
    }

    jointNames.forEach((name, index) => {
        const slider = document.getElementById(`slip-j${index}`);
        const valLabel = document.getElementById(`val-j${index}`);

        slider.oninput = function() {
            const val = parseFloat(this.value);
            valLabel.innerText = val.toFixed(2);
            jointValues[index] = val;
            throttleUpdateJoints();
        };
    });

    let lastUpdate = 0;
    function throttleUpdateJoints() {
        const now = Date.now();
        if (now - lastUpdate > 100) { 
            updateJoints();
            lastUpdate = now;
        }
    }

    // 5. Head Control
    const headTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sawyer_skill_server/cmd_head',
        messageType: 'sensor_msgs/JointState'
    });

    document.getElementById('slip-head').oninput = function() {
        const val = parseFloat(this.value);
        document.getElementById('val-head').innerText = val.toFixed(2);
        headTopic.publish(new ROSLIB.Message({
            name: ['head_pan'],
            position: [val]
        }));
    };

    // 6. Gripper Control
    const gripperTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sawyer_skill_server/cmd_gripper',
        messageType: 'std_msgs/Bool'
    });

    let gripperOpen = true;
    document.getElementById('btn-gripper').onclick = function() {
        gripperOpen = !gripperOpen;
        log(`发送夹爪指令: ${gripperOpen ? '打开' : '关闭'}`);
        this.innerText = gripperOpen ? '关闭 夹爪' : '打开 夹爪';
        gripperTopic.publish(new ROSLIB.Message({ data: gripperOpen }));
    };

    window.addEventListener('resize', () => {
        const viz = document.getElementById('viz-container');
        viewer.resize(viz.offsetWidth, viz.offsetHeight);
    });
};
