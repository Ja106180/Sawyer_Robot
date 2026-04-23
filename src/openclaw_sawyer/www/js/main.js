window.onload = function () {
    const hostname = window.location.hostname;
    const ros = new ROSLIB.Ros({ url: `ws://${hostname}:9090` });

    const statusBadge = document.getElementById('connection-status');
    ros.on('connection', () => {
        statusBadge.innerText = 'ROS 在线';
        statusBadge.className = 'status-badge online';
    });
    ros.on('error', () => {
        statusBadge.innerText = '连接错误';
        statusBadge.className = 'status-badge';
    });
    ros.on('close', () => {
        statusBadge.innerText = 'ROS 离线';
        statusBadge.className = 'status-badge';
    });

    // 1. Video Feeds
    // Using web_video_server (default port 8080)
    document.getElementById('video-head').src = `http://${hostname}:8080/stream?topic=/io/internal_camera/head_camera/image_rect_color&type=mjpeg&quality=30`;
    document.getElementById('video-hand').src = `http://${hostname}:8080/stream?topic=/io/internal_camera/right_hand_camera/image_rect_color&type=mjpeg&quality=30`;

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

    // Try to load URDF if available
    new ROS3D.UrdfClient({
        ros: ros,
        tfClient: tfClient,
        path: `http://${hostname}:8000/`, // URDF files served by our web_host
        rootObject: viewer.scene,
        loader: ROS3D.COLLADA_LOADER
    });

    // 3. Mode Toggles (Grasp / Follow)
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
        graspSrv.callService(new ROSLIB.ServiceRequest({ data: graspActive }), (result) => {
            if (result.success) {
                btn.innerText = graspActive ? '关闭 视觉抓取' : '开启 视觉抓取';
                btn.classList.toggle('active', graspActive);
            } else {
                graspActive = !graspActive; // revert
                alert(result.message);
            }
        });
    };

    let followActive = false;
    document.getElementById('btn-follow').onclick = function() {
        followActive = !followActive;
        const btn = this;
        followSrv.callService(new ROSLIB.ServiceRequest({ data: followActive }), (result) => {
            if (result.success) {
                btn.innerText = followActive ? '关闭 手臂跟随' : '开启 手臂跟随';
                btn.classList.toggle('active', followActive);
            } else {
                followActive = !followActive; // revert
                alert(result.message);
            }
        });
    };

    // 4. Joint Sliders (Manual Control)
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
        jointTopic.publish(msg);
    }

    // Connect sliders to joint values
    jointNames.forEach((name, index) => {
        const sliderId = `slip-j${index}`;
        const valId = `val-j${index}`;
        const slider = document.getElementById(sliderId);
        const valLabel = document.getElementById(valId);

        slider.oninput = function() {
            const val = parseFloat(this.value);
            valLabel.innerText = val.toFixed(2);
            jointValues[index] = val;
            throttleUpdateJoints();
        };
    });

    // Throttling for performance
    let lastUpdate = 0;
    function throttleUpdateJoints() {
        const now = Date.now();
        if (now - lastUpdate > 50) { // Limit to 20Hz
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

    const headSlider = document.getElementById('slip-head');
    headSlider.oninput = function() {
        const val = parseFloat(this.value);
        document.getElementById('val-head').innerText = val.toFixed(2);
        const msg = new ROSLIB.Message({
            name: ['head_pan'],
            position: [val]
        });
        headTopic.publish(msg);
    };

    // 6. Gripper Control
    const gripperTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sawyer_skill_server/cmd_gripper',
        messageType: 'std_msgs/Bool'
    });

    let gripperOpen = true;
    const gripperBtn = document.getElementById('btn-gripper');
    gripperBtn.onclick = function() {
        gripperOpen = !gripperOpen;
        gripperBtn.innerText = gripperOpen ? '关闭 夹爪' : '打开 夹爪';
        gripperTopic.publish(new ROSLIB.Message({ data: gripperOpen }));
    };

    // Handle window resize
    window.addEventListener('resize', () => {
        const viz = document.getElementById('viz-container');
        viewer.resize(viz.offsetWidth, viz.offsetHeight);
    });
};
