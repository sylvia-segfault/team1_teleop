pan_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/head_pan_cmd",
    messageType : 'std_msgs/Float64'
});

move_pan = function () {
    let data = Number(document.getElementById("pan_input").value);
    let msg = new ROSLIB.Message({data : data});
    console.log("In move: " + data);
    pan_cmd.publish(msg);
}

tilt_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/head_tilt_cmd",
    messageType : 'std_msgs/Float64'
});

move_tilt = function () {
    let data = Number(document.getElementById("tilt_input").value);
    let msg = new ROSLIB.Message({data : data});
    console.log("In move: " + data);
    tilt_cmd.publish(msg);
}

lift_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/lift_cmd",
    messageType : 'std_msgs/Float64'
});

move_lift = function () {
    let data = Number(document.getElementById("lift_input").value);
    let msg = new ROSLIB.Message({data : data});
    console.log("In move: " + data);
    lift_cmd.publish(msg);
}

arm_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/arm_cmd",
    messageType : 'std_msgs/Float64'
});

move_arm = function () {
    let data = Number(document.getElementById("arm_input").value);
    let msg = new ROSLIB.Message({data : data});
    console.log("In move: " + data);
    arm_cmd.publish(msg);
}

wrist_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/wrist_cmd",
    messageType : 'std_msgs/Float64'
});

move_wrist = function () {
    let data = Number(document.getElementById("wrist_input").value);
    let msg = new ROSLIB.Message({data : data});
    console.log("In move: " + data);
    wrist_cmd.publish(msg);
}

grip_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/grip_cmd",
    messageType : 'std_msgs/Float64'
});

move_grip = function () {
    let data = Number(document.getElementById("grip_input").value);
    let msg = new ROSLIB.Message({data : data});
    console.log("In move: " + data);
    grip_cmd.publish(msg);
}

pose_cmd = new ROSLIB.Topic({
    ros : ros,
    name : "/pose_cmd",
    messageType : 'std_msgs/Float64MultiArray'
});

move_to_pose = function (sing_pose) {
    let msg = new ROSLIB.Message({data : sing_pose});
    console.log("In move pose: " + sing_pose);
    pose_cmd.publish(msg);
}

  