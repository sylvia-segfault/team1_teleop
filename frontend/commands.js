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

move_all = function () {
  let all_joints = [
    Number(lift_pos),
    Number(arm_pos),
    Number(gripper_pos),
    Number(wrist_pos),
    Number(head_pan_pos),
    Number(head_tilt_pos),
  ]
  move_to_pose(all_joints);
}

translate_base_cmd = new ROSLIB.Topic({
  ros : ros,
  name : "/translate_base_cmd",
  messageType : 'std_msgs/Float64'
});

rotate_base_cmd = new ROSLIB.Topic({
  ros : ros,
  name : "/rotate_left_cmd",
  messageType : 'std_msgs/Float64'
});

translate_forward = function () {
  let msg = new ROSLIB.Message({data : 0});
  console.log("translate forward");
  translate_forward_cmd.publish(msg);
}



rotate_left = function () {
  let msg = new ROSLIB.Message({data : 0});
  console.log("rotate left");
  rotate_left_cmd.publish(msg);
}

var mousedownID = -1;  //Global ID of mouse down interval
var move_type = '';
var move_amount = 0.0;
function mousedown(type, data) {
  console.log("mouse down");
  move_type = type;
  move_amount = data;
  if(mousedownID==-1)  //Prevent multimple loops!
     mousedownID = setInterval(whilemousedown, 100 /*execute every 100ms*/);


}
function mouseup() {
  console.log("mouse up");
  if(mousedownID!=-1) {  //Only stop if exists
    clearInterval(mousedownID);
    move_type = '';
    move_amount = 0.0;
    mousedownID=-1;
  }

}
function whilemousedown() {
  let msg = new ROSLIB.Message({data : move_amount});
  if (move_type === "translate") {
    console.log("translate base");
    translate_base_cmd.publish(msg);
  } else if (move_type === "rotate") {
    console.log("rotate base");
    rotate_base_cmd.publish(msg);
  }

}
//Also clear the interval when user leaves the window with mouse
//document.addEventListener("mouseout", mouseup);

  