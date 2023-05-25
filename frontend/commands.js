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

move_all = function () {
  const all_joints = {
    pose_type : document.getElementById("frame_select").value,
    head_pan : Number(document.getElementById("pan_input").value),
    head_tilt : Number(document.getElementById("tilt_input").value),
    lift : Number(document.getElementById("lift_input").value),
    arm : Number(document.getElementById("arm_input").value),
    grip : Number(document.getElementById("grip_input").value),
    wrist : Number(document.getElementById("wrist_input").value)
  }
  move_to_pose(all_joints);
}

pos_cmd = new ROSLIB.Topic({
  ros: ros,
  name : "/pose_cmd",
  messageType: 'std_msgs/String'
});

move_to_pose = function (name) {
  let msg = new ROSLIB.Message({data: name});
  pos_cmd.publish(msg);
}

translate_base_cmd = new ROSLIB.Topic({
  ros : ros,
  name : "/translate_base_cmd",
  messageType : 'std_msgs/Float64'
});

rotate_base_cmd = new ROSLIB.Topic({
  ros : ros,
  name : "/rotate_base_cmd",
  messageType : 'std_msgs/Float64'
});

stop_base_cmd = new ROSLIB.Topic({
  ros : ros,
  name : "/stop_base_cmd",
  messageType : 'std_msgs/Float64'
});

var mousedownID = -1;  //Global ID of mouse down interval
var move_type = '';
var move_amount = 0.0;
function mousedown(type, data) {
  console.log("mouse down");
  move_type = type;
  move_amount = data * Number(document.getElementById("velocity").value);
  if(mousedownID==-1)  //Prevent multimple loops!
     mousedownID = setInterval(whilemousedown, 100 /*execute every 100ms*/);


}
function mouseup() {
  console.log("mouse up");
  if(mousedownID!=-1) {  //Only stop if exists
    clearInterval(mousedownID);
    let msg = new ROSLIB.Message({data : 0});
    stop_base_cmd.publish(msg);
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

save_pos_cmd = new ROSLIB.Topic({
  ros: ros,
  name : "/save_pose_cmd",
  messageType: 'team1_teleop/SavePose'
});

save_pose = function () {
  const frame = document.getElementById("frame_select").value;
  const name = document.getElementById("pose_name").value;
  if (name === '') {
    alert("please provide name");
    return;
  }
  document.getElementById("pose_name").value = "";
  const msg = new ROSLIB.Message(
    {
      pose_name : name,
      pose_frame_id: frame
    }
  );
  save_pos_cmd.publish(msg);
} 

pose_in_cf_cmd = new ROSLIB.Topic({
  ros: ros,
  name : "/pose_in_cf_cmd",
  messageType: 'std_msgs/String'
});

move_to_cf_pose = function (name) {
  console.log("move to pose in cf called");
  let msg = new ROSLIB.Message({data: name});
  pose_in_cf_cmd.publish(msg);
}

/////////////////////////// Person Detection //////////////////////////////////
return_walker_cmd = new ROSLIB.Topic({
  ros : ros,
  name : "/return_walker_cmd",
  messageType : 'std_msgs/Float64'
});

function return_walker() {
  let msg = new ROSLIB.Message(1);
  return_walker_cmd.publish(msg);
}

