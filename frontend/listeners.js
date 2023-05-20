var joint_positions;
var joint_velocities;
var joint_efforts;

var joint_state_listener = new ROSLIB.Topic({
  ros : ros,
  name : '/joint_states',
  messageType : 'sensor_msgs/JointState'
});

joint_state_listener.subscribe(function(msg) {
  //  0  - joint_gripper_finger_left 
  //  1  - joint_gripper_finger_right
  //  2  - joint_right_wheel
  //  3  - joint_left_wheel
  //  4  - joint_lift
  //  5  - joint_arm_l3
  //  6  - joint_arm_l2
  //  7  - joint_arm_l1
  //  8  - joint_arm_l0
  //  9  - joint_wrist_yaw
  // 10  - joint_head_pan
  // 11  - joint_head_tilt

  joint_positions = msg.position;
  joint_velocities = msg.velocity;
  joint_efforts = msg.effort;

  let arm_total = msg.position[8] + msg.position[7] + msg.position[6] + msg.position[5];
  let grip_total = msg.position[0];

  document.getElementById("head_pan_msg").innerHTML = msg.position[10].toFixed(4);
  document.getElementById("head_tilt_msg").innerHTML = msg.position[11].toFixed(4);
  document.getElementById("lift_msg").innerHTML = msg.position[4].toFixed(4);
  document.getElementById("arm_msg").innerHTML = arm_total.toFixed(4);
  document.getElementById("wrist_msg").innerHTML = msg.position[9].toFixed(4);
  document.getElementById("grip_msg").innerHTML = grip_total.toFixed(4);
});
  
var pose_listener = new ROSLIB.Topic({
  ros : ros,
  name : '/saved_pos_list',
  messageType : 'std_msgs/String'
});

pose_listener.subscribe(function(m) {
  if (m.data === "") {
    console.log("list from backend was empty");
    return;
  }
  saved_pose = m.data.split(',');
  const anchor = document.getElementById("saved_poses");
  while (anchor.firstChild) {
    anchor.removeChild(anchor.firstChild);
  }
  saved_pose.forEach(pose_name => {
    const button = document.createElement('button');
    button.setAttribute('name', pose_name);
    button.setAttribute('text', pose_name);
    button.setAttribute('class', 'pose_button');
    button.textContent = pose_name;
    button.onclick = function () {
      move_to_cf_pose(pose_name);
    }
    anchor.appendChild(button);
  });
});

