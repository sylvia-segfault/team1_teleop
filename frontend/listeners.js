var joint_positions;
var joint_velocities;
var joint_efforts;

var joint_state_listener = new ROSLIB.Topic({
  ros : ros,
  name : '/joint_states',
  messageType : 'sensor_msgs/JointState'
});

joint_state_listener.subscribe(function(msg) {
  joint_positions = msg.position;
  joint_velocities = msg.velocity;
  joint_efforts = msg.effort;
  document.getElementById("head_pan_msg").innerHTML = msg.position[0].toFixed(4);
  document.getElementById("head_tilt_msg").innerHTML = msg.position[1].toFixed(4);
  document.getElementById("lift_msg").innerHTML = msg.position[2].toFixed(4);
  document.getElementById("arm_msg").innerHTML = msg.position[3].toFixed(4);
  document.getElementById("wrist_msg").innerHTML = msg.position[4].toFixed(4);
  document.getElementById("grip_msg").innerHTML = msg.position[5].toFixed(4);
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

