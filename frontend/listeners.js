    var head_pan_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/head_pan_pos',
    messageType : 'std_msgs/Float64'
  });

  head_pan_listener.subscribe(function(m) {
    document.getElementById("head_pan_msg").innerHTML = m.data.toFixed(4);
    head_pan_pos = m.data;
  });

  var head_tilt_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/head_tilt_pos',
    messageType : 'std_msgs/Float64'
  });

  head_tilt_listener.subscribe(function(m) {
    document.getElementById("head_tilt_msg").innerHTML = m.data.toFixed(4);
    head_tilt_pos = m.data;
  });


  var lift_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/lift_pos',
    messageType : 'std_msgs/Float64'
  });

  lift_listener.subscribe(function(m) {
    document.getElementById("lift_msg").innerHTML = m.data.toFixed(4);
    lift_pos = m.data;
  });

  var arm_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/arm_pos',
    messageType : 'std_msgs/Float64'
  });

  arm_listener.subscribe(function(m) {
    document.getElementById("arm_msg").innerHTML = m.data.toFixed(4);
    arm_pos = m.data;
  });

  var wrist_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/wrist_pos',
    messageType : 'std_msgs/Float64'
  });

  wrist_listener.subscribe(function(m) {
    document.getElementById("wrist_msg").innerHTML = m.data.toFixed(4);
    wrist_pos = m.data;
  });

  var grip_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/grip_pos',
    messageType : 'std_msgs/Float64'
  });

  grip_listener.subscribe(function(m) {
    document.getElementById("grip_msg").innerHTML = m.data.toFixed(4);
    gripper_pos = m.data;
  });

  var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/pose_',
    messageType : 'std_msgs/Float64'
  });

  grip_listener.subscribe(function(m) {
    document.getElementById("grip_msg").innerHTML = m.data.toFixed(4);
    gripper_pos = m.data;
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
      
      // document.getElementById("pose_name").value = "";
      button.setAttribute('name', pose_name);
      button.setAttribute('text', pose_name);
      button.setAttribute('class', 'pose_button');
      button.textContent = pose_name;
      button.onclick = function () {
        move_to_cf_pose(pose_name);
      }
      anchor.appendChild(button);
    });

    }
     
  );

