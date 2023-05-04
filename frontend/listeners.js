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
    grip_pos = m.data;
  });

