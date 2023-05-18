var scan_service = new ROSLIB.Service({
  ros : ros,
  name : '/funmap/trigger_local_localization',
  serviceType : 'std_srvs/Trigger'
  });

perf_scan = function() {
  let request = new ROSLIB.ServiceRequest();
  console.log("Called Scan func");
  scan_service.callService(request, function(result) {
    console.log("Service returned: " 
    + result.success + " and " 
    + result.message);
  });
} 