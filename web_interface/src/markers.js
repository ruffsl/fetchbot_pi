function MarkerViz(ros, topic, markerID) {

  let imuMarkerViewer = new ROS3D.Viewer({
    divID : markerID,
    width : 400,
    height : 300,
    antialias : true
  });

  let tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/imu_link'
      });
      // Setup the marker client.
  let markerClient = new ROS3D.MarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : topic,
    rootObject : imuMarkerViewer.scene
  });

}
