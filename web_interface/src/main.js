globals = {mapID: 'map', imuID: 'imuChart', markerID: 'markers'};


function init() {
    // Connect to ROS.
    let port = '9090';
    let url = '192.168.43.29'; // set to IP of ROS server -- localhost if local.
    let me = this;
    var ros = new ROSLIB.Ros({
    	url : 'ws://'+url + ':'+port
    });

    // Create the main viewer.
    let viewer = new ROS2D.Viewer({
    	divID : globals.mapID,
    	width : 800,
    	height : 600
    });

    // Setup the map client.
    let gridClient = new ROS2D.OccupancyGridClient({
    	ros : ros,
    	rootObject : viewer.scene,
      continuous: true
    });
    // subscribe to the r
    // let gridClient = new ROSLIB.Topic({
    //   ros: ros,
    //   name: 'map',
    //   messageType: 'OccupancyGrid'
    // });

    // gridClient.subscribe(function(grid) {
    //
    // })
    //
    // this.path = path;
    // this.type = type;
    // this.ros = ros;
    // let grid = new ROS2D.OccupancyGrid({
    //
    // });
    // Scale the canvas to fit to the map
    let first = true;
    gridClient.on('change', function(){
      if (first) {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        // console.log(gridClient.currentGrid.pose.position);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        first = false;
      }
      // that.viewer.shift(client.currentImage.pose.position.x, client.currentImage.pose.position.y);
      // console.log('changed');
    });

    let gp = new GoalPublisher(ros, viewer);
    // let gp = new NAV2D.Navigator({ros: ros, rootObject: viewer.stage, withOrientation: true}); // well then
    //let logger = new LogSubscriber(ros, '/move_base_simple/goal', 'geometry_msgs/PoseStamped');
    //logger.on();

    let igrapher = new ImuGrapher(globals.imuID, ros, '/ball_imu');

    let markerGrapher = new MarkerViz(ros, '/imu/marker', globals.markerID+"_imu")
    let sslGrapher = new MarkerViz(ros, '/ssl/marker', globals.markerID+"_ssl")
}
