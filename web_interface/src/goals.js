// class for publishing goals when a scene is clicked. the published goal
// is a PoseStamped for going to the position specified by the click.

function GoalPublisher (ros, viewer) {
  if (!viewer) {
    console.log('uninitialized viewer in goals');
    return;
  }
  // subscribe to the map to update the transform

  let listener = new ROSLIB.Topic({
    ros : ros,
    name : '/map_metadata',
    messageType : 'nav_msgs/MapMetaData'
  });

  let height = viewer.height;
  let width = viewer.width;
  listener.subscribe(function(mapInfo) {
    this.scaleX = mapInfo.width / width;
    this.scaleY = mapInfo.height / height;
  });

  let publisher = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base_simple/goal',
    messageType: 'geometry_msgs/PoseStamped'
  });

  // /robot_0/move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 0.25, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

  this.scaleX = 1.0; //viewer.width / 800.0; // guesses
  this.scaleY = 1.0; //viewer.height / 600.0;
  this.scene = viewer.scene;
  let scene = this.scene;
  let me = this;
  scene.addEventListener('stagemouseup', function(click) {
    if (scene.mouseInBounds) {
      let pos = scene.globalToRos(click.stageX, click.stageY);
      // let posVec3 = ROSLIB.Vector3(pos);
      // console.log('publi')
      publisher.publish(me.makeGoal(pos));

      // TODO: publish
      // let pose = this.fromXY()
    }
  });

}

GoalPublisher.prototype.makeGoal = function(pos) {
  // /robot_0/move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 0.25, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
  return {
    header: {
      frame_id: "map"
    },
    pose: {
      position: {
        x: pos.x,
        y: pos.y
      },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    }
  };
}

GoalPublisher.prototype.fromXY = function(x, y) {
  let alpha = 0.0; // TODO: rotations
  let theta = 0.0;
  // assumes map origin and click origin are the same
  let v1 = new ROSLIB.Vector3({
    x : x*this.scaleX,
    y : y*this.scaleY,
    z : 0.0
  });
  let q1 = new ROSLIB.Quaternion({
    x : 0.0,
    y : 0.0,
    z : alpha,
    w : theta
  });

  return new Pose(v1, q1);
}
