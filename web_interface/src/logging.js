function LogSubscriber(ros, path, type){
  this.listener = new ROSLIB.Topic({
    ros: ros,
    name: path,
    messageType: type
  });

  this.path = path;
  this.type = type;
  this.ros = ros;

}

LogSubscriber.prototype.off = function() {
  this.listener.unsubscribe();
}

LogSubscriber.prototype.on = function() {
  let me = this;
  this.listener.subscribe(function(msg){
    console.log('received msg on: ' + me.path);
    console.log(msg);
  });
}
