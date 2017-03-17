function ImuSubscriber(ros, name, callback) {
  let listener = new ROSLIB.Topic({
    ros : ros,
    name : name,
    messageType : 'sensor_msgs/Imu'
  });

  listener.subscribe(function(msg){return callback(msg);})
}

function ImuGrapher(imuID, ros, subTopic) {
  let normPoints = [];
  let imuX = [];
  let imuY = [];
  let imuZ = [];

  let normChart = new CanvasJS.Chart(imuID+"_norm",{
    title :{
      text: "IMU Norm"
    },
    data: [{
      type: "line",
      dataPoints: normPoints
    }
  ]
  });

  let xyzChart = new CanvasJS.Chart(imuID+"_xyz", {

    title: { text:"IMU XYZ" },
    data: [
      {
        type: "line",
        dataPoints: imuX
      }, {
        type: "line",
        dataPoints: imuY
      },{
        type: "line",
        dataPoints: imuZ
      }

    ]
  });


  let t = 0;

  let dataLength = 100;

  let updateChart = function (imuDater) {
    // count = count || 1;
    // count is number of times loop runs to generate random dataPoints.
    let {norm, x, y, z} = imuDater;

    normPoints.push({ x: t, y: norm });
    imuX.push({ x: t, y: x });
    imuY.push({ x: t, y: y });
    imuZ.push({ x: t, y: z });
    t++;

    // assumes all arrays are same length
    if (normPoints.length > dataLength) {
      normPoints.shift();
      imuX.shift();
      imuY.shift();
      imuZ.shift();
    }

    normChart.render();
    xyzChart.render();

  };

  // generates first set of dataPoints
  //updateChart(dataLength);

  let subscriber = new ImuSubscriber(ros, subTopic, function(imuMsg){
    // console.log('got the imu msg:');
    // console.log(imuMsg);
    let sqr = (x) => x * x;
    let orient = imuMsg.orientation;

    let norm = Math.sqrt(sqr(orient.x) + sqr(orient.y) + sqr(orient.z));
    // console.log(norm);
    let chartData = {norm: norm, x: orient.x, y: orient.y, z: orient.z};
    updateChart(chartData);
  });

  // update chart after specified time.
  //setInterval(function(){updateChart()}, updateInterval);
}
