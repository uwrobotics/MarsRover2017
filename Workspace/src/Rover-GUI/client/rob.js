import React,{Component} from 'react'
import ReactDOM,{ render} from 'react-dom'
//import App from '../components/App'
import Gps from '../components/Gps'
import Wheel from '../components/Wheel'
import Battery from '../components/battery'

const Line = require('rc-progress').Line;
const Circle = require('rc-progress').Circle;

const MAX_VOLTAGE = 12;
const MIN_VOLTAGE = 10;
const MAX_CURRENT = 100;
const MAX_TEMPERATURE = 50; //deg C

function MyButton(props) {
  return (
    <button className="square" onClick={() => props.onClick()}>
      {props.value}
    </button>
  );
}



class RobotPanel extends React.Component {
  changeBattery(percentageData, currentData, voltageData){
    this.setState(
          {
          battery: {
            percentage: +percentageData.toFixed(2),
            current: +currentData.toFixed(2),
            voltageRemain: +voltageData.toFixed(2),
            currentPercentage: currentData/MAX_CURRENT * 100,
            voltageRemainPercentage: (voltageData-MIN_VOLTAGE)/(MAX_VOLTAGE-MIN_VOLTAGE) * 100,
          },

        });
  }

  changeTemp(outsideTempData, pcTempData, roboteqTempData){
    this.setState(
          {
          temperature: {
              outsideTemp: +outsideTempData.toFixed(2),
              pcTemp: +pcTempData.toFixed(2),
              roboteqTemp: +roboteqTempData.toFixed(2),
          },

        });
  }

//  *********************** this is our button part *******************************
  // connectButtonClick(clas) {
  //   var listener = new ROSLIB.Topic({
  //     ros : this.state.ros,
  //     name : '/listener',
  //     messageType : 'std_msgs/String'
  //   });                               // 5 m      this is for what will happen, if we click those 9 squares
  //
  //
  //   printToScreen(function(data){
  //     console.log('string is '+  ': ' + data);
  //     clas.changeString(data);
  //   });
  //
  //   function printToScreen(callback){
  //     listener.subscribe(function(message) {
  //     //console.log('Received message on Int' + listener.name + ': ' + message.data);
  //     callback(message.data);
  //   });
  //   }
  // }

  // disconnectButtonClick() {                                    // 5 m      this is for what will happen, if we click those 9 squares
  //   //**** connect with roblisjs ************//
  //   var listener = new ROSLIB.Topic({
  //     ros : this.state.ros,
  //     name : '/myListener',
  //     messageType : 'geometry_msgs/Twist'
  //   });
  //
  //   // Then we add a callback to be called every time a message is published on this topic.
  //   listener.subscribe(function(message) {
  //     console.log('Linear' + message.data.linear + ': ' + message.data);
  //     string = message.data;
  //     //console.log(' recievedString is '+recievedString);
  //     // If desired, we can unsubscribe from the topic as well.
  //     //listener.unsubscribe();
  //   });
  // }
  //



  refreshBattery(thisClass){
    var listener = new ROSLIB.Topic({
      ros : this.state.ros,
      name : '/battery_data',
      messageType : 'std_msgs/Float32MultiArray'
    });
    var percentageData;
    var currentData;
    var voltageData;
    listener.subscribe(function(message) {
      console.log("got battery message");
      percentageData = message.data[0];
      currentData = message.data[1];
      voltageData = message.data[2];
      thisClass.changeBattery(percentageData, currentData, voltageData);
    });
  }

  refreshTemp(thisClass){
    var listener = new ROSLIB.Topic({
      ros : this.state.ros,
      name : '/temperature_data',
      messageType : 'std_msgs/Float32MultiArray'
    });
    listener.subscribe(function(message) {
      console.log("got temperature message");
      thisClass.changeTemp(message.data[0],  message.data[1], message.data[2]);
    });
  }


  renderConnectSquare() {
    return <MyButton value="connect" onClick={() => this.connectButtonClick(this)} />
  }

  renderUnConnectSquare() {
    return <MyButton value="disconnect" onClick={() => this.disconnectButtonClick()} />
  }

  renderRefreshSquare() {
    return <MyButton value="Recieveing Data" onClick={() => this.refreshBattery(this)} />
  }

//  **************************** above is button part ******************************


//  ******************** this part define the data of this page *******************


  constructor() {                       // game's constructor, set states
    super();
    this.state = {
      ros : new ROSLIB.Ros(),
      battery: {
        percentage: 100,
        current: 0,
        currentPercentage: 0,
        voltageRemain: 12,
        voltageRemainPercentage: 100,
      },
      temperature: {
        outsideTemp: 25,
        pcTemp: 30,
        roboteqTemp: 33,
      },
      gps : {
        latitude: 60,             // wei du
        longitude : 60
      },
      wheels:[{
        working: "working",
        name : '1st wheel',
        status :'doing good'
      },
      {
        working: "not working",
        name : '2ed wheel',
        status :'doing good'
      },
      {
        working: "working",
        name : '3rd wheel',
        status :'doing good'
      },
      {
        working: "working",
        name : '4st wheel',
        status :'doing good'
      }]
    };
    setUpRos(this.state.ros);
    publicTopisMessage(this.state.ros);
    this.refreshBattery(this);
    this.refreshTemp(this);
  }


// **************************** above is the data of the page **********************

//{this.renderConnectSquare()}
//{this.renderUnConnectSquare()}
//{this.renderRefreshSquare()}
  render() {
  const containerStyle = {
    width: '200px',
    padding:'0px',
    margin:'auto',
  };
const smallLine = {
  width: '90px',
  padding:'0px',
  margin:'auto',
}
    const circleContainerStyle = {
   width: '200px',
   height: '200px',
   padding: '0px',
   margin:'auto',

   };
  return(
    <div>
      <div className="video_stream_container">
        <div className="video_stream">
          <img src="http://localhost:8080/stream?topic=/usb_cam/image_raw" height="370"/>
        </div>
        <div className="video_stream">
          <img src="http://localhost:8080/stream?topic=/usb_cam2/image_raw" height="370"/>
        </div>
      </div>

      <div className="rosData">
        <div className="battery">
          <div className="battery_element">
          <p>Battery Percentage {this.state.battery.percentage}%</p>
            <div style={circleContainerStyle}>
              <Circle
                percent={this.state.battery.percentage}
                strokeWidth="6"
                strokeLinecap="square"
                strokeColor="#85D262"
              />
            </div>
          </div>


          <div className="battery_element">
          <p>Voltage Level {this.state.battery.voltageRemain} V</p>
            <div style={circleContainerStyle}>
              <Circle
                percent={this.state.battery.voltageRemainPercentage}
                strokeWidth="6"
                strokeLinecap="square"
                strokeColor="#85D262"
              />
            </div>
          </div>

          <div className="battery_element">
          <p>Current: {this.state.battery.current} A</p>
            <div style={circleContainerStyle}>
              <Circle
                percent={this.state.battery.currentPercentage}
                strokeWidth="6"
                strokeLinecap="square"
                strokeColor="#85D262"
              />
            </div>
          </div>


        </div>

      <div className="temperature">
        <div className="temperature_element">
          <p>Outside Temp {this.state.temperature.outsideTemp} &#8451;</p>
           <div style={containerStyle}>
             <Line percent={this.state.temperature.outsideTemp} strokeWidth="4" strokeColor="#bf2020" />
           </div>
        </div>
        <div className="temperature_element">
          <p>CPU Temp {this.state.temperature.pcTemp} &#8451;</p>
           <div style={containerStyle}>
             <Line percent={this.state.temperature.pcTemp} strokeWidth="4" strokeColor="#bf2020" />
           </div>
        </div>
        <div className="temperature_element">
          <p>Roboteq Temp {this.state.temperature.roboteqTemp} &#8451;</p>
           <div style={containerStyle}>
             <Line percent={this.state.temperature.roboteqTemp} strokeWidth="4" strokeColor="#bf2020" />
           </div>
        </div>
      </div>


            <div className="cell_temperature">
              <div className="small_temp">
                <p>Cell 1: {this.state.temperature.outsideTemp} &#8451;</p>
                 <div style={smallLine}>
                   <Line percent={this.state.temperature.outsideTemp} strokeWidth="4" strokeColor="#bf2020" />
                 </div>
              </div>
              <div className="small_temp">
                <p>Cell 2: {this.state.temperature.pcTemp} &#8451;</p>
                 <div style={smallLine}>
                   <Line percent={this.state.temperature.pcTemp} strokeWidth="4" strokeColor="#bf2020" />
                 </div>
              </div>
              <div className="small_temp">
                <p>Cell 3: {this.state.temperature.roboteqTemp} &#8451;</p>
                 <div style={smallLine}>
                   <Line percent={this.state.temperature.roboteqTemp} strokeWidth="4" strokeColor="#bf2020" />
                 </div>
              </div>
              <div className="small_temp">
                <p>Cell 4: {this.state.temperature.roboteqTemp} &#8451;</p>
                 <div style={smallLine}>
                   <Line percent={this.state.temperature.roboteqTemp} strokeWidth="4" strokeColor="#bf2020" />
                 </div>
              </div>
              <div className="small_temp">
                <p>Cell 5: {this.state.temperature.roboteqTemp} &#8451;</p>
                 <div style={smallLine}>
                   <Line percent={this.state.temperature.roboteqTemp} strokeWidth="4" strokeColor="#bf2020" />
                 </div>
              </div>
              <div className="small_temp">
                <p>Cell 6: {this.state.temperature.roboteqTemp} &#8451;</p>
                 <div style={smallLine}>
                   <Line percent={this.state.temperature.roboteqTemp} strokeWidth="4" strokeColor="#bf2020" />
                 </div>
              </div>
            </div>
          </div>

    </div>  );
  }
}




// ========================================

// correct
ReactDOM.render(
  <RobotPanel />,
  document.getElementById('container')
);




// Connecting to ROS
// -----------------

function setUpRos(ros){
  //var ros = new ROSLIB.Ros();
  //myRos = ros;

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
      document.getElementById('connecting').style.display = 'none';
      document.getElementById('connected').style.display = 'none';
      document.getElementById('closed').style.display = 'none';
      document.getElementById('error').style.display = 'inline';
      console.log(error);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
      console.log('Connection made!');
      document.getElementById('connecting').style.display = 'none';
      document.getElementById('error').style.display = 'none';
      document.getElementById('closed').style.display = 'none';
      document.getElementById('connected').style.display = 'inline';
    });

    ros.on('close', function() {
      console.log('Connection closed.');
      document.getElementById('connecting').style.display = 'none';
      document.getElementById('connected').style.display = 'none';
      document.getElementById('closed').style.display = 'inline';
    });

    // Create a connection to the rosbridge WebSocket server.
    ros.connect('ws://localhost:9090');
}

//*******************************ROS part *********************************************


//create a topic and send a message
//
function publicTopisMessage(ros){
  // Publishing a Topic
  // ------------------

  // First, we create a Topic object with details of the topic's name and message type.
  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  // Then we create the payload to be published. The object we pass in to ros.Message matches the
  // fields defined in the geometry_msgs/Twist.msg definition.
  var twist = new ROSLIB.Message({
    linear : {
      x : 0.1,
      y : 0.2,
      z : 0.3
    },
    angular : {
      x : -0.1,
      y : -0.2,
      z : -0.3
    }
  });

  // And finally, publish.
  cmdVel.publish(twist);
}



  //Subscribing to a Topic
  //----------------------

  // Like when publishing a topic, we first create a Topic object with details of the topic's name
  // and message type. Note that we can call publish or subscribe on the same topic object.
  function recieveMessage(){
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/listener',
      messageType : 'std_msgs/String'
    });
    var recievedString;

    // Then we add a callback to be called every time a message is published on this topic.
    listener.subscribe(function(message) {
      console.log('Received message on ' + listener.name + ': ' + message.data);
      recievedString = message.data;
      // If desired, we can unsubscribe from the topic as well.
      listener.unsubscribe();
    });
    return recievedString;
  }
