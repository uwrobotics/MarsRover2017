import React,{Component} from 'react'
import ReactDOM,{ render} from 'react-dom'
//import App from '../components/App'
import Gps from '../components/Gps'
import Wheel from '../components/Wheel'
import Battery from '../components/battery'

function MyButton(props) {
  return (
    <button className="square" onClick={() => props.onClick()}>
      {props.value}
    </button>
  );
}



class RobotPanel extends React.Component {
  changeState(data){
    this.setState(
          {
          battery: {
            percentage: data,
            estimateTime: 6,
            actualRemain:20000
          },

        });
  }

  changeString(data){
    this.setState(
          {
          
          wheels:[{
            working: data,
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
        });
  }

//  *********************** this is our button part *******************************
  connectButtonClick(clas) {     
    var listener = new ROSLIB.Topic({
      ros : this.state.ros,
      name : '/listener',
      messageType : 'std_msgs/String'
    });                               // 5 m      this is for what will happen, if we click those 9 squares
    

    printToScreen(function(data){
      console.log('string is '+  ': ' + data);
      clas.changeString(data);
    });

    function printToScreen(callback){
      listener.subscribe(function(message) {
      //console.log('Received message on Int' + listener.name + ': ' + message.data);
      callback(message.data);  
    });
    }
  }

  disconnectButtonClick() {                                    // 5 m      this is for what will happen, if we click those 9 squares
    //**** connect with roblisjs ************//
    var listener = new ROSLIB.Topic({
      ros : this.state.ros,
      name : '/myListener',
      messageType : 'geometry_msgs/Twist'
    });

    // Then we add a callback to be called every time a message is published on this topic.
    listener.subscribe(function(message) {
      console.log('Linear' + message.data.linear + ': ' + message.data);
      string = message.data;
      //console.log(' recievedString is '+recievedString);
      // If desired, we can unsubscribe from the topic as well.
      //listener.unsubscribe();
    });
  }
  

  

  refreshState(clas){
    var listener = new ROSLIB.Topic({
      ros : this.state.ros,
      name : '/simple_msg',
      messageType : 'std_msgs/Int32'
    });
    console.log("The button works");
    var string;
    printToScreen(function(data){
      console.log('Received message on Int printToScreen' + ': ' + data);
      clas.changeState(data);
    });

   
    function printToScreen(callback){
      listener.subscribe(function(message) {
      //console.log('Received message on Int' + listener.name + ': ' + message.data);
      string = message.data;
      callback(string);  
    });
    }
  }

  

  renderConnectSquare() {
    return <MyButton value="connect" onClick={() => this.connectButtonClick(this)} />
  }

  renderUnConnectSquare() {
    return <MyButton value="disconnect" onClick={() => this.disconnectButtonClick()} />
  }

  renderRefreshSquare() {
    return <MyButton value="Recieveing Data" onClick={() => this.refreshState(this)} />
  }
  
//  **************************** above is button part ******************************


//  ******************** this part define the data of this page *******************


  constructor() {                       // game's constructor, set states
    super();
    this.state = {
      ros : new ROSLIB.Ros(),
      battery: {
        percentage: 100,
        estimateTime: 6,
        actualRemain:20000
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
  }


// **************************** above is the data of the page ********************** 
  
//{this.renderConnectSquare()}
//{this.renderUnConnectSquare()}
//{this.renderRefreshSquare()}
  render() {
  return(
    <div>
        <div id="bng">
        <div className="button">{this.renderRefreshSquare()}</div>
          <div className="col">
            <Battery
              percentage={this.state.battery.percentage}
              estimateTime={this.state.battery.estimateTime}
              actualRemain={this.state.battery.actualRemain}
            />
          </div>
          <div className="col">
           <Gps
              latitude={this.state.gps.latitude}
              longitude={this.state.gps.longitude}
              />
          </div>
        </div>
      <div className="Wheel">
        <Wheel
          wheels={this.state.wheels}
        />
      </div>
      
    </div>);
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

