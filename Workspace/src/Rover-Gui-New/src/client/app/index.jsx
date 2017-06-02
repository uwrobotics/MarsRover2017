import React from 'react';
import {render} from 'react-dom';

function requireAll(r) { r.keys().forEach(r); }
requireAll(require.context('./', true, /\.less$/));

import TopBar from './topbar/topbar.jsx';
import SensorDisplay from './sensordisplay/sensordisplay.jsx';
import GraphicDisplay from './graphicdisplay/graphicdisplay.jsx';

import sensorDisplays from './config/sensorDisplays.js';

const MAX_TEMPERATURE = 50; //deg C

class App extends React.Component {
    
    constructor() {
        super();
        this.state = {
            ros: new ROSLIB.Ros(),
            sensorData: sensorDisplays,
            orientation: {
                x: 0,
                y: 0,
                z: 32,
            },
            gps: {
                lat: 38.405905, 
                lon: -110.792088,
            },
            battery: {
                percent: 0,
                current: 0,
                voltage: 0,
                maxCurrent: 100,
                maxVoltage: 12,
                minVoltage: 10,
            },
            roboteq_ch1: {
                current: 0,
                voltage: 0,
                velocity: 0,
                temperature: 0,
            },
            roboteq_ch2: {
                current: 0,
                voltage: 0,
                velocity: 0,
                temperature: 0,
            },
            limitWrist: {
                sw1: 0,
                sw2: 0,
            },
            limitForearm: {
                sw1: 0,
                sw2: 0,
            },
            limitShoulder: {
                sw1: 0,
                sw2: 0,
            },
        };
        this.setUpROS(this.state.ros);
               
    }    
    
    setUpROS(ros) {
        
        ros.on('connection', () => {
            console.log('ROS Connected');
        });
        
        ros.on('close', () => {
            console.log('ROS Closed');
        });
        
        ros.on('error', (err) => {
            console.log('ROS Error', err);
            console.log('Try running roslaunch rosbridge_server rosbridge_websocket.launch')
        });
        
        ros.connect('ws://localhost:9090');
        
        this.initializeRosSubscribers(ros,this);
        
    }

    updateDials(){
        let sensorData = this.state.sensorData;

        //update battery data - example
        sensorData.sensorGroup1.sensors.D1.value = this.state.battery.percent;
        sensorData.sensorGroup1.sensors.D2.value = this.state.battery.current;
        sensorData.sensorGroup1.sensors.D3.value = this.state.battery.voltage;

        //update roboteq left data
        sensorData.sensorGroup2.sensors.D1.value = this.state.roboteq_ch1.current;
        sensorData.sensorGroup2.sensors.D2.value = this.state.roboteq_ch1.voltage;
        sensorData.sensorGroup2.sensors.D3.value = this.state.roboteq_ch1.velocity;
        sensorData.sensorGroup2.sensors.D4.value = this.state.roboteq_ch1.temperature;

        //update roboteq right data
        sensorData.sensorGroup3.sensors.D1.value = this.state.roboteq_ch2.current;
        sensorData.sensorGroup3.sensors.D2.value = this.state.roboteq_ch2.voltage;
        sensorData.sensorGroup3.sensors.D3.value = this.state.roboteq_ch2.velocity;
        sensorData.sensorGroup3.sensors.D4.value = this.state.roboteq_ch2.temperature;

        this.setState({ sensorData });
    }
    
    initializeRosSubscribers(ros,thisClass) {

        //example code
        var batteryListener = new ROSLIB.Topic({
            ros : this.state.ros,
            name : '/battery_data',
            messageType : 'std_msgs/Float32MultiArray',
            throttle_rate: 500, // 500ms between messages means more responsive
        });
        batteryListener.subscribe(function(msg) {
            console.log("got battery message");
            thisClass.setState({
                battery: {
                    percent: +msg.data[0].toFixed(2),
                    current: +msg.data[1].toFixed(2),
                    voltage: +msg.data[2].toFixed(2),
                }
            }, thisClass.updateDials);
        });

        //actual devices
        var roboteqLeftListener = new ROSLIB.Topic({
            ros : this.state.ros,
            name : '/left/feedback',
            messageType : 'roboteq_msgs/Feedback',
            throttle_rate: 500, // 500ms between messages means more responsive
        });
        roboteqLeftListener.subscribe(function(msg) {
            console.log("got roboteq left");
            thisClass.setState({
                roboteq_ch1: {
                    current: msg.motor_current,
                    voltage: msg.supply_voltage,
                    velocity: msg.measured_velocity,
                    temperature: msg.channel_temperature,
                }
            }, thisClass.updateDials);
        });

        
        var roboteqRightListener = new ROSLIB.Topic({
            ros : this.state.ros,
            name : '/right/feedback',
            messageType : 'roboteq_msgs/Feedback',
            throttle_rate: 500, // 500ms between messages means more responsive
        });
        roboteqRightListener.subscribe(function(msg) {
            console.log("got roboteq right");
            thisClass.setState({
                roboteq_ch2: {
                    current: msg.motor_current,
                    voltage: msg.supply_voltage,
                    velocity: msg.measured_velocity,
                    temperature: msg.channel_temperature,
                }
            }, thisClass.updateDials);
        });  

        var navsatListener = new ROSLIB.Topic({
            ros : this.state.ros,
            name : '/navsat/fix',
            messageType : 'sensor_msgs/NavSatFix',
            throttle_rate: 500, // 500ms between messages means more responsive
        });
        navsatListener.subscribe(function(msg) {
            console.log("got gps");
            thisClass.setState({
                gps: {
                    lat: msg.latitude,
                    lon: msg.longitude,
                }
            }, thisClass.updateDials);
        });  

        var limitWristListener = new ROSLIB.Topic({
            ros : this.state.ros,
            name : '/switchesWristFlags',
            messageType : 'std_msgs/UInt8',
            throttle_rate: 500, // 500ms between messages means more responsive
        });
        limitWristListener.subscribe(function(msg) {
            console.log("got gps");
            thisClass.setState({
                limitWrist: {
                    sw1: 1,
                    sw2: 1,
                }
            }, thisClass.updateDials);
        });  

    }
    
    
    
    render() {
        return (
            <div>
                <div className='vgroupflex'>
                    <TopBar />
                    <div className='hgroupflex'>
                        <SensorDisplay sensorData={this.state.sensorData}/>
                        <GraphicDisplay lat={this.state.gps.lat} lon={this.state.gps.lon} dir={this.state.orientation.z} ros={this.state.ros}/>
                    </div>
                </div>
            </div>
        );
    };
}

render(<App/>, document.getElementById('app'));
