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
        };
        this.setUpROS(this.state.ros);
        this.simulateUpdates();
               
    }

    componentDidUpdate(prevProps, prevState) {
        // console.log('battery update');
        // console.log('1: %f', this.state.battery.percent);
    }
    
    simulateUpdates() {
        let inte = setInterval(() => {
            let rand = Math.random();
            let incAmount = rand > 0.5 ? -0.3 : 0.3;
            
            let sensorData = this.state.sensorData;
            sensorData.sensorGroup5.sensors.G5T2.value += incAmount;
            
            if (sensorData.sensorGroup5.sensors.G5T2.value > 100) {
                sensorData.sensorGroup5.sensors.G5T2.value = 0;
            }
            if (sensorData.sensorGroup5.sensors.G5T2.value < 0) {
                sensorData.sensorGroup5.sensors.G5T2.value = 100;
            }
                                    
            this.setState({ sensorData });
        }, 30); 
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

        //update battery data
        sensorData.sensorGroup2.sensors.G2T1.value = this.state.battery.percent;
        sensorData.sensorGroup2.sensors.G2T2.value = this.state.battery.current;
        sensorData.sensorGroup2.sensors.G2T3.value = this.state.battery.voltage;

        this.setState({ sensorData });
    }
    
    initializeRosSubscribers(ros,thisClass) {

        var batteryListener = new ROSLIB.Topic({
            ros : this.state.ros,
            name : '/battery_data',
            messageType : 'std_msgs/Float32MultiArray'
        });

        batteryListener.subscribe(function(message) {
            console.log("got battery message");
            thisClass.setState(
            {
                battery: {
                    percent: +message.data[0].toFixed(2),
                    current: +message.data[1].toFixed(2),
                    voltage: +message.data[2].toFixed(2),
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
