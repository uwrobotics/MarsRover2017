import React from 'react';

import ControlBox from './controlbox/controlbox.jsx';
import Map from './map/map.jsx';

class GraphicDisplay extends React.Component {
    
    constructor(props) {
        super(props);
    };
 
    //1000 to go 10 degrees positive (CCW (left) pan, down tilt)
    //1001 to go down 10 degrees
    moveUp(){
        //ros = this.props.ros;
        console.log('moving up');
        var gimbal = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/gimbal_cmd',
            messageType: 'std_msgs/Int32MultiArray',
        })
        var msg = new ROSLIB.Message({
            data: [-999, 1001],
        });
        gimbal.publish(msg);
    }

    moveLeft(){
        //ros = this.props.ros;
        console.log('moving left');
        var gimbal = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/gimbal_cmd',
            messageType: 'std_msgs/Int32MultiArray',
        })
        var msg = new ROSLIB.Message({
            data: [1000, -999],
        });
        gimbal.publish(msg);
    }

    moveRight(){
        //ros = this.props.ros;
        console.log('moving right');
        var gimbal = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/gimbal_cmd',
            messageType: 'std_msgs/Int32MultiArray',
        })
        var msg = new ROSLIB.Message({
            data: [1001, -999],
        });
        gimbal.publish(msg);
    }

    moveDown(){
        //ros = this.props.ros;
        console.log('moving down');
        var gimbal = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/gimbal_cmd',
            messageType: 'std_msgs/Int32MultiArray',
        })
        var msg = new ROSLIB.Message({
            data: [-999, 1000],
        });
        gimbal.publish(msg);
    }
    moveReset(){
        //ros = this.props.ros;
        console.log('resetting gimbal');
        var gimbal = new ROSLIB.Topic({
            ros: this.props.ros,
            name: '/gimbal_cmd',
            messageType: 'std_msgs/Int32MultiArray',
        })
        var msg = new ROSLIB.Message({
            data: [0, 0],
        });
        gimbal.publish(msg);
    }


    render() {
        return (
            <div className='graphicdisplay'>
                <ControlBox ros={this.props.ros} dir={this.props.dir}/>
                <Map lat={this.props.lat} lon={this.props.lon} dir={this.props.dir}/>
                <div className='buttons'>
                    <button onClick={this.moveUp.bind(this)}>UP</button><br></br>
                    <button onClick={this.moveLeft.bind(this)}>LEFT</button>
                    <button onClick={this.moveRight.bind(this)}>RIGHT</button><br></br>
                    <button onClick={this.moveDown.bind(this)}>DOWN</button><br></br>
                    <button onClick={this.moveReset.bind(this)}>Reset to 0,0</button>
                </div>
            </div>
        );
    };
    
}

export default GraphicDisplay;