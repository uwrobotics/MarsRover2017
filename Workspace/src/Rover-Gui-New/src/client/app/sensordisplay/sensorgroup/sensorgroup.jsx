import React from 'react';

import DialGraph from './dialgraph/dialgraph.jsx';

class SensorGroup extends React.Component {
    
    constructor(props) {
        super(props);
        this.state = {
            urgencyStatus: 0
        };
    };
    
    componentWillReceiveProps(nextProps) {
        let maxUrgency = 0;
        _.forEach(nextProps.data.sensors, (sensor) => {
            let urgency = 0;
            if (_.isNumber(sensor.value + sensor.warnValue + sensor.criticalValue)) {
                if (sensor.value >= sensor.criticalValue) {
                    urgency = 2;
                } else if (sensor.value >= sensor.warnValue) {
                    urgency = 1;
                }
            }
            if (urgency > maxUrgency) {
                maxUrgency = urgency;
            }
        });
        this.setState({
            urgencyStatus: maxUrgency
        })
    }
    
    renderSensors() {
        
        return _.map(this.props.data.sensors, (sensor) => {
            
            if (sensor.type === 'temp') {
                return (<DialGraph 
                    currentValue={sensor.value}
                    minValue={sensor.minValue}
                    warnValue={sensor.warnValue}
                    criticalValue={sensor.criticalValue}
                    maxValue={sensor.maxValue}
                    units='°C'
                    displayName={sensor.displayName}
                />)
            } else if (sensor.type === 'current') {
                return (<DialGraph 
                    currentValue={sensor.value} 
                    minValue={sensor.minValue}
                    warnValue={sensor.warnValue}
                    criticalValue={sensor.criticalValue}
                    maxValue={sensor.maxValue}
                    units='A' 
                    displayName={sensor.displayName}
                />)
            } else if (sensor.type === 'voltage') {
                return (<DialGraph 
                    currentValue={sensor.value} 
                    minValue={sensor.minValue}
                    warnValue={sensor.warnValue}
                    criticalValue={sensor.criticalValue}
                    maxValue={sensor.maxValue}
                    units='V' 
                    displayName={sensor.displayName}
                />)
            }
        })
    }

    render() {
        let urgencyStatus = 'okay';
        if (this.state.urgencyStatus === 1) {
            urgencyStatus = 'warning';
        } else if (this.state.urgencyStatus === 2) {
            urgencyStatus = 'critical';
        }
        
        
        return (
            <div className={`sensorgroup ${urgencyStatus}`}>
                <div className='groupname'>
                    {this.props.data.groupDisplayName}
                </div>
                <div className='sensorblock'>
                    {this.renderSensors()}
                </div>
            </div>
        );
    };
    
}

export default SensorGroup;