import React from 'react';

import SensorGroup from './sensorgroup/sensorgroup.jsx';

class SensorDisplay extends React.Component {
    
    constructor(props) {
        super(props);
    };
    
    renderSensors() {
        return _.map(this.props.sensorData, (sensorGroup) => {
            return <SensorGroup data={sensorGroup}/>
        });
    }

    render() {
        return (
            <div className='sensordisplay'>
                <div className='sensorcontainer'>
                    {this.renderSensors()}
                </div>
            </div>
        );
    };
    
}

export default SensorDisplay;