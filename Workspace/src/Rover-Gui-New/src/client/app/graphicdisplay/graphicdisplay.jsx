import React from 'react';

import ControlBox from './controlbox/controlbox.jsx';
import Map from './map/map.jsx';

class GraphicDisplay extends React.Component {
    
    constructor(props) {
        super(props);
    };

    render() {
        return (
            <div className='graphicdisplay'>
                <ControlBox ros={this.props.ros} dir={this.props.dir}/>
                <Map lat={this.props.lat} lon={this.props.lon} dir={this.props.dir}/>
            </div>
        );
    };
    
}

export default GraphicDisplay;