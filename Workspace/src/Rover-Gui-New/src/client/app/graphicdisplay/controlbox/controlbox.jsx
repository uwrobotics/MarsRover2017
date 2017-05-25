import React from 'react';

class ControlBox extends React.Component {
    
    constructor(props) {
        super(props);
        this.panoService;
    };
    
    takePanorama() {
                
        if (!this.panoService) {
            this.panoService = new ROSLIB.Service({
                ros : this.props.ros,
                name : '/panorama_service_name',
                serviceType : 'panorama_service_type'
            });
        }
        
        let panoServiceRequest = new ROSLIB.ServiceRequest({
            a : 1,
            b : 2,
        });
        
        this.panoService.callService(panoServiceRequest, (result) => {
            console.log('Panorama Service Response', result);
        });
        
    };

    render() {
        
        return (
            <div className='controlbox'>
                <div className='subbox pano'>
                    <div className='panobutton' onClick={this.takePanorama.bind(this)}>
                        Take Panorama
                    </div>
                </div>
                <div className='subbox feeds'>
                    <p>Camera Feeds</p>
                    <select name='feed1' ref='feed1'>
                        <option value='firstfeed' selected>First Feed</option>
                        <option value='secondfeed'>Second Feed</option>
                        <option value='thirdfeed'>Third Feed</option>
                    </select>
                    <select name='feed2' ref='feed2'>
                        <option value='firstfeed'>First Feed</option>
                        <option value='secondfeed' selected>Second Feed</option>
                        <option value='thirdfeed'>Third Feed</option>
                    </select>
                </div>
            </div>
        );
    };
    
}

export default ControlBox;