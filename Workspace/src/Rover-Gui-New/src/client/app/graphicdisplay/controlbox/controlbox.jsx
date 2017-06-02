import React from 'react';

class ControlBox extends React.Component {
    
    constructor(props) {
        super(props);
        this.panoService;
        this.sciService;
        this.state = {
            science: 'Start Science'
        }
    };
    
    takePanorama() {
                
        if (!this.panoService) {
            this.panoService = new ROSLIB.Service({
                ros : this.props.ros,
                name : '/run_pano',
                serviceType : 'python_service_test/PanoService'
            });
        }
        
        let panoServiceRequest = new ROSLIB.ServiceRequest({
            go : 1,
        });
        
        this.panoService.callService(panoServiceRequest, (result) => {
            console.log('Panorama Service Response', result);
        });
        
    };

    startScience(){
        if (this.state.science == 'Start Science'){
            this.setState({science:'Stop Science'});
            console.log('start science');
        } else {
            this.setState({science:'Start Science'});
            console.log('stop science')
        }

        if (!this.sciService) {
            this.sciService = new ROSLIB.Service({
                ros : this.props.ros,
                name : '/start_science',
                serviceType : 'std_srvs/Trigger'
            });
        }
        
        let sciServiceReq = new ROSLIB.ServiceRequest({
        });
        
        this.sciService.callService(sciServiceReq, (result) => {
            console.log('Panorama Service Response', result);
        });

    }
    render() {
        
        return (
            <div className='controlbox'>
                <div className='subbox pano'>
                    <div className='panobutton' onClick={this.takePanorama.bind(this)}>
                        Take Panorama
                    </div>
                </div>
                <div className='subbox science'>
                    <div className='scienceButton' onClick={this.startScience.bind(this)}>
                        {this.state.science}
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