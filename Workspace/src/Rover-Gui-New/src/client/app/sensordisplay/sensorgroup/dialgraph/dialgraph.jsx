import React from 'react';
import CanvasMeter from 'canvas-meter';

class DialGraph extends React.Component {
    
    constructor(props) {
        super(props);
    };
    
    componentDidMount() {
        this.updateCanvas();
    }
    
    componentDidUpdate() {
        this.updateCanvas();
    }
    
    updateCanvas() {
        const meter = new CanvasMeter(this.refs.canvas);
        meter.setOptions({
            meter: {
                colors: {
                    background: '#333',
                    pointer: '#000'
                },
                arc: {
                    size: 0.9,
                    width: 5
                },
                ramp: [
                    {stop:0, color:'#0F0'},
                    {stop:0.1, color:'#0F0'},
                    {stop:0.5, color:'#EE0'},
                    {stop:0.75, color:'#FF6347'},
                    {stop:1, color:'#F00'},
                ],
                ticks:{
                    major:{
                      count:5
                    },
                    minor: {
                      count:4,
                      width:1
                    },
                    pointer: {
                      height: 20,
                      width: 40
                    }
                },
                offsets:{
                    tick: -2,
                    tickLabel:24,
                    value: -6,
                    pointer: 15,
                    label: 19,
                    meter: -26
                },
                fonts: {
                    label:'10pt sans',
                    tickLabel:'8pt sans',
                    ideal:'16pt sans',
                    value:'14pt sans',
                  },
                formatters: {
                    value:(v) => typeof v===typeof undefined ? "" :`${v.toFixed(0).toString()}${this.props.units}`,
                    ideal:(v) => '',
                    label:(v) => typeof v===typeof undefined ? "" : `${v.toString()}`,
                    tickLabel:(v) => typeof v===typeof undefined ? "" : `${Math.round(v).toString()}`,
                },
            }
        });
        meter.draw(this.props.currentValue,this.props.minValue,this.props.maxValue,0, this.props.displayName);
    }

    render() {
        const meterState = this.props.currentValue < this.props.warnValue ? 'okay' : (this.props.currentValue < this.props.criticalValue ? 'warning' : 'critical');
        
        return (
            <div className='dialgraph'>
                <canvas className={`graphic ${meterState}`} ref="canvas" width={150} height={100}/>
            </div>
        );
    };
    
}

export default DialGraph;