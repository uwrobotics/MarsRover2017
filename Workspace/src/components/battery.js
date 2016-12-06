import React, {Component} from 'react'
class Battery extends Component {                 // this is board class, it has its own properties, it rener squres, pass some of its properties to squares

  render() {
    return (
      <div className="Battery instruction">
        <h2>Battery</h2>
        <p><b>percentge remaining</b></p>

        <div className="data">{this.props.percentage}</div>

        <p><b>estimateTime is </b></p>

        <div className="data">{this.props.estimateTime}</div>

        <p><b>actual remaining battery is</b></p>

        <div className="data">{this.props.actualRemain}</div>

      </div>
    );
  }
}

//module.exports = Battery;
export default Battery