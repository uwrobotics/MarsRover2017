import React, {Component} from 'react'
class Gps extends Component {                 // this is board class, it has its own properties, it rener squres, pass some of its properties to squares
  render() {
    return (
      <div className="Gps instruction">
        <h2>Gps data</h2>
        <p><b>latitude</b></p>

        <div className="data">{this.props.latitude}</div>

        <p><b>longitude</b></p>

        <div className="data">{this.props.longitude}</div>

      </div>
    );
  }
}

//module.exports = Gps;
export default Gps