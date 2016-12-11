import React, {Component} from 'react'
class Wheel extends Component {                 // this is board class, it has its own properties, it rener squres, pass some of its properties to squares

  render() {
    return (
      <div className="Wheels instruction">
        <h2><b>Wheels</b></h2>
        <p className="data"><h3>{this.props.wheels[0]["name"]}</h3></p>
        <p className="data">{this.props.wheels[0]["working"]}</p>
        <p className="data">{this.props.wheels[0]["status"]}</p>
 
        <p className="data"><h3>{this.props.wheels[1]["name"]}</h3></p>
        <p className="data">{this.props.wheels[1]["working"]}</p>
        <p className="data">{this.props.wheels[1]["status"]}</p>

        <p className="data"><h3>{this.props.wheels[2]["name"]}</h3></p>
        <p className="data">{this.props.wheels[2]["working"]}</p>
        <p className="data">{this.props.wheels[2]["status"]}</p>
    
        <p className="data"><h3>{this.props.wheels[3]["name"]}</h3></p>
        <p className="data">{this.props.wheels[3]["working"]}</p>
        <p className="data">{this.props.wheels[3]["status"]}</p>

      </div>
    );
  }

}

//module.exports = Wheel;
export default Wheel