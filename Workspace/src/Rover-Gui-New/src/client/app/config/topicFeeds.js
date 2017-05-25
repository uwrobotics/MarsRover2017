module.exports = {
   "/topicNameWithArray": [
      {
         "statePath": "path",
         "stateSubPath": "string.here"
      },
      {
         "statePath": "path",
         "stateSubPath": "string.here"
      }
   ],
   "/topicNameWithFloat": {
     "statePath": "path",
     "stateSubPath": "string.here",
     "func": (a,b) => { return a + b }
  },
   "/battery_data": [
      {
         "statePath": "battery",
         "stateSubPath": "percent",
         "func": (i) => { return _.toNumber(i).toFixed(2) }
      },
      {
         "statePath": "battery",
         "stateSubPath": "current",
         "func": (i) => { return _.toNumber(i).toFixed(2) }
      },
      {
         "statePath": "battery",
         "stateSubPath": "voltage",
         "func": (i) => { return _.toNumber(i).toFixed(2) }
      }
   ]
};