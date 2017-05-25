var webpack = require('webpack');
var path = require('path');
var ExtractTextPlugin = require("extract-text-webpack-plugin");

var BUILD_DIR = path.resolve(__dirname, 'src/client/public');
var APP_DIR = path.resolve(__dirname, 'src/client/app');

const extractLess = new ExtractTextPlugin({
    filename: "[name].[contenthash].css"
});

var config = {
    entry: APP_DIR + '/index.jsx',
    output: {
        path: BUILD_DIR,
        filename: 'bundle.js'
    },
    module : {
        rules: [
            {
                test: /\.less$/,
                use: [{
                    loader: "style-loader"
                }, {
                    loader: "css-loader", options: {
                        sourceMap: true
                    }
                }, {
                    loader: "less-loader", options: {
                        sourceMap: true
                    }
                }]
            },
            {
                test: /\.jsx$/,
                include: APP_DIR,
                use: [{
                    loader: "babel-loader"
                }]
            },
        ]
    },
    plugins: [
        extractLess
    ]
    
};

module.exports = config;