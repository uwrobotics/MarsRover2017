module.exports = {
	devtool:'inline-source-map',
	entry: ['./client/rob.js'],
	output:{
		path: './dist',
		filename:'bundle.js',
		publicPath:'/'
	},
	module:{
		loaders:[
			{
				test: /\.js$/,
				loader: 'babel-loader',
				exclude:/node_modules/,
				query:{
					presets:['react', 'es2015']
				}
			},
			{
				test:/\.css$/, loader:"style-loader!css-loader"	
			}
		]
	}
}
