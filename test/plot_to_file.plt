# set final image size
set term png size 1024, 768

## Making histograms within gnuplot - from http://www.inference.phy.cam.ac.uk/teaching/comput/C++/examples/gnuplot/index.shtml
#bin_width = 1.0; ## edit this 
#bin_number(x) = floor(x/bin_width)
#rounded(x) = bin_width * ( bin_number(x) + 0.5 )
#UNITY = 1
### column number of data to be histogrammed is here assumed to be 1
### - change $1 to another column if desired
#plot 'dat' u (rounded($2)):(UNITY) t 'data' smooth frequency w histeps

# do a log10 histogram plot
set output "dat.log_histogram.png"
set logscale y 10
plot 'dat.his' using 1:2 t 'histogram' w histeps
unset logscale y

# do a linear histogram plot
set output "dat.lin_histogram.png"
plot 'dat.his' using 1:2 t 'histogram' w histeps

# now do a scatter plot
set output "dat.scatter.png"
plot 'dat' using 1:2 t 'data'

