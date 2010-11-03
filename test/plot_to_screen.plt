set term wxt 0
set logscale y 10
plot 'dat.his' using 1:2 t 'histogram' w histeps
#smooth frequency w histeps
pause -1 "press ENTER"

# now do a scatter plot
set term wxt 1
unset logscale
plot 'dat' using 1:2

pause -1 "press ENTER"
