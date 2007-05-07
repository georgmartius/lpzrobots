
main:
' Define i2c slave address for the Motor Driver
	i2cslave %10110001, i2cslow, i2cbyte
	for b0 = 128 to 150	
' Send the motor speed
	  writei2c 0,(b0)
	  pause 500
	next b0
		







