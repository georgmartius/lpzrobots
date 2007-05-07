	' Define i2c slave address for the Motor Driver
	i2cslave %10110001, i2cslow, i2cbyte
main:

	high 5
	pause 1000
	low 5
	pause 2000
	goto 	main







