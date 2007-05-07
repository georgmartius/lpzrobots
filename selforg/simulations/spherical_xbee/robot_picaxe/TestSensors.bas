' Address of Motor Driver
symbol MOTORDRIVER1 = %10110001

' Address of SENSOR Boards
symbol SENSORBOARD1 = %10010000
symbol SENSORBOARD2 = %10010010


main:
  high 7
  pause 100
  low 7
  i2cslave SENSORBOARD1, i2cslow, i2cbyte
  goto sense1234
'  i2cslave MOTORDRIVER1, i2cslow, i2cbyte
'	goto sense11_12
'	let b12 = MOTORDRIVER1
'	goto addrcheck

addrcheck:
	i2cslave b12, i2cslow, i2cbyte	
	serout 7, T4800, ("ADDR:",b12,CR,LF)
  writei2c 0, (180)
	pause 1000
	writei2c 0, (80)
	pause 1000
	let b12 = b12+2
	goto addrcheck
	
		

  
sense11_12:	
  writei2c 0, (b5)
  writei2c 1, (b6)
	let b5 = b5+10
	let b6 = b6-5
	for b12 = 11 to 12
		let b13 = b12
		serout 7, T4800, ("Sensor:")
		gosub print
		readi2c b12, (b13)
		gosub print
	next b12
	pause 1000
	goto sense11_12

sense1234:
	for b12 = 1 to 4
		serout 7, T4800, ("Sensor:")
		readi2c b12, (b13)
		gosub print
	next b12
	pause 1000
	goto sense1234

	
print:
	let b2=b13/100
	let b1=b13/10
	let b1=b1 % 10
	let b0=b13//10
	let b2 = b2+48
	let b1 = b1+48
	let b0 = b0+48
	serout 7, T4800, (b2,b1,b0,CR,LF)
	return






