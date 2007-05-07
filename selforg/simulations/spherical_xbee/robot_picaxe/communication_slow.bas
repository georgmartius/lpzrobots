' settings: 18X, enhanced compiler!
'
'
' settings: 18X, enhanced compiler!
'
' slow implementation with possibly any number of motors
'
'Protokoll Binary 
' Format: CLXXXXX ...
'  C=command byte 0cccaaaa 
'    ccc Command:  000 R = Reset, 001 D = Dimension, 
'                  010 = Sensors, 011 M = Motors 
'                  100 E = Error, 101 V = Verbose
'                  111 A = Ack (not used here)
'    aaaa Address: 4 subnets - from 0 to 3 and 4 to 7...
'             0,4,8,12  master(s) (PC)
'             rest indiviual addresses
'            }
'  L=number of bytes : format 1lllllll
'    lllllll: Length of message (number of X)
'  X= value byte
'
'  The robot always received 10 bytes, regardless of the packet length. In this
'   implementation (slow) there are mulitple packets posible
'
' Communication (< from Robot, > from Workstation)
' > R1 0
' < D0 2 NumSensors NumMotors
' > M1 2 0 0   ' initial motor values 0
' < S0 3 s1 s2 s3  
' > M1 2 m1 m2 
' < S0 3 s1 s2 s3
' .....
' > M1 2 m1 m2 
' < E0 1 1
' < V0 23 "everything is all right"

' Storage
' PICAXE-18X 80 to 127 ($50 to $7F), 192 to 239 ($C0 to $EF)
'
' $50 number sensors
' $51 number motors
' $52 current motor offset (for multiple motor packets)
' 
' $5F temporary swapping space
' $60-$7F motor values
' $C0-$EF sensors values


' Registers
'  b0:  command received
'  b1:  address received
'  b2 - b9:  data received
'  
'  b10-12: working registers
'  b13: state

' CONFIGURATION

symbol NET = 0 ' subnet number (starts with 48 meaning ascii "0")
symbol ID  = 1 ' my id in subnet


symbol NUMBERSENSORS = 6 ' if you change this make sure you adapt sense 
symbol NUMBERMOTORS = 2 ' if you change this make sure you adapt controlmotors 

symbol BAUD = T4800

' END CONFIGURATION

' commands

symbol  C_RESET  = 0
symbol  C_DIM    = 1
symbol  C_SENSOR = 2
symbol  C_MOTOR  = 3
symbol  C_ERROR  = 4
symbol  C_VERBOSE= 5
symbol  C_ACK    = 7

' states
symbol NOTINIT = 0
symbol INITIALISED = 1 ' in this state we send sensor values and switch to mode sended
symbol SENDED = 2      ' in this state we wait for motor values and switch to mode initialised

' errors
symbol ERROR_WRONGSTATE = 0
symbol ERROR_COMMAND = 1
symbol ERROR_WRONGMOTOR = 2

' address stuff
symbol MASTER = NET * 4
symbol MYADDR = MASTER + ID

' Address of Motor Driver
symbol MOTORDRIVER1 = %10110001

' Address of SENSOR Boards
symbol SENSORBOARD1 = %10010000
symbol SENSORBOARD2 = %10010010



main:
  gosub init
  goto run
  end

' Todo: check for i2c stuff here  	
init:
 	poke $50, NUMBERSENSORS  ' number sensors 
  poke $51, NUMBERMOTORS  ' number motors
  poke $52, 0             ' motor offset
  let b10 = C_VERBOSE*16
  let b10 = b10 + MASTER
	low 7  ' this is for the serial port
 	pause 100   	
 	high 7 ' this is for the serial port
  serout 7, BAUD, (b10,$84,"INIT");
  return

run:
	  serin 7, BAUD, b0,b1,b2,b3,b4,b5,b6,b7,b8,b9
    let b10=b0/128
    if b10!=0 then run ' command byte starts with 0
	  let b10=b1/128
    if b10!=1 then run ' length byte starts with 1
    let b10 = b0 & $0F ' address
    let b0 = b0 / 16   ' command
    let b1 = b1 & $7F ' length
'	  let b12 = b0 + 48: gosub dbg
    if b10!=MYADDR then goto run ' skip packet because is not for us
    select b0 ' b0 contains command (R,M)
      case C_RESET
        gosub senddim
        let b13 = SENDED
      case C_MOTOR
        if b13 != SENDED then 
    	    let b10 = C_ERROR*16
    	    let b10 = b10 + MASTER
				  serout 7, BAUD, (b10,$81,ERROR_WRONGSTATE);
        else
          gosub receivemotors
          gosub controlmotors
          let b13 = INITIALISED
        endif
      else
   	    let b10 = C_ERROR*16
   	    let b10 = b10 + MASTER
			  serout 7, BAUD, (b10,$81,ERROR_COMMAND);
			  end
    endselect
    ' check what to do depending on state
    if b13 = INITIALISED then
    	gosub sense
      gosub sendsensors
      let b13 = SENDED
    endif
  goto run
  
  
    	
senddim:
  peek $50, b11 
  peek $51, b12 
  let b10 = C_DIM*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b10,$82,b11,b12);
  return
  
  
receivemotors:
  peek $51, b10 ' number of motors
  peek $52, b11 ' motor offset
	' if offset is 0 (first packet) and motor number not equal to length of motor packet
	if b11=0 and b10 != b1 then 
	  let b10 = C_ERROR*16
  	let b10 = b10 + MASTER
  	serout 7, BAUD, (b10,$81,ERROR_WRONGMOTOR);  
  	return		
	endif
  
  let b11 = b11 + $60 ' convert to address
  poke b11, b2         ' store motor value 1
  let b11 = b11 + 1
  poke b11, b3
  let b11 = b11 + 1
  poke b11, b4
  let b11 = b11 + 1
  poke b11, b5
  let b11 = b11 + 1
  poke b11, b6
  let b11 = b11 + 1
  poke b11, b7
  let b11 = b11 + 1
  poke b11, b8
  let b11 = b11 + 1
  poke b11, b9

	let b12 = b11 - $60 ' convert back to offset
	if b11>=b10 then  ' read all motor values (no further packet expected)
		poke $52,0 ' offset is 0
	else
		poke $52,b11 ' offset is b11
		' send ack
	  let b10 = C_ACK*16
  	let b10 = b10 + MASTER
  	serout 7, BAUD, (b10,$80);
  	return		
	endif
  return
  
controlmotors:
  ' Define i2c slave address for the Motor Driver
	i2cslave MOTORDRIVER1, i2cslow, i2cbyte
	peek $51, b11 ' number of motors
	if b11!=2 then
	  let b10 = C_ERROR*16
	  let b10 = b10 + MASTER		
		serout 7, BAUD, (b10,$81,ERROR_WRONGMOTOR);  
		end
	else
		let b11= b11-1
  	for b10 = 0 to b11	
    	let b12 = $60+b10
    	peek b12, b12  ' motor value is now in b12
	    writei2c b10,(b12)  
	  next b10
	endif
'  let b10 = C_VERBOSE*16
'  let b10 = b10 + MASTER
'  serout 7, BAUD, (b10,$87,"motor ", b12);  
  return
  
' this function polls all senors and stores the values in the storage
'  here we use also registers b6-.. because we don't need the
'  received data anymore
sense:
  ' get sensors from i2c and store them
	peek $50, b6 ' number of sensors
	let b6 = b6 - 1
	for b7 = 0 to b6	
	  if b7<2 then ' first 2 motor sensors
	    i2cslave MOTORDRIVER1, i2cslow, i2cbyte
	  	let b9 = 11 + b7	  	
			readi2c b9, (b8)
		elseif b7<6 then ' other sensors
		  i2cslave SENSORBOARD1, i2cslow, i2cbyte
	  	let b9 = b7-1 ' range from 1 to 4	  	
			readi2c b9, (b8) ' b8 has motor current
	  else
	    let b8 = 0
		endif
	  let b9 = $C0+b7
	  poke b9, b8    ' sensor value is now in b8 and stored
	next b7
 ' let b10 = C_VERBOSE*16
 ' let b10 = b10 + MASTER
 ' serout 7, BAUD, (b10,$86,"sensed");  
  return
  
  
'  here we use also registers b6-.. because we don't need the
'  received data anymore
sendsensors:
	peek $50, b6 ' number of sensors
  let b10 = C_SENSOR*16
  let b10 = b10 + MASTER
  let b11 = $80 + b6
  serout 7, BAUD, (b10,b11);
	let b6 = b6 - 1
	for b7 = 0 to b6	
		let b9 = $C0+b7 ' address of sensor
	  peek b9, b8    ' sensor value is now in b8
    serout 7, BAUD, (b8)	  
	next b7
  return

memdump:
	for b10 = $50 to $7F
		peek b10, b11		
    serout 7, BAUD, (b10,b11,CR,LF)
  next b10
  return
  
dbg:
	poke $5F, b10
  let b10 = C_VERBOSE*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b10,$85,"DBG:",b12);
  peek $5F, b10
  return
	  




