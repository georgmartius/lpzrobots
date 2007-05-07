' settings: 18X, enhanced compiler!
'
'
'Protokoll Binary 
' Format: CLXXXXX ...
'  C=command byte 0cccaaaa 
'    ccc Command:  000 R = Reset, 001 D = Dimension, 
'                  010 = Sensors, 011 M = Motors 
'                  100 E = Error, 101 V = Verbose
'    aaaa Address: 4 subnets - from 0 to 3 and 4 to 7...
'             0,4,8,12  master(s) (PC)
'             rest indiviual addresses
'            }
'  L=number of bytes : format 1lllllll
'    lllllll: Length of message (number of X)
'  X= value byte

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
' $52-$62 motor values
' $C0-$EF sensors values


' Registers
'  b0:  command received
'  b1:  address received
'  
'  b6-b10: working registers
'  b12: parameter0 for functions
'  b13: state

' CONFIGURATION

symbol NET = 0 ' subnet number (starts with 48 meaning ascii "0")
symbol ID  = 1 ' my id in subnet


symbol NUMBERMOTORS = 2 ' if you change this make sure you adapt controlmotors 

symbol BAUD = 2400

' END CONFIGURATION

' commands

symbol  C_RESET  = 0
symbol  C_DIM    = 1
symbol  C_SENSOR = 2
symbol  C_MOTOR  = 3
symbol  C_ERROR  = 4
symbol  C_VERBOSE= 5

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
  pause 500
  gosub init
  goto run
  end

' Todo: check for i2c stuff here  	
init:
	low 7  ' this is for the serial port
 	pause 100   	
 	high 7 ' this is for the serial port
 	poke $50, 2  ' number sensors 
  poke $51, NUMBERMOTORS  ' number motors
  let b10 = C_VERBOSE*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b10,$84,"INIT");
  return

run:
	  serin 7, BAUD, b0
  	serout 7, BAUD, (b0)
	  serin 7, BAUD, b1
'    serin 7, BAUD, b1
  	serout 7, BAUD, (b1)
  ' serout 7, BAUD, (b1)
  	goto run

run2:
    serin 7, BAUD, b0
    let b2=b0/128
    if b2!=0 then run
    serin 7, BAUD, b1
	  let b2=b0/128
    if b2!=1 then run
    let b2 = b0 / 16   ' command
    let b3 = b0 & $0F ' address
    let b4 = b1 & $7F ' length
	  let b12 = b2 + 48: gosub dbg
    if b3!=MYADDR then 
    	for b5 = 1 to b4 ' skip data because the packet is not for us
	      serin 7, BAUD, b6 ' read and forget
      next b5			  		  
	    goto run
    endif
    select b2 ' b0 contains command (R,M)
      case C_RESET
        gosub senddim
        let b13 = INITIALISED
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
  peek $50, b6 
  peek $51, b7 
  let b10 = C_DIM*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b10,$82,b6,b7);
  return
  
  
receivemotors:
  peek $51, b6 ' number of motors
  let b6 = b6 - 1
  for b7 = 0 to b6
    serin 7, BAUD, b8 ' one motor value in b8
    let b9 = $52+b7    ' calculate address for storing motor value
    poke b9, b8        ' store motor value 
  next b7
  let b10 = C_VERBOSE*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b12,$8A,"got motors");  
  return
  
controlmotors:
  ' Define i2c slave address for the Motor Driver
	i2cslave MOTORDRIVER1, i2cslow, i2cbyte
	peek $51, b6 ' number of motors
	if b6!=2 then
	  let b10 = C_ERROR*16
	  let b10 = b10 + MASTER		
		serout 7, BAUD, (b10,$81,ERROR_WRONGMOTOR);  
		end
	else
  	for b7 = 0 to 1	
    	let b9 = $52+b7
    	peek b9, b8  ' motor value is now in b8
	    writei2c b7,(b8)  
	  next b7
	endif
  return
  
' this function polls all senors and stores the values in the storage
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
  let b10 = C_VERBOSE*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b10,$86,"sensed");  
  return
  
  
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
	for b7 = $50 to $7F
		peek b7, b6		
    serout 7, BAUD, (b7,b8,CR,LF)
  next b7
  return
  
dbg:
  let b10 = C_VERBOSE*16
  let b10 = b10 + MASTER
  serout 7, BAUD, (b10,$85,"DBG:",b12);
  return
	  


