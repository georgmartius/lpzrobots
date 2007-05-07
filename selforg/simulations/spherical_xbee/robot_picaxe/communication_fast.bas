' settings: 18X, enhanced compiler!
'
' fast implementation with only 8 possible motors
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
'   implementation (fast) there is only one packet allowed, i.e. max 8 Motors
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
' $52 current motor offset (for multiple motor packets , not used here)
'
' Registers
'  b0:  command received
'  b1:  address received
'  b2 - b9:  data received
'  
'  b10-12: working registers
'  b13: state

' BEGIN CONFIGURATION *************************************************

symbol NET = 0 ' subnet number (starts with 48 meaning ascii "0")
symbol ID  = 1 ' my id in subnet

symbol SENSE_MOTOR_BOARD1 = 0
symbol SENSE_MOTOR_BOARD2 = 0

symbol SENSE_IR_BOARD1 = 1
symbol SENSE_IR_BOARD2 = 0
symbol SENSE_IR_BOARD3 = 0
symbol SENSE_IR_BOARD4 = 0

	' motor 1 and 2	 on driver board 1
symbol CONTROL_MOTOR_BOARD1 = 1
	' motor 3 and 4	 on driver board 2
symbol CONTROL_MOTOR_BOARD2 = 0

symbol BAUD = T4800

' END CONFIGURATION   *************************************************

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
symbol CONTROLLED = 1 ' in this state we send sensor values and switch to mode sended
symbol SENDED = 2      ' in this state we wait for motor values and switch to mode controlled

' errors
symbol ERROR_WRONGSTATE = 0
symbol ERROR_COMMAND = 1
symbol ERROR_WRONGMOTOR = 2

' address stuff
symbol MASTER = NET * 4
symbol MYADDR = MASTER + ID

' Address of Motor Driver
symbol MOTORDRIVER1 = %10110001
symbol MOTORDRIVER2 = %10110011

' Address of SENSOR Boards
symbol SENSORBOARD1 = %10010000
symbol SENSORBOARD2 = %10010010
symbol SENSORBOARD3 = %10010100
symbol SENSORBOARD4 = %10010110



main:
  gosub init
  goto run
  end

' Todo: check for i2c stuff here  	
init:
	let b0 = SENSE_MOTOR_BOARD1 * 2; 
	let b1 = SENSE_MOTOR_BOARD2 * 2; 
	let b10 = b0 + b1;
	let b0 = SENSE_IR_BOARD1 * 4; 
	let b10 = b10 + b0;
	let b0 = SENSE_IR_BOARD2 * 4; 
	let b10 = b10 + b0;
	let b0 = SENSE_IR_BOARD3 * 4; 
	let b10 = b10 + b0;
	let b0 = SENSE_IR_BOARD4 * 4; 
	let b10 = b10 + b0;
	
 	poke $50, b10           ' number sensors 
 	
 	let b0 = CONTROL_MOTOR_BOARD1 * 2;
	let b1 = CONTROL_MOTOR_BOARD2 * 2;
	let b10 = b0 + b1
	 	
  poke $51, b10 ' number motors (<=8 !)
  
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
          gosub receive_control_motors
          let b13 = CONTROLLED
        endif
      else
   	    let b10 = C_ERROR*16
   	    let b10 = b10 + MASTER
			  serout 7, BAUD, (b10,$81,ERROR_COMMAND);
			  end
    endselect
    ' check what to do depending on state
    if b13 = CONTROLLED then
    	gosub sense_send_sensors
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
  
  
receive_control_motors:
  peek $51, b10 ' number of motors
	' motor number not equal to length of motor packet
	if b10 != b1 then 
	  let b10 = C_ERROR*16
  	let b10 = b10 + MASTER
  	serout 7, BAUD, (b10,$81,ERROR_WRONGMOTOR);  
  	return		
	endif

	' motor 1 and 2	 on board 1
	let b11=CONTROL_MOTOR_BOARD1
	if b11=1 then 
		i2cslave MOTORDRIVER1, i2cslow, i2cbyte
		writei2c 0,(b2)  
		writei2c 1,(b3)  
	endif

	 ' motor 3 and 4	 on board 2
	let b12=CONTROL_MOTOR_BOARD2
	if b12=1 then ' check which motor value to take
		i2cslave MOTORDRIVER2, i2cslow, i2cbyte
		if b11 = 1 then
			writei2c 0,(b4)  
			writei2c 1,(b5)  
		else 
			writei2c 0,(b2)  
			writei2c 1,(b3)  
		endif
	endif
  return

  
' this function polls all senors and 
' 
sense_send_sensors:
  ' get sensors from i2c and store them
	peek $50, b6 ' number of sensors	
	let b10 = C_SENSOR*16
  let b10 = b10 + MASTER
  let b11 = $80 + b6
  serout 7, BAUD, (b10,b11);
   
	let b12=SENSE_MOTOR_BOARD1
	if b12 = 1 then
		let b12 = MOTORDRIVER1
		gosub sense_send_motorcurrent
	endif

	let b12=SENSE_MOTOR_BOARD2
	if b12 = 1 then
		let b12 = MOTORDRIVER2
		gosub sense_send_motorcurrent
	endif
	
	let b12=SENSE_IR_BOARD1
	if b12 = 1 then
		let b12 = SENSORBOARD1
		gosub sense_send_ir
	endif
	let b12=SENSE_IR_BOARD2
	if b12 = 1 then
		let b12 = SENSORBOARD2
		gosub sense_send_ir
	endif
	let b12=SENSE_IR_BOARD3
	if b12 = 1 then
		let b12 = SENSORBOARD3
		gosub sense_send_ir
	endif
	let b12=SENSE_IR_BOARD4
	if b12 = 1 then
		let b12 = SENSORBOARD4
		gosub sense_send_ir
	endif
	
 ' let b10 = C_VERBOSE*16
 ' let b10 = b10 + MASTER
 ' serout 7, BAUD, (b10,$86,"sensed");  
  return

sense_send_motorcurrent:
	i2cslave b12, i2cslow, i2cbyte
	readi2c 11, (b8)	
	readi2c 12, (b9)
	serout 7, BAUD, (b8,b9)
	return
  
sense_send_ir:
  i2cslave b12, i2cslow, i2cbyte
	readi2c 1, (b7)	
	readi2c 2, (b8)
	readi2c 3, (b9)	
	readi2c 4, (b10)
	serout 7, BAUD, (b7,b8,b9,b10)
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
	  



