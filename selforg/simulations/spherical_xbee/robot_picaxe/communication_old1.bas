' settings: 18X, enhanced compiler!
'
'
'Protokoll Binary 
' Format: CAXXXXX ...
'  C=command byte {D = Dimension, S = Senors, M = Motors, R = Reset, 
'                  V = verbose debug/error message}
'  A=address {16 subnets - from 0 to 15 and 16 to 31...
'             15,31,... broadcast
'             0,16,...  master(s) (PC)
'             rest indiviual addresses
'            }
'  X=value byte

' Communication (< from Robot, > from Workstation)
' > R 0
' < "###########!" 'initialiser
' < D  NumSensors NumMotors
' < S s1 s2 ....  
' > M m1 m2 ....
' < S s1 s2 ...
' .....
'.< V "robot has bad mood :-)"
' < E code ' see senderror function

'Protokoll Binary 
' like ASCII, but with command format: CXXX...
'  C=command byte {'D' = Dimension, 'S' = Senors, 'M' = Motors, 'R' = Reset}


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

symbol NET = 3 ' subnet number (starts with 48 meaning ascii "0")
symbol ID  = 1 ' my id in subnet


symbol NUMBERMOTORS = 2 ' if you change this make sure you adapt controlmotors 

symbol BAUD = T4800

' END CONFIGURATION

' states
symbol NOTINIT = 0
symbol INITIALISED = 1 ' in this state we send sensor values and switch to mode sended
symbol SENDED = 2      ' in this state we wait for motor values and switch to mode initialised

' address stuff
symbol MASTER = NET * 16
symbol MYADDR = MASTER + ID
symbol BROADCAST = MASTER + 15

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
 	poke $50, 6  ' number sensors 
  poke $51, NUMBERMOTORS  ' number motors
  return


run:
    serin 7, BAUD, b0
    serin 7, BAUD, b1
    if b1!=BROADCAST and b1!=MYADDR then run
    select b0 ' b0 contains command (R,M)
      case "R"
        gosub senddim
        let b13 = INITIALISED
      case "M"
        if b13 != SENDED then 
    	    let b12 = 1
				  serout 7, BAUD, ("V",MASTER," Got motors in wrong state",CR,LF);
        else
          gosub receivemotors
          gosub controlmotors
          let b13 = INITIALISED
        endif
      else
			  serout 7, BAUD, ("V",MASTER," Unknown cmd!",CR,LF);      
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
  serout 7, BAUD, ("##################!"); 
  serout 7, BAUD, ("D",MASTER,b6,b7);
  return
  
  
receivemotors:
  peek $51, b6 ' number of motors
  let b6 = b6 - 1
  for b7 = 0 to b6
    serin 7, BAUD, b8 ' one motor value in b8
    let b9 = $52+b7    ' calculate address for storing motor value
    poke b9, b8        ' store motor value 
  next b7
  serout 7, BAUD, ("V",MASTER," got motors",CR,LF);  
  return
  
controlmotors:
  ' Define i2c slave address for the Motor Driver
	i2cslave MOTORDRIVER1, i2cslow, i2cbyte
	peek $51, b6 ' number of motors
	if b6!=2 then
		serout 7, BAUD, ("V",MASTER," Wrong motor number!",CR,LF);      
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

  serout 7, BAUD, ("V",MASTER," sensed", CR,LF);
  return
  
  
sendsensors:
  serout 7, BAUD, ("S",MASTER);
	peek $50, b6 ' number of sensors
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
  
  

