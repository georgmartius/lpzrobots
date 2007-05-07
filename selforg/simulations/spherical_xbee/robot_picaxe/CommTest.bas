symbol BAUD = T4800

main:
  low 7
  pause 100
  high 7
	serout 7, T4800, ("LONG TEST",CR,LF)
'  goto comm
'	gosub speedinit
'  goto speed
	goto speed2

comm:
	serin 7, T4800, b0 ' ,b1 ' ,b2,b3,b4,b5,b6,b7,b8,b9
	serout 7, T4800, (b0) ' ,b1,b2,b3,b4,b5,b6,b7,b8,b9)
	goto comm

speedinit:
	serin 7, T4800, b0,b1,b2,b3,b4,b5,b6,b7,b8,b9
  let b10 = 1*16
  let b10 = b10 + 0
  serout 7, BAUD, (b10,$82,2,2);
  return

speed:
	serin 7, T4800, b0,b1,b2,b3,b4,b5,b6,b7,b8,b9
	let b10 = 2*16
  let b10 = b10 + 0
  let b12=b12+1;
  let b13=b13-2;
  serout 7, BAUD, (b10,$82,b12,b13);  
	goto speed


speed2:
	serin 7, T4800, b0,b1,b2,b3,b4,b5,b6,b7,b8,b9
  let b12=b12+1;
  let b13=b13-2;
  for b0 = 1 to 10
'  	if b0 = 3 then	
'  		serout 7, BAUD, (b12)
'  	else if b0 = 4 then	
'  		serout 7, BAUD, (b13)
'  	else 
  		serout 7, BAUD, (0)
'  	endif
  next b0
  goto speed2
	
