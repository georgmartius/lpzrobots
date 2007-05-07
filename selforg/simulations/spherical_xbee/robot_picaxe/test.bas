init: low 7
		let b1=48
   	pause 100   	
   	high 7
  	serout 7, T4800, ("TEST")
   	
main:
'	serout 7, T4800, ("Value=",#b1,CR,LF)
	serout 7, T4800, (b1)
	let b1=b1+1
	if b1=126 then
		end 
	else
		goto main
	endif




