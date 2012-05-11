/********************************************************************************

			The reference programm to control the e-puck with the PC      
			Version 2.0							                          
			Michael Bonani


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Michael Bonani

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \brief The main file of the programm.
 *
 * The sercom works as folowing. Into the infinite loop we look if something is
 * coming from the uart1. If a good command is coming we branch it to the correct
 * code and we send back at first the comand followed by the result (if there is one).
 * \n \n For exemple if the "a + return" is coming, we return first the command: "a"
 * and then the three values of the accelerometer separated by a coma, the returned
 * value will be for exemple: "a,2001,2300,1890+return"
 * \n \n Another exemple is controling the motors. In this case we receive for exemple
 * "d,200,-200+return". Then we put the motor speed left on 200 and motor speed right
 * on -200 and send back "d+return".
 * \warning To compile the programm, you HAVE to compile with a "large data model" option you find in built option project 
 * \n The linker command lign must be -mlarge-data
 * \n or you set the define variable MIC_SAMP_NB in the file library\a_d\advance_ad_scan\e_ad_con.h with 100
 * \n It must look like this: #define MIC_SAMP_NB 100
 * \author Jonathan Besuchet
 */

/*! \mainpage BTcom programm
 * \section intro_sec Introduction
 * BTcom is a simple text-based protocol that allows the PC to control the e-puck via Bluetooth. With BtCom, the PC can read
 * the e-puck's sensors and set the e-puck's actuators. The PC can be a human using a serial communication program
 * (such as Hyperterminal) or a program using a serial port library for exemple you can use the e-puck_monitor.exe
 * locate in "tool\e-puck_monitor", a part of the transfert data can be also done in a binary mode(big endian).
 *
 * \section sect_run Compiling and running
 * First of all you have to compile the programm.
 * \n \n Flash the programm to the e-puck.
 * \warning To compile the programm, you HAVE to compile with a "large data model" option you find in built option project 
 * \n The linker command lign must be -mlarge-data 
 * \n or you set the define variable MIC_SAMP_NB in the file library\a_d\advance_ad_scan\e_ad_con.h with 100
 * \n It must look like this: #define MIC_SAMP_NB 100
 * 
 * \section sect_hyperterminal Using sercom with Hyperterminal
 * Pair the e-puck with your computer: this will create a new
 * virtual COM port that you can use to communicate with the e-puck.
 * \n With Hyperterminal, connect to this COM port and play with the e-puck serial protocol. The available commands
 * are listed in this file: http://asl.epfl.ch/epfl/education/courses/MicroInfo2/TP/sercom1.02.pdf . You can also
 * press 'H' + return to see the list of available commands.
 * 
 * \section link_sec External links
 * - http://www.e-puck.org/                 The official site of the e-puck
 * - https://gna.org/projects/e-puck/       The developpers area at gna
 * - http://lsro.epfl.ch/                   The site of the lab where the e-puck has been created
 * - http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45 The license
 *
 * \author Code: Micahael Bonani \n Doc: Jonathan Besuchet
 */

#include <p30f6014A.h>

//#define FLOOR_SENSORS	// define to enable floor sensors
//#define LIS_SENSORS_TURRET
#define IR_RECIEVER

#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include <motor_led/e_epuck_ports.h>
#include <motor_led/e_init_port.h>
#include <motor_led/advance_one_timer/e_led.h>
#include <motor_led/advance_one_timer/e_motors.h>

#include <uart/e_uart_char.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>
#include <a_d/advance_ad_scan/e_acc.h>
#include <a_d/advance_ad_scan/e_prox.h>
#include <a_d/advance_ad_scan/e_micro.h>
#include <motor_led/advance_one_timer/e_agenda.h>
#include <camera/fast_2_timer/e_poxxxx.h>
#include <codec/e_sound.h>

#ifdef LIS_SENSORS_TURRET
//#include <contrib/LIS_sensors_turret/e_devantech.h>
#include <contrib/LIS_sensors_turret/e_sharp.h>
#include <contrib/LIS_sensors_turret/e_sensext.h>
#include <I2C/e_I2C_master_module.h>
#include <I2C/e_I2C_protocol.h>
#endif

#ifdef FLOOR_SENSORS
#include <./I2C/e_I2C_protocol.h>
#endif 

#ifdef IR_RECIEVER
#include <motor_led/advance_one_timer/e_remote_control.h>
#define SPEED_IR 600
#endif 

#define uart_send_static_text(msg) do { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); } while(0)
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)


static char buffer[39*52*2+3+80];
	
extern int e_mic_scan[3][MIC_SAMP_NB];
extern unsigned int e_last_mic_scan_id;


/* \brief The main function of the programm */
int main(void) {
	char c,c1,c2,wait_cam=0;
	int	i,j,n,speedr,speedl,positionr,positionl,LED_nbr,LED_action,accx,accy,accz,selector,sound;
	int cam_mode,cam_width,cam_heigth,cam_zoom,cam_size;
	static char first=0;
	char *address;
	char *ptr;
#ifdef LIS_SENSORS_TURRET
	int sensext_pres=1, sensext_param[2], sensext_debug=0; 
	unsigned int sensext_value[2];
#endif
#ifdef IR_RECIEVER
	char ir_move = 0,ir_address= 0, ir_last_move = 0;
#endif
	TypeAccSpheric accelero;
	e_init_port();    // configure port pins
	e_start_agendas_processing();
	e_init_motors();
	e_init_uart1();   // initialize UART to 115200 Kbaud
	e_init_ad_scan(ALL_ADC);
#ifdef FLOOR_SENSORS
	#ifndef LIS_SENSORS_TURRET
		e_i2cp_init();
	#endif
#endif

#ifdef IR_RECIEVER
	e_init_remote_control();
#endif
	if(RCONbits.POR) {	// reset if power on (some problem for few robots)
		RCONbits.POR=0;
		RESET();
	}
	
	/*Cam default parameter*/
	cam_mode=RGB_565_MODE;
	cam_width=40;
	cam_heigth=40;
	cam_zoom=8;
	cam_size=cam_width*cam_heigth*2;
	e_poxxxx_init_cam();
	e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,(ARRAY_HEIGHT-cam_heigth*cam_zoom)/2,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode);
    e_poxxxx_set_mirror(1,1);
    e_poxxxx_write_cam_registers();

#ifdef LIS_SENSORS_TURRET //check if sensor extension is present and initalizes ports accordingly 

	e_i2cp_init();
	e_i2cp_enable();
	sensext_debug = e_i2cp_write (I2C_ADDR_SENSEXT , 0, 49);
	e_i2cp_disable();
	
	// Wait for I2C eeprom on tourret to answer. 
	e_activate_agenda(e_stop_sensext_wait,0);
	e_start_sensext_wait();									
	e_set_agenda_cycle(e_stop_sensext_wait, 100);
	while(e_get_sensext_wait());
	e_set_agenda_cycle(e_stop_sensext_wait, 0);
	
	e_i2cp_enable();
	sensext_debug = e_i2cp_read(I2C_ADDR_SENSEXT, 0);
	e_i2cp_disable();
	
	if(sensext_debug!=49) // no SENSORS_TURRET reply
	{
		sensext_pres=0;
	
	}
	else
	{
		sensext_pres=1;
		e_init_sensext();
		e_init_sharp();						//a executer s'il y a la tourelle
	//	uart_send_static_text("ePic\r\n");
	}	
#endif

    
    e_acc_calibr();
	e_calibrate_ir();
    
	uart_send_static_text("\f\a"
	                      "WELCOME to the SerCom protocol on e-Puck\r\n"
	                      "the EPFL education robot type \"H\" for help\r\n");
	while(1) {
		while (e_getchar_uart1(&c)==0)
		#ifdef IR_RECIEVER
		{
			
			ir_move = e_get_data();
			ir_address = e_get_address();
			if (((ir_address ==  0)||(ir_address ==  8))&&(ir_move!=ir_last_move)){
				switch(ir_move)
				{
					case 1:
						speedr = SPEED_IR;
						speedl = SPEED_IR/2;
						break;
					case 2:
						speedr = SPEED_IR;
						speedl = SPEED_IR;
						break;
					case 3:
						speedr = SPEED_IR/2;
						speedl = SPEED_IR;
						break;
					case 4:
						speedr = SPEED_IR;
						speedl = -SPEED_IR;
						break;
					case 5:
						speedr = 0;
						speedl = 0;
						break;
					case 6:
						speedr = -SPEED_IR;
						speedl = SPEED_IR;
						break;
					case 7:
						speedr = -SPEED_IR;
						speedl = -SPEED_IR/2;
						break;
					case 8:
						speedr = -SPEED_IR;
						speedl = -SPEED_IR;
						break;
					case 9:
						speedr = -SPEED_IR/2;
						speedl = -SPEED_IR;
						break;
					case 0:
						if(first==0){
							e_init_sound();
							first=1;
						}
						e_play_sound(11028,8016);
						break;
					default:
						speedr = speedl = 0;
				}
				ir_last_move = ir_move;
				e_set_speed_left(speedl);
				e_set_speed_right(speedr);
				}
	
		}
#else 
		;
#endif
		if (c<0) { // binary mode (big endian)
			i=0;
			do {
				switch(-c) { 
				case 'a':  // Read acceleration sensors in a non filtered way, some as ASCII
					accx=e_get_acc(0);
					accy=e_get_acc(1);
					accz=e_get_acc(2);
					ptr=(char *)&accx;
					buffer[i++]=accx & 0xff;
					buffer[i++]=accx>>8;

					buffer[i++]=accy & 0xff;
					buffer[i++]=accy>>8;

					buffer[i++]=accz & 0xff;
					buffer[i++]=accz>>8;

					break;
				case 'A': // read acceleration sensors
					accelero=e_read_acc_spheric();
					ptr=(char *)&accelero.acceleration;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
				
					ptr=(char *)&accelero.orientation;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
		
					ptr=(char *)&accelero.inclination;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
					ptr++;
					buffer[i++]=(*ptr);
				
					break;
				case 'D': // set motor speed
					while (e_getchar_uart1(&c1)==0);
					while (e_getchar_uart1(&c2)==0);
					speedl=(unsigned char)c1+((unsigned int)c2<<8);
					while (e_getchar_uart1(&c1)==0);
					while (e_getchar_uart1(&c2)==0);
					speedr=(unsigned char)c1+((unsigned  int)c2<<8);
					e_set_speed_left(speedl);
					e_set_speed_right(speedr);
					break;
				case 'E': // get motor speed
					buffer[i++]=speedl & 0xff;
					buffer[i++]=speedl >> 8;

					buffer[i++]=speedr & 0xff;
					buffer[i++]=speedr >> 8;

					break;
				case 'I': // get camera image
					e_poxxxx_launch_capture(&buffer[i+3]);
					wait_cam=1;
					buffer[i++]=(char)cam_mode&0xff;//send image parameter
					buffer[i++]=(char)cam_width&0xff;
					buffer[i++]=(char)cam_heigth&0xff;
					i+=cam_size;
					break;
				case 'L': // set LED
					while (e_getchar_uart1(&c1)==0);
					while (e_getchar_uart1(&c2)==0);
					switch(c1) {
					case 8:
					  e_set_body_led(c2);
					  break;
					case 9:
					  e_set_front_led(c2);
					  break;
					default:
            e_set_led(c1,c2);
            break;
          }
					break;
				case 'M': // optional floor sensors
#ifdef FLOOR_SENSORS
					e_i2cp_enable();

					for (j=0; j<3; ++j) {
						buffer[i++] = e_i2cp_read(0xC0,2*j+1);
						buffer[i++] = e_i2cp_read(0xC0,2*j);
					}

					e_i2cp_disable();
#else
					for(j=0;j<6;j++) buffer[i++]=0;
#endif
					break;
				case 'N': // read proximity sensors
					for(j=0;j<8;j++) {
						n=e_get_calibrated_prox(j);
						buffer[i++]=n&0xff;
						buffer[i++]=n>>8;
					}
					break;
				case 'O': // read light sensors
					for(j=0;j<8;j++) {
						n=e_get_ambient_light(j);
						buffer[i++]=n&0xff;
						buffer[i++]=n>>8;
					}
					break;
				case 'Q': // read encoders
                    n=e_get_steps_left();
					buffer[i++]=n&0xff;
					buffer[i++]=n>>8;
                    n=e_get_steps_right();
					buffer[i++]=n&0xff;
					buffer[i++]=n>>8;
					break;
				case 'u': // get last micro volumes
					n=e_get_micro_volume(0);
					buffer[i++]=n&0xff;
					buffer[i++]=n>>8;

					n=e_get_micro_volume(1);
					buffer[i++]=n&0xff;
					buffer[i++]=n>>8;

					n=e_get_micro_volume(2);
					buffer[i++]=n&0xff;
					buffer[i++]=n>>8;
					break;
				case 'U': // get micro buffer
					address=(char *)e_mic_scan;
					e_send_uart1_char(address,600);//send sound buffer
					n=e_last_mic_scan_id;//send last scan
					buffer[i++]=n&0xff;
					break;
			case 'W': // read Devantec ultrasonic range sensor or Sharp Ir sensor (optional)
#ifdef LIS_SENSORS_TURRET
					while (e_getchar_uart1(&c1)==0);
					while (e_getchar_uart1(&c2)==0);
					sensext_param[0] = (unsigned char)c1+((unsigned int)c2<<8);
//			sscanf(buffer,"W,%d\r",&sensext_param[0]);
				if (sensext_pres)				// If the TP_sensors tourret is present
				{
					if(e_sensext_process(sensext_param, sensext_value))
					{
						switch (sensext_param[0])
						{
							case -1:	//i2c SRFxxx
								buffer[i++] = sensext_value[0]&0xff;
								buffer[i++] = sensext_value[0]>>8;
							
								buffer[i++] = sensext_value[1]&0xff;
								buffer[i++] = sensext_value[1]>>8;
//								sprintf(buffer,"w,%u,%u\r\n", sensext_value[0],  sensext_value[1]);
								break;
							case -2:	// i2c cmps03
								buffer[i++] = sensext_value[0]&0xff;
								buffer[i++] = sensext_value[0]>>8;
							
								buffer[i++] = sensext_value[1]&0xff;
								buffer[i++] = sensext_value[1]>>8;
//								sprintf(buffer,"w,%d,%d\r\n",  sensext_value[0],  sensext_value[1]);	
								break;
							default: //analog (sharp,...)
								buffer[i++] = (sensext_value[0]&0xff);
								buffer[i++] = (sensext_value[0]>>8);
//							sprintf(buffer,"w,%d\r\n",  sensext_value[0]);
								break;
						}
//						uart_send_text(buffer);
					}
					else
					{
						switch(sensext_param[0])
						{
							case -1:
							case -2:
								sensext_value[0] = -1;
								sensext_value[1] = -1;
								buffer[i++] = sensext_value[0]&0xff;
								buffer[i++] = sensext_value[0]>>8;
							
								buffer[i++] = sensext_value[1]&0xff;
								buffer[i++] = sensext_value[1]>>8;
								break;
							default:
								sensext_value[0] = -1;
								sensext_value[1] = -1;
								buffer[i++] = sensext_value[0]&0xff;
								buffer[i++] = sensext_value[0]>>8;
								break;															
						}
//						uart_send_static_text("wrong parameter\r\n");
					}
				
				}
				else
				{
					uart_send_static_text("LIS sensors turret not present\r\n");
				}
#endif
				break;
				default: // silently ignored
					break;
				}
				while (e_getchar_uart1(&c)==0); // get next command
			} while(c);
			if (i!=0){
				if (wait_cam) {
					wait_cam=0;
					while(!e_poxxxx_is_img_ready());
				}
				e_send_uart1_char(buffer,i); // send answer
				while(e_uart1_sending());
			}
		} else if (c>0) { // ascii mode
			while (c=='\n' || c=='\r')
				 e_getchar_uart1(&c);
			buffer[0]=c;
			i = 1;
			do if (e_getchar_uart1(&c)) 
				buffer[i++]=c;
			while (c!='\n' && c!='\r');
			buffer[i++]='\0';
			buffer[0]=toupper(buffer[0]); // we also accept lowercase letters
			switch (buffer[0]) {
			case 'A': // read accelerometer
				accx=e_get_acc(0);
				accy=e_get_acc(1);
				accz=e_get_acc(2);
				sprintf(buffer,"a,%d,%d,%d\r\n",accx,accy,accz);				
				uart_send_text(buffer);
			/*	accelero=e_read_acc_spheric();
				sprintf(buffer,"a,%f,%f,%f\r\n",accelero.acceleration,accelero.orientation,accelero.inclination);				
				uart_send_text(buffer);*/
				break;
			case 'B': // set body led
				sscanf(buffer,"B,%d\r",&LED_action);
			 	e_set_body_led(LED_action);
				uart_send_static_text("b\r\n");
				break;
			case 'C': // read selector position
				selector = SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
				sprintf(buffer,"c,%d\r\n",selector);
				uart_send_text(buffer);
				break;
			case 'D': // set motor speed
				sscanf(buffer, "D,%d,%d\r", &speedl, &speedr);
				e_set_speed_left(speedl);
				e_set_speed_right(speedr);
				uart_send_static_text("d\r\n");
				break;
			case 'E': // read motor speed
				sprintf(buffer,"e,%d,%d\r\n",speedl,speedr);
				uart_send_text(buffer);
				break; 
			case 'F': // set front led
				sscanf(buffer,"F,%d\r",&LED_action);
				e_set_front_led(LED_action);
				uart_send_static_text("f\r\n");
				break;
#ifdef IR_RECIEVER				
			case 'G':
                  sprintf(buffer,"g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n", e_get_check(), e_get_address(), e_get_data());
                  uart_send_text(buffer);
                  break;
#endif
			case 'H': // ask for help
				uart_send_static_text("\n");
				uart_send_static_text("\"A\"         Accelerometer\r\n");
				uart_send_static_text("\"B,#\"       Body led 0=off 1=on 2=inverse\r\n");
				uart_send_static_text("\"C\"         Selector position\r\n");
				uart_send_static_text("\"D,#,#\"     Set motor speed left,right\r\n");
				uart_send_static_text("\"E\"         Get motor speed left,right\r\n");
				uart_send_static_text("\"F,#\"       Front led 0=off 1=on 2=inverse\r\n");
#ifdef IR_RECIEVER
				uart_send_static_text("\"G\"         IR receiver\r\n");
#endif
				uart_send_static_text("\"H\"	     Help\r\n");
				uart_send_static_text("\"I\"         Get camera parameter\r\n");
				uart_send_static_text("\"J,#,#,#,#\" Set camera parameter mode,width,heigth,zoom(1,4 or 8)\r\n");
				uart_send_static_text("\"K\"         Calibrate proximity sensors\r\n");
				uart_send_static_text("\"L,#,#\"     Led number,0=off 1=on 2=inverse\r\n");
#ifdef FLOOR_SENSORS
				uart_send_static_text("\"M\"         Floor sensors\r\n");
#endif
				uart_send_static_text("\"N\"         Proximity\r\n");
				uart_send_static_text("\"O\"         Light sensors\r\n");
				uart_send_static_text("\"P,#,#\"     Set motor position left,right\r\n");
				uart_send_static_text("\"Q\"         Get motor position left,right\r\n");
				uart_send_static_text("\"R\"         Reset e-puck\r\n");
				uart_send_static_text("\"S\"         Stop e-puck and turn off leds\r\n");
				uart_send_static_text("\"T,#\"       Play sound 1-5 else stop sound\r\n");
				uart_send_static_text("\"U\"         Get microphone amplitude\r\n");
				uart_send_static_text("\"V\"         Version of SerCom\r\n");
#ifdef LIS_SENSORS_TURRET
				if (sensext_pres)				// If the TP_sensors tourret is present
					uart_send_static_text("\"W,#\"       Sensor extension turret. Analog: W,0; analog 5 LEDs: W,0->31; i2c dist: W,-1; i2c comp: W,-2\r\n");
				else
					uart_send_static_text("\"W,#\"       Sensor extension turret not detected. Function deactivated.");
#endif
				
				break;
			case 'I':  
				sprintf(buffer,"i,%d,%d,%d,%d,%d\r\n",cam_mode,cam_width,cam_heigth,cam_zoom,cam_size);
				uart_send_text(buffer);
				break;
			case 'J'://set camera parameter see also cam library
				sscanf(buffer,"J,%d,%d,%d,%d\r",&cam_mode,&cam_width,&cam_heigth,&cam_zoom);
				if(cam_mode==GREY_SCALE_MODE)
					cam_size=cam_width*cam_heigth;
				else
				cam_size=cam_width*cam_heigth*2;
				e_poxxxx_init_cam();
				e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,(ARRAY_HEIGHT-cam_heigth*cam_zoom)/2,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode);
    			e_poxxxx_set_mirror(1,1);
   				e_poxxxx_write_cam_registers();
   				uart_send_static_text("j\r\n");
   				break;
			case 'K':  // calibrate proximity sensors
				uart_send_static_text("k, Starting calibration - Remove any object in sensors range\r\n");
				e_calibrate_ir();
				uart_send_static_text("k, Calibration finished\r\n");
				break;
			case 'L': // set led
				sscanf(buffer,"L,%d,%d\r",&LED_nbr,&LED_action);
				e_set_led(LED_nbr,LED_action);
				uart_send_static_text("l\r\n");
				break;
			case 'M': // read floor sensors (optional)
#ifdef FLOOR_SENSORS
				e_i2cp_enable();
				for (i=0; i<6; i++)	buffer[i] = e_i2cp_read(0xC0,i);
				e_i2cp_disable();
				sprintf(buffer,"m,%d,%d,%d\r\n",
				(unsigned int)buffer[1] | ((unsigned int)buffer[0] << 8),
				(unsigned int)buffer[3] | ((unsigned int)buffer[2] << 8),
				(unsigned int)buffer[5] | ((unsigned int)buffer[4] << 8));
				uart_send_text(buffer);
#else
				uart_send_static_text("m,0,0,0\r\n");
#endif
				break;
			case 'N': // read proximity sensors
				sprintf(buffer,"n,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
				        e_get_calibrated_prox(0),e_get_calibrated_prox(1),e_get_calibrated_prox(2),e_get_calibrated_prox(3),
				        e_get_calibrated_prox(4),e_get_calibrated_prox(5),e_get_calibrated_prox(6),e_get_calibrated_prox(7));
				uart_send_text(buffer);
				break;
			case 'O': // read ambient light sensors
				sprintf(buffer,"o,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
				        e_get_ambient_light(0),e_get_ambient_light(1),e_get_ambient_light(2),e_get_ambient_light(3),
				        e_get_ambient_light(4),e_get_ambient_light(5),e_get_ambient_light(6),e_get_ambient_light(7));
				uart_send_text(buffer);
				break;
			case 'P': // set motor position
				sscanf(buffer,"P,%d,%d\r",&positionl,&positionr);
				e_set_steps_left(positionl);
				e_set_steps_right(positionr);
				uart_send_static_text("p\r\n");
				break;
			case 'Q': // read motor position
				sprintf(buffer,"q,%d,%d\r\n",e_get_steps_left(),e_get_steps_right());
				uart_send_text(buffer);
				break;
			case 'R': // reset
				uart_send_static_text("r\r\n");
				RESET();
				break;
			case 'S': // stop
				e_set_speed_left(0);
				e_set_speed_right(0);
				e_set_led(8,0);
				
				uart_send_static_text("s\r\n");
				break;
			case 'T': // stop
				sscanf(buffer,"T,%d",&sound);
				if(first==0){
					e_init_sound();
					first=1;
				}
				switch(sound)
				{
					case 1: e_play_sound(0,2112);break;
					case 2: e_play_sound(2116,1760);break;
					case 3: e_play_sound(3878,3412);break;
					case 4: e_play_sound(7294,3732);break;
					case 5: e_play_sound(11028,8016);break;
					default:
						e_close_sound();
						first=0;
						break;
				}		
				uart_send_static_text("t\r\n");
				break;
			case 'U':
				sprintf(buffer,"u,%d,%d,%d\r\n",e_get_micro_volume(0),e_get_micro_volume(1),e_get_micro_volume(2));
				uart_send_text(buffer);
				break;
			case 'V': // get version information
				uart_send_static_text("v,Version 2.0.0 January 2008\r\n");
				break;
			case 'W': // read Devantec ultrasonic range sensor or Sharp Ir sensor (optional)
#ifdef LIS_SENSORS_TURRET
			sscanf(buffer,"W,%d\r",&sensext_param[0]);
				if (sensext_pres)				// If the TP_sensors tourret is present
				{
					if(e_sensext_process(sensext_param, sensext_value))
					{
						switch (sensext_param[0])
						{
							case -1:	//i2c SRFxxx
								sprintf(buffer,"w,%u,%u\r\n", sensext_value[0],  sensext_value[1]);
									break;
							case -2:	// i2c cmps03
								sprintf(buffer,"w,%d,%d\r\n",  sensext_value[0],  sensext_value[1]);	
									break;
							default: //analog (sharp,...)
							sprintf(buffer,"w,%d\r\n",  sensext_value[0]);

						}
						uart_send_text(buffer);
					}
					else
					{
						uart_send_static_text("wrong parameter\r\n");
					}
				
				}
				else
				{
					uart_send_static_text("LIS sensors turret not present\r\n");
				}
#endif
				break;

			case 'X':  // Dummy command returning a number of bytes given as parameter
				sscanf(buffer,"X,%d\r",&positionl);
				buffer[positionl+2]='\r';
				buffer[positionl+3]='\n';
				while(positionl>0) {
					buffer[positionl+1]='1';
					positionl--;
				}
				buffer[0]='x';
				buffer[1]=',';
				uart_send_text(buffer);
				break;
			default:
				uart_send_static_text("z,Command not found\r\n");
				break;
			}
		}
	}
}
