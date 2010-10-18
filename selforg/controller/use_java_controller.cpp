
#ifndef WIN32

#include <stdio.h>
#include <cmath>
#include "use_java_controller.h"
#include <unistd.h>
#include <stdlib.h>





using namespace std;

use_java_controller::~use_java_controller()
{
	
	closeJavaController();
	
	//schliessen der Verbindung und wieder-Freigabe der Ports
	close(client_controller);
	printf("\nJava-Controller wurde geschlossen - Controller-Socket geschlossen");
	close(client_internalParams);
	printf("\nJava-Controller wurde geschlossen - Guilogger-Socket geschlossen");
	printf("\nSimulation wird beendet!\n");
	close(server_controller);
	close(server_internalParams);
	
}

/**
* Konstruktor: Initialisiert die Serververbindungen zu Java
* @param port_controller Portnummer für die Kommunikation des Controllers
* @param port_internalParams Portnummer für die Kommunikation des Guiloggers
*/
use_java_controller::use_java_controller ( const char* port_controller, const char* port_internalParams, const char* name )
  : AbstractController ( "Javacontroller", "$Id: use_java_controller.cpp,Rocco Gwizdziel" ),
    name(name)
{
	addController();
	char* port_iP;
	//wenn port_internalParams nicht gesetz dann port_internalParams = port_controller + 1 !
	if ( port_internalParams == NULL )
	{
	  port_iP = ( char* )  malloc ( 64 * sizeof(char) );
	  
	  sprintf (port_iP,"%d", ( atoi ( port_controller ) +1 ) );
	  printf ( "port: %s", port_iP );
	}else{
	  port_iP = strdup(port_internalParams);
	}

	//Zeitschritt initialisieren t=0
 	t=0;

    //sockets für den ControlServer und GuiloggerServer 
	server_internalParams = socket ( AF_INET, SOCK_STREAM, 0 );
	server_controller     = socket ( AF_INET, SOCK_STREAM, 0 );

	//wenn Sockets nicht angelegt werden können
	if ( server_internalParams == -1 || server_controller == -1 )
	{
		perror ( "socket() failed" );
	}

	//Die Socketadress-Strukturen (IP und Port)
	server_controller_addr.sin_family      = AF_INET;
	server_controller_addr.sin_addr.s_addr = htonl ( INADDR_ANY );
	server_controller_addr.sin_port        = htons ( ( unsigned short int ) atol ( port_controller ) );


	server_internalParams_addr.sin_family      = AF_INET;
	server_internalParams_addr.sin_addr.s_addr = htonl ( INADDR_ANY );
	server_internalParams_addr.sin_port        = htons ( ( unsigned short int ) atol ( port_iP ) );


	

	//Die Sockets an die Socketadress-Strukturen binden
	if ( bind ( server_controller, ( struct sockaddr* ) &server_controller_addr, sizeof ( server_controller_addr ) ) == -1 )
		{perror ( "bind() failed" ); exit ( 1 );}
	if ( bind ( server_internalParams, ( struct sockaddr* ) &server_internalParams_addr, sizeof ( server_internalParams_addr ) ) == -1 )
		{perror ( "bind() failed" ); exit ( 1 );}

	//Sockets dem System übergeben
	if ( listen ( server_controller, 1 ) == -1 )
		{perror ( "listen() failed" );}
	if ( listen ( server_internalParams, 1 ) == -1 )
		{perror ( "listen() failed" );}

	

	printf ( "\n****\nInitialisierung gestartet...\n" );
	printf ( "\nbitte den Java-Controller starten !!\nStatus: wait ...\n\n\n" );

	//ControlServer wartet auf den ControlClient (Java)
	client_controller_size = sizeof ( client_controller_addr );
	client_controller = accept ( server_controller, ( struct sockaddr* ) &client_controller_addr, &client_controller_size );

	if ( client_controller == -1 ) {perror ( "accept() failed" );}
	printf ( "ControllerServer gestartet (%s: %s)\nStatus: listen ...\n\n",inet_ntoa ( client_controller_addr.sin_addr ) ,port_controller );


	//GuiloggerServer wartet auf den GuiloggerClient (Java)
	client_internalParams_size = sizeof ( client_internalParams_addr );
	client_internalParams = accept ( server_internalParams, ( struct sockaddr* ) &client_internalParams_addr, &client_internalParams_size );
	if ( client_internalParams == -1 ) {perror ( "accept() failed" );}
	printf ( "GuiloggerServer gestartet (%s: %s)\nStatus: listen ...\n\n",inet_ntoa ( client_controller_addr.sin_addr ) ,port_internalParams );


	this->number_controlled=-1;
};



/**
* Methode verschickt Protokolle nach Java
*/
void use_java_controller::sendToJava ( const char* message, bool abbruch, const char* meldung )
{
   
    int bytes = send (client_controller, message, strlen ( message ),MSG_DONTWAIT);
	if ( bytes == -1 )
	{
          printf ("%s", meldung );
          if ( abbruch ) exit ( 1 );
          
	}
   

}


/**
* Methode verschickt Protokolle nach Java
*/
void use_java_controller::closeJavaController()
{
   
    send (client_controller,"X\n", strlen ( "X\n" ),0 );
	send (client_internalParams, "X\n", strlen ( "X\n"),0 );

}



/** 
* initialisation of the controller with the given sensor/ motornumber
* Must be called before use.
* 
*/
void use_java_controller::init ( int sensornumber, int motornumber, RandGen* randgen)
{

  // Todo: send also random seed to java controller

	//initialisierung nach java senden
	char *temp_init;
	temp_init = ( char* )  malloc ( BUFFER_SIZE );
	sprintf ( temp_init,"I#0#%d#%d#%s\n",sensornumber,motornumber,name);

	//verschicken

	sendToJava ( temp_init ,false );
 
	free ( temp_init );


	//empfangen der Initialisierungs Daten von Java
	//warten bis wirklich empfangen
	while ( 1 )
	{
	
		bool ok_I = false;
		bool ok_G = false;

		char recvData_controller[BUFFER_SIZE];
 		
        
		fcntl ( client_controller,F_SETFL,O_NONBLOCK );
		int bytes = recv ( client_controller, recvData_controller, sizeof ( recvData_controller ) - 1, 0 );
	
		
		recvData_controller[bytes] = '\0';

		
		if ( bytes > 0 && strlen ( recvData_controller ) != 0 && recvData_controller[0] == 'I' )
		{
			//speichert die configParams
			char values[1+ (MAX_CONFIG_PARAM+2) ][100];
			
            //splitten (anhand von #)
			int m = 0;
			int counter=2;
			int counter_help = 0;

			while ( 1 )
			{

				if ( recvData_controller[counter]=='#' )
					{ values[m][counter_help]='\0'; m++; counter_help=0; counter++;}
				values[m][counter_help++]= recvData_controller[counter];
				if ( recvData_controller[counter++]=='\0' ) break;
			}


			anz_config_param = atoi ( values[0] );

			config_param_list.clear();
			//configParam list mit den werten füllen
			for ( int p=0; p < anz_config_param*2 ; p+=2 )
			{
				config_param_list += pair<paramkey, paramval> ( string ( values[p+1] ), atof ( values[p+2] ) );
			}
		
			ok_I = true;
		}
		



		//internal params empfangen auf anderen Port 
		char recvData_internalParams[BUFFER_SIZE];
		
        //methode recv() blockiert nicht, wenn keine daten empfangen werden
        fcntl ( client_internalParams,F_SETFL,O_NONBLOCK );
		int bytes1 = recv ( client_internalParams, recvData_internalParams, sizeof ( recvData_internalParams ) - 1, 0 );
		
       
		
		recvData_internalParams[bytes1] = '\0';


		if (bytes1 > 0  && strlen ( recvData_internalParams ) != 0 && recvData_internalParams[0] == 'G' && recvData_internalParams[1] == 'I' )
		{
			//speichert die interalParams
			char values_param[MAX_INTERNAL_PARAM][100];

			//splitten (anhand von #)
			int m = 0;
			int counter=3;
			int counter_help = 0;

			
	while ( 1 )
			{

				if ( recvData_internalParams[counter]=='#' )
				{ values_param[m][counter_help]='\0'; m++; counter_help=0; counter++;}
				values_param[m][counter_help++]= recvData_internalParams[counter];
				if ( recvData_internalParams[counter++]=='\0' ) break;
			}


		
			
            anz_internal_param = 0;
			 
			for ( int ip=0; ; ip ++ )
			{
				
				if(*values_param[ip]== '$') break;
				printf ( "%s\n",values_param[ip] );
				internal_keylist += string ( values_param[ip] );
				anz_internal_param++;
			}
			
			
			printf ( "Anzahl der InternalParameter: %d\n",anz_internal_param );

			ok_G = true;
		}
		else{ok_G = true; anz_internal_param = 0;}


		if ( ok_I && ok_G ){isFirst = true; break;}
		
		
		sleep(1);
	}
	//ende empfangen der Initialisierung
	printf ( "\n\nInitialisierung erfolgreich abgeschlossen!!\n****\n\n" );

	serverOK = true;
    can_send = true;
    
	isClosed = false;
	server_controller_isClosed = false;
	server_guilogger_isClosed  = false;

   

	motor_values_alt = ( double* ) malloc ( motornumber*sizeof ( double ) );
	for ( int i=0;  i < motornumber; i++ ) {motor_values_alt[i] = 0;}

	internal_vallist_alt = std::list<iparamval>();
 
};


/** performs one step (includes learning).
    Calculates motor commands from sensor inputs.
    @param sensors inputs scaled to [-1,1]
    @param sensornumber length of the sensor array
    @param motors outputs. MUST have enough space for motor values!
    @param motornumber length of the provided motor array
*/
void use_java_controller::step ( const sensor* sensors, int sensornumber,
                             motor* motors, int motornumber )
{
	stepNoLearning ( sensors, sensornumber, motors, motornumber );
};

/** performs one step without learning.
    @see step
*/
void use_java_controller::stepNoLearning ( const sensor* sensors, int number_sensors,
                                       motor* motors, int number_motors )
{


	if ( serverOK )
	{

		char *temp;
	    char *temp1;

	    temp =  ( char* )  malloc ( BUFFER_SIZE );
	    temp1 = ( char* )  malloc ( BUFFER_SIZE );


		sprintf ( temp,"N#%d#%d#%d#",t++,number_sensors,number_motors );
		strcpy ( temp1,temp );

		//sammeln der zusendenden daten
		//sensoren
		for ( int i= 0; i < number_sensors-1; i++ )
		{
			sprintf ( temp,"%f&",sensors[i] );
			strcat ( temp1,temp );
		}
		sprintf ( temp,"%f#",sensors[number_sensors-1] );
		strcat ( temp1,temp );

		//motoren
		for ( int i= 0; i < number_motors-1; i++ )
		{
			sprintf ( temp,"%f&",motors[i] );
			strcat ( temp1,temp );
		}
		sprintf ( temp,"%f",motors[number_motors-1] );
		strcat ( temp1,temp );

		sprintf ( temp,"\n" );
		strcat ( temp1,temp );

        //verschicken
		if(can_send) sendToJava ( temp1,true);
		free ( temp );
		free ( temp1 );
       
     
		//daten empfangen********************************************************************
		char recvData_controller[BUFFER_SIZE];
   		//recv() blockiert nicht
    	fcntl (client_controller,F_SETFL,O_NONBLOCK );
		int bytes = recv ( client_controller, recvData_controller, sizeof ( recvData_controller ) - 1, 0 );
		//if ( bytes == -1 ) { printf ( "Fehler beim Empfangen der Daten vom Java-Controller(stepNoLearning)\nProgramm beendet!\n" );exit ( 1 );}
		recvData_controller[bytes] = '\0';
		//printf("hier-%d-%sENDE\n",bytes,recvData_controller);

		if ( bytes > 0 && strlen ( recvData_controller ) != 0 && recvData_controller[0] == 'X' ){
			printf("\n%s - Controller-Socket geschlossen",name);
			//schliessen der Verbindung und wieder-Freigabe der Ports
			close(client_controller);
			server_controller_isClosed = true;
		}

		//if(bytes == 0) printf("0\n");
		
		
		if ( bytes > 0 && strlen ( recvData_controller ) != 0 && recvData_controller[0] == 'N')
		{
			can_send = true;
			
			//speichert motorwerte und im letzten feld anzahl von configParam
			char values[number_motors][100];
			//splitten (anhand von #)
			int m = 0;
			int counter=2;
			int counter_help = 0;

			while ( 1 )
			{
				if ( recvData_controller[counter]=='#' )
					{ values[m][counter_help]='\0'; m++; counter_help=0; counter++;}
				values[m][counter_help++]= recvData_controller[counter];
				if ( recvData_controller[counter++]=='\0' ) break;
			}

			//übergeben der Motorwerte an den Robot
			for ( int i=0;  i < number_motors; i++ )
			{
				//nach float casten
				double floatval;
				floatval = atof ( values[i] );
				motors[i] = floatval;
			}

			motor_values_alt = motors;
			isFirst = false;

		}
		else
		//wenn fehler beim empfang, alte Motorwerte benutzen
		//java-Controller hat noch keine neuen werte gesendet
		{
			//printf("%d",bytes);
			if(!isFirst) can_send = false; 
			motors = motor_values_alt;
		}




	
    
	//sammel der daten für den guilogger
	internal_vallist =  std::list<iparamval>();

	char recvData_internalParams[BUFFER_SIZE];
	//recv() blockiert nicht
    fcntl ( client_internalParams,F_SETFL,O_NONBLOCK );
	int bytes1 = recv ( client_internalParams, recvData_internalParams, sizeof ( recvData_internalParams )-1 , 0 );
	recvData_internalParams[bytes1] = '\0';

	if ( bytes1 > 0 && strlen ( recvData_internalParams ) != 0 && recvData_internalParams[0] == 'X'){
		printf("\n%s - Guilogger-Socket geschlossen",name);
		//schliessen der Verbindung und wieder-Freigabe der Ports
		close(client_internalParams);
		server_guilogger_isClosed = true;
	} 

	if ( bytes1 > 0 && strlen ( recvData_internalParams ) != 0 && recvData_internalParams[0] == 'G' && recvData_internalParams[1] == 'D' )
	{
		//speichert die configParams
		char values_param[MAX_INTERNAL_PARAM][20];

		//splitten (anhand von #)
		int m = 0;
		int counter=3;
		int counter_help = 0;

		while ( 1 )
		{

			if ( recvData_internalParams[counter]=='#' )
				{ values_param[m][counter_help]='\0'; m++; counter_help=0; counter++;}
			values_param[m][counter_help++]= recvData_internalParams[counter];
			if ( recvData_internalParams[counter++]=='\0' ) break;
		}


		for ( int ip=0; ip <= anz_internal_param; ip ++ )
		{
			internal_vallist += atof ( values_param[ip] );
		}

		internal_vallist_alt = internal_vallist; 

	}
	//wenn keine internalParameter angegeben wurden
    else
	{
		internal_vallist =  internal_vallist_alt;
	}

	if(server_controller_isClosed && server_guilogger_isClosed && !isClosed){
		 printf("\nSimulation von %s ist beendet!\n",name);
         isClosed = true;
		 anzahl_Java_controller --;
		 if(anzahl_Java_controller == 0)exit(0);
		} 

	}

	
}



/**
* executed when a parameter is asked from the console (value should be sent from java-controller and returned)
*/
Configurable::paramval use_java_controller::getParam ( const paramkey& key ) const
{
	paramlist::const_iterator i;
	for ( i=config_param_list.begin(); i != config_param_list.end(); ++i )
	{

		if ( ( ( ( pair<paramkey, paramval> ) *i ).first ) == key )
		{
			return ( ( ( pair<paramkey, paramval> ) *i ).second );
		}
	}

	return AbstractController::getParam ( key ) ;

}

/**
* executed when a parameter is set on the console (new value should be sent to java-controller)
*/
bool use_java_controller::setParam ( const paramkey& key, paramval val )
{
	//wenn server läuft
	if ( serverOK )
	{
		//im der paramliste ändern
		//parameter suchen und val ändern
		paramlist::const_iterator i;
		for ( i=config_param_list.begin(); i != config_param_list.end(); ++i )
		{
			if ( ( ( ( pair<paramkey, paramval> ) *i ).first ) == key )
			{
				config_param_list.remove ( *i );
				config_param_list += pair<paramkey, paramval> ( key, val );
				char *temp;
				temp = ( char* )  malloc ( BUFFER_SIZE );
				sprintf ( temp,"C#%s#%lf\n",key.data(),val );
				//verschicken
				sendToJava ( temp ,false );
				free ( temp );

				return true;
			}
		}

	}
	//wenn nicht in der ConfigParamlist
	return AbstractController::setParam ( key, val );


}

/**
* executed once from simulation for asking the parameter names (parameter names should be sent from java-controller and returned)
*/
Configurable::paramlist use_java_controller::getParamList() const
{
	return config_param_list;
}


//guilogger
list<Inspectable::iparamkey> use_java_controller::getInternalParamNames() const
{
	return internal_keylist;
}


list<Inspectable::iparamval> use_java_controller::getInternalParams() const
{
	return internal_vallist;
}


int use_java_controller::anzahl_Java_controller=0;


#endif //win32
