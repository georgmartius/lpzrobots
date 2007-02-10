/*
 Converts sensor values into sound.
*/

//import javax.sound.sampled.*;
import java.io.*;

public class Sound {
 public static void main(String[] args) {
//  try {
//   PipedOutputStream pos=new PipedOutputStream();
   SoundManipulation sm=new SoundManipulation(/*new PipedInputStream(pos), */"",System.in);
sm.start();
try{
 sm.join();
} catch(InterruptedException ie) {}
//   String input="";
//   byte[] tmp=new byte[1];
//   byte[] data=new byte[1024];
//   int numBytesRead=0;
/*boolean started=false;
   while(true) {
    numBytesRead=System.in.read(tmp,0,1);
    if(numBytesRead==-1) break;
    input+=(char)tmp[0];
    if((char)tmp[0]=='\n') {
     if(input.startsWith("#N neuron x[")) {
      sm.setNumSensors(input.charAt(12)-47);
     } else if(!input.startsWith("#")) {
      for(int i=0; i<input.length(); i++) {
       pos.write(input.charAt(i));
      }
//      pos.flush(); //?
if(!started) {sm.start(); started=true;}
     }
     input="";
    }
   }*/
/*  } catch(IOException ioe) {
   System.out.println(ioe.getMessage());
   System.exit(0);
  }*/
  System.exit(0);
 }
}

/*
public class Sound {
 public static void main(String[] args) {
  AudioFormat format=new AudioFormat(48000.0f,16,1,true,false);
  DataLine.Info sourceInfo=new DataLine.Info(SourceDataLine.class,format);
  if(!AudioSystem.isLineSupported(sourceInfo)) {System.out.println("SoundOutput: Line not supported."); System.exit(0);}
  SourceDataLine sourceLine=null;
  try {
   sourceLine=(SourceDataLine)AudioSystem.getLine(sourceInfo);
   sourceLine.open(format);
  } catch(LineUnavailableException lue) {System.out.println(lue.getMessage()); System.exit(0);}
  sourceLine.start();

  int numSensors=0;
  String input="";
  byte[] tmp=new byte[1];
  byte[] data=new byte[1024];
  int dataIndex=0;
  int numBytesRead=0;
  float x=0.0f, average=0.0f;
  while(true) {
   try {numBytesRead=System.in.read(tmp,0,1);}
   catch(IOException ioe) {System.out.println(ioe.getMessage()); System.exit(0);}
   if(numBytesRead==-1) break;
   input+=(char)tmp[0];
   if((char)tmp[0]=='\n') {
    if(input.startsWith("#N neuron x[")) numSensors=input.charAt(12)-47;
    else if(!input.startsWith("#")) {
     String[] values=input.trim().substring(0,numSensors*10).split(" ");
     for(int i=0; i<numSensors; i++) average+=new Float(values[i]).floatValue();
     average/=numSensors;
    }
    input="";
   }

   data[dataIndex]=(byte)((Math.sin(x)+average)*255);
   x+=0.0001f;
   dataIndex++;
   if(dataIndex==1024) {
    sourceLine.write(data,0,1024);
    dataIndex=0;
   }
  }
  System.exit(0);

/*
  byte[] data=new byte[1024];
  do {
   try {
    numBytesRead=System.in.read(data,0,1024);
   } catch(IOException ioe) {System.out.println("4"); System.exit(0);}
   sourceLine.write(data,0,1024);
  } while(numBytesRead!=-1);
  System.exit(0);
/
 }
}*/