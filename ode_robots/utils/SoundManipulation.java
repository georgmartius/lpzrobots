/*
 Converts sensor values into sound.
*/

import javax.sound.sampled.*;
import java.io.*;

public class SoundManipulation extends Thread {

// private PipedInputStream pis;
 private String mode;
 private int numSensors;
 private SourceDataLine sourceLine;
 private InputStream is;

 public SoundManipulation(/*PipedInputStream pis, */String mode, InputStream is) {
//  this.pis=pis;
  this.mode=mode;
  this.is=is;

  AudioFormat format=new AudioFormat(48000.0f,16,1,true,false);
  DataLine.Info sourceInfo=new DataLine.Info(SourceDataLine.class,format);
  if(!AudioSystem.isLineSupported(sourceInfo)) {
   System.out.println("SoundOutput: Line not supported.");
   System.exit(0);
  }
  try {
   sourceLine=(SourceDataLine)AudioSystem.getLine(sourceInfo);
   sourceLine.open(format);
   sourceLine.start();
  } catch(LineUnavailableException lue) {
   System.out.println(lue.getMessage());
   System.exit(0);
  }
 }

// public void setNumSensors(int numSensors) {
//  this.numSensors=numSensors;
// }

 public void run() {
  byte[] data=new byte[128];
  String input="";

  while(true) {
   int next=0;
   try {
    next=is.read();
   } catch(IOException ioe) {
    System.out.println(ioe.getMessage());
    System.exit(0);
   }
   if(next==-1) break;
   input+=(char)next;
   if((char)next=='\n') {
    if(input.startsWith("#N neuron x[")) {
     numSensors=input.charAt(12)-47;
    } else if(!input.startsWith("#")) {
     String[] values=input.trim().substring(0,numSensors*10).split(" ");
     for(int i=0; i<numSensors; i++) {
      if(Math.abs(new Float(values[i]).floatValue())>0.7f) {
       for(int j=0; j<data.length; j++) {
        data[j]=(byte)(Math.sin(j/(i+1.0))*127);
       }
       sourceLine.write(data,0,data.length);
      }
     }
    }
    input="";
   }
  }
/*
   input="";
   try {
    char next;
    do {
     next=(char)pis.read();
     input+=next;
    } while(next!='\n');
   } catch(IOException ioe) {
    System.out.println(ioe.getMessage());
    System.exit(0);
   }
   String[] values=input.trim().substring(0,numSensors*10).split(" ");
   for(int i=0; i<numSensors; i++) {
    if(Math.abs(new Float(values[i]).floatValue())>0.7f) {
     for(int j=0; j<data.length; j++) {
      data[j]=(byte)(Math.sin(j/(i+1.0))*127);
     }
     sourceLine.write(data,0,data.length);
    }
   }
  }*/
 }
}