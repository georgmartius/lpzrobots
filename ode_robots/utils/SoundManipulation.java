/*
 Converts sensor values into sound.
*/

import javax.sound.sampled.*;
import java.io.*;

public class SoundManipulation extends Thread {

// private PipedInputStream pis;
 private int mode;
 private int numSensors;
 private SourceDataLine sourceLine;
 private InputStream is;

 public SoundManipulation(/*PipedInputStream pis, */int mode, InputStream is) {
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
     float sensorSum=0.0f;
     for(int i=0; i<numSensors; i++) {

      switch(mode) {
       case 1: // discrete
        if(Math.abs(new Float(values[i]).floatValue())>0.7f) {
         for(int j=0; j<data.length; j++) {
          data[j]=(byte)(Math.sin(j/(i+1.0))*127);
         }
         sourceLine.write(data,0,data.length);
        }
        break;
       case 2: // amplitude
        sensorSum+=Math.abs(new Float(values[i]).floatValue());
        if(i==numSensors-1) {
         float sensorAverage=sensorSum/numSensors;
         for(int j=0; j<data.length; j++) {
          data[j]=(byte)(sensorAverage*Math.sin(j/10.0f)*127);
         }
         sourceLine.write(data,0,data.length);
        }
        break;
       case 3: // frequency
        sensorSum+=Math.abs(new Float(values[i]).floatValue());
        if(i==numSensors-1) {
         float sensorAverage=sensorSum/numSensors;
         for(int j=0; j<data.length; j++) {
          data[j]=(byte)(Math.sin(sensorAverage*j/50.0f)*127);
         }
         sourceLine.write(data,0,data.length);
        }
        break;

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