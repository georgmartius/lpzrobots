/*
 Converts sensor values into sound.
*/

import javax.sound.midi.*;
import java.io.*;

public class SoundManipulation extends Thread {
 private int mode;
 private float param;
 private int numSensors;
 private InputStream is;
 private Receiver synthRcvr;
 private Synthesizer synth;
 private SoundManGUI gui;
 private String[] instrNames;
 private float[][] oldSensorValues;
 private int oldNote[];
 private int oldVolume[];
 private long time;

 public SoundManipulation(int mode, float param, InputStream is) {
  this.mode=mode;
  this.param=param;
  this.is=is;

  // initialize sound output
  Sequencer seq;
  Transmitter seqTrans;
  try {
   seq=MidiSystem.getSequencer();
   seqTrans=seq.getTransmitter();
   synth=MidiSystem.getSynthesizer();
   synth.open();
   Soundbank sb = synth.getDefaultSoundbank();
   Instrument[] instruments=sb.getInstruments();
   instrNames=new String[instruments.length];
   for(int i=0; i<instrNames.length; i++) instrNames[i]=instruments[i].getName();
   synthRcvr=synth.getReceiver();
   seqTrans.setReceiver(synthRcvr);
  } catch(MidiUnavailableException mue) {
   System.out.println(mue.getMessage());
   System.exit(0);
  }
 }

 public void setGUI(SoundManGUI gui) {
  this.gui=gui;
  this.gui.setInstruments(instrNames);
 }

 public void run() {
  String input="";
  float sensorDiff=0.0f;
  float[] sensorDiffs=null;
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
     // change the number of sensors of the robot
     numSensors=Integer.parseInt(input.substring(input.indexOf("[")+1,input.indexOf("]")))+1;
     oldSensorValues=new float[numSensors][2];
     sensorDiffs=new float[numSensors];
     oldNote=new int[numSensors];
     oldVolume=new int[numSensors];
    } else if(!input.startsWith("#")) {
     // set the number of sensors of the robot
     // and handle sensor values
     gui.setNumSensors(numSensors);
     String[] values=input.trim().substring(0,numSensors*10).split(" ");
     float sensorMax=0.0f;
     param=gui.getParam();
     mode=gui.getMode();
     if(gui.instrumentChanged()) {
      // change instrument for playback
      synth.getChannels()[0].allNotesOff();
      synth.getChannels()[0].programChange(gui.getInstrument());
     }
     updateOldSensorValues();
     if(gui.isPlaybackActive()) {
      for(int i=0; i<numSensors; i++) {
       // smoothing the changes of sensor values
       oldSensorValues[i][1]=Math.abs(new Float(values[i]).floatValue());
       sensorDiff=oldSensorValues[i][1]-oldSensorValues[i][0];
       sensorDiffs[i]+=0.01f*(sensorDiff-sensorDiffs[i]);
       sensorDiff=Math.abs(sensorDiffs[i]);
       gui.setSensorValue(i,sensorDiff*100);
       switch(mode) {
        case 1: // discrete
         if(10.0f*sensorDiff>param) {
          try {
           // If the tone length is exceeded, the old note
           // is turned off and a new one is generated.
           if(Math.abs(System.currentTimeMillis()-time)>gui.getToneLength()) {
            time=System.currentTimeMillis();
            int note=gui.getNote()+27*i/numSensors;
            ShortMessage sm=new ShortMessage();
            sm.setMessage(ShortMessage.NOTE_OFF, 0, oldNote[i], oldVolume[i]);
            synthRcvr.send(sm, -1);
            sm.setMessage(ShortMessage.NOTE_ON, 0, note, 90);
            synthRcvr.send(sm, -1);
            oldNote[i]=note;
            oldVolume[i]=90;
           }
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
          }
         }
         break;
        case 2: // amplitude
         sensorMax=Math.max(sensorMax,sensorDiff);
         if(i==numSensors-1) {
          try {
           // If the tone length is exceeded, the old note
           // is turned off and a new one is generated.
           if(Math.abs(System.currentTimeMillis()-time)>gui.getToneLength()) {
            time=System.currentTimeMillis();
            int note=gui.getNote()+27;
            int volume=(int)Math.min(127,param*sensorMax*127000.0f);
            ShortMessage sm=new ShortMessage();
            sm.setMessage(ShortMessage.NOTE_OFF, 0, oldNote[i], oldVolume[i]);
            synthRcvr.send(sm, -1);
            sm.setMessage(ShortMessage.NOTE_ON, 0, note, volume);
            synthRcvr.send(sm, -1);
            oldNote[i]=note;
            oldVolume[i]=volume;
           }
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
          }
          sensorMax=0.0f;
         }
         break;
        case 3: // frequency
         sensorMax=Math.max(sensorMax,sensorDiff);
         if(i==numSensors-1) {
          try {
           // If the tone length is exceeded, the old note
           // is turned off and a new one is generated.
           if(Math.abs(System.currentTimeMillis()-time)>gui.getToneLength()) {
            time=System.currentTimeMillis();
            int note=(int)Math.min(127,param*sensorMax*12700.0f);
            gui.setNote(note);
            ShortMessage sm=new ShortMessage();
            sm.setMessage(ShortMessage.NOTE_OFF, 0, oldNote[i], oldVolume[i]);
            synthRcvr.send(sm, -1);
            sm.setMessage(ShortMessage.NOTE_ON, 0, note, 90);
            synthRcvr.send(sm, -1);
            oldNote[i]=note;
            oldVolume[i]=90;
           }
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
           }
          sensorMax=0.0f;
         }
         break;
        case 4: // master mode
         if(sensorDiff>0.0f) { 
          try {
           // If the tone length is exceeded, the old note
           // is turned off and a new one is generated.
           if(Math.abs(System.currentTimeMillis()-time)>gui.getToneLength()) {
            time=System.currentTimeMillis();
            int volume=(int)Math.min(100,sensorDiff*12500.0f);
            int note=volume+27*i/(numSensors-1);
            ShortMessage sm=new ShortMessage();
            sm.setMessage(ShortMessage.NOTE_OFF, 0, oldNote[i], oldVolume[i]);
            synthRcvr.send(sm, -1);
            sm.setMessage(ShortMessage.NOTE_ON, 0, note, volume);
            synthRcvr.send(sm, -1);
            oldNote[i]=note;
            oldVolume[i]=volume;
           }
          } catch(InvalidMidiDataException imde) {
           System.out.println(imde.getMessage());
           System.exit(0);
          }
         }
        break;
       }
      }
     } else {
      // mute playback
      synth.getChannels()[0].allNotesOff();
     }

    }
    input="";
   }
  }
 }

 private void updateOldSensorValues() {
  for(int i=0; i<oldSensorValues.length; i++) oldSensorValues[i][0]=oldSensorValues[i][1];
 }

}