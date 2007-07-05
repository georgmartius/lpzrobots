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
  long time=0;
  float[] sensorDiffs=null;
  int next=0;
  int index=-1;
  boolean initIsOver=false;
  String[] values=null;
  while(true) {
    try {
     next=is.read(); // read next input as byte
    } catch(IOException ioe) {
     System.out.println(ioe.getMessage());
     System.exit(0);
    }
    if(next==-1) break; // end of input stream reached
    // parse input
    if(index<numSensors) input+=(char)next;
    if(initIsOver && next==' ' && index<numSensors) {
     if(index>=0) {
      values[index]=input;
      input="";
     }
     index++;
    }
    if(next=='\n') {
     index=-1;
     if(input.startsWith("#N neuron x[")) {
      // adjust the number of sensors of the robot
      numSensors=Integer.parseInt(input.substring(input.indexOf("[")+1,input.indexOf("]")))+1;
      oldSensorValues=new float[numSensors][2];
      sensorDiffs=new float[numSensors];
      oldNote=new int[numSensors];
      oldVolume=new int[numSensors];
      values=new String[numSensors];
      for(int i=0; i<numSensors; i++) values[i]="0.0";
     } else if(Math.abs(System.currentTimeMillis()-time)>gui.getToneLength()) {
      // new data received and timestep is over
      time=System.currentTimeMillis();
      if(!input.startsWith("#")) {
       initIsOver=true;
       // set the number of sensors of the robot
       // and handle sensor values
       gui.setNumSensors(numSensors);
       float sensorMax=0.0f;
       param=gui.getParam();
       mode=gui.getMode();
       if(gui.instrumentChanged()) {
        // change instrument for playback
        clearOldSensorValues();
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
         if(gui.isSensorChecked(i)) {
          switch(mode) {
           case 1: // discrete
            if(10.0f*sensorDiff>param) {
             int note=gui.getNote()+27*i/numSensors;
             play(i,note,90);
            }
            break;
           case 2: // amplitude
            sensorMax=Math.max(sensorMax,sensorDiff);
            if(i==numSensors-1) {
             int note=gui.getNote()+27;
             int volume=(int)Math.min(127,param*sensorMax*127000.0f);
             play(i,note,volume);
             sensorMax=0.0f;
            }
            break;
           case 3: // frequency
            sensorMax=Math.max(sensorMax,sensorDiff);
            if(i==numSensors-1) {
             int note=(int)Math.min(127,param*sensorMax*12700.0f);
             play(i,note,90);
             gui.setNote(note);
             sensorMax=0.0f;
            }
            break;
           case 4: // master mode
            if(sensorDiff>0.0f) { 
             int volume=(int)Math.min(100,sensorDiff*12500.0f);
             int note=volume+27*i/(numSensors-1);
             play(i,note,volume);
            }
           break;
          }
         }
        }
       } else {
        // mute playback
        synth.getChannels()[0].allNotesOff();
       }
      }
     }
     input="";
   }
  }
 }

 private void play(int i, int note, int volume) {
  try {
   ShortMessage sm=new ShortMessage();
   sm.setMessage(ShortMessage.NOTE_OFF, 0, oldNote[i], oldVolume[i]);
   synthRcvr.send(sm, -1);
   sm.setMessage(ShortMessage.NOTE_ON, 0, note, volume);
   synthRcvr.send(sm, -1);
   oldNote[i]=note;
   oldVolume[i]=volume;
  } catch(InvalidMidiDataException imde) {
   System.out.println(imde.getMessage());
   System.exit(0);
  }
 }

 private void clearOldSensorValues() {
  for(int i=0; i<oldSensorValues.length; i++) {
   oldSensorValues[i][0]=0.0f;
   oldSensorValues[i][1]=0.0f;
  }
 }

 private void updateOldSensorValues() {
  for(int i=0; i<oldSensorValues.length; i++) oldSensorValues[i][0]=oldSensorValues[i][1];
 }

}