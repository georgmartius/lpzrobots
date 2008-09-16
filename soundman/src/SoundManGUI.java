import javax.swing.*;
import javax.swing.event.*;
import java.awt.event.*;
import java.text.*;

public class SoundManGUI extends JFrame {
 private float param;
 private int mode;
 private int numSensors;
 private int curInstrument;
 private boolean isPlaybackActive;
 private boolean smoothing;
 private int note;
 private int toneLength;
 private JComboBox instruCB;
 private JLabel l4;
 private JSlider noteSlide;
 private JSlider toneLSlide;
 private JSlider[] sensorSlide;
 private JCheckBox[] sensorCheck;
 private JSlider[] customSlider;

 private static final long serialVersionUID = 1L;

 public SoundManGUI(float p, int m) {
  param=p;
  mode=m;
  toneLength=150;
  if(mode>=3) noteSlide.setEnabled(false);
  isPlaybackActive=true;
  smoothing=false;
  getContentPane().setLayout(null);
  setTitle("SoundMan");
  setResizable(false);
  setSize(350,335);


  ////////// CUSTOM SLIDERS //////////
  // Slider values can be queried in SoundManipulation.java
  // via gui.getCustomSliderValue(i) with i from 0-2 being
  // the number of the slider from left to right.
  customSlider=new JSlider[3];
  for(int i=0; i<3; i++) {
   customSlider[i]=new JSlider(JSlider.VERTICAL,0, 100, 50);
   customSlider[i].setBounds(250+i*25, 10, 15, 250);
   getContentPane().add(customSlider[i]);
   getContentPane().repaint();
  }

  ////////// CUSTOM BUTTON //////////
  JButton customButton=new JButton("Custom");
  customButton.setBounds(250, 275, 75, 20);
  getContentPane().add(customButton);
  customButton.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     // What's to do when pushing the button?
    }
   }
  );

  ////////// param //////////
  JLabel l1=new JLabel("Parameter:");
  l1.setBounds(10, 70, 80, 20);
  getContentPane().add(l1);

  DecimalFormat df=new DecimalFormat();
  df.setMaximumIntegerDigits(1);
  df.setMinimumIntegerDigits(1);
  df.setMinimumFractionDigits(1);
  df.setDecimalSeparatorAlwaysShown(true);
  final JFormattedTextField paramFTF=new JFormattedTextField(df);
  paramFTF.setText(NumberFormat.getInstance().format(param));
  paramFTF.setBounds(85, 70, 50, 20);
  paramFTF.setToolTipText("The motor value change needed to play sound.");
  getContentPane().add(paramFTF);
  paramFTF.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     float newParam=new Float(paramFTF.getText().replace(',','.')).floatValue();
     if(newParam<0.0f) {
      newParam=Math.abs(newParam);
      paramFTF.setText(NumberFormat.getInstance().format(newParam));
     }
     param=newParam;
    }
   }
  );

  final JButton paramB=new JButton("Accept");
  paramB.setBounds(150, 70, 75, 20);
  getContentPane().add(paramB);
  paramB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     float newParam=new Float(paramFTF.getText().replace(',','.')).floatValue();
     if(newParam<0.0f) {
      newParam=Math.abs(newParam);
      paramFTF.setText(NumberFormat.getInstance().format(newParam));
     }
     param=newParam;
    }
   }
  );

  ////////// mode //////////
  JRadioButton discRB=new JRadioButton("Discrete");
  discRB.setBounds(5, 10, 75, 20);
  getContentPane().add(discRB);
  discRB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     mode=1;
     noteSlide.setEnabled(true);
     paramFTF.setEnabled(true);
     paramFTF.setToolTipText("The motor value change needed to play sound.");
     paramB.setEnabled(true);
    }
   }
  );

  JRadioButton amplRB=new JRadioButton("Amplitude");
  amplRB.setBounds(80, 10, 80, 20);
  getContentPane().add(amplRB);
  amplRB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     mode=2;
     noteSlide.setEnabled(true);
     paramFTF.setEnabled(true);
     paramFTF.setToolTipText("Boosts the volume.");
     paramB.setEnabled(true);
    }
   }
  );

  JRadioButton freqRB=new JRadioButton("Frequency");
  freqRB.setBounds(160, 10, 80, 20);
  getContentPane().add(freqRB);
  freqRB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     mode=3;
     noteSlide.setEnabled(false);
     paramFTF.setEnabled(true);
     paramFTF.setToolTipText("Amplifies changing of notes.");
     paramB.setEnabled(true);
    }
   }
  );

  JRadioButton masterRB=new JRadioButton("Master mode (all three combined)");
  masterRB.setBounds(5, 40, 240, 20);
  getContentPane().add(masterRB);
  masterRB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     mode=4;
     noteSlide.setEnabled(false);
     paramFTF.setEnabled(false);
     paramFTF.setToolTipText("No usage in master mode.");
     paramB.setEnabled(false);
    }
   }
  );

  ButtonGroup modeBG=new ButtonGroup();
  modeBG.add(discRB);
  modeBG.add(amplRB);
  modeBG.add(freqRB);
  modeBG.add(masterRB);

  if(mode==1) discRB.setSelected(true);
  else if(mode==2) amplRB.setSelected(true);
  else if(mode==3) freqRB.setSelected(true);
  else masterRB.setSelected(true);

  ////////// note //////////
  JLabel l2=new JLabel("Note:");
  l2.setBounds(10, 95, 35, 20);
  getContentPane().add(l2);

  noteSlide=new JSlider(0, 100, 50);
  noteSlide.setBounds(45, 95, 190, 20);
  getContentPane().add(noteSlide);
  note=noteSlide.getValue();
  noteSlide.addChangeListener(
   new ChangeListener() {
    public void stateChanged(ChangeEvent e) {
     note=noteSlide.getValue();
    }
   }
  );

  ////////// timestep //////////
  JLabel l6=new JLabel("Timestep in ms:");
  l6.setBounds(10, 120, 110, 20);
  getContentPane().add(l6);

  toneLSlide=new JSlider(50, 1000, 150);
  toneLSlide.setBounds(115, 120, 125, 35);
  toneLSlide.setMajorTickSpacing(950);
  toneLSlide.setPaintLabels(true);
  getContentPane().add(toneLSlide);
  toneLSlide.addChangeListener(
   new ChangeListener() {
    public void stateChanged(ChangeEvent e) {
     toneLength=toneLSlide.getValue();
    }
   }
  );

  ////////// smoothing //////////
  JCheckBox smooth=new JCheckBox("Smoothing of sound over time");
  smooth.setSelected(false);
  smooth.setBounds(10, 150, 230, 20);
  getContentPane().add(smooth);
  smooth.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     smoothing=!smoothing;
    }
   }
  );

  ////////// instruments //////////
  JLabel l3=new JLabel("Instrument:");
  l3.setBounds(10, 170, 80, 20);
  getContentPane().add(l3);

  instruCB=new JComboBox();
  instruCB.setBounds(90, 170, 140, 20);
  getContentPane().add(instruCB);

  ////////// sensors //////////
  l4=new JLabel("Number of sensors: -");
  l4.setBounds(10, 190, 200, 20);
  getContentPane().add(l4);

  ////////// playback //////////
  JLabel l5=new JLabel("Playback:");
  l5.setBounds(10, 275, 200, 20);
  getContentPane().add(l5);

  final JButton playbackB=new JButton("Mute");
  playbackB.setBounds(90, 275, 140, 20);
  getContentPane().add(playbackB);
  playbackB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     isPlaybackActive=!isPlaybackActive;
     if(isPlaybackActive) playbackB.setText("Mute");
     else playbackB.setText("Unmute");
    }
   }
  );
 }

 // CUSTOM SLIDER GETTER
 public int getCustomSliderValue(int i) {
  return customSlider[i].getValue();
 }

 public int getToneLength() {
  return toneLength;
 }

 public int getNote() {
  return note;
 }

 public void setNote(int newNote) {
  note=newNote;
  noteSlide.setValue(note);
 }

 public boolean isPlaybackActive() {
  return isPlaybackActive;
 }

 public void setNumSensors(int n) {
  if(numSensors==n || n==0) return;
  numSensors=n;
  l4.setText("Number of sensors: "+n);

  sensorSlide=new JSlider[n];
  for(int i=0; i<n; i++) {
   sensorSlide[i]=new JSlider(JSlider.VERTICAL,0, 100, 0);
   sensorSlide[i].setEnabled(false);
   sensorSlide[i].setBounds((i+1)*230/(n+1), 205, 10, 50);
   getContentPane().add(sensorSlide[i]);
   getContentPane().repaint();
  }

  sensorCheck=new JCheckBox[n];
  for(int i=0; i<n; i++) {
   sensorCheck[i]=new JCheckBox("",true);
   sensorCheck[i].setBounds((i+1)*230/(n+1)-5, 256, 17, 13);
   getContentPane().add(sensorCheck[i]);
   getContentPane().repaint();
  }
 }

 public boolean isSmoothingEnabled() {
  return smoothing;
 }

 public boolean isSensorChecked(int i) {
  return sensorCheck[i].isSelected();
 }

 public void setSensorValue(int num, float val) {
  sensorSlide[num].setValue((int)(val*100.0));
 }

 public float getParam() {
  return param;
 }

 public int getMode() {
  return mode;
 }

 public boolean instrumentChanged() {
  if(curInstrument!=instruCB.getSelectedIndex()) {
   curInstrument=instruCB.getSelectedIndex();
   return true;
  }
  return false;
 }

 public int getInstrument() {
  return instruCB.getSelectedIndex();
 }

 public void setInstruments(String[] s) {
  for(int i=0; i<s.length; i++) instruCB.addItem(s[i]);
 }

}
