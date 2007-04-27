import javax.swing.*;
import java.awt.event.*;
import java.text.*;

public class SoundManGUI extends JFrame {
 private float param;
 private int mode;
 private int numSensors;
 private int curInstrument;
 private boolean isPlaybackActive;
 private JComboBox instruCB;
 private JLabel l3;

 public SoundManGUI(float p, int m) {
  param=p;
  mode=m;
  isPlaybackActive=true;
  getContentPane().setLayout(null);
  setTitle("SoundMan");
  setResizable(false);
  setSize(250,185);

  ////////// mode //////////
  JRadioButton discRB=new JRadioButton("Discrete");
  discRB.setBounds(5, 10, 75, 20);
  getContentPane().add(discRB);
  discRB.addActionListener(
   new ActionListener() {
    public void actionPerformed(ActionEvent e) {
     mode=1;
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
    }
   }
  );

  ButtonGroup modeBG=new ButtonGroup();
  modeBG.add(discRB);
  modeBG.add(amplRB);
  modeBG.add(freqRB);

  if(mode==1) discRB.setSelected(true);
  else if(mode==2) amplRB.setSelected(true);
  else freqRB.setSelected(true);

  ////////// param //////////
  JLabel l1=new JLabel("Parameter:");
  l1.setBounds(10, 40, 80, 20);
  getContentPane().add(l1);

  DecimalFormat df=new DecimalFormat();
  df.setMaximumIntegerDigits(1);
  df.setMinimumIntegerDigits(1);
  df.setMinimumFractionDigits(1);
  df.setDecimalSeparatorAlwaysShown(true);
  final JFormattedTextField paramFTF=new JFormattedTextField(df);
  paramFTF.setText(NumberFormat.getInstance().format(param));
  paramFTF.setBounds(85, 40, 50, 20);
  getContentPane().add(paramFTF);

  JButton paramB=new JButton("Accept");
  paramB.setBounds(150, 40, 75, 20);
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

  ////////// instruments //////////
  JLabel l2=new JLabel("Instrument:");
  l2.setBounds(10, 70, 80, 20);
  getContentPane().add(l2);

  instruCB=new JComboBox();
  instruCB.setBounds(90, 70, 140, 20);
  getContentPane().add(instruCB);

  ////////// sensors //////////
  l3=new JLabel("Number of sensors: -");
  l3.setBounds(10, 100, 200, 20);
  getContentPane().add(l3);

  ////////// playback //////////
  JLabel l4=new JLabel("Playback:");
  l4.setBounds(10, 125, 200, 20);
  getContentPane().add(l4);

  final JButton playbackB=new JButton("Mute");
  playbackB.setBounds(90, 125, 140, 20);
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

 public boolean isPlaybackActive() {
  return isPlaybackActive;
 }

 public void setNumSensors(int n) {
  numSensors=n;
  l3.setText("Number of sensors: "+n);
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
