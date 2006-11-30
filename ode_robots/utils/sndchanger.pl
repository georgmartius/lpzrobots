#!/usr/bin/perl -w

use strict;

#use Audio::Play;
#use Audio::Data;
#$audio = Audio::play->new();
#$audio->tone('1000','0.001','50');
#$svr = Audio::Play->new();
#$svr->play($audio);


my $robotID=rand();
my $altCount = 2500;
my $alternation = 0;
while(<>) {
 if($alternation == 0) {open(SNDFILE, ">snd0$robotID.txt");}
 elsif($alternation == $altCount/2) {
  close(SNDFILE);
  system("sndchanger -f 8000 snd0$robotID.txt snd0$robotID.aif");
  system("amarok snd0$robotID.aif");
  open(SNDFILE, ">snd1$robotID.txt");
 }

 chomp;
 my $line = $_;
 if(!/^#/) {
  my @Fields = split(/ /,$line);
  shift @Fields;
  foreach my $i (@Fields) {
   $i=0.0001*$i;
   if($i < -1) {$i = -1;}
   elsif($i > 1) {$i = 1;}
   print SNDFILE "$i\n";
  }
  if($alternation == $altCount) {
   $alternation = 0;
   close(SNDFILE);
   system("sndchanger -f 8000 snd1$robotID.txt snd1$robotID.aif");
   system("amarok snd0$robotID.aif");
  }
  else {
   $alternation++;
  }
 }
}
close(SNDFILE);
system("rm -f snd0$robotID.txt snd1$robotID.txt snd0$robotID.aif snd1$robotID.aif");