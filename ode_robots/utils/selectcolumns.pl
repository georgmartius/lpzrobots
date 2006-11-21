#!/usr/bin/perl -w

my $argc=scalar @ARGV;
my @args = @ARGV;
while(shift){}
if ($argc<1) { 
    print "Usage: selectrows.pl fieldpatterns < logfile > newlogfile \n";  
    print "\t fieldpatterns: regular expressions that match field descriptions in #C line\n";  
    print "\t \t please note that some special characters have to be quoted, see below.\n";  
    print "\t Example: \"x\\[\" \"C\\[0\\]\"\n";  
    print "\t \t the file will contain all sensor values x[..] and the first row of the controller matrix C[0][..]\n";  
    print "\t Hint: If you want to get rid of the comment lines then use: | grep -v \"^#\" > newlogfile\n";
    exit;
}
my @columns;
while(<>){
    my $line=$_;
    if(/^#[^C]/ )
       { print $line; 
     }elsif($line =~ /^#C/) 
     { 
	 @columns=();
	 print STDERR "I use the following fields: \n";
	 print "#C";
	 my @Fields = split(/ /,$line);       
	 shift @Fields;
	 my $i =0;
	 foreach my $f (@Fields) {	     
	     foreach my $a (@args) {	     
		 if($f =~ /$a/) {
		     print STDERR $f . " ";
		     print " " . $f;
		     push @columns, $i;
		 }
	     }
	     $i++; 
	 }
	 print STDERR "\nColumns: " . (join(" ", @columns)) . "\n";
	 print "\n";	 
   } else{ # normal data line
       $line =~ s/^\s+//; # remove leading space
       my @values=split(/ /, $line);
       foreach my $i (@columns) {	     
	   print $values[$i] . " ";
       }
       print "\n";
   }
      
}


 

