#!/usr/bin/perl -w

my $argc=scalar @ARGV;
my @args = @ARGV;
while(shift){}
if ($argc<1) {
    print "Usage: selectrows.pl fieldpatterns < logfile > newlogfile \n";
    print "\t fieldpatterns: regular expressions that match field descriptions in #C line\n";
    print "\t\t please note that some special characters have to be quoted, see below.\n";
    print "\t Example: \"x\\[\" \"C\\[0\\]\"\n";
    print "\t\t the file (newlogfile) will contain all sensor values x[..] \n\t\t and the first row of the controller matrix C[0][..]\n";
    print "\t Hint: If you want to get rid of the comment lines then use: \n\t\t| grep -v \"^#\" > newlogfile\n";
    exit;
}
my @columns;
while(<>){
    my $line=$_;
    if(/^#[^C]/ ){
       print $line;
     }elsif($line =~ /^#C/)
     {
	 @columns=();
	 print STDERR "I use the following fields: ";
	 print "#C";
	 chomp $line;
	 my @Fields = split(/\s/,$line);
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
	 print STDERR "\nColumns: " . join(" ", @columns) . "\n";
	 print "\n";
   } else{ # normal data line
       $line =~ s/^\s+//; # remove leading space
       chomp $line;       # remove tailing space
       my @values=split(/\s+/, $line);
       foreach my $i (@columns) {
           print $values[$i] . " ";
       }
       print "\n";
   }

}
