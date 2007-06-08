#!/usr/bin/perl -w
use strict;


if(defined $ARGV[0]){
    my $delay = shift;
    my $every = 1;
    if($ARGV[0]){
       $every = shift;
    }
    my $cnt=0;

    while (<>) {
	if(/^#/ ){ 
          $cnt-=1;	      
	}else{	
          select(undef,undef,undef,$delay/1000) if($delay);
	}
        $cnt+=1;
	if($cnt % $every == 0){
          print $_;
	}	
    }

}else{
    print "Usage: feedfile.pl delay [every]< logfile\n\tdelay in milliseconds\n\tevery xth dataset to use\n";
}
