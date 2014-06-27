#!/usr/bin/perl -w
use strict;


if(defined $ARGV[0]){
    my $delay = shift;
    my $every = 1;
    my $start = 0;
    if($ARGV[0]){
       $every = shift;
    }
    if($ARGV[0]){
       $start = shift;
    }
    my $noquit=0;
    if($ARGV[0]){
       $noquit = shift;
    }
    my $cnt=0;

    while (<>) {
        if(/^#/ ){
            print $_;
        }else{
            if($cnt >= $start){
                select(undef,undef,undef,$delay/1000) if($delay);
                if($cnt % $every == 0){
                    print $_;
                }
            }
            $cnt+=1;
        }
    }
    if(!$noquit){
        print "#QUIT\n";
    }

}else{
    print "Usage: feedfile.pl delay [every] [start] [noquit] < logfile\n\tdelay in milliseconds (on original data)\n\tevery xth dataset to use\n\tstart dataset number to start with\n\tif not noquit then the quit command is send at the end\n";
}
