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
    my $cnt=0;

    while (<>) {
        my $cmt=0;
        if(/^#/ ){
          $cnt-=1;
          $cmt=1;
        }else{
          select(undef,undef,undef,$delay/1000) if($delay);
        }
        $cnt+=1;
        if($cmt || ($cnt % $every == 0 && $cnt >= $start)){
            print $_;
        }
    }

}else{
    print "Usage: feedfile.pl delay [every] [start] < logfile\n\tdelay in milliseconds\n\tevery xth dataset to use\n\tstart dataset number to start with\n";
}
