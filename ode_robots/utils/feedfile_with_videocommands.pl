#!/usr/bin/perl -w
use strict;


if(defined $ARGV[0] && defined $ARGV[1]){
    my $name  = shift;
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
    my $framenr=0;
    while (<>) {
        if(/^#V / ){ # skip original video tags
            next;
        }elsif(/^#/ ){
            print $_;
        }else{
            if($cnt >= $start){
                select(undef,undef,undef,$delay/1000) if($delay);
                if($cnt % $every == 0){
                    print $_;
                    print "#V " . $framenr . " " . $name . "\n";
                    $framenr=$framenr+1;
                }
            }
            $cnt+=1;
        }
    }
    print "#QUIT\n";

}else{
    print "Usage: feedfile_with_videocommands.pl videoname delay [every] [start] < logfile\n";
    print "\t outputs the logfile with \"#V CNT videoname\" lines every given line\n";
    print "\t for video recording with matrixviz etc. if no video was recorded in the simulation.\n";
    print "\t otherwise use feedfile.pl directly\n";
    print "\tdelay in milliseconds (on original data)\n\tevery xth dataset to use for the video\n\tstart dataset number to start with\n";
}
