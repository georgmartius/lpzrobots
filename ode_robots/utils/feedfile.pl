#!/usr/bin/perl -w
use strict;

if($ARGV[0]){
    my $delay = shift;

    while (<>) {
        select(undef,undef,undef,$delay/1000);
        print $_;
    }

}else{
    print "Usage: feedfile.pl delay < logfile\n\tdelay in milliseconds\n";
}
