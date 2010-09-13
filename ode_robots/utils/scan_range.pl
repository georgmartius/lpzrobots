#!/usr/bin/perl -w
# simple perl script to scan a parameter range.
#  this is just a template. you need to customize everything

use strict;

my ($i,$param1,$param2);
# Headline
print("eps|disc|expl|sarsa|interval\n");
foreach $param1 (0, 0.05, 0.2) {
    foreach $param2 (1, 2, 3) {
        foreach $i (&range(0,1,4)){
    	    system("./start_opt -p $param1 -o param2 -out $i");
	}
    }
}


sub range {
    my $min=shift;
    my $step=shift;
    my $max=shift;
    my @array;
    for(my $i=$min; $i<$max; $i+=$step){
	push @array,$i;
    }
    return @array;
}
