#!/usr/bin/perl -w
#
# This script generates a 2D terrain of points
#
# by Dimitrios Zorbas (dimitrios.zormpas(at)inria.fr), Dimitris Glynos (daglyn(at)storm.cs.unipi.gr)
#
# Distributed under the GPLv3 (see LICENSE file)

use strict;
use Math::Random;

my $SHOW_PROGRESS=1; # make this 0 if you don't want to see anything on STDERR

my ($terrain_x, $terrain_y) = (1000 * 10, 1000 * 10); 	# 1km by 1km terrain

sub progress_bar {
	my $title = shift;
	my $cur_progress = shift;
	my $max_progress = shift;
	my $prev_progress = shift;

	return unless $SHOW_PROGRESS;

	return $max_progress if ($prev_progress == $max_progress);

	if ($cur_progress == 0){
		printf STDERR "%35s [", $title;
	} elsif ($cur_progress == $max_progress){
		printf STDERR ".] Done!\n";
	} else {
		my $change = int ((10*($cur_progress - $prev_progress))/ $max_progress);
		if ($change >= 1){
			print STDERR "." x $change;
			$prev_progress = $cur_progress;
		}
	}
	return $prev_progress;
}

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2))+(($y1-$y2)*($y1-$y2)) );
}

sub random_int {
	my $low = shift;
	my $high = shift;
	return Math::Random::random_uniform_integer(1, $low, $high);
}

(@ARGV==2) || die "usage: $0 <num_of_points> <area_of_interest%>\n";

my $num_events = $ARGV[0];
my $density = $ARGV[1]/100;

($num_events < 1) && die "num_points must be greater than one!\n";
(($density <= 0) || ($density > 100)) && die "area of interest must lie between 0 < x <= 100.0\n";

my $norm_x = int($terrain_x * $density);	# normalised terrain_x
my $norm_y = int($terrain_y * $density);	# normalised terrain_y

my @events;

my $stats_state = 0;
my $stats_cnt = 0;
my $stats_total = 0;


### GENERATING POSITIONS ###

my %events_temp = ();
($stats_state, $stats_total) = (0, $num_events - 1);
for(my $i=1; $i<=$num_events; $i++){
	my ($x, $y) = (random_int(1, $norm_x), random_int(1, $norm_y));

	while (exists $events_temp{$x}{$y}){
		($x, $y) = (random_int(1, $norm_x), random_int(1, $norm_y));
	}
	$events_temp{$x}{$y} = 1;
	push(@events, [$x, $y]);
	$stats_state = progress_bar("Generating events:", $i-1, $stats_total, $stats_state);
}

print "# event coords:";
my $e = "A";
foreach my $event (@events){
	my ($x, $y) = @$event;
	printf " %s [%i %i]", $e, $x, $y;
	$e++;
}
print "\n";

print  	"# generated with: $0 ",join(" ",@ARGV),"\n";
printf 	"# stats: points=%i terrain=%.1fm^2\n", scalar @events, $norm_x*$norm_y/100;
printf 	"# %s\n", '$Id: generate_events.pl 189 2014-02-25 18:23:59Z jim $';
