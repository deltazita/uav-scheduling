#!/usr/bin/perl -w
#
# Localised algorithm for drone scheduling and mobile event monitoring
# (version with the smallest enclosing circle)
#
# by Dimitrios Zorbas (dimitrios.zormpas(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)

use strict;
use GD::SVG;
use Math::Trig ':pi';

die "$0 <finish_time> <terrain_file>\n" unless(@ARGV == 2);

my $finish_time = shift;
my @events = ();
my %direction = ();
my $terrain = 0;
my %ecoords = ();
my $min_height = 10;
my $theta_tan = 0.57735027; # tan of 30 degrees
my $max_height = 100;
my %my_position = ();
my %my_covered_events = ();
my %my_neig_covered = ();
my %my_neighbours = ();
my %free = ();
my %my_height = ();
my $r_c = 150; # in meters
my $alpha = 100;
my %timestamp = ();
my $total_energy = 0;
my $avg_drones = 0;
my $mobility = "random";
# my $mobility = "rwp";
# my $mobility = "attractors";

#============================================================================================================================#

read_data();


# first cover all the active events with the minimum possible
# number of drones and with as less as possible doublecoverings
my $drones = 1;
my @available_drones = ();
my %active_drones = ();
push (@available_drones, $drones);
$my_position{$drones} = [0, 0];
$my_covered_events{$drones} = ();
$my_neig_covered{$drones} = ();
$my_height{$drones} = $min_height;
my $active_events = compute_active_events();
my $remaining_events = scalar @$active_events;
while ($remaining_events > 0){
	my $u = shift (@available_drones);
	first_cover($u, $active_events);
}
# draw_terrain(0, $active_events, \%active_drones);

# for each instance of time, calculate events' and drones' route
for (my $t=1; $t<=$finish_time; $t+=1){
	print "############# time $t #############\n";
		
	# update events' position
	move_events($active_events);
	
	# check what you can cover
	foreach my $u (keys %active_drones){
		print "# $u (b): ";
		foreach my $e (@$active_events){
			if (distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]) <= ($my_height{$u}*$theta_tan)){
				$my_covered_events{$u}{$e} = 1;
				print "$e, ";
			}
		}
		print "\n";
	}
	
	foreach my $u (keys %active_drones){
		$timestamp{$u} = rand(0.01) + $max_height/$my_height{$u};
	}
	# update neighbours
	foreach my $u (keys %active_drones){
		$my_neighbours{$u} = ();
		foreach my $u_ (keys %active_drones){
			next if ($u_ eq $u);
			if (zdistance($my_position{$u}[0], $my_position{$u_}[0], $my_position{$u}[1], $my_position{$u_}[1], $my_height{$u}, $my_height{$u_}) <= $r_c){
				foreach my $e (keys %{$my_covered_events{$u_}}){
					push (@{$my_neighbours{$u}}, [$e, $u_, $my_position{$u_}[0], $my_position{$u_}[1], $my_height{$u_}, $timestamp{$u_}]);
				}
			}
		}
		foreach my $el (@{$my_neighbours{$u}}){
			my ($e_, $u_, $x_, $y_, $hu_, $ts) = @$el;
			if (($timestamp{$u} < $ts) && ((distance($my_position{$u}[0], $x_, $my_position{$u}[1], $y_)+$my_height{$u}*$theta_tan+$hu_*$theta_tan)/2 < ($max_height*$theta_tan))){
				if (($my_height{$u}+$hu_) > (distance($my_position{$u}[0], $x_, $my_position{$u}[1], $y_)*$theta_tan)){
					foreach my $e (keys %{$my_covered_events{$u_}}){
						$my_covered_events{$u}{$e} = 1;
					}
				}
			}
			if ((exists $my_covered_events{$u}{$e_}) && ($timestamp{$u} >= $ts) && (distance($my_position{$u}[0], $x_, $my_position{$u}[1], $y_) < ($my_height{$u}*$theta_tan+$hu_*$theta_tan))){
				delete $my_covered_events{$u}{$e_};
				print "# event $e_ deleted by $u\n";
			}
		}
	}
	
	# update drones' position
	foreach my $u (keys %active_drones){
		if ((scalar keys %{$my_covered_events{$u}}) != 0){
			$active_drones{$u} = 1;
			delete $free{$u};
			
			my $check_altitude = 1;
			my $new_height = 0;
			while ($check_altitude == 1){
				my ($x, $y) = (0, 0);
				my @points = ();
				my $max_r = 0;
				foreach my $e (keys %{$my_covered_events{$u}}){
					push (@points, $e);
				}
				if (scalar @points == 1){
					my $ee = undef;
					foreach my $e (keys %{$my_covered_events{$u}}){
						$ee = $e;
					}
					$x = $ecoords{$ee}[0];
					$y = $ecoords{$ee}[1];
					$max_r = $min_height*$theta_tan;
				}else{
					($max_r, $x, $y) = sec(\@points);
				}
				my $distant_event = undef;
				my $max = 0;
				foreach my $e (keys %{$my_covered_events{$u}}){
					if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $max){
						$max = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
						$distant_event = $e;
					}
				}
				$new_height = $max_r/$theta_tan + 1/$theta_tan;
				$my_position{$u} = [$x, $y];
				if ($new_height < $min_height){
					$new_height = $min_height;
				}elsif ($new_height > $max_height){
					print "# I will call another drone for $distant_event\n";
					call_internal_drone($distant_event);
					delete $my_covered_events{$u}{$distant_event};
					print "# event $distant_event deleted by $u\n";
					$max_r = 0;
					my @points = ();
					foreach my $e (keys %{$my_covered_events{$u}}){
						# next if ($e eq $distant_event);
						push (@points, $e);
					}
					if (scalar @points == 1){
						my $ee = undef;
						foreach my $e (keys %{$my_covered_events{$u}}){
							$ee = $e;
						}
						$x = $ecoords{$ee}[0];
						$y = $ecoords{$ee}[1];
						$max_r = $min_height*$theta_tan;
					}else{
						($max_r, $x, $y) = sec(\@points);
					}
					$new_height = $max_r/$theta_tan + 1/$theta_tan;
					$my_position{$u} = [$x, $y];
				}
				if ($new_height <= $max_height){
					$check_altitude = 0;
				}
				print "# altitude check\n";
			}
			if ($new_height > $max_height){
				exit;
			}
			print "# $u(a): ";
			foreach my $e (keys %{$my_covered_events{$u}}){
				print "$e ";
			}
			$my_height{$u} = $new_height;
		}else{
			$free{$u} = 1;
			delete $active_drones{$u};
			print "# drone $u left\n";
		}
		print "\n";
	}
	my $energy = 0;
	foreach my $u (keys %active_drones){
		$energy += $alpha * $my_height{$u};
	}
	print "e\t $t\t $energy\n";
	$total_energy += $energy;
	printf "u\t %d\t %d\n", $t, scalar keys %active_drones;
	$avg_drones += (scalar keys %active_drones);
# 	draw_terrain($t, $active_events, \%active_drones);
}

# statistics here
print "#\n";
printf "# total energy consumed: %.2fJ\n", $total_energy;
printf "# number of drones used: %.2f\n", $avg_drones/$finish_time;
printf "# %s\n", '$Id: las-sec.pl 196 2014-03-05 10:35:22Z jim $';


sub first_cover {
	my $u = shift;
	my $active_events = shift;
	my $benefit = 1;
	while ($benefit == 1){
		my $min_dist = 999999;
		my $new_height = $min_height;
		my $closest_event = undef;
		my $prev_position = $my_position{$u};
		foreach my $e (@$active_events){
			next if (exists $my_covered_events{$u}{$e});
			if (distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]) < $min_dist){
				$min_dist = distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]);
				$closest_event = $e;
			}
		}
		if (defined $closest_event){
			if (grep {$_ eq $closest_event} @{$my_neig_covered{$u}}){
				$closest_event = undef;
			}
		}
		if (defined $closest_event){
			if (scalar keys %{$my_covered_events{$u}} == 0){
				$my_position{$u} = $ecoords{$closest_event};
				$active_drones{$u} = 1;
			}else{
				my ($x, $y) = (0, 0);
				my @points = ();
				push (@points, $closest_event);
				foreach my $e (keys %{$my_covered_events{$u}}){
					push (@points, $e);
				}
				(my $max_r, $x, $y) = sec(\@points);
				$my_position{$u} = [$x, $y];
				$new_height = $max_r/$theta_tan + 1/$theta_tan;
				if ($new_height < $min_height){
					$new_height = $min_height;
				}
			}
			if ($new_height < $max_height){
				$benefit = 1;
			}else{
				$benefit = 0;
			}
			if ($benefit == 1){
				$my_covered_events{$u}{$closest_event} = 1;
				push (@{$my_neig_covered{$u}}, $closest_event);
				$my_height{$u} = $new_height;
				$remaining_events--;
				#print "$remaining_events\n";
				print "# height: $my_height{$u}\n";
				print "# event $closest_event covered by $u\n";
				foreach my $e (@$active_events){
					next if (grep {$_ eq $e} @{$my_neig_covered{$u}});
					if (distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]) <= ($my_height{$u}*$theta_tan)){
						$my_covered_events{$u}{$e} = 1;
						push (@{$my_neig_covered{$u}}, $e);
						$remaining_events--;
						#print "$remaining_events\n";
					}
				}
			}else{
				$my_position{$u} = $prev_position;
				my $distant_event = undef;
				my $max_dist = 0;
				foreach my $e (@$active_events){
					next if (grep {$_ eq $e} @{$my_neig_covered{$u}});
					if (distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]) > $max_dist){
						$max_dist = distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]);
						$distant_event = $e;
					}
				}
				call_drone($distant_event, \@{$my_neig_covered{$u}});
			}
		}else{
			# I covered all the events I could
			$benefit = 0;
			my $distant_event = undef;
			my $max_dist = 0;
			foreach my $e (@$active_events){
				next if (grep {$_ eq $e} @{$my_neig_covered{$u}});
				if (distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]) > $max_dist){
					$max_dist = distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]);
					$distant_event = $e;
				}
			}
			if (defined $distant_event){
				call_drone($distant_event, \@{$my_neig_covered{$u}});
			}
		}
	}
}

sub call_drone {
	my $e = shift;
	my $covered = shift;
	if ((scalar keys %free) > 0){
		my $sel = 0;
		foreach my $u (keys %free){
			if ($u > $sel){
				$sel = $u;
			}
		}
		push (@available_drones, $sel);
		$my_position{$sel} = [$ecoords{$e}[0], $ecoords{$e}[1]];
		$my_covered_events{$sel} = ();
		$my_height{$sel} = $min_height;
		$my_neig_covered{$sel} = ();
		foreach my $ev (@$covered){
			push (@{$my_neig_covered{$sel}}, $ev);
		}
		print "# a free drone ($sel) has been called\n";
	}else{
		$drones++;
		push (@available_drones, $drones);
		$my_position{$drones} = [$ecoords{$e}[0], $ecoords{$e}[1]];
		$my_covered_events{$drones} = ();
		$my_height{$drones} = $min_height;
		$my_neig_covered{$drones} = ();
		foreach my $ev (@$covered){
			push (@{$my_neig_covered{$drones}}, $ev);
		}
		print "# another drone has been called\n";
	}
}

sub call_internal_drone {
	my $e = shift;
	if ((scalar keys %free) > 0){
		my $sel = 0;
		foreach my $u (keys %free){
			if ($u > $sel){
				$sel = $u;
			}
		}
		delete $free{$sel};
		$active_drones{$sel} = 1;
		$my_position{$sel} = [$ecoords{$e}[0], $ecoords{$e}[1]];
		$my_covered_events{$sel} = ();
		$my_height{$sel} = $min_height;
		$my_covered_events{$sel}{$e} = 1;
		print "# a free drone ($sel) has been called to cover $e\n";
	}else{
		$drones++;
		$active_drones{$drones} = 1;
		$my_position{$drones} = [$ecoords{$e}[0], $ecoords{$e}[1]];
		$my_covered_events{$drones} = ();
		$my_height{$drones} = $min_height;
		$my_covered_events{$drones}{$e} = 1;
		print "# another drone has been called\n";
	}
}

sub draw_terrain {
	my $t = shift;
	my $active_e = shift;
	my $active_u = shift;
	my ($display_x, $display_y) = (800, 800); # 800x800 pixel display pane
	my ($norm_x, $norm_y) = (sqrt($terrain)*10, sqrt($terrain)*10);
	my $im = new GD::SVG::Image($display_x, $display_y);
	my $blue = $im->colorAllocate(0,0,255);
	my $black = $im->colorAllocate(0,0,0);
	my $red = $im->colorAllocate(255,0,0);
		
	foreach my $e (@$active_e){
		my ($x, $y) = ($ecoords{$e}[0], $ecoords{$e}[1]);
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
		$im->rectangle($x-2, $y-2, $x+2, $y+2, $red);
		#$im->string(gdSmallFont,$x-2,$y-20,$e,$blue); 
	}

	foreach my $u (keys %{$active_u}){
		my ($x, $y) = ($my_position{$u}[0], $my_position{$u}[1]);
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
		my $r = $my_height{$u} * $theta_tan * 10;
		$r = int(($r * $display_x)/$norm_x);
		$im->arc($x, $y, 2*$r, 2*$r, 0, 360, $blue);
		$im->string(gdLargeFont,$x-2,$y-12,$u,$black);
		$im->rectangle($x-4, $y-14, $x+16, $y+6, $black);
	}

	my $image_file = undef;
	if ($t < 10){
		$t = join ('', "000", $t);
		$image_file = join('.', "time", $t, "svg");
	}elsif (($t >= 10) && ($t < 100)){
		$t = join ('', "00", $t);
		$image_file = join('.', "time", $t, "svg");
	}elsif (($t >= 100) && ($t < 1000)){
		$t = join ('', "0", $t);
		$image_file = join('.', "time", $t, "svg");
	}else{
		$image_file = join('.', "time", $t, "svg");
	}

	open(FILEOUT, ">$image_file") or
		die "could not open file $image_file for writing!";
	binmode FILEOUT;
	print FILEOUT $im->svg;
	close FILEOUT;
}

sub compute_active_events {
	my @actives = ();
	foreach my $ev (@events){
		my ($e, $x, $y) = @$ev;
		push (@actives, $e);
	}
	return \@actives;
}

sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( ($x1-$x2)*($x1-$x2) + ($y1-$y2)*($y1-$y2) ) / 10;
}

sub zdistance {
	my ($x1, $x2, $y1, $y2, $z1, $z2) = @_;
	return sqrt( ($x1-$x2)*($x1-$x2) + ($y1-$y2)*($y1-$y2) + ($z1-$z2)*($z1-$z2) ) / 10;
}

sub read_data {
	while(<>){
		if (/^# stats: (.*)/){
			my $stats_line = $1;
			if ($stats_line =~ /terrain=([0-9]+\.[0-9]+)m\^2/){
				$terrain = $1;
			}
		} elsif (/^# event coords: (.*)/){
			my $point_coord = $1;
			my @coords = split(/\] /, $point_coord);
			@events = map { /([A-Z]+) \[([0-9]+) ([0-9]+)/; [$1, $2, $3]; } @coords;
		}
	}

	my @attractors = ();
	if ($mobility eq "attractors"){
		my $x = int(sqrt($terrain)*10)/2;
		my $y = int(sqrt($terrain)*10)/2;
		my $espeed = rand(10);
		while ($espeed < 1){
			$espeed = rand(10);
		}
		push (@attractors, [1, $x, $y, $espeed]);
		
		my $i = 2;
		my $c = int(sqrt($terrain)*10/3);
		for (my $cx = $c; $cx < 3*$c; $cx += $c){
			for (my $cy = $c; $cy < 3*$c; $cy += $c){
				$espeed = rand(10);
				while ($espeed < 1){
					$espeed = rand(10);
				}
				push (@attractors, [$i, $cx, $cy, $espeed]);
				$i++;
			}
		}
	}
	
	foreach my $event (@events){
		my ($e, $x, $y) = @$event;
		$ecoords{$e} = [$x, $y];
		if ($mobility eq "random"){
			my $espeed = rand(10);
			my $min_angle = rand(2*pi);
			my $max_angle = rand(2*pi - $min_angle) + $min_angle;
			$direction{$e} = [$min_angle, $max_angle, $espeed];
		}elsif ($mobility eq "rwp"){
			my $nx = int(rand(sqrt($terrain)*10));
			my $ny = int(rand(sqrt($terrain)*10));
			my $espeed = rand(10);
			while ($espeed < 1){
				$espeed = rand(10);
			}
			$direction{$e} = [$nx, $ny, $espeed];
		}elsif ($mobility eq "attractors"){
			my $rn = int(rand(scalar @attractors - 1)) + 1;
			foreach my $attractor (@attractors){
				my ($nx, $ny) = (0, 0);
				my ($at, $ax, $ay, $espeed) = @$attractor;
				if ($rn == $at){
					my $check = 1;
					while ($check == 1){
						$check = 0;
						my $f = rand(1);
						if ($f < 0.5){
							$f = -1;
						}else{
							$f = 1;
						}
						$nx = $ax + int(rand(50))*$f;
						$ny = $ay + int(rand(50))*$f;
						if (($nx > sqrt($terrain)*10) || (($ny > sqrt($terrain)*10)) || ($nx < 0) || ($ny < 0)){
							$check = 1;
						}
					}
					$direction{$e} = [$nx, $ny, $espeed];
				}
			}
		}
	}
}

sub move_events {
	my $a_events = shift;
	foreach my $e (@$a_events){
		my ($x0, $y0) = ($ecoords{$e}[0], $ecoords{$e}[1]);
		my ($x, $y) = (0, 0);
		if ($mobility eq "random"){
			my $check = 1;
			while ($check == 1){
				$check = 0;
				my $theta = rand($direction{$e}[1]-$direction{$e}[0]) + $direction{$e}[0];
				my $r = $direction{$e}[2];
				$x = $x0 + $r*cos($theta);
				$y = $y0 + $r*sin($theta);
				if (($x > sqrt($terrain)*10) || (($y > sqrt($terrain)*10)) || ($x < 0) || ($y < 0)){
					$check = 1;
					my $espeed = rand(10);
					my $min_angle = rand(2*pi);
					my $max_angle = rand(2*pi - $min_angle) + $min_angle;
					$direction{$e} = [$min_angle, $max_angle, $espeed];
				}
			}
			$ecoords{$e} = [$x, $y];
			#print "$e: was $x0 $y0 and is $x $y\n";
		}elsif (($mobility eq "rwp") || ($mobility eq "attractors")){
			if (distance($x0, $direction{$e}[0], $y0, $direction{$e}[1]) < ($direction{$e}[2]/10)){
				$x = $direction{$e}[0];
				$y = $direction{$e}[1];
			}else{
				($x, $y) = compute_location($direction{$e}[0], $direction{$e}[1], $x0, $y0, $direction{$e}[2]);
			}
			$ecoords{$e} = [$x, $y];
		}
	}
}

sub compute_location{
	my ($x1, $y1, $x0, $y0, $speed) = @_;
	my $x2 = undef;
	my $y2 = undef;
	if (($x0 - $x1) != 0){ ## avoid division by zero
		my $a = ($y0-$y1)/($x0-$x1);
		my $b = $y1 - $a*$x1;
		
		$x2 = ($speed**2 - (distance($x0, $x1, $y0, $y1)*10 - $speed)**2 - $x0**2 - $y0**2 + $x1**2 + $y1**2 - 2*$b*$y1 + 2*$b*$y0)/(2*$x1 - 2*$x0 + 2*$a*$y1 - 2*$a*$y0);
		$y2 = $a*$x2 + $b;
	}else{
		$x2 = $x1;
		if ($y1 > $y0){
			$y2 = $y0 + $speed;
		}else{
			$y2 = $y0 - $speed;
		}
	}
	return ($x2, $y2);
}

sub sec {
	my $points = shift;
	my $max_dist = 0;
	my ($selected_a, $selected_b, $selected_c) = (undef, undef, undef);
	foreach my $p (@$points){
		foreach my $p_ (@$points){
			next if ($p_ eq $p);
			my $d = distance($ecoords{$p}[0], $ecoords{$p_}[0], $ecoords{$p}[1], $ecoords{$p_}[1]);
			if ($d > $max_dist){
				$max_dist = $d;
				$selected_a = $p;
				$selected_b = $p_;
			}
		}
	}
	$max_dist = $max_dist/2 + 0.00001; 
	my $x_0 = ($ecoords{$selected_a}[0] + $ecoords{$selected_b}[0])/2;
	my $y_0 = ($ecoords{$selected_a}[1] + $ecoords{$selected_b}[1])/2;
	my $fail = 0;
	foreach my $p (@$points){
		if (distance($ecoords{$p}[0], $x_0, $ecoords{$p}[1], $y_0) > $max_dist){
			$fail = 1;
		}
	}
	while ($fail == 1){
		$fail = 0;
		$max_dist = 0;
		foreach my $p (@$points){
			next if (($p eq $selected_a) || ($p eq $selected_b));
			if (distance($ecoords{$p}[0], $x_0, $ecoords{$p}[1], $y_0) > $max_dist){
				$max_dist = distance($ecoords{$p}[0], $x_0, $ecoords{$p}[1], $y_0);
				$selected_c = $p;
			}
		}
		my $D = 2*($ecoords{$selected_a}[0]*($ecoords{$selected_b}[1]-$ecoords{$selected_c}[1])+
			$ecoords{$selected_b}[0]*($ecoords{$selected_c}[1]-$ecoords{$selected_a}[1])+
			$ecoords{$selected_c}[0]*($ecoords{$selected_a}[1]-$ecoords{$selected_b}[1]));
		$x_0 = (($ecoords{$selected_a}[0]**2 + $ecoords{$selected_a}[1]**2)*($ecoords{$selected_b}[1] - $ecoords{$selected_c}[1]) +
			($ecoords{$selected_b}[0]**2 + $ecoords{$selected_b}[1]**2)*($ecoords{$selected_c}[1] - $ecoords{$selected_a}[1]) +
			($ecoords{$selected_c}[0]**2 + $ecoords{$selected_c}[1]**2)*($ecoords{$selected_a}[1] - $ecoords{$selected_b}[1]))/$D;
		$y_0 = (($ecoords{$selected_a}[0]**2 + $ecoords{$selected_a}[1]**2)*($ecoords{$selected_c}[0] - $ecoords{$selected_b}[0]) +
			($ecoords{$selected_b}[0]**2 + $ecoords{$selected_b}[1]**2)*($ecoords{$selected_a}[0] - $ecoords{$selected_c}[0]) +
			($ecoords{$selected_c}[0]**2 + $ecoords{$selected_c}[1]**2)*($ecoords{$selected_b}[0] - $ecoords{$selected_a}[0]))/$D;
		$max_dist = distance($ecoords{$selected_a}[0], $x_0, $ecoords{$selected_a}[1], $y_0) + 0.00001;
		my $selected_c_ = undef;
		foreach my $p (@$points){
			my $d = distance($ecoords{$p}[0], $x_0, $ecoords{$p}[1], $y_0);
			if ($d > $max_dist){
				$fail = 1;
				$selected_c_ = $p;
			}
		}
		if (defined $selected_c_){
			$max_dist = 0;
			foreach my $p (@$points){
				next if (($p ne $selected_a) && ($p ne $selected_b));
				if (distance($ecoords{$p}[0], $ecoords{$selected_c_}[0], $ecoords{$p}[1], $ecoords{$selected_c_}[0]) > $max_dist){
					$max_dist = distance($ecoords{$p}[0], $ecoords{$selected_c_}[0], $ecoords{$p}[1], $ecoords{$selected_c_}[0]);
					$selected_a = $p;
				}
			}
			$selected_b = $selected_c;
			$selected_c = $selected_c_;
		}
		print "# sec check\n";
	}
	
	return ($max_dist, $x_0, $y_0);
}
