#!/usr/bin/perl -w
#
# Localised algorithm for drone scheduling and mobile event monitoring
#
# by Dimitrios Zorbas (dimitrios.zormpas(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)

use strict;
use GD::SVG;
use Math::Trig ':pi';
use Math::Round qw(:all);

die "$0 <finish_time> <terrain_file>\n" unless(@ARGV == 2);

my $finish_time = shift;
my $minimise_energy = 0;
my $generate_figures = 0;
my @events = ();
my %direction = ();
my $terrain = 0;
my %ecoords = ();
my $min_height = 10;
my $theta_tan = 0.57735027; # tan of 30 degrees
# my $theta_tan = 1.7320508; # tan of 60 degrees
my $max_height = 100 - 1/$theta_tan;
my %my_position = ();
my %my_covered = ();
my %my_neighbours = ();
my %may_be_covered = ();
my %free = ();
my %my_height = ();
my $r_c = 150; # in meters
my $alpha = 100;
my %timestamp = ();
my $total_energy = 0;
my $avg_drones = 0;
my $calls = 0;
my $d = 1;
# my $mobility = "random";
my $mobility = "rwp";
# my $mobility = "attractors";

#============================================================================================================================#

read_data();


# first cover all the events with the minimum possible number of drones
my $uav = 0;
my %active_drones = ();
my @active_events = keys %ecoords;
my @already_covered = ();
central_algo();
draw_terrain(0) if ($generate_figures == 1);

# for each instance of time, calculate events' and drones' route
for (my $t=1; $t<=$finish_time; $t+=1){
	print "############# time $t #############\n";
	
	# update events' position
	move_events(\@active_events);
	
	# check what you can cover
	foreach my $u (keys %active_drones){
		print "# $u (b): ";
		foreach my $e (@active_events){
			if (distance($ecoords{$e}[0], $my_position{$u}[0], $ecoords{$e}[1], $my_position{$u}[1]) <= ($my_height{$u}*$theta_tan)){
				$my_covered{$u}{$e} = 1;
				print "$e ";
			}
		}
		print "\n";
	}
	
	foreach my $u (keys %active_drones){
		$timestamp{$u} = rand(0.01) + $max_height/$my_height{$u};
		@{$may_be_covered{$u}} = ();
	}
	
	# update neighbours
	foreach my $u (keys %active_drones){
		$my_neighbours{$u} = ();
		foreach my $u_ (keys %active_drones){
			next if ($u_ eq $u);
			if (zdistance($my_position{$u}[0], $my_position{$u_}[0], $my_position{$u}[1], $my_position{$u_}[1], $my_height{$u}, $my_height{$u_}) <= $r_c){
				foreach my $e (keys %{$my_covered{$u_}}){
					push (@{$my_neighbours{$u}}, [$e, $u_, $my_position{$u_}[0], $my_position{$u_}[1], $my_height{$u_}, $timestamp{$u_}]);
				}
			}
		}
	}
	foreach my $u (keys %active_drones){
		foreach my $el (@{$my_neighbours{$u}}){
			my ($e_, $u_, $x_, $y_, $hu_, $ts) = @$el;
			if (($timestamp{$u} < $ts) && ((distance($my_position{$u}[0], $x_, $my_position{$u}[1], $y_)+$my_height{$u}*$theta_tan+$hu_*$theta_tan)/2 < ($max_height*$theta_tan))){
				if (($my_height{$u}+$hu_) > (distance($my_position{$u}[0], $x_, $my_position{$u}[1], $y_)*$theta_tan)){
					#next if ($minimise_energy == 1);
					#next if ($t%2 != 0);
					foreach my $e (keys %{$my_covered{$u_}}){
						next if (exists $my_covered{$u}{$e});
						$my_covered{$u}{$e} = 1;
						push (@{$may_be_covered{$u}}, $e);
					}
				}
			}
			if ((exists $my_covered{$u}{$e_}) && ($timestamp{$u} >= $ts) && (distance($my_position{$u}[0], $x_, $my_position{$u}[1], $y_) < ($my_height{$u}*$theta_tan+$hu_*$theta_tan))){
				delete $my_covered{$u}{$e_};
				print "# event $e_ deleted by $u\n";
			}
		}
	}
	
	# update drones' position
	foreach my $u (keys %active_drones){
		if ((scalar keys %{$my_covered{$u}}) > 0){
			#next if ($t%2 != 0);
			my @my_abandoned = ();
			$active_drones{$u} = 1;
			delete $free{$u};
			
			print "# $u(i): ";
			foreach my $e (keys %{$my_covered{$u}}){
				print "$e ";
			}
			print "\n";
			
			my $check_altitude = 1;
			my $new_height = 0;
			while ($check_altitude == 1){
				my ($x, $y) = (0, 0);
				foreach my $e (keys %{$my_covered{$u}}){
					$x += $ecoords{$e}[0];
					$y += $ecoords{$e}[1];
				}
				$x = $x/(scalar keys %{$my_covered{$u}});
				$y = $y/(scalar keys %{$my_covered{$u}});
				$x = nearest_ceil($d, $x);
				$y = nearest_ceil($d, $y);
				$my_position{$u} = [$x, $y];
				my $max_r = 0;
				my $distant_event = undef;
				foreach my $e (keys %{$my_covered{$u}}){
					if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $max_r){
						$max_r = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
						$distant_event = $e;
					}
				}
				$new_height = $max_r/$theta_tan + 1/$theta_tan;
				if ($new_height < $min_height){
					$new_height = $min_height;
				}elsif ($new_height > $max_height){
					if (!grep {$_ eq $distant_event} @{$may_be_covered{$u}}){
						push (@my_abandoned, $distant_event);
					}
					delete $my_covered{$u}{$distant_event};
					print "# event $distant_event deleted by $u\n";
					$max_r = 0;
					foreach my $e (keys %{$my_covered{$u}}){
						if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $max_r){
							$max_r = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
						}
					}
					$new_height = $max_r/$theta_tan + 1/$theta_tan;
				}
				if ($new_height <= $max_height){
					$check_altitude = 0;
				}
			}
			if ($new_height > $max_height){
				exit;
			}
			if (scalar @my_abandoned > 0){
				print "\n";
				call_internal_drone(\@my_abandoned);
			}
			print "# $u(a): ";
			foreach my $e (keys %{$my_covered{$u}}){
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
	draw_terrain($t) if ($generate_figures == 1);
}

# statistics here
print "#\n";
printf "# total energy consumed: %.2fJ\n", $total_energy;
printf "# number of drones used: %.2f\n", $avg_drones/$finish_time;
printf "# total number of calls: %d\n", $calls;
printf "# %s\n", '$Id: las.pl 196 2014-03-05 10:35:22Z jim $';


sub call_internal_drone {
	my $ab_events = shift;
	my ($x, $y) = (0, 0);
	foreach my $e (@$ab_events){
		$x += $ecoords{$e}[0];
		$y += $ecoords{$e}[1];
	}
	$x = $x / (scalar @$ab_events);
	$y = $y / (scalar @$ab_events);
	$x = nearest_ceil($d, $x);
	$y = nearest_ceil($d, $y);
	my $sel = 0;
	foreach my $u (keys %free){
		if ($u > $sel){
			$sel = $u;
		}
	}
	if ($sel == 0){
		$uav++;
		$sel = $uav;
	}
	delete $free{$sel};
	$active_drones{$sel} = 1;
	$my_covered{$sel} = ();
	print "# another drone ($sel) has been called to cover ";
	foreach my $e (@$ab_events){
		$my_covered{$sel}{$e} = 1;
		print "$e ";
	}
	print "\n";
	$calls++;
	my $max_r = 0;
	my $new_height = 0;
	foreach my $e (keys %{$my_covered{$sel}}){
		if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $max_r){
			$max_r = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
		}
	}
	$my_position{$sel} = [$x, $y];
	$new_height = $max_r/$theta_tan + 1/$theta_tan;
	if ($new_height < $min_height){
		$new_height = $min_height;
	}elsif ($new_height > $max_height){
		print "## the invited drone cannot cover all the events (height = $new_height)\n";
		my @ab = ();
		my $check_altitude = 1;
		while ($check_altitude == 1){
			my $r = 0;
			my $distant_event = undef;
			foreach my $e (@$ab_events){
				next if (grep {$_ eq $e} @ab);
				if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $r){
					$r = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
					$distant_event = $e;
				}
			}
			push (@ab, $distant_event);
			delete $my_covered{$sel}{$distant_event};
			print "## event $distant_event deleted by $sel\n";
			$r = 0;
			foreach my $e (keys %{$my_covered{$sel}}){
				if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $r){
					$r = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
				}
			}
			$new_height = $r/$theta_tan + 1/$theta_tan;
			if ($new_height <= $max_height){
				$check_altitude = 0;
			}
		}
		if (scalar @ab > 0){
			#print "\n";
			call_internal_drone2(\@ab);
		}
	}
	$my_height{$sel} = $new_height;
}

sub call_internal_drone2 {
	my $ab_events = shift;
	my ($x, $y) = (0, 0);
	foreach my $e (@$ab_events){
		$x += $ecoords{$e}[0];
		$y += $ecoords{$e}[1];
	}
	$x = $x / (scalar @$ab_events);
	$y = $y / (scalar @$ab_events);
	$x = nearest_ceil($d, $x);
	$y = nearest_ceil($d, $y);
	my $sel = 0;
	foreach my $u (keys %free){
		if ($u > $sel){
			$sel = $u;
		}
	}
	if ($sel == 0){
		$uav++;
		$sel = $uav;
	}
	delete $free{$sel};
	$active_drones{$sel} = 1;
	$my_covered{$sel} = ();
	print "# another drone ($sel) has been called to cover ";
	foreach my $e (@$ab_events){
		$my_covered{$sel}{$e} = 1;
		print "$e ";
	}
	print "\n";
	$calls++;
	my $max_r = 0;
	my $new_height = 0;
	foreach my $e (keys %{$my_covered{$sel}}){
		if (distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) > $max_r){
			$max_r = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y);
		}
	}
	$my_position{$sel} = [$x, $y];
	$new_height = $max_r/$theta_tan + 1/$theta_tan;
	if ($new_height < $min_height){
		$new_height = $min_height;
	}elsif ($new_height > $max_height){
		print "### the invited drone cannot cover all the events (height = $new_height)\n";
		exit;
	}
	$my_height{$sel} = $new_height;
}

sub draw_terrain {
	my $t = shift;
	my ($display_x, $display_y) = (800, 800); # 800x800 pixel display pane
	my ($norm_x, $norm_y) = (sqrt($terrain)*10, sqrt($terrain)*10);
	my $im = new GD::SVG::Image($display_x, $display_y);
	my $blue = $im->colorAllocate(0,0,255);
	my $black = $im->colorAllocate(0,0,0);
	my $red = $im->colorAllocate(255,0,0);
	
	foreach my $e (@active_events){
		my ($x, $y) = ($ecoords{$e}[0], $ecoords{$e}[1]);
		($x, $y) = (int(($x * $display_x)/$norm_x), int(($y * $display_y)/$norm_y));
		$im->rectangle($x-2, $y-2, $x+2, $y+2, $red);
		#$im->string(gdSmallFont,$x-2,$y-20,$e,$blue); 
	}
	
	foreach my $u (keys %active_drones){
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

sub central_algo{
	# Step 1: Cover all the events
	foreach my $e (keys %ecoords){
		next if (grep {$_ eq $e} @already_covered);
		$uav += 1;
		my $x = $ecoords{$e}[0];
		my $y = $ecoords{$e}[1];
		$my_position{$uav} = [$x, $y];
		$my_height{$uav} = distance($ecoords{$e}[0], $x, $ecoords{$e}[1], $y) / $theta_tan;
		
		if ($my_height{$uav} < $min_height){
			$my_height{$uav} = $min_height;
		}elsif ($my_height{$uav} > $max_height){
			print "# Error! the event cannot be reached anymore!\n";
			exit;
		}
		$my_covered{$uav}{$e} = 1;
		push (@already_covered, $e);
		foreach my $e_ (keys %ecoords){
			next if ($e_ eq $e);
			my $dist = distance($ecoords{$e_}[0], $x, $ecoords{$e_}[1], $y);
			if ($dist <= ($theta_tan*$my_height{$uav})){
				if (!grep {$_ eq $e_} @already_covered){
					push (@already_covered, $e_);
				}
				$my_covered{$uav}{$e_} = 1;
			}
		}
		print "# $uav\'s final altitude is $my_height{$uav}\n";
	}
	
	# Step 2: Merge neighbouring UAVs
	foreach my $u (keys %my_position){
		my @points = ();
		foreach my $e (keys %{$my_covered{$u}}){
			push (@points, $e);
		}
		my $sel = 0;
		my @candidates = ();
		while (defined $sel){ # sort the neighbours according to distance
			my $min_dist = 999999;
			$sel = undef;
			foreach my $u_ (keys %my_position){
				next if (($u_ == $u) || (!exists $my_position{$u}) || (grep {$_ eq $u_} @candidates));
				next if (distance($my_position{$u_}[0], $my_position{$u}[0], $my_position{$u_}[1], $my_position{$u}[1]) > (2*$theta_tan*$max_height));
				my $dist = distance($my_position{$u_}[0], $my_position{$u}[0], $my_position{$u_}[1], $my_position{$u}[1]);
				if ($dist < $min_dist){
					$sel = $u_;
					$min_dist = $dist;
				}
			}
			if (defined $sel){
				push (@candidates, $sel);
			}
		}
		while (scalar @candidates > 0){ # check one-by-one all the neighbours
			$sel = shift (@candidates);
			print "# trying $u<->$sel\n";
			my $is_efficient = 1;
			
			# merge all the events
			foreach my $e (keys %{$my_covered{$sel}}){
				if (!grep {$_ eq $e} @points){
					push (@points, $e);
				}
			}
			
			# compute new altitude
			my ($r, $x, $y) = sec(\@points);
			$r += 0.1; # solve rounding problems
			my $temp_height = $r/$theta_tan;
			if ($temp_height < $min_height){
				$temp_height = $min_height;
			}elsif ($temp_height > $max_height){
				$is_efficient = 0;
			}
			
			# check if the new altitude is more energy efficient than having two uavs
			if ($minimise_energy == 1){
				if (($my_height{$u} + $my_height{$sel}) < $temp_height){
					$is_efficient = 0;
				}
			}
			
			if ($is_efficient == 1){
				# put the node on the grid
				my $nx = nearest_ceil($d, $x);
				my $ny = nearest_ceil($d, $y);
				my $limit = sqrt($terrain)*10;
				if ($nx > $limit){
					$nx = $limit;
				}
				if ($ny > $limit){
					$ny = $limit;
				}
				
				# check if all the events are covered
				my $reach_check = 1;
				foreach my $e (@points){
					my $dist = distance($ecoords{$e}[0], $nx, $ecoords{$e}[1], $ny);
					if ($dist > ($theta_tan*$temp_height)){
						$reach_check = 0;
					}
				}
				if ($reach_check == 1){
					$my_height{$u} = $temp_height;
					$my_position{$u} = [$nx, $ny];
					$my_covered{$u} = ();
					foreach my $e (keys %ecoords){
						my $dist = distance($ecoords{$e}[0], $nx, $ecoords{$e}[1], $ny);
						if ($dist <= ($theta_tan*$my_height{$u})){
							if (!exists $my_covered{$u}{$e}){
								$my_covered{$u}{$e} = 1;
							}
						}
					}
					delete $my_position{$sel};
					delete $my_covered{$sel};
					delete $my_height{$sel};
					print "### $u merged with $sel\n";
				}
			}else{
				print "### $u<->$sel not efficient ($temp_height)\n";
			}
		}
	}
	
	# Step 3: Rearrange UAVs
	my %my_double_covered = ();
	my @descending = ();
	while (scalar @descending != scalar keys %my_position){ # sort uavs according to their coverage status
		my $max_cover = 0;
		my $sel = undef;
		foreach my $u (keys %my_position){
			next if (grep {$_ eq $u} @descending);
			my $cover = scalar keys %{$my_covered{$u}};
			if ($cover > $max_cover){
				$max_cover = $cover;
				$sel = $u;
			}
		}
		if (defined $sel){
			push (@descending, $sel);
		}
	}
	while (scalar @descending > 0){
		my $u = pop (@descending);
		my @points = ();
		foreach my $e (keys %{$my_covered{$u}}){
			push (@points, $e);
			foreach my $u_ (@descending){
				next if ($u_ == $u);
				foreach my $e_ (keys %{$my_covered{$u_}}){
					if ($e_ eq $e){
						push (@{$my_double_covered{$u}}, $e);
						print "### $u abandoned $e\n";
					}
				}
			}
		}
		$my_covered{$u} = ();
		foreach my $e (@points){
			next if (grep {$_ eq $e} @{$my_double_covered{$u}});
			$my_covered{$u}{$e} = 1;
		}
		my ($r, $x, $y) = (0, 0, 0);
		if (scalar keys %{$my_covered{$u}} == 1){
			my $p = undef;
			foreach my $e (keys %{$my_covered{$u}}){
				$p = $e;
			}
			$my_covered{$u} = ();
			$r = $min_height*$theta_tan;
			$x = $ecoords{$p}[0];
			$y = $ecoords{$p}[1];
		}else{
			next if (scalar keys %{$my_covered{$u}} == 0);
			my @pts = (keys %{$my_covered{$u}});
			($r, $x, $y) = sec(\@pts);
			$r += 0.1; # solve rounding problems
		}
		my $temp_height = $r/$theta_tan;
		if ($temp_height < $min_height){
			$temp_height = $min_height;
		}
		my $nx = nearest_ceil($d, $x);
		my $ny = nearest_ceil($d, $y);
		my $limit = sqrt($terrain)*10;
		if ($nx > $limit){
			$nx = $limit;
		}
		if ($ny > $limit){
			$ny = $limit;
		}
		my $reach_check = 1;
		foreach my $e (keys %{$my_covered{$u}}){
			my $dist = distance($ecoords{$e}[0], $nx, $ecoords{$e}[1], $ny);
			if ($dist > ($theta_tan*$temp_height)){
				$reach_check = 0;
			}
		}
		if ($reach_check == 1){
			$my_height{$u} = $temp_height;
			$my_position{$u} = [$nx, $ny];
		}
	}
	
	foreach my $u (keys %my_position){
		$active_drones{$u} = 1;
	}
}

sub sec {
	my $points = shift;
	my $max_dist = 0;
	my ($selected_a, $selected_b, $selected_c) = (undef, undef, undef);
	foreach my $p (@$points){
		foreach my $p_ (@$points){
			next if ($p_ eq $p);
			my $dist = distance($ecoords{$p}[0], $ecoords{$p_}[0], $ecoords{$p}[1], $ecoords{$p_}[1]);
			if ($dist > $max_dist){
				$max_dist = $dist;
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
		my $D = 2*($ecoords{$selected_a}[0]*($ecoords{$selected_b}[1]-$ecoords{$selected_c}[1])+$ecoords{$selected_b}[0]*($ecoords{$selected_c}[1]-$ecoords{$selected_a}[1])+$ecoords{$selected_c}[0]*($ecoords{$selected_a}[1]-$ecoords{$selected_b}[1]));
		$x_0 = (($ecoords{$selected_a}[0]**2 + $ecoords{$selected_a}[1]**2)*($ecoords{$selected_b}[1] - $ecoords{$selected_c}[1]) +
		($ecoords{$selected_b}[0]**2 + $ecoords{$selected_b}[1]**2)*($ecoords{$selected_c}[1] - $ecoords{$selected_a}[1]) +
		($ecoords{$selected_c}[0]**2 + $ecoords{$selected_c}[1]**2)*($ecoords{$selected_a}[1] - $ecoords{$selected_b}[1]))/$D;
		$y_0 = (($ecoords{$selected_a}[0]**2 + $ecoords{$selected_a}[1]**2)*($ecoords{$selected_c}[0] - $ecoords{$selected_b}[0]) +
		($ecoords{$selected_b}[0]**2 + $ecoords{$selected_b}[1]**2)*($ecoords{$selected_a}[0] - $ecoords{$selected_c}[0]) +
		($ecoords{$selected_c}[0]**2 + $ecoords{$selected_c}[1]**2)*($ecoords{$selected_b}[0] - $ecoords{$selected_a}[0]))/$D;
		$max_dist = distance($ecoords{$selected_a}[0], $x_0, $ecoords{$selected_a}[1], $y_0) + 0.00001;
		my $selected_c_ = undef;
		foreach my $p (@$points){
			my $dist = distance($ecoords{$p}[0], $x_0, $ecoords{$p}[1], $y_0);
			if ($dist > $max_dist){
				$fail = 1;
				$selected_c_ = $p;
			}
		}
		if (defined $selected_c_){
			$max_dist = 0;
			foreach my $p (@$points){
				next if (($p ne $selected_a) && ($p ne $selected_b));
				if (distance($ecoords{$p}[0], $ecoords{$selected_c_}[0], $ecoords{$p}[1], $ecoords{$selected_c_}[0]) > $max_dist){
					#$max_dist = distance($ecoords{$p}[0], $x_0, $ecoords{$p}[1], $y_0);
					$max_dist = distance($ecoords{$p}[0], $ecoords{$selected_c_}[0], $ecoords{$p}[1], $ecoords{$selected_c_}[0]);
					$selected_a = $p;
				}
			}
			$selected_b = $selected_c;
			$selected_c = $selected_c_;
		}
	}
	
	return ($max_dist, $x_0, $y_0);
}
