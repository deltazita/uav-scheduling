# UAV target coverage
Migration of gforge.inria.fr website

## Related papers: 

["Energy efficient mobile target tracking using flying drones"](http://www.sciencedirect.com/science/article/pii/S1877050913006248)\
["Optimal drone placement and cost-efficient target coverage"](https://deltazita.github.io/files/PointCoverage.pdf)

### Errata in first paper:

-   Formula 10 (right part): h\_u+h\_{u'} \> dist(u,u')\*tan\\theta'
-   the same at line 9 of Algorithm 1

* * * *

### Input scenarios (targets with random positions):

-   [10000-90000 m\^2 terrain size, 100 events](input_files/scenarios100ev.tar.gz)
-   [40000 m\^2 terrain size, 25-175 events](input_files/scenarios20ts.tar.gz)
-   [10000 m\^2 terrain size, 10-50 events](input_files/scenarios10ts.tar.gz)

### Energy consumption models:

-   EC\_i\^t = \\alpha \* altitude\_i\^t, computed as the potential
    energy of an object (mass\*gravity\*height) (less realistic)
-   EC\_i = (\\beta + \\alpha\*h\_i)\*\\Delta t + P\_{max}\*h\_i/U,
    where \\alpha and \\beta are motors coefficients, \\Delta t is the
    amount of time the drone i will stay at altitude h\_i, P\_{max} is
    the maximum power of the motors, and U is the vertical speed of the
    drone. (\\beta + \\alpha\*h\_i)\*\\Delta t corresponds to the energy
    needed to maintain the drone at altitude h and P\_{max}\*h\_i/U is
    the energy needed to rise the drone at the same altitude (more
    realistic).

### Source code

Check Perl scripts

### Animated pictures

["Attractors" mobility model](animated_pictures/attractors-500.gif)\
["RWP" mobility model (fast animation)](animated_pictures/rwp.gif)\
["Random" mobility model](animated_pictures/random-500.gif)\

### Videos

["Attractors" mobility model](videos/attractors.ogv)\
["Random Way Point" mobility model](videos/rwp.ogv)\
["Random" mobility model](videos/attractors.ogv)\
["Random" mobility model with call-n-go's](videos/las-real.ogv)\

