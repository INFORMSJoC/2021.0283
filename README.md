[![INFORMS Journal on Computing Logo](https://INFORMSJoC.github.io/logos/INFORMS_Journal_on_Computing_Header.jpg)](https://pubsonline.informs.org/journal/ijoc)

# Benchmarking Instances Generator for Aircraft Conflict Resolution
This archive is distributed in association with the [INFORMS Journal on Computing](https://pubsonline.informs.org/journal/ijoc) under the [MIT License](LICENSE).

The software and data in this repository are a snapshot of the software and data that were used in the research reported on the paper 
"Aircraft conflict resolution: A benchmark generator" by  M. Pelegrín and M. Cerulli.

## Cite

To cite this software, please cite the [paper]() using its DOI and the software itself, using the following DOI.

[![DOI](https://zenodo.org/badge/285853815.svg)]()

Below is the BibTex for citing this version of the code.

```
@article{TDinstancesGenerator,
  author =        {Pelegrín, M. and Cerulli, M.},
  publisher =     {INFORMS Journal on Computing},
  title =         {TD instances generator},
  year =          {2022},
  doi =           {},
  url =           {},
}  
```

## Description
Aircraft Conflict Resolution is one of the major tasks of computer-aided Air Traffic Management and represents a challenging optimization problem. This code generates benchmarking instances, each of them consisting of a set of flights with initial positions and vectors of velocities.

You will be able to generate:

	* Predefined 2D and 3D scenarios, including flow patterns and single encounter patterns.
	
	* Random 2D and 3D scenarios.
	
	* Pseudo-random 2D and 3D scenarios, where the trajectories meet a predefined traffic congestion.

The [src](https://github.com/MartinaCerulli/TDInstancesGenerator/tree/main/src) folder contains generator's source (.cpp) code. The [scripts](https://github.com/MartinaCerulli/TDInstancesGenerator/tree/main/scripts) folder contains bash scripts which can be executed to perform the computational study reported in Section 4 of the paper. In the [results](https://github.com/MartinaCerulli/TDInstancesGenerator/tree/main/results) folder the figures associated to different generable configurations included in the paper are collected to illustrate the potential outputs of the generator. Finally, in the [examples](https://github.com/MartinaCerulli/TDInstancesGenerator/tree/main/examples) folder, we report the outputs of the generator when generating two easy circle instances, differing only in the speed range of the aircraft. Some explanations on these outputs are provided in the [ToyExampleOutputs.pdf](https://github.com/MartinaCerulli/TDInstancesGenerator/tree/main/examples/ToyExampleOutputs.pdf) file.


## Generating instances
### **Compile the program**
In order to compile the program, type:

`g++ generator-TD.cpp -o generator`

### **Run the program**
To run the program, type:

`./generator [FLAG] [value]`

In the following, we list the different flags that can be used with their associated values with the format:

FLAG	\[value\] \[default_value\]

The coordinates of initial positions of aircraft are represented with the following notation:

(x0,y0,z0)=(radius\*cos(*theta*)\*sin(*phi*),radius\*sin(*theta*)\*sin(*phi*),radius\*cos(*phi*))) for 3D instances,
(x0,y0)=(radius\*cos(*theta*), radius\*sin(*theta*)) for 2D instances.

See M. Pelegrín and M.Cerulli (2022) "Aircraft conflict resolution: A benchmark generator", for more details.

**General flags**
- -V   \[\] \[verbose=off\] (set verbose output on, for pseudo-random configuration)
- -f   \["instance_file_name"\] \[instance\] (print generated instance to a file with a specific name)
- -seed	\[random_seed\] \[14\]
- -mode	\[0,...,15/ "CP","RCP","RP","RRP","GP","RGP","PR2","R2","SP","RSP","PL","RPL","QP","RQP","PR3","R3"\] \[6\]
	
	0/"CP": circle
        1/"RCP": random circle        
        2/"RP": rhomboidal
        3/"RRP": random rhomboidal        
        4/"GP": grid        
        5/"RGP": random grid        
        6/"PR2": 2D pseudo-random        
        7/"R2": 2D random        
        8/"SP": sphere        
        9/"RSP": random sphere        
        10/"PL": polyhedral        
        11/"RPL": random polyhedral        
        12/"QP": cubic 
        13/"RQP": random cubic
        14/"PR3": 3D pseudo-random        
        15/"R3": 3D random
- -n      \[number of aircraft\] \[10 if mode is not in {2,3,4,5,10,11,12,13}\] 
- -vmin   \[min_speed\] \[400\]
- -vmax   \[max_speed\] \[400\]
- -vdefault \[speed\] \[400\] (vmin=vmax=speed)
- -D      \[safety_distance\] \[5\]

**Flags for circle (mode in {0,1}) and spheric problems (mode in {8,9})**
- -r      \[radius\] \[200\]

*theta* in \[secInix,secInix+secAngx\]; alternatively, *theta* in \[secInixd,secInixd+secAngxd\]
- -secInix    \[lower bound for *theta* in radians\] \[0\]
- -secAngx    \[amplitude of the sector containing *theta* in radians\] \[2\*\pi\]
- -secInixd   \[lower bound for *theta* in degrees\] \[0\]
- -secAngxd   \[amplitude of the sector containing *theta* in degrees\] \[360\]

ONLY SPHERE, *phi* in \[secIniz,secIniz+secAngz\]; alternatively, *phi* in \[secInizd,secInizd+secAngzd\]
- -secIniz    \[lower bound for *phi* in radians\] \[0\]   
- -secAngz    \[amplitude of the sector containing *phi* in radians\] \[\pi\]  
- -secInizd   \[lower bound for *phi* in degrees\] \[0\] 
- -secAngzd   \[amplitude of the sector containing *phi* in degrees\] \[\pi\]     

**Flags for rhomboidal (mode in {2,3}), grid (mode in {4,5}), polyhedral (mode in {10,11}), and cubic problems (mode in {12,13})** 

Rhomboidal and grid scenarios (2D) only contain a single horizontal plane. Polyhedral and cubic ones (3D) contain both horizontal and vertical planes. In all cases: horizontal planes contain horizontal and slopping trails; vertical planes only contain slopping trails.
- -HP         \[number of horizontal planes (HPs)\] \[3\]   (only polyhedral)
- -VP         \[number of vertical planes (VPs)\] \[2\]     (only polyhedral)
- -dHP        \[distance between consecutive HPs\] \[3\*D\]  (only polyhedral)
- -dVP        \[distance between consecutive VPs\] \[3\*D\]  (only polyhedral)
- -mx         \[list of number of horizontal trails at each horizontal plane, separated by spaces\] \[2 2 2\]
- -my         \[list of number of slopping trails at each horizontal plane, separated by spaces\] \[2 2 2\]      
- -mz         \[list of number of slopping trails at each vertical plane, separated by spaces\] \[2 2\]  (only polyhedral)
- -nx         \[number of aircraft on each horizontal trail\] \[1\]
- -ny         \[number of aircraft on each slopping trail located at a horizontal plane\] \[1\]
- -nz         \[number of aircraft on each slopping trail located at a vertical plane\] \[1\] (only polyhedral)
- -dx         \[distance between consecutive horizontal trails (on the same plane, if polyhedral)\] \[3\*D\] 
- -dy         \[distance between consecutive slopping trails (on the same horizontal plane, if polyhedral)\] \[3\*D\]  
- -dz         \[distance between consecutive slopping trails on the same vertical plane\] \[3\*D\]  (only polyhedral)
- -d_aircraft \[distance between consecutive aircraft on the same trail\] \[2\*D\] 
- -alpha      \[list of slopes in radians of slopping trails at the different horizontal planes, separated by spaces\] \[\pi/6 \pi/6 \pi/6\](polyhedral/cubic) \[\pi/4\](rhomboidal/grid)
- -alphad     \[list of slopes in degrees of slopping trails at the different horizontal planes, separated by spaces\] \[30 30 30\](polyhedral/cubic) \[45\](rhomboidal/grid)
- -beta       \[list of slopes in radians of slopping trails at the different vertical planes, separated by spaces\] \[2\pi/3 2\pi/3\] (only polyhedral)
- -betad      \[list of slopes in degrees of slopping trails at the different vertical planes, separated by spaces\] \[120 120\] (only polyhedral)

**Flags for random predefined scenarios (modes 1,3,5 and 9,11,13)**

(angle=predefined_angle+dev, dev in \[hamin,hamax\])
- -hamin  \[min heading angle deviation\] \[-\pi/6\]
- -hamax  \[max heading angle deviation\] \[\pi/6\]

**Flags for random and pseudo-random problems (mode in {6,7} for 2D, {14,15} for 3D)**
- -h          \[height\] (2D rectangle: w x h) \[400 for 2D, 100 for 3D\]
- -w          \[width\]  (2D rectangle: w x h) \[400 for 2D, 100 for 3D\]
- -a          \[altitude\] (3D cube: w x h x a) \[100\] (only for 3D)

**Flags for pseudo-random problems**
- -pc         \[probability of conflict\] \[0.5\]
- -maxc       \[max number of conflicts for a fixed aircraft\] \[n-1\]
- -nc         \[desired number of conflicts\] \[round( pc\*(n/2)\*((1+maxc)/2) )\]
- -airconfig  \["W-N"/"N-S"/"W-E"/"W-U"/"N-U"/"U-D"/"all"\]  \[all\]

	"W-N": initial positions in West and North
        "N-S": initial positions in North and South     
        "W-E": initial positions in West and East        
        "W-U": initial positions in West and Upper      (only 3D)        
        "N-U": initial positions in North and Upper     (only 3D)        
        "U-D": initial positions in Upper and Bottom    (only 3D)        
        "all": initial positions in the 4 borders of the rectangle (2D) or the 6 faces of the parallelepiped (3D)
### **Remarks**
* In grid problems alpha will be set to M_PI/2 (i.e. 90°), in cubic problems both alpha and beta will be set to M_PI/2 (i.e. 90°).
* In polyhedral/cubic instances it is possible to insert a different value of mx, my, mz, alpha, alphad, beta, betad for each horizontal plane. 
For example the user could type: `./generator -mode PL -HP 3 -VP 2 -mx 2 3 5 -my 1 -alphad 45 60 100 -beta 3.14159`.
The corresponding scenario will be polyhedral, with 3 horizontal planes and 2 vertical planes. The first horizontal plane will have 2 horizontal trails and 1 slopping trail with a slope of 45°. The second horizontal plane will have 3 horizontal trails and 2 vertical trails (default value, since the user inserted only one value for my) with a slope of 60°. The third horizontal plane will have 5 horizontal trails and 2 vertical trails (default value) with a slope of 100°. The first vertical plane will have 2 vertical trails (default value, since the user inserted no value for mz) with a slope of M_PI. Finally, the last vertical plane will have 2 vertical trails (default value) with a slope of 2\*M_PI/3  (default value, since the user inserted only one value for beta).
* In polyhedral/cubic instances, each of the options dx, dy, dz, nx, ny, and nz, corresponds to a single value, which will be the same for every horizontal/vertical plane.


### **Usage examples**
```
0. CP - Circle problem: ./generator -mode 0 -vmin 380 -vmax 400 -r 220
1. RCP - Random circle problem: ./generator -mode RCP -r 300 -seed 15 -secInixd 90
2. RP - Rhomboidal problem: ./generator -mode 2 -w 8 -h 8 -nx 1 -ny 1 
3. RRP - Random rhomboidal problem: ./generator -mode 3 -w 8 -h 8 -mx 3 -ny 2 -alphad 30
4. GP - Grid problem: ./generator -mode GP -mx 3 -my 4 -nx 2 -d_aircraft 15
5. RGP - Random grid problem: ./generator -mode RGP -mx 3 -my 4 -nx 2 -hamin 0 -hamax 0.628
6. PR2 - 2D pseudo-random problem: ./generator -mode 6 -n 20 -nc 21 -h 300
7. R2 - 2D random problem: ./generator -mode 7 -n 20 -w 500 -seed 1
8. SP - Sphere problem: ./generator -mode SP -n 13 -r 300 -secAngx 3.14 -secInizd 45 -secAngzd 45
9. RSP - Random sphere problem: ./generator -mode RSP -secAngxd 45 -hamin 0 -hamax 0.30
10. PL - Polyhedral problem: ./generator -mode PL -VP 3 -mx 2 -ny 2 -alphad 45 30 -d_aircraft 13
11. RPL - Random polyhedral problem: ./generator -mode 11 -my 4 -mz 2 -betad 20 230
12. QP - Cubic problem: ./generator -mode 12 -HP 3 -mx 3 3 -my 4 4 4 -dHP 30
13. RQP - Random cubic problem: ./generator -mode RQP -HP 4 -my 2 3 4 -dVP 20 -vmax 500
14. PR3 - 3D pseudo-random problem: ./generator -mode PR3 -n 30 -nc 15 -airconfig 'W-U'
15. R3- 3D random problem: ./generator -mode 15 -n 40 -a 200 -h 200 -w 200
```
### **Output format**

The program will produce 2 files:

1) instance.dat: contains the list of initial positions p0=(x0,y0,z0), and vectors of velocities of the aircraft (either in polar coordinates V_polar=(v,*theta*,*phi*) or in Cartesian coordinates V=(Vx,Vy,Vz)=(v\*cos(*theta*)\*sin(*phi*),v\*sin(*theta*)\*sin(*phi*),v\*cos(*phi*))

2) Figure.tex: LaTeX code (TikZ) for a graphical representation of the instance in a Cartesian coordinate system, i.e. initial positions and vectors of velocities of the aircraft. The pairs in conflict, and the number of conflicts for each aircraft are reported as well. Each aircraft trajectory is represented with different colors. Use LaTeX compiler to generate a .pdf with the corresponding figure. You might need to change 'scale' in `\begin{tikzpicture} [scale=...]` for a proper visualization.


Additional information on the generated instance can be found as output:
- Pairs in conflict
- Distance at the time of minimal separation
- Duration of conflict
- Number of conflicts per aircraft
- Proportion of aircraft with at least one conflict
- Proportion of pairs of conflicts


## Replicating results
If the generated instances are used as testbed, besides citing the paper "Aircraft conflict resolution: A benchmark generator", please specify to which values the different parameters are set. In this way, if anyone wants to use the same set of instances for testing purpose, it will only suffice to use the same values for the involved parameters.


## How to contribute
Thank you for considering contributing to our project! To report bugs and ask questions, please refer to the official [issue tracker](https://github.com/MartinaCerulli/TDInstancesGenerator/issues). You can also address a problem by (1) forking the project, (2) correcting the bug / adding a feature, and (3) generating a pull request. However, we recommend that you first contact the [authors](https://github.com/MartinaCerulli/TDInstancesGenerator/blob/main/AUTHORS) and discuss the desired feature request.

You are also very welcome if you want to upload in our repository the instances you generated with our generator and used in your work! In this way, other researchers can use the same set of instances as benchmark, without having to re-generate such instances using the parameters you used. If this is the case, please send us an email and we will take care of handling your request. 
