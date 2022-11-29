### Generating paper instances
Once the generator is compiled, the executable file `./generator` is produced in the current working directory. In the following, we report the commands that can be used to generate the instances illustrated in the different sections of the paper and collected in this folder. 
## Circle instances
- Figure 1a: `./generator -mode CP -n 10`
- Figure 1b: `./generator -mode CP -n 10 -secInixd 20 -secAngxd 50 -seed 14`
- Figure 2a: `./generator -mode RCP -n 10`
- Figure 2b: `./generator -mode RCP -n 10 -secInixd 20 -secAngxd 50 -seed 14`.


## Sphere instances
- Figure 3a: `./generator -mode SP -n 15`
- Figure 3b: `./generator -mode SP -n 15 -secInixd 0 -secAngxd 90 -secInizd 0 -secAngzd 90`
- Figure 4a: `./generator -mode RSP -n 15`
- Figure 4b: `./generator -mode RSP -n 15 -secInixd 0 -secAngxd 90 -secInizd 0 -secAngzd 90`


## Rhomboidal instances
- Figure 5a: `./generator -mode RP -mx 5 -my 5 -alphad 120`
- Figure 5b: `./generator -mode RP -mx 5 -my 5 -nx 3 -ny 3 -alphad 60`
- Figure 5c: `./generator -mode RRP -mx 5 -my 5 -seed 14`


## Polyhedral instances
- Figure 6b: `./generator -mode PL -HP 4 -VP 3 -mx 5 5 5 5 -my 3 3 3 3 -mz 3 3 3 -alphad 80 80 80 80`
- Figure 6c: `./generator -mode RPL -HP 4 -VP 3 -mx 5 5 5 5 -my 3 3 3 3 -mz 3 3 3 -alphad 80 80 80 80`


## Grid instances
- Figure 7a: `./generator -mode GP -mx 5 -my 5`
- Figure 7b: `./generator -mode GP -mx 5 -my 5 -nx 3 -ny 3`
- Figure 7c: `./generator -mode RGP -mx 5 -my 5 -seed 14`


## Cubic instances
- Figure 8a: `./generator -mode QP -mx 5 -my 3 -mz 3`
- Figure 8b: `./generator -mode RQP ./generator -mode QP -mx 5 -my 3 -mz 3`


## Pseudo-random instances
- Figure 11a: `./generator -mode PR2 -n 20 -nc 21 -maxc 18 -h 500 -w 500 -seed 14`
- Figure 11b: `./generator -mode PR3 -n 20 -nc 13 -seed 14`
