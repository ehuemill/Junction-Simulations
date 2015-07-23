%Explanation of base program


%The program splits a single junciton up into xmax discrete sections.  Each
%section has a supercurrent density, and a phase difference that is
%contributed to by the field (PhaseF), and an arbitrary phase set at x=1
%called Phase1.  For each value of field and Phase1 the supercurrent at
%each point is calculated and the net supercurrent (sum along the junction)
%is calcualted to find the supercurrent carried at that phase and that
%field.  To find the maximum supercurrent for a given field, the max of
%that matrix is found for each field.  This is plotted against the magnetic
%field and shows the fraunhofer pattern.  It is essentially what nature
%does naturally in the Josephson interferometry experiments we do.  

%This is also the starting point for many other variations that can be put
%on top.  



%% Clearing memory and input screen.

clear;
clc;
close all;
%% Defining the Parameters of the Simulaiton
xmax=11;
x(1,:)=(1:xmax);

%Flux Loop Parameters
f=1;
fmax=151;
FluxinJuncMin=-3;
FluxinJuncMax=3;


%Phase Loop parameters
p=1;
pmax=101;
Phase1Min=0*pi;
Phase1Max=2*pi;


%Pre Allocating memory to the arrays to decrease runtime
Phase1=zeros(1,pmax);
FluxinJunc=zeros(1,fmax);

SCurrentDensityNoise=(2*rand(1,xmax)-1);
SCurrentDensity=ones(1,xmax)+0.01*SCurrentDensityNoise;


SCurrent=zeros(xmax,pmax,fmax);
SCurrentNet=zeros(1,pmax);
MaxSCurrentNet=zeros(1,fmax);

%% Loops for running the simulation Meat of the Simulation



%Field Contribution to the Phase 
%Define the loop setp size, then run the for loop
FluxinJuncSS=(FluxinJuncMax-FluxinJuncMin)/(fmax-1);
for f=1:fmax

    FluxinJunc(f)=FluxinJuncMin+(f-1)*FluxinJuncSS;
    PhaseF=2*pi*x./xmax*FluxinJunc(f);

    %Phase1 Loop of externally set phase in 
    %Define the loop setp size, then run the for loop
    Phase1SS=(Phase1Max-Phase1Min)/(pmax-1);
    for p=1:pmax

        Phase1(p)=Phase1Min+(p-1)*Phase1SS;

        SCurrent=SCurrentDensity.*sin(PhaseF+Phase1(p));
        SCurrentNet(p)=sum(SCurrent)/xmax;


    end

    MaxSCurrentNet(f)=max(SCurrentNet);

end


figure
plot(FluxinJunc,MaxSCurrentNet)




    




