

%The program splits a single junciton up into xmax discrete sections.  Each
%section has a supercurrent density, and a phase difference that is
%contributed to by the field (PhaseF), and an arbitrary phase set at x=1
%called Phase1.  For each value of field and Phase1 the supercurrent at
%each point is calculated and the net supercurrent (sum along the junction)
%is calcualted to find the supercurrent carried at that phase and that
%field.  To find the maximum supercurrent for a given field, the max of
%that vector is taken, and can be plotted against the magnetic field.  This
%is the exact measurment of the critical current vs field that we do in the
%lab.  


%For Version 2, a geometrical loop was added so that the supercurrent vs
%field plot could include different amounts of phase shift for a determined
%portion of the junction.


%%

clear;
clc;
close all;
%% Defining the Parameters of the Simulaiton
xmax=101;
x(1,:)=(1:xmax);


%Geometrical Phase Loop Parameters
g=1;
gmax=4;
SCurrentDensity2Min=0;
SCurrentDensity2Max=.5;



%Flux Loop Parameters
f=1;
fmax=501;
FluxinJuncMin=-5;
FluxinJuncMax=5;


%Phase Loop parameters
p=1;
pmax=201;
Phase1Min=0*pi;
Phase1Max=4*pi;



%Pre Allocating memory to the arrays to decrease runtime
Phase1=zeros(1,pmax);
PhaseG=zeros(1,xmax);
SCurrentDensity2=zeros(xmax,gmax);
FluxinJunc=zeros(1,fmax);


SCurrentDensityNoise=(2*rand(1,xmax)-1);


SCurrent=zeros(xmax,pmax,fmax);
SCurrentNet=zeros(1,pmax);
MaxSCurrentNet=zeros(fmax,gmax);


%% Loops for running the simulation (Meat of the Simulation)

%Critical Current Harmonics
%Define the loop setp size, then run the for loop
SCurrentDensity2SS=(SCurrentDensity2Max-SCurrentDensity2Min)/(gmax-1);
for g=1:gmax
    
    %Defining the Super Current Densities for each component
    SCurrentDensity1=ones(1,xmax)+0.1*SCurrentDensityNoise-(g-1)*SCurrentDensity2SS;
    SCurrentDensity2=1-SCurrentDensity1;
    
    
    
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
            PhaseTotal=Phase1(p)+PhaseF;
            SCurrent=SCurrentDensity1.*sin(PhaseTotal)+SCurrentDensity2.*sin((PhaseTotal)/2);
            SCurrentNet(p)=sum(SCurrent)/xmax;


        end

        MaxSCurrentNet(f,g)=max(SCurrentNet);

    end

end

figure
plot(FluxinJunc,MaxSCurrentNet(:,:),'.')
xlabel('Flux Quanta in Junction');ylabel('Net Supercurrent');
title('Fraunhofer Pattern for different Harmonic Critical Currents');





    




