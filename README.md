# Kerbal Space Program Space Shuttle Ascent Guidance

My KSP ascent guidance script for the Space Shuttle which uses UPFG, adapted and modified to perform aborts.

# Remarks

This script was last tested in KSP 1.9 with a full RO install. If you use a different version of KSP or the mods there may be different configuration parameters that throw the whole thing off-balance.
In addition, my adaptations are very iffy, i.e. constructed from some idea that I had which happened to work by some miracle.
I provide these scripts as they are, with no guarantee that they'll work perfectly every time. **YMMV**

# Installation

**Required mods:**
- A complete install of RSS/Realism Overhaul with Ferram Aerospace Resarch. Confrmed to work with FAR 0.16.0.1, should now also work with 0.16.0.4
- Kerbal Operating System
- Space Shuttle System mod, SpaceODY's version seems to be the most up-to-date : https://github.com/SpaceODY/Space-Shuttle-System-Expanded
- RO configs that come with the Realism Overhaul package
- **[My Shuttle entry script](https://github.com/giuliodondi/kOS-ShuttleEntrySim) required by RTLS and TAL aborts. Grab the latest version from its repo**


You will find one folder: 
- **Script**

Put the contents of the Scripts folder inside Ship/Script so that kOS can see all the files.
In particular, there wil be only one script to run:
- **shuttle.ks** to launch the Shuttle from the launchpad (read on to learn about mission setup).


# Setup  

Please read this section carefully to understand how to configure your vessel in the VAB and how this is reflected in the config files.

## VAB setup

In the VAB, make sure the vessel staging is as follows (from the first stage onwards) :
- SSMEs
- SRBs and any launch clamps
- SRB decouplers and separation motors (both nosecone and skirt)
- External tank separation plus OMS engines and RCS actuators
- Anything in the payload bay
- Tail parachute

In addition, set Action Group 1 so that it activates the Fuel Cells . AG1 will be triggered right after liftoff so you may place anything else you might want here

### Don't forget to set the FAR control surface settings as required by the Entry script README.

Finally, right-click on the SSMEs and then open up the Real Fuels GUI. Make sure you are selecting an appropriate version of SSME (refer to [this Wikipedia table](https://en.wikipedia.org/wiki/RS-25#/media/File:SSME_Flight_History.png) if you want to select the version accurately). Make sure you select the same version for all three SSMEs. Whatever version you choose write down the following numbers:
- Vacuum Thrust in kN
- Vacuum ISP
- Mass flow as the sum of LH2 and LO2 usage in kg per second.
Write down the numbers for a single SSME and not the sum of all three.

## kOS configuration files

The mission parameters are specified in the main launch script is **shuttle.ks**. It contains variable definitions for:
- the name of the vessel config file to be used, it must exist in the UPFG_OPS1/VESSELS/ folder
- a "target_orbit" structure describing the shape of the desired orbit. Apoapsis, periapsis and cutoff altitude are standard for a Shuttle launch and shouldn't be changed. Only change the inclination to whatever you desire (read more about this later on).
- a TAL_site definition, which must match some landing site defined in the **landing_sites.ks** file in the Entry script source or the script will not run
- a variable to enable telemetry logging in the UPFG_OPS1/LOGS/ folder if you want to plot it with your own means

Vessel config files can be found under UPFG_OPS1/VESSELS/. There are several files that define the main Vehicle struct containing all the vessel information needed for the mission. 

As an example, open up **Discovery.ks** .  

The vehicle structure contains the vessel name, a "stages" key with parameters about the various vehicle configurations and a "SSME" key with data about the engine version you are using.  
You must first check that the engine data is correct (The files provided are configured for RS-25D) or update it if required. I advise to make a separate vessel configuration file for different engine variants.

A shuttle flight is a sequence of three stages: time-limited SRB phase (first stage), constant-thrust SSME phase until 3G acceleration is reached and a G-limited phase (both second stage).  
The initialisation function will build everything by itselv, provided that a few parameters are provided:
- "m_initial" for the first stage corresponds to the **fully-fueled vehicle with no payload or launch towers/clamps** 
- "m_final" for stage1 and "m_initial" for stage " are placeholder values which you can leave alone, the script will update those measurements
- "m_final" for stage 2 is the orbiter mass + RCS fuel + empty External Tank, with no payload
- "Tstage" for stage 1 is the trigger for SRB separation and should be adjusted so that the SRB are jettisoned when their TWR is below 1. IF the RO configs ever change, this value should also be updated.

Finally, the vessel config file contains an "events" definition, which is how we tell the script to make certain things happen during launch aside from staging events. An event is specified by time (in seconds MET), the type of event and some additional information depending on the type of event.
Most events are of the "action" type, where the action is specified as a piece of code within brackets {} in the event structure. For instance, there is an event to toggle AG1, modify the kOS steering gains and throttle the engines down/up during first stage. The **Discovery - RTLS.ks** and **Discovery - TAL.ks** vessel files provided also contain an event to trigger automatically an engine shutdown (more about aborts later on).

# Mission profiles

## Nominal launch

As mentioned, the mission is started by running **shuttle.ks**. Running this script is the only action required for a nominal launch (aborts are different). Simply running the script will program the launch a few seconds after running the script, meaning that the LAN of the target orbit depends on the time of day you choose to launch. It is possible to launch in the orbital plane of a ship in orbit by selecting it as a target in the map view **BEFORE** running the script. This will override the launch inclination to match and warp to the right time to launch so the LAN is correct.  
The script will guide the shuttle during first stage atmospheric flight, then use closed-loop PEG guidance for the second stage phase until MECO. 
After MECO the script wil automatically:
- trigger ET sep
- command an RCS vertical translation manoeuvre
- close the umbilical doors
- disable SSME gimballing

The script then enters an infinite loop displaying the results of an orbital analysis, calculating the erros with respect to the desired orbit. At this point you can halt the script with ctrl+C in the script window.
**Do not forget that the nominal ascent puts the shuttle on a trajectory that dips back into the atmosphere for ET disposal. You must perform manually an OMS bun to circularise.** 

## RTLS abort 

RTLS can be triggered automatically with an event contained in the vessel file or manually by shutting down an engine during flight. If the failure happens during stage 1 guidance will not kick in until after SRB separation, if the failure occurs during stage 2 it will be triggered immediately.  
The script performs automatically all three phases of RTLS:
- dissipation, flying outbound for a certain time to waste fuel. The script uses the PEG algorithm to estimate the right time to turn around.
- flyback, where the shuttle points back to the launch site and the outbound trajectory is slowly reversed to bring it home. The script uses PEG for guidance all throughout this phase. The target MECO conditions are 78 km altitude and variable distance from the launch site, at a velocity that depends on MECO distance. Throttling is used to match Time-To-Go with the time necessary to burn all propellant down to less than 2%.
- Glide-RTLS activated after MECO and separation, where the Shuttle pitches up to 40° and performs an aerobraking manoeuvre to stabilise the falling trajectory into a more nominal reentry trajectory
At the end of these phases the Shuttle will be at around 30km descending gently. The entry script will automatically be called and from there on you take over like a normal reentry. You will have to make sure that the landing site is the correct one, and engage steering control and guidance manually in the entry GUI.
During dissipation and flyback, the script will also burn the OMS engines to dump fuel. This is completely automatic.  

I tested early aborts (MET 80s) all the way to negative return (MET 225s). The earlier the abort the longer the fuel dissipation phase lasts and the further away the Shuttle will be when it finally starts to fly back. At Negative Return there is no fuel dissipation at all.  
The script is not super-precise about cutoff conditions so your results may vary depeding on when you trigger the abort.  
The Glide-RTLS phase is more iffy as there is no closed-loop guidance, just a control loop exeuting pre-programmed manoeuvres triggered by the vessel state. The 40° angle of attack should be no problem for the Shuttle, assuming the control surfaces are set as per the Entry script instructions. Do not shift the CG aft, as the script will deploy the body flap to stabilise pitch and I've seen that this induces yaw instability in this phase.  
The end conditions of Glide-RTLS depend a lot on the position and velocity at MECO. I programmed guidance to leave the shuttle with plenty of energy, but I've seen that sometimes the shuttle engages entry guidance **really** high on energy, like 30km and 1000 m/s at about 100km away. In this case I suggest to switch to approach mode earlier than usual (~60km from the target) and do a steep manual descent into thicker air.


## TAL abort

The TAL abort is triggered if an engine is shut down between MET 225s and MET 340s. Closed-loop Guidance will target the site specified in the **shuttle.ks** file and alter the PEG target state so that the trajectory falls within 600km crossrange of the landing site. Of course there is a limited ability to do so, hence the need to choose the TAL site along with the mission parameters.  
Apart from the internal targeting, the abort is carried out like a normal ascent, the only difference being an automatic OMS fuel dump. After MECO and separation the Shuttle will be at around 110km and about to descend. Stop the ascent script immediately and begin entry preparations. I chose not to do this automatically as you do have a small window to do small corrections using the Deorbit script. 
The Shuttle **usually** manages to steer the entry trajectory towards the landing site without issue.

## ATO/AOA aborts

Both aborts use the same guidance and differ only in what you decide to do after MECO. They are triggered if an engine is shut down between MET 340s and MET 420s. After that, the script will continue to regular MECO targets with only two engines.  
This abort mode lowers the cutoff altitude a bit and the apoapsis to about 160km, just outside of the upper atmosphere. Additionally it forces guidance not to thrust out of plane anymore, giving more performance margin at the cost of a MECO orbital inclination lower than desired.  
After MECO you will have the option to either circularise and carry out the mission in a lower orbit or do an OMS plane change burn to re-enter on the way down. USe the deorbit tool that comes with my entry script to help you with that.
