# Kerbal Space Program Space Shuttle Ascent Guidance

My KSP ascent guidance script for the Space Shuttle, intended to be used in KSP Realism Overhaul.
Uses Powered Explicit Guidance (also called UPFG) for vacum guidance, adapted and modified to perform intact aborts.

# Remarks

This script was last tested in 1.10 with a full RO install. It shoudl also work with KSP 1.9. If you use a different version of KSP or the mods there may be different configuration parameters that throw the whole thing off-balance.
In addition, some of my adaptations are somewhat iffy, i.e. constructed from some idea that I had which happened to work by some miracle.
I provide these scripts as they are, with no guarantee that they'll work perfectly every time. **YMMV**

# Installation

**Required mods:**
- A complete install of RSS/Realism Overhaul with Ferram Aerospace Resarch. 
- Kerbal Operating System
- Space Shuttle System mod, [Use my own fork of SpaceODY's Shuttle System so that the RO configs will be identical to what I use](https://github.com/giuliodondi/Space-Shuttle-System-Expanded). 
- **[My Shuttle entry script](https://github.com/giuliodondi/kOS-ShuttleEntrySim) required by RTLS and TAL aborts. Grab the latest version from its repo**

As is it does not work with SpaceODY's original fork (https://github.com/SpaceODY/Space-Shuttle-System-Expanded) for several reasons:
- That version offers the External Tank variants as separate VAB parts with different names, the script is only able to measure the ET propellants with the White ET, the other parts have a different internal name. My fork is immune from this problem as it uses B9 to catalog the variants.
- This script will soon have a revised OMS dump scheme for aborts which **may or may not** use a resource drain module I placed on the OMS engines parts in my fork. OMS dumping is necessary to bring the CG within limits or the Shuttle will lose pitch control during reentry.

You will find one folder: 
- **Script**

Put the contents of the Script folder inside Ship/Script so that kOS can see all the files.
In particular, you will only run one of two script files:
- **shuttle.ks** to launch the Shuttle from the launchpad according to specified mission parameters (read on to learn about mission setup).
- **shuttle3a.ks** is an identical script with special parameters for Polar orbit launches from Vandenberg


# Setup  

The script needs to know accurately the mass of orbiter + payload + ET + propellants for closed-loop guidance, without the mass of SRB or launch clamps. The script will measure everything automatically provided that the part tree is set up correctly in the VAB.  

Take care of the following things while building the Shuttle Stack in the VAB:
- The root part must be one of the orbiter parts (the cabin is fine)
- The ET must be a child part of some orbiter part (for the Space Shuttle System mod it's attached to the cargo bay by default)
- The SRB decouplers must be attached to the External Tank, so that all SRB-related parts are children of the ET
- Any launch clamps/towers must be attached either to the ET or the SRBs, don't attach anything to the Orbiter

Make sure the vessel staging is as follows (from the first stage onwards) :
- SSMEs
- SRBs and any launch clamps
- SRB decouplers and separation motors (both nosecone and skirt)
- External tank separation plus OMS engines and RCS actuators
- Anything in the payload bay
- Tail parachute


### Don't forget to set the FAR control surface settings as required by the Entry script README.

Finally, right-click on the SSMEs and then open up the Real Fuels GUI. Make sure you are selecting an appropriate version of SSME (refer to [this Wikipedia table](https://en.wikipedia.org/wiki/RS-25#/media/File:SSME_Flight_History.png) if you want to select the version accurately).  
**Make sure you select the same version for all three SSMEs.**

IF you don't use SpaceShuttleSystem parts, or if you mismatch SSME versions, the script should detect this and break during vehicle initialisation. This is intended and meant to signal to you that something is wrong.
 

## kOS configuration file

The mission parameters are specified in the main launch script **shuttle.ks** or **shuttle3a.ks**.  
It contains variable definitions for:
- a "target_orbit" structure describing the shape of the desired orbit. Apoapsis, periapsis and cutoff altitude are standard for a Shuttle launch and shouldn't be changed. Only change the inclination to whatever you desire (read more about this later on).
- a disabled variable *engine_failure_time* which you can uncomment to trigger an automatic engine failure at the specified time. More on aborts later on.
- a disabled variable TAL_site which you can uncomment to select a specific TAL landing site (the selection is automatic by default). The value must match the name of some landing site defined in the **Shuttle_entrysim/landing_sites.ks** file in the Entry script source or the script will crash.
- a variable to enable telemetry logging in the UPFG_OPS1/LOGS/ folder if you want to plot it with your own means

There is no longer any need for a specific vessel configuration file as the script now measures both vehicle mass and SSME parameters automatically. **The script assumes that the Shuttle stack has been assembled properly in the VAB, else it might fail to measure things accurately.**

# Mission profiles

## Nominal launch

As mentioned, the mission is started by running **shuttle.ks**. Running this script is the only action required for a nominal launch (aborts are different). Simply running the script will program the launch a few seconds after running the script, meaning that the LAN of the target orbit depends on the time of day you choose to launch. It is possible to launch in the orbital plane of a ship in orbit by selecting it as a target in the map view **BEFORE** running the script. This will override the launch inclination to match and warp to the right time to launch so the LAN is correct.  
Fuel cells are automatically activated at liftoff. On a nominal mission a roll to heads-up attitude is performed at T+5:50.  

Although the Shuttle was a two-stage vehicle, the script treats it as a four-stage vehicle:
- stage 1 is the SRB atmospheric phase, with open-loop guidance. It terminates 5 seconds after SRB sep.
- stage 2 is closed-loop PEG guidance with the engines at full constant throttle. It terminates when the acceleration reaches 3G
- stage 3 is closed-loop PEG guidance with the engines throttling back continuously to maintain aroung 3G acceleration. It terminates either at MECO or when the minimum throttle setting is reached. For missions with very heavy payloads this might be the last phase overall, as fuel will be depleted before the minimum throttle setting is reached.
- stage 4 s closed-loop PEG guidance with the engines at minimum throttle. It terminates at MECO or fuel depletion. This phase is only ever entered for missions out of Vandenberg because of the extra deltaV required by the retrograde launch. 

After MECO the script wil automatically:
- trigger ET sep
- command an RCS vertical translation manoeuvre
- close the umbilical doors
- disable SSME gimballing

The script then enters an infinite loop displaying the results of an orbital analysis, calculating the erros with respect to the desired orbit. At this point you can halt the script with ctrl+C in the script window.
**Do not forget that the nominal ascent puts the shuttle on a trajectory that dips back into the atmosphere for ET disposal. You must perform manually an OMS bun to circularise.** 

## Aborts

### General considerations

Aborts can be triggered by uncommenting the *engine_failure_time* variable in the main **shuttle.ks** script. The time specified will trigger a different abort mode, each with its own guidance and targeting scheme. Alternatively a failure can be triggered manually by shutting down an engine mid-flight, the program is able to detect both situations.  
All abort scenarios discard the SSME throttling stages and keep the throttle at maximum until MECO or fuel depletion, except for RTLS which uses throttling for guidance.

Bear in mind that only **intact aborts** are covered right now. A double engine failure is a Contingency scenario and the script will not attempt to handle them.

**Caveat:**
The Shuttle has its engines pointed away from the main vehicle axis and as such there is coupling between yaw and roll. This script uses the kOS built-in steering manager which is unaware of this coupling and thus struggles at times.  Oscillations are expected and should be fine, unless they are too severe.


## RTLS abort 

### Only works for Shuttles using RS-25D variant at the moment. I haven't yet pintpointed which parameter must be adjusted based on engine thrust

RTLs is triggered if an engine fails between liftoff and MET 225 seconds. The actual abort guidance is not activated until the vehicle enters second stage, if the engine goes dow nafter SRB sep, guidance is activated immediately.  
The script performs automatically all three phases of RTLS:
- dissipation, flying outbound for a certain time to waste fuel. The script uses the PEG algorithm to estimate the right time to turn around.
- flyback, where the shuttle points back to the launch site and the outbound trajectory is slowly reversed to bring it home. The script uses PEG for guidance all throughout this phase. If the initial trajectory entails a large off-plane component to bring the Shuttle back to the target site, PEG will steer sideways, this is normal and reliable as long as the algorithm is converged. 
The target MECO conditions are 78 km altitude and variable distance from the launch site, at a velocity that depends on MECO distance. Throttling is used to match Time-To-Go with the time necessary to burn all propellant down to less than 2%. Throttling is disabled 60 seconds before MECO as it is a bit unstable, thus the 2% constraint might actually be violated in some cases, but not by much.
- Glide-RTLS activated after MECO and separation, where the Shuttle pitches up to 40° and performs an aerobraking manoeuvre to stabilise the falling trajectory into a more nominal reentry trajectory.  

At the end of these phases the Shuttle will be at around 30km descending gently. The entry script will automatically be called and from there on you take over like a normal reentry. You will have to make sure that the landing site is the correct one, and engage steering control and guidance manually in the entry GUI.
During dissipation and flyback, the script will also burn the OMS engines to dump fuel. This is completely automatic.  

I tested early aborts (MET 80s) all the way to negative return (MET 225s). The earlier the abort the longer the fuel dissipation phase lasts and the further away the Shuttle will be when it finally starts to fly back. At Negative Return there is no fuel dissipation at all.  
The script is not super-precise about cutoff conditions so your results may vary depeding on when you trigger the abort.  
The Glide-RTLS phase is more iffy as there is no closed-loop guidance, just a control loop exeuting pre-programmed manoeuvres triggered by the vessel state. The 40° angle of attack should be no problem for the Shuttle, assuming the control surfaces are set as per the Entry script instructions. Do not shift the CG aft, as the script will deploy the body flap to stabilise pitch and I've seen that this induces yaw instability in this phase.  
The end conditions of Glide-RTLS depend a lot on the position and velocity at MECO. I programmed guidance to leave the shuttle with plenty of energy, but I've seen that sometimes the shuttle engages entry guidance **really** high on energy, like 30km and 1000 m/s at about 100km away. In this case I suggest to switch to approach mode earlier than usual (~60km from the target) and do a steep manual descent into thicker air.


## TAL abort

The TAL abort is triggered if an engine is shut down between MET 225s and MET 340s. The TAL site is selected automatically from the landing sites defined in **Shuttle_entrysim/landing_sites.ks** based on whether they lie downrange and estimating if there is enough delta-V to alter the trajectory within 600km crossrange of them. One site is chosen at random out of all the ones satisfying these conditions, to simulate weather availability. The site choice can be overridden by specifying the site name in the **shuttle.ks** file.  

Once the site is selected, Closed-loop Guidance will alter the PEG target state so that the trajectory falls within 600km crossrange of the landing site. The later the TAL abort, the faster the Shuttle is already and the more deltaV it takes to curve the trajectory.  
Apart from the internal targeting, the abort is carried out like a normal ascent, the only difference being an automatic OMS fuel dump. After MECO and separation the Shuttle will be at around 110km and about to descend. Stop the ascent script immediately and begin entry preparations. I chose not to do this automatically as you do have a small window to do small corrections using the Deorbit script. 
The Shuttle **usually** manages to steer the entry trajectory towards the landing site without issue.

## ATO/AOA aborts

Both aborts use the same guidance and differ only in what you decide to do after MECO. They are triggered if an engine is shut down between MET 340s and MET 420s.  
This abort mode lowers the cutoff altitude a bit and the apoapsis to about 160km, just outside of the upper atmosphere. Additionally it forces guidance not to thrust out of plane anymore, giving more performance margin at the cost of a MECO orbital inclination lower than desired. Also no OMs dump is performed as you will need the fuel to do orbital corrections later on.  

After MECO you will have the option to either circularise and carry out the mission in a lower orbit or do an OMS plane change burn to re-enter on the way down. USe the deorbit tool that comes with my entry script to help you with that. 
The SpaceShuttleSystem has less crossrange capability than in real life, for AOA aborts out of KSC I've only been able to reenter back at the Cape for launches to an inclination of 40° or less. For ISS launches (52° inclination) Northrup strip (White Sands) is the preferred AoA landing site. For launches out of Vandenberg, SpaceShuttleSystem sadly does not have the crossrange to make it back to any site in the continental US

## Post-ATO engine failure 

After the ATO boundary, the program will assume that it's able to achieve the nominal MECO targets. The only action here is to throttle the remaining engines up to 100%.
