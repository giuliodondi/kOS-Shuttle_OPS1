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
- DECQ's Space Shuttle System mod, this fork seems to be the most up-to-date : https://github.com/DylanSemrau/Space-Shuttle-System
- RO configs that come with the Realism Overhaul package
- **My Shuttle entry script https://github.com/giuliodondi/kOS-ShuttleEntrySim required by RTLS and TAL aborts**

**CAVEAT** 
I'm aware that there is a more up-to-date Shuttle version made by SpaceODY, I never tested my scripts with his version, there may be different part configurations so, once again, **YMMV**


You will find one folder: 
- **Scripts**

Put the contents of the Scripts folder inside Ship/Scripts so that kOS can see all the files.
In particular, there wil be only one script to run:
- **shuttle.ks** to launch the Shuttle from the launchpad (read on to learn about mission setup).


# Setup  

Please read this section carefully to understand how to configure your vessel in the VAB and how this is reflected in the config files.

In the VAB, make sure the vessel staging is as follows (from the first stage onwards) :
- SSMEs
- SRBs and any launch clamps
- SRB decouplers and separation motors (both nosecone and skirt)
- External tank
- OMS and RCS actuators
- anything in the payload bay

In addition, set Action Group 1 so that it activates the Fuel Cells . AG1 will be triggered right after liftoff so you may place anything else you want to happen automatically during ascent here

Finally, don't forget to set the FAR control surface settings as required by the Entry script README.


There are two kinds of configuration files: Vessel and Launch.  

The main launch script is **shuttle.ks**, this is what you will run to launch the shuttle and it's the only action needed for a nominal ascent (aborts are different). It contains variable definitions for:
- the name of the vessel config file to be used, it must exist in the UPFG_OPS1/VESSELS/ folder
- a "target_orbit" structure describing the shape of the desired orbit. Apoapsis, periapsis and cutoff altitude are standard for a Shuttle launch and shouldn't be changed. Only change the inclination to whatever you desire (read more about this later on).
- a TAL_site definition, which must match some landing site defined in the **landing_sites.ks** file in the Entry script source or the script will not run
- a variable to enable telemetry logging in the UPFG_OPS1/LOGS/ folder if you want to plot it with your own means

Vessel config files can be found under UPFG_OPS1/VESSELS/. There are several files that define the main Vehicle struct containing all the vessel information needed for the mission. 

As an example, open up **Discovery.ks** .  

The vehicle structure contains the vessel name, a "stages" key with parameters about the various vehicle configurations and a "SSME" key with data about the engine version you are using.  
You must first check that the engine data is correct (The files provided are configured for RS-25D) or update it if required. You can read engine data using the RealFuels GUI in the VAB. You shoudl look for vacuum ISp, vaccum Thrust in kN, mass flow as the sum of LH2 and LOx in kg.  
I advise to make a separate vessel configuration file for different engine variants.

A shuttle flight is a sequence of three stages: time-limited SRB phase (first stage), constant-thrust SSME phase until 3G acceleration is reached and a G-limited phase (both second stage).  
The initialisation function will build everything by itselv, provided that a few parameters are provided:
- "m_initial" for the first stage corresponds to the **fully-fueled vehicle with no payload or launch towers/clamps** 
- "m_final" for stage1 and "m_initial" for stage " are placeholder values which you can leave alone, the script will update those measurements
- "Tstage" is the trigger for SRB separation and should be adjusted so that the SRB are jettisoned when their TWR is below 1. IF the RO configs ever change, this value should also be updated.

Finally, the vessel config file contains an "events" definition, which is how we tell the script to make certain things happen during launch aside from staging events. An event is specified by time (in seconds MET), the type of event and some additional information depending on the type of event.
Most events are of the "action" type, where the action is specified as a piece of code within brackets {} in the event structure. For instance, there is an event to toggle AG1, modify the kOS steering gains and throttle the engines down/up during first stage. The **Discovery - RTLS.ks** and **Discovery - TAL.ks** vessel files provided also contain an event to trigger automatically an engine shutdown (more about aborts later on).

# Mission profiles

## Nominal launch

As mentioned, the mission is started by running **shuttle.ks**. In the nominal case this will program the launch a few seconds after runnign the script. It is possiblre to launch in the orbital plane of a ship in orbit by selecting it as a target in the map view **BEFORE** running the script. This will override the launch inclination to match and warp to the right time to launch.  
The script will guide the shuttle during first stage atmospheric flight, then use closed-loop PEG guidance for the second stage phase until MECO. 
After MECO the script wil automatically:
- trigger ET sep
- command an RCS vertical translation manoeuvre
- close the umbilical doors
- disable SSME gimballing

The script then enters an infinite loop displaying the results of an orbital analysis, calculating the erros with respect to the desired orbit. At this point you can halt the script with ctrl+C in the script window.
**Do not forget that the nominal ascent puts the shuttle on a trajectory that dips back into the atmosphere for ET disposal. You must perform mamually an OMS bun to circularise.** 

## RTLs abort

RTLS can be triggered automatically with an event contained in the vessel file or manually by shutting down an engine during flight. If the failure happens during stage 1 guidance will not kick in until after SRB separation, if the failure occurs during stage 2 it will be triggered immediately.  
I tested early aborts (MET 80s) all the way to 

The last RTLS opportunity is at MET 220s (3m 40s), which is late enough into the flight that the fuel dissipation phase shouldn't even be performed. 






