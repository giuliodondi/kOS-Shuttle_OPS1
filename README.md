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
- **shuttle.ks** to launch the Shuttle from the launchpad


# Setup  

Please read this section carefully to understand how to configure your vessel in the VAB and how this is reflected in the config files.

There are two kinds of configuration files: Vessel and Launch.  
Vessel config files can be found under UPFG_OPS1/VESSELS/. There are various files that define the main Vehicle struct containign all the vessel information, one different file for the different orbiter mass and engine versions.

As an example, open up **Discovery.ks** .  
First there are several control settings that should never be touched.  
The interesting bit is under the "stages" key, this is where the logical stages are defined. A Stage is a phase of flight that begins with some event and ends with another event, during which a particular set of engines burns. Under this definition the Shuttle is a two-stage vehicle, where the second stage will be in control of closed loop guidance.  
Stages have an initial mass "m_initial" and a final mass "m_final" parameter. **These should be measured in the VAB with a fully-fueled vehicle with no payload or launch towers/clamps**. The final mass of the first stage is the mass of the entire vehicle with empty SRBs and ~2 minutes of SSME fuel missing from the ET. The initial mass of the second stage is this value less the empty mass of the SRBs. These values are not critical since the staging action is not triggered by mass.  
The final mass of the second stage is the orbiter + empty ET. **This value is critical**  
Immediately after liftoff, the script will measure the vehicle mass, infer if there is any mass unaccounted for (payload) and add it to these values. This is why it's critical to measure the empry vehicle accurately.  
**The "staging" struct should never be touched or the vehicle will not behave correctly.** The only parameter that should ever be adjusted is "Tstage" for stage 1, this is related to the SRB burn time in seconds and should match exactly the moment the SRBs no longer produce any useful thrust, right before burnout. If you see the SRBs zooming forward after staging then the script is jettisoning them too early and this value should be increased by a second or two.  
The "engines" struct contains information on the SSMEs, namely vacuum thrust, vacuum ISP and LH2+LOX mass flow which can all be queried in the VAB using RealFuels. **Ensure that you select the appropriate RS-25 version in the VAB and please check that the thrust, ISP and mass flow are correct**

# TO BE CONTINUED - make changes to the way engines are initialised since there is a redundant "engines" and "SSME" set of keys





