# Kerbal Space Program Space Shuttle Ascent Guidance
## Updated January 2024

My KSP ascent guidance script for the Space Shuttle, intended to be used in KSP Realism Overhaul.
Uses Powered Explicit Guidance (also called UPFG) for vacum guidance, adapted and modified to perform intact aborts.

# Remarks

This script was last tested in 1.12.3 with a full RO install. The algorithms are adapted from the real-life Space Shuttle GN&C document [OI-34 SPACE SHUTTLE ORBITER OPERATIONAL LEVEL C FUNCTIONAL SUBSYSTEM SOFTWARE REQUIREMENTS GUIDANCE, NAVIGATION AND CONTROL PART A](https://www.ibiblio.org/apollo/Shuttle/STS83-0002-34%20-%20Guidance%20Ascent-RTLS.pdf). In real life the alghorithms would probably be tailored to the specific mission by means of I-loaded constants, while I tried to make everything work for generic cases. As such, sometimes behaviour can be a bit iffy, YMMV.

# Installation

**Required mods:**
- A complete install of RSS/Realism Overhaul with Ferram Aerospace Resarch. 
- kOS version 1.3 at least
- kOS Ferram, now available on CKAN
- [My own fork of SpaceODY's Space Shuttle System](https://github.com/giuliodondi/Space-Shuttle-System-Expanded). 
  - if you use the latest version you will be required to also grab my Ferram Fork to use the custom aerodynamics module. Refer to the README
- **[My Shuttle entry script](https://github.com/giuliodondi/kOS-ShuttleEntrySim) required by RTLS and TAL aborts. Grab the latest version from its repo**

**Not compatible with SpaceODY's original fork or any other Shuttle mod.**

You will find one folder: 
- **Script**

Put the contents of the Script folder inside Ship/Script so that kOS can see all the files.
In particular, you will only run one of two script files:
- **shuttle.ks** to launch the Shuttle from the launchpad according to specified mission parameters (read on to learn about mission setup).
- **shuttle3a.ks** is an identical script with special parameters for Polar orbit launches from Vandenberg

There is also a **node.ks** script, this is a little manoeuvre node executor that I use to execute OMS burns in orbit, it takes care of the offset OMS thrust position (so it even works with a single OMS burn) and is compatible with nodes created with Principia


# Setup  

The script needs to know accurately the mass of orbiter + payload + ET + propellants for closed-loop guidance, without the mass of SRB or launch clamps. The script will measure everything automatically provided that the part tree is set up correctly in the VAB.  

Take care of the following things while building the Shuttle Stack in the VAB:
- ~~The root part must be one of the orbiter parts (the cabin is fine)~~ IF you use my latest SpaceShuttleSystem version, there is a single Orbiter assembled part that I advise you use instead, this is your root part.
- The ET must be a child part of some orbiter part (for the Space Shuttle System mod it's attached to the cargo bay by default)
- The SRB decouplers must be attached to the External Tank, so that all SRB-related parts are children of the ET
- Any launch clamps/towers must be attached either to the ET or the SRBs, don't attach anything to the Orbiter

Make sure the vessel staging is as follows (from the bottom stage upwards) :
- SSMEs and RCS staging toggles (required for aborts and MECO attiude control)
- SRBs and any launch clamps / platform
- SRB decouplers and separation motors (both nosecone and skirt)
- External tank separation and OMS engines
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

The script now implements a functional GUI which re-creates the real-world Space Shuttle monitor displays during ascent. Although the GUI is fully functional, so far it is just a fancy display since there is no possibility of manual control.

### ASCENT TRAJ 1 display

![ascent_traj_1_gui](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/ascent_traj1_gui.png)

This is the display during the majority of first stage, until right before SRB separation.
- At the top you have the display title and the running mission elapsed time
- On the left data box you have vertical speed (H-dot) and the roll (R), pitch (P) and yaw (Y) angles with respect to the surface prograde direction
- On the right data box you have:
    - a slider indicating the current acceleration in units of G
    - the remaining propellant quantity (PROP) as a percentage
    - the current throttle setting (THR) as a percentage of Rated Power Level (more on this later)
    - TGO and VGO are yellow and inactive during first stage
- In the middle is a plot of altitude on the vertical vs. surface velocity on the horizontal. The solid represents a nominal ascent trajectory. The numbered ticks indicate roughly the surface pitch that the Shuttle should have at that moment
- The dashed line is the first-stage ascent in case of an engine failure at liftoff. Because of lower acceleration and trajectory biasing done by Guidance, the hsuttle climbs faster than it accelerates comapred to nominal
- The yellow triangle indicates the Shuttle's state right now
- The circle is the predicted state 30 seconds into the future, the prediction is a trajectory integration assuming the thrust direction is invariant and doesn't accoutn for the fact that the Shuttle is constantly pitching down
- at the bottom you have a message box detailing the status of the underlying script

### ASCENT TRAJ 2 display

![ascent_traj_2_gui](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/ascent_traj2_gui.png)

This is the display from the final moments of first stage all the way to MECO, during nominal ascent, TAL and ATO aborts (RTLS has its own display).

- The left data box is identical to the TRAJ1 display
- the right data box is also identical to the TRAJ1 display, with the exception of the TGO and VGO fields which are now active:
    - TGO is the guidance calculated time-to-go until the MECO target is reached
    - VGO is the guidance calculated velocity-to-go in m/s until the MECO target is reached
- both fields will be yellow when the guidance algorithm is unconverged, then turn green once the algorithm stabilises
- At the top, below the title, you have the MECO velocity indicator. It's a slider which ranges from 7000 to 8000 m/s and the CO symbol indicates the desired cutoff speed. The triangle indicates the current orbital velocity and should stop at the CO mark at MECO.
- In the middle is now a plot of altitude on the vertical vs. orbital velocity on the horizontal. The long central curve is the nominal trajectory, which droops during the late stages of ascent (this is normal and realistic).
    - The dashed track to the left of the nominal trajectory is the trajectory for a retrograde launch out of Vandenberg. In that case the Shuttle moves against the Earth's rotation and so orbital velocity is lower at the start
- The ticks along the trajectory plot represent abort boundaries:
    - TAL indicates the last TAL opportunity
    - ATO indicates the earliest ATO opportunity, although ATO is currently only available after TAL
    - RTLS indicates the last RTLS opportunity. The RTLS boundary is surface velocity, not orbital. For retrograde launches it's reached at a much lower orbital velocity, hence why it's a double tick.

### Meaning of Rated Power Level and the THR indicator

The Space Shuttle Main Engine had several performance improvements in its operational life, and you have all the variants at your disposal in KSP RO.  
The caveat is that "full power" is an ambiguous term in this situation. For this reason we define the Rated Power Level (100% RPL) as 2090 kN, the max thrust of the original variant. 
This is significant because for the later part of the Shuttle program, when they used the RS-25D variant, the nominal throttle setting was 104% RPL and boosted to the full 109% RPL in case of an abort. This was done to reduce wear and risk of failure on the engines, it's all realistic and simulated by the script.

After MECO the script will automatically:
- trigger ET sep
- command an RCS vertical translation manoeuvre
- close the umbilical doors
- disable SSME gimballing

The script then enters an infinite loop displaying the results of an orbital analysis, calculating the erros with respect to the desired orbit. At this point you can halt the script with ctrl+C in the script window.
**Do not forget that the nominal ascent puts the shuttle on a trajectory that dips back into the atmosphere for ET disposal. You must perform manually an OMS bun to circularise.** 

## Aborts

### General considerations

Aborts can be triggered by uncommenting the *engine_failure_time* variable in the main **shuttle.ks** script. The time specified will trigger a different abort mode, each with its own guidance and targeting scheme. Alternatively a failure can be triggered manually by shutting down an engine mid-flight, the program is able to detect both situations.  
The boundaries between abort scenarios depend on velocity, so they may be reached at significantly different METs depending on the SSME condiguration and the payload.  
All abort scenarios discard the SSME throttling stages and keep the throttle at maximum until MECO or fuel depletion, except for RTLS which uses throttling for guidance.

Bear in mind that only **intact aborts** are covered right now. A double engine failure is a Contingency scenario and the script will not attempt to handle them.

**Caveat:**
The Shuttle has its engines pointed away from the main vehicle axis and as such there is coupling between yaw and roll. This script uses the kOS built-in steering manager which is unaware of this coupling and thus struggles at times.  Oscillations are expected and should be fine, unless they are too severe.


## Return to Launch Site (RTLS) abort 

RTLS is triggered if an engine fails before a certain surface-relative velocity is reached. The actual speed varies between 2600 and 2200 m/s depending on how large the desired orbital inclination is. The boundary is called **negative return** and a TAL abort is commanded after that. This abort scenario is quite involved and has a powered phase (until MECO) and a Glide phase after that.

Powered-RTLS guidance aims to bring the Shuttle to the following conditions at MECO:
- Altitude about 72 km
- Moving towards the launch site with velocity that depends on distance at MECO
- Less than 2% propellant remaining, no more than 20 s of burn time on two engines

There are several phases to Powered-RTLS abort:
- **Lofting** if the engine is lost before SRB sep, to try to achieve close to nominal altitude and vertical speed. Actual active RTLS guidance kicks in after SRB sep
- **Dissipation**, flying outbound for a certain time to waste fuel. Guidance is open-loop, limited to pitching up by an angle that depends the surface velocity at engine failure (or SRB sep).  
The script uses the PEG algorithm as a predictor to estimate the fuel needed to reverse course, when this matches the fuel remaining minus 2%, the dissipation phase ends. If RTLS is triggered very close to the negative return boundary, this step is skipped.
Automatic OMS dump is initiated during fuel dissipation.
- **Flyback**, where the shuttle pitches around towards the launch site and the outbound trajectory is slowly reversed to bring it home. The script uses a modified version of PEG for guidance. Since velocity-to-go and time-to-go need to match the 2% propellant constraint at MECO, as well as the constraint between position and velocity, active throttling is used as an extra degree-of-freedom. Throttling is disabled some time seconds before MECO as it can become unstable. 
The OMS fuel dump will cease before or at MECO during flyback.

After MECO and separation, the **Glide-RTLS (GRTLS)** guidance is activated.  
The Shuttle pitches up to 40° as it performs an aerobraking manoeuvre to stabilise the falling trajectory into a more nominal reentry trajectory, controlling vertical G forces.  At the end of GRTLS the Shuttle will be about 200-250 km from the launch site, 35/40km altitude at about Mach 4, in a gentle descent. The entry script will automatically be called and from there on you take over like a normal reentry. You will have to make sure that the landing site is the correct one, and engage steering control and guidance manually in the entry GUI.

### RTLS TRAJ 2 display

![rtls_traj_2_gui](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/rtls_traj2_gui.png)

This display is rendered when the RTLS abort is initialised (not when the engine is lost) and lasts until after MECO.

- At the top you have another cutoff velocity indicator, this time in terms of surface velocity. The range is from 1500 to 2500 m/s, and since the goal is to turn back higher cutoff speeds are indicated from right to left.
    - While the CO mark is actively placed in the right place, you should expect the actual speed at cutoff to be off this mark by a little.
- The right data box only contains the G indicator
- the left data box contains:
    - vertical speed (H-dot)
    - propellant left (PROP)
    - the current RPL throttle value (THR)
    - Time- and velocity-to-go (TGO, VGO)
    - Again, both TGO and VGO are yellow until guidance converges to a good solution. This is true for both dissipation and flyback phases
    - The desired burnout delta-time (T_C), this is the main indicator of a good guidance state, more on this later
- the central plot is a little complicated. It represents altitude on the vertical versus the horizontal component of downrange velocity from the launch site. This means there are positive velocity (i.e. travelling away) and negative velocity (travelling back) regions of the plot in the horizontal direction.
    - The Shuttle bug should move within the curved lines, first it will be in the lower right region, moving up and further right during fuel dissipation. During flyback it will start moving to the left as the Shuttle decelerates, and also down as it loses altitude. It will cross the "0" line when the Shuttle starts moving back to the launch site, then it will keep moving left and gradually start climbing once again.
    - the bottom-right curve is the nominal ascent trajectory, the Shuttle shouldn't be to the right of this
    - the top slanted curve is the maximum lofted trajectory during flyback for aborts close to the Negative Return boundary
    - The bottom segment is the depressed trajectory for a liftoff engien failure, the Shuttle is slow and heavy in this case
    - The left-most horizontal line with "CO" indicates the cutoff line at 72km altitude, the Shuttle should be close to this line at MECO

### The meaning of T_C

T_C is just a name for the difference between the time to burn down to 2% propellant minus the time-to-go coming from the PEG algorithm, it's the main figure that dictates when we should stop dissipating fuel and begin flying back
During the dissipation phase this will be positive, because TGO assumes we turn back immediately but we still have too much propellant. Actually, if the engine failure is before SRB sep and the algorithm is started when we're still close to the launch site, there might not be a good solution to find and T_C will have some nonsense values. TGO and VGO will be yellow until we're safely far enough away and T_C is stable, only then the program will accept its value to check if we should begin flyback.  
Once we trigger Flyback, T_C will settle around zero as the PEG algorithm adjusts the throttle so that we burn propellant all the way to 2% during the rest of the manoeuvre. 


### Results from my RTLS tests

Here I present my test results for RTLS aborts. The scenario was STS-9 (RS-25A, Lightweight tank, Spacelab payload at 57° inclination). The only variable is the time of engine failure _t_fail_ which ranges from liftoff to just before Negative Return. The plots include both Powered and GLide RTLS phases 

![rtls_trajplot](https://github.com/giuliodondi/kOS-UPFG_OPS1/blob/master/Ships/Script/Shuttle_OPS1/rtls_traj.png)

The first plot is altitude vs. downrange distance, and shows the general shape of the trajectory. You can see:
- engine failures before SRB sep result in lofted trajectories, as commanded
- The maximum altitude during the manoeuvre is geenrally higher the later the engine failure
- all trajectories target the same altitude at MECO, it's the "hump" at 72km
- The downrange distance at MECO is different for all trajectories

![](https://github.com/giuliodondi/kOS-UPFG_OPS1/blob/master/Ships/Script/Shuttle_OPS1/rtls_vel.png)

The second plot is horizontal velocity vs. altitude. Here you can see clearly:
- when the engine was lost (the point of deviaton from the dashed nominal line)
- when flyback was triggered (the point where the lines "bounce to the left"
- that the Shuttle is already on its way down when flyback is triggered, in the case of early aborts
- that the Shuttle does not come to a dead stop at the inversion point, as there is some sideways motion
- That, just like downrange distance, all velocities at MECO are different

![](https://github.com/giuliodondi/kOS-UPFG_OPS1/blob/master/Ships/Script/Shuttle_OPS1/rtls_rvline.png)

The third plot shows downrange distance vs. velocity, centered around the point of MECO. All the lines terminate close to the Range-Velocity (RV) line, which is the equation that Guidance uses to target MECO. Where exactly you land on the RV line is not important as long as the script cuts off the engines on it, which you can see as the velocity becomes almost constant (before increasing again during the Glide-RTLS descent).  
Curiously, very early and very late aborts produce trajectories that arrive closer to the site than mid aborts, this trend can also be seen in the farthest distance reached during flyback (see plot 1). Early aborts have low speed and acceleration and so the Shuttle cannot travel very far before flyback, while late aborts have the highest acceleration.


![](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/rtls_throt.png)

The final plot shows the throttle setting in terms of RPL percentage. The thottle is kept at maximum during fuel dissipation, and is actively adjusted during the flyback phase. You can see:
- Max-Q throttle-down is done for all cases except a liftoff engine failure
- MECO is where the lines end, right before it there is a period of minimum throttle (67% RPL), this is Terminal Guidance
- The earlier the engine failure, the more fuel we need to dissipate, which results in a later pitcharound and flyback and a later MECO
- All plots show a flatlining right before Terminal Guidance. This indicates when active throttling is deactivated during active PEG guidance because it's too sensitive
- During flyback, throttle initially settles around 98-99% RPL. This is a canned value given to the PEG algorithm during dissipation to determine the flyback moment with margin for error
- An exception to this is the very latest abort since the dissipation phase is skipped for this one, as such there is more propellant that there ought to be which requires less thrust to meet the MECO constraints
- Right before throttling deactivation, the throttle starts to show signs of instability

The Glide-RTLS phase, which starts after arrival on the RV line, is completely open-loop, just a control loop exeuting pre-programmed manoeuvres triggered by the vessel state. The 40° angle of attack should be no problem for the Shuttle, assuming the control surfaces are set as per the Entry script instructions. Do not shift the CG aft, as the script will deploy the body flap to stabilise pitch and I've seen that this induces yaw instability in this phase.  


## TAL abort

The TAL abort is triggered if an engine is shut down between negative return and 4350 inertial velocity. The boundary is called **press to ATO** and an ATO/AOA abort is commanded after that.  
The TAL site is selected automatically from the landing sites defined in **Shuttle_entrysim/landing_sites.ks** based on whether they lie downrange and estimating if there is enough delta-V to alter the trajectory within ~800km crossrange of them. One site is chosen at random out of all the ones satisfying these conditions, to simulate weather availability. The site choice can be overridden by specifying the site name in the **shuttle.ks** file.  

Once the site is selected, Closed-loop Guidance will alter the PEG target state so that the trajectory falls within the crossrange limits to the landing site. Apart from the internal targeting, the abort is carried out like a normal ascent, the only difference being an automatic OMS fuel dump.  
After MECO and separation the Shuttle will be at around 110km and about to descend. Stop the ascent script immediately and begin entry preparations.

In some cases, attemtping to reduce reentry crossrange may result in a MECO underspeed. This happens in cases where only one TAL site is available and late in the ascent, since the faster the Shuttle is already, the more deltaV it takes to steer the trajectory within crossrange limits.  
If the underspeed is very severe, you have a small window to do an OMS burn to add a little horizontal velocity before engaging Reentry guidance.  
**Do not add too much velocity**, even in the nominal TAL case the ballistic trajectory should fall well short of the target.

## ATO/AOA aborts

Both aborts use the same guidance and differ only in what you decide to do after MECO. They are triggered if an engine is shut down between Press to ATO and 6100 inertial velocity. The boundary is called **press to MECO** and no abort is commanded after that.  
This abort mode lowers the cutoff altitude a bit and the apoapsis to about 160km, just outside of the upper atmosphere. Additionally it forces guidance not to thrust out of plane anymore, giving more performance margin at the cost of a MECO orbital inclination lower than desired. Also no OMS dump is performed as you will need the fuel to do orbital corrections later on.  

After MECO you have two options:
- do an OMS burn to raise the orbit and continue your mission
- firstly do a combined circularization - plane change OMS burn to bring you closer to a landing site on the next pass. Later on do a second OMS deorbit burn with the deorbit planner tool

If you use my latest version of SpaceShuttleSystem and the fork of FAR that I created, with the realistic aerodymamics, you have enough crossrange to cover all Abort Once Around trajectories.  
For AoA aborts out of KSC, you can return to KSC for inclinations up to ~45° . For higher inclinations, Northrup (White Sands) is your best choice.  
For Vandenberg launches, AoA back to Vandenberg is your only option

## Post-ATO engine failure 

After the ATO boundary, the program will assume that it's able to achieve the nominal MECO targets. The only action here is to get rid of the G-throttling logic.
