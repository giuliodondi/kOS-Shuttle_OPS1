# Kerbal Space Program Space Shuttle Ascent Guidance
## Updated April 2024 - please go through the README

My KSP ascent guidance script for the Space Shuttle, intended to be used in KSP Realism Overhaul.
Uses Powered Explicit Guidance (also called UPFG) for vacuum guidance, adapted and modified to perform aborts. Allows for assisted manual flight.

# References

- [SPACE SHUTTLE ORBITER OPERATIONAL LEVEL C SOFTWARE REQUIREMENTS - GUIDANCE ASCENT/RTLS](https://www.ibiblio.org/apollo/Shuttle/STS83-0002-34%20-%20Guidance%20Ascent-RTLS.pdf)
- [Flight Procedures Handbook - Ascent/Aborts (OI-30)](https://www.ibiblio.org/apollo/Shuttle/Crew%20Training/Flight%20Procedures%20Ascent-Aborts.pdf)

# Installation

**Required mods:**
- A complete install of RSS/Realism Overhaul with Ferram Aerospace Resarch. 
- kOS version 1.3 at least
- kOS Ferram, now available on CKAN
- [My own fork of SpaceODY's Space Shuttle System](https://github.com/giuliodondi/Space-Shuttle-System-Expanded). 
  - if you use the latest version you will be required to also grab my Ferram Fork to use the custom aerodynamics module. Refer to the README
- **[My OPS3 Shuttle entry program](https://github.com/giuliodondi/kOS-Shuttle-OPS3) required by RTLS and TAL aborts. Grab the latest version from its repo**
  - **no longer compatible with the older kOS-ShuttleEntrySim program**

**Not compatible with SpaceODY's original fork or any other Shuttle mod.**

You will find one folder: 
- **Script**

Put the contents of the Script folder inside Ship/Script so that kOS can see all the files.
There are a few scripts you can run:
- **ops1.ks** to setup a launch of the Shuttle from the launchpad according to specified mission parameters (read on to learn about mission setup).
- **ops13a.ks** is an identical script with special parameters for Polar orbit launches from Vandenberg
- **node.ks** a little manoeuvre node executor that I use to execute OMS burns in orbit, it takes care of the offset OMS thrust position (so it even works with a single OMS burn) and is compatible with nodes created with Principia


# Setup  

The script needs to know accurately the mass of orbiter + payload + ET + propellants for closed-loop guidance, without the mass of SRB or launch clamps. The script will measure everything automatically provided that the part tree is set up correctly in the VAB.  

Take care of the following things while building the Shuttle Stack in the VAB:
- The root part must be the Space Shuttle Orbiter part you find in my fork of Space Shuttle System
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
- ### Don't forget to set the FAR control surface settings as required by the Entry script README.
- right-click on the SSMEs and then open up the Real Fuels GUI. Make sure you are selecting an appropriate version of SSME (refer to [this Wikipedia table](https://en.wikipedia.org/wiki/RS-25#/media/File:SSME_Flight_History.png) if you want to select the version accurately). **Make sure you select the same version for all three SSMEs.**
- if you do a launch from Vandenberg, switch the SRB type to 'Filament-wound casing' for extra performance

If you don't use SpaceShuttleSystem parts, or if you mismatch SSME versions, the script should detect this and break during vehicle initialisation. This is intended and meant to signal to you that something is wrong.
 

## kOS configuration file

The mission parameters are specified in the main launch script **ops1.ks** or **ops13a.ks**.  **The scripts assume that the Shuttle stack has been assembled properly in the VAB, else it might fail to measure things accurately.**

It contains variable definitions for:
- a "target_orbit" structure describing the shape of the desired orbit. Periapsis and cutoff altitude are standard for a Shuttle launch and shouldn't be changed. Only change the apoapsis and inclination to suit your needs.
- a disabled variable *engine_failure_time* which you can uncomment to trigger an automatic engine failure at the specified time. More on aborts later on.
- a variable to enable telemetry logging in the UPFG_OPS1/LOGS/ folder if you want to plot it with your own means
 
# The main Ascent script

Upon running **ops1.ks** you will be greeted with a GUI with buttons and a data display to interface with the program:

![ascent_gui](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/ascent_gui.png)

- The _X_ button will interrupt the program at any point
- _DAP_ selects the digital autopilot modes. By default it's AUTO but it can be switched to CSS for manual flight. More on autopilot modes later on.
- The red _ABORT_ button triggers a manual abort and the selector menu to its left selects the modes currently available. More on aborts later on.
- the _TAL site_ selector is specifically to choose an available landing site for a manual TAL abort
- the _Display_ will show different things depending on the flight phase. There will be plenty more about this later on.
- the _Message Window_ at the bottom displays printouts of the program state. Useful to monitor the abort state

## The ASCENT TRAJ displays

![ascent_traj_displays](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/ascent_traj_displays.png)

These two displays show useful data and trajectory printouts:
- **_TRAJ 1_** displays first stage (SRB) flight until right before SRB sep
- **_TRAJ 2_** takes over from there all the way to MECO
- At the top you have the display title and the running mission elapsed time
  - the display title is ASCENT in the nominal case, but in an abort it will change to RTLS, TAL, ATO or CONT
- On the left side you have:
  - current vertical speed **Ḣ**
  - the errors with respect to guidance commands: roll **_R_**, pitch **_P_**, yaw **_Y_**, throttle **_T_**
  - R,P,Y errors are in angles with respect to the commanded thrust direction in the vehicle-fixed frame of reference, T is a percentage.
  - When the errors are too large, the numbers will turn yellow
- **_CONT ABORT_** shows the current active contingency abort regions for 2- and 3-engine-out situations. More about this in the aborts section
- The TRAJ 1 central plot shows altitude vs. surface velocity
  - The ticks show the pitch angle that the Shuttle should have at that moment
  - The solid line is the typical nominal trajectory
  - The dashed line is the trajectory in case of one engine out at liftoff
- The TRAJ 2 plot is altitude vs. inertial (orbital) velocity
  - the solid line shows the early lofting and droop which is typical of the Shuttle
  - the ticks show roughly the intact abort boundaries: negative return (RTLS), press to ATO (ATO), last TAL opportunity (TAL)
- On the right side you have:
  - a slider with the current acceleration in G-units, the indicator will turn yellow if it exceeds 3G
  - **_THROT_** is the current throttle level in units of Rated Power Level
  - **_PROP_** is the percentage of remaining propellant in the External Tank
- On the TRAJ 2 display, below the title, there is the MECO velocity line indicator, close to MECO it shows how far off from the targeted cutoff velocity (**_CO_**) you are
- On the TRAJ2 display, in the top right corner, are some UPFG-specific values:
  - **_TGO_** is the guidance-calculated time to MECO
  - **_VGO_** is the guidance-calculated delta-V to gain to MECO
  - the numbers will be yellow when the algorithm is unconverged, green upon convergence


## Meaning of Rated Power Level and the THROT indicator

The Space Shuttle Main Engine had several performance improvements in its operational life, and you have all the variants at your disposal in KSP RO.  
The caveat is that "full power" is an ambiguous term in this situation. For this reason we define the Rated Power Level (100% RPL) as 2090 kN, the max thrust of the original variant. 
This is significant because for the later part of the Shuttle program, when they used the RS-25D variant, the nominal throttle setting was 104% RPL and boosted to the full 109% RPL in case of an abort. This was done to reduce wear and risk of failure on the engines, it's all realistic and simulated by the script.

## How to fly manually with CSS

CSS means control-stick-steering, a fly-by-wire mode that allows you to use keyboard or joystick to command changes in pitch, roll or yaw letting kOS worry about keeping stability.  
At any time you can switch between the two. **A manual ascent from liftoff to orbit in CSS is possible but challenging**.
Here are some tips and remarks on how to do it:
- switch the KSP camera to _Locked_ or switch to IVA mode
- **the errors are defined relative to the body axes, not the Earth's horizon**
- Positive Roll and Yaw errors means you need to correct to the **right**. Positive pitch error means you need to pitch **up**. Positive throttle error means you need to throttle up
- You will need to constantly correct, especialy in pitch
- Make small corrections and see the result in the Display before correcting further
- **The initial roll manoeuvre is by far the hardest:**
  - it's a combined roll, pitch and yaw manoeuvre that puts you on the right azimuth for ascent
  - It's useful not to rely solely on the numbers and also have a mental image of where the Shuttle is and how it needs to move to ascend heads-down
  - use continuous roll inputs and correct pitch and yaw as required by the error numbers
- During TRAJ 1 you can use the trajectory plot as a guide to correct your pitch, you shouldn't go too far off the line
- During TRAJ 2 I don't suggest you try to follow the line closely, the Guidance commands already account for any error you accumulated

# Nominal mission scenario

Running the **ops1.ks** script is the only action required for a nominal launch (aborts are different). Simply running the script will program the launch about 10 seconds after running the script, meaning that the LAN of the target orbit depends on the time of day you choose to launch.  
**You can launch to rendezvous with a vessel in orbit by selecting it as a target in the map view BEFORE running the script.** This will override the launch inclination to match and warp to the right time to launch so the LAN is correct.

These are the main events durign ascent:
- Fuel cells are automatically activated at liftoff
- Dynamic pressure (**Q**) is monitored throughout and SSME throttle will be reduced to 75% RPL during the Max-Q period. **If you fly CSS you have to be alert and adjust throttle**
- SRB separation is thrust-dependent and usually happens around the T+02:00 mark
- 5 seconds after SRB sep the program transitions to closed-loop guidance using the UPFG algorithm
- On a nominal mission a roll to heads-up attitude is performed at T+05:50
- About a minute before MECO, Guidance will start throttling down the engines to maintain a maximum acceleration of 3G. **Again, if you're flying CSS you need to be alert for this**
- 5 seconds before the targeted MECO the program transitions to terminal guidance at minimum throttle
  - alternatively it can happen 5 seconds before the Shuttle runs out of propellant
- After MECO the program will automatically:
  - disable SSME gimballing
  - trigger ET sep
  - command an RCS vertical translation manoeuvre
  - close the umbilical doors
- The program then prints a message in the message window, displaying the results of an orbital analysis calculating the erros with respect to the targeted orbit
- 5 seconds after this, the program will quit itself
- **Do not forget that the nominal ascent puts the shuttle on a trajectory that dips back into the atmosphere for ET disposal. You must perform manually an OMS bun to circularise.** 

# Abort scenarios

### General remarks

- The Shuttle requires abort modes whenever reaching the nominal orbit is impossible or not appropriate, either for an SSME failure or some other problem (fuel cells, windows, hydraulics...)
- There are two kinds of abort modes:
  - **Intact aborts** where there always is a procedure that will take the Shuttle to a landing runway with sufficient energy
  - **Contingency aborts** where the Shuttle is not guaranteed to reach a runway or even survive the reentry into the lower atmosphere
- Inta


### General considerations

Aborts can be triggered by uncommenting the *engine_failure_time* variable in the main **shuttle.ks** script. The time specified will trigger a different abort mode, each with its own guidance and targeting scheme. Alternatively a failure can be triggered manually by shutting down an engine mid-flight, the program is able to detect both situations.  
The boundaries between abort scenarios depend on velocity, so they may be reached at significantly different METs depending on the SSME condiguration and the payload.  
All abort scenarios discard the SSME throttling stages and keep the throttle at maximum until MECO or fuel depletion, except for RTLS which uses throttling for guidance.

Bear in mind that only **intact aborts** are covered right now. A double engine failure is a Contingency scenario and the script will not attempt to handle them.

**Caveat:**
The Shuttle has its engines pointed away from the main vehicle axis and as such there is coupling between yaw and roll. This script uses the kOS built-in steering manager which is unaware of this coupling and thus struggles at times.  Oscillations are expected and should be fine, unless they are too severe.


## Return to Launch Site (RTLS) abort 

RTLS is triggered if an engine fails before a certain surface-relative velocity is reached. The actual speed varies between 2600 and 2200 m/s depending on how large the desired orbital inclination is. The boundary is called **negative return** and a TAL abort is commanded after that. This abort scenario is quite involved and has a powered phase (until MECO) and a Glide phase after that.

### Powered-RTLS

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

### Glide-RTLS

After MECO and separation, the script will stop and call the OPS3 reentry script which takes case of  **Glide-RTLS (GRTLS)** guidance.  
In a nutshell: the Shuttle pitches up to 45° as it performs an aerobraking manoeuvre to stabilise the falling trajectory into a more nominal reentry trajectory, controlling vertical G forces. At the end of GRTLS the Shuttle will be about 200 km from the launch site, 35/40km altitude at about Mach 4, in a gentle descent.  
**Please read carefully the OPS3 documentation for more about Glide-RTLS**

### RTLS TRAJ 2 display

![rtls_traj_2_gui](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/rtls_traj2_gui.png)

This display is rendered when the RTLS abort is initialised (not when the engine is lost) and lasts until after MECO.

- At the top you have another cutoff velocity indicator, this time in terms of surface velocity. The range is from 1500 to 2500 m/s, and since the goal is to turn back higher cutoff speeds are indicated from right to left.
    - While the CO mark is actively placed in the right place, you should expect the actual speed at cutoff to be off this mark by a little.
- The right data box only contains the G indicator and attitude angles
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

![rtls_trajplot](https://github.com/giuliodondi/kOS-UPFG_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/rtls_traj.png)

The first plot is altitude vs. downrange distance, and shows the general shape of the trajectory. You can see:
- engine failures before SRB sep result in lofted trajectories, as commanded
- The maximum altitude during the manoeuvre is geenrally higher the later the engine failure
- all trajectories target the same altitude at MECO, it's the "hump" at 72km
- The downrange distance at MECO is different for all trajectories

![](https://github.com/giuliodondi/kOS-UPFG_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/rtls_vel.png)

The second plot is horizontal velocity vs. altitude. Here you can see clearly:
- when the engine was lost (the point of deviaton from the dashed nominal line)
- when flyback was triggered (the point where the lines "bounce to the left"
- that the Shuttle is already on its way down when flyback is triggered, in the case of early aborts
- that the Shuttle does not come to a dead stop at the inversion point, as there is some sideways motion
- That, just like downrange distance, all velocities at MECO are different

![](https://github.com/giuliodondi/kOS-UPFG_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/rtls_rvline.png)

The third plot shows downrange distance vs. velocity, centered around the point of MECO. All the lines terminate close to the Range-Velocity (RV) line, which is the equation that Guidance uses to target MECO. Where exactly you land on the RV line is not important as long as the script cuts off the engines on it, which you can see as the velocity becomes almost constant (before increasing again during the Glide-RTLS descent).  
Curiously, very early and very late aborts produce trajectories that arrive closer to the site than mid aborts, this trend can also be seen in the farthest distance reached during flyback (see plot 1). Early aborts have low speed and acceleration and so the Shuttle cannot travel very far before flyback, while late aborts have the highest acceleration.


![](https://github.com/giuliodondi/kOS-Shuttle_OPS1/blob/master/Ships/Script/Shuttle_OPS1/images/rtls_throt.png)

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
The TAL site is selected automatically from the landing sites defined in **Shuttle_OPS3/landing_sites.ks** based on whether they lie downrange and estimating if there is enough delta-V to alter the trajectory within ~800km crossrange of them. One site is chosen at random out of all the ones satisfying these conditions, to simulate weather availability. The site choice can be overridden by specifying the site name in the **shuttle.ks** file.  

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
