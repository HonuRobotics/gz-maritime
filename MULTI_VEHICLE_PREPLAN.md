# Multi-vehicle support pre-plan

The purpose of this document is to provide an artifact for collaboration at the first step of a new feature development. The idea is to get consensus on "what" we intend to build, and why, before we work out "how" we build it, and before we (and our agents) actually build it. This "pre-plan" would describe what we plan to develop (and why) and would feed into #28 to describe "how" and feasibility.

Below is an enumeration of the decisions I anticipate in the effort and a mix of proposals (stated as a decision, but open to feedback/discussion) and decision call outs (where I think someone else has a better sense for the best decision, or we need to do some work to inform the decision. )

## What and Why

### What we should do

The SOW says we must ...

```
1.4. Multi-vehicle coordination infrastructure. Develop the simulation infrastructure
required for multi-vehicle operations:
1.4.1. Competition server communication protocol for reporting, task activation, and
scoring.
1.4.2. Namespace isolation and multi-vehicle spawning support.
1.4.3. Performance optimization for running 3+ vehicles with full sensor suites simultaneously.
```

How much of 1.4 does this development task cover? Issue #23 is scoped to 1.4.2 alone, but from the meta-issue I thought #23 was for all of 1.4. 1.4.1 names a competition server that I do not believe exists yet. The rest is a proposed pre-plan for all of 1.4, without 1.4.1.

I'd suggest the deliverable we can point to for this task is a documented demo/tutorial that includs:
- refers to the build instructions to use - so that the the tutorial specifies the starting point
- launches a prototype VRX world with the BlueBoat, the BlueROV2 and the UAV all operating simultaneously.  By "operating" we mean no actuation (in the tutorial/demo), but returning sensor data.   
- description of what success looks like - screenshots, ros2 topic echo/pub examples, etc. 
I don't think there is a suitable world included, so that will be part of this, but it should be a minimum viable world for the demo (we'll add task elements later)

We should also decide if SITL is within the scope of this task. I'd suggest it be outside the scope for now because that work is still ongoing, and it limits, at least a little, the scope of this chunk of work. If we agree, I'll open/ an issue up integrate SITL for each vessel later. The practical consequence is that the tutorial drives all three vehicles over namespaced ROS topics. This suggests that the UAV should be spawned on land, because there is no controller to keep it airborne.

> **Decision 3 — SITL.** Is SITL out of scope here, with the consequences that the tutorial commands all three vehicles over namespaced ROS topics and the UAV sits on land rather than flying?

Another decision is what we should include in "full sensor suites" for each vehicle. Most are standard, but how many cameras for each, multibeam sonar on the ROV, pinger on the ROV, how many lidars on the USV, and so on. This would be a good opportunity to define a "stock" configuration for each robot — one named sensor set per vehicle that the tutorial, the performance target and any future benchmark all refer to.

> **Open Issue — Stock sensor suites.** We should define "stock" sensor config for each robot.

It would also be good to rough out what we mean by "performance optimization" within the context of this task. My suggestion for this subtask is just to measure the performance in some sort of standard way that normalizes for local compute resources. Unless performance (measured only by RTF and FPS?) is terrible, we don't spend any additional time on this, at this time. Instead we open an issue to do an in-depth profiling and optimization thread when time allows. If we can show the full world with three vehicles running on a reasonable desktop at 1.0 RTF, that would be amazing.


The other day we mentioned two interpretations of "multi-vehicle": (A) more than one of the same vehicle, and (B) more than one vehicle and more than one type — UAV, UUV, USV. I believe we only need the VRX-inspired case, multi-vehicle = 1 USV + 1 UUV + 1 UAV. I don't think case A adds anything the SOW asks for. While it would probably work, I'd suggest we plan to try it, with a world with three USVs (or UUVs). If it works, we have another example to document. But it isn't strictly required, so if there are any unexpected issues, we don't let it block us and open an issue for later.
We build for case B, and attempt case A opportunistically — documented as a second example if it works, deferred to an issue if it does not, and blocking nothing either way.

So a potential issue with our prototype VRX world is the interaction between the USV and UUV when they are close together in waves. I don't believe the current hydrodynamics of the UUV and wavefield interaction capture the near-surface effects. If not, when they are close together in waves, the relative motion will not be well represented — it will not "look right". I'd suggest we handle it this way:

* The VRX prototype world we build has the UUV at significant depth, more than 1.5 wavelengths of the modal wavelength of the wavefield. We don't want to automate that as a variable — just pick a constant depth value that is "deep enough" for most cases. We should do a test of the proximity issue to see how bad it is and then, if there is a problem, open an issue to track it for later, but not let it block this first effort.

### Things we should do later

Work I think we are genuinely on the hook for, but not in this task. Each of these should have an issue created for tracking purposes.


* SITL integration for each vessel, per the scope decision above. The ArduRover work on the BlueBoat is in flight; ArduSub for the BlueROV2 and ArduCopter for the X500 are each their own bring-up.
* The competition server communication protocol, SOW 1.4.1, assuming we confirm it does not already exist.
* Whatever performance work the measurement turns up but does not require. If the numbers come back healthy we stop, rather than putting much effort into optimizing.

### Things we could do

In interpreting the SOW we need to define what is required (what we "should" do) and what would be a nice to have (what we "could" do). This typically arises when we need to make a judgement in interpreting a SOW item. The stuff we deem outside the SOW can then be optional items, if we have time and budget. Each of these becomes an issue, which is also good for any community members who are interested in contributing.

* An RViz preset and Gazebo GUI layout for watching three namespaced vehicles at once.
* The VRX prototype world should be a minimal model to demonstrate the three vehicles.   Later, it would be nice to add some task elements (undersea pipe, etc.).


## Timing

A note on timing. I think we should, if we can, merge the SITL work before starting the implementation of this. There is a change to the thruster interface within that feature set — gz-maritime #18 and bluerobotics_models #53 — and it would save a little time to have that in `lyrical` before we branch.

> **Decision 8 — SITL merge first?** Do we wait for the normalized thruster interface (SITL work) into `lyrical` before branching this work?


## What success looks like

The CI will evolve naturally to cover new tests associated with the features of this task.  The final end-to-end check will be the tutorial/demo described above, run in the drydock container (so that any compatiblity issues with drydock are discovered)


