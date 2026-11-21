# 1.4 Complete humanoid locomotion knowledge list

This document reproduces and organizes Section 1.4 of *Building a Humanoid
Robot Brain – AI Model and Knowledge Graph in C++17-26*. It preserves the
section's examples and content, and separates source guidance from explicitly
illustrative additions. The source's control numbers are not validated
hardware settings.

## 1.4.1 Move in any direction

The robot should accept a body-relative velocity command:

| Command | Meaning |
| --- | --- |
| `vx` | Forward/backward velocity |
| `vy` | Left/right lateral velocity |
| `wz` | Turning velocity around the vertical axis |

Typical sign convention:

| Command | Meaning |
| --- | --- |
| `vx > 0` | Walk forward |
| `vx < 0` | Walk backward |
| `vy > 0` | Step left |
| `vy < 0` | Step right |
| `wz > 0` | Turn left |
| `wz < 0` | Turn right |
| `vx + vy` | Walk diagonally |
| `vx + wz` | Walk while turning |

The sign convention is a design choice and must be documented consistently
between the command interface, planner, controller, and physical coordinate
frame.

The locomotion controller should support:

- Standing
- Starting and stopping
- Forward and backward walking
- Side-stepping
- Diagonal walking
- Turning in place
- Walking along curved paths
- Changing speed smoothly
- Transitioning between gait modes
- Recovering from pushes
- Handling slopes, stairs, uneven surfaces, and low-friction terrain
- Stopping safely when balance or perception is unreliable

## 1.4.2 Standing

Standing is the first locomotion capability to get right. If the robot cannot
maintain a controlled standing pose, walking controllers have no reliable
initial or recovery state.

### Neutral standing

The neutral stand is a repeatable home position:

```cpp
struct StandTarget {
    double pelvisHeight = 0.78;
    double hipPitch = 0.0;
    double kneePitch = 0.15;
    double anklePitch = -0.08;
};

void enterStand()
{
    gaitMode = GaitMode::STAND;
    desiredVelocity = {0.0, 0.0, 0.0};
}
```

This known configuration supports startup, calibration, stopping, and beginning
another behavior. The dimensions and angles are illustrative; they must be
replaced with measured values for a physical robot.

### IMU-assisted standing

The source example uses PD error terms:

```cpp
double rollError = 0.0 - imu.roll;
double pitchError = 0.0 - imu.pitch;

ankleRollTorque =
    kpRoll * rollError -
    kdRoll * imu.rollRate;

anklePitchTorque =
    kpPitch * pitchError -
    kdPitch * imu.pitchRate;
```

The controller is the torque-corrective form

\\[
  \\tau_r = K_{p,r}(\theta_{r,d}-\\theta_r)
           - K_{d,r}\\dot{\\theta}_r
\\]

and

\\[
  \\tau_p = K_{p,p}(\\theta_{p,d}-\\theta_p)
           - K_{d,p}\\dot{\\theta}_p.
\\]

For a desired vertical orientation, the desired roll and pitch are zero. Gains
must be selected from robot dynamics and stability analysis. Torque commands
must remain within actuator limits.

### Center-of-pressure standing

Foot force or pressure sensors estimate where the robot's load acts:

```cpp
double copErrorX = desiredCopX - measuredCopX;
double copErrorY = desiredCopY - measuredCopY;

anklePitchCorrection = kpCop * copErrorX;
ankleRollCorrection = kpCop * copErrorY;
```

For measured pressure points with vertical forces \\(F_{z,i}\\),

\\[
p_x = \\frac{\\sum_i F_{z,i}x_i}{\\sum_i F_{z,i}},
\\qquad
p_y = \\frac{\\sum_i F_{z,i}y_i}{\\sum_i F_{z,i}},
\\qquad
\\sum_i F_{z,i} > 0.
\\]

The result is valid only if the denominator is positive and the pressure
mapping is calibrated. Joint encoders alone do not provide this load-location
information.

### Weight shifting

Before lifting the right foot:

```cpp
desiredComY = +0.04;     // illustration only
supportLeg = Leg::LEFT;
```

Then verify:

```cpp
if (leftFoot.forceZ > requiredSupportForce &&
    balanceMargin > minimumMargin)
{
    allowRightFootLift = true;
}
```

Single-support walking requires transferring enough of the robot's weight onto
the support leg. These thresholds must come from measured mass, friction,
actuator capability, and balance analysis; `0.04 m` is not a general setting.

### Disturbance-resistant standing

```cpp
if (std::abs(imu.pitchRate) > disturbanceThreshold)
{
    increaseDamping();
    activateHipBalance();
    evaluateRecoveryStep();
}
```

Standing must tolerate modest disturbances instead of assuming a perfectly
stationary floor and robot. The correction must include a controlled stop or
recovery path, timing checks, and actuator limits.

## 1.4.3 ZMP and CoP

The Zero-Moment Point (ZMP) and Center of Pressure (CoP) are foundational
metrics for evaluating and ensuring bipedal stability.

- **CoP:** the point where the net resultant of ground contact forces acts.
  The physical CoP cannot exist outside the foot or support polygon.
- **ZMP:** the theoretical ground point where the net horizontal moment of
  gravity and inertial forces is zero.
- **Stable condition:** if the planned ZMP remains inside the support
  polygon, the CoP can match it and the feet can remain flat.
- **Tipping condition:** if a push or motion drives the ZMP outside the
  support polygon, the physical CoP reaches the footprint boundary and a
  non-zero edge moment can tip the robot.

For a footprint in the `x-y` ground plane, a test point is stable only if:

\\[
x_{\\min} \\le x \\le x_{\\max},
\\qquad
y_{\\min} \\le y \\le y_{\\max}.
\\]

For a convex polygon, use a general point-in-polygon test; a rectangular
bounds test is a special case.

## 1.4.4 Linear Inverted Pendulum Model

The Linear Inverted Pendulum Model (LIPM) is a simplified planning model that
treats the robot's mass as a point CoM supported by a massless, rigid leg.

For constant CoM height \\(z_c\\), zero angular-momentum assumption, and ground
ZMP \\(p\\), the planar equations are:

\\[
\\ddot{x}=\\frac{g}{z_c}(x-p),
\\qquad
\\ddot{y}=\\frac{g}{z_c}(y-p).
\\]

Solving for the ZMP gives:

\\[
p_x=x_c-\\frac{z_c}{g}\\ddot{x}_c,
\\qquad
p_y=y_c-\\frac{z_c}{g}\\ddot{y}_c.
\\]

The source also distinguishes capture-point reasoning:

\\[
\\xi_x=x_c+\\frac{\\dot{x}_c}{\\sqrt{g/z_c}},
\\qquad
\\xi_y=y_c+\\frac{\\dot{y}_c}{\\sqrt{g/z_c}},
\\]

where \\(\\xi\\) identifies where a nominal constant-height LIPM step can help
bring the model to rest. Capture point is a planning aid, not a proof of
stability for a full humanoid.

The executable implementation is in
`src/section_1_4_locomotion.cpp`. It demonstrates, without performing hardware
motion:

- body-relative velocity limits;
- neutral-standing state;
- IMU roll/pitch PD corrections;
- vertical-force-weighted CoP;
- support-force and balance-margin foot-lift gating;
- an illustrative rectangular support-polygon test;
- LIPM ZMP calculation.

## 1.4.5 Source corrections and scope

The following clarifications are necessary for implementation:

1. **The sign of `vy` is convention-dependent.** The source's table assigns
   `vy > 0` to stepping left. Many robot frames use `vy > 0` for right;
   the repository follows the source exactly and documents the convention.
2. **The neutral pose is illustrative.** The pelvis, hip, knee, and ankle
   values are not physical-robot limits.
3. **ZMP and CoP are related but not identical concepts.** ZMP is a dynamic
   model point; CoP is constrained by actual ground contacts.
4. **LIPM is a template model.** It assumes constant horizontal CoM height,
   no angular momentum, point-mass dynamics, and suitable ground conditions.
5. **No sample gain or threshold is certified.** All gains and margins require
   robot-specific identification and stability testing.
6. **Low-friction terrain and unreliable perception require conservative
   stopping.** A velocity command should not be executed without current,
   trustworthy state and contact estimates.
