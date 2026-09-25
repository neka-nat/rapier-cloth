# Rapier end-effector control

The experimental implicit towel demo accepts world poses for two Rapier
kinematic bodies. Hard vertex attachments carry the cloth through body-local
anchors. The headless example and live server share the same host adapter:
[implicit_robot.rs](../examples/support/implicit_robot.rs). This is an example-level
integration contract, not a new stable robot-control API.

## Run and operate

```bash
npm --prefix demos/viewer ci
npm --prefix demos/viewer run live:implicit
```

Open **http://127.0.0.1:5173/live.html?scene=implicit_towel&paused=1**.
The launcher builds the Rust server with `f64,implicit`. This scene uses four
CPU workers, one 0.1 s physical solve per request, and one 32×32 towel.
The library's default execution remains serial.

Press **Resume** for the fold, release and settle sequence. The default is
**Stop (strict)**. To run the bounded approximate mode, select a **Script**, choose
**Continue after validation (approximate)** under **At iteration limit**, then
press **Reset**. Reset applies those selections to a new task. For the 5 mm lift:

**http://127.0.0.1:5173/live.html?scene=implicit_towel&paused=1&case=lift_5mm&cap_policy=approximate**

With the default solver settings the scripted conditions converge, so this policy
only takes effect if a step reaches the iteration limit, for example during manual
operation. The solver status displays **Approximate · not converged** for a capped return.
The cumulative **Approximate steps** count remains visible on later converged
frames and resets to zero with the task. Accepted approximations continue the
script; solver errors still stop it.
For manual operation, leave the task paused, select a hand, use **X/Y/Z ±**
(5 mm) or **Roll ±** (5 degrees about world X), then press **Step frame**.
A pose button queues the target without advancing simulation time.
**Release left/right gripper** and **Grasp selected patch** act without stepping.
Any manual command ends the scripted trajectory and holds the other hand at its
current pose. **Reset** restores the initial cloth, hands, grasps, clock and script.
Each task ends at 80 accepted steps (8 simulated seconds), including manual tasks.

The orange and blue boxes mark end-effector frames. They have no collision
geometry or force feedback. Both hands initially hold fixed material patches
(left 27 vertices, right 26). Regrasp selects that hand's original patch and stores
its current shape as local anchors; it does not teleport the cloth. The hand's
origin must be within 2 cm of the patch's first vertex. Arbitrary layer selection,
physical fingertips and a robot arm/controller are outside this example.

![Implicit towel with Rapier hand controls](images/robot-control.png)

Captured from the live Rust server in Chromium software WebGL.

## Connect a host controller

Use metres, seconds, a Y-up world frame and unit quaternions in **[x, y, z, w]**
order. The shared Rust adapter accepts:

```rust,ignore
// Construction defaults to strict; opt in explicitly when appropriate:
// RobotTowel::with_policy("lift_5mm", ImplicitExecution::Parallel4, CapPolicy::Approximate)?;
task.set_pose(0, translation, rotation, task.step + 1)?;
task.set_pose(1, other_translation, other_rotation, task.step + 1)?;
task.tick()?; // advances Rapier and cloth together by 0.1 s
task.release(0)?; // retains current cloth positions and velocities
```

In manual mode, a target remains in force until replaced. Commands are validated
before mutation. A pose must target exactly the next accepted step, remain in
x/z = ±0.75 m and y = 0..0.75 m, and move no more than 5 cm or rotate more than
22.5 degrees relative to the last accepted pose. Multiple pose updates do not
bypass these per-step bounds. These limits bound this demo's input; they do not
guarantee convergence for every allowed motion.

The matching browser protocol (version 2) uses the existing local WebSocket.
For example, at accepted step 0:

```json
{"request_id":1,"command":{"type":"set_gripper_pose","gripper":0,"translation":[-0.25,0.005636,0.25],"rotation":[0,0,0,1],"at_step":1}}
```

Read the current pose from `frame.implicit.grippers` before constructing a target.
Other commands are `step`, `release_gripper`, `grasp_gripper`, and
`reset` with `scene: "implicit_towel"`. Gripper IDs are 0 (left) and 1 (right).
A reset can include `implicit: {"variant":"lift_5mm","cap_policy":"approximate"}`;
omitting it selects nominal/strict. Unknown policies, variants or options are
rejected before replacing the current world. For example:

```json
{"request_id":2,"command":{"type":"reset","scene":"implicit_towel","implicit":{"variant":"lift_5mm","cap_policy":"approximate"}}}
```

A pose response acknowledges the pending target in `implicit.desired`; actual
body poses and cloth positions change only after a successful `step`.
For coordinated motion, acknowledge both hands' commands before sending `step`.

Frames also expose the accepted clock, current grasp anchors, `automatic`,
`completed`, `stopped` and measured extension/speeds. `implicit.cap_policy` and
`implicit.variant` identify the active configuration. `implicit.sample.outcome`
is null at step zero, otherwise it contains `termination` (`converged` or
`approximate_iteration_cap`), `converged`, energy and residual forces.
`implicit.sample.approximate_steps` is cumulative, and `iterations` reports
actual work. An accepted approximation advances cloth, Rapier and time exactly
once, with `advanced_substeps = 1`. Its `converged` flag stays false. See
[the cap-policy contract](implicit.md#iteration-limits-and-accepted-outcomes). `implicit_available`
advertises server capability. The ordinary f32 server cannot create this scene.
The browser keeps one request in flight and coalesces unsent commands per hand;
it does not interpolate poses or accumulate catch-up physics. See the
[live protocol](live-demo.md#architecture) for transport and origin settings.

## Failure and recovery

Before each solve, the adapter checkpoints cloth, Rapier bodies/caches/joints,
and grasp ownership. Failure restores the last accepted physical state and clock,
preserves the pending manual targets, and latches the error. The live server sends
that accepted state with zero advanced substeps; the browser stops.
Use **Reset** before sending further motion. Neither extra physical substeps nor
larger Newton budgets are silently inserted.

An invalid command returns an error without changing the task. The browser pauses;
a valid replacement command or reset can be sent. Disconnect/reconnect creates a
new independent world.

## Headless reproduction and evidence

```bash
cargo run --locked --release --no-default-features --features f64,implicit --example robot_towel_implicit -- --output robot-fold.jsonl
cargo run --locked --release --no-default-features --features f64,implicit --example robot_towel_implicit -- --case lift_5mm --cap-policy approximate --workers 4
```

The output path must be new. JSONL includes actual hand poses, grasp ownership,
accepted cloth states, timing and terminal status. This diagnostic format is
separate from the recording viewer's schema; use the live page to inspect robot
control. Each sample retains its typed outcome and cumulative approximate count.
`--cap-policy strict|approximate` defaults to strict. `--workers 1` selects serial
execution; this example defaults to 4.

The directly prescribed h=0.1 s fixture passes seven bounded cases in strict
mode: nominal, grasp inset by one grid column, lift increased by 5 mm, left
release one step early, right release one step late, and friction 0.4/0.6. All
560 accepted steps converge; the lift's former capped step 24 converges in about
90 iterations. These checks do not qualify arbitrary manual motion.

The Rapier adapter completes the nominal and lift scripts with all 80 live frames
exactly matching the headless adapter on the same Linux build/host. Direct pin
commands and the Rapier adapter are separate control paths and are not claimed
bitwise identical. Continuous-motion protection remains the runtime CCD.

Physical task success at h=0.1 and wall-clock responsiveness are separate
requirements. See the [current solver timing](performance.md#implicit-solver-timing):
the 8 s task takes about 8.5–14 s of solve time with four workers, depending on
host load and temperature, so real time is not guaranteed. Budget the four
workers alongside the rest of the robot simulator.
