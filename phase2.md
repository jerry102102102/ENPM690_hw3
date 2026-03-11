ENPM690 Homework 3 – Phase 2 Implementation Instructions

You are implementing Phase 2 on top of the existing Phase 1 repository.

Do not delete or break any working Phase 1 functionality.
Keep Phase 1 teleoperation fully intact so the user can still launch the world and manually play one round.
Phase 2 must add:

A shark-hunt game scenario in Gazebo.

Rule-based prey species behavior.

A rule-based autonomous shark baseline controller.

A Gymnasium-compatible RL environment for later training.

A headless training mode and a visible demo/play mode.

Score, timer, catch logic, and reset logic.

Primary design decision:
The robot remains the TurtleBot3 Burger from Phase 1.
The robot is the shark.
The fish are not new ROS robots.
They are lightweight game actors managed by a Python scenario manager and visualized in Gazebo as simple colored entities.
Their authoritative state lives in Python.
Gazebo is used for robot simulation, obstacle world, LiDAR, odometry, visualization, and optional fish visuals.
Do not attempt to make each fish a fully physical autonomous robot in Phase 2.

Important scope rule:
Phase 2 is for:

a complete playable/demoable shark-vs-fish scenario,

a working autonomous baseline,

an RL-ready training environment for the shark only.
Phase 2 is not for:

evolving fish populations,

multi-agent learning for the fish,

recurrent memory models,

large-scale distributed training.
Those belong to Phase 3.

Overall game concept:
The TurtleBot3 shark hunts fish in an obstacle-filled arena for 30 seconds.
Catching fish yields points.
The robot uses LiDAR for obstacle awareness.
The game manager controls fish spawning, movement, scoring, episode timing, and reset.
The same environment must support:

teleop play,

rule-based autonomous play,

RL training play.

Game entities and counts:
Use exactly these default counts in Phase 2:

Tuna: 5

Sardines: 10

Seaweed: 8
Total: 23 entities

Species definitions:

Tuna

Highest value prey

Larger than sardines

Fast

Mostly inertial forward motion with mild heading drift

Score = 10

Speed = 0.8 × shark max linear speed

Sardines

Medium value prey

Medium size

Medium speed

Group / schooling tendency

Try to avoid the shark when it is near

Score = 3

Speed = 0.4 × shark max linear speed

Seaweed

Lowest value prey/object

Smallest

Static

Spawn only near obstacles and walls

Score = 1

Speed = 0

Shark rules:

Use the existing TurtleBot3 Burger robot.

Keep teleop support from Phase 1.

Add autonomous control modes without removing teleop.

The shark has a maximum linear speed and angular speed.

The shark must be faster than tuna and sardines.

When the shark collides with a wall or obstacle, it becomes stunned for 1.0 second:

publish zero velocity during stun,

apply a collision penalty in training,

keep the robot in place or allow physics stop naturally,

do not let the controller keep issuing effective movement during cooldown.

Episode rules:

One episode = 30.0 seconds simulated time.

Score accumulates across the episode.

Catching a fish immediately gives points.

Caught fish respawn to maintain a constant population.

At timeout, the episode ends.

For RL, timeout must be treated as truncation, not a failure termination, because this is a fixed time-limit episode. This matches SB3/Gymnasium guidance on handling time limits.

High-level package strategy:
Do not overwrite the working Phase 1 package.
Add a new Phase 2 package, for example:

enpm690_hw3_phase2

Phase 2 package responsibilities:

game manager

fish manager

prey behaviors

autonomous shark baseline controller

RL observation builder

Gymnasium environment wrapper

PPO training script

demo/play launch files

headless training launch files

RViz markers / score display helpers

Recommended file structure:
repo root

existing Phase 1 package unchanged

new Phase 2 package:

launch/

phase2_demo.launch.py

phase2_play_teleop.launch.py

phase2_play_auto.launch.py

phase2_train.launch.py

phase2_eval.launch.py

config/

phase2_params.yaml

rl_params.yaml

worlds/

phase2_ocean_arena.sdf

rviz/

phase2.rviz

models/

tuna_simple/

sardine_simple/

seaweed_simple/

enpm690_hw3_phase2/

game_manager.py

fish_manager.py

prey_behaviors.py

shark_auto_controller.py

shark_collision_monitor.py

observation_builder.py

reward_builder.py

gym_env.py

train_ppo.py

eval_policy.py

marker_publisher.py

geometry_utils.py

spawn_utils.py

constants.py

README_phase2.md

World design:
Create a new Phase 2 world file instead of mutating the Phase 1 obstacle world in place.
The world should still be obstacle-based, but visually themed as an arena/ocean if desired.
Requirements:

keep boundaries

keep several obstacles

leave open corridors for shark movement

ensure there are obstacle-adjacent regions where seaweed can spawn

ensure enough open area for tuna and sardine movement

initial shark spawn must be in a clear zone

Do not make the world huge.
Keep it compact enough that a 30-second episode produces multiple catches.

Fish visualization:
Represent fish as simple SDF entities:

tuna: larger blue or dark-cyan sphere/cylinder

sardines: medium yellow or silver sphere/cylinder

seaweed: small green sphere/cylinder
These do not need full articulated models.
Keep them visually simple and computationally cheap.

Authoritative fish state:
The authoritative fish state must be maintained in Python, not inferred from Gazebo.
For each fish store:

unique id

species type

position (x, y)

heading

scalar speed

radius

alive / active flag

stun or respawn timer if used

optional school_id for sardines

In demo/play mode:

sync fish state to Gazebo visuals.
In headless training mode:

it is acceptable to keep Gazebo fish visuals disabled or updated at a lower frequency, as long as the logical fish state is correct for scoring and RL observation.

How to manage fish entities:
Use Gazebo’s ROS 2 simulation interfaces for spawn / delete / set entity state when practical. Gazebo documents ROS-side services for reset, step, spawn, delete, query, and set entity state; these are the preferred control points for a simulator-driven RL environment.

Fish motion update model:
Use a purely kinematic 2D update for fish.
Do not use complex underwater physics.
At each fish update:

compute desired heading / steering based on species behavior

clamp turn rate

integrate heading

integrate position

resolve wall/obstacle collisions

resolve fish-fish collisions

sync visual pose if needed

Collision behavior for fish:
Fish-fish collisions:

simple elastic or semi-elastic bounce

separate overlapping bodies

reflect or perturb headings

Fish-wall / fish-obstacle collisions:

bounce / reflect heading

push the fish back outside the obstacle boundary

do not let fish remain embedded in obstacles

Seaweed:

static

no motion update

can be “caught” by the shark

respawn near obstacles after being eaten

Spawn rules:
Tuna:

spawn in open water, away from walls and obstacles

do not spawn inside the shark catch radius

Sardines:

spawn in open water, loosely clustered into schools

two schools of 5 is a good default

Seaweed:

spawn only near obstacles or near walls

use a valid spawn band such as 0.20 m to 0.45 m from obstacle edges

do not let seaweed overlap obstacles or walls

Game manager:
Implement one main node/class that owns the episode state.
Responsibilities:

start/reset episode

maintain score

maintain timer

spawn fish

update fish logic

detect catches

handle respawns

publish game status

coordinate play mode vs auto mode vs training mode

Publish at least:

current score

time remaining

current mode

catch events in readable logs

optional per-species catch counts

Use simple standard ROS messages if possible:

score: std_msgs/Int32

time remaining: std_msgs/Float32

mode: std_msgs/String

optional catch event log: std_msgs/String
Avoid introducing custom ROS messages unless truly necessary.

Modes:
Implement these modes and make them switchable by launch arguments:

teleop_play

Gazebo GUI on

RViz on

fish visuals on

shark controlled by existing teleop

score/timer active

auto_play

Gazebo GUI on

RViz on

fish visuals on

shark controlled by baseline autonomous controller

score/timer active

train_headless

server-only or headless Gazebo

no GUI

RViz off

fish visuals optional or low-frequency

shark controlled only through the Gym environment / RL loop

score/timer active

eval_demo

GUI optional

load a saved policy

play a visible round

no training updates

Headless training requirement:
Training mode should prefer Gazebo server-only launch or headless rendering mode.
ros_gz_sim provides a server-only launch path, and Gazebo also supports --headless-rendering for remote/headless sensor use. Use the simpler server-only path by default; only add headless rendering if needed by later sensors.

Shark autonomous baseline:
Implement a hybrid reactive controller:

target-seeking term

LiDAR-based obstacle avoidance term

collision stun handling

optional wall-escape term

This baseline is not the final RL policy.
It is a sanity-check controller and a playable autonomous behavior for the assignment demo.

Target selection for baseline:
At each control cycle:

find the nearest active tuna

find the nearest active sardine

find the nearest active seaweed

compute a utility score for each candidate type

Recommended utility:
utility = species_score / (distance + epsilon)

Choose the candidate with the highest utility.
This naturally prioritizes tuna when reasonably reachable, while still allowing nearby lower-value objects to matter.

Target features:
For the chosen target compute:

relative distance

relative bearing in shark frame

Obstacle avoidance:
Compute obstacle proximity from LiDAR front sectors.
Use the LiDAR to keep the shark away from walls and obstacles, while the target term steers it toward prey.

Baseline control law:
Use this conceptual form:

target_turn = k_target * target_bearing

avoid_turn = k_avoid * (right_proximity - left_proximity)

angular_z = clamp(target_turn + avoid_turn, -max_angular_speed, +max_angular_speed)

linear_x = max_linear_speed * speed_factor_from_front_clearance

if stunned: linear_x = 0 and angular_z = 0

This is a good hybrid because it preserves the spirit of reactive obstacle avoidance while making the shark chase prey.

LiDAR preprocessing for baseline and RL:
Do not feed raw LaserScan.ranges directly into the agent as-is.
Build a fixed-size vector.

Preprocessing steps:

read ranges

replace NaN and inf with range_max

clip to [range_min, range_max]

divide the scan into a fixed number of angular bins

for each bin, use min-pooling

normalize to [0, 1]

Recommended default:

use 24 bins across the full scan

convert each bin to proximity:
proximity = 1.0 - clipped_distance / range_max

Interpretation:

0.0 means far / clear

1.0 means very close obstacle

Reasoning:
A fixed-size normalized vector is much easier for MLP policies, and it follows SB3 guidance to normalize observations whenever bounds are known. The specific 24-bin min-pooled design is the project-specific choice for this environment.

Sector extraction from LiDAR:
Reuse the same 24-bin vector to derive:

left-front proximity

center-front proximity

right-front proximity

Do not maintain two separate LiDAR representations.
One canonical preprocessed LiDAR vector should feed both:

the baseline controller

the RL observation builder

RL observation design:
Even though LiDAR is required, LiDAR alone is not enough for a meaningful prey-hunting task because LiDAR does not naturally encode prey type or score priority.
Therefore the Phase 2 RL observation must include:

shark kinematic state

preprocessed LiDAR

compact prey-state features from the game manager

Use this observation vector:

shark position x normalized to world bounds

shark position y normalized to world bounds

cos(shark_heading)

sin(shark_heading)

normalized linear speed

normalized angular speed

normalized collision cooldown remaining

normalized time remaining

24-bin normalized LiDAR proximity vector

nearest tuna relative distance normalized

sin(relative bearing to nearest tuna)

cos(relative bearing to nearest tuna)

nearest sardine relative distance normalized

sin(relative bearing to nearest sardine)

cos(relative bearing to nearest sardine)

nearest seaweed relative distance normalized

sin(relative bearing to nearest seaweed)

cos(relative bearing to nearest seaweed)

Optional extra feature:

normalized current score
This is optional; default to excluding score from the observation unless debugging shows it helps.

Do not include raw heading angle directly.
Use sin/cos to avoid wraparound discontinuity.

No memory requirement:
Do not use recurrent policies in Phase 2.
Do not implement an explicit memory module.
Do not frame-stack by default.
If later experiments show strong partial observability issues, allow an optional 2-frame stack wrapper, but keep it off by default. SB3 notes that if delays or hidden state break the Markov assumption, a history input may become useful; do not build that complexity into Phase 2 unless needed.

Action space for RL:
Use a continuous 2D action space with symmetric normalized bounds:

action[0] in [-1, 1] = throttle command

action[1] in [-1, 1] = turn command

Rescale internally to:

linear_x in [0, shark_max_linear_speed]

angular_z in [-shark_max_angular_speed, +shark_max_angular_speed]

Map throttle as:
linear_x = 0.5 * (action[0] + 1.0) * shark_max_linear_speed

Map turn as:
angular_z = action[1] * shark_max_angular_speed

Do not use reverse motion for the RL shark in Phase 2.
Teleop mode may still allow reverse for manual play.
For the RL policy, forward-only motion makes the shark behavior cleaner and the task easier.
This action design follows SB3 guidance to use normalized bounded continuous actions and keep them symmetric when possible.

Gymnasium environment:
Implement a custom environment class that follows the Gymnasium API:

define observation_space

define action_space

implement reset()

implement step()

return obs, reward, terminated, truncated, info

Gymnasium’s custom env documentation explicitly expects observation and action spaces plus reset/step structure; build exactly to that interface.

Environment timing:
Use a fixed control step.
Recommended defaults:

control rate = 10 Hz

dt = 0.1 s per RL step

episode duration = 30.0 s

total decision steps per episode = 300

Simulation control in training:
Prefer deterministic step-based control.
Use Gazebo’s ROS 2 simulation interfaces to:

reset simulation at episode start

pause/play or step simulation in fixed increments

optionally set entity states directly during reset
Gazebo documents reset, step, and simulation state services and actions on the ROS side. Use them so the env step size is explicit and repeatable.

Training loop architecture:
Implement the environment so that one env step does:

receive normalized action

convert to /cmd_vel

apply shark action

update fish logic for one control interval

advance simulation by the corresponding fixed time step

read fresh shark state and LiDAR

compute catches, score updates, collision status

build next observation

compute reward

return done flags and info

Collision detection for shark:
Do not rely on a missing bumper topic.
Implement geometric collision checks in the game manager using:

world boundaries

known obstacle geometry

shark radius
This is robust and keeps the collision penalty under your control.

Shark collision handling:
On shark collision with wall/obstacle:

set collision cooldown timer = 1.0 s

issue zero velocity while cooldown > 0

apply reward penalty

log collision event

keep episode running

Catch detection:
A catch occurs when shark center is within:
catch_radius + fish_radius
of an active fish center.

Recommended default:

shark catch radius: 0.18 m
Tune if needed to feel fair in gameplay.

When caught:

add points

publish/log catch event with species and new score

mark fish inactive

respawn fish after a short delay or immediately
Recommended default:

respawn after 0.25 s logical delay in play modes

immediate respawn is acceptable in training mode for stable density

Reward function:
Use shaped reward.
SB3 explicitly recommends starting with informative shaped rewards for custom problems.

Recommended reward:

+10 for catching tuna

+3 for catching sardine

+1 for catching seaweed

-2.0 for shark collision with wall/obstacle

-0.01 per step time penalty

+0.05 × positive reduction in distance to current selected target

-0.05 × increase in distance to selected target

-0.1 × front_obstacle_proximity when very close to collision range

optional +0.01 survival / movement bonus if the shark is moving productively

Reward design notes:

keep point rewards dominant

keep shaping terms small

never let the dense shaping dominate tuna catches

make collision clearly bad but not immediately terminal

Episode end flags:

terminated = True only for simulator failure, invalid state, or unrecoverable environment error

truncated = True at 30-second timeout
This separation is important for correct RL bookkeeping.

Prey behaviors in detail:
Tuna behavior:

mostly inertial

maintain heading

low-frequency random heading perturbation

low max turn rate

bounce off walls and obstacles

no sophisticated evasion

optional mild flee response if shark is extremely close, but keep it weak

Sardine behavior:

implement simplified boids-like schooling

separation: avoid crowding neighbors

cohesion: move toward school centroid

alignment: roughly match neighbor heading

flee: if shark within flee radius, add a strong away-from-shark steering term

clamp to medium speed and limited turn rate
Keep the implementation simple and stable; no need for a biologically perfect model.

Seaweed behavior:

static

obstacle-adjacent spawn only

respawn near obstacle after being caught

Recommended update rates:

game logic update: 10 Hz

fish visual sync in demo/play: 10 Hz

marker publication: 5–10 Hz

shark baseline controller: 10 Hz

RViz / visual feedback:
Phase 2 demo mode should provide clear visualization.
At minimum:

keep existing LiDAR and robot state displays from Phase 1

add fish markers with species color coding

add a text marker or terminal display for score and time remaining

optionally show the current baseline target with a marker or line

Use visualization_msgs/MarkerArray for:

fish positions

target highlight

score text

time remaining text
This makes the demo much easier to understand.

Preserve teleop:
Do not delete or replace the Phase 1 teleop node.
Instead:

keep teleop for teleop_play

add Phase 2 game manager around it

ensure score/timer/fish still run during teleop play
The user must be able to manually play a 30-second round.

Training library choice:
Use Stable-Baselines3 with PPO as the default Phase 2 training baseline.
Do not introduce multiple RL libraries.
Do not compare algorithms in Phase 2.
One stable baseline is enough.

Environment validation:
Before any training:

run check_env on the custom environment

run a random-action smoke test

run a short scripted-action smoke test
SB3 provides check_env specifically for custom env validation.

Training defaults:
Use conservative PPO defaults.
Do not spend time on aggressive tuning in Phase 2.
The goal is to produce a working trainable setup, not to maximize final score yet.

Training artifacts:
Save:

trained model checkpoints

training logs

evaluation summary JSON or CSV

optional TensorBoard logs

best model path

Evaluation:
Implement a separate evaluation script that:

loads a saved model

runs N evaluation episodes

reports average score

reports average tuna/sardine/seaweed catches

reports collision count

reports episode length

optionally runs a visible demo round

Launch requirements:
Create these launch entry points:

phase2_play_teleop.launch.py
Starts:

Phase 2 world

shark

bridge

robot state publisher / TF support

game manager

fish marker publisher

RViz
Does not automatically start keyboard teleop if stdin handling is fragile; document if teleop should run in a separate terminal.

phase2_play_auto.launch.py
Starts:

all of the above

baseline autonomous shark controller
No teleop required

phase2_train.launch.py
Starts:

Gazebo server only or headless

bridge

robot state support

game manager in training mode

no RViz

no GUI
This should be callable from the training script or as a separate launch

phase2_eval.launch.py
Starts:

environment in evaluation mode

optional GUI

saved policy playback

Reuse from Phase 1:

existing bridge patterns

/scan

/odom

/tf

teleop node

working robot spawn

working RViz basics

Do not fork or duplicate Phase 1 unnecessarily.
Import and reuse the working pieces where possible.

Logging requirements:
Keep logs readable.
Examples of useful logs:

[GAME] episode started

[GAME] tuna caught, +10, score=27

[GAME] shark collision, stun=1.0s

[AUTO] target=tuna_3 dist=1.42 bearing=-0.37

[TRAIN] episode=12 score=34 catches={tuna:2,sardine:3,seaweed:5}
Avoid spamming full arrays to the console.

Definition of done for Phase 2:
Phase 2 is complete only when all of the following are true:

Phase 1 teleop still works.

The new Phase 2 world launches.

Fish entities or fish markers appear correctly in demo mode.

A 30-second teleop game can be played with scoring and catches.

The baseline autonomous controller can complete a visible 30-second round.

LiDAR is still active and influences the autonomous controller.

The custom Gymnasium environment passes check_env.

A PPO training run can start in headless mode.

A saved policy can be loaded for evaluation.

The fish remain rule-based in Phase 2; no fish learning is implemented yet.

Non-goals:
Do not implement:

fish evolution

fish neural policies

memory / recurrence

domain randomization

curriculum learning

distributed rollouts

camera observations

image policies

Docker changes
Keep Phase 2 focused and stable.

Final design note:
The assignment’s sensor requirement is satisfied by LiDAR-based obstacle awareness.
The hunting objective is provided through the game-state prey features.
This hybrid design is intentional:

LiDAR keeps the robot physically safe in the obstacle arena,

prey features make the scoring task learnable,

teleop and autonomous play remain easy to demo,

the environment remains simple enough to train and extend later.