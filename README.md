# Vortix Physics Engine

Vortrix Physics: A real-time 3D rigid body physics engine for games and simulation, designed for multicore scalability.

<table align = "center"> 
  <tr>
      <td>
          <a href="https://youtu.be/Q1xmI2qV5-k"><img src = "https://img.youtube.com/vi/Q1xmI2qV5-k/hqdefault.jpg"/a>
      </td>
  </tr>
  <tr>
     <th>
         <i>Sample video (youtube)</i>
     </th>
  </tr>
</table>
            
<!--More [sample videos]()-->

## Features 
* Physics & Collision
  * 3D rigid body simulation with symplectic Euler integration.
  * Analytical shapes: Sphere, Box, Capsule, Plane
  * Dynamic BVH Broadphase: SAH rebuilds
  * Constraint: contact(non-penetration and friction), distance, point
  * Sequential Impulse (PGS) solver with warm starting
  * Multicore island solving
  * Symplectic Euler body integration
  * World Queries: Raycasting
  * Ragdoll

* System & Performance
  *  Task coordination with a lock-free CAS work queue
  *  SIMD math (Vec3/Vec4/Mat44 intrinsics)
  *  Data-oriented design
  *  Reduce allocation during step loop via scratch allocator

* Sample Framework
  * OpenGl renderer (GLFW/GLEW), ImGui editor
  * Scenario system with JSON serialisation. 

## Quick
### Quick Launch
- Building [see](#build).
- Launching Release build, <a href="Docs/Vortrix Physics Quick Manual – Navigating the Application.pdf">see sample framework navigation</a>

### Quick Manual 
 For detailed physics engine documentation, see the ["Documentation and Performance"](#documentation-and-performance) section, for a quick physics engine setup:
 - <a href="Docs/Vortrix Physics Quick Manual – Installation (Project Generation and Compilation).pdf">Installation of the physics engine</a>, either source code or launching the provided executable
 - <a href="Docs/Vortrix Physics Quick Manual – Navigating the Application.pdf">Navigating the physics engine sample framework</a>

## Documentation and Performance
### Architecture
This is an architectural overview of the simulation pipeline, with multi-threaded components, designed around data-oriented, multicore island solving. 
<img src = "Docs/Physics Engine Architecture.png"/>
* **The Physics World Simulation Step:** The global frame step entry point `PhysicsWorld::StepSimulation`. The execution pipeline is broken down into parallelised, decoupled stages where specific phases run concurrently alongside each other.
* **Broadphase Compute Phase:** Processes dirty body tracking nodes from the previous step, executing dynamics BVH tree bounding refitting(or Surface Area Heuristic tree rebuilding if an imbalance threshold is triggered) and executes potential intersecting body bounding pairs.
* **Acceleration Integration & Pair Processing:** Integrates active bodies' acceleration along with acceleration due to gravity. This **runs concurrently in parallel batches alongside the Broadphase Compute phase**, while one thread processes the *Broadphase Compute Phase*, the remaining worker threads execute *acceleration integration*.
* **Narrowphase/Pair processing:** Executes exact analytical shape intersection checks on the *Broadphase pair's*. If intersection occurs, it handles contact constraints and constructs simulation islands by linking bodies and constraints in parallel batches.
* **Contact constraint processing:** Construction of contact constraints; island construction by linking bodies along constraints on active non-contact constraints.
* **Finalising islands:** Normalises and maps all Linked Bodies and constraints, then assigns bodies, contact constraints, and non-contact constraints into isolated simulation island indices. Islands are sorted from largest to smallest to minimise execution tail latency, splitting blocks that exceed a large-island capacity threshold.
* **Warm Start:** Sequentially re-applies the previous step's accumulated impulses (caches `lambda` values) to the solver bodies before iteration begins, so the solver converges in fewer passes. *Currently run as a single sequential pass ahead of the parallel island solve, per-batch warm starting inside island tasks is disabled pending a race condition fix [see Future Prospects](#future-prospects)*.
* **Solving Island Velocity Constraint:** Dispatches sorted simulation islands concurrently across available worker threads using atomic indices, splitting heavy island workloads into mask-based independent task batches.
* **Write Back Bodies:** Writes solved linear/angular velocities back to the owning physics bodies.
* **Updating bodies Island activation state:** Iterates through active island groups to assess rest velocity boundaries, putting entire non-moving body island batches to sleep, preserving CPU execution cycles.
* **Velocity Integration:** Integrates body velocities in parallel batches to update states(position and orientation), clamping speeds to prevent simulation explosion.
* **Solving Island Position Constraint:** Positional correction of sorted island constraints in parallel, ensuring non-contact limits are processed before contact constraints for contact to have a positional priority.
* Per-step working memory (solver rows, broadphase pairs, island bins) is allocated from a pre-reserved bump allocator, zero heap allocations during the simulation step.

<!--**For More Architecture Details, See <s>[Full ARCHITECTURE](Docs/ARCHITECTURE.md)</s>.**-->
**For More Architecture Details, See *Coming Soon Full ARCHITECTURE*.**

### Motivation
Vortrix started as a personal deep-dive into physics simulation, building on a mechanical engineering background and prior graphics work (OpenGL, Vulkan). The broadphase BVH was adapted from an earlier [mesh ray tracer](https://www.youtube.com/watch?v=4LQCXBlVSwc) I built with an OpenGL compute shader; a transfer of the same acceleration structure technique from graphics to physics. The engine later became the experimental platform for my MSc dissertation, *Scalability of Task-Based Island Solvers under Non-Uniform Workload in Real-time Rigid Body Simulation*, which motivated its multicore island-solving architecture.


### Future Prospects
The long-term roadmap for Vortrix is partitioned across physics optimisation, diagnostics tooling, and game AI subsystem; extending the same scalability investigation themes beyond the solver. 

* **Core Physics & Parallel Optimisation**
  * **Multithreaded warm starting:** Resolve the existing data race on shared `SolverBody` velocities so the current sequential warm-start pass can move into per-batch island tasks. Also `CommitStateConstraint` lambda persistence so non-contact constraints benefit from warm starting, not just contacts.
  * **Split-island position correction:** Extend the velocity solver's `Splitter` batching to the positional correction phase, which currently claims whole islands atomically, removing the tail-latency bottleneck when one worker is left holding the largest island.
  * **Parallel broadphase:** Parallelise BVH dirty-node traversal and pair generation `BVHBroadphase::ComputeCollidingPair` currently runs on a single worker while the rest integrate velocities.
  * **Convex hull collision (GJK/EPA):** Implement a Gilbert-Johnson-Keerthi distance solver and Expanding Polytope Algorithm pipeline for contact manifold generation on arbitrary convex shapes, as a fallback beyond the closed analytical shape set (sphere, box, capsule, plane).
  * **Shape casting (`CastShape`):** Extend the `WorldQuery`/`Broadphase` query interface with swept shape tests alongside the existing `CastRay`, the foundation for CCD, and independently useful for spawn-safety and character controller overlap checks.
  * **Continuous collision detection (CCD):** Build time-of-impact(TOI) sweeps on top of shape casting to mitigate fast-body tunnelling through thin geometry.

* **Diagnostic & Renderer Extensions (`SampleFramework`)**
  * Framework code cleanup and reorganisation
  * **Order-independent transparency (OIT):** Implement a compute-driven OIT buffer for correctly sorted alpha rendering; required because the renderer's single instanced draw call per mesh/geometry type forbids CPU-side draw-order sorting.
  * **Screen-space ambient occlusion (SSAO):** Add an SSAO pass to the debug renderer to give penetration and contact stacking clearer visual depth cues during inspection.

* **Game AI**
  * **Runtime navmesh generation:** Cook pathfinding surface directly from static broadphase geometry outputs rather than a separate mesh pipeline.
  * **Steering & Perception:** Raycast-based perception (via `WorldQuery`) and obstacle-avoidance steering for actors/gameobject interacting with dynamic bodies.

* **Long-Term Exploration**
  * **Soft body dynamics:** Cloth and deformable bodies via XPBD of shape-matching constraints.
  * **Mesh destruction:** Fracture and sub-shape generation for breakable rigid bodies.
  * **GPU compute simulation (Vulkan):** Prototype Vulkan compute offloads of the `ParticleWorld` kernel (integration, spring constraints, particle collision). The subsystem's embarrassingly parallel per-particle workload makes it the natural first GPU target, ahead of mesh destruction, cloth, and hair.

  
## Build
Quick building
* Requires Visual Studio 2022 or 2026
* Requires CMake 3.18+
* Run `generate_vs2022.bat`, `generate_vs2026.bat` respectively.
* Build without pre-configured `generate_vs2022.bat` & `generate_vs2026.bat`
  * Command line `cmake -S . -B build -G "Visual Studio 17 2022"`, `cmake -S . -B build -G "Visual Studio 18 2026"` respectively.

See more details in <a href="Docs/Vortrix Physics Quick Manual – Installation (Project Generation and Compilation).pdf">"Quick Manual - Installing"</a>

