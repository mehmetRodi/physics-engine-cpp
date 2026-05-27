# Design Decisions

This file records project decisions that are easy to forget but important for
future design, implementation, and interview explanation.

Use this for choices where there is more than one reasonable option, especially
when the current implementation is intentionally simple but should grow into a
cleaner engine architecture later.

Each entry should explain:

- the decision
- the context that made the decision necessary
- the tradeoffs
- the current status, such as accepted, temporary, or needs revisit

Use this rough format for new entries:

```md
## Decision Title

Decision: what was chosen.

Context: why the choice came up.

Tradeoff: what this buys and what it costs.

Status: accepted, temporary, or needs revisit.

Future direction: what should change later, if anything.
```

## Project Scope

Decision: the project is a general C++20 physics simulation engine, not a
rigid-body-only engine.

Why now: rigid-body sphere simulation is the first implemented module because it
is the smallest useful path for learning integration, collision detection,
contact response, determinism, tests, and benchmarks. That should not constrain
the final product identity or force future APIs to assume every simulated object
is a rigid body.

Tradeoff: keeping the top-level scope broader requires discipline. The project
should not start adding particles, deformables, fluids, or many domain modules
before the current rigid-body foundation has clean timing, ownership, collision,
solver, test, and benchmark boundaries.

Future direction: keep rigid-body-specific names where they describe real code,
such as `RigidBody`, but make shared architecture names domain-neutral where
appropriate. Future simulation domains should plug into the same fixed-step,
headless-testable, benchmarkable architecture instead of becoming renderer-driven
demo code.

## Rigid-Body API Boundary On World

Decision: `World` remains the top-level simulation coordinator, but public APIs
for the currently implemented domain are named explicitly as rigid-body APIs:
`RigidBodyId`, `createRigidBody`, `rigidBody`, and `reserveRigidBodies`.

Context: Phase 3 is separating top-level simulation concepts from
rigid-body-specific state. The engine currently simulates rigid-body spheres
only, but future particles, constraints, soft bodies, or fluids should not be
forced through a generic "body" API that already means rigid body in practice.

Tradeoff: explicit rigid-body names make the current implementation honest and
leave room for other domains without adding an abstraction layer before it has
real users. The cost is a slightly more verbose API today.

Status: accepted for the current architecture checkpoint.

Future direction: add future domains through explicit domain storage and APIs
first. Introduce shared interfaces or plugin-style stepping only when two or
more implemented domains show the common requirements clearly through tests,
benchmarks, and ownership constraints.

## Fixed-Step Domain Integration Boundary

Decision: `World::step(dt)` represents one deterministic simulation tick. The
caller owns frame-time accumulation and should pass a fixed timestep for
replayable simulation. The visual demo may accumulate render frame time, but
that accumulator stays outside the core physics types.

Context: Phase 3 needs a clear stepping boundary before additional simulation
domains are added. The current rigid-body path integrates forces, updates body
state, builds collision proxies, finds sphere pairs, and resolves contacts
inside one `World::step` call. Tests, headless demos, and benchmarks already
drive the world with explicit timestep values.

Tradeoff: keeping `World::step` as a simple fixed-tick API is easy to test,
benchmark, and replay. It avoids hiding variable frame time or renderer behavior
inside the simulation core. The cost is that applications must manage their own
accumulators and choose a timestep policy explicitly.

Status: accepted for the current architecture checkpoint.

Future direction: future domains should join the same fixed-tick flow through
explicit world-owned storage and deterministic step functions. Do not introduce
a generic virtual module/plugin interface until multiple implemented domains
show the shared stepping, ownership, testing, and benchmark requirements.

## Vec3 Scalar Division Policy

Decision: `Vec3::operator/(float)` performs direct component-wise floating-point
division and assumes the scalar is nonzero.

Context: scalar division is needed for basic math completeness and lets
normalization reuse a single vector division path. The current implementation
already handles zero-length normalization before division, so the common
normalization edge case remains explicit and tested.

Tradeoff: direct division keeps the math primitive small, predictable, and
cheap. It does not silently clamp invalid input or return a made-up vector,
which avoids hiding physics bugs. The cost is that general division by zero is
not guarded yet and follows the platform's floating-point behavior.

Status: temporary. Valid scalar division is implemented and tested. General
division-by-zero policy still needs to be documented with the broader
floating-point assumptions work.

Future direction: decide whether invalid scalar division should remain a
documented precondition, gain debug assertions, or be handled by a named safe
helper. Avoid exceptions in core hot-path math unless the project deliberately
changes its error-handling policy.

## Floating-Point Reproducibility Policy

Decision: deterministic simulation results are guaranteed only for the same
source code, input data, platform, compiler, build type, and floating-point
compiler flags. Core simulation code currently uses scalar `float` arithmetic
and assumes normal IEEE-754 single-precision behavior from the compiler and
standard library.

Context: fixed-step replay tests protect same-build determinism, but bitwise
cross-platform reproducibility is a larger policy that depends on compiler
optimization choices, math-library behavior, CPU features, and future SIMD
paths. The current engine should be honest about the guarantee it can test
today.

Tradeoff: this keeps the implementation simple and inspectable while avoiding
false claims about cross-machine bit identity. It also rules out optimization
flags that let the compiler reassociate operations, assume finite values only,
replace division with approximate reciprocals, or otherwise ignore IEEE edge
behavior. Disallowed flags include `-ffast-math`, `-Ofast`,
`-funsafe-math-optimizations`, `-fassociative-math`, `-freciprocal-math`,
`-ffinite-math-only`, and MSVC `/fp:fast`.

Status: accepted for the current scalar engine. CMake rejects the known
non-reproducible flags above when they are passed through the standard
`CMAKE_CXX_FLAGS*` variables.

Future direction: if the project later needs stronger cross-platform replay,
add a replay hash/checkpoint format, document exact supported compiler versions
and CPU modes, and isolate any SIMD or approximate math behind tested,
determinism-aware build options.

## Restitution Mixing

Decision: use the maximum restitution value of the two colliding bodies for the
current simple sphere collision path.

Why now: this keeps restitution on bodies/materials instead of making it a
global `World` setting. It also gives intuitive simple behavior: a bouncy body
can still bounce when it hits a less bouncy or static body.

Tradeoff: `max` is a material policy, not a universal physics rule. Other
engines may use `min`, average, multiplication, or configurable combine modes.

Future direction: move restitution mixing out of `World::resolveContact` and
into a named contact/material path. A later design should have explicit contact
data, for example:

```cpp
struct Contact {
  World::RigidBodyId a;
  World::RigidBodyId b;
  Vec3 normal;
  float penetration;
  float restitution;
  float friction;
};
```

At that point, material policy can live in a small function such as:

```cpp
float mixRestitution(const BodyMaterial& a, const BodyMaterial& b);
```

That keeps `World` focused on simulation flow while contact generation prepares
solver-ready data.

## Identical-Position Dynamic Overlaps

Decision: the current simple sphere collision path does not resolve dynamic
bodies that have exactly identical positions.

Why now: when two sphere centers are identical, the contact normal is ambiguous.
Choosing an arbitrary normal inside `World::resolveContact` would make the simple
solver look more complete than it is and could hide a policy that belongs in
contact generation.

Tradeoff: overlapping bodies at identical positions can remain overlapped for
now. The current behavior is deliberate and tested: no ambiguous impulse is
applied.

Future direction: when contact generation becomes explicit, choose a
deterministic fallback normal or use previous-frame/contact-cache information to
recover a stable normal. That decision should live near contact creation, not in
the impulse solver.

## Initial Benchmark Harness

Decision: start with a small in-repo `std::chrono` benchmark executable for
`World::step` before adding Google Benchmark.

Why now: the first performance goal is to establish repeatable methodology and
latency distribution reporting, not to make optimization claims. A simple custom
harness can report per-step min, average, p95, p99, and max latency without
adding another dependency or hiding the measurement loop.

Tradeoff: Google Benchmark provides a mature framework for microbenchmarks and
should still be considered later. The custom harness is less feature-rich and
must be kept honest about warmup, build type, workload, and environment.

Future direction: once the project has several benchmark cases, either keep the
custom harness for simulation-level latency distribution tests or add Google
Benchmark for focused microbenchmarks such as `Vec3` operations, broadphase pair
generation, and solver kernels.

## Microbenchmark Result Consumption

Decision: initial math microbenchmarks consume their computed results by
accumulating them into a printed checksum.

Why now: a tight benchmark loop that computes values but never observes them can
be optimized away by the compiler, especially in Release builds. Printing a
checksum is a simple, inspectable way to make the work observable without adding
benchmark-framework-specific escape hatches.

Tradeoff: the checksum itself is not part of the intended math workload, so it
must be kept outside the timed region where possible or limited to simple
accumulation that represents realistic dependent work. The printed value is a
correctness guard, not a performance metric.

Future direction: if the project adopts Google Benchmark later, use its
`DoNotOptimize` and `ClobberMemory` helpers for focused microbenchmarks.

## Pair-Check Benchmark Scope

Decision: benchmark the current naive sphere pair-generation API in a standalone
benchmark instead of exposing `World::resolveCollisions` internals.

Why now: the goal is to measure candidate-pair scanning separately from
integration and collision response. Making private `World` methods public only
for benchmarking would weaken the simulation API, while a standalone benchmark
keeps the measured workload obvious and isolated.

Tradeoff: the benchmark is still a synthetic workload. It measures pair
generation cost through `findSpherePairs`, but it does not include integration,
contact resolution, or future broadphase/narrowphase data flow.

Future direction: once broadphase becomes a richer module, keep this benchmark
focused on the public pair-generation API and add separate benchmarks for
contact generation and solving.

## Pair Generation Output Buffers

Decision: provide both a convenient return-by-value `findSpherePairs` API and a
hot-path-friendly overload that writes into a caller-owned
`std::vector<CollisionPair>`.

Why now: pair generation is a simulation hot path. Returning a vector is easy to
use in tests and simple callers, but repeated calls can allocate or grow storage.
The output-buffer overload lets benchmarks and `World` stepping reserve capacity
once and reuse memory across iterations.

Tradeoff: the output-buffer API is slightly more verbose and requires callers to
understand that the function clears the output vector before writing. This is a
reasonable explicit ownership and allocation tradeoff for hot-path code.

Future direction: when broadphase becomes a richer module, keep allocation
behavior explicit. Consider preallocated pair storage, spans, or arena-backed
buffers only after the simpler reusable-vector API has measured limitations.

## World Collision Pair Discovery

Decision: `World::resolveCollisions` builds sphere proxies from the current body
storage, calls `findSpherePairs`, and then resolves the returned body-index
pairs.

Why now: this moves pair discovery out of `World` without changing the current
body ownership model. `World` remains responsible for simulation flow and contact
response, while `collision/` owns the deterministic sphere-pair query.

Tradeoff: `World` still builds rigid-body sphere proxies each step because the
current body storage is the source of truth. This is simple and correct for the
current engine shape, but it is not the final broadphase representation.

Future direction: as contacts become explicit solver data, move from direct pair
resolution toward a pipeline of body proxies, collision pairs, contact
manifolds, and solver constraints.

## Linear Damping Policy

Decision: apply linear damping as a per-body material property during rigid-body
velocity integration, clamping the damping scale to `[0, 1]`.

Why now: damping is part of the first rigid-body dynamics policy, and the engine
needs deterministic behavior for large timesteps or high damping values. Without
clamping, damping can reverse velocity, which is not the intended behavior.

Tradeoff: this is a simple explicit damping model, not a physically complete drag
or material interaction model. It is deterministic, easy to test, and appropriate
for the current scalar rigid-body module.

Future direction: when contacts, constraints, and richer materials are added,
revisit whether damping belongs directly on the body, in material data, or in an
integrator policy shared by multiple simulation domains.
