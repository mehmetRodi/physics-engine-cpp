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

## RigidBodySystem Owns Rigid-Body State

Decision: rigid-body storage, integration, sphere proxy generation, collision
pair buffers, and contact response live in `RigidBodySystem`. `World` owns
world-level policy such as gravity and delegates rigid-body stepping through
`RigidBodySystem::step(dt, gravity)`.

Context: Phase 3 separates top-level simulation concepts from
rigid-body-specific state. The previous `World` implementation directly owned
rigid bodies, collision proxy buffers, pair buffers, and contact resolution.
That made `World` look like a rigid-body-only container even though the project
is intended to grow into additional simulation domains.

Tradeoff: `RigidBodySystem` is more specific than a generic simulation-module
interface, which keeps the code simple and avoids virtual dispatch or plugin
machinery before there is a second domain. It is broader than
`RigidBodyStorage` because the moved responsibility is not only storage; it is
the current rigid-body update pipeline.

Status: accepted for the current architecture checkpoint.

Future direction: if a particle system or another domain is added, give it an
explicit system with its own storage, deterministic step, tests, and benchmarks.
Only extract a shared domain interface after multiple real systems reveal common
requirements.

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

## Demo Inspection Controls Stay Outside Core Physics

Decision: visual demos should grow into proper physics inspection tools with
camera/navigation controls and immersive interactions, but those controls remain
outside the core simulation modules.

Context: the project should function as a physics engine that can be used to
experiment with and inspect physical behavior. That requires more than a static
debug view: users need to pan, zoom, reset the view, pause, single-step, adjust
playback speed, spawn or select objects, and inspect trails, contacts,
constraints, and solver behavior as those systems are implemented.

Tradeoff: richer demo controls make physical behavior easier to understand and
improve portfolio presentation. The cost is that demo code can drift toward game
or editor scope if the boundary is not kept clear.

Status: accepted as demo-layer architecture guidance. Current controls are
limited and should be documented as implemented or planned rather than implied
as complete.

Future direction: keep camera state, input handling, overlays, and scene
interaction in demo/rendering code. Core physics should expose deterministic
state and debug data through explicit APIs, but must not depend on SFML or any
interactive renderer.

## Shared Simulation Concepts Across Domains

Decision: shared simulation concepts are handles, materials, contacts,
constraints, integrators, and solver data. These names describe architecture
boundaries, not a claim that every concept is implemented today.

Context: Phase 3 needs to define which ideas should remain domain-neutral as
the engine grows beyond rigid-body spheres. The current implementation has
rigid-body handles and a simple body material. Contacts are still implicit in
`World::resolveContact`, constraints are not implemented, and solver data has
not been separated from collision response yet.

Tradeoff: documenting these concepts early helps keep future features from
landing in arbitrary places, especially material mixing, contact generation,
constraint solving, and integration policy. The cost is that the project must
avoid overbuilding empty abstractions before tests and benchmarks need them.

Status: accepted as architecture guidance.

Future direction: keep handles domain-specific until a shared handle model is
justified by more than one owned domain. Move material mixing into named contact
or material policy code when contact data becomes explicit. Represent contacts
and constraints as compact solver-facing data before adding richer collision
features. Keep integrators deterministic and benchmarkable, with ownership and
allocation behavior visible at the world/domain boundary.

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

## Reserved Rigid-Body Step Allocation Policy

Decision: `World::reserveRigidBodies(capacity)` preallocates current
rigid-body hot-path storage for bodies, sphere proxies, and worst-case naive
sphere collision pairs. For a reserved world that does not exceed that body
capacity, `World::step(dt)` is expected not to allocate heap memory in the
current rigid-body sphere path.

Context: Phase 4 focuses on making data ownership and allocation behavior
visible before introducing more complex data-oriented layouts. The current
collision path is an all-pairs sphere check, so worst-case collision-pair
storage for `n` bodies is `n * (n - 1) / 2`.

Tradeoff: reserving worst-case pair capacity makes the hot-path allocation
behavior simple and testable, but it can use much more memory than a sparse
scene needs. This is acceptable for the current baseline because the broadphase
is intentionally simple and measured optimization has not yet justified a more
complex structure.

Status: accepted for the current rigid-body sphere implementation and protected
by a test that counts allocations during `World::step`.

Future direction: replace worst-case pair preallocation with cache-friendly
contact and broadphase storage once Phase 5 introduces a real broadphase and
Phase 6 separates solver-facing contact data. Keep deterministic iteration
order and no-allocation hot-path tests when changing the storage model.

## Rigid-Body Contact Storage

Decision: rigid-body sphere contacts are stored as compact `RigidBodyContact`
records containing body handles, contact normal, penetration depth, and mixed
restitution. Contacts are generated in collision-pair order and resolved by
looking up the live bodies from the stored handles.

Context: the previous collision path resolved each `CollisionPair` immediately
and recomputed contact geometry inside the resolver. Phase 4 needs an explicit
solver-facing contact buffer before broadphase, narrowphase, and constraint
solver work expands the pipeline.

Tradeoff: contacts still reference array-backed body storage during resolution
because positions and velocities must be mutated. This keeps contact records
small and cache-friendly, but the current solver still gathers body state by
handle instead of operating on a fully packed constraint batch.

Status: accepted for the current rigid-body sphere path. A focused test checks
that generated contacts preserve deterministic collision-pair order and store
the expected contact data.

Future direction: when Phase 6 introduces solver constraints, split contact
generation from constraint preparation. Keep contact iteration deterministic,
and only move additional fields into contact records when benchmarks or solver
requirements justify the extra memory bandwidth.

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

## Structure-Of-Arrays Migration Timing

Decision: defer moving rigid-body hot state from the current `std::vector<RigidBody>`
array-of-structs layout to structure-of-arrays storage until benchmarks show
body-state iteration is a material bottleneck.

Context: Phase 4 asks for data-oriented engine shape, but layout changes should
follow measurement. A Release benchmark on Apple M4 with AppleClang 21.0.0
measured `World::step` for 1024 bodies at about 565306 ns average after the
AABB broadphase baseline was introduced. The sparse 1024-body AABB broadphase
case takes about 556426 ns per iteration. The current full-step benchmark is
therefore still dominated by O(n^2) pair scanning rather than by the body object
layout.

Tradeoff: keeping AoS storage is simple, readable, and compatible with the
current public `RigidBody&` access API. It may leave integration-only cache
efficiency on the table, but changing the layout now would add API and wrapper
complexity without evidence that it improves the measured hot path.

Status: accepted as a temporary policy. The current evidence does not justify a
SoA migration for rigid-body state.

Future direction: add focused integration, contact generation, and solver
benchmarks as those systems mature. Revisit SoA when a benchmark isolates body
state iteration as a significant cost or when public wrappers can be kept from
hiding hot-path gather/scatter overhead.

## World Collision Pair Discovery

Decision: rigid-body collision discovery now builds AABB proxies from the
current body storage, generates deterministic AABB broadphase candidate pairs,
then runs sphere narrowphase/contact generation over those candidates.

Why now: Phase 5 needs a correct broadphase baseline before optimizing with
sweep-and-prune, BVH, or another spatial structure. A conservative AABB
broadphase can produce false positives, such as touching sphere bounds, while
the sphere narrowphase still decides whether an actual contact exists.

Tradeoff: the current AABB broadphase is still O(n^2), so it is not faster by
design. Its value is architectural: it separates broadphase candidate generation
from narrowphase contact creation while preserving deterministic pair order and
the no-allocation reserved hot path.

Future direction: benchmark this baseline across body counts and distributions,
then compare optimized broadphase options against it. Keep the contact pipeline
conservative: broadphase may over-report candidates, but it must not miss pairs
that narrowphase should inspect.

## Baseline AABB Broadphase Benchmark

Decision: add `aabb_pair_bench` to measure the deterministic O(n^2) AABB
broadphase across 128, 512, and 1024 bodies using sparse-grid, dense-grid, and
all-overlapping distributions.

Why now: Phase 5 needs a baseline before replacing the all-pairs broadphase with
sweep-and-prune, BVH, or another spatial structure. Different distributions
matter because sparse scenes mostly measure overlap predicate cost, while
all-overlapping scenes also measure the cost of writing every candidate pair.

Tradeoff: this benchmark is synthetic and measures broadphase candidate
generation only. It does not include AABB proxy construction, sphere
narrowphase, contact generation, or solver work. That narrow scope is useful
for comparing broadphase algorithms, but it should not be presented as full
simulation latency.

Status: accepted as the current baseline broadphase benchmark. Initial Release
samples on Apple M4 show sparse and dense 1024-body cases near 1.06-1.10 ns per
pair check, while the all-overlapping 1024-body case is about 1.99 ns per pair
check because every candidate pair is written.

Future direction: keep the same body counts and distributions when evaluating
optimized broadphase implementations. Add larger counts or distribution-specific
workloads only when the benchmark runtime and memory behavior remain practical.

## Sweep-And-Prune AABB Broadphase

Decision: add an x-axis sweep-and-prune AABB broadphase as an alternative to
the deterministic O(n^2) baseline, then integrate it into `RigidBodySystem`
using caller-owned scratch storage. The implementation sorts proxy indices by
`bounds.min.x`, scans forward until the candidate minimum x is past the current
maximum x, then runs the existing full AABB overlap check before emitting a
candidate pair.

Why now: Phase 5 needs a measured optimized broadphase before choosing whether
to replace the world collision path. Sweep-and-prune is the smallest useful next
step after the all-pairs baseline because it works with the existing flat
`std::vector<AABBProxy>` input and is easier to test than a BVH.

Tradeoff: contact order is part of deterministic behavior, so the current
implementation sorts emitted candidates back into baseline proxy order. This
makes SAP directly comparable to `findAABBPairs`, but it adds extra candidate
storage and sorting cost. The convenience overload still allocates temporary
scratch vectors, while the scratch-backed overload is used by `RigidBodySystem`
to preserve the reserved `World::step` no-allocation hot path.

Status: accepted as the current rigid-body broadphase. The no-allocation
`World::step` test passes with scratch-backed SAP. Release samples on Apple M4
show the 1024-body sparse-grid standalone broadphase case improving from about
546312 ns per iteration to about 23681 ns per iteration, and the 1024-body
dense-grid case improving from about 578246 ns to about 90277 ns. The
all-overlapping 1024-body standalone case regresses from about 1022290 ns to
about 2340630 ns because every candidate pair is still emitted and the
order-preserving candidate sort adds work. On the sparse 1024-body
`World::step` benchmark, average step latency improved from the earlier 565306
ns sample to 60727.5 ns after integrating scratch-backed SAP.

Future direction: keep the O(n^2) baseline for correctness comparison and
worst-case reference. Add more `World::step` distributions, especially dense
and all-overlapping scenes, before claiming broadphase performance broadly.
Evaluate BVH only after SAP's measured tradeoffs are documented against
workloads that justify the additional tree-building complexity.

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
