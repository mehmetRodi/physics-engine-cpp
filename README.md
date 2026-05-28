# 3D Physics Engine

A planned real-time 3D physics simulation engine written in C++20, built as a portfolio project for deterministic simulation, numerical robustness, performance measurement, and low-latency systems engineering tradeoffs.

The current implementation starts with rigid-body sphere simulation because it is the smallest useful foundation for integration, collision, contacts, and benchmarking. The project is not intended to be rigid-body-only; future modules should fit the same deterministic, testable, benchmarkable architecture.

> **Status:** In active development

---

## Implemented Features

- **Scalar math primitives** — `Vec3` arithmetic, dot/cross products, length, and normalization tests
- **Rigid-body sphere simulation** — world-owned bodies, fixed-step integration, and deterministic storage order
- **Sphere collision baseline** — narrowphase pair checks and simple collision response for spheres
- **Headless tests** — CTest/GoogleTest coverage for math, world stepping, deterministic replay, rigid bodies, and sphere collision
- **Benchmark harnesses** — in-repo `std::chrono` benchmarks for `Vec3`, sphere pair checks, and `World::step`
- **SFML debug demo** — visual rigid-body sphere scene with fixed-step controls, trails, contact highlights, static obstacles, and click-to-spawn bodies

## Planned Work

- Force accumulation, materials, restitution, friction, and damping policies
- Broadphase collision with measured baseline and later optimized layouts
- Constraint solving with documented convergence and latency behavior
- Additional simulation domains only after the shared stepping, testing, and benchmarking model is clear
- Camera/navigation and immersive inspection controls for visual demos, so simulations can be inspected at useful scales without coupling physics to rendering
- SIMD, custom allocation, and multithreading only after scalar correctness and benchmarks justify them

---

## Performance

Performance work is measured only in Release or RelWithDebInfo builds. Current
benchmarking is intentionally simple: an in-repo `std::chrono` harness reports
latency distributions for fixed workloads before any optimization claims are
made.

Current benchmark target:

- `world_step_bench`: measures `World::step` for 1024 non-overlapping sphere
  bodies over a fixed timestep. This includes integration, AABB proxy creation,
  the current deterministic O(n^2) AABB broadphase, sphere narrowphase contact
  generation, and mostly avoids collision response.
- `vec3_bench`: measures one million scalar `Vec3::dot` operations and reports
  total time plus nanoseconds per operation. The result is accumulated into a
  printed checksum so Release builds cannot optimize the loop away.
- `sphere_pair_bench`: measures the current naive O(n^2) sphere overlap
  predicate separately from integration and collision response, using the
  reusable output-buffer pair-generation API.
- `aabb_pair_bench`: measures the baseline deterministic O(n^2) AABB
  broadphase across sparse, dense, and all-overlapping distributions at
  multiple body counts.

Initial local sample:

| Benchmark | Bodies | Warmup | Samples | Avg | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `World::step` | 1024 | 100 | 1000 | 565306 ns | 601917 ns | 621791 ns | 742709 ns |

| Benchmark | Operations | Total | Per operation |
| --- | ---: | ---: | ---: |
| `Vec3::dot` | 1000000 | 766000 ns | 0.766 ns |

| Benchmark | Bodies | Iterations | Pair checks | Total | Per pair check |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sphere pair check | 1024 | 100 | 52377600 | 67440208 ns | 1.28758 ns |

| AABB broadphase distribution | Bodies | Iterations | Candidate pairs | Total | Per pair check |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sparse grid | 128 | 2460 | 0 | 39192250 ns | 1.96011 ns |
| Dense grid | 128 | 2460 | 316 | 27289750 ns | 1.36484 ns |
| All overlapping | 128 | 2460 | 8128 | 39908542 ns | 1.99594 ns |
| Sparse grid | 512 | 152 | 0 | 21084083 ns | 1.06035 ns |
| Dense grid | 512 | 152 | 1834 | 22055875 ns | 1.10923 ns |
| All overlapping | 512 | 152 | 130816 | 38630584 ns | 1.94279 ns |
| Sparse grid | 1024 | 38 | 0 | 21144166 ns | 1.06233 ns |
| Dense grid | 1024 | 38 | 3858 | 21826750 ns | 1.09663 ns |
| All overlapping | 1024 | 38 | 523776 | 39612792 ns | 1.99024 ns |

Interpretation: at this scale, `World::step` is still dominated by the
O(n^2) broadphase-style pair scan. The sparse 1024-body AABB broadphase case
takes about 556 microseconds per iteration, close to the 565 microsecond
`World::step` average, so current evidence still points toward broadphase and
collision-pipeline work before an AoS-to-SoA body-state migration.

Environment for the sample above:

- Machine: MacBook Air M4, 24 GB RAM
- OS: macOS 26.5, build 25F71
- Architecture: arm64
- Compiler: AppleClang 21.0.0
- CMake: 4.3.2
- Build type: Release
- Release flags from CMake: `-O3 -DNDEBUG`

These numbers are an initial local baseline, not an optimization claim. Future
performance work should compare before/after results using the same workload,
build type, machine, and methodology.

---

## Target Architecture

```
physics_engine/
├── math/           # Vec3, Mat3, Quaternion, AABB, SIMD intrinsics
├── physics/        # World, simulation state, integrators, solver-facing data
├── collision/      # Broadphase (BVH), Narrowphase (GJK/EPA), Contact
├── dynamics/       # Future domain modules: rigid bodies, particles, deformables
├── renderer/       # Debug wireframe renderer (SFML)
├── bench/          # Custom benchmark harnesses
├── tests/          # Unit tests (Google Test)
└── main.cpp        # Entry point + demo scene
```

Planned design direction:

- **SoA layout** in broadphase — positions and AABBs stored in separate arrays to maximise cache line utilisation during BVH traversal, once profiling justifies the layout
- **AVX2 SIMD** for Vec3 batch operations — explicit intrinsics only after scalar code is correct and measured
- **No heap allocations on the hot path** — physics storage should be reserved or pre-allocated before simulation steps where practical
- **Deterministic integrator** — fixed timestep with semi-implicit Euler, reproducible across runs

## Floating-Point Reproducibility

Deterministic replay is currently scoped to the same source code, input data,
platform, compiler, build type, and floating-point compiler flags. The engine
uses scalar `float` math and assumes normal IEEE-754 single-precision behavior.

Do not build simulation or benchmark targets with flags that weaken IEEE
floating-point semantics or permit unsafe reassociation, including
`-ffast-math`, `-Ofast`, `-funsafe-math-optimizations`, `-fassociative-math`,
`-freciprocal-math`, `-ffinite-math-only`, or MSVC `/fp:fast`. CMake rejects
these flags when they are passed through the standard `CMAKE_CXX_FLAGS*`
variables.

---

## Build

**Dependencies:**

```bash
# Ubuntu/Debian
sudo apt install cmake clang libsfml-dev

# macOS with Homebrew
brew install cmake llvm sfml@2
```

The visual demo depends on SFML 2.x. Headless tests and benchmarks can be built
without SFML by passing `-DBUILD_DEMO=OFF`.

**Debug build with demo and tests:**

```bash
git clone https://github.com/yourusername/physics-engine-cpp
cd physics-engine-cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/physics
```

**Release build for benchmarks:**

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DBUILD_DEMO=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=ON
cmake --build build-release
```

**RelWithDebInfo build for profiling:**

```bash
cmake -S . -B build-relwithdebinfo -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_DEMO=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=ON
cmake --build build-relwithdebinfo
```

**Visual demo controls:**

- `Space`: pause or resume fixed-step simulation
- `N`: advance one fixed simulation step while paused
- `R`: reset to the deterministic initial sphere scene
- `T`: toggle motion trails
- `Up`/`Down`: adjust fixed-step playback speed
- Left click inside the arena: add one dynamic sphere
- `Esc`: quit

The current SFML demo is a debug renderer for rigid-body spheres. Sphere-sphere
collision comes from the physics core; the rectangular arena walls are handled
in demo code until general boundary/collision-shape support exists. Static
obstacle spheres are represented as zero-inverse-mass bodies in the same core
rigid-body system.

**Run the deterministic console demo:**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target headless_replay
./build/headless_replay
```

This demo runs fixed inputs for a fixed number of simulation steps and prints the
final body state. It is intended to demonstrate deterministic replay behavior,
not benchmark latency.

**Run benchmarks:**

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DBUILD_DEMO=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=ON
cmake --build build-release --target world_step_bench
./build-release/world_step_bench
cmake --build build-release --target vec3_bench
./build-release/vec3_bench
cmake --build build-release --target sphere_pair_bench
./build-release/sphere_pair_bench
cmake --build build-release --target aabb_pair_bench
./build-release/aabb_pair_bench
```

**Run tests:**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

## Developer Workflow

GitHub Actions builds the Debug demo and test binary on Ubuntu and macOS, then
runs the headless CTest suite. This keeps the SFML demo build reproducible
without requiring the visual app to launch in CI.

**Formatting:**

```bash
git ls-files '*.hpp' '*.cpp' | xargs clang-format -i
```

If Homebrew LLVM is installed but not on `PATH`, use:

```bash
git ls-files '*.hpp' '*.cpp' | xargs $(brew --prefix llvm)/bin/clang-format -i
```

**Static analysis:**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
git ls-files 'math/*.cpp' 'physics/*.cpp' 'collision/*.cpp' 'bench/*.cpp' | xargs clang-tidy -p build
```

If Homebrew LLVM is installed but not on `PATH`, use:

```bash
git ls-files 'math/*.cpp' 'physics/*.cpp' 'collision/*.cpp' 'bench/*.cpp' | xargs $(brew --prefix llvm)/bin/clang-tidy -p build
```

---

## Roadmap

- [x] Initial Vec3 scalar math
- [x] Rigid-body sphere integration with fixed timestep
- [ ] Particle simulation module
- [x] SFML debug renderer
- [ ] BVH broadphase with SoA layout
- [ ] GJK + EPA narrowphase
- [ ] Sequential impulse constraint solver
- [ ] AVX2 SIMD math pass
- [x] Initial benchmark harness with repeatable methodology and latency distributions
- [ ] Friction and restitution
- [ ] Camera and immersive inspection controls for visual demos
- [ ] Demo scenes (Newton's cradle, dominos, stacking)

---

## References

- Erin Catto — [GDC Physics Talks](https://box2d.org/publications/) (constraint solver design)
- Randy Gaul — [Physics Engine Architecture](https://randygaul.github.io/)
- Christer Ericson — _Real-Time Collision Detection_ (BVH, GJK, EPA)
- Intel Intrinsics Guide — AVX2 reference

---

## License

MIT
