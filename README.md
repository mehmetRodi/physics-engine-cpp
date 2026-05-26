# 3D Physics Engine

A planned real-time 3D physics simulation engine written in C++20, built as a portfolio project for deterministic simulation, numerical robustness, performance measurement, and low-latency systems engineering tradeoffs.

The current implementation starts with rigid-body sphere simulation because it is the smallest useful foundation for integration, collision, contacts, and benchmarking. The project is not intended to be rigid-body-only; future modules should fit the same deterministic, testable, benchmarkable architecture.

> **Status:** In active development

---

## Planned Features

- **Core simulation architecture** — fixed timestep, deterministic replay, explicit ownership, testable headless execution
- **Rigid body dynamics** — forces, torque, angular momentum, inertia tensor
- **Particle and constraint systems** — added after the rigid-body foundation has clean solver and contact boundaries
- **Broadphase collision** — BVH tree with SoA memory layout for cache efficiency
- **Narrowphase collision** — GJK + EPA for convex shapes
- **Constraint solver** — sequential impulse solver with friction and restitution
- **SIMD math** — Vec3/Quaternion operations accelerated with AVX2 intrinsics, after scalar correctness and benchmarks
- **Debug renderer** — orthographic wireframe view via SFML

---

## Performance

Performance work is measured only in Release or RelWithDebInfo builds. Current
benchmarking is intentionally simple: an in-repo `std::chrono` harness reports
latency distributions for fixed workloads before any optimization claims are
made.

Current benchmark target:

- `world_step_bench`: measures `World::step` for 1024 non-overlapping sphere
  bodies over a fixed timestep. This includes integration and the current
  deterministic O(n^2) sphere-pair scan, but mostly avoids collision response.
- `vec3_bench`: measures one million scalar `Vec3::dot` operations and reports
  total time plus nanoseconds per operation. The result is accumulated into a
  printed checksum so Release builds cannot optimize the loop away.
- `sphere_pair_bench`: measures the current naive O(n^2) sphere overlap
  predicate separately from integration and collision response.

Initial local sample:

| Benchmark | Bodies | Warmup | Samples | Avg | p95 | p99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `World::step` | 1024 | 100 | 1000 | 685016 ns | 695958 ns | 711083 ns | 781667 ns |

| Benchmark | Operations | Total | Per operation |
| --- | ---: | ---: | ---: |
| `Vec3::dot` | 1000000 | 946584 ns | 0.946584 ns |

| Benchmark | Bodies | Iterations | Pair checks | Total | Per pair check |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sphere pair check | 1024 | 100 | 52377600 | 66938542 ns | 1.278 ns |

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

---

## Build

**Dependencies:**

```bash
# Ubuntu/Debian
sudo apt install libsfml-dev cmake clang

# macOS
brew install sfml cmake llvm
```

**Build & run:**

```bash
git clone https://github.com/yourusername/physics-engine-cpp
cd physics-engine-cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/physics
```

**Run benchmarks:**

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DBUILD_DEMO=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=ON
cmake --build build-release --target world_step_bench
./build-release/world_step_bench
cmake --build build-release --target vec3_bench
./build-release/vec3_bench
cmake --build build-release --target sphere_pair_bench
./build-release/sphere_pair_bench
```

**Run tests:**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## Roadmap

- [ ] Vec3, Mat3, Quaternion math library
- [ ] Rigid-body module integration (semi-implicit Euler)
- [ ] Particle simulation module
- [ ] SFML debug renderer
- [ ] BVH broadphase with SoA layout
- [ ] GJK + EPA narrowphase
- [ ] Sequential impulse constraint solver
- [ ] AVX2 SIMD math pass
- [ ] Benchmark suite with repeatable methodology and latency distributions
- [ ] Friction and restitution
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
