# 3D Rigid Body Physics Engine

A real-time 3D physics engine written in C++20, built with a focus on cache-efficient data layouts, SIMD-accelerated math, and low-latency constraint solving.

> **Status:** In active development

---

## Features

- **Rigid body dynamics** — forces, torque, angular momentum, inertia tensor
- **Broadphase collision** — BVH tree with SoA memory layout for cache efficiency
- **Narrowphase collision** — GJK + EPA for convex shapes (in progress)
- **Constraint solver** — sequential impulse solver with friction and restitution
- **SIMD math** — Vec3/Quaternion operations accelerated with AVX2 intrinsics
- **Debug renderer** — orthographic wireframe view via SFML

---

## Performance

> Benchmarks run with [Google Benchmark](https://github.com/google/benchmark)
> on AMD R7-5800H, Linux 6.19.10-200.fc43.x86_64, compiler: Clang 16 `-O2`.

| System                              | Naive | Optimized | Speedup |
| ----------------------------------- | ----- | --------- | ------- |
| Broadphase — 1000 bodies            | —     | —         | TBD     |
| Constraint solver — 500 constraints | —     | —         | TBD     |
| Vec3 dot product — 1M ops           | —     | —         | TBD     |

_Numbers will be updated as each optimization pass is completed and measured with `perf stat`._

---

## Architecture

```
physics_engine/
├── math/           # Vec3, Mat3, Quaternion, AABB, SIMD intrinsics
├── physics/        # RigidBody, World, Integrator, Solver, Constraints
├── collision/      # Broadphase (BVH), Narrowphase (GJK/EPA), Contact
├── renderer/       # Debug wireframe renderer (SFML)
├── bench/          # Google Benchmark suites
├── tests/          # Unit tests (Google Test)
└── main.cpp        # Entry point + demo scene
```

Key design decisions:

- **SoA layout** in broadphase — positions and AABBs stored in separate arrays to maximise cache line utilisation during BVH traversal
- **AVX2 SIMD** for Vec3 batch operations — explicit intrinsics rather than relying on auto-vectorisation
- **No heap allocations on the hot path** — all physics objects pre-allocated in a pool at startup
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
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang++
make -j$(nproc)
./physics
```

**Run benchmarks:**

```bash
cmake .. -DBUILD_BENCHMARKS=ON
make -j$(nproc)
./bench/broadphase_bench
./bench/solver_bench
```

**Run tests:**

```bash
cmake .. -DBUILD_TESTS=ON
make -j$(nproc)
ctest --output-on-failure
```

---

## Roadmap

- [ ] Vec3, Mat3, Quaternion math library
- [ ] RigidBody integration (semi-implicit Euler)
- [ ] SFML debug renderer
- [ ] BVH broadphase with SoA layout
- [ ] GJK + EPA narrowphase
- [ ] Sequential impulse constraint solver
- [ ] AVX2 SIMD math pass
- [ ] Google Benchmark suite with perf stat measurements
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
