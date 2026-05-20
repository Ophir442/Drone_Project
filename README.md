# Parallel Bread Delivery Simulation

A parallel discrete-event simulation of a drone-based bread-delivery network.
Bakeries produce stochastically, customers wait with growing priority, and a
self-scaling drone fleet picks routes through a **GRASP + 2-Opt** assignment
planner. Built around a custom `std::future`-based thread pool with **two-phase
commit** synchronization — no mutexes on shared state.

The C++17 backend can run headless or as an HTTP server feeding a React
frontend canvas visualization.

---

## Architecture

| Layer | Stack |
|---|---|
| Simulation engine | C++17, custom `ThreadPool`, GRASP + 2-Opt |
| Distance cache | Per-round symmetric matrix over bakeries / customers / drones / base |
| HTTP server | POSIX sockets, JSON via nlohmann (vendored) |
| Frontend | React 18 + Canvas 2D, live polling, interactive controls |

Each round runs a four-stage pipeline:

1. **State update** — bakeries produce (parallel) and drones advance along their planned routes (parallel). Delivery events are applied on the main thread because `std::priority_queue` is not thread-safe.
2. **Assignment** — `grasp_iterations` GRASP iterations dispatch across the thread pool. Each worker takes a **private snapshot** of drones and bakeries and emits `Intent` objects; the real shared state is never touched.
3. **Commit** — main thread resolves bakery contention by score, stamps final amounts onto each drone's route.
4. **Reposition** — idle drones move toward the highest **Gravity-Score** bakery (fullness ÷ distance) to soak up production before it caps.

See `PRESENTATION.txt` for the full design walkthrough and spec-compliance audit.

---

## Build & Run

### Backend (headless)

```bash
cd backend
make            # produces ./bread_delivery
./bread_delivery config.json
```

### Backend (HTTP server mode for the frontend)

```bash
cd backend
make server     # builds + runs on port 8080
```

### Frontend

```bash
cd frontend
npm install
npm start       # opens http://localhost:3000
```

The frontend polls the backend over HTTP. With both running, you can step
the simulation, add/remove customers live, and watch drones plan routes.

---

## Configuration

Edit `backend/config.json`:

| Key | Description |
|---|---|
| `world.grid_width` / `grid_height` | Grid bounds |
| `world.base_pos` | Drone base coordinates |
| `simulation.max_rounds` | Round cap |
| `simulation.priority_increment` | Bump applied each round to unserved customers (spec rule `w_c + 1`) |
| `algorithm.grasp_iterations` | Total GRASP iterations per round (split across workers) |
| `algorithm.rcl_size` | Restricted Candidate List size for the GRASP random pick |
| `algorithm.thread_count` | Worker threads in the pool |
| `drone_template` | `capacity`, `velocity_min/max`, `initial_count` |
| `bakeries[]` | `pos`, `capacity`, `initial_inventory`, `production_distribution: [[amount, prob], ...]` |
| `customers[]` | `pos`, `order_quantity` |

Customer `priority_weight` is **not** in config — it starts at 1.0 and grows at runtime per the spec rule for unserved orders.

The config is validated at startup (`Simulation::validate_config()`) — empty
customer list, non-positive drone capacity, or a bakery set that can never
produce bread will throw a `std::runtime_error` before any state is allocated.

---

## Key design decisions

- **Custom `ThreadPool` over OpenMP.** Pool is persistent across the entire run; reused for both bakery/drone motion (Stage 1) and GRASP planning (Stages 2–3). Avoids the oversubscription that would result from two competing pools.
- **Two-phase commit, not mutexes.** GRASP workers operate on value-copy snapshots; only `stage4_commit` mutates shared state. Race-freedom is structural.
- **Organic fleet sizing.** `should_spawn_drone()` spawns iff bread exists *and* fleet capacity is less than total demand. Fleet stabilizes at exactly `⌈D_demand / capacity⌉` drones — no hardcoded cap.
- **Order splitting.** Customers with orders larger than a single drone's capacity are served by multiple drones (within a round) or across rounds. `amount_to_take = min(achievable, max_capacity - future_load)`.
- **Pre-computed distance matrix per round.** Built once on the main thread (serially, to keep the thread pool unconflicted), then read O(1) by every GRASP worker.
- **2-Opt local search with validity check.** Pickup-before-delivery ordering is preserved across segment reversals.

---

## Project layout

```
.
├── backend/
│   ├── Makefile
│   ├── config.json
│   ├── include/
│   │   ├── types.h           # All data structures
│   │   ├── distance.h        # DeliveryGraph + DistanceCache
│   │   ├── grasp.h           # GraspSolver (GRASP + 2-Opt)
│   │   ├── simulation.h      # Simulation orchestrator
│   │   ├── thread_pool.h     # std::future-based thread pool
│   │   └── http_server.h
│   ├── src/                  # Matching .cpp files
│   └── third_party/nlohmann/ # vendored JSON library
├── frontend/
│   ├── src/
│   │   ├── App.jsx
│   │   ├── components/       # SimulationMap, ControlPanel, StatsPanel, ...
│   │   └── utils/
│   └── package.json
├── Bakery (1) (2).pdf        # Assignment spec
├── PRESENTATION.txt          # Full design report
└── README.md
```

---

## Tech

- **C++17**, POSIX threads, POSIX sockets, `nlohmann/json` (vendored)
- **React 18**, Canvas 2D, plain CSS
- **GCC `-Wall -Wextra -O2`** — builds with zero warnings
