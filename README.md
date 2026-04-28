# Drone Bread Delivery Simulation

Autonomous drone fleet delivering bread from bakeries to customers on a 2D grid, optimized per round using parallel GRASP (Greedy Randomized Adaptive Search Procedure) with 2-opt local search.

## Architecture

**Backend (C++17)** -- Simulation engine with multi-threaded GRASP solver and built-in HTTP server.  
**Frontend (React 18)** -- Canvas-based real-time visualization with interactive controls.

## Quick Start

### Backend

```bash
cd backend
mkdir build && cd build
cmake .. && make
cd ..
./build/bread_delivery --server --port 8080
```

### Frontend

```bash
cd frontend
npm install
npm start
```

Open `http://localhost:3000`.

### Batch Mode (headless)

```bash
cd backend
./build/bread_delivery
```

## How It Works

Each round runs four stages:

1. **State Update** -- Bakeries produce bread (stochastic), unserved customers gain priority, drones advance along routes.
2. **Assignment** -- Parallel GRASP selects customer-bakery-drone triples, maximizing `priority / delivery_time`.
3. **Commit** -- Conflicts resolved by score (bakery inventory deducted, drone routes updated).
4. **Delivery** -- Drones physically pick up bread and deliver it. Customers leave only after receiving their full order.

Drones are **unlimited** -- new ones spawn from the base automatically when all existing drones are busy. Partial deliveries reduce the customer's remaining order; they stay in the queue until fully served.

## Configuration

Edit `backend/config.json`:

| Parameter | Description |
|---|---|
| `grid_width/height` | Grid dimensions |
| `max_rounds` | Simulation rounds |
| `grasp_iterations` | GRASP iterations per round |
| `thread_count` | Parallel threads |
| `drones` | Templates (velocity, capacity) -- new drones are spawned from these as needed |
| `bakeries` | Position, capacity, production distribution |
| `customers` | Position and order size |

## Tech

- C++17, POSIX sockets, nlohmann/json (vendored)
- React 18, Canvas 2D
