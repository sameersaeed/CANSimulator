# CANSimulator

**An OBD-II / CAN bus ECU simulator and diagnostic tool**

CANSimulator simulates a vehicle's **Electronic Control Unit (ECU)** on a virtual
**Controller Area Network (CAN)** interface (`vcan0`). It responds to OBD-II
(SAE J1979) Mode 01 diagnostic requests in real time

The simulator runs two components:
- A vehicle simulator that continuously updates engine and vehicle state
- An OBD server that listens for diagnostic requests on the CAN bus

Clients send OBD-II requests to CAN ID **0x7DF**, and ECUs respond with PID values
on IDs **0x7E8–0x7EF**. See the [Wikipedia](https://en.wikipedia.org/wiki/OBD-II_PIDs#CAN_(11-bit)_bus_format)
page for details on the 11-bit CAN format

## Prerequisites
- Linux
- gcc or clang
- cmake
- can-utils

Optional:
- Docker
- Docker Compose
- libsdl2-dev + libsdl2-ttf-dev (for the live dashboard)

---

## Build

```bash
git clone https://github.com/sameersaeed/CANSimulator.git
cd CANSimulator

# setup virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

ip link show vcan0  # expected: vcan0: <NOARP,UP,LOWER_UP> ...

# build (dashboard is auto-enabled if SDL2 is installed)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

./build/cansimulator --help
```

### Debug build with AddressSanitizer + UBSan

```bash
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j$(nproc)
./build-debug/cansimulator --help
```

---

## Quick Start

Run the simulator in one terminal and query it from another:

**Server terminal: Running the simulator:**
```bash
./build/cansimulator --simulate

[12:00:01] [INFO ] CANSimulator starting on vcan0
[12:00:01] [INFO ] OBDServer listening on vcan0 (CAN ID 0x7DF)
[12:00:01] [INFO ] Simulator + OBDServer running. Ctrl+C to stop.
[12:00:06] [INFO ] Requests: 5 | Avg: 0.042 ms | MaxLat: 0.091 ms | RPM: 1247 | Speed: 14 km/h
```

**Client terminal: Querying a PID:**
```bash
./build/cansimulator --query RPM

Engine RPM                  1247.00  rpm  (0.038 ms)
```

**Client terminal: Dumping all PIDs at once:**
```bash
./build/cansimulator --dump

=== CANSimulator Vehicle State Dump ===
Engine Load                   18.43  %
Coolant Temp                  91.20  °C
Engine RPM                  2318.00  rpm
Vehicle Speed                 50.00  km/h
Intake Air Temp               26.40  °C
MAF Air Flow                   5.63  g/s
Throttle Position             18.62  %
Runtime Since Start          142.00  s
Fuel Level                    74.96  %
Distance Since DTC Clear        0.04  km
```

**Spy on raw CAN traffic (can-utils package required):**
```bash
candump vcan0

vcan0  7DF   [8]  02 01 0C 00 00 00 00 00   # RPM request
vcan0  7E8   [8]  04 41 0C 12 34 00 00 00   # RPM response (0x1234/4 = 1165 rpm)
```

---

## Docker / docker-compose

The vcan interface has to be created on the host first:
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

**Start simulator + demo client:**
```bash
docker compose up
```

**Run the benchmark profile:**
```bash
docker compose run --rm bench
```

**Build image manually:**
```bash
docker build -t cansimulator .
docker run --rm --privileged --network host cansimulator --dump
```

---

## Usage

| Command                      | Flag(s)           | What it does                                 |
|------------------------------|-------------------|----------------------------------------------|
| `cansimulator --simulate`    |                   | Start ECU simulator + OBD server             |
| `cansimulator --query`       | `<PID>`           | Query one PID and print result               |
| `cansimulator --dump`        |                   | Query all supported PIDs                     |
| `cansimulator --bench`       | `<PID>` `<count>` | latency / throughput benchmark               |
| `cansimulator --dashboard`   |                   | Live SDL2 instrument cluster (requires SDL2) |
| `cansimulator --help`        |                   | Show this message                            |

Supported PID names (case-sensitive):
- `RPM`
- `SPEED`
- `COOLANT_TEMP`
- `ENGINE_LOAD`
- `THROTTLE`
- `INTAKE_TEMP`
- `MAF`
- `FUEL_LEVEL`
- `RUNTIME`
- `DIST_DTC`

---

## Dashboard

The `--dashboard` flag opens a live SDL2 instrument cluster. It starts the ECU simulator and OBD server, so you don’t need to run anything else in another terminal.
```bash
./build/cansimulator --dashboard
```

The dashboard includes:
- Speed (km/h) and RPM arc gauges with color-coded ranges (green / yellow / red)
- Stat boxes for coolant temp, engine load, MAF, fuel level, and runtime
- Live CAN request counter in the header

<img width="895" height="557" alt="CANSimulator-dashboard" src="https://github.com/user-attachments/assets/8000e181-2177-4912-a40d-f39c773e5c83" />


Controls:

| Key          | Action      |
|--------------|-------------|
| W / Up       | Accelerate  |
| S / Down     | Brake       |
| Q / Escape   | Quit        |

Hold **W** to increase throttle and RPM - releasing it lets the engine coast down
Hold **S** to brake - sensor values like coolant temp, MAF, and engine load update live as RPM changes

---

## Running Unit Tests

```bash
# build tests 
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON
cmake --build build -j$(nproc)

# run all tests
cd build && ctest --output-on-failure
```

```
[==========] Running 22 tests from 4 test suites.
[----------] 11 tests from PIDRoundTrip
[ RUN      ] PIDRoundTrip.RPM_Midrange ..................... OK
[ RUN      ] PIDRoundTrip.RPM_Idle ......................... OK
[ RUN      ] PIDRoundTrip.RPM_Clamp_Max .................... OK
[ RUN      ] PIDRoundTrip.Speed ............................ OK
[ RUN      ] PIDRoundTrip.CoolantTemp ...................... OK
[ RUN      ] PIDRoundTrip.EngineLoad ....................... OK
[ RUN      ] PIDRoundTrip.Throttle ......................... OK
[ RUN      ] PIDRoundTrip.MAF .............................. OK
[ RUN      ] PIDRoundTrip.FuelLevel ........................ OK
[ RUN      ] PIDRoundTrip.Runtime .......................... OK
[ RUN      ] PIDRoundTrip.DistDTC .......................... OK
[----------] 6 tests from PIDNameLookup
...
[==========] 22 tests from 4 test suites ran.
[  PASSED  ] 22 tests.
```

---

## Benchmarking

The `--bench` mode sends the specified number of sequential OBD-II requests and reports the latencyUs statistics

Sample benchmark with 1000 RPM requests:
```bash
./build/cansimulator --bench RPM 1000

=== CANSimulator Benchmark: RPM x1000 ===
  Successful:   1000/1000
  Avg latency:  0.041 ms
  p50 latency:  0.038 ms
  p95 latency:  0.089 ms
  p99 latency:  0.142 ms
  Max latency:  0.391 ms
  Throughput:   24390 req/s
```

**Stress test with concurrent clients (5 clients × 500 requests)**:
```bash
seq 5 | parallel -j5 './build/cansimulator --bench RPM 500'
```

**Observe raw frames with candump during a benchmark:**
```bash
candump vcan0 &
./build/cansimulator --bench RPM 20
```

---
