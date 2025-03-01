# DroneSecFS: Formal Methods for Optimized and Verified Drone Delivery Systems
 This project applies formal methods to address key challenges in drone delivery systems. It consists of two complementary components: (1) a path optimization solution that adapts the Traveling Salesman Problem to drone delivery using Answer Set Programming, and (2) a formal verification framework that models drone flight maneuvers using temporal logic (LTL/CTL). The path optimization balances distance and battery consumption to identify optimal delivery routes, while the formal verification ensures that drone flight behaviors satisfy critical safety and operational requirements. The implementation uses Potassco/clingo for ASP path optimization and NuSMV for model checking of temporal properties. This approach provides provable guarantees for drone delivery operations, balancing efficiency and safety in autonomous aerial logistics.


# Formal Methods for Drone Delivery Systems 🚁📦

## Project Overview

This repository contains the implementation of formal methods for security in a drone delivery system scenario. The project addresses optimization problems related to path planning and formal verification of drone maneuvers using Answer Set Programming (ASP) and temporal logic (LTL/CTL).

The work focuses on two main components:
1. **Path Optimization**: Adapting the Traveling Salesman Problem using ASP
2. **Formal Verification**: Modeling drone maneuvers using temporal logic

## Problem Statement 🔍

The project considers a delivery drone that must:
- Find the optimal delivery route between multiple points
- Perform various flight maneuvers safely (takeoff, landing, hovering, delivery)
- Adhere to specific altitude requirements during different flight phases
- Ensure safety and operational constraints are satisfied

## Implementation Details

### 1. Traveling Salesman Problem Adaptation 🗺️

#### Background
The Traveling Salesman Problem (TSP) is a classical combinatorial optimization problem where a salesperson must visit each city exactly once and return to the origin city while minimizing the total distance traveled. The problem can be represented as a graph where cities are vertices and the connections between them are edges with associated weights.

#### Our Approach
We adapted the classical TSP to the drone delivery context by:

- **Graph Representation**: Cities in the Matera province (Italy) are represented as vertices, with partial connectivity to reflect realistic flight constraints
- **Cost Function**: The optimization considers both distance traveled and battery consumption
- **Implementation Tool**: Potassco/clingo (Answer Set Programming system)

#### ASP Implementation
The implementation uses Answer Set Programming (ASP) through the Potassco system. The code includes:

```prolog
% Definition of edges (connections between cities)
edge(1,(2;3;4)). % Connections from Matera
edge(2,(1;4;5)). % Connections from Montescaglioso

% Distances in km
distance(1,2,15). distance(2,1,15).
distance(1,3,20). distance(3,1,20).

% Battery costs in percentage
battery_cost(1,2,12). battery_cost(2,1,14).
battery_cost(1,3,15). battery_cost(3,1,16).
```

The core ASP rules include:

```prolog
% For each node, exactly one outgoing edge must be selected
{ cycle(X,Y) : edge(X,Y) } = 1 :- node(X).

% For each node, exactly one incoming edge must be selected
{ cycle(X,Y) : edge(X,Y) } = 1 :- node(Y).

% Reachability rules
reached(Y) :- cycle(1,Y).
reached(Y) :- cycle(X,Y), reached(X).
:- node(Y), not reached(Y).

% Calculate total cost as sum of distance and battery consumption
cost(X,Y,C) :- distance(X,Y,D), battery_cost(X,Y,B), C = D + B.

% Calculate total distance
total_distance(TD) :- TD = #sum { D,X,Y : cycle(X,Y), distance(X,Y,D) }.

% Calculate total battery consumption
total_battery(TB) :- TB = #sum { B,X,Y : cycle(X,Y), battery_cost(X,Y,B) }.

% Verify completeness of the cycle
cycle_complete :- N = #count { X : node(X) },
                 N = #count { X : reached(X) }.
:- not cycle_complete.

% Minimize total cost
#minimize { C,X,Y : cycle(X,Y), cost(X,Y,C) }.
```

This ASP implementation efficiently solves the problem by:
1. Defining possible routes as edges with associated costs
2. Enforcing a single entry and exit for each node
3. Ensuring all nodes are reached (complete cycle)
4. Optimizing for the combined cost (distance + battery consumption)

#### Key Features
- Non-completely connected graph to represent realistic drone navigation constraints
- Battery consumption varies from 8% to 17% per route segment
- Return routes incur slightly higher battery consumption (1-2% increase) to account for potential headwinds
- The optimal path minimizes a combined cost of distance and battery consumption

#### Results
The model successfully identified an optimal Hamiltonian cycle:
```
Matera(1) → Pomarico(3) → Grassano(6) → Salandra(7) → Grottole(5) → Miglionico(4) → Montescaglioso(2) → Matera(1)
```
With a total cost of 186 units (105 distance units + 81 battery consumption units).

This result demonstrates the ability of ASP to find an optimal solution that balances two cost factors (distance and battery) while respecting the connectivity constraints of the problem.

### 2. Formal Verification of Drone Maneuvers ✅

#### Modeling Approach
We modeled the drone as a finite state machine with states corresponding to different flight phases and altitudes:

- **States**: Inactive, Takeoff, Hovering, In Flight, Landing, Delivery
- **Altitudes**: Below Target, At Target, Above Target
- **Formal Framework**: Used Kripke structure for state transitions

#### Kripke Structure
The drone behavior is modeled as a Kripke structure M = (S, I, R, L) where:
- S = {S₁, S₂, S₃, S₄, S₅, S₆, S₇, S₈}: Set of all states
- I = {S₁}: Initial state (inactive)
- R ⊆ S × S: Transition relation
- L: Labeling function mapping states to sets of atomic propositions

The atomic propositions include:
- i: Inactive (drone on ground)
- d: Takeoff phase
- h: Hovering
- r: Delivery/release
- v: In flight
- a: Landing
- t: At target altitude
- s: Below target altitude
- u: Above target altitude

#### Temporal Logic Properties
We formalized key safety and operational requirements using:

##### LTL (Linear Temporal Logic) Properties:
- `G(release → X hovering)`: After each delivery, the drone must stabilize in hovering
- `G(inactive → X takeoff)`: From inactive state, the system must proceed with takeoff
- `G((flight ∧ altitude = above_target) → X(flight ∧ altitude = target))`: Gradual altitude adjustment
- `G((flight ∧ altitude = below_target) → X(flight ∧ altitude = target))`: Gradual altitude adjustment
- `G(hovering ∧ altitude = target → X (landing ∨ flight ∨ release))`: Controlled transitions from hovering
- `G((flight ∧ altitude = above_target) → (¬landing U altitude = target))`: Safe landing requirement
- `G(takeoff → (altitude = below_target ∧ F(altitude = target)))`: Proper takeoff sequence
- `F(G(hovering))`: Violation check for hovering deadlock (correctly not satisfied)

##### CTL (Computation Tree Logic) Properties:
- `AG(flight ∧ altitude = target → EX(hovering ∨ (flight ∧ ¬(altitude = target))))`: Operational flexibility
- `AG(flight → E (flight U hovering))`: Path existence to hovering
- `AG(hovering → EF(landing) ∧ EF(flight))`: Mission completion options
- `AG(flight → EF(altitude = above_target) ∧ EF(altitude = below_target))`: Vertical navigation freedom
- `AG(flight → EF(landing))`: Safety landing guarantee
- `AG(hovering → AF release)`: Violation check for forced delivery (correctly not satisfied)

#### NuSMV Implementation
The model was implemented in NuSMV with the following structure:

```
MODULE main
VAR
  state : {s1, s2, s3, s4, s5, s6, s7, s8};
  inactive : boolean;
  takeoff : boolean;
  hovering : boolean;
  release : boolean;
  flight : boolean;
  landing : boolean;
  altitude : {below_target, target, above_target};

ASSIGN
  init(inactive) := TRUE;
  init(takeoff) := FALSE;
  init(hovering) := FALSE;
  init(release) := FALSE;
  init(flight) := FALSE;
  init(landing) := FALSE;
  init(altitude) := below_target;
  init(state) := s1;

  next(state) := case
    state = s1 : s2;                           -- From inactive to takeoff
    state = s2 : s4;                           -- From takeoff to hovering
    state = s3 : s1;                           -- From landing to inactive
    state = s4 : {s3, s5, s6, s7, s8};         -- From hovering to various states
    state = s5 : {s4, s6, s7};                 -- Flight transitions
    state = s6 : s5;                           -- Above target to target altitude
    state = s7 : s5;                           -- Below target to target altitude
    state = s8 : s4;                           -- From release to hovering
  esac;

  -- Other state variable updates follow...

SPEC G (release -> X hovering)
SPEC G (inactive -> X takeoff)
SPEC G ((flight & altitude = above_target) -> X (flight & altitude = target))
SPEC G ((flight & altitude = below_target) -> X (flight & altitude = target))
SPEC G ((hovering & altitude = target) -> X (landing | flight | release))
SPEC G ((flight & altitude = above_target) -> (!landing U altitude = target))
SPEC G (takeoff -> (altitude = below_target & F(altitude = target)))
SPEC F (G hovering)  -- false

SPEC AG (flight & altitude = target -> EX(hovering | (flight & altitude != target)))
SPEC AG (flight -> E [ flight U hovering ])
SPEC AG (hovering -> EF landing & EF flight)
SPEC AG (flight -> EF altitude = above_target & EF altitude = below_target)
SPEC AG (flight -> EF landing)
SPEC AG (hovering -> AF release)  -- false
```

#### Verification Tool
We implemented and verified the model using NuSMV, a symbolic model checker. The verification process confirmed that the drone behavior satisfies critical safety properties and identified counterexamples for properties that are correctly violated.

## Methodology 🔬

The project employed a systematic approach:

1. **Problem Analysis**: 
   - Study of TSP and drone delivery constraints
   - Analysis of altitude requirements for different maneuvers
   - Definition of safety properties

2. **Formal Modeling**:
   - ASP for path optimization
   - Kripke structure for state transitions
   - LTL/CTL formulas for behavioral properties

3. **Implementation**:
   - ASP code in clingo
   - NuSMV model for verification

4. **Verification**:
   - Model checking of temporal properties
   - Identification of satisfied and violated properties

## Key Insights 💡

1. **Altitude Management**:
   - Hovering at target altitude is essential before delivery or landing
   - Gradual altitude changes are necessary for stability
   - Different altitude requirements for takeoff vs. landing

2. **Battery Optimization**:
   - Hovering consumes significantly more energy than cruising flight
   - Minimizing hovering time is crucial for energy efficiency
   - Wind conditions affect battery consumption on return paths

3. **Safety Guarantees**:
   - The model ensures landing is always possible from any flight state
   - The system prevents dangerous transitions (e.g., landing from high altitude)
   - Non-deterministic choices allow for emergency handling

## Conclusion 🏁

This project demonstrates the application of formal methods to ensure safety and optimize performance in drone delivery systems. By combining ASP for route optimization with temporal logic verification, we achieved:

1. Optimal delivery routes that balance distance and energy consumption
2. Formal verification of critical safety properties
3. Comprehensive modeling of drone maneuvers and altitude management

The approach provides a strong foundation for developing reliable and efficient autonomous drone delivery systems.

## Future Work 🔮

Potential extensions of this work include:
- Incorporating real-time obstacle avoidance
- Modeling multi-drone coordination
- Adding weather condition variables
- Implementing battery recharging constraints
- Expanding to 3D path planning with building avoidance

## Technical Stack 🛠️

- **Answer Set Programming**: Potassco/clingo
- **Model Checking**: NuSMV
- **Temporal Logic**: LTL and CTL
- **Visualization**: Graph representations

---

*This project was completed as part of the Formal Methods for Security course at the University of Bari "Aldo Moro", Master's Degree in Cyber Security.*
