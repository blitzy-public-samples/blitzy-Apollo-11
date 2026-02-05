# Apollo Guidance Computer - Fault Recovery and State Preservation Subsystem

## Overview

The Apollo Guidance Computer (AGC) Fault Recovery and State Preservation subsystem provides fault-tolerant restart capability that enables the computer to resume mission-critical programs after transient hardware faults without losing progress. This system was essential for the success of the Apollo missions, where computer reliability was paramount to crew safety.

### Purpose and Capabilities

The recovery system was designed to address the unique challenges of space computing in the 1960s:

- **Hardware Fault Tolerance**: Cosmic radiation and power transients could cause spurious hardware interrupts (GOJAM). The recovery system allowed the AGC to resume operation without requiring a full system reset.

- **State Checkpointing**: Through the Phase Table system, programs could periodically checkpoint their state, enabling recovery to a known-good point rather than starting from scratch.

- **Priority-Based Recovery**: The Executive scheduler integration ensured that mission-critical guidance programs received priority during recovery, maintaining flight safety.

### Design Constraints

The recovery system operated within severe hardware constraints:

| Resource | Constraint | Impact on Design |
|----------|-----------|------------------|
| RAM (Erasable Memory) | 2,048 words | Limited checkpoint storage; phase encoding optimized for space efficiency |
| ROM (Fixed Memory) | 36,864 words | Restart tables in fixed memory; recovery logic carefully optimized |
| CPU Speed | ~12 µs per instruction | Recovery must complete quickly to maintain guidance timing |
| Reliability | Critical mission functions | Multiple validation stages before resuming execution |

### Success Metric

A developer can trace P63 Landing Guidance resumption after a transient hardware fault without requiring IMU realignment—the phase table preserves sufficient state to continue powered descent guidance calculations from the last checkpoint.

---

## Component Inventory

The recovery system consists of several interconnected components distributed across multiple source files. The following table provides a complete inventory:

### Core Recovery Entry Points

| Component | File Location | Address/Line | Purpose |
|-----------|--------------|--------------|---------|
| **GOPROG** | FRESH_START_AND_RESTART.agc | Line 206 (Address 4000) | Hardware restart entry point at GOJAM vector |
| **DOFSTART** | FRESH_START_AND_RESTART.agc | Line 65 | Fresh start entry - complete reinitialization with engine control |
| **DOFSTRT1** | FRESH_START_AND_RESTART.agc | Line 71 | Fresh start continuation without engine control |
| **DORSTART** | FRESH_START_AND_RESTART.agc | Line 247 | Controlled restart entry after E-memory validation |
| **RESTARTS** | RESTARTS_ROUTINE.agc | Line 35 | Restart dispatch routine - routes to job/task/longcall handlers |

Source: Luminary099/FRESH_START_AND_RESTART.agc:65-247, Luminary099/RESTARTS_ROUTINE.agc:35

### Initialization Subroutines

| Component | File Location | Purpose |
|-----------|--------------|---------|
| **STARTSUB** | FRESH_START_AND_RESTART.agc:389 | Common initialization for fresh start and restart |
| **STARTSB1** | FRESH_START_AND_RESTART.agc:399 | Timer initialization (TIME3, TIME4, TIME5) |
| **STARTSB2** | FRESH_START_AND_RESTART.agc:428 | Channel and waitlist initialization |

Source: Luminary099/FRESH_START_AND_RESTART.agc:389-428

### Phase Clearing Routines

| Component | File Location | Purpose |
|-----------|--------------|---------|
| **MR.KLEAN** | FRESH_START_AND_RESTART.agc:180 | Clears all phase groups (1-6) |
| **P00KLEAN** | FRESH_START_AND_RESTART.agc:185 | Clears phase groups 4, 3, 1, 5, 6 (preserves group 2) |
| **V37KLEAN** | FRESH_START_AND_RESTART.agc:188 | Clears phase groups 1, 3, 5, 6 (preserves groups 2, 4) |

Source: Luminary099/FRESH_START_AND_RESTART.agc:180-200

### Phase Table Management

| Component | File Location | Purpose |
|-----------|--------------|---------|
| **PHASCHNG** | PHASE_TABLE_MAINTENANCE.agc:228 | Primary phase change entry point |
| **2PHSCHNG** | PHASE_TABLE_MAINTENANCE.agc:202 | Double phase change (two groups simultaneously) |
| **NEWMODEX** | PHASE_TABLE_MAINTENANCE.agc:40 | Update MODREG (major mode) from fixed memory |
| **NEWMODEA** | PHASE_TABLE_MAINTENANCE.agc:44 | Update MODREG from accumulator |
| **CHECKMM** | PHASE_TABLE_MAINTENANCE.agc:52 | Verify current major mode |

Source: Luminary099/PHASE_TABLE_MAINTENANCE.agc:40-228

---

## System Architecture Integration

The recovery system integrates tightly with two primary AGC subsystems: the Executive (job scheduler) and the Waitlist (task scheduler).

### Executive Scheduler Integration

The Executive provides priority-based job scheduling for the AGC. Recovery system integration occurs at several points:

```
┌─────────────────────────────────────────────────────────────────┐
│                    EXECUTIVE SCHEDULER                          │
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  FINDVAC    │  │   NOVAC     │  │  ENDOFJOB   │             │
│  │ (Job+VAC)   │  │ (Job only)  │  │ (Terminate) │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                     │
│         └────────────────┼────────────────┘                     │
│                          │                                      │
│                   ┌──────▼──────┐                               │
│                   │  Core Sets  │  8 job contexts              │
│                   │  PRIORITY   │  Priority registers          │
│                   │  PUSHLOC    │  Push-down pointers          │
│                   └─────────────┘                               │
└─────────────────────────────────────────────────────────────────┘
                           │
                           │ On Restart
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                    RESTARTS ROUTINE                             │
│                                                                 │
│  Phase table lookup → Dispatch via FINDVAC/NOVAC               │
│  Positive priority = FINDVAC (needs VAC area)                   │
│  Negative priority = NOVAC (basic job, no VAC)                  │
└─────────────────────────────────────────────────────────────────┘
```

**Key Integration Points:**

1. **Job Restart via FINDVAC**: Interpretive jobs requiring VAC (Vector Accumulator) areas are restarted through FINDVAC. The restart tables store positive priorities for these jobs.

2. **Job Restart via NOVAC**: Basic jobs not requiring VAC areas are restarted through NOVAC. The restart tables store negative priorities (which are complemented before use).

3. **Priority Preservation**: Job priorities are stored in PRDTTAB (Priority/Delta-Time Table) and restored during restart.

4. **Core Set Reallocation**: The Executive's eight core sets are reinitialized during STARTSUB, making all job slots available for restart scheduling.

Source: Luminary099/EXECUTIVE.agc:37-60, Luminary099/RESTARTS_ROUTINE.agc:153-166

### Waitlist Scheduler Integration

The Waitlist provides time-based task scheduling. Recovery integration handles delta-time recalculation:

```
┌─────────────────────────────────────────────────────────────────┐
│                    WAITLIST SCHEDULER                           │
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  WAITLIST   │  │  LONGCALL   │  │   TASKOVER  │             │
│  │ (≤163.83s)  │  │ (>163.83s)  │  │ (Complete)  │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                     │
│         └────────────────┼────────────────┘                     │
│                          │                                      │
│                   ┌──────▼──────┐                               │
│                   │   LST1/2    │  Time-ordered task list       │
│                   │   TBASE1-6  │  Group time bases             │
│                   │   LONGBASE  │  Long call time base          │
│                   └─────────────┘                               │
└─────────────────────────────────────────────────────────────────┘
                           │
                           │ On Restart
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                    RESTARTS ROUTINE                             │
│                                                                 │
│  FINDTIME: Calculate remaining time = TBASE - TIME1 + stored_dt│
│  ITSLNGCL: Longcall recovery via LONGBASE                       │
│  IMEDIATE: If time elapsed, restart immediately (10ms)          │
└─────────────────────────────────────────────────────────────────┘
```

**Key Integration Points:**

1. **Time Base Preservation**: Each restart group maintains a TBASE register capturing the TIME1 value when the phase was set. This enables accurate delta-time recalculation.

2. **Waitlist Reinitialization**: STARTSUB reinitializes LST1/LST2 (waitlist delta-time and address tables) before processing restart groups.

3. **Delta-Time Recovery**: FINDTIME in RESTARTS_ROUTINE calculates remaining time using: `remaining = stored_delta - (TIME1 - TBASE)`

4. **Immediate Restart**: If calculated time is negative (task overdue), the task restarts immediately with minimal delay.

Source: Luminary099/RESTARTS_ROUTINE.agc:82-137, Luminary099/FRESH_START_AND_RESTART.agc:475-505

### Alarm System Integration

The Alarm and Abort system provides error reporting and recovery escalation:

| Entry Point | Purpose | Restart Impact |
|------------|---------|----------------|
| **ALARM** | Display program alarm, continue execution | Recovery continues |
| **PRIOLARM** | High-priority alarm display | Recovery continues |
| **BAILOUT** | Abort current job on fatal error | Job terminated, recovery escalated |
| **POODOO** | Abort with restart to P00 | Fresh start initiated |

Source: Luminary099/ALARM_AND_ABORT.agc:50-100

---

## Hardware Dependencies

The recovery system relies on specific AGC hardware features for proper operation.

### GOJAM Vector (Address 4000)

The hardware GOJAM (Go to JAM) signal triggers automatic transfer to address 4000 (octal) when:

- **Power Transient**: Voltage fluctuation triggers hardware reset
- **Oscillator Failure**: Clock circuit instability detected
- **Parity Error**: Memory parity check failure (rare in flight)
- **Watchdog Timeout**: Software failed to reset counter (NEWJOB not set)
- **Manual Reset**: Ground command or crew action

The GOJAM vector unconditionally transfers control to GOPROG at address 4000.

### Automatic Register Preservation

Hardware automatically preserves these registers during GOJAM:

| Register | Size | Purpose |
|----------|------|---------|
| A (Accumulator) | 16 bits | General computation |
| L (Lower Accumulator) | 16 bits | Double-precision operations |
| Q (Return Address) | 16 bits | Subroutine return |
| BBANK | 4 bits | Erasable bank selection |

**Software must preserve:**

```agc
GOPROG		INCR	REDOCTR		# ADVANCE RESTART COUNTER.
		LXCH	Q		# SAVE Q REGISTER
		EXTEND
		ROR	SUPERBNK	# COMBINE WITH SUPERBANK
		DXCH	RSBBQ		# STORE Q AND SUPERBNK
```

The RSBBQ register pair preserves Q and SUPERBNK for potential cross-bank return addressing after recovery.

Source: Luminary099/FRESH_START_AND_RESTART.agc:206-211

### Channel I/O Preservation

During restart, specific I/O channels require careful handling:

| Channel | Function | Restart Handling |
|---------|----------|------------------|
| 5, 6 | RCS Jet Commands | Cleared during fresh start to prevent spurious firing |
| 11 | Engine On/Off | Preserved based on ENGONBIT flag |
| 12 | ISS/Optics Control | Coarse align, zero CDU, enable counter preserved |
| 13 | Radar/Test | Telemetry flags, T6RUPT enable preserved |
| 14 | Gyro/Engine | Gyro enable flag preserved |

Source: Luminary099/FRESH_START_AND_RESTART.agc:89-98, 446-469

### Timer Dependencies

The recovery system depends on hardware timers:

| Timer | Resolution | Use in Recovery |
|-------|-----------|-----------------|
| TIME1 | 10 ms | Current time reference for delta calculations |
| TIME2 | 163.84 s | Extended time (TIME2:TIME1 = 28-bit counter) |
| TIME3 | 10 ms | T3RUPT scheduling, reinitialized on restart |
| TIME4 | 10 ms | T4RUPT for DSKY, reinitialized on restart |
| TIME5 | 10 ms | T5RUPT for DAP, reinitialized on restart |
| TIME6 | 1/1600 s | T6RUPT for fine timing, disabled then restored |

Source: Luminary099/FRESH_START_AND_RESTART.agc:400-426

---

## Data Flow: Fault Detection to Resumption

The complete recovery sequence follows a defined path from hardware fault detection through program resumption.

### Recovery Decision Flowchart

```mermaid
flowchart TD
    A[Hardware Fault Detected] --> B[GOJAM Interrupt]
    B --> C[GOPROG Entry at Address 4000]
    C --> D[Increment REDOCTR<br/>Save Q and SUPERBNK to RSBBQ]
    D --> E{ERESTORE Check}
    
    E -->|ERESTORE ≠ 0<br/>and ≠ SKEEP7| F[E-Memory May Be Corrupted]
    E -->|ERESTORE = SKEEP7| G[Restore E-Memory from SKEEP5/6]
    E -->|ERESTORE = 0| H[E-Memory Valid]
    
    F --> I[DOFSTART<br/>Fresh Start]
    G --> H
    H --> J[DORSTART<br/>Controlled Restart]
    
    J --> K[STARTSUB Initialization]
    K --> L{OSC FAIL + AGC WARN?}
    
    L -->|Both On| I
    L -->|No| M{MARK REJECT + ERROR RESET?}
    
    M -->|Both On| I
    M -->|No| N[GOPROG2/GOPROG3]
    
    N --> O[Phase Table Validation<br/>PCLOOP]
    O --> P{All Phases<br/>PHASE = -(-PHASE)?}
    
    P -->|Mismatch| Q[Alarm 1107<br/>Phase Table Failure]
    Q --> R[DOFSTRT1<br/>Fresh Start without Engine]
    
    P -->|Match| S{Any Active<br/>Groups?}
    
    S -->|No| T[GOTOPOOH<br/>Go to P00]
    S -->|Yes| U[RESTARTS Routine]
    
    U --> V[Process Each Active Group]
    V --> W{Phase Type?}
    
    W -->|Type A| X[Fixed Phase<br/>Table Lookup]
    W -->|Type B| Y[Variable Job +<br/>Fixed Entry]
    W -->|Type C| Z[Variable Phase<br/>Direct Storage]
    
    X --> AA{Entry Type?}
    Y --> AA
    Z --> AA
    
    AA -->|Job| AB[Schedule via<br/>FINDVAC/NOVAC]
    AA -->|Waitlist| AC[Schedule via<br/>WAITLIST with delta-t]
    AA -->|Longcall| AD[Schedule via<br/>LONGCALL with time]
    
    AB --> AE[Resume Mission Programs]
    AC --> AE
    AD --> AE
    
    AE --> AF[ENDRSTRT<br/>Return to DUMMYJOB]
```

### Sequence of Operations

**1. Hardware Fault Detection (GOJAM Trigger)**

When hardware detects a fault condition, the GOJAM signal asserts:
- CPU halts current instruction
- Program counter forced to address 4000 (octal)
- Registers A, L, Q, BBANK automatically saved
- Interrupt system inhibited (INHINT state)

**2. GOPROG Entry and Register Save**

```agc
GOPROG		INCR	REDOCTR		# Count restart for telemetry
		LXCH	Q		# Q → L
		EXTEND
		ROR	SUPERBNK	# L | SUPERBNK → A
		DXCH	RSBBQ		# Save for potential bank-switched return
```

**3. E-Memory Validation**

The ERESTORE/SKEEP7 mechanism detects whether a memory-modifying operation (ERASCHK) was interrupted:

```
IF ERESTORE = +0:
    E-memory is valid, proceed to DORSTART
ELSE IF ERESTORE = SKEEP7 (and valid range):
    Restore interrupted E-memory from SKEEP5/SKEEP6
    Clear ERESTORE, proceed to DORSTART
ELSE:
    E-memory may be corrupted, force DOFSTART
```

Source: Luminary099/FRESH_START_AND_RESTART.agc:221-246

**4. Phase Table Validation (PCLOOP)**

After passing E-memory checks, PCLOOP verifies phase table consistency:

```agc
GOPROG3		CAF	NUMGRPS		# NUMGRPS = 5 (groups 1-6, 0-indexed)
PCLOOP		TS	MPAC +5
		DOUBLE
		EXTEND
		INDEX	A
		DCA	-PHASE1		# Load PHASE and -PHASE for group
		EXTEND
		RXOR	LCHAN		# XOR: result must be -0 for match
		CCS	A
		TCF	PTBAD		# Any non-zero = mismatch
		TCF	PTBAD
		CCS	MPAC +5		# Next group
		TCF	PCLOOP
```

The PHASE/-PHASE redundancy detects single-bit corruption in phase registers.

Source: Luminary099/FRESH_START_AND_RESTART.agc:290-304

**5. Restart Dispatch (RESTARTS Routine)**

For each active group, RESTARTS decodes the phase type and schedules recovery:

```
Phase Types:
- Type A: Fixed phase (table lookup in CADRTAB/PRDTTAB)
- Type B: Variable job + fixed table entry
- Type C: Variable phase (direct CADR in PHSNAME registers)

Entry Types:
- Job: Positive priority → FINDVAC, Negative → NOVAC
- Waitlist: Negative 2CADR in table, delta-time in PRDTTAB
- Longcall: Negative GENADR, positive BBCON, delta in PRDTTAB
```

Source: Luminary099/RESTARTS_ROUTINE.agc:35-324

**6. Program Resumption**

After scheduling all pending jobs/tasks, control transfers to ENDRSTRT which jumps to DUMMYJOB. The Executive then dispatches the highest-priority job, resuming mission operations.

---

## Traceability Matrix: Recovery Entry Points to Mission Phases

The following matrix maps restart groups to their associated mission phases and programs:

| Restart Group | Primary Mission Phase | Key Programs | Recovery Scope | Example SPOT Entries |
|--------------|----------------------|--------------|----------------|---------------------|
| **Group 1** | General Timing | Ullage control tasks | Timer-based tasks for auxiliary functions | 1.3SPOT: ULLGTASK |
| **Group 2** | State Vector Integration | P20/P22 Tracking, P25 | Navigation state preservation, rendezvous | 2.3SPOT: STATEINT, 2.5SPOT: STATINT1, 2.7SPOT: P20LEMC1 |
| **Group 3** | Guidance Calculations | S40.13 Steering | Steering computation restart | 3.3SPOT: ZOOM, 3.5SPOT: S40.13 |
| **Group 4** | Powered Flight Sequences | P12, P40, P63, P70, P71 | **Powered descent/ascent recovery** | 4.7SPOT: TIG-0, 4.27SPOT: P70A, 4.31SPOT: P71A |
| **Group 5** | Servicer Functions | SERVICER, NORMLIZE | Sensor processing, attitude reference | 5.2SPOT: NORMLIZE, 5.4SPOT: SERVICER |
| **Group 6** | Clock/Timing | CLOKTASK, TIMEDIDR | Time reference maintenance | 6.3SPOT: CLOKTASK, 6.5SPOT: TIMEDIDR |

Source: Luminary099/RESTART_TABLES.agc:106-294

### Group 4: Powered Flight Recovery Detail

Group 4 is critical for landing and abort scenarios. Key restart points include:

| SPOT Entry | Priority/Time | Target | Mission Function |
|-----------|---------------|--------|------------------|
| 4.2SPOT | 2500 cs | TIG-5 | Ignition minus 5 seconds |
| 4.5SPOT | 50 cs | ULLAGOFF | Ullage cutoff |
| 4.7SPOT | 500 cs | TIG-0 | Ignition sequence |
| 4.13SPOT | PRIO 12 | POSTBURN | Post-burn cleanup |
| 4.27SPOT | PRIO 52 | P70A | Abort from powered descent |
| 4.31SPOT | PRIO 52 | P71A | Abort from braking phase |

### P63 Landing Guidance Recovery Path

During P63 (Powered Descent Guidance), a hardware restart follows this recovery path:

1. **GOJAM** → GOPROG entry
2. **E-memory check** → Valid (descent state preserved)
3. **Phase validation** → Group 4 phase non-zero (powered flight active)
4. **RESTARTS dispatch** → Lookup 4.XSPOT entry
5. **TIG recovery** → Restart TIG countdown or continue burn
6. **Guidance resume** → SERVICER (Group 5) restarts sensor processing
7. **No IMU realignment** → Phase tables preserve orientation reference

The phase table system stores sufficient state that P63 can resume guidance calculations without requiring a full IMU realignment procedure (which would take several minutes and was not feasible during powered descent).

---

## Historical Context: Apollo 11 Landing Alarms

The AGC recovery system proved its worth during the historic Apollo 11 lunar landing on July 20, 1969.

### The 1202 Alarms

During the powered descent phase (P63/P66), the AGC triggered multiple 1202 "Executive Overflow" alarms:

| Time (GET) | Alarm | Phase | Altitude |
|-----------|-------|-------|----------|
| 102:38:22 | 1202 | P63 | ~33,500 ft |
| 102:38:58 | 1202 | P63 | ~27,000 ft |
| 102:39:02 | 1201 | P63 | ~26,000 ft |
| 102:42:17 | 1202 | P66 | ~3,000 ft |
| 102:42:43 | 1202 | P66 | ~2,000 ft |

### Root Cause

The alarms were caused by the rendezvous radar being left in a mode that generated excessive interrupts:

```
┌─────────────────────────────────────────────────────────────────┐
│                    EXECUTIVE RESOURCE CONTENTION                │
│                                                                 │
│  Normal Operation:                                              │
│  ┌─────────────┐                                               │
│  │ P63 Guidance│ Priority 20   ────► Uses core sets 0-2       │
│  │ SERVICER    │ Priority 22   ────► Uses core sets + VAC     │
│  └─────────────┘                                               │
│                                                                 │
│  Apollo 11 Situation:                                           │
│  ┌─────────────┐                                               │
│  │ P63 Guidance│ Priority 20   ────► Core set 0               │
│  │ SERVICER    │ Priority 22   ────► Core set 1               │
│  │ RR Tracking │ Priority 17   ────► Flooding with tasks      │
│  │ T4RUPT jobs │ Low priority  ────► Queued, never execute    │
│  └─────────────┘                                               │
│                      │                                          │
│                      ▼                                          │
│              ┌───────────────┐                                  │
│              │  All 8 core   │                                  │
│              │  sets in use  │                                  │
│              │  ALARM 1202!  │                                  │
│              └───────────────┘                                  │
└─────────────────────────────────────────────────────────────────┘
```

The rendezvous radar (scheduled for lunar orbit rendezvous) was stealing Executive cycles by requesting position updates. These low-priority tasks accumulated, exhausting available core sets.

### Why the Mission Continued

The recovery system's priority-based design saved the landing:

1. **Priority Precedence**: P63 guidance (priority 20) and SERVICER (priority 22) had higher priority than the radar tracking tasks (priority 17).

2. **Graceful Degradation**: When BAILOUT was called due to core set exhaustion, only the lowest-priority pending job was abandoned. Critical guidance continued.

3. **Phase Table Preservation**: The phase tables for Group 4 (powered flight) and Group 5 (servicer) were never corrupted. After each alarm cleared, guidance programs resumed from their last checkpoint.

4. **Real-Time Decision**: Flight controller Steve Bales made the "GO" call based on understanding that the alarms indicated resource exhaustion, not guidance failure.

### Alarm 1201 vs 1202

| Alarm | Meaning | Cause | Impact |
|-------|---------|-------|--------|
| 1201 | No VAC areas | All 5 VAC areas in use | FINDVAC jobs cannot start |
| 1202 | No core sets | All 8 core sets in use | No new jobs can be scheduled |

Both alarms indicate Executive overflow. The recovery action (BAILOUT) terminates the requesting job and continues with higher-priority work.

Source: Luminary099/EXECUTIVE.agc:147 (1201), Luminary099/EXECUTIVE.agc:208 (1202)

---

## Luminary099 vs Comanche055 Differences

The recovery system exists in both the Lunar Module software (Luminary099) and the Command Module software (Comanche055). While the core mechanisms are identical, there are module-specific differences:

### Structural Similarities

| Component | Luminary099 | Comanche055 | Notes |
|-----------|-------------|-------------|-------|
| GOPROG entry | Line 206 | Similar | Identical core logic |
| Phase encoding | Types A/B/C | Types A/B/C | Same encoding scheme |
| RESTARTS dispatch | Lines 35-324 | Similar | Same algorithm |
| E-memory validation | ERESTORE/SKEEP7 | ERESTORE/SKEEP7 | Identical mechanism |
| Phase groups | 1-6 | 1-6 | Same group structure |

### Key Differences

| Aspect | Luminary099 (LM) | Comanche055 (CM) |
|--------|------------------|------------------|
| **Restart Tables** | LM-specific programs (P63, P70, P71) | CM-specific programs (re-entry, navigation) |
| **Flag Initialization** | Landing radar flags | No landing radar |
| **Engine Control** | Descent/ascent engine bits | SPS engine control |
| **DAP Integration** | LM DAP (DAPIDLER) | CM DAP |
| **Page Numbers** | 211-237 (FRESH_START) | 181-210 (FRESH_START) |

### Common Design Principles

Both modules share:

1. **Phase table redundancy** (PHASE/-PHASE pairs)
2. **Hierarchical restart** (fresh start → controlled restart → phase recovery)
3. **Priority-based Executive integration**
4. **Time-base recovery** (TBASE1-6, LONGBASE)
5. **Alarm escalation** (1107 → fresh start)

Source: Luminary099/FRESH_START_AND_RESTART.agc, Comanche055/FRESH_START_AND_RESTART.agc

---

## Terminology Translations

The following table maps 1960s AGC terminology to modern software engineering equivalents:

| 1960s AGC Term | Modern Equivalent | Description |
|----------------|-------------------|-------------|
| Phase Tables | State Checkpointing | Mechanism to save program state at defined points |
| GOPROG | Hardware Interrupt Handler | Entry point for hardware-triggered recovery |
| CADRTAB | Recovery Routing Table | Table mapping phase values to restart addresses |
| PRDTTAB | Priority/Time Storage | Table storing job priorities or task delta-times |
| Temporal Multiplexing | Cooperative Multitasking | Multiple jobs sharing single CPU via voluntary yields |
| PHASCHNG | State Checkpoint Update | API to update recovery checkpoint |
| Executive | Priority-Based Job Scheduler | Long-running job management subsystem |
| Waitlist | Time-Based Task Scheduler | Short-duration task scheduling (≤163.84s) |
| FINDVAC | Job with VAC Area Allocation | Start job requiring vector accumulator |
| NOVAC | Job without VAC Area | Start basic job (no interpreter) |
| 2CADR | Two-Word Bank-Switched Address | Full address including bank information |
| BBCON | Bank-Bank Configuration Word | Encoding of fixed and erasable bank settings |
| GOJAM | Hardware Reset/Restart | Signal forcing CPU to address 4000 |
| Fresh Start | Cold Boot | Complete reinitialization |
| Controlled Restart | Warm Boot | Recovery with state preservation |
| Core Set | Job Context | Set of registers for one job's state |
| VAC Area | Interpreter Workspace | Memory area for interpretive program variables |
| TBASE | Time Base Register | Timestamp when phase was set |
| LONGBASE | Long Call Time Base | Extended timestamp for calls >163.84s |

---

## Cross-References

For detailed information on specific aspects of the recovery system, see:

- **[Phase Table Maintenance](PHASE_TABLE_MAINTENANCE.md)**: Detailed documentation of PHASE1-6 registers, CADRTAB/PRDTTAB structure, and Type A/B/C phase encoding
- **[Restart Flow](RESTART_FLOW.md)**: Step-by-step sequence diagram of the complete restart process from GOPROG through program resumption
- **[Alarm Reference](ALARM_REFERENCE.md)**: Analysis of recovery-related alarms (1107, 1201, 1202) including root causes and recovery actions
- **[Complete Alarm Listing](../../../ALARMS.md)**: Repository-wide reference for all AGC alarm codes

---

## Source Citations

| Section | Primary Source | Lines |
|---------|---------------|-------|
| GOPROG Entry | Luminary099/FRESH_START_AND_RESTART.agc | 206-248 |
| Phase Clearing | Luminary099/FRESH_START_AND_RESTART.agc | 180-200 |
| E-Memory Validation | Luminary099/FRESH_START_AND_RESTART.agc | 221-246 |
| Phase Table Validation | Luminary099/FRESH_START_AND_RESTART.agc | 290-350 |
| Initialization | Luminary099/FRESH_START_AND_RESTART.agc | 389-579 |
| Phase Change API | Luminary099/PHASE_TABLE_MAINTENANCE.agc | 84-300 |
| Restart Dispatch | Luminary099/RESTARTS_ROUTINE.agc | 35-324 |
| Restart Tables | Luminary099/RESTART_TABLES.agc | 29-294 |
| Executive (1201/1202) | Luminary099/EXECUTIVE.agc | 134-208 |
| CM Recovery | Comanche055/FRESH_START_AND_RESTART.agc | 34-100 |

---

*This documentation is part of the Apollo Guidance Computer Fault Recovery Architecture series. For the complete documentation set, see the [docs/architecture/recovery/](.) directory.*
