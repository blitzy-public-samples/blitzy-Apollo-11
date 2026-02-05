# Apollo Guidance Computer Alarm Code Reference

This document provides a comprehensive reference of all alarm codes used in the Apollo Guidance Computer (AGC) software for both the Lunar Module (Luminary099) and Command Module (Comanche055). These alarm codes are critical for understanding the AGC's fault detection and recovery mechanisms.

## Table of Contents

- [Introduction](#introduction)
- [Alarm Code Format](#alarm-code-format)
- [Alarm Categories](#alarm-categories)
- [Recovery-Related Alarms](#recovery-related-alarms)
  - [Alarm 1107: Phase Table Failure](#alarm-1107-phase-table-failure)
  - [Alarm 1201: Executive Overflow - No VAC Areas](#alarm-1201-executive-overflow---no-vac-areas)
  - [Alarm 1202: Executive Overflow - No Core Sets](#alarm-1202-executive-overflow---no-core-sets)
  - [Alarm 1203: Waitlist Overflow](#alarm-1203-waitlist-overflow)
- [Historical Mission Events: Apollo 11](#historical-mission-events-apollo-11)
- [Luminary099 (Lunar Module) Alarm Codes](#luminary099-lunar-module-alarm-codes)
- [Comanche055 (Command Module) Alarm Codes](#comanche055-command-module-alarm-codes)
- [Terminology Translation](#terminology-translation)
- [Cross-References](#cross-references)

---

## Introduction

The Apollo Guidance Computer's alarm system serves as the primary mechanism for communicating system status and error conditions to both the flight crew and Mission Control. When an alarm condition is detected, the AGC:

1. Stores the alarm code in the FAILREG (failure register) cascade
2. Illuminates the PROG (Program Alarm) light on the DSKY (Display and Keyboard)
3. In severe cases, initiates recovery actions such as software restarts or aborts

The alarm system is tightly integrated with the AGC's fault recovery and state preservation mechanisms, which use phase tables (modern equivalent: state checkpointing) to enable controlled restart after transient faults.

**Primary Audience**: Modern software engineers analyzing historical fault-tolerant systems, aerospace historians, and computer science researchers.

**Source Files**:
- `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc:896-1022`
- `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc:867-959`
- `Luminary099/ALARM_AND_ABORT.agc:1-200`

---

## Alarm Code Format

Alarm codes in the AGC follow the **AAANN** format, where:

| Component | Description | Example |
|-----------|-------------|---------|
| **AAA** | General area code (octal) | `012` for Executive alarms |
| **NN** | Specific alarm number within area | `01` or `02` |

Alarm codes are stored and displayed in **octal** (base-8) notation, a common convention in 1960s computing systems. When reading alarm codes:

- Each digit ranges from 0-7
- The format typically uses 5 octal digits
- Leading zeros may be omitted in some contexts

**Example**: Alarm `01202` breaks down as:
- Area `012` = Executive/Job Scheduler area
- Number `02` = No core sets available

The alarm code is passed to the ALARM routine following the calling instruction:

```agc
TC      ALARM           # Call alarm subroutine
OCT     1202            # Alarm code follows immediately
                        # Execution continues here after ALARM returns
```

---

## Alarm Categories

The AGC classifies alarms into four categories based on severity and required response. These categories are indicated by markers in the alarm documentation:

| Marker | Category | Description | System Response |
|--------|----------|-------------|-----------------|
| `*` | Abort (Software Restart) | Results in a software restart via BAILOUT | Current job terminated; system attempts controlled restart |
| `**` | Serious Abort (R00) | Results in the program going to R00 (POODOO) | All active programs terminated; requires crew intervention |
| `P` | Priority Alarm | Displayed via priority display (V05N09) | Non-abortive; alerts crew while allowing continued operation |
| (none) | Non-Abortive | Informational alarm only | Program continues execution; alarm stored in FAILREG |

**Understanding Abort Types**:

- **BAILOUT** (`*` alarms): Triggers a software restart. The AGC terminates the current job and attempts to resume operations using the phase table (state checkpointing) mechanism. This is a recoverable condition.

- **POODOO** (`**` alarms): A more severe abort that terminates all active programs and returns the system to an idle state (P00/R00). The name "POODOO" was Apollo-era humor for a serious situation. Requires crew action to restart programs.

**Source**: `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc:1014-1021`

---

## Recovery-Related Alarms

The following alarms are directly related to the AGC's fault recovery system. Understanding these alarms is essential for analyzing how the AGC maintained system integrity during the Apollo missions.

### Alarm 1107: Phase Table Failure

| Property | Value |
|----------|-------|
| **Code** | `01107` (octal) |
| **Category** | Non-abortive alarm, but triggers fresh start |
| **Set By** | RESTART (phase table validation) |
| **Recovery Action** | DOFSTART (fresh start) |

**Description**:

Alarm 1107 indicates that the phase table consistency check has failed during a restart sequence. The phase table is the AGC's state checkpointing mechanism, storing the current execution state of up to 6 program groups (PHASE1-PHASE6 registers).

During a hardware restart (GOPROG entry), the AGC validates phase table integrity by checking that each phase register matches its complement (stored in the corresponding -PHASE register). If this validation fails, the erasable memory may be corrupted, and a controlled restart is not possible.

**Trigger Mechanism**:

```agc
# Source: Luminary099/FRESH_START_AND_RESTART.agc:347-350
PTBAD   TC      ALARM           # SET ALARM TO SHOW PHASE TABLE FAILURE.
        OCT     1107

        TCF     DOFSTRT1        # Branch to fresh start
```

**Modern Equivalent**: This alarm indicates that the state checkpoint data has been corrupted. The system cannot perform a warm restart and must perform a cold restart (fresh start), losing all program state.

**Impact**: All active programs are terminated. The AGC reinitializes to a clean state, and the crew must manually restart any required programs.

---

### Alarm 1201: Executive Overflow - No VAC Areas

| Property | Value |
|----------|-------|
| **Code** | `01201` (octal) |
| **Category** | `*` Abort (Software Restart via BAILOUT) |
| **Set By** | EXEC (Executive/Job Scheduler) |
| **Recovery Action** | BAILOUT - software restart |

**Description**:

Alarm 1201 indicates that a job requiring a VAC (Vector Accumulator) area was requested, but all 5 VAC areas are currently in use. VAC areas are 44-word scratch pad memory regions used by interpretive programs for temporary storage during complex calculations.

The Executive (modern equivalent: priority-based job scheduler) allocates VAC areas to jobs that need them. When FINDVAC is called and no VAC areas are available, the system cannot schedule the requested job.

**Trigger Mechanism**:

```agc
# Source: Luminary099/EXECUTIVE.agc:144-147
        LXCH    EXECTEM1
        CA      Q
        TC      BAILOUT1
        OCT     1201            # NO VAC AREAS.
```

**Root Cause Analysis**:

This alarm typically occurs when:
1. Multiple computationally intensive programs are running simultaneously
2. A lower-priority program is holding a VAC area while a higher-priority program needs one
3. The system is under heavy computational load (as during powered descent)

**Impact**: The current job requesting the VAC area is terminated via BAILOUT. The system attempts a controlled restart using the phase table mechanism. Other jobs may continue if they don't require the terminated job's services.

---

### Alarm 1202: Executive Overflow - No Core Sets

| Property | Value |
|----------|-------|
| **Code** | `01202` (octal) |
| **Category** | `*` Abort (Software Restart via BAILOUT) |
| **Set By** | EXEC (Executive/Job Scheduler) |
| **Recovery Action** | BAILOUT - software restart |

**Description**:

Alarm 1202 indicates that a new job was requested, but all 7 core sets are in use. A core set is an 11-register block that stores a job's execution context (priority, return address, and working registers). Every active or dormant job in the system requires a dedicated core set.

**Trigger Mechanism**:

```agc
# Source: Luminary099/EXECUTIVE.agc:205-208
        LXCH    EXECTEM1
        CA      Q
        TC      BAILOUT1        # NO CORE SETS AVAILABLE.
        OCT     1202
```

**Root Cause Analysis**:

This alarm occurs when:
1. Seven jobs are already scheduled (the maximum the AGC can handle)
2. An eighth job is requested before any existing job completes
3. Jobs are accumulating faster than they can be executed

The most famous occurrence of this alarm was during the Apollo 11 lunar landing, where the rendezvous radar was inadvertently drawing Executive resources, causing job queue saturation.

**Impact**: The job that triggered the overflow is terminated via BAILOUT. The system attempts a controlled restart. Critical guidance programs typically have higher priority and continue execution, while lower-priority tasks are shed.

---

### Alarm 1203: Waitlist Overflow

| Property | Value |
|----------|-------|
| **Code** | `01203` (octal) |
| **Category** | `*` Abort (Software Restart via BAILOUT) |
| **Set By** | WAITLIST (Time-based task scheduler) |
| **Recovery Action** | BAILOUT - software restart |

**Description**:

Alarm 1203 indicates that a new waitlist task was requested, but the waitlist is full. The Waitlist (modern equivalent: time-based task scheduler) manages tasks that must execute at specific future times. The AGC can maintain a limited number of pending timed tasks.

**Root Cause**: Too many time-delayed tasks are pending execution. This can occur when:
1. Programs are scheduling many future events
2. Timed tasks are accumulating faster than they expire
3. A timing anomaly causes task scheduling to outpace execution

**Impact**: The task that triggered the overflow is terminated via BAILOUT. The system attempts a controlled restart using the phase table mechanism.

---

## Historical Mission Events: Apollo 11

The 1202 alarm became famous during the Apollo 11 lunar landing on July 20, 1969. Understanding this event provides critical insight into how the AGC's fault-tolerant design enabled mission success despite system overloads.

### The Apollo 11 1202 Alarms During P63 Descent

During the powered descent to the lunar surface, astronauts Neil Armstrong and Buzz Aldrin experienced multiple 1202 (and some 1201) program alarms. These alarms occurred during Program 63 (P63), the Braking Phase of lunar descent.

**Timeline of Events**:

| Mission Time | Event |
|--------------|-------|
| T-5:00 before landing | P63 Braking Phase active |
| T-4:30 | First 1202 alarm |
| Multiple occurrences | Additional 1202 and 1201 alarms |
| T-0:00 | Successful landing in Sea of Tranquility |

**Root Cause**:

The alarms were caused by the rendezvous radar (RR) being left in a mode that caused it to repeatedly request computer cycles, even though it was not needed during landing. Specifically:

1. The rendezvous radar was set to track the Command Module (a backup capability)
2. The radar's data requests were consuming Executive job slots and VAC areas
3. During the computationally intensive descent phase, these additional demands pushed the system to its limits
4. When the Executive could not accommodate new jobs or VAC areas, it triggered 1201/1202 alarms

**Why the Mission Continued**:

The AGC's priority-based scheduling and fault recovery design proved critical:

1. **Priority Scheduling**: Guidance programs (P63, P66) had higher priority than the radar tracking tasks
2. **Graceful Degradation**: When overloaded, the system shed lower-priority tasks while preserving critical guidance
3. **Phase Table Recovery**: The state checkpointing system (phase tables) allowed quick recovery after each BAILOUT
4. **Mission Control Decision**: Flight controller Steve Bales, with support from Jack Garman, determined the alarms were recoverable and called "GO" for landing

**Lessons for Modern Systems**:

This event demonstrates several timeless principles of fault-tolerant system design:

- **Priority-based resource allocation** ensures critical functions continue during overload
- **Graceful degradation** is preferable to complete system failure
- **State checkpointing** enables rapid recovery from transient faults
- **Alarm visibility** allows operators to make informed GO/NO-GO decisions

**Source**: Historical records and `Luminary099/EXECUTIVE.agc:147,208`

---

## Luminary099 (Lunar Module) Alarm Codes

The following table lists all alarm codes defined in the Luminary099 (LM) software. This is the software that flew on the Apollo 11 Lunar Module "Eagle."

**Source**: `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc:896-1022`

### IMU and Alignment Alarms (001xx - 002xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 00105 | `**` | AOTMARK system in use | AOTMARK |
| 00107 | - | More than 5 mark pairs | AOTMARK |
| 00111 | - | Mark missing | AOTMARK |
| 00112 | - | Mark or mark reject not being accepted | AOTMARK |
| 00113 | - | No inbits | AOTMARK |
| 00114 | - | Mark made but not desired | AOTMARK |
| 00115 | - | No marks in last pair to reject | AOTMARK |
| 00206 | - | Zero encode not allowed with coarse align + gimbal lock | IMU Mode Switching |
| 00207 | - | ISS turnon request not present for 90 sec | T4RUPT |
| 00210 | - | IMU not operating | IMU Mode Switch, IMU-2, RD2, P51, P57 |
| 00211 | - | Coarse align error | IMU Mode Switch |
| 00212 | - | PIPA fail but PIPA is not being used | IMU Mode Switch, T4RPT |
| 00213 | - | IMU not operating with turn-on request | T4RUPT |
| 00214 | - | Program using IMU when turned off | T4RUPT |
| 00217 | - | Bad return from IMUSTALL | P51, P52, P57 |
| 00220 | - | IMU not aligned - no REFSMMAT | R02, R47 |

### Gimbal and Attitude Alarms (004xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 00401 | - | Desired gimbal angle yields gimbal lock | INF Align, IMU-2, FINDCDUW |
| 00402 | - | FINDCDUW not controlling attitude | FINDCDUW |
| 00404 | - | Two stars not available in any detent | R59, Lunar Surface |
| 00405 | - | Two stars not available | P52 |
| 00421 | - | W-matrix overflow | INTEGRV |
| 00430 | `**` | Acceleration overflow in integration | Orbital Integration |

### Radar Alarms (005xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 00501 | `P` | Radar antenna out of limits | R23 |
| 00502 | - | Bad radar gimbal angle input | V41N72 |
| 00503 | `P` | Radar antenna designate fail | R21, Non-P in V41N72 |
| 00510 | - | Radar auto discrete not present | R25 |
| 00511 | - | LR not in position 2 or repositioning | SERVICER |
| 00514 | `P` | RR goes out of auto mode while in use | P20 |
| 00515 | - | RR CDU fail discrete present | R25 |
| 00520 | - | Radar rupt not expected at this time | Radar Read |
| 00521 | - | Could not read radar | P20 |
| 00522 | - | Landing radar position change | Radar Read |
| 00523 | `P` | LR antenna didn't achieve position 2 | SERVICER, V60 |
| 00525 | `P` | Delta theta greater than 3 degrees | R22 |
| 00526 | `P` | Range greater than 400 naut. miles | P20, P22 |
| 00527 | `P` | LOS not in mode II coverage while on lunar surface, or vehicle maneuver required | R21, R24 |
| 00530 | `P` | LOS not in mode 2 coverage on lunar surface after 600 secs | R21 |

### Rendezvous Alarms (006xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 00600 | - | Imaginary roots on first iteration | P32, P72 |
| 00601 | - | Perigee altitude CSI < PMIN1 | P32, P72 |
| 00602 | - | Perigee altitude CDH < PMIN2 | P32, P72 |
| 00603 | - | CSI to CDH time < TMIN12 | P32, P72, P33, P73 |
| 00604 | - | CDH to TPI time < TMIN23, or computed CDH time > input TPI time | P32, P72 |
| 00605 | - | Number of iterations exceeds loop maximum | P32, P72 |
| 00606 | - | DV exceeds maximum | P32, P72 |
| 00607 | `**` | No solution from TIME-THETA or TIME-RADIUS | TIMETHET, TIMERAD |
| 00611 | - | No TIG for given elevation angle | P34, P74 |

### Miscellaneous Alarms (007xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 00701 | - | Illegal option code selected | P57 |
| 00777 | - | PIPA fail caused the ISS warning | T4RUPT |

### Executive and System Alarms (011xx - 012xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 01102 | - | AGC self test error | Self Check |
| 01103 | `**` | Unused CCS branch executed | ABORT |
| 01104 | `*` | Delay routine busy | EXEC |
| 01105 | - | Downlink too fast | T4RUPT |
| 01106 | - | Uplink too fast | T4RUPT |
| 01107 | - | Phase table failure - assume erasable memory is suspect | RESTART |
| 01201 | `*` | Executive overflow - no VAC areas | EXEC |
| 01202 | `*` | Executive overflow - no core sets | EXEC |
| 01203 | `*` | Waitlist overflow - too many tasks | WAITLIST |
| 01204 | `**` | Waitlist, VARDELAY, FIXDELAY, or LONGCALL called with zero or negative delta-time | Waitlist Routines |
| 01206 | `**` | Second job attempts to go to sleep via keyboard and display program | PINBALL |
| 01207 | `*` | No VAC areas for marks | AOTMARK |
| 01210 | `*` | Two programs using device at same time | Mode Switching |
| 01211 | `*` | Illegal interrupt of extended verb | AOTMARK |

### Interpreter Alarms (013xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 01301 | - | ARCSIN-ARCCOS argument too large | Interpreter |
| 01302 | `**` | SQRT called with negative argument | Interpreter |

### Guidance Alarms (014xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 01406 | - / `**` | Bad return from ROOTPSRS | Descent Guidance Eqs. / Ignition Algorithm |
| 01407 | - | VG increasing (delta-V accumulated > 90° away from desired thrust vector) | S40.8 |
| 01410 | - | Unintentional overflow in guidance | Descent Guidance Eqs. |
| 01412 | - | Descent IGNALG not converging | P63 |

> **Note**: Alarm 01406 is a POODOO during the ignition algorithm and an ALARM during the actual guidance phase.

### Display Alarms (015xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 01501 | `**` | Keyboard and display alarm during internal use (NVSUB) - abort | PINBALL |
| 01502 | `**` | Illegal flashing display | GOPLAY |
| 01520 | - | V37 request not permitted at this time | V37 |

### Calibration and Test Alarms (016xx - 017xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 01600 | - | Overflow in drift test | IMU 4 |
| 01601 | - | Bad IMU torque | Opt Pre Align Calib, IMU 4 (LEM) |
| 01703 | - | Ignition time slipped | MIDTOAVE |
| 01706 | - | Incorrect program requested for vehicle configuration | P40, P42 |

### DAP Alarms (020xx)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 02000 | `*` | DAP still in progress at next TIME5 rupt | DAP |
| 02001 | - | Jet failures have disabled Y-Z translation | DAP |
| 02002 | - | Jet failures have disabled X translation | DAP |
| 02003 | - | Jet failures have disabled P-rotation | DAP |
| 02004 | - | Jet failures have disabled U-V rotation | DAP |

### ISS Warning Alarms (x7777)

| Code | Category | Description | Set By |
|------|----------|-------------|--------|
| 03777 | - | ICDU fail caused the ISS warning | T4RUPT |
| 04777 | - | ICDU, PIPA fails caused the ISS warning | T4RUPT |
| 07777 | - | IMU fail caused the ISS warning | T4RUPT |
| 10777 | - | IMU, PIPA fails caused the ISS warning | T4RUPT |
| 13777 | - | IMU, ICDU fails caused the ISS warning | T4RUPT |
| 14777 | - | IMU, ICDU, PIPA fails caused the ISS warning | T4RUPT |

---

## Comanche055 (Command Module) Alarm Codes

The following table lists all alarm codes defined in the Comanche055 (CM) software. This is the software that flew on the Apollo 11 Command Module "Columbia."

**Source**: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc:867-959`

### Optics and Mark Alarms (001xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 00110 | - | No mark since last mark reject | SXTMARK | ALARM |
| 00112 | - | Mark not being accepted | SXTMARK | ALARM |
| 00113 | - | No inbits | SXTMARK | ALARM |
| 00114 | - | Mark made but not desired | SXTMARK | ALARM |
| 00115 | - | Optics torque request with switch not at CGC | Ext Verb Optics CDU | ALARM |
| 00116 | - | Optics switch altered before 15 sec zero time elapsed | T4RUPT | ALARM |
| 00117 | - | Optics torque request with optics not available (OPTIND=-0) | Ext Verb Optics CDU | ALARM |
| 00120 | - | Optics torque request with optics not zeroed | T4RUPT | ALARM |
| 00121 | - | CDUs no good at time of mark | SXTMARK | ALARM |
| 00122 | - | Marking not called for | SXTMARK | ALARM |
| 00124 | - | P17 TPI search - no safe pericenter here | TPI Search | ALARM |

### IMU and Alignment Alarms (002xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 00205 | - | Bad PIPA reading | SERVICER | ALARM |
| 00206 | - | Zero encode not allowed with coarse align + gimbal lock | IMU Mode Switching | ALARM |
| 00207 | - | ISS turnon request not present for 90 sec | T4RUPT | ALARM |
| 00210 | - | IMU not operating | IMU Mode Switch, IMU-2, R02, P51 | ALARM, VARALARM |
| 00211 | - | Coarse align error - drive > 2 degrees | IMU Mode Switch | ALARM |
| 00212 | - | PIPA fail but PIPA is not being used | IMU Mode Switch, T4RPT | ALARM |
| 00213 | - | IMU not operating with turn-on request | T4RUPT | ALARM |
| 00214 | - | Program using IMU when turned off | T4RUPT | ALARM |
| 00215 | - | Preferred orientation not specified | P52, P54 | ALARM |
| 00217 | - | Bad return from stall routines | CURTAINS | ALARM2 |
| 00220 | - | IMU not aligned - no REFSMMAT | R02, P51 | VARALARM |

### Gimbal and Attitude Alarms (004xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 00401 | - | Desired gimbal angles yield gimbal lock | IMF Align, IMU-2 | ALARM |
| 00404 | - | Target out of view - turn angle > 90 deg | R52 | PRIOLARM |
| 00405 | - | Two stars not available | P52, P54 | ALARM |
| 00406 | - | Rendezvous navigation not operating | R21, R23 | ALARM |
| 00407 | - | Auto optics request turn angle > 50 deg | R52 | ALARM |
| 00421 | - | W-matrix overflow | INTEGRV | VARALARM |
| 00430 | `*` | Integration abort due to subsurface state vector | All Calls to INTEG | POODOO |

### Rendezvous Alarms (006xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 00600 | - | Imaginary roots on first iteration | P32, P72 | VARALARM |
| 00601 | - | Perigee altitude < PMIN1 | P32, P72 | VARALARM |
| 00602 | - | Perigee altitude < PMIN2 | P32, P72 | VARALARM |
| 00603 | - | CSI to CDH time < PMIN22 | P32, P72, P33, P73 | VARALARM |
| 00604 | - | CDH to TPI time < PMIN23 | P32, P72 | VARALARM |
| 00605 | - | Number of iterations exceeds loop maximum | P32, P72, P37 | VARALARM |
| 00606 | - | DV exceeds maximum | P32, P72 | VARALARM |
| 00607 | `*` | No solution from TIME-THETA or TIME-RADIUS | TIMETHET, TIMERAD | POODOO |
| 00610 | `*` | Lambda less than unity | P37 | POODOO |
| 00611 | - | No TIG for given elevation angle | P34, P74 | VARALARM |
| 00612 | - | State vector in wrong sphere of influence | P37 | VARALARM |
| 00613 | - | Reentry angle out of limits | P37 | VARALARM |

### Miscellaneous Alarms (007xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 00777 | - | PIPA fail caused ISS warning | T4RUPT | VARALARM |

### Executive and System Alarms (011xx - 012xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 01102 | - | CMC self test error | Self Check | ALARM2 |
| 01103 | `*` | Unused CCS branch executed | ABORT | POODOO |
| 01104 | `*` | Delay routine busy | EXEC | BAILOUT |
| 01105 | - | Downlink too fast | T4RUPT | ALARM |
| 01106 | - | Uplink too fast | T4RUPT | ALARM |
| 01107 | - | Phase table failure - assume erasable memory is destroyed | RESTART | ALARM |
| 01201 | `*` | Executive overflow - no VAC areas | EXEC | BAILOUT |
| 01202 | `*` | Executive overflow - no core sets | EXEC | BAILOUT |
| 01203 | `*` | Waitlist overflow - too many tasks | WAITLIST | BAILOUT |
| 01204 | `*` | Negative or zero waitlist call | WAITLIST | POODOO |
| 01206 | `*` | Second job attempts to go to sleep via keyboard and display program | PINBALL | POODOO |
| 01207 | `*` | No VAC area for marks | SXTMARK | BAILOUT |
| 01210 | `*` | Two programs using device at same time | IMU Mode Switch | POODOO |
| 01211 | `*` | Illegal interrupt of extended verb | SXTMARK | BAILOUT |

### Interpreter Alarms (013xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 01301 | - | ARCSIN-ARCCOS argument too large | Interpreter | ALARM |
| 01302 | `*` | SQRT called with negative argument - abort | Interpreter | POODOO |

### Guidance Alarms (014xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 01407 | - | VG increasing | S40.8 | ALARM |
| 01426 | - | IMU unsatisfactory | P61, P62 | ALARM |
| 01427 | - | IMU reversed | P61, P62 | ALARM |

### Display Alarms (015xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 01501 | `*` | Keyboard and display alarm during internal use (NVSUB) - abort | PINBALL | POODOO |
| 01502 | `*` | Illegal flashing display | GOPLAY | POODOO |
| 01520 | - | V37 request not permitted at this time | V37 | ALARM |
| 01521 | `*` | P01 illegally selected | P01, P07 | POODOO |

### Calibration Alarms (016xx - 017xx)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 01600 | - | Overflow in drift test | Opt Pre Align Calib | ALARM |
| 01601 | - | Bad IMU torque | Opt Pre Align Calib | ALARM |
| 01602 | - | Bad optics during verification | OPTALGN Calib (CSM) | ALARM |
| 01703 | - | Insufficient time for integration, TIG was slipped | R41 | ALARM |

### ISS Warning Alarms (x7777)

| Code | Category | Description | Set By | Alarm Routine |
|------|----------|-------------|--------|---------------|
| 03777 | - | ICDU fail caused the ISS warning | T4RUPT | VARALARM |
| 04777 | - | ICDU, PIPA fails caused the ISS warning | T4RUPT | VARALARM |
| 07777 | - | IMU fail caused the ISS warning | T4RUPT | VARALARM |
| 10777 | - | IMU, PIPA fails caused the ISS warning | T4RUPT | VARALARM |
| 13777 | - | IMU, ICDU fails caused the ISS warning | T4RUPT | VARALARM |
| 14777 | - | IMU, ICDU, PIPA fails caused the ISS warning | T4RUPT | VARALARM |

---

## Terminology Translation

The following table provides modern equivalents for 1960s AGC terminology used in the alarm documentation:

| 1960s AGC Term | Modern Equivalent | Description |
|----------------|-------------------|-------------|
| Executive | Priority-Based Job Scheduler | Manages job scheduling and resource allocation |
| Waitlist | Time-Based Task Scheduler | Schedules tasks for future execution at specific times |
| Phase Tables | State Checkpointing | Stores program state for recovery after faults |
| GOPROG | Hardware Interrupt Handler | Entry point for hardware restart recovery |
| CADRTAB | Recovery Routing Table | Table of restart addresses for each phase group |
| PHASCHNG | State Checkpoint Update | Subroutine to update phase table entries |
| VAC Area | Vector Accumulator Scratch Space | 44-word temporary storage for interpretive programs |
| Core Set | Job Context Block | 11-register block storing job execution state |
| FINDVAC | Job with VAC Area Allocation | Creates job requiring VAC scratch space |
| NOVAC | Job without VAC Area | Creates job not requiring VAC scratch space |
| BAILOUT | Recoverable Abort Handler | Terminates job and initiates controlled restart |
| POODOO | Fatal Abort Handler | Terminates all programs and returns to idle state |
| DSKY | Display and Keyboard | Astronaut interface panel |
| FAILREG | Failure Register Cascade | Three-register chain storing alarm codes |
| 2CADR | Two-Word Bank-Switched Address | Full address including bank information |
| BBCON | Bank-Bank Configuration Word | Memory bank configuration register |
| REFSMMAT | Reference Stable Member Matrix | IMU orientation reference |

---

## Cross-References

For more detailed analysis of the AGC's fault recovery system, see the following documentation:

### Recovery Architecture Documentation

- **[Recovery System Overview](docs/architecture/recovery/RECOVERY_OVERVIEW.md)**: Comprehensive overview of the AGC's fault recovery capabilities, including component inventory, system integration, and traceability matrices.

- **[Alarm Reference (Detailed Analysis)](docs/architecture/recovery/ALARM_REFERENCE.md)**: In-depth analysis of recovery-related alarms, including Executive queue impacts, resource exhaustion scenarios, and alarm handling flowcharts.

- **[Restart Flow](docs/architecture/recovery/RESTART_FLOW.md)**: Complete documentation of the hardware restart sequence from GOPROG entry through phase validation to program resumption, with Mermaid sequence diagrams.

- **[Phase Table Maintenance](docs/architecture/recovery/PHASE_TABLE_MAINTENANCE.md)**: Detailed documentation of the PHASE1-6 registers, CADRTAB structure, and Type A/B/C phase encoding mechanisms.

### Source Code Files

- `Luminary099/ALARM_AND_ABORT.agc`: Alarm handling routines (ALARM, BAILOUT, POODOO)
- `Luminary099/EXECUTIVE.agc`: Job scheduler with 1201/1202 alarm triggers
- `Luminary099/FRESH_START_AND_RESTART.agc`: GOPROG entry and phase validation
- `Luminary099/RESTARTS_ROUTINE.agc`: Restart dispatch logic
- `Comanche055/ALARM_AND_ABORT.agc`: CM alarm handling routines
- `Comanche055/EXECUTIVE.agc`: CM job scheduler

### External Resources

- [Virtual AGC Project](http://www.ibiblio.org/apollo): Authoritative AGC technical reference
- [Original Source Scans](http://www.ibiblio.org/apollo/ScansForConversion/Luminary099/): MIT Museum digitized printouts

---

*This document is part of the Apollo 11 AGC source code documentation project. The alarm codes and descriptions are derived directly from the original MIT Instrumentation Laboratory documentation as preserved in the AGC source code.*

*Source: Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc:896-1022, Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc:867-959*
