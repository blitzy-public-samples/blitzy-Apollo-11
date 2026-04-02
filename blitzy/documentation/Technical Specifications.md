# Technical Specification

# 0. Agent Action Plan

## 0.1 Intent Clarification



### 0.1.1 Core Documentation Objective

Based on the provided requirements, the Blitzy platform understands that the documentation objective is to **produce a single concise markdown report** that performs a read-only architectural analysis of the Apollo 11 Guidance Computer (AGC) source code, documenting the engineering tradeoffs, design constraints, and architectural decisions embedded across the Comanche055 (Command Module / Colossus 2A) and Luminary099 (Lunar Module / Luminary 1A) source modules.

- **Category**: Create new documentation
- **Documentation type**: Technical analysis report — architectural decision record with evidence-backed observations drawn directly from source code examination
- **Scope**: The report covers five explicit decision domains identified by the user:
  - Memory and resource constraints and how they shaped code structure
  - Real-time scheduling and priority architecture (Executive, Waitlist, restart protection)
  - Error handling and recovery philosophy (alarm hierarchy, graceful degradation, BAILOUT/POODOO/ALARM architecture)
  - Naming conventions and inline documentation practices (humor, readability, cultural references)
  - Light modern context — how AGC patterns prefigure or contrast with contemporary software engineering

Each requirement translates to a dedicated section in the output report, with specific source file citations, label references, and routine names drawn from the repository at `/Comanche055/` and `/Luminary099/`.

### 0.1.2 Special Instructions and Constraints

The user has specified the following critical directives that govern all documentation output:

- **Read-only analysis** — No source code modifications of any kind. The repository is a historical artifact preserved for fidelity to the original MIT Instrumentation Lab printouts.
- **Scope boundary** — Analysis is strictly limited to AGC source code (LUMINARY and COLOSSUS modules). Ground systems, hardware schematics, and non-AGC artifacts are explicitly excluded.
- **Citation standard** — Every observation in the report MUST reference specific filenames and labels from the repository (e.g., `Comanche055/EXECUTIVE.agc`, label `BAILOUT`, routine `CHANJOB`).
- **Explanatory focus** — Comments in the report must explain WHY decisions were made, not provide line-by-line WHAT narration. The goal is engineering rationale, not code walkthrough.
- **Conciseness mandate** — No verbose prose. Sections must be tight, evidence-backed, and observation-driven.
- **Output format** — Single markdown file as the sole deliverable.
- **Style preferences** — Technical, authoritative tone. Evidence-first structure where claims are immediately supported by file/label citations. Concise paragraphs interleaved with specific code references.

No user-provided templates were supplied. No Figma URLs or design attachments are referenced. The documentation style should follow the repository's own contribution standards (tab indentation width 8 for `.agc` files, UTF-8 encoding, LF line endings per `.editorconfig`).

### 0.1.3 Technical Interpretation

These documentation requirements translate to the following technical documentation strategy:

- To **document memory and resource constraints**, we will create a report section analyzing `Comanche055/ERASABLE_ASSIGNMENTS.agc` (3,785 lines), `Luminary099/ERASABLE_ASSIGNMENTS.agc` (2,635 lines), and `Comanche055/INTER-BANK_COMMUNICATION.agc` (184 lines) — extracting evidence of how fixed-rope/core-rope memory limitations dictated bank-switching architecture, register sharing via `EQUALS`, and the compact notation system (M/SIZE/N convention for erasable classification).
- To **document real-time scheduling and priority architecture**, we will create a report section analyzing `Comanche055/EXECUTIVE.agc` (497 lines), `Luminary099/EXECUTIVE.agc` (503 lines), `Comanche055/WAITLIST.agc` (557 lines), `Luminary099/WAITLIST.agc` (564 lines), and `Comanche055/PHASE_TABLE_MAINTENANCE.agc` (417 lines) — documenting the priority-based preemptive multitasking model, NOVAC/FINDVAC job creation, CHANJOB context switching, T3RUPT-driven task dispatch, and the phase/group restart protection system that produced the famous 1202/1201 alarms.
- To **document error handling and recovery philosophy**, we will create a report section analyzing `Comanche055/ALARM_AND_ABORT.agc` (231 lines), `Luminary099/ALARM_AND_ABORT.agc` (251 lines), `Comanche055/FRESH_START_AND_RESTART.agc`, `Comanche055/RESTARTS_ROUTINE.agc` (338 lines), and `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc` — mapping the three-tier alarm hierarchy (ALARM → BAILOUT → POODOO), the FAILREG cascade, restart group processing, and the deliberate graceful degradation design.
- To **document naming conventions and inline documentation**, we will create a report section surveying comments and labels across all source files — cataloging the cultural references (BURN BABY BURN, PINBALL, FLAGORGY), Latin phrases (HONI SOIT QUI MAL Y PENSE, NOLI SE TANGERE), sardonic humor (NO ROOM IN THE INN, PLEASE CRANK THE SILLY THING AROUND), and the functional naming patterns (POODOO, WHIMPER, CURTAINS, MR.KLEAN, ENEMA).
- To **provide modern context**, we will annotate each section with brief comparisons to contemporary software patterns — priority inversion handling, watchdog timers, graceful degradation under load, phase-based checkpointing, and resource exhaustion strategies.

### 0.1.4 Inferred Documentation Needs

Based on repository analysis, the following implicit documentation needs surface from the user's requirements:

- **CM vs. LM architectural divergences** — The user mentions "LUMINARY and COLOSSUS modules" as scope. Analysis reveals material differences between the two implementations (8 core sets in LM vs. 7 in CM, `BAILOUT1`/`POODOO1` alternate entry points in LM, different `ABORT EQUALS` targets, SERVICER state handling in LM's `GOPOODOO`). The report must document these divergences as they reflect mission-specific engineering tradeoffs.
- **Interrupt architecture context** — The user's scheduling domain implicitly requires documenting the interrupt vector table (`Comanche055/INTERRUPT_LEAD_INS.agc`) and T3RUPT/T5RUPT dispatch mechanisms that drive the Waitlist timer system.
- **Restart table encoding** — The phase table maintenance system (`PHASCHNG` Type A/B/C encoding, `RESTART_TABLES.agc` job/waitlist/longcall entry formats) is central to understanding restart protection but not explicitly called out. It must be documented as the mechanism behind the 1202/1201 alarm recovery behavior.
- **Bank-switching as a memory constraint artifact** — The `INTER-BANK_COMMUNICATION.agc` module (BANKCALL, POSTJUMP, SUPERSW, superbank addressing) directly evidences how the 36KB fixed memory limitation shaped every cross-module call pattern. This deserves explicit treatment in the memory constraints section.
- **Alarm code catalog** — The `ASSEMBLY_AND_OPERATION_INFORMATION.agc` file contains a comprehensive alarm code table (OCT 00217 through 01706) that serves as the definitive reference for the error handling section. A curated subset of critical alarm codes should appear in the report.



## 0.2 Documentation Discovery and Analysis



### 0.2.1 Existing Documentation Infrastructure Assessment

Repository analysis reveals a **minimal documentation infrastructure** consisting exclusively of markdown files with no documentation generator framework (no mkdocs, Sphinx, Docusaurus, or similar tooling). The repository is a historical preservation project, not a software development project with API documentation needs.

**Documentation files discovered** (79 total `.md` files):

| File | Purpose |
|------|---------|
| `README.md` | Project overview with multi-language links, build badges, repository description |
| `CONTRIBUTING.md` | Contribution guidelines: scan fidelity, formatting rules (tab width 8, UTF-8, LF), proofing instructions |
| `LICENSE.md` | Public Domain license declaration |
| `Comanche055/README.md` | Manifest of CM source files with scanned page ranges and digitization links |
| `Luminary099/README.md` | Manifest of LM source files with scanned page ranges and digitization links |
| `.github/ISSUE_TEMPLATE/Discussion.md` | Issue template for general discussion |
| `.github/ISSUE_TEMPLATE/Humour.md` | Issue template for humor-related observations |
| `.github/ISSUE_TEMPLATE/Proof_Comanche.md` | Issue template for Comanche proofreading |
| `.github/ISSUE_TEMPLATE/Proof_Luminary.md` | Issue template for Luminary proofreading |
| `.github/PULL_REQUEST_TEMPLATE.md` | PR template |
| `Translations/*.md` | ~69 translated README and CONTRIBUTING files across 30+ languages |

**Documentation tooling detected**:
- **Markdown linter**: `markdownlint-cli2 ^0.16.0` (sole devDependency in `package.json`)
- **Linting config**: `.markdownlint.yml` — disables rules MD007, MD010, MD013, MD026, MD033, MD034, MD036, MD041, MD050, MD053 (accommodates AGC-specific formatting needs)
- **Editor config**: `.editorconfig` — enforces `charset=utf-8`, `end_of_line=lf`, tab indentation with width 8 for `.agc` files
- **No documentation generators**: No mkdocs.yml, docusaurus.config.js, sphinx conf.py, or .readthedocs.yml detected
- **No diagram tools**: No Mermaid CLI, PlantUML, or similar detected in dependencies
- **No API documentation tools**: No JSDoc, Sphinx, typedoc, or similar in use

**Current documentation coverage**: The repository has zero analytical documentation. All existing `.md` files serve repository management purposes (README, CONTRIBUTING, issue templates, translations). No technical analysis, architectural documentation, or engineering tradeoff reports exist.

### 0.2.2 Repository Code Analysis for Documentation

**Search patterns employed for source code analysis**:

- AGC assembly source files: `Comanche055/*.agc` (86 files, 65,348 total lines) and `Luminary099/*.agc` (92 files, 64,838 total lines)
- Scheduling subsystem: `EXECUTIVE.agc`, `WAITLIST.agc`, `PHASE_TABLE_MAINTENANCE.agc`, `RESTART_TABLES.agc`, `RESTARTS_ROUTINE.agc`
- Error handling: `ALARM_AND_ABORT.agc`, `FRESH_START_AND_RESTART.agc`
- Memory architecture: `ERASABLE_ASSIGNMENTS.agc`, `INTER-BANK_COMMUNICATION.agc`
- System initialization: `FRESH_START_AND_RESTART.agc`, `INTERRUPT_LEAD_INS.agc`
- Program organization: `MAIN.agc` (include manifests for both modules)
- Alarm catalog: `ASSEMBLY_AND_OPERATION_INFORMATION.agc` (comprehensive alarm code table)
- Mission programs: `BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc`, `THE_LUNAR_LANDING.agc`
- Cultural references: Broad grep across all `.agc` files for comments containing humor, Latin phrases, and naming patterns

**Key directories examined**:

| Directory | File Count | Total Lines | Content |
|-----------|-----------|-------------|---------|
| `Comanche055/` | 86 `.agc` + 1 `.md` | 65,348 | Command Module (Colossus 2A) — assembled Apr 1, 1969 |
| `Luminary099/` | 92 `.agc` + 1 `.md` | 64,838 | Lunar Module (Luminary 1A) — assembled Jul 14, 1969 |
| `Translations/` | ~69 `.md` files | N/A | Multi-language README/CONTRIBUTING translations |
| `.github/` | 5 templates | N/A | Issue and PR templates |

**Related documentation context**: The existing `Comanche055/README.md` and `Luminary099/README.md` serve as file manifests listing each `.agc` source file with its corresponding scanned page ranges and links to the original MIT printout digitizations. These provide the authoritative source-to-scan mapping but contain no architectural analysis.

### 0.2.3 Web Search Research Conducted

No external web searches were required for this documentation task. The analysis is entirely source-code-driven, drawing conclusions from direct reading of the AGC assembly files in the repository. The user explicitly scoped the report to repository-internal evidence with citations referencing specific filenames and labels. Existing tech spec sections (1.1 Executive Summary, 4.2 System Initialization and Restart Workflows, 4.3 Executive Job Scheduling Workflows) provide sufficient contextual background on the AGC architecture.

The repository's `CONTRIBUTING.md` and `README.md` provided documentation style context: contributions must match original scans exactly, tab indentation width 8, and trailing whitespace trimmed. The output report will follow standard markdown conventions consistent with the repository's `.markdownlint.yml` configuration.



## 0.3 Documentation Scope Analysis



### 0.3.1 Code-to-Documentation Mapping

The output report must cover five decision domains. Each domain maps to specific source modules that provide the evidence base:

**Domain 1: Memory and Resource Constraints**

| Module | Path | Evidence Type | Documentation Needed |
|--------|------|---------------|---------------------|
| Erasable Assignments (CM) | `Comanche055/ERASABLE_ASSIGNMENTS.agc` | 3,785 lines of erasable register allocation with M/SIZE/N notation | Register sharing via EQUALS, bank-sensitive vs bank-insensitive classification, memory pressure evidence |
| Erasable Assignments (LM) | `Luminary099/ERASABLE_ASSIGNMENTS.agc` | 2,635 lines (smaller LM allocation footprint) | CM vs LM allocation divergence, reduced erasable budget |
| Inter-Bank Communication | `Comanche055/INTER-BANK_COMMUNICATION.agc` | 184 lines: BANKCALL, POSTJUMP, SUPERSW, superbank addressing | Bank-switching overhead as memory constraint artifact, 36KB fixed memory organization |
| Interpreter | `Comanche055/INTERPRETER.agc` / `Luminary099/INTERPRETER.agc` | ~78KB each; interpretive pseudo-instruction set | Compact interpretive language as memory compression strategy |
| Tags for Relative SETLOC | `Comanche055/TAGS_FOR_RELATIVE_SETLOC.agc` / `Luminary099/TAGS_FOR_RELATIVE_SETLOC.agc` | Bank layout and SETLOC directives | Fixed memory bank organization evidence |
| Fixed-Fixed Constant Pool | `Comanche055/FIXED_FIXED_CONSTANT_POOL.agc` / `Luminary099/FIXED_FIXED_CONSTANT_POOL.agc` | Shared constants in fixed-fixed memory | Memory optimization through constant sharing |

**Domain 2: Real-Time Scheduling and Priority Architecture**

| Module | Path | Evidence Type | Documentation Needed |
|--------|------|---------------|---------------------|
| Executive (CM) | `Comanche055/EXECUTIVE.agc` | 497 lines: NOVAC, FINDVAC, CHANJOB, JOBSLEEP/JOBWAKE | Priority-based preemptive scheduling, 7 core sets, 5 VAC areas |
| Executive (LM) | `Luminary099/EXECUTIVE.agc` | 503 lines: Same structure + 8th core set entry | LM-specific 8 core sets, WAITPOOH/LONGPOOH additions |
| Waitlist (CM) | `Comanche055/WAITLIST.agc` | 557 lines: LST1/LST2 arrays, T3RUPT dispatch | 9 concurrent tasks, 1-16250 cs range, LONGCALL extension |
| Waitlist (LM) | `Luminary099/WAITLIST.agc` | 564 lines: Adds FILLED routine, WAITPOOH | LM waitlist overflow handling differences |
| Phase Table Maintenance | `Comanche055/PHASE_TABLE_MAINTENANCE.agc` | 417 lines: PHASCHNG Type A/B/C, 2PHSCHNG | Phase encoding system, TBASE/LONGBASE timing |
| Restart Tables | `Comanche055/RESTART_TABLES.agc` | Job/waitlist/longcall restart entry encoding | Restart group table format and recovery dispatch |
| Interrupt Lead-Ins | `Comanche055/INTERRUPT_LEAD_INS.agc` | 122 lines: 10 interrupt vectors at SETLOC 4000 | T3RUPT/T5RUPT dispatch driving waitlist timing |

**Domain 3: Error Handling and Recovery Philosophy**

| Module | Path | Evidence Type | Documentation Needed |
|--------|------|---------------|---------------------|
| Alarm and Abort (CM) | `Comanche055/ALARM_AND_ABORT.agc` | 231 lines: ALARM, BAILOUT, POODOO, CURTAINS, CCSHOLE | Three-tier alarm hierarchy, FAILREG cascade |
| Alarm and Abort (LM) | `Luminary099/ALARM_AND_ABORT.agc` | 251 lines: Adds BAILOUT1/POODOO1, GOPOODOO | LM-specific SERVICER state handling in abort |
| Fresh Start and Restart (CM) | `Comanche055/FRESH_START_AND_RESTART.agc` | Four initialization pathways: SLAP1, DOFSTART, GOPROG, ENEMA | Restart vs fresh start decision tree, state preservation |
| Restarts Routine | `Comanche055/RESTARTS_ROUTINE.agc` | 338 lines: ITSAVAR, ITSATBL, phase processing | Variable vs table restart dispatch, FINDTIME delta-T recovery |
| Assembly and Operation Info | `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc` | Comprehensive alarm code catalog (OCT 00217–01706) | Alarm code reference table with trigger conditions |

**Domain 4: Naming Conventions and Inline Documentation**

| Source Pattern | Files | Evidence Type | Documentation Needed |
|----------------|-------|---------------|---------------------|
| Routine names with cultural references | `Luminary099/BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc`, `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` | BURN BABY BURN (DJ Magnificent Montague), PINBALL (arcade game) | How naming aided memorability despite assembly constraints |
| Abort/alarm naming hierarchy | `*/ALARM_AND_ABORT.agc` | POODOO, WHIMPER, CURTAINS, BAILOUT | Severity conveyed through dramatic naming |
| Latin/literary phrases | `Luminary099/BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc` | HONI SOIT QUI MAL Y PENSE, NOLI SE TANGERE | Comment culture as documentation layer |
| Sardonic humor in comments | All `.agc` files | "NO ROOM IN THE INN", "OFF TO SEE THE WIZARD", FLAGORGY / "DIONYSIAN FLAG WAVING" | Humor as mnemonic and morale tool |
| Ironic equivalences | `*/ALARM_AND_ABORT.agc` | DOALARM EQUALS ENDOFJOB | Design philosophy conveyed through symbol definitions |

**Domain 5: Modern Context**

This domain draws cross-cutting evidence from all files above, annotating AGC patterns with contemporary parallels. No additional source modules are required — this section synthesizes observations from the other four domains.

### 0.3.2 Documentation Gap Analysis

Given the requirements and repository analysis, documentation gaps include:

- **Zero existing analytical documentation** — The repository contains no architectural analysis, engineering tradeoff documentation, or design decision records. All existing markdown files serve repository management and contribution workflow purposes exclusively.
- **No alarm code documentation outside source comments** — The alarm code catalog in `ASSEMBLY_AND_OPERATION_INFORMATION.agc` exists only as inline assembly comments. No standalone reference document maps alarm codes to their engineering rationale.
- **No cross-module architectural narrative** — While individual `.agc` files have inline comments explaining local behavior, no document connects the Executive → Waitlist → Phase Table → Restart chain as a coherent architectural story.
- **No CM vs. LM comparison** — The repository provides parallel Comanche055 and Luminary099 directories with no documentation comparing their architectural differences or explaining mission-driven divergences.
- **No modern context annotations** — No existing documentation relates AGC design patterns to contemporary software engineering concepts.

The proposed report will fill all five gaps in a single concise markdown file, scoped strictly to the evidence available in the AGC source code.



## 0.4 Documentation Implementation Design



### 0.4.1 Documentation Structure Planning

The output is a single markdown file placed at the repository root. The report follows a flat section structure matching the five decision domains, bracketed by a preamble and appendix:

```
docs/
└── AGC_ENGINEERING_ANALYSIS.md
    ├── Title and Preamble
    │   ├── Report purpose statement
    │   ├── Scope declaration (Comanche055/Luminary099 only)
    │   └── Repository structure overview
    ├── 1. Memory and Resource Constraints
    │   ├── Core-rope / fixed-rope memory architecture
    │   ├── Bank-switching and cross-bank call overhead
    │   ├── Erasable register allocation and sharing
    │   ├── Interpretive language as memory compression
    │   └── CM vs LM memory budget differences
    ├── 2. Real-Time Scheduling and Priority Architecture
    │   ├── Executive: priority-based preemptive multitasking
    │   ├── Waitlist: timer-driven task scheduling
    │   ├── Phase table / restart group protection
    │   ├── The 1202/1201 alarm mechanism
    │   └── CM vs LM core set allocation
    ├── 3. Error Handling and Recovery Philosophy
    │   ├── Three-tier alarm hierarchy (ALARM → BAILOUT → POODOO)
    │   ├── FAILREG cascade and alarm code catalog
    │   ├── Restart architecture (GOPROG, ENEMA, DOFSTART, SLAP1)
    │   ├── Graceful degradation: availability over correctness
    │   └── CCSHOLE and defensive programming patterns
    ├── 4. Naming Conventions and Inline Documentation
    │   ├── Cultural and literary references catalog
    │   ├── Abort/alarm naming as severity indicator
    │   ├── Humor as mnemonic and documentation layer
    │   ├── Erasable notation system (M/SIZE/N convention)
    │   └── Readability tradeoffs in 6-character label space
    ├── 5. Modern Context
    │   ├── Priority inversion parallels
    │   ├── Watchdog timer / heartbeat patterns
    │   ├── Graceful degradation under load
    │   ├── Phase-based checkpointing vs modern journaling
    │   └── Resource exhaustion strategies
    └── Appendix: Key Alarm Codes Reference
        └── Curated alarm code table from ASSEMBLY_AND_OPERATION_INFORMATION.agc
```

### 0.4.2 Content Generation Strategy

**Information Extraction Approach**:

- Extract scheduling architecture from `Comanche055/EXECUTIVE.agc` and `Luminary099/EXECUTIVE.agc` by analyzing NOVAC/FINDVAC/CHANJOB routines and core set allocation constants (`DEC 6` vs `DEC 7`)
- Extract error handling hierarchy from `*/ALARM_AND_ABORT.agc` by tracing ALARM → BAILOUT → POODOO call chains and FAILREG storage patterns
- Extract memory constraint evidence from `*/ERASABLE_ASSIGNMENTS.agc` by analyzing the M/SIZE/N notation system, EQUALS chaining, and bank allocation directives
- Extract restart architecture from `Comanche055/FRESH_START_AND_RESTART.agc`, `RESTARTS_ROUTINE.agc`, `PHASE_TABLE_MAINTENANCE.agc`, and `RESTART_TABLES.agc` by mapping the four initialization pathways and phase group encoding
- Extract naming conventions by surveying labels and comments across all `.agc` files — cataloging cultural references, Latin phrases, humor, and ironic equivalences
- Extract alarm code catalog from `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc` lines 917-977 for the appendix reference table
- Generate modern context annotations by synthesizing observations from the four analytical domains and comparing to contemporary patterns

**Documentation Standards**:

- Markdown formatting with `#` through `###` headers for progressive section depth
- Source citations as inline references in the format: `Source: Comanche055/EXECUTIVE.agc, label BAILOUT`
- Tables for structured data (alarm codes, register allocations, CM vs LM comparisons)
- Concise paragraphs — no verbose prose per user directive
- Mermaid diagrams for architectural relationships (alarm hierarchy flow, restart decision tree, scheduling model)
- Consistent terminology: "core set" (not "register bank"), "waitlist task" (not "timer event"), "phase group" (not "checkpoint"), matching the AGC source vocabulary

### 0.4.3 Diagram and Visual Strategy

The report will include the following Mermaid diagrams to visually communicate complex architectural relationships:

- **Alarm Hierarchy Flowchart**: ALARM (non-abortive) → BAILOUT (abortive, stores erasables) → POODOO (full abort with flag cleanup), showing the three-tier escalation path and their respective behaviors
- **Restart Decision Tree**: GOPROG → ERESTORE check → ENEMA/DOFSTART branching, MR.KLEAN cascade, phase table processing flow
- **Executive Scheduling Model**: Job creation (NOVAC/FINDVAC) → priority queue → CHANJOB context switch → DUMMYJOB idle loop, showing core set and VAC area allocation
- **Waitlist Timer Architecture**: T3RUPT → LST1/LST2 dispatch → TASKOVER chaining → LONGCALL cycling for extended durations
- **Phase Table Protection**: PHASCHNG (Type A/B/C encoding) → phase tables → RESTARTS_ROUTINE dispatch → job/waitlist/longcall recovery

All diagrams use Mermaid `graph` or `flowchart` syntax compatible with GitHub markdown rendering and standard Mermaid renderers. No external image assets are required.



## 0.5 Documentation File Transformation Mapping



### 0.5.1 File-by-File Documentation Plan

| Target Documentation File | Transformation | Source Code/Docs | Content/Changes |
|---------------------------|----------------|------------------|-----------------|
| `docs/AGC_ENGINEERING_ANALYSIS.md` | CREATE | `Comanche055/*.agc`, `Luminary099/*.agc` | Complete engineering tradeoff analysis report covering all five decision domains with source citations, Mermaid diagrams, and alarm code appendix |

**Detailed source file mapping for the single output file**:

| Report Section | Primary Source Files | Secondary Source Files | Content Extracted |
|----------------|---------------------|----------------------|-------------------|
| Preamble | `Comanche055/MAIN.agc`, `Luminary099/MAIN.agc` | `README.md` | Repository structure overview, module organization, assembly dates |
| Memory Constraints | `Comanche055/ERASABLE_ASSIGNMENTS.agc`, `Comanche055/INTER-BANK_COMMUNICATION.agc` | `Luminary099/ERASABLE_ASSIGNMENTS.agc`, `*/INTERPRETER.agc`, `*/TAGS_FOR_RELATIVE_SETLOC.agc`, `*/FIXED_FIXED_CONSTANT_POOL.agc` | Bank-switching architecture, erasable allocation notation, register sharing patterns, interpretive language rationale |
| Scheduling Architecture | `Comanche055/EXECUTIVE.agc`, `Comanche055/WAITLIST.agc`, `Comanche055/PHASE_TABLE_MAINTENANCE.agc` | `Luminary099/EXECUTIVE.agc`, `Luminary099/WAITLIST.agc`, `Comanche055/INTERRUPT_LEAD_INS.agc`, `Comanche055/RESTART_TABLES.agc` | Priority scheduling, core sets, VAC areas, timer dispatch, phase protection, 1202/1201 alarms |
| Error Handling | `Comanche055/ALARM_AND_ABORT.agc`, `Comanche055/FRESH_START_AND_RESTART.agc`, `Comanche055/RESTARTS_ROUTINE.agc` | `Luminary099/ALARM_AND_ABORT.agc`, `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc` | Three-tier alarm hierarchy, FAILREG cascade, restart pathways, alarm code catalog, CCSHOLE defensive pattern |
| Naming Conventions | `Luminary099/BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc`, `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` | `*/ALARM_AND_ABORT.agc`, `Luminary099/THE_LUNAR_LANDING.agc`, all `.agc` files (comment grep) | Cultural references, Latin phrases, humor catalog, M/SIZE/N notation, ironic equivalences |
| Modern Context | All source files listed above | N/A | Cross-cutting synthesis comparing AGC patterns to contemporary software engineering |
| Alarm Code Appendix | `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc` | `Comanche055/ALARM_AND_ABORT.agc` | Curated alarm code table with codes, descriptions, source modules, and severity markers |

### 0.5.2 New Documentation File Detail

```
File: docs/AGC_ENGINEERING_ANALYSIS.md
Type: Technical Analysis Report
Source Code: Comanche055/*.agc, Luminary099/*.agc (130,186 total lines across 178 .agc files)
Sections:
    - Preamble: Scope, repository structure, module overview
    - Section 1: Memory and Resource Constraints
      - Core-rope limitations (from ERASABLE_ASSIGNMENTS.agc notation system)
      - Bank-switching overhead (from INTER-BANK_COMMUNICATION.agc: BANKCALL, POSTJUMP, SUPERSW)
      - Register sharing and EQUALS chaining (from ERASABLE_ASSIGNMENTS.agc)
      - Interpretive language as memory compression (from INTERPRETER.agc)
      - CM vs LM erasable budget comparison
    - Section 2: Real-Time Scheduling and Priority Architecture
      - Executive model (from EXECUTIVE.agc: NOVAC, FINDVAC, CHANJOB, core set counts)
      - Waitlist model (from WAITLIST.agc: LST1/LST2, T3RUPT, LONGCALL, TWIDDLE)
      - Phase table restart protection (from PHASE_TABLE_MAINTENANCE.agc: PHASCHNG Type A/B/C)
      - 1202/1201 alarm mechanism (from EXECUTIVE.agc: BAILOUT OCT 1201/1202)
      - CM 7 core sets vs LM 8 core sets comparison
    - Section 3: Error Handling and Recovery Philosophy
      - Three-tier alarm hierarchy (from ALARM_AND_ABORT.agc: ALARM→BAILOUT→POODOO)
      - FAILREG cascade (from ALARM_AND_ABORT.agc: FAILREG, FAILREG+1, FAILREG+2)
      - Four restart pathways (from FRESH_START_AND_RESTART.agc: SLAP1, DOFSTART, GOPROG, ENEMA)
      - Phase group restart dispatch (from RESTARTS_ROUTINE.agc: ITSAVAR, ITSATBL)
      - CCSHOLE defensive pattern (from ALARM_AND_ABORT.agc: OCT 1103)
      - Availability vs correctness tradeoff analysis
    - Section 4: Naming Conventions and Inline Documentation
      - Cultural reference catalog (BURN BABY BURN, PINBALL, FLAGORGY)
      - Latin phrase inventory (HONI SOIT QUI MAL Y PENSE, NOLI SE TANGERE)
      - Abort naming hierarchy (POODOO→WHIMPER→CURTAINS→BAILOUT)
      - Sardonic humor catalog (NO ROOM IN THE INN, DOALARM EQUALS ENDOFJOB)
      - M/SIZE/N erasable notation as self-documenting convention
    - Section 5: Modern Context
      - Priority inversion parallels (Executive BAILOUT as load shedding)
      - Watchdog/heartbeat patterns (DUMMYJOB activity light toggle)
      - Graceful degradation (restart protection, POODOO vs ALARM severity tiers)
      - Phase-based checkpointing vs journaling/WAL
      - Resource pool exhaustion handling (BAILOUT on no VAC/core sets)
    - Appendix: Key Alarm Codes Reference Table
Diagrams:
    - Alarm hierarchy flowchart (ALARM → BAILOUT → POODOO)
    - Restart decision tree (GOPROG → ERESTORE → ENEMA/DOFSTART)
    - Executive scheduling model (job creation → priority dispatch → idle)
    - Waitlist timer architecture (T3RUPT → task dispatch → LONGCALL)
Key Citations: 30+ specific .agc files across Comanche055/ and Luminary099/
```

### 0.5.3 Documentation Configuration Updates

No documentation configuration files require creation or modification. The repository has no documentation generator (no mkdocs.yml, docusaurus.config.js, sphinx conf.py, or .readthedocs.yml). The sole output file `docs/AGC_ENGINEERING_ANALYSIS.md` is a standalone markdown document requiring no build pipeline, navigation configuration, or deployment setup.

The existing `package.json` lint script covers `*.md` at the repository root and within `Comanche055/` and `Luminary099/` directories. The new file in `docs/` is outside this glob. If markdown linting is desired for the output file, the lint script could be extended, but this is a source code modification and falls **outside the read-only analysis scope**.

### 0.5.4 Cross-Documentation Dependencies

- **No shared content/includes** — The report is a self-contained markdown file with no templating or include system.
- **No navigation links required** — No documentation framework generates navigation. The report is standalone.
- **No table of contents updates** — No existing TOC references the new file.
- **Internal cross-references** — The report sections will use markdown anchor links for cross-referencing between the five decision domains and the alarm code appendix (e.g., `[alarm code table](#appendix-key-alarm-codes-reference)`).
- **Repository README relationship** — The `README.md` does not need updating (read-only analysis constraint). The new report exists as an independent analytical artifact.



## 0.6 Dependency Inventory



### 0.6.1 Documentation Dependencies

The project has minimal tooling. The output is a standalone markdown file with embedded Mermaid diagram syntax. No documentation build pipeline exists or is required.

| Registry | Package Name | Version | Purpose |
|----------|--------------|---------|---------|
| npm | markdownlint-cli2 | ^0.16.0 | Markdown linting for repository `.md` files (existing devDependency in `package.json`) |

**No additional documentation dependencies are required for this task.** The report uses:

- **Standard GitHub-Flavored Markdown (GFM)** — Natively rendered by GitHub without any build step. No additional markdown processor needed.
- **Mermaid diagram syntax** — GitHub natively renders Mermaid code blocks since November 2022. No Mermaid CLI or rendering tool is required for the diagrams to display correctly on GitHub.
- **No documentation generator** — The report is a single `.md` file, not part of a documentation site. No mkdocs, Sphinx, Docusaurus, or similar framework is involved.
- **No image generation** — All visual elements use inline Mermaid syntax. No external diagram tools (PlantUML, draw.io) or image assets are needed.

### 0.6.2 Documentation Reference Updates

No documentation link updates are required. The new file `docs/AGC_ENGINEERING_ANALYSIS.md` is a standalone addition that does not replace, supersede, or conflict with any existing documentation file. No existing `.md` files reference or link to a prior version of this report.

**Link transformation rules**: Not applicable — no existing links require updating. The report is an entirely new artifact.



## 0.7 Coverage and Quality Targets



### 0.7.1 Documentation Coverage Metrics

**Current coverage analysis** (pre-report baseline):

| Coverage Area | Documented | Total | Percentage | Target |
|---------------|-----------|-------|------------|--------|
| Memory constraint mechanisms documented | 0 | 4 key mechanisms (core-rope, bank-switching, erasable sharing, interpretive compression) | 0% | 100% |
| Scheduling subsystem routines documented | 0 | 12 key routines (NOVAC, FINDVAC, CHANJOB, JOBSLEEP, JOBWAKE, ENDOFJOB, TWIDDLE, FIXDELAY, VARDELAY, LONGCALL, T3RUPT, TASKOVER) | 0% | 100% |
| Error handling routines documented | 0 | 8 key routines (ALARM, BAILOUT, POODOO, CURTAINS, CCSHOLE, GOPROG, ENEMA, MR.KLEAN) | 0% | 100% |
| Naming convention patterns cataloged | 0 | 5 categories (cultural references, Latin phrases, abort hierarchy, sardonic humor, notation system) | 0% | 100% |
| Modern context annotations | 0 | 5 comparison domains (priority inversion, watchdog timers, graceful degradation, checkpointing, resource exhaustion) | 0% | 100% |
| Critical alarm codes documented | 0 | 15+ codes (OCT 00217 through 01706 subset) | 0% | 100% |
| CM vs LM divergences documented | 0 | 4 key divergences (core sets, abort paths, entry points, module differences) | 0% | 100% |

**Target coverage**: 100% of all items identified in the five user-specified decision domains, evidenced by specific source file citations.

**Coverage gaps to address**:
- All items above represent gaps — no analytical documentation currently exists
- Every gap maps directly to a specific report section
- The report must cite a minimum of 25 distinct `.agc` source files across both Comanche055 and Luminary099

### 0.7.2 Documentation Quality Criteria

**Completeness requirements**:
- Every claim in the report must cite a specific filename and label/routine from the repository
- All five decision domains must have dedicated sections with substantive analysis
- The alarm code appendix must include all critical alarm codes (1103, 1107, 1110, 1201, 1202, 1203, 1204) with descriptions and triggering modules
- CM vs LM differences must be explicitly identified where they exist (scheduling, error handling, module composition)
- Each Mermaid diagram must accurately reflect the code flow it represents

**Accuracy validation**:
- All cited label names, routine names, and alarm codes must match the exact spelling in the source files (e.g., `BAILOUT` not `BAIL_OUT`, `POODOO` not `POO_DOO`, `CHANJOB` not `CHANGEJOB`)
- Register counts must match source constants: 7 core sets CM (`DEC 6` = 6 + current = 7), 8 core sets LM (`DEC 7` = 7 + current = 8), 5 VAC areas both
- Alarm code values must match `ASSEMBLY_AND_OPERATION_INFORMATION.agc` exactly (e.g., OCT 01201, not decimal 641)
- File paths must use the exact directory and filename spelling from the repository (e.g., `BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc` with double dashes)

**Clarity standards**:
- Technical accuracy with accessible language — assume reader has basic software engineering knowledge but no AGC assembly expertise
- Each section begins with a one-sentence summary of the key engineering tradeoff before diving into evidence
- Comments explain WHY, not WHAT — per user directive
- No verbose prose — concise paragraphs interleaved with evidence citations

**Maintainability**:
- Source citations in every section enable traceability back to specific `.agc` files
- Standalone markdown format requires no build system or rendering pipeline to read
- Consistent heading structure enables navigation via markdown TOC

### 0.7.3 Example and Diagram Requirements

- **Minimum source citations per section**: 3 distinct `.agc` files with specific labels/routines
- **Diagram types required**: 5 Mermaid diagrams (alarm hierarchy, restart decision tree, scheduling model, waitlist architecture, phase table protection)
- **Diagram validation**: Each diagram must trace to specific routine names found in the source code
- **Alarm code table**: Minimum 15 entries covering all severity levels (informational, starred, double-starred)
- **No screenshots or images**: All visual content uses Mermaid text-based diagrams for portability and version control friendliness



## 0.8 Scope Boundaries



### 0.8.1 Exhaustively In Scope

**New documentation files**:
- `docs/AGC_ENGINEERING_ANALYSIS.md` — The sole deliverable: a concise markdown report covering all five decision domains

**Source files analyzed for documentation content** (read-only, no modifications):

| Category | File Pattern | Count | Purpose |
|----------|-------------|-------|---------|
| Scheduling subsystem | `Comanche055/EXECUTIVE.agc`, `Luminary099/EXECUTIVE.agc`, `Comanche055/WAITLIST.agc`, `Luminary099/WAITLIST.agc` | 4 | Priority scheduling model, core sets, waitlist timer architecture |
| Error handling | `Comanche055/ALARM_AND_ABORT.agc`, `Luminary099/ALARM_AND_ABORT.agc` | 2 | Three-tier alarm hierarchy, FAILREG cascade |
| Restart system | `Comanche055/FRESH_START_AND_RESTART.agc`, `Comanche055/RESTARTS_ROUTINE.agc`, `Comanche055/PHASE_TABLE_MAINTENANCE.agc`, `Comanche055/RESTART_TABLES.agc` | 4 | Restart pathways, phase encoding, restart dispatch |
| Memory architecture | `Comanche055/ERASABLE_ASSIGNMENTS.agc`, `Luminary099/ERASABLE_ASSIGNMENTS.agc`, `Comanche055/INTER-BANK_COMMUNICATION.agc` | 3 | Bank-switching, erasable allocation, register sharing |
| Interrupt system | `Comanche055/INTERRUPT_LEAD_INS.agc` | 1 | Interrupt vector table, T3RUPT/T5RUPT dispatch |
| Program organization | `Comanche055/MAIN.agc`, `Luminary099/MAIN.agc` | 2 | Module include manifests |
| Mission programs | `Luminary099/BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc`, `Luminary099/THE_LUNAR_LANDING.agc` | 2 | Naming conventions, cultural references, comment style |
| Reference data | `Luminary099/ASSEMBLY_AND_OPERATION_INFORMATION.agc` | 1 | Alarm code catalog |
| Display system | `Comanche055/PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` | 1 | Naming convention evidence (PINBALL) |
| Naming/comment survey | `Comanche055/*.agc`, `Luminary099/*.agc` (grep survey) | 178 | Broad survey for humor, Latin phrases, cultural references |

**Existing documentation files referenced for context** (read-only):
- `README.md` — Repository purpose and structure
- `CONTRIBUTING.md` — Formatting standards and contribution guidelines
- `Comanche055/README.md` — CM file manifest with page ranges
- `Luminary099/README.md` — LM file manifest with page ranges

### 0.8.2 Explicitly Out of Scope

- **Source code modifications** — No `.agc` files will be modified. The user directive specifies "Read-only analysis — no code modifications."
- **Ground systems** — User explicitly excludes ground systems from scope
- **Hardware schematics** — User explicitly excludes hardware schematics from scope
- **Non-AGC source** — Files outside `Comanche055/` and `Luminary099/` are not analysis targets (though `README.md`, `CONTRIBUTING.md` provide context)
- **Translation files** — `Translations/*.md` files are not relevant to AGC source analysis
- **GitHub templates** — `.github/ISSUE_TEMPLATE/*.md` and `.github/PULL_REQUEST_TEMPLATE.md` are repository management artifacts, not analysis subjects
- **Documentation framework setup** — No mkdocs, Sphinx, Docusaurus, or other documentation generator will be installed or configured
- **Test file modifications** — No test files exist in this repository; not applicable
- **Feature additions or code refactoring** — Explicitly prohibited by the read-only constraint
- **Deployment configuration** — No deployment pipeline exists or is needed for a single markdown file
- **Line-by-line code narration** — User directive: "Comments explain WHY decisions were made, not WHAT the code does line-by-line"
- **Package.json lint script extension** — Modifying `package.json` to include `docs/` in the lint glob would be a source modification and is out of scope
- **GAP-generated tables** — The `GAP-GENERATED TABLES` sections at the end of each MAIN.agc manifest represent assembler output artifacts, not engineering decisions, and are excluded from analysis



## 0.9 Execution Parameters



### 0.9.1 Documentation-Specific Instructions

- **Documentation build command**: Not applicable — the output is a standalone markdown file requiring no build step. GitHub renders `.md` files natively.
- **Documentation preview command**: Any markdown viewer or GitHub preview. For local preview with the repository's existing tooling:
  - `npx markdownlint-cli2 docs/AGC_ENGINEERING_ANALYSIS.md --config .markdownlint.yml` (lint validation only)
- **Diagram generation command**: Not applicable — Mermaid diagrams are embedded inline and rendered natively by GitHub. No separate generation step is required.
- **Documentation deployment command**: Not applicable — no documentation hosting infrastructure exists. The file is served directly from the repository.
- **Default format**: Markdown (GFM) with Mermaid diagram blocks
- **Citation requirement**: Every section must reference specific source files, labels, and routines from the `Comanche055/` and `Luminary099/` directories. Citations use the format: `Source: <directory>/<filename>.agc, label <LABEL_NAME>` or inline parenthetical references.
- **Style guide**: Follow the repository's existing markdown conventions:
  - UTF-8 encoding (per `.editorconfig`)
  - LF line endings (per `.editorconfig`)
  - Consistent heading hierarchy (`#` through `###`)
  - No trailing whitespace (per `CONTRIBUTING.md`)
- **Documentation validation**: `npx markdownlint-cli2 docs/AGC_ENGINEERING_ANALYSIS.md --config .markdownlint.yml` — validates against the repository's `.markdownlint.yml` rules (default: true, with MD007, MD010, MD013, MD026, MD033, MD034, MD036, MD041, MD050, MD053 disabled)
- **Octal convention**: Alarm codes and memory addresses must use octal notation (OCT prefix) matching the AGC source convention, not decimal or hexadecimal



## 0.10 Rules for Documentation



The following rules are derived directly from user-specified constraints and directives:

- **Read-only analysis — no code modifications**: The repository is a historical preservation artifact. No `.agc` file, `package.json`, `README.md`, or any other existing file may be modified. The only permitted file system operation is creating the new report file.
- **Scope limited to AGC source (LUMINARY, COLOSSUS modules)**: Analysis must draw evidence exclusively from `Comanche055/*.agc` and `Luminary099/*.agc` files. Ground systems, hardware schematics, and non-AGC artifacts must not be analyzed or cited as evidence.
- **Report citations MUST reference specific filenames and labels from the repository**: Every engineering observation must be traceable to a named source file and, where applicable, a specific label, routine, or comment. Generic claims without citations are prohibited.
- **Comments explain WHY decisions were made, not WHAT the code does line-by-line**: The report is an architectural analysis, not a code walkthrough. Each section should explain the engineering rationale behind design choices, using code structure as evidence rather than narrating instruction sequences.
- **No verbose prose — concise sections with evidence-backed observations**: Brevity is mandatory. Each paragraph should lead with an observation, follow with evidence, and close with engineering rationale. Filler prose, speculative commentary, and extended background paragraphs must be avoided.
- **Output: single markdown file**: The entire report must be delivered as one self-contained markdown file at `docs/AGC_ENGINEERING_ANALYSIS.md`. No supplementary files, no multi-file documentation site, no external assets.
- **Alarm codes in octal**: All alarm code references must use the OCT notation matching the AGC source convention (e.g., OCT 01202, not decimal 642).
- **Exact label spelling**: All cited labels, routine names, and register names must match the exact spelling and capitalization found in the source files (e.g., `POODOO` not `POOPOO`, `CHANJOB` not `CHANGE_JOB`, `DUMMYJOB` not `IDLE_JOB`).
- **CM vs LM differences must be explicit**: Where Comanche055 and Luminary099 implementations diverge, both variants must be documented with their respective file paths. Do not generalize across modules when differences exist.
- **Mermaid diagrams for architectural relationships**: Complex multi-component relationships (alarm hierarchy, restart decision tree, scheduling model) must include Mermaid diagrams for visual clarity, embedded directly in the markdown.



## 0.11 References



### 0.11.1 Codebase Files and Folders Searched

The following files and folders were systematically searched, read, and analyzed to derive the conclusions in this Agent Action Plan:

**Repository Root Files**:

| File | Purpose | Key Findings |
|------|---------|-------------|
| `README.md` | Project overview | Repository goal: faithful Apollo 11 AGC source reproduction; digitized from MIT Museum printouts |
| `CONTRIBUTING.md` | Contribution guidelines | Tab width 8, UTF-8, LF line endings, comments must match scans exactly |
| `LICENSE.md` | License | Public Domain |
| `package.json` | npm config | Sole dependency: markdownlint-cli2 ^0.16.0 |
| `.editorconfig` | Editor config | charset=utf-8, end_of_line=lf, tab indentation width 8 for .agc files |
| `.markdownlint.yml` | Lint config | 11 rules disabled to accommodate AGC-specific formatting |

**Comanche055 (Command Module / Colossus 2A) Files Read**:

| File | Lines | Key Evidence Extracted |
|------|-------|----------------------|
| `MAIN.agc` | 100 | Include manifest: 86 .agc files organized by subsystem section |
| `EXECUTIVE.agc` | 497 | NOVAC/FINDVAC/CHANJOB/JOBSLEEP/JOBWAKE, DEC 6 (7 core sets), 5 VAC areas, BAILOUT OCT 1201/1202, DUMMYJOB idle loop |
| `WAITLIST.agc` | 557 | LST1/LST2 arrays (9 tasks), T3RUPT dispatch, TWIDDLE, FIXDELAY/VARDELAY, LONGCALL, WTABORT OCT 1203 |
| `ALARM_AND_ABORT.agc` | 231 | ALARM/BAILOUT/POODOO/CURTAINS/CCSHOLE/VARALARM, FAILREG cascade, VAC5STOR erasable dump, DOALARM EQUALS ENDOFJOB |
| `FRESH_START_AND_RESTART.agc` | 300+ | SLAP1/DOFSTART/GOPROG/ENEMA pathways, MR.KLEAN/P00KLEAN/V37KLEAN cascade, SWINIT flag initialization, alarm 1107/1110 |
| `PHASE_TABLE_MAINTENANCE.agc` | 417 | PHASCHNG Type A/B/C encoding, 2PHSCHNG, TBASE/LONGBASE, NEWMODEX, phase group semantics |
| `RESTARTS_ROUTINE.agc` | 338 | ITSAVAR/ITSATBL dispatch, FINDTIME delta-T recovery, job/waitlist/longcall restart entry processing |
| `RESTART_TABLES.agc` | 80+ | Two table forms per group (even/odd), priority encoding (+FINDVAC/-NOVAC), longcall entry format |
| `INTERRUPT_LEAD_INS.agc` | 122 | 10 interrupt vectors at SETLOC 4000, T3RUPT/T5RUPT special handling |
| `INTER-BANK_COMMUNICATION.agc` | 184 | BANKCALL/SWCALL/POSTJUMP/BANKJUMP/MAKECADR/SUPDACAL, superbank SB3-SB6, channel 07 switching |
| `ERASABLE_ASSIGNMENTS.agc` | 3,785 | M/SIZE/N notation, EQUALS chaining, bank-sensitive registers, NEWJOB at LOC 67 (hardwired) |
| `PINBALL_GAME_BUTTONS_AND_LIGHTS.agc` | 50+ (header) | PINBALL naming convention, DSKY verb-noun interface |

**Luminary099 (Lunar Module / Luminary 1A) Files Read**:

| File | Lines | Key Evidence Extracted |
|------|-------|----------------------|
| `MAIN.agc` | 92 | Include manifest: 92 .agc files with LM-specific modules (THE_LUNAR_LANDING, ASCENT_GUIDANCE, etc.) |
| `EXECUTIVE.agc` | 503 | DEC 7 (8 core sets), PRIORITY +84D (8th scan entry), BAILOUT1, WAITPOOH/LONGPOOH |
| `WAITLIST.agc` | 564 | FILLED routine for overflow, WAITPOOH/LONGPOOH error paths (POODOO1 OCT 01204) |
| `ALARM_AND_ABORT.agc` | 251 | BAILOUT1/POODOO1 alternate entries, GOPOODOO with SERVICER state handling, ABORT EQUALS WHIMPER |
| `BURN_BABY_BURN--MASTER_IGNITION_ROUTINE.agc` | 80+ (header) | DJ Magnificent Montague origin, HONI SOIT QUI MAL Y PENSE, NOLI SE TANGERE, table-driven design |
| `THE_LUNAR_LANDING.agc` | 336 | FLAGORGY/"DIONYSIAN FLAG WAVING", "OFF TO SEE THE WIZARD", "PLEASE CRANK THE SILLY THING AROUND", P63LM |
| `ASSEMBLY_AND_OPERATION_INFORMATION.agc` | 977+ | Comprehensive alarm code catalog (OCT 00217 through 01706), severity markers (* and **), source module mapping |
| `ERASABLE_ASSIGNMENTS.agc` | 2,635 | LM erasable allocation (smaller than CM: 2,635 vs 3,785 lines) |

**Broad Repository Surveys**:

| Search Method | Scope | Findings |
|---------------|-------|---------|
| `grep -rn "BAILOUT\|POODOO\|ALARM"` | All `Luminary099/*.agc` | Top alarm-referencing files: PINBALL (54), T4RUPT (44), P20-P25 (40), ALARM_AND_ABORT (25) |
| `grep` for humor/cultural references | All `.agc` files | Catalog of Latin phrases, sardonic comments, cultural references, ironic equivalences across 178 files |
| `find -name "*.agc"` | Full repository | 178 .agc files total (86 Comanche055 + 92 Luminary099) |
| `wc -l` | All `.agc` files | 130,186 total lines (65,348 Comanche + 64,838 Luminary) |
| `ls -la` | Both directories | File size analysis identifying largest modules (ERASABLE_ASSIGNMENTS, PINBALL, INTERPRETER, P20-P25) |

**Folders Explored**:

| Folder | Method | Contents |
|--------|--------|----------|
| Root (`""`) | `get_source_folder_contents` | .editorconfig, .gitignore, .markdownlint.yml, CONTRIBUTING.md, LICENSE.md, README.md, package.json, 4 subdirectories |
| `Comanche055/` | `get_source_folder_contents` + `bash ls -la` | 86 .agc files + README.md |
| `Luminary099/` | `get_source_folder_contents` + `bash ls -la` | 92 .agc files + README.md |
| `.github/` | `get_source_folder_contents` | ISSUE_TEMPLATE/ (4 templates), PULL_REQUEST_TEMPLATE.md |
| `Translations/` | `get_source_folder_contents` | ~69 translated README and CONTRIBUTING files |

**Tech Spec Sections Retrieved**:

| Section | Key Information Used |
|---------|---------------------|
| 1.1 Executive Summary | Repository overview: 175 source files, ~3,494 pages, Public Domain, assembly dates, key stakeholders (Margaret Hamilton, MIT IL) |
| 4.2 System Initialization and Restart Workflows | GOPROG/SLAP1/ENEMA/DOFSTART pathways, phase table structure, ERESTORE validation, alarm codes 1107/1110 |
| 4.3 Executive Job Scheduling Workflows | NOVAC/FINDVAC/CHANJOB, core set architecture (7 CM / 8 LM), VAC area allocation, priority scheduling model |

### 0.11.2 Attachments and External Metadata

- **User attachments**: None provided (0 attachments)
- **Figma URLs**: None referenced
- **External URLs**: None required — analysis is entirely repository-internal
- **Environment files**: None found at `/tmp/environments_files/`
- **User-specified environment variables**: None
- **User-specified secrets**: None
- **User-specified implementation rules**: None



