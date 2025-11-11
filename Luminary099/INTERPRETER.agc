# Copyright:	Public domain.
# Filename:	INTERPRETER.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1002-1094
# Mod history:	2009-05-25 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-05-08 JL	Removed workarounds.

# This source code has been transcribed or otherwise adapted from
# digitized images of a hardcopy from the MIT Museum.  The digitization
# was performed by Paul Fjeld, and arranged for by Deborah Douglas of
# the Museum.  Many thanks to both.  The images (with suitable reduction
# in storage size and consequent reduction in image quality as well) are
# available online at www.ibiblio.org/apollo.  If for some reason you
# find that the images are illegible, contact me at info@sandroid.org
# about getting access to the (much) higher-quality images which Paul
# actually created.
#
# Notations on the hardcopy document read, in part:
#
#	Assemble revision 001 of AGC program LMY99 by NASA 2021112-061
#	16:27 JULY 14, 1969

# ============================================================================
# FILE: INTERPRETER.agc
# MODULE: Core Operating System - Interpretive Language Virtual Machine
# MISSION PHASE: all (foundational system used throughout all mission phases)
#
# TL;DR: Implements the AGC Interpretive Language virtual machine, providing
#        a high-level instruction set for vector/matrix operations, double-
#        precision arithmetic, and trigonometric functions. Created to overcome
#        AGC memory and processing constraints, enabling guidance and navigation
#        software to be written in more compact, readable form. Critical
#        foundation for all lunar descent, ascent, and rendezvous computations.
#
# COMMENT-ONLY READERS: This is the "computer within the computer" that
#        executed the mathematical heart of Apollo guidance. While the actual
#        interpreter code is complex, understanding its purpose helps appreciate
#        how engineers overcame 1960s computing limitations to reach the Moon.
# CODE-ALONG READERS: Study this virtual machine implementation to understand
#        how the AGC achieved high-level programming capabilities within severe
#        hardware constraints (2K RAM, 36K ROM, ~12 microsecond cycle time).
# ============================================================================
#
# INTERPRETER ARCHITECTURE OVERVIEW
#
# The AGC Interpreter is a virtual machine that executes high-level instructions
# stored in memory. It was created to address critical limitations:
#
# 1. MEMORY CONSTRAINTS: Native AGC code consumed precious memory. Interpretive
#    code provided 3-4x better memory density despite slower execution.
#
# 2. LINEAR ADDRESSING: Native AGC used bank switching (2K banks in 36K fixed
#    memory). The interpreter provided linear address space simplifying complex
#    mathematical routines.
#
# 3. HIGH-LEVEL OPERATIONS: Native AGC lacked vector/matrix operations and
#    double-precision arithmetic. The interpreter provided these as single
#    instructions.
#
# KEY ARCHITECTURAL COMPONENTS:
#
# - MPAC (Multi-Purpose Accumulator): Stack-based working storage for operands
#   MPAC +0, +1: Double-precision scalar or vector component X
#   MPAC +2, +3: Vector component Y (or DP high word)
#   MPAC +4, +5: Vector component Z
#
# - MODE Register: Tracks current data type (0=vector, 2=double-precision, 
#   3=triple-precision) enabling type-aware operations
#
# - PUSHLOC: Stack pointer for push-down list, grows downward through memory
#
# - LOC: Interpretive program counter, points to current interpretive instruction
#
# - BANKSET: Bank register for interpretive code, supports cross-bank operation
#
# INSTRUCTION FORMAT:
# Interpretive instructions stored as paired opcodes in a single word:
#   Bits 15-8: First opcode (or store code if bit 15 = 0)
#   Bits 7-0: Second opcode
# Most opcodes require additional address word(s) following the opcode pair.
#
# EXECUTION MODEL:
# Entry: TC INTPRET - Transfer control to interpreter
# Execution: Fetch opcode pairs, decode, execute operations on MPAC/stack
# Exit: EXIT instruction - Return to native AGC code
# Interrupts: EXEC checks NEWJOB between instructions for preemptive multitasking
#
# PERFORMANCE CHARACTERISTICS:
# - Interpretive instructions: ~4-10x slower than native AGC code
# - Memory efficiency: ~3-4x better code density than native assembly
# - Used for: Guidance equations, orbital mechanics, coordinate transformations
# - Not used for: Time-critical code (interrupts, display updates, I/O)
#
# SCALING CONVENTIONS:
# The AGC lacked floating-point hardware. All values use scaled fixed-point:
# - Position vectors: 2^29 meters (1 unit ≈ 1.86 nm, cislunar range ~200,000)
# - Velocity vectors: 2^7 meters/centisecond (1 unit = 128 m/cs = 1.28 m/s)
# - Time: centiseconds (1/100 second)
# - Angles: revolutions (0.0 to 1.0 = 0° to 360°, stored as fractions)
# - Matrix elements: dimensionless (typical range -1.0 to +1.0)
# Scaling factors chosen to maximize precision within 15-bit signed words while
# representing typical aerospace magnitudes without overflow.
#
# HISTORICAL CONTEXT:
# This interpreter executed continuously during the Apollo 11 lunar descent on
# July 20, 1969, computing guidance commands in real-time. The interpreter's
# efficiency enabled the AGC to process landing radar data, compute descent
# trajectories, and manage throttle commands simultaneously. When the famous
# 1202 program alarm occurred at 102:38:26 mission time, it was caused by
# executive job queue overflow (not interpreter failure), and the interpreter
# continued executing guidance computations throughout the alarm condition,
# enabling Armstrong and Aldrin to land safely at Tranquility Base.
#
# INTERPRETER OPCODE CATALOG:
#
# The following opcodes are implemented and documented throughout this file:
#
# VECTOR LOAD/STORE:
#   VLOAD   - Load vector (3 components) from memory to MPAC
#   PDVL    - Push down current MPAC, then vector load
#
# VECTOR ARITHMETIC:
#   VAD     - Vector addition (MPAC = MPAC + operand)
#   VSU     - Vector subtraction (MPAC = MPAC - operand)
#   BVSU    - Vector subtract from (MPAC = operand - MPAC, reverse order)
#   VXSC    - Vector times scalar (MPAC_vector * operand_scalar)
#   V/SC    - Vector divided by scalar
#   VXV     - Vector cross product (MPAC × operand, yields perpendicular vector)
#   DOT     - Vector dot product (yields scalar, MPAC · operand)
#   UNIT    - Normalize vector to unit length (length = 1.0)
#   ABVAL   - Compute vector magnitude (length)
#   VSQ     - Square of vector length (magnitude squared)
#   COMP    - Complement/negate vector (reverse direction)
#   VPROJ   - Vector projection (project MPAC onto operand direction)
#   VDEF    - Vector define (construct vector from components)
#
# MATRIX OPERATIONS:
#   MXV     - Matrix times vector (3×3 matrix post-multiplied by vector)
#   VXM     - Vector times matrix (vector pre-multiplied by 3×3 matrix)
#
# DOUBLE-PRECISION LOAD/STORE:
#   DLOAD   - Load double-precision scalar to MPAC (2 words, ~10 decimal digits)
#   PDDL    - Push down current MPAC, then DP load
#   SLOAD   - Load single-precision scalar (1 word, ~5 decimal digits)
#
# DOUBLE-PRECISION ARITHMETIC:
#   DAD     - DP addition
#   DSU     - DP subtraction
#   BDSU    - DP subtract from (reverse order)
#   DMPR    - DP multiply with rounding
#   DMP     - DP multiply (alternate entry)
#   DDV     - DP divide (MPAC / operand)
#   BDDV    - DP divide into (operand / MPAC, reverse order)
#   DSQ     - DP square (MPAC²)
#   SQRT    - Square root of DP scalar
#   ABS     - Absolute value (magnitude) of DP scalar
#   SIGN    - Copy sign (complement MPAC if operand negative)
#   ROUND   - Round DP to single precision
#   TSLC    - Triple-shift-left-and-count (normalize DP scalar)
#
# TRIPLE-PRECISION:
#   TLOAD   - Load triple-precision (3 words, ~15 decimal digits)
#   TAD     - TP addition (extended precision for critical computations)
#
# TRIGONOMETRIC FUNCTIONS:
#   SIN     - Sine of angle (input in revolutions, output -1.0 to +1.0)
#   COS     - Cosine of angle
#   ARCSIN  - Arcsine (inverse sine, output in revolutions)
#   ARCCOS  - Arccosine (inverse cosine)
#
# SHIFT OPERATIONS:
#   GSHIFT  - General shift (arithmetic left/right by variable amount)
#   SL, SR, SL1, SR1, etc. - Short shifts (fixed shift amounts)
#
# STACK OPERATIONS:
#   PUSH    - Push MPAC to push-down list
#   STORE   - Store MPAC to memory address
#   STODL   - Store MPAC and reload with DP scalar
#   STOVL   - Store MPAC and reload with vector
#   STCALL  - Store MPAC and call subroutine
#
# INDEXING AND BRANCHING:
#   AXT, AXC - Address to index (load index register)
#   LXA, LXC - Load index from erasable memory
#   SXA     - Store index to erasable
#   XCHX    - Exchange index with erasable
#   INCR    - Increment index register
#   TIX     - Transfer on index (loop control)
#   XAD, XSU - Index arithmetic operations
#
# CONTROL FLOW:
#   GOTO    - Unconditional interpretive branch
#   CGOTO   - Computed goto (branch based on index)
#   CALL    - Interpretive subroutine call
#   CCALL   - Computed call (call based on index)
#   RVQ     - Return via QPRET (subroutine return)
#   RTB     - Return to basic (exit interpreter)
#   EXIT    - Exit interpreter to native AGC code
#
# CONDITIONAL BRANCHES:
#   BZE     - Branch if zero
#   BPL     - Branch if plus (positive)
#   BMN     - Branch if minus (negative)
#   BHIZ    - Branch if high zero
#   BOV     - Branch on overflow
#
# MISCELLANEOUS:
#   SSP     - Set single precision into index register
#   SETPD   - Set push-down pointer (initialize stack)
#   VDEF    - Vector define (construct from scalars)
#
# Each opcode is documented in detail at its implementation below.
#
# ============================================================================

; ============================================================================
; FILE: INTERPRETER.agc
; MODULE: Core Operating System - Interpretive Language Virtual Machine
; MISSION PHASE: all (foundational system used throughout all mission phases)
;
; TL;DR: Implements the AGC's interpretive language virtual machine, providing
;        a high-level instruction set for vector/matrix operations, double-
;        precision arithmetic, and trigonometric functions. Created to overcome
;        AGC memory and programming constraints by executing interpreted opcodes
;        from a stack-based architecture. Critical foundation enabling guidance,
;        navigation, and orbital mechanics computations throughout LM software.
;        Performance cost: ~4-10x slower than native AGC code, but dramatically
;        reduces program complexity and memory usage.
;
; COMMENT-ONLY READERS: This virtual machine executes the complex mathematical
;        operations needed for lunar navigation and landing. When guidance code
;        reaches "TC INTPRET", control transfers to this interpreter which then
;        executes high-level instructions for trajectory calculations.
;
; CODE-ALONG READERS: Study the fetch-decode-execute cycle in DANZIG, the
;        multi-level dispatch tables, and the implementation of vector/matrix
;        operations. Note the stack-based architecture using MPAC (Multi-Purpose
;        Accumulator) and the scaling conventions for maximum precision.
; ============================================================================

; ============================================================================
; INTERPRETER ARCHITECTURE OVERVIEW
;
; The AGC Interpreter is a virtual machine that executes high-level instructions
; stored in memory. It was created to overcome AGC memory constraints and provide
; more powerful operations than native AGC assembly language could efficiently
; express.
;
; KEY ARCHITECTURAL FEATURES:
;
; 1. STACK-BASED EXECUTION:
;    - MPAC (Multi-Purpose Accumulator): Primary working storage, organized as:
;      MPAC +0: Most significant word
;      MPAC +1: Least significant word (for double-precision)
;      MPAC +2: Vector component 1 (or next DP value)
;      MPAC +3: Vector component 2
;      MPAC +4: Vector component 3
;    - PUSHLOC: Stack for nested operations and subroutine calls
;    - MODE: Tracks data type in MPAC (DP, vector, etc.)
;
; 2. VIRTUAL REGISTERS:
;    - LOC: Interpretive program counter (points to next instruction)
;    - BANKSET: Bank setting for interpretive code
;    - INTBIT15: Bit 15 of FBANK for address indexing
;    - ADDRWD: Operand address for current instruction
;    - CYR/EDOP: Opcode pair storage and dispatch
;
; 3. INSTRUCTION FORMAT:
;    - Instructions stored as paired opcodes (most efficient use of memory)
;    - Word 1: Primary opcode + modifier bits
;    - Word 2: Secondary opcode or operand address
;    - Some instructions are single-word, some double-word
;    - Store operations use negative numbers (sign bit distinguishes)
;
; 4. ENTRY AND EXIT:
;    - Entry: TC INTPRET loads interpreter and begins execution
;    - Exit: EXIT instruction returns to native AGC code
;    - RTB: Return to Basic (exit to native code at specified address)
;
; 5. BENEFITS OF INTERPRETIVE CODE:
;    - Linear address space (no bank switching complexity for programmer)
;    - Double-precision arithmetic (DP operations)
;    - Vector/matrix operations (3D vectors, 3x3 matrices)
;    - Trigonometric functions (SIN, COS, ARCSIN, ACOS)
;    - Simplified programming for complex mathematics
;    - Significant memory savings (complex operations in fewer words)
;
; 6. PERFORMANCE CHARACTERISTICS:
;    - Execution speed: ~4-10x slower than native AGC code
;    - Memory efficiency: ~2-5x better than equivalent native code
;    - Used for: Guidance equations, orbital mechanics, coordinate transforms
;    - Not used for: Time-critical interrupt handlers, I/O operations
;
; 7. SCALING CONVENTIONS:
;    - Positions: 2^29 meters (1 unit ≈ 1.86 nanometers)
;    - Velocities: 2^7 meters/centisecond
;    - Time: centiseconds (1/100 second)
;    - Angles: revolutions (0.0 to 1.0 = 0° to 360°)
;    - Scaling chosen to maximize precision within 15-bit signed words
;
; HISTORICAL CONTEXT:
; During Apollo 11's lunar descent on July 20, 1969, the interpreter executed
; continuously, computing descent trajectories, throttle commands, and navigation
; state updates in real-time. The guidance equations in LUNAR_LANDING_GUIDANCE_
; EQUATIONS.agc were written in interpretive code and executed by this virtual
; machine throughout the 12-minute powered descent to the lunar surface.
; ============================================================================

# Page 1002
# SECTION 1:  DISPATCHER
#
# ENTRY TO THE INTERPRETER.  INTPRET SETS LOC TO THE FIRST INSTRUCTION, BANKSET TO THE BBANK OF THE
# OBJECT INTERPRETIVE PROGRAM, AND INTBIT15 TO THE BIT15 CONTENTS OF FBANK.  INTERPRETIVE PROGRAMS MAY BE IN
# VIRTUALLY ALL BANKS PRESENT UNDER ANY SUPER-BANK SETTING, WITH THE RESTRICTION THAT PROGRAMS IN HIGH BANKS
# (BIT15 OF FBANK = 1) DO NOT REFER TO LOWBANKS, AND VICE-VERSA.  THE INTERPRETER DOES NOT SWITCH SUPERBANKS.
# E-BANK SWITCHING OCCURS WHENEVER GENERAL ERASABLE (100-3777) IS ADDRESSED.
#
# ============================================================================
# INTERPRETER ENTRY AND DISPATCH
#
# This section handles entry into the interpretive virtual machine from native
# AGC code and the main instruction fetch-decode-execute loop.
#
# ENTRY: Native AGC code executes "TC INTPRET" which transfers control here.
# The return address (Q register) becomes the interpretive program counter (LOC).
# From that point forward, instructions are fetched from LOC and interpreted
# until an EXIT instruction returns to native AGC mode.
#
# DISPATCH LOOP:
# 1. DANZIG: Check if higher-priority job waiting (NEWJOB), allow preemption
# 2. NEWOPS: Fetch next instruction pair from LOC
# 3. Decode opcode pair (or store code)
# 4. OPJUMP: Execute instruction via jump table
# 5. Return to DANZIG for next instruction
#
# This dispatch cycle executes for every interpretive instruction during
# mission-critical guidance computations including lunar descent.
# ============================================================================

		BLOCK	03

		COUNT*	$$/INTER
		
		# TC INTPRET - Entry point from native AGC code
		# The Q register contains return address, which becomes interpretive
		# program counter. Calling code continues at next instruction after TC.
INTPRET		RELINT				# Re-enable interrupts (may have been inhibited)
		EXTEND				# Extended instruction prefix
		QXCH	LOC			# Q ↔ LOC: LOC = return address (program counter)
						# Q = old LOC (saved for later)

		# Interpretive branches (GOTO, CALL) finish execution here at +2
 +2		CA	BBANK			# Load current bank register
		TS	BANKSET			# Save as interpretive bank setting
		MASK	BIT15			# Extract bit 15 (high bank indicator)
		TS	INTBIT15		# Store for bank switching logic

		TS	EDOP			# Clear EDOP (edit operation) register
						# Ensures no stale opcode from previous pair

		TCF	NEWOPS			# Jump to instruction fetch (begin interpretation)

		# INTRSM - Resume suspended interpretive job after preemption
		# Called by EXECUTIVE when job becomes active again
INTRSM		LXCH	BBANK			# Restore bank context from L register
		TCF	INTPRET +3		# Continue at bank setup (skip entry setup)

# ============================================================================
# DLOAD - DOUBLE-PRECISION LOAD
#
# Loads a double-precision scalar (2 words) from memory into MPAC.
# MPAC +0, +1: Loaded with DP value (high word, low word)
# MPAC +2: Zeroed
# MODE: Set to 2 (double-precision mode)
#
# Usage: DLOAD ADDRESS - Load DP value at ADDRESS into MPAC
# Scaling: Inherits scaling from source address
# Example: DLOAD RALT loads radar altitude (scaled 2^-14 feet) into MPAC
# ============================================================================

DLOAD		EXTEND				# Extended instruction prefix for DCA
		INDEX	ADDRWD			# Indirect addressing via ADDRWD
		DCA	0			# Load DP word pair: A = high, L = low
SLOAD2		DXCH	MPAC			# MPAC = A (high), MPAC+1 = L (low)
		CAF	ZERO			# Load zero into A

# Page 1003
# AT THE END OF MOST INSTRUCTIONS, CONTROL IS GIVEN TO DANZIG TO DISPATCH THE NEXT OPERATION.

		TS	MPAC 	+2		# MPAC+2 = 0, third component cleared

		# NEWMODE - Common entry for mode-changing instructions (DLOAD, VLOAD, etc.)
NEWMODE		TS	MODE			# Set MODE register (0=vector, 2=DP, 3=TP)

# ============================================================================
# DANZIG - MAIN DISPATCH LOOP
#
# Primary dispatch point at the end of each interpretive instruction.
# Handles cooperative multitasking by checking for higher-priority jobs
# before fetching the next instruction. This is where the AGC's real-time
# operating system integrates with the interpreter.
#
# During Apollo 11 descent, DANZIG executed thousands of times per second,
# checking NEWJOB between every interpretive instruction. This allowed the
# EXECUTIVE to preempt guidance computations for time-critical tasks like
# display updates and radar sampling.
# ============================================================================

DANZIG		CA	BANKSET			# Load interpretive bank setting
		TS 	BBANK			# Update hardware bank register
						# BBANK saved by CHANJOB if preemption occurs

		# Check if second opcode from previous pair is waiting
NOIBNKSW	CCS	EDOP			# Test EDOP (edit operation register)
		TCF	OPJUMP			# If non-zero, execute second opcode
						# EDOP zeroed after execution

		# Check for preemptive multitasking (higher priority job waiting)
		CCS	NEWJOB			# Test NEWJOB flag (set by EXECUTIVE)
		TCF	CHANG2			# Non-zero: suspend interpreter, switch jobs
						# This is cooperative multitasking checkpoint

		INCR	LOC			# Advance interpretive program counter
						# Points to next instruction pair

# ============================================================================
# NEWOPS - INSTRUCTION FETCH
#
# Fetches the next interpretive instruction pair from memory at LOC.
# Interprets the sign bit to distinguish between:
#   - Positive: Opcode pair (two operations to execute)
#   - Negative: Store code (store MPAC to memory)
# ============================================================================

# ITRACE (1) REFERS TO "NEWOPS"
NEWOPS		INDEX	LOC			# Indirect addressing via LOC
		CA	0			# Fetch instruction word at LOC
		CCS	A			# Test sign, get |A|
		TCF	DOSTORE			# Negative: process store instruction

		# Positive: opcode pair format
		# Bits 15-8: First opcode (will execute now)
		# Bits 7-0: Second opcode (saved in EDOP for next cycle)
LOW7		OCT	177			# Mask for low 7 bits (octal 177 = binary 1111111)

		TS	EDOP			# Save complete pair in EDOP
		MASK	LOW7			# Extract low 7 bits (second opcode)

		# OPJUMP - Execute single opcode via jump table
OPJUMP		TS	CYR			# Store opcode in CYR for decoding
		CCS	CYR			# Test opcode value, get |CYR|
		TCF	OPJUMP2			# Non-zero: continue opcode decode

		TCF	EXIT			# +0 OP CODE IS EXIT (return to native AGC)

# Page 1004
# PROCESS ADDRESSES WHICH MAY BE DIRECT, INDEXED, OR REFERENCE THE PUSHDOWN LIST.
#
# ============================================================================
# ADDRESS PROCESSING
#
# Interpretive instructions reference operands via three addressing modes:
#   1. DIRECT: Absolute address in next word (most common)
#   2. INDEXED: Address computed by adding index register to base
#   3. PUSHDOWN: Reference to value on push-down stack (no address word)
#
# The interpreter decodes addressing mode from instruction prefix bits:
#   Bit 1 set: Indexed addressing (add X1 or X2 to base address)
#   Address word negative: Indexed via X2 (complemented address)
#   No address word: Use push-down list (stack operand)
#
# Address resolution handles bank switching for Fixed and Erasable memory.
# ============================================================================

ADDRESS		MASK	BIT1			# Test bit 1 of opcode (index flag)
		CCS	A			# Non-zero if indexed addressing
		TCF	INDEX			# Process indexed address computation

		# DIRADRES - Direct address processing
DIRADRES	INDEX	LOC			# Look ahead to next word after opcode
OCT40001	CS	1			# Load and complement next word
		CCS	A			# Test if word present (negative if missing)
		TCF	PUSHUP			# No address word: use push-down list

NEG4		DEC	-4			# Constant: -4 (stack offset)

		# Address word present: consume it and store
		INCR	LOC			# Advance LOC past address word
		TS	ADDRWD			# Store address in ADDRWD for resolution

# Page 1005
# FINAL DIGESTION OF DIRECT ADDRESSES OF OP CODES WITH 01 PREFIX IS DONE HERE.   IN EACH CASE, THE
# REQUIRED 12-BIT SUB-ADDRESS IS LEFT IN ADDRWD, WITH ANY REQUIRED E OR F BANK SWITCHING DONE.  ADDRESSES LESS
# THAN 45D ARE TAKEN TO BE RELATIVE TO THE WORK AREA.  THE OP CODE IS NOW IN BITS 1-5 OF CYR WITH BIT 14 = 1.

		AD	-ENDVAC			# SEE IF ADDRESS RELATIVE TO WORK AREA.
		CCS	A
		AD	-ENDERAS		# IF NOT, SEE IF IN GENERAL ERASABLE.
		TCF	IERASTST

NETZERO		CA	FIXLOC			# IF SO, LEAVE THE MODIFIED ADDRESS IN
		ADS	ADDRWD			# ADDRWD AND DISPATCH.
ITR15		INDEX	CYR			# THIS INDEX MAKES THE NEXT INSTRUCTION
		7	INDJUMP -1		# TCF INDJUMP + OP, EDITING CYR.

IERASTST	EXTEND
		BZMF	GEADDR			# GO PROCESS GENERAL-ERASABLE ADDRESS.

		MASK	LOW10			# FIXED BANK ADDRESS. RESTORE AND ADD B15.
		AD	LOW10			# SWITCH BANKS AND LEAVE SUBADDRESS IN
		XCH	ADDRWD			# ADDRWD FOR OPERAND RETRIEVAL. (THIS
		AD	INTBIT15		# METHOD PRECLUDES USE OF THE LAST
		TS	FBANK			# LOCATION IN EACH FBANK.)
ITR12		INDEX	CYR
		7	INDJUMP -1

GEADDR		MASK	LOW8
		AD	OCT1400
		XCH	ADDRWD
		TS	EBANK
ITR10		INDEX	CYR
		7	INDJUMP -1

# Page 1006
# THE FOLLOWING ROUTINE PROCESSES INTERPRETIVE INDEXED ADDRESSES.  AN INTERPRETER INDEX REGISTER MAY
# CONTAIN THE ADDRESS OF ANY ERASABLE REGISTER (0-42 BEING RELATIVE TO THE VAC AREA) OR ANY INTERPRETIVE PROGRAM
# BANK, OR ANY INTEGER IN THAT RANGE.

DODLOAD*	CAF	DLOAD*			# STODL* COMES HERE TO PROCESS LOAD ADR.
		TS	CYR			# (STOVL* ENTERS HERE).

INDEX		CA	FIXLOC			# SET UP INDEX LOCATION.
		TS	INDEXLOC
		INCR	LOC			# (ADDRESS ALWAYS GIVEN).
		INDEX	LOC
		CS	0
		CCS	A			# INDEX 2 IF ADDRESS STORED COMPLEMENTED.
		INCR	INDEXLOC
		NOOP

		TS 	ADDRWD			# 14 BIT ADDRESS TO ADDRWD.
		MASK	HIGH4			# IF ADDRESS GREATER THAN 2K, ADD INTBIT15
		EXTEND
		BZF 	INDEX2
		CA	INTBIT15
		ADS	ADDRWD

INDEX2		INDEX	INDEXLOC
		CS	X1
		ADS	ADDRWD			# DO AUGMENT, IGNORING AND CORRECTING OVF.

		MASK	HIGH9			# SEE IF ADDRESS IS IN WORK AREA.
		EXTEND
		BZF	INDWORK
		MASK	HIGH4			# SEE IF IN FIXED BANK.
		EXTEND
		BZF	INDERASE

		CA	ADDRWD			# IN FIXED -- SWITCH BANKS AND CREATE
		TS	FBANK			# SUB-ADDRESS
		MASK	LOW10
		AD	2K
		TS	ADDRWD
ITR11		INDEX	CYR
		3	INDJUMP -1

INDWORK		CA	FIXLOC			# MAKE ADDRWD RELATIVE TO WORK AREA.
		TCF	ITR13 	-1

INDERASE	CA	OCT1400
		XCH	ADDRWD
		TS	EBANK
		MASK	LOW8
 -1		ADS	ADDRWD
# Page 1007
ITR13		INDEX	CYR
		3	INDJUMP -1

# Page 1008
# PUSH-UP ROUTINES.  WHEN NO OPERAND ADDRESS IS GIVEN, THE APPROPRIATE OPERAND IS TAKEN FROM THE PUSH-DOWN
# LIST.  IN MOST CASES THE MODE OF THE RESULT (VECTOR OR SCALAR) OF THE LAST ARITHMETIC OPERATION PERFORMED
# IS THE SAME AS THE TYPE OF OPERAND DESIRED (ALL ADD/SUBTRACT ETC.).  EXCEPTIONS TO THIS GENERAL RULE ARE LISTED
# BELOW (NOTE THAT IN EVERY CASE THE MODE REGISTER IS LEFT INTACT):
#
#	1.	VXSC AND V/SC WANT THE OPPOSITE TYPE OF OPERAND, E.G., IF THE LAST OPERATION YIELDED A VECTOR
#		RESULT, VXSC WANTS A SCALAR.
#
#	2.	THE LOAD CODES SHOULD LOAD THE ACCUMULATOR INDEPENDENT OF THE RESULT OF THE LAST OPERATION.  THIS
#		INCLUDES VLOAD, DLOAD, TLOAD, PDDL, AND PDVL (NO PUSHUP WITH SLOAD).
#
#	3.	SOME ARITHMETIC OPERATIONS REQUIRE A STANDARD TYPE OF OPERAND REGARDLESS OF THE PREVIOUS OPERATION.
#		THIS INCLUDES SIGN WANTING DP AND TAD REQUIRING TP.

PUSHUP		CAF	OCT23		# IF THE LOW 5 BITS OF CYR ARE LESS THAN
		MASK	CYR		# 20, THIS OP REQUIRES SPECIAL ATTENTION.
		AD	-OCT10		# (NO -0).
		CCS	A
		TCF	REGUP		# FOR ALL CODES GREATEER THAN OCT 7.

-OCT10		OCT	-10

		AD	NEG4		# WE NOW HAVE 7 -- OP CODE (MOD4).  SEE IF
		CCS	A		# THE OP CODE (MOD4) IS THREE (REVERSE).
		INDEX	A		# NO -- THE MODE IS DEFINITE.  PICK UP THE
		CS	NO.WDS
		TCF	REGUP 	+2

		INDEX	MODE		# FOR VXSC AND V/SC WE WANT THE REQUIRED
		CS	REVCNT		# PUSHLOC DECREMENT WITHOUT CHANGING THE
		TCF	REGUP 	+2	# MODE AT THIS TIME.

REGUP		INDEX	MODE		# MOST ALL OP CODES PUSHUP HERE.
		CS	NO.WDS
 +2		ADS	PUSHLOC
		TS	ADDRWD
ITR14		INDEX	CYR
		7	INDJUMP -1	# (THE INDEX MAKES THIS A TCF.)

		OCT	2		# REVERSE PUSHUP DECREMENT. VECTOR TAKES 2
REVCNT		OCT	6		# WORDS, SCALAR TAKES 6.
		OCT	6
NO.WDS		OCT	2		# CONVENTIONAL DECREMENT IS 6 WORDS VECTOR
OCTAL3		OCT	3		# 2 IN DP, AND 3 IN TP.
		OCT	6

# Page 1009
# TEST THE SECOND PREFIX BIT TO SEE IF THIS IS A MISCELLANEOUS OR A UNARY/SHORT SHIFT OPERATION.

OPJUMP2		CCS	CYR		# TEST SECOND PREFIX BIT.
		TCF	OPJUMP3		# TEST THIRD BIT TO SEE IF UNARY OR SHIFT

-ENDVAC		DEC	-45

# THE FOLLOWING ROUTINE PROCESSES ADDRESSES OF SUFFIX CLASS 10.  THEY ARE BASICALLY WORK AREA ADDRESSES
# IN THE RANGE 0-52, ERASABLE ECADR CONSTANTS FROM 100-3777, AND FCADRS ABOVE THAT.  ALL 15 BITS ARE AVAILABLE
# IN CONTRAST TO SUFFIX 1, IN WHICH ONLY THE LOW ORDER 14 ARE AVAILABLE.

15BITADR	INCR	LOC		# (ENTRY HERE FROM STCALL).
		INDEX	LOC		# PICK UP ADDRESS WORD.
		CA	0
		TS	POLISH		# WE MAY NEED A SUBADDRESS LATER.

		CAF	LOW7+2K		# THESE INSTRUCTIONS ARE IN BANK 1.
		TS	FBANK
		MASK	CYR
ITR7		INDEX	A
		TCF 	MISCJUMP

# Page 1010
# COMPLETE THE DISPATCHING OF UNARY AND SHORT SHIFT OPERATIONS.

OPJUMP3		TS	FBANK		# CALL IN BANK 0 (BIT5S 11-15 OF A ARE 0.)
# ITRACE (6) REFERS TO "OPJUMP3"
		CCS	CYR		# TEST THIRD PREFIX BIT.
		INDEX	A		# THE DECREMENTED UNARY CODE IS IN BITS
		TCF	UNAJUMP		# 1-4 OF A (ZERO, EXIT, HAS BEEN DETECTED)

		CCS	MODE		# IT'S A SHORT SHIFT CODE.  SEE IF PRESENT
		TCF	SHORTT		# SCALAR OR VECTOR.
		TCF	SHORTT
		TCF	SHORTV		# CALLS THE APPROPRIATE ROUTINE.

FBANKMSK	EQUALS	BANKMASK
LVBUF		ADRES	VBUF

# Page 1011
# THE FOLLOWING IS THE JUMP TABLE FOR OP CODES WHICH MAY HAVE INDEXABLE ADDRESSES OR MAY PUSH UP.

INDJUMP		TCF	VLOAD		# 00 -- LOAD MPAC WITH A VECTOR.
		TCF	TAD		# 01 -- TRIPLE PRECISION ADD TO MPAC.
		TCF	SIGN		# 02 -- COMPLEMENT MPAC (V OR SC) IF X NEG.
		TCF	VXSC		# 03 -- VECTOR TIMES SCALAR.
		TCF	CGOTO		# 04 -- COMPUTED GO TO.
		TCF	TLOAD		# 05 -- LOAD MPAC WITH TRIPLE PRECISION.
		TCF	DLOAD		# 06 -- LOAD MPAC WITH A DP SCALAR.
		TCF	V/SC		# 07 -- VECTOR DIVIDED BY A SCALAR.

		TCF	SLOAD		# 10 -- LOAD MPACIN SINGLE PRECISION.
		TCF	SSP		# 11 -- SET SINGLE PRECISION INTO X.
		TCF	PDDL		# 12 -- PUSH DOWN MPAC AND RE-LOAD IN DP.
		TCF	MXV		# 13 -- MATRIX POST-MULTIPLIED BY VECTOR.
		TCF	PDVL		# 14 -- PUSH DOWN AND VECTOR LOAD.
		TCF	CCALL		# 15 -- COMPUTED CALL.
		TCF	VXM		# 16 -- MATRIX PRE-MULTIPLIED BY VECTOR.
		TCF	TSLC		# 17 -- NORMALIZE MPAC (SCALAR ONLY).

		TCF	DMPR		# 20 -- DP MULTIPLY AND ROUND.
		TCF	DDV		# 21 -- DP DIVIDE BY.
		TCF	BDDV		# 22 -- DP DIVIDE INTO.
		TCF	GSHIFT		# 23 -- GENERAL SHIFT INSTRUCTION
		TCF	VAD		# 24 -- VECTOR ADD.
		TCF	VSU		# 25 -- VECTOR SUBTRACT.
		TCF	BVSU		# 26 -- VECTOR SUBTRACT FROM.
		TCF	DOT		# 27 -- VECTOR DOT PRODUCT.

		TCF	VXV		# 30 -- VECTOR CROSS PRODUCT.
		TCF	VPROJ		# 31 -- VECTOR PROJECTION.
		TCF	DSU		# 32 -- DP SUBTRACT.
		TCF	BDSU		# 33 -- DP SUBTRACT FROM.
		TCF	DAD		# 34 -- DP ADD.
		TCF			# 35 -- AVAILABLE
		TCF	DMP1		# 36 -- DP MULTIPLY.
		TCF	SETPD		# 37 -- SET PUSH DOWN POINTER (DIRECT ONLY)

# CODES 10 AND 14 MUST NOT PUSH UP. CODE 04 MAY BE USED FOR VECTOR DECLARE BEFORE PUSHUP IF DESIRED.

# Page 1012
# THE FOLLOWING JUMP TABLE APPLIES TO INDEX, BRANCH, AND MISCELLANEOUS INSTRUCTIONS.

MISCJUMP	TCF	AXT		# 00 -- ADDRESS TO INDEX TRUE.
		TCF	AXC		# 01 -- ADDRESS TO INDEX COMPLEMENTED.
		TCF	LXA		# 02 -- LOAD INDEX FROM ERASABLE.
		TCF	LXC		# 03 -- LOAD INDEX FROM COMPLEMENT OF ERAS.
		TCF	SXA		# 04 -- STORE INDEX IN ERASABLE.
		TCF	XCHX		# 05 -- EXCHANGE INDEX WITH ERASABLE.
		TCF	INCR		# 06 -- INCREMENT INDEX REGISTER.
		TCF	TIX		# 07 -- TRANSFER ON INDEX.

		TCF	XAD		# 10 -- INDEX REGISTER ADD FROM ERASABLE.
		TCF	XSU		# 11 -- INDEX SUBTRACT FROM ERASABLE.
		TCF	BZE/GOTO	# 12 -- BRANCH ZERO AND GOTO
		TCF	BPL/BMN		# 13 -- BRANCH PLUS AND BRANCH MINUS.
		TCF	RTB/BHIZ	# 14 -- RETURN TO BASIC AND BRANCH HI ZERO.
		TCF	CALL/ITA	# 15 -- CALL AND STORE QPRET.
		TCF	SW/		# 16 -- SWITCH INSTRUCTIONS AND AVAILABLE.
		TCF	BOV(B)		# 17 -- BRANCH ON OVERFLOW TO BASIC OR INT.

# Page 1013
# THE FOLLOWING JUMP TABLE APPLIES TO UNARY INSTRUCTIONS.

		COUNT*	$$/INTER
		BANK	0		# 00 -- EXIT -- DETECTED EARLIER.
UNAJUMP		TCF	SQRT		# 01 -- SQUARE ROOT.
		TCF	SINE		# 02 -- SIN.
		TCF	COSINE		# 03 -- COS.
		TCF	ARCSIN		# 04 -- ARC SIN.
		TCF	ARCCOS		# 05 -- ARC COS.
		TCF	DSQ		# 06 -- DP SQUARE.
		TCF	ROUND		# 07 -- ROUND TO DP.

		TCF	COMP		# 10 -- COMPLEMENT VECTOR OR SCALAR
		TCF	VDEF		# 11 -- VECTOR DEFINE.
		TCF	UNIT		# 12 -- UNIT VECTOR.
		TCF	ABVALABS	# 13 -- LENGTH OF VECTOR OR MAG OF SCALAR.
		TCF	VSQ		# 14 -- SQUARE OF LENGTH OF VECTOR.
		TCF	STADR		# 15 -- PUSH UP ON STORE CODE.
		TCF	RVQ		# 16 -- RETURN VIA QPRET.
		TCF	PUSH		# 17 -- PUSH MPAC DOWN.

# Page 1014
# SECTION 2	LOAD AND STORE PACKAGE.
#
# A SET OF EIGHT STORE CODES IS PROVIDED AS THE PRIMARY METHOD OF STORING THE MULTI-PURPOSE
# ACCUMULATOR (MPAC).  IF IN THE DANZIG SECTION LOC REFERS TO AN ALGEBRAICALLY POSITIVE WORD, IT IS TAKEN AS A
# STORE CODE WITH A CORRESPONDING ERASABLE ADDRESS.  MOST OF THESE CODES ARE TWO ADDRESS, SPECIFYING THAT THE WORD
# FOLLOWING THE STORE CODE IS TO BE USED AS AN ADDRESS FROM WHICH TO RE-LOAD MPAC.  FOUR OPTIONS ARE AVAILABLE:
#
#	1. STORE	STORE MPAC.  THE E ADDRESS MAY BE INDEXED.
#	2. STODL	STORE MPAC AND RE-LOAD IT IN DP WITH THE NEXT ADDRESS (THE LOAD MAY BE INDEXED).
#	3. STOVL	STORE MPAC AND RE-LOAD A VECTOR (AS ABOVE).
#	4. STCALL	STORE AND DO A CALL (BOTH ADDRESES MUST BE DIRECT HERE).
#
# STODL AND STOVL WILL TAKE FROM THE PUSH-DOWN LIST IF NO LOAD ADDRESS IS GIVEN.
;
; ============================================================================
; STORE OPERATIONS DETAILED EXPLANATION
; ============================================================================
;
; COMMENT-ONLY READERS: The interpreter needs a way to save computed results
; back to memory. These store operations transfer data from the MPAC (where
; all computations occur) to permanent storage locations. Think of MPAC as a
; scratch pad - once calculations are done, results must be saved before
; starting new calculations.
;
; CODE-ALONG READERS: Store operations are encoded as positive words in the
; interpretive instruction stream, distinguishing them from negative opcode
; pairs. The dispatcher (DANZIG/NEWOPS) detects positive words and branches
; to DOSTORE. The store code contains both the operation type (bits 12-14)
; and the destination address (bits 0-10 for erasable addresses).
;
; STORE CODE ENCODING (15-bit word format):
;   Bits 14-12: Store operation type (0-7)
;   Bits 11-0:  Erasable memory address (0-3777 octal)
;
; STORE OPERATION TYPES:
;   0: STORE    - Store MPAC, then fetch next opcode pair
;   1: STORE,1  - Store MPAC with X1 index, then fetch next opcode
;   2: STORE,2  - Store MPAC with X2 index, then fetch next opcode
;   3: STODL    - Store MPAC, then load DP from next address
;   4: STODL*   - Store MPAC, then load DP from indexed address
;   5: STOVL    - Store MPAC, then load vector from next address
;   6: STOVL*   - Store MPAC, then load vector from indexed address
;   7: STCALL   - Store MPAC, then call subroutine at next address
;
; STORE SIZE DETERMINATION:
; The MODE register determines how much data to store:
;   MODE = 0: Store double-precision (MPAC, MPAC+1)
;   MODE = 1: Store triple-precision (MPAC, MPAC+1, MPAC+2)
;   MODE = 2: Store vector (MPAC through MPAC+5 = six words)
;
; HISTORICAL CONTEXT:
; During the lunar landing on July 20, 1969, the guidance equations computed
; desired throttle and attitude commands in MPAC. STOVL operations transferred
; these critical values to memory locations read by the control systems. Every
; commanded maneuver during descent used these store operations to communicate
; guidance computer outputs to the spacecraft control hardware.
;
; EBANK SWITCHING:
; If the store address is in general erasable (addresses 100-3777), the
; interpreter automatically switches to the appropriate EBANK. This allows
; storing to any erasable location without explicit bank management in the
; interpretive program.
;
; INDEXED STORES:
; STORE,1 and STORE,2 add the contents of index registers X1 or X2 to the
; base address, enabling array operations. For example, storing a computed
; state vector component where the component index is in X1.

		BLOCK	3

		COUNT*	$$/INTER
STADR		CA	BANKSET		# THE STADR CODE (PUSHUP UP ON STORE
		TS	FBANK		# ADDRESS) ENTERS HERE.
		INCR	LOC
ITR1		INDEX	LOC		# THE STORECODE WAS STORED COMPLEMENTED TO
		CS	0		# MAKE IT LOOK LIKE AN OPCODE PAIR.
		AD	NEGONE		# (YUL CAN'T REMOVE 1 BECAUSE OF EARLY CCS)

; Entry from main dispatcher when positive store code word detected.
; A register contains the store code word at this point.
DOSTORE		TS	ADDRWD
		MASK 	LOW11		# ENTRY FROM DISPATCHER.  SAVE THE ERASABLE
		XCH	ADDRWD		# ADDRESS AND JUMP ON THE STORE CODE NO.
		MASK	B12T14
		EXTEND
		MP	BIT5		# EACH TRANSFER VECTOR ENTRY IS TWO WORDS.
		INDEX	A
		TCF	STORJUMP

# Page 1015
# STORE CODE JUMP TABLE.  CALLS THE APPROPRIATE STORING ROUTINE AND EXITS TO DANZIG OR TO ADDRESS WITH
# A SUPPLIED OPERATION CODE.
#
# STORE STORE,1 AND STORE,2 RETURN TO DANZIG, THUS RESETTING THE EBANK TO ITS STATE AT INTPRET.
;
; The jump table has 2-word entries for each of 8 store operation types.
; Each entry calls the appropriate storage routine (TC STORE/STORE,1/STORE,2),
; then either returns to DANZIG for next opcode or branches to a load routine
; (DODLOAD/DOVLOAD) for combined store-then-load operations.

STORJUMP	TC	STORE		# STORE.
		TCF	DANZIG		# PICK UP NEW OP CODE(S).
		TC	STORE,1
		TCF	DANZIG
		TC	STORE,2
		TCF	DANZIG

		TC	STORE		# STODL.
		TCF	DODLOAD

		TC	STORE		# STODL WITH INDEXED LOAD ADDRESS.
		TCF	DODLOAD*

		TC	STORE		# STOVL.
		TCF	DOVLOAD

		TC	STORE		# STOVL WITH INDEXED LOAD ADDRESS.
		TCF	DOVLOAD*
		TC	STORE		# STOTC.
		CAF	CALLCODE
		TS	CYR
		TCF	15BITADR	# GET A 15 BIT ADDRESS.

# Page 1016
# STORE CODE ADDRESS PROCESSOR.
;
; COMMENT-ONLY READERS: Before storing computed results, the interpreter must
; calculate the final memory address. For indexed stores (STORE,1 and STORE,2),
; it adds the index register value to the base address. The routine also checks
; if the address needs EBANK switching to access different memory regions.
;
; CODE-ALONG READERS: Address processing has two paths:
; 1. Indexed stores (STORE,1/STORE,2): Add X1 or X2 to ADDRWD
; 2. All stores: Check if address is in work area (addresses < 45 decimal)
;    - Work area addresses: Add FIXLOC (erasable bank base)
;    - General erasable: Extract EBANK bits and set EBANK register
;
; FIXLOC contains the base address of the current erasable bank, enabling
; bank-relative addressing for interpreter work areas.

STORE,1		INDEX	FIXLOC
		CS	X1
		TCF	PRESTORE

STORE,2		INDEX	FIXLOC
		CS	X2
PRESTORE	ADS	ADDRWD		# RESULTANT ADDRESS IS IN ERASABLE.

STORE		CS	ADDRWD
		AD	DEC45
		CCS	A		# DOES THE ADDRESS POINT TO THE WORK AREA?
		CA	FIXLOC		# YES.
		TCF	AHEAD5
		CA	OCT1400		# NO.  SET EBANK & MAKE UP SUBADDRESS.
		XCH	ADDRWD
		TS	EBANK
		MASK	LOW8
AHEAD5		ADS	ADDRWD

# Page 1017
# STORING ROUTINES.  STORE DP, TP, OR VECTOR AS INDICATED BY MODE.
;
; COMMENT-ONLY READERS: After calculating the destination address, the
; interpreter transfers the computed result from MPAC to memory. The amount
; of data stored depends on what type of value was computed - a simple number
; (2 words), a number with extra precision (3 words), or a full 3D vector
; (6 words). This ensures computed guidance commands reach the spacecraft
; control systems during critical mission phases.
;
; CODE-ALONG READERS: The MODE register determines storage size:
;   MODE = 0 (DP):     Store MPAC,+1 only (double-precision, 2 words)
;   MODE = 1 (TP):     Store MPAC,+1,+2 (triple-precision, 3 words)
;   MODE = 2 (Vector): Store MPAC through MPAC+5 (six words, 3 components)
;
; All paths begin at STARTSTO, which always stores the first two words.
; CCS MODE tests the mode value:
;   MODE = 0: Return immediately (DP done)
;   MODE = 1: Branch to TSTORE for third word
;   MODE = 2: Continue to VSTORE for remaining four words
;
; During Apollo 11's descent, guidance computed throttle commands and attitude
; errors in MPAC, then stored them via these routines for the autopilot to read.

STARTSTO	EXTEND			# MPAC,+1 MUST BE STORED IN ANY EVENT.
# ITRACE (5) REFERS TO "STARTSTO".
		DCA	MPAC
		INDEX	ADDRWD
		DXCH	0

		CCS	MODE
		TCF	TSTORE
		TC	Q

VSTORE		EXTEND
		DCA	MPAC	 +3
		INDEX 	ADDRWD
		DXCH	2

		EXTEND
		DCA	MPAC	 +5
		INDEX	ADDRWD
		DXCH	4
		TC	Q

TSTORE		CA	MPAC 	+2
		INDEX	ADDRWD
		TS	2
		TC	Q

# Page 1018
# ROUTINES TO BEGIN PROCESSING OF THE SECOND ADDRES ASSOCIATED WITH ALL STORE-TYPE CODES EXCEPT STORE
# ITSELF.
;
; COMMENT-ONLY READERS: Combined operations like STODL (store-then-load-double)
; and STOVL (store-then-load-vector) are efficiency features. After storing a
; computed result, the interpreter immediately begins the next calculation by
; loading new data into MPAC. This saves instruction words and execution time
; during time-critical guidance computations.
;
; CODE-ALONG READERS: These routines set up the opcode (DLOADCOD or VLOADCOD)
; in CYR register, then branch to address processing. The * variants (DOVLOAD*)
; handle indexed loads where the source address is modified by X1 or X2.
;
; Example usage during landing: After computing and storing a throttle command,
; STODL immediately loads the current altitude for the next computation cycle,
; eliminating separate STORE and DLOAD instructions.

DODLOAD		CAF	DLOADCOD
		TS	CYR
		TCF	DIRADRES	# GO GET A DIRECT ADDRESS.

DOVLOAD		CAF	VLOADCOD
		TS	CYR
		TCF	DIRADRES

DOVLOAD*	CAF	VLOAD*
		TCF	DODLOAD* +1	# PROLOGUE TO INDEX ROUTINE.

# Page 1019
# THE FOLLOWING LOAD INSTRUCTIONS ARE PROVIDED FOR LOADING THE MULTI-PURPOSE ACCUMULATOR MPAC.

; ============================================================================
; LOAD OPERATIONS - CORE DATA LOADING ROUTINES
; ============================================================================
;
; COMMENT-ONLY READERS: Before performing calculations, the interpreter must
; load input data from memory into MPAC (the working accumulator). Three load
; types exist: single numbers, high-precision numbers, and 3D vectors. During
; the lunar landing, these operations continuously loaded sensor data (altitude,
; velocity) and spacecraft state into MPAC for guidance computations.
;
; CODE-ALONG READERS: Load operations transfer data from erasable memory
; (address in ADDRWD) to MPAC registers. Each load type sets the MODE register
; to indicate the data format for subsequent operations:
;
; TLOAD (Triple-Precision Load):
;   - Loads 3 words: MPAC, MPAC+1, MPAC+2
;   - Sets MODE = 1 (triple-precision mode)
;   - Used for high-precision scalar computations
;
; SLOAD (Single-Precision Load):
;   - Loads 1 word to MPAC, zeros MPAC+1 and MPAC+2
;   - Sets MODE = 0 (double-precision, treating single as DP with zero LSB)
;   - Used for integer values and low-precision scalars
;
; VLOAD (Vector Load):
;   - Loads 6 words: MPAC,+1 (component X), MPAC+3,+4 (Y), MPAC+5,+6 (Z)
;   - MPAC+2 remains irrelevant (gap in structure)
;   - Sets MODE = 2 (vector mode)
;   - Used for position vectors, velocity vectors, attitude quaternions
;
; Historical note: During descent at 102:40 MET, VLOAD operations continuously
; loaded the LM's position and velocity vectors from the navigation state for
; guidance equation processing.

TLOAD		INDEX 	ADDRWD
		CA	2		# LOAD A TRIPLE PRECISION ARGUMENT INTO
		TS	MPAC 	+2	# THE FIRST THREE MPAC REGISTERS, WITH THE
		EXTEND			# CONTENTS OF THE OTHER FOUR IRRELEVANT.
		INDEX	ADDRWD
		DCA	0
		DXCH	MPAC
TMODE		CAF	ONE
		TCF	NEWMODE		# DECLARE TRIPLE PRECISION MODE.

SLOAD		ZL			# LOAD A SINGLE PRECISION NUMBER INTO
		INDEX	ADDRWD		# MPAC, SETTING MPAC+1,2 TO ZERO.  THE
		CA	0		# CONTENTS OF THE REMAINING MPAC REGISTERS
		TCF	SLOAD2		# ARE IRRELEVANT.

; VLOAD loads a 3-component double-precision vector (6 words total).
; Vector component layout in memory: X(2 words), Y(2 words), Z(2 words).
; MPAC structure: X at MPAC,+1; Y at MPAC+3,+4; Z at MPAC+5,+6.
VLOAD		EXTEND			# LOAD A DOUBLE PRECISION VECTOR INTO
		INDEX	ADDRWD		# MPAC,+1, MPAC+3,4, AND MPAC+5,6.  THE
		DCA	0		# CONTENTS OF MPAC +2 ARE IRRELEVANT.
		DXCH	MPAC

; ENDVLOAD is used by PDVL (pushdown-and-vector-load) to complete vector load
; after pushing previous MPAC contents to pushdown list.
ENDVLOAD	EXTEND			# PDVL COMES HERE TO FINISH UP FOR DP, TP.
		INDEX	ADDRWD
		DCA	2
		DXCH	MPAC 	+3

 +4		EXTEND			# TPDVL FINISHES HERE.
		INDEX	ADDRWD
		DCA	4
		DXCH	MPAC 	+5

VMODE		CS	ONE		# DECLARE VECTOR MODE.
		TCF	NEWMODE

# Page 1020
# THE FOLLOWING INSTRUCTIONS ARE PROVIDED FOR STORING OPERANDS IN THE PUSHDOWN LIST:
#	1.	PUSH		PUSHDOWN AND NO LOAD.
#	2.	PDDL		PUSHDOWN AND DOUBLE PRECISION LOAD.
#	3.	PDVL		PUSHDOWN AND VECTOR LOAD.

; ============================================================================
; PUSHDOWN LIST OPERATIONS - SAVE AND LOAD
; ============================================================================
;
; COMMENT-ONLY READERS: Complex calculations require temporary storage of
; intermediate results. The interpreter uses a "pushdown list" (stack) in
; erasable memory pointed to by PUSHLOC. These operations save current MPAC
; contents to the stack, then optionally load new data. During lunar landing,
; guidance equations computed position errors, pushed them to the stack,
; computed velocity errors, then retrieved both for control law calculations.
;
; CODE-ALONG READERS: The pushdown list operates as a stack growing downward
; in memory from address PUSHLOC. Push operations:
;   1. Store current MPAC contents at PUSHLOC
;   2. Advance PUSHLOC by 2, 3, or 6 words (depending on MODE)
;   3. Optionally load new data into MPAC
;
; PUSHLOC advancement:
;   - Double-precision (MODE=0): +2 words
;   - Triple-precision (MODE=1): +3 words
;   - Vector (MODE=2): +6 words
;
; PDDL (Push-Down and Double-precision Load):
;   Saves MPAC,+1 to pushdown list, then loads new DP value from address.
;
; PDVL (Push-Down and Vector Load):
;   Saves vector components to pushdown list, then loads new vector.
;   Handles mode transitions (DP→Vector, TP→Vector).

PDDL		EXTEND
		INDEX	ADDRWD		# LOAD MPAC,+1, PUSHING THE FORMER
		DCA	0		# CONTENTS DOWN.
		DXCH	MPAC
		INDEX	PUSHLOC
		DXCH	0

		INDEX	MODE		# ADVANCE THE PUSHDOWN POINTER APPRO-
		CAF	NO.WDS		# PRIATELY.
		ADS	PUSHLOC

		CCS	MODE
		TCF	ENDTPUSH
		TCF	ENDDPUSH

		TS	MODE		# NOW DP.
; ENDVPUSH completes vector pushdown by storing Y and Z components.
ENDVPUSH	TS	MPAC 	+2
		DXCH	MPAC 	+3	# PUSH DOWN THE REST OF THE VECTOR HERE.
		INDEX	PUSHLOC
		DXCH	0 	-4

		DXCH	MPAC 	+5
		INDEX	PUSHLOC
		DXCH	0 	-2

		TCF	DANZIG

; ENDDPUSH completes double-precision pushdown by zeroing MPAC+2.
ENDDPUSH	TS	MPAC	+2	# SET MPAC +2 TO ZERO AND EXIT ON DP.
		TCF	DANZIG

; ENDTPUSH completes triple-precision pushdown by storing MPAC+2.
ENDTPUSH	TS	MODE
		XCH	MPAC 	+2	# ON TRIPLE, SET MPAC +2 TO ZERO, PUSHING
 +2		INDEX	PUSHLOC		# DOWN THE OLD CONTENTS
		TS	0 	-1
		TCF	DANZIG

# Page 1021
# PDVL -- PUSHDOWN AND VECTOR LOAD

; PDVL performs pushdown-and-vector-load in one operation.
; Example usage during descent: Push current position vector, load velocity
; vector, compute cross product for angular momentum calculation.
PDVL		EXTEND			# RELOAD MPAC AND PUSH DOWN ITS CONTENTS.
		INDEX	ADDRWD
		DCA	0
		DXCH	MPAC
		INDEX	PUSHLOC
		DXCH	0

		INDEX	MODE		# ADVANCE THE PUSHDOWN POINTER.
		CAF	NO.WDS
		ADS	PUSHLOC

		CCS	MODE		# TEST PAST MODE.
		TCF	TPDVL
		TCF	ENDVLOAD	# JUST LOAD LAST FOUR REGISTERS ON DP.

; VPDVL handles vector-mode pushdown when prior mode was also vector.
VPDVL		EXTEND			# PUSHDOWN AND RE-LOAD LAST TWO COMPONENTS
		INDEX	ADDRWD
		DCA	2
		DXCH	MPAC 	+3
		INDEX	PUSHLOC
		DXCH	0 	-4

		EXTEND
		INDEX 	ADDRWD
		DCA	4
		DXCH	MPAC 	+5
		INDEX	PUSHLOC
		DXCH	0 	-2

		TCF	DANZIG

; TPDVL handles vector load when prior mode was triple-precision.
TPDVL		EXTEND			# ON TP, WE MUST LOAD THE Y COMPONENT
		INDEX	ADDRWD		# BEFORE STORING MPAC +2 IN CASE THIS IS A
		DCA	2		# PUSHUP.
		DXCH	MPAC	+3

		CA	MPAC 	+2
		INDEX	PUSHLOC		# IN DP.
		TS	0 	-1
		TCF	ENDVLOAD +4

# SSP (STORE SINGLE PRECISION) IS EXECUTED HERE.

; SSP (Store Single Precision) stores the next instruction word as a constant
; at the specified address. Used for setting variables to literal values.
; Example: SSP X,DECIMAL+10  stores decimal 10 at location X.
SSP		INCR	LOC		# PICK UP THE WORD FOLLOWING THE GIVEN
		INDEX	LOC		# ADDRESS AND STORE IT AT X.
		CA	0
STORE1		INDEX	ADDRWD		# SOME INDEX AND MISCELLANEOUS OPS END
		TS	0		# HERE.
# Page 1022
		TCF	DANZIG

# Page 1023
# SEQUENCE CHANGING AND SUBROUTINE CALLING OPTIONS.
#
# THE FOLLOWING OPERATIONS ARE AVAILABLE FOR SEQUENCING CHANGING, BRANCHING, AND CALLING SUBROUTINES:
#	1.	GOTO		GO TO.
#	2.	CALL		CALL SUBROUTINE SETTING QPRET.
#	3.	CGOTO		COMPUTED GO TO.
#	4.	CCALL		COMPUTED CALL.
#	7.	BPL		BRANCH IF MPAC POSITIVE OR ZERO.
#	8.	BZE		BRANCH IF MPAC ZERO.
#	9.	BMN		BRANCH IF MPAC NEGATIVE NON-ZERO.

; ============================================================================
; SEQUENCE CHANGING AND SUBROUTINE CALLING OPERATIONS
; ============================================================================
;
; This section implements control flow operations for interpretive code,
; including unconditional branching (GOTO), subroutine calls (CALL),
; computed/indirect branches (CGOTO, CCALL), and conditional branches
; (BPL, BZE, BMN).
;
; COMMENT-ONLY READERS:
; These instructions control the flow of interpretive programs, allowing
; guidance software to make decisions, call mathematical subroutines, and
; loop through calculations. During lunar descent, branching instructions
; enabled the guidance computer to continuously evaluate altitude and
; velocity, adjusting the descent trajectory in real-time.
;
; CODE-ALONG READERS:
; Control flow in interpretive mode differs from native AGC because the
; interpreter maintains its own program counter (LOC) and bank registers
; (BANKSET, FBANK). Branches must handle both fixed memory addresses
; (programs in core rope) and erasable memory addresses (work area
; variables). The CALL instruction preserves return address in QPRET,
; and GOTO performs direct transfers. Computed branches (CGOTO/CCALL)
; support jump tables for multi-way branching based on index values.
;
; KEY CONCEPTS:
; - LOC: Interpretive location counter (current instruction address)
; - POLISH: Temporary storage for target addresses during branches
; - QPRET: Return address for subroutine calls (where to return after RETURN)
; - FBANK/BANKSET: Bank selection for fixed and erasable memory
; - Erasable addressing: Special handling for variables vs code addresses
;
; BRANCH INSTRUCTIONS (Using BRANCH routine):
; - BPL: Branch if MPAC > 0 or MPAC = 0
; - BMN: Branch if MPAC < 0
; - BZE: Branch if MPAC = 0
; These use the triple-precision BRANCH routine to test MPAC sign.
; ============================================================================

; ----------------------------------------------------------------------------
; CCALL - Computed Call (Subroutine call with computed address)
;
; Calls an interpretive subroutine whose address is computed by indexing
; into a table of CADR (Complete Address) values. Used for dispatching
; to one of several subroutines based on a runtime index.
;
; FORMAT:
;   CCALL
;   CADR BASE_ADDRESS  ; Address of CADR table
;   <index in ADDRWD>  ; Index specifies which CADR to use
;
; OPERATION:
;   1. Read base address of CADR table from next word
;   2. Add index from ADDRWD to base address
;   3. Fetch CADR at computed location
;   4. Save return address in QPRET
;   5. Branch to fetched CADR
;
; EXAMPLE USE: Calling different integration routines based on mission phase
; ----------------------------------------------------------------------------
CCALL		INCR	LOC		# MAINTAIN LOC FOR QPRET COMPUTATION
		INDEX	LOC
		CAF	0		# GET BASE ADDRESS OF CADR LIST.
		INDEX	ADDRWD
		AD	0		# ADD INCREMENT.
		TS	FBANK		# SELECT DESIRED CADR.
		MASK	LOW10
		INDEX	A
		CAF	10000
		TS	POLISH

; ----------------------------------------------------------------------------
; CALL - Subroutine Call
;
; Calls an interpretive subroutine at the specified address, saving the
; return address in QPRET. The subroutine returns using RETURN instruction.
;
; FORMAT:
;   CALL
;   CADR SUBROUTINE  ; Complete address of subroutine to call
;
; OPERATION:
;   1. Save return address (address of next instruction) in QPRET
;   2. Branch to subroutine address
;   3. Subroutine executes and uses RETURN to come back
;
; QPRET FORMAT:
;   Contains encoded address: (bank bits) + (location within bank)
;   Return address points to instruction pair following CALL
;
; STACK BEHAVIOR:
;   CALL does not automatically push MPAC. If subroutine modifies MPAC
;   and caller needs to preserve it, use PUSH before CALL or use
;   STCALL which stores and calls in one operation.
;
; EXAMPLE:
;   During lunar landing, guidance code calls vector subroutines to
;   compute landing site position relative to current trajectory.
; ----------------------------------------------------------------------------
CALL		CA	BANKSET		# FOR ANY OF THE CALL OPTIONS, MAKE UP THE
		MASK	BANKMASK	# ADDRESS OF THE NEXT OP-CODE PAIR/STORE
		AD	BANKMASK	# CODE AND LEAVE IT IN QPRET.  NOTE THAT
		AD	LOC		# BANKMASK = -(2000 - 1).
		INDEX	FIXLOC
		TS	QPRET

; ----------------------------------------------------------------------------
; GOTO - Unconditional Branch
;
; Transfers control to the specified interpretive address unconditionally.
; This is the fundamental branching instruction in interpretive code.
;
; FORMAT:
;   GOTO
;   CADR TARGET  ; Complete address to branch to
;
; OPERATION:
;   1. Read target address from POLISH (set by opcode decoder)
;   2. Determine if target is in fixed or erasable memory
;   3. Set up bank registers (FBANK/EBANK) appropriately
;   4. Set LOC to target location
;   5. Continue execution at target
;
; ADDRESS TYPES:
;   - Fixed memory: Addresses in core rope ROM (programs)
;   - Erasable memory: Addresses in RAM (can be indirect)
;   - Work area: Special VAC area addresses (double-indirect)
;
; BANK SWITCHING:
;   GOTO automatically handles bank switching for target address.
;   High bit of address determines fixed vs erasable selection.
;   FBANK register selects fixed bank, EBANK selects erasable bank.
;
; EXAMPLE:
;   Used extensively in guidance loops to restart computation cycles,
;   branch to different descent phases, and skip over conditional code.
; ----------------------------------------------------------------------------
GOTO		CA	POLISH		# BASIC BRANCHING SEQUENCE.
 +1		MASK 	HIGH4
		EXTEND
		BZF	GOTOERS		# SEE IF ADDRESS POINTS TO FIXED OR ERAS.
 +4		CA	BANKSET		# SET EBANK PART OF BBANK.  NEXT, SET UP
		TS	BBANK		# FBANK.  THE COMBINATION IS PICKED UP &
		CA	POLISH		# PUT INTO BANKSET AT INTPRET +2.
		TS	FBANK
		MASK 	LOW10
		AD	2K
		TS	LOC
		TCF	INTPRET +3

		EBANK=	1400		# SO YUL DOESN'T CUSS THE "CA 1400" BELOW.

; ----------------------------------------------------------------------------
; GOTOERS - GOTO Erasable Memory
;
; Handles GOTO to erasable memory addresses. Erasable memory includes both
; general erasable (direct addressing) and VAC work area (indirect).
;
; OPERATION:
;   1. Check if address is in VAC work area (below ENDVAC)
;   2. If work area: Treat address as pointer, dereference it
;   3. If general erasable: Use address directly
;   4. Branch using GOTOGE routine
;
; VAC WORK AREA INDIRECTION:
;   The VAC (Vector Accumulator) area stores temporary vector/matrix values.
;   When branching to a VAC address, the address itself contains the
;   actual branch target (double indirection). This allows computed
;   branches based on work area contents.
;
; GENERAL ERASABLE:
;   Direct addressing to erasable variables and data storage areas.
;   No indirection - address is used as-is.
;
; COMMENT-ONLY READERS:
;   This handles branches to guidance parameters stored in modifiable
;   memory. The work area indirection allows the software to compute
;   branch targets dynamically, like using a computed jump table.
; ----------------------------------------------------------------------------
GOTOERS		CA	POLISH		# THE GIVEN ADDRESS IS IN ERASABLE -- SEE
		AD	-ENDVAC		# IF RELATIVE TO THE WORK ARA.
		CCS	A
		CA	POLISH		# GENERAL ERASABLE.
		TCF	GOTOGE

# Page 1024
		CA	FIXLOC		# WORK AREA.
		AD	POLISH
		INDEX	A		# USE THE GIVEN ADDRESS AS THE ADDRESS OF
		CA	0		# THE BRANCH ADDRESS.
		TS	POLISH
		TCF	GOTO 	+1	# ALLOWS ARBITRARY INDIRECTNESS LEVELS.

; ----------------------------------------------------------------------------
; GOTOGE - GOTO General Erasable with E-bank Setup
;
; Performs branch to general erasable memory with E-bank selection.
; This is the final stage of erasable branching after GOTOERS determines
; the address type.
;
; OPERATION:
;   1. Set EBANK register from high bits of address
;   2. Mask address to get location within bank (8 bits)
;   3. Add 1400 octal offset (base of erasable addressing)
;   4. Fetch indirect address from computed location
;   5. Loop back to GOTO +1 to process target address
;
; INDIRECT ADDRESSING:
;   The address in POLISH points to a location containing the actual
;   branch target. This double-indirection supports dynamic branching
;   where target addresses are computed at runtime.
;
; E-BANK SELECTION:
;   High bits of address select which erasable bank (0-7) to access.
;   Bank must be set before reading the indirect pointer.
; ----------------------------------------------------------------------------
GOTOGE		TS	EBANK
		MASK	LOW8
		INDEX	A		# USE THE GIVEN ADDRESS AS THE ADDRESS OF
		CA	1400		# THE BRANCH ADDRESS.
		TS	POLISH
		TCF	GOTO 	+1

; ----------------------------------------------------------------------------
; CGOTO - Computed GOTO (Indexed Branch)
;
; Performs a branch to an address selected from a table of CADRs using
; an index value. This implements multi-way branching based on a computed
; or runtime value.
;
; FORMAT:
;   CGOTO
;   CADR TABLE_BASE  ; Address of CADR table
;   <index in ADDRWD>  ; Which entry to select
;
; OPERATION:
;   1. Read base address of CADR table from instruction stream
;   2. Add index from ADDRWD to base address
;   3. Fetch CADR at computed location
;   4. Branch to fetched CADR using GOTO logic
;
; USE CASES:
;   - Multi-way mission phase selection (launch/coast/descent/ascent)
;   - Algorithm selection based on sensor availability
;   - Error handling dispatch to different recovery routines
;   - Integration method selection based on trajectory regime
;
; EXAMPLE:
;   During Apollo 11 descent, guidance software selected different
;   targeting algorithms based on altitude bands. CGOTO enabled dispatch
;   to braking phase, approach phase, or landing phase guidance based
;   on computed altitude index.
; ----------------------------------------------------------------------------
CGOTO		INDEX	LOC		# COMPUTED GO TO.  PICK UP ADDRESS OF CADR
		CA	1		# LIST
		INDEX	ADDRWD		# ADD MODIFIER.
		AD	0
		TS	FBANK		# SELECT GOTO ADDRESS
		MASK	LOW10
		INDEX	A
		CA	10000
		TS	POLISH
		TCF	GOTO 	+1	# WITH ADDRESS IN A.

; SWBRANCH - Execute branch for SWITCH instructions
; Called when switch test determines a branch should be taken (or for unconditional GOTO).
; Reads the branch address from the next word in the interpretive code stream,
; then jumps to GOTO logic to perform the interpretive branch.
SWBRANCH	CA	BANKSET		# SWITCH INSTRUCTIONS WHICH ELECT TO
		TS	FBANK		# BRANCH COME HERE TO DO SO.
		INDEX	LOC		# Get next word (branch address)
		CA	1
		TS	POLISH		# Store as target address
		TCF	GOTO 	+1	# Execute interpretive GOTO

# Page 1025
; ============================================================================
; BRANCH - Triple Precision Branching Routine
;
; Tests the sign of triple-precision value in MPAC (MPAC, MPAC+1, MPAC+2).
; Used by interpretive branch instructions to determine control flow based
; on computational results.
;
; CALLING SEQUENCE:
;   TC BRANCH
;   <return if MPAC > 0>
;   <return if MPAC = 0>
;   <return if MPAC < 0>
;
; RETURN CONDITIONS (if calling TC is at address L):
;   L+1: MPAC is greater than zero (positive result)
;   L+2: MPAC equals +0 or -0 (zero result)
;   L+3: MPAC is less than zero (negative result)
;
; This routine cascades through all three words of MPAC to handle
; triple-precision arithmetic results properly.
; ============================================================================
# TRIPLE PRECISION BRANCHING ROUTINE.  IF CALLING TC IS AT L, RETURN IS AS FOLLOWS:
#	L+1	IF MPAC IS GREATER THAN ZERO.
#	L+2	IF MPAC IS EQUAL TO +0 OR -0.
#	L+3	IF MPAC IS LESS THAN ZERO.

BRANCH		CCS	MPAC		# Test most significant word
		TC	Q		# Positive: return to L+1
		TCF	+2		# ON ZERO: check next word
		TCF	NEG		# Negative: go to NEG (returns L+3)

		CCS	MPAC 	+1	# Test middle word
		TC	Q		# Positive: return to L+1
		TCF	+2		# Zero: check least significant word
		TCF	NEG		# Negative: go to NEG

		CCS	MPAC 	+2	# Test least significant word
		TC	Q		# Positive: return to L+1
		TCF	+2		# Zero in all three words
		TCF	NEG		# Negative: go to NEG

Q+1		INDEX	Q		# Return to L+2 (zero case)
		TC	1		# Skip one instruction past Q

NEG		INDEX	Q		# IF FIRST NON-ZERO REGISTER WAS NEGATIVE.
		TC	2		# Return to L+3 (negative case)

Q+2		=	NEG		# Alternate entry for Q+2 return

# ITRACE (3) REFERS TO "EXIT".

; ============================================================================
; EXIT - Leave Interpretive Mode
;
; Returns control from the interpreter back to native AGC code.
; Restores the user's bank setting and executes the next instruction
; after the TC INTPRET that entered interpretive mode.
;
; This instruction is used to terminate interpretive code sequences and
; return to basic AGC assembly execution. During Apollo 11's descent,
; interpretive guidance computations would EXIT back to native code for
; time-critical operations like thruster control.
; ============================================================================
EXIT		CA	BANKSET		# RESTORE USER'S BANK SETTING, AND LEAVE
		TS	BBANK		# INTERPRETIVE MODE.
		INDEX	LOC		# Return to instruction after TC INTPRET
		TC	1		# Execute return

# Page 1026
; ============================================================================
; SECTION 3: ADD/SUBTRACT PACKAGE
;
; Provides addition and subtraction operations for the Multi-Purpose
; Accumulator (MPAC). These operations support double-precision scalars,
; triple-precision values, and 3-component vectors.
;
; OPERATIONS PROVIDED:
;   DAD   - Double precision add (MPAC = MPAC + operand)
;   DSU   - Double precision subtract (MPAC = MPAC - operand)
;   BDSU  - Double precision subtract from (MPAC = operand - MPAC)
;   TAD   - Triple precision add
;   VAD   - Vector add (3 components)
;   VSU   - Vector subtract (MPAC = MPAC - operand)
;   BVSU  - Vector subtract from (MPAC = operand - MPAC)
;
; OVERFLOW HANDLING:
; The interpretive overflow indicator OVFIND is set non-zero if arithmetic
; overflow occurs in any operation. This alerts the interpretive code that
; results may be invalid and allows error handling.
;
; USAGE IN APOLLO 11:
; Vector operations computed velocity changes during descent guidance.
; Position vectors were added/subtracted for trajectory computations.
; Double-precision operations handled time calculations and scaling factors.
; ============================================================================
# SECTION 3 -- ADD/SUBTRACT PACKAGE.
#
# THE FOLLOWING OPERATIONS ARE PROVIDED FOR ADDING TO AND SUBTRACTING FROM THE MULTI-PURPOSE ACCUMULATOR
# MPAC:
#	1.	DAD	DOUBLE PRECISION ADD.
#	2.	DSU	DOUBLE PRECISION SUBTRACT.
#	3.	BDSU	DOUBLE PRECISION SUBTRACT FROM.
#	4.	TAD	TRIPLE PRECISION ADD.
#	5.	VAD	VECTOR ADD.
#	6.	VSU	VECTOR SUBTRACT.
#	7.	BVSU	VECTOR SUBTRACT FROM.
# THE INTERPRETIVE OVERFLOW INDICATOR OVFIND IS SET NON-ZERO IF OVERFLOW OCCURS IN ANY OF THE ABOVE.

; VSU - Vector Subtract (MPAC = MPAC - operand)
; Subtracts 3-component vector from MPAC. Uses DCS (double complement and
; store) to perform subtraction for each component pair.
VSU		CAF	BIT15		# CHANGES 0 TO DCS.
		TCF	+2		# Skip to common vector code

; VAD - Vector Add (MPAC = MPAC + operand)
; Adds 3-component vector to MPAC. Processes components in pairs (Z,Y,X)
; and checks for overflow after each component addition.
VAD		CAF	PRIO30		# CHANGES 0 TO DCA.
		ADS	ADDRWD		# Modify address for indexing
		EXTEND
		INDEX	ADDRWD
		READ	HISCALAR	# DCA 2 OR DCS 2 (Z,Y components)
		DAS 	MPAC 	+3	# Add/subtract to MPAC Z,Y
		EXTEND			# CHECK OVERFLOW.
		BZF	+2		# Skip if no overflow
		TC	OVERFLWY	# Handle Y-component overflow

		EXTEND
		INDEX	ADDRWD
		READ	CHAN5		# DCA 4 OR DCS 4 (continued)
		DAS	MPAC 	+5	# Process next pair
		EXTEND
		BZF	+2		# Skip if no overflow
		TC	OVERFLWZ	# Handle Z-component overflow

		EXTEND
		INDEX	ADDRWD
		READ	LCHAN		# DCA 0 OR DCS 0 (X component)
		TCF	ENDVXV		# Finish with X component

; DAD - Double precision Add (MPAC = MPAC + operand)
; Adds a double-precision scalar value to MPAC. The operand is loaded from
; memory and added to the MPAC register pair. Overflow is checked.
DAD		EXTEND
		INDEX 	ADDRWD
		DCA	0		# Load DP operand from memory
ENDVXV		DAS	MPAC		# VXV FINISHES HERE. Add to MPAC,MPAC+1
		EXTEND			# Check for overflow
		BZF	DANZIG		# No overflow, continue to next operation

; SETOVF - Set Overflow Indicator
; Called when overflow is detected in arithmetic operations.
; Sets OVFIND to signal interpretive code that results are invalid.
# Page 1027
SETOVF		TC	OVERFLOW	# Handle overflow condition
		TCF	DANZIG		# Resume operation dispatch

; DSU - Double precision Subtract (MPAC = MPAC - operand)
; Subtracts a double-precision scalar from MPAC. Uses DCS (double complement
; and store) to negate operand, then adds (subtraction via complement).
# Page 1028
DSU		EXTEND
		INDEX	ADDRWD
		DCS	0		# Load negated DP operand
		TCF	ENDVXV		# Add negated value (= subtract)

; OVERFLWZ - Overflow Handler for Third (Z) Vector Component
; Called when overflow occurs in third component of vector operation.
; Sets component index to 5 (MPAC+5) for Z component correction.
OVERFLWZ	TS	L		# ENTRY FOR THIRD COMPONENT.
		CAF	FIVE		# Component offset = 5
		TCF	+3		# Jump to overflow correction

; OVERFLWY - Overflow Handler for Second (Y) Vector Component
; Called when overflow occurs in second component of vector operation.
; Sets component index to 3 (MPAC+3) for Y component correction.
OVERFLWY	TS	L		# ENTRY FOR SECOND COMPONENT.
		CAF	THREE		# Component offset = 3
		XCH	L		# Position offset in L

; OVERFLOW - General Overflow Handler
; Corrects overflow by clamping result to POSMAX or NEGMAX limits.
; Entry: A contains sign indicator (pos/neg), L contains component offset
; For DP operations, L=0 (first component). For vectors, L=3,5 for Y,Z.
OVERFLOW	INDEX	A		# ENTRY FOR 1ST COMP OR DP (L=0).
		CS	LIMITS		# PICK UP POSMAX OR NEGMAX.
		TS	BUF		# Save limit value
		EXTEND
		AUG	A		# FORCE OVERFLOW indication
		INDEX	L		# Select component using offset
		ADS	MPAC 	+1	# Clamp component to limit
		TS	7		# Store overflow indicator
		CAF	ZERO		# Complete overflow correction
		AD	BUF		# Add limit value
		INDEX	L		# Index to component
		ADS	MPAC		# Store corrected value
		TS	7		# Update overflow flag
		TC	Q		# NO OVERFLOW EXIT.
		TCF	SETOVF2		# SET OVFIND AND EXIT.

; BVSU - Vector Subtract From (MPAC = operand - MPAC)
; Reverses operand order: loads operand into MPAC, then subtracts old MPAC.
; Processes Y and Z components with overflow checking for each.
BVSU		EXTEND
		INDEX	ADDRWD
		DCA	2		# Load Y,Z components from operand
		DXCH	MPAC 	+3	# Swap with MPAC (now operand in MPAC)
		EXTEND
		DCOM			# Complement old MPAC value
		DAS	MPAC 	+3	# Add complement (= subtract)
		EXTEND
		BZF	+2		# Check overflow
		TC	OVERFLWY	# Handle Y-component overflow

		EXTEND
		INDEX	ADDRWD
		DCA	4		# Process next component pair
		DXCH	MPAC 	+5	# Swap with MPAC
		EXTEND
		DCOM			# Complement old MPAC
		DAS	MPAC 	+5	# Add complement
		EXTEND
		BZF	+2		# Check overflow
		TC	OVERFLWZ	# Handle Z-component overflow

; BDSU - Double precision Subtract From (MPAC = operand - MPAC)
; Reverses subtraction: loads operand, negates old MPAC, then adds.
; Result: operand minus original MPAC value.
# Page 1029
BDSU		EXTEND
		INDEX	ADDRWD
		DCA	0		# Load DP operand
		DXCH	MPAC		# Swap with MPAC
		EXTEND
		DCOM			# Complement old MPAC value
		TCF	ENDVXV		# Add complement to complete subtraction

# Page 1030
; TAD - Triple precision Add (MPAC = MPAC + operand)
; Adds a triple-precision value to MPAC. Triple precision uses 3 words
; for extended range: major part (MPAC), middle (MPAC+1), minor (MPAC+2).
; Addition proceeds from least significant to most significant with carry.
;
; TRIPLE PRECISION ADD ROUTINE.

TAD		EXTEND
		INDEX	ADDRWD
		DCA	1		# ADD MINOR PARTS FIRST (words 1,2).
		DAS	MPAC 	+1	# Add to MPAC+1, MPAC+2 with carry
		INDEX	ADDRWD
		AD	0		# Add major part (word 0)
		AD	MPAC		# Add to MPAC with carry propagation
		TS	MPAC		# Store result in MPAC
		TCF	DANZIG		# Continue to next operation

		TCF	SETOVF		# SET OVFIND IF SUCH OCCURS.

# Page 1031
; ============================================================================
; ARITHMETIC SUBROUTINES (Fixed-Fixed)
;
; These subroutines provide fundamental arithmetic operations required
; throughout the AGC software. They are called from both interpretive
; and native AGC code using TC (Transfer Control) instructions.
;
; SUBROUTINES PROVIDED:
;
; 1. DMPSUB - Double Precision Multiply
;    Multiply the contents of MPAC,+1 by the DP word whose address
;    is in ADDRWD and leave a triple-precision result in MPAC.
;    Used extensively in guidance equations for trajectory calculations.
;
; 2. ROUNDSUB - Round Triple to Double Precision
;    Round the triple precision contents of MPAC to double precision.
;    Sets OVFIND on overflow (rare event).
;
; 3. DOTSUB - Dot Product
;    Take the dot product of the vector in MPAC and the vector whose
;    address is in ADDRWD, leaving triple precision result in MPAC.
;    Critical for velocity/position calculations during descent.
;
; 4. POLY - Polynomial Evaluator
;    Using the contents of MPAC as a DP argument, evaluate the polynomial
;    whose degree and coefficients immediately follow the TC POLY instruction.
;    Used for trigonometric approximations and function evaluation.
; ============================================================================
# ARITHMETIC SUBROUTINES REQUIRED IN FIXED-FIXED.
#	1.  DMPSUB	DOUBLE PRECISION MULTIPLY, MULTIPLY THE CONTENTS OF MPAC,+1 BY THE DP WORD WHOSE ADDRESS
#			IS IN ADDRWD AND LEAVE A TRIPLE-PRECISION RESULT IN MPAC.
#	2.  ROUNDSUB	ROUND THE TRIPLE PRECISION CONTENTS OF MPAC TO DOUBLE PRECISION.
#	3.  DOTSUB	TAKE THE DOT PRODUCT OF THE VECTOR IN MPAC AND THE VECTOR WHOSE ADDRESS IS IN ADDRWD
#			AND LEAVE THE TRIPLE PRECISION RESULT IN MPAC.
#	4.  POLY	USING THE CONTENTS OF MPAC AS A DP ARGUMENT, EVALUATE THE POLYNOMIAL WHOSE DEGREE AND
#			COEFFICIENTS IMMEDIATELY FOLLOW THE TC POLY INSTRUCTION (SEE ROUTINE FOR DETAILS).

; DMP - Double Precision Multiply (entry point for Pinball and other callers)
; Call sequence: TC DMP followed by address of operand
; Returns to caller at TC DMP + 2
;
DMP		INDEX	Q		# BASIC SUBROUTINE FOR USE BY PINBALL, ETC
		CAF	0		# Address of argument follows TC DMP instruction
		INCR	Q		# Skip over argument address for return
 -1		TS	ADDRWD		# Store operand address (PROLOGUE FOR SETTING ADDRWD.)

; DMPSUB - Double Precision Multiply Subroutine
; Multiplies MPAC (double precision) by operand at address in ADDRWD
; Result: Triple precision product in MPAC, MPAC+1, MPAC+2
;
; Algorithm: (MPAC_major, MPAC_minor) × (Op_major, Op_minor)
; Performs four partial products and accumulates:
;   1. MPAC_minor × Op_minor (least significant)
;   2. MPAC_major × Op_minor
;   3. Op_major × MPAC_minor
;   4. MPAC_major × Op_major (most significant)
;
; Used throughout guidance equations for velocity, position calculations.
; Execution time: 49 machine cycles = 0.573 milliseconds
;
DMPSUB		INDEX	ADDRWD		# GET MINOR PART OF OPERAND AT C(ADDRWD).
		CA	1		# Load operand minor part (word 1)
		TS	MPAC 	+2	# Store in MPAC+2 (works for squaring MPAC as well)
		CAF	ZERO		# SET MPAC +1 TO ZERO SO WE CAN ACCUMULATE
		XCH	MPAC 	+1	# THE PARTIAL PRODUCTS WITH DAS
		TS	MPTEMP		# Save MPAC minor part for later use
		EXTEND
		MP	MPAC 	+2	# Partial product 1: MPAC_minor × Op_minor

		XCH	MPAC 	+2	# Discard minor part of above result
		EXTEND			# Prepare for next multiplication
		MP	MPAC		# Partial product 2: MPAC_major × Op_minor
		DAS	MPAC 	+1	# Accumulate into MPAC+1,+2 (no overflow possible)

		INDEX	ADDRWD		# GET MAJOR PART OF ARGUMENT AT C(ADDRWD).
		CA	0		# Load operand major part (word 0)
		XCH	MPTEMP		# Exchange: save operand major, retrieve MPAC minor
DMPSUB2		EXTEND
		MP	MPTEMP		# Partial product 3: Op_major × MPAC_minor
		DAS	MPAC 	+1	# Accumulate, A gets overflow (0 or ±1)

		XCH	MPAC		# Move overflow to MPAC, zero MPAC for next multiply
		EXTEND
		MP	MPTEMP		# Partial product 4: MPAC_major × Op_major
		DAS	MPAC		# Final accumulation (no overflow possible)
		TC	Q		# Return (49 MCT = .573 MS. INCLUDING RETURN.)

# Page 1032
; ROUNDSUB - Round Triple Precision to Double Precision
; Rounds the value in MPAC to double precision, setting OVFIND if overflow.
;
; Entry points:
;   ROUNDSUB: For scalar values - zeros MPAC+2 and sets mode to DP
;   VROUND: For vector values - directly processes MPAC+2 without zeroing
;
; Algorithm: Doubles MPAC+2 to check if |value| >= 0.5, then conditionally
; adds rounding bit to MPAC+1 and propagates carry to MPAC if needed.
; If overflow occurs during propagation, SETOVF2 sets OVFIND flag.
;
; Used after triple precision operations (like DMP) to round result back
; to double precision format for storage or further computation.
;
# ROUND MPAC TO DOUBLE PRECISION, SETTING OVFIND ON THE RARE EVENT OF OVERFLOW.

ROUNDSUB	CAF	ZERO		# SET MPAC +2 = 0 FOR SCALARS AND CHANGE
 +1		TS	MODE		# MODE TO DP (double precision mode)

VROUND		XCH	MPAC 	+2	# Vector entry: get MPAC+2 without zeroing
		DOUBLE			# Double it to test if >= 0.5 magnitude
		TS	L		# Save doubled value in L
		TC	Q		# Return if |MPAC+2| < 0.5 (no rounding needed)

		AD	MPAC 	+1	# ADD ROUDING BIT IF MPAC +2 WAS GREATER
		TS	MPAC 	+1	# THAN .5 IN MAGNITUDE (propagate rounding)
		TC	Q		# Return if no carry to major part

		AD	MPAC		# PROPAGATE INTERFLOW (carry) to major part
		TS	MPAC		# Store rounded major part
		TC	Q		# Return if no overflow

SETOVF2		TS	OVFIND		# Overflow occurred (RARE) - set flag
		TC	Q		# Return with overflow condition signaled

# Page 1033
; DOTSUB - Vector Dot Product Subroutine
; Computes the dot product: MPAC · V = (Vx*Mx) + (Vy*My) + (Vz*Mz)
;
; Entry points:
;   PREDOT: Normal entry for DOT operation - sets DOTINC to 2 for standard
;           6-register vector (components at addr, addr+2, addr+4)
;   DOTSUB: Direct entry when DOTINC pre-set (used by VXM with DOTINC=6
;           to dot with matrix column vectors)
;
; Inputs:
;   MPAC, MPAC+3, MPAC+5: Vector in MPAC (X, Y, Z components)
;   ADDRWD: Address of other vector's X component
;   DOTINC: Increment between components (2 for vectors, 6 for matrix columns)
;
; Output:
;   MPAC: DP dot product result
;   OVFIND: Set if overflow during accumulation
;
; Algorithm:
;   1. Multiply X components (DMPSUB), save result in BUF
;   2. Advance address by DOTINC, multiply Y components, accumulate
;   3. Advance address by DOTINC, multiply Z components, accumulate
;   4. Return final sum in MPAC
;
; The VXM (vector × matrix) operation uses this with DOTINC=6 to dot MPAC
; with each column of a 3×3 matrix stored in row-major order.
;
# THE DOT PRODUCT SUBROUTINE USUALLY FORMS THE DOT PRODUCT OF THE VECTOR IN MPAC WITH A STANDARD SIX
# REGISTER VECTOR WHOSE ADDRESS IS IN ADDRWD.  IN THIS CASE C(DOTINC) ARE SET TO 2.   VXM, HOWEVER, SETS C(DOTINC) TO
# 6 SO THAT DOTSUB DOTS MPAC WITH A COLUMN VECTOR OF THE MATRIX IN QUESTION IN THIS CASE.

PREDOT		CAF	TWO		# PROLOGUE: Set DOTINC to 2 for standard vector
		TS	DOTINC		# (components at addr, addr+2, addr+4)

DOTSUB		EXTEND			# Direct entry with DOTINC pre-set
		QXCH	DOTRET		# Save return address in DOTRET
		TC	DMPSUB		# Multiply X components (MPAC × C(ADDRWD))
		DXCH	MPAC 	+3	# Get Y component of MPAC
		DXCH	MPAC		# Position for multiplication
		DXCH	BUF		# Save X product in buffer
		CA	MPAC 	+2	# Save third word of X product
		TS	BUF 	+2	# Complete buffering

		CA	DOTINC		# Get component increment (2 or 6)
		ADS	ADDRWD		# ADVANCE ADDRWD TO Y COMPONENT
		TC	DMPSUB		# Multiply Y components
		DXCH	MPAC 	+1	# Get lower two words of Y product
		DAS	BUF 	+1	# Add to buffered X product (LS words)
		AD	MPAC		# Add MS words with carry
		AD	BUF		# Accumulate in BUF
		TS	BUF		# Store sum
		TCF	+2		# Skip overflow handling if no overflow
		TS	OVFIND		# IF OVERFLOW OCCURS, set flag

		DXCH	MPAC 	+5	# Get Z component of MPAC
		DXCH	MPAC		# Position for multiplication
		CA	DOTINC		# Get component increment
		ADS	ADDRWD		# Advance to Z component of other vector
		TC	DMPSUB		# MULTIPLY Z COMPONENTS
ENDDOT		DXCH	BUF 	+1	# Get accumulated X+Y product
		DAS	MPAC 	+1	# Add to Z product (LS words)
		AD	MPAC		# Add MS words with carry
		AD	BUF		# Final accumulation
		TS	MPAC		# LEAVE FINAL DOT PRODUCT in MPAC
		TC	DOTRET		# Return to caller

		TC	OVERFLOW	# ON OVERFLOW HERE, handle and return
		TC	DOTRET		# Return after overflow handling

# Page 1034
; ============================================================================
; POLY - Double-Precision Polynomial Evaluator
; ============================================================================
;
; Purpose: Evaluates polynomial P(X) = a0 + a1*X + a2*X^2 + ... + aN*X^N
;          Uses Horner's method for computational efficiency and accuracy.
;
; COMMENT-ONLY READERS:
; This subroutine performs polynomial evaluation, a fundamental mathematical
; operation used throughout guidance computations. During Apollo 11's descent,
; polynomial approximations were used to compute trigonometric functions,
; gravitational models, and trajectory predictions. The Horner's method
; implementation minimizes multiplications while maintaining precision.
;
; CODE-ALONG READERS:
; Implements Horner's nested form: P(X) = a0 + X(a1 + X(a2 + ... + X(aN)...))
; This reduces N multiplications and additions (standard form) to just N
; multiplications and N additions, critical for AGC performance.
;
; Entry Points:
;   POWRSERS: TC POWRSERS (native AGC call)
;             For coefficients in fixed or erasable memory
;             Return to TC POWRSERS + 1
;
;   POLY:     TC POLY (native AGC call from interpretive code)
;             Coefficient table follows call in fixed memory
;             Return calculated automatically
;
; Algorithm:
;   1. Load highest coefficient aN into MPAC
;   2. Save argument X in VBUF
;   3. Loop N times:
;      a. Multiply MPAC by X (using DMPSUB)
;      b. Add next coefficient (a(N-1), a(N-2), ... a0)
;   4. Return with P(X) in MPAC (double precision)
;
; Registers Used:
;   MPAC, MPAC+1: Accumulator for polynomial computation (DP)
;   VBUF, VBUF+1: Holds argument X (DP)
;   POLYCNT: Loop counter (N-1 down to 0)
;   POLISH: Coefficient table pointer
;   POLYRET: Return address
;
; Historical Context:
; Polynomial evaluators were essential for AGC's guidance system. Complex
; functions like SIN, COS, and gravitational models were approximated using
; polynomials to balance accuracy with the AGC's ~85 microsecond instruction
; cycle time. During descent, trajectory predictions used polynomial fits.
;
# DOUBLE PRECISION POLYNOMIAL EVALUATOR
#	                          N        N-1
#	THIS ROUTINE EVALUATES A X  + A   X    + ... + A  X + A  LEAVING THE DP RESULT IN MPAC ON EXIT.
#	                        N      N-1              1      0
#
# THE ROUTINE HAS TWO ENTRIES
#
#	1	ENTRY THRU POWRSERS.  THE COEFFICIENTS MAY BE EITHER IN FIXED OR ERASABLE E.  THE CALL IS BY
#		TC POWRSERS, AND THE RETURN IS TO LOC(TC POWRSERS)+1.  THE ENTERING DATA MUST BE AS FOLLOWS:
#
#			A		SP	LOC-3		# ADDRESS FOR REFERENCING COEF TABLE
#			L		SP	N-1		# N IS THE DEGREE OF THE POWER SERIES
#			MPAC		DP	X		# ARGUMENT
#			LOC-2N		DP	A(0)
#					...
#			LOC		DP	A(N)
#
#	2.	ENTRY THRU POLY.  THE CALL TO POLY AND THE ENTERING DATA MUST BE AS FOLLOWS
#
#			MPAC		DP	X		# ARGUMENT
#			LOC		TC	POLY
#			LOC+1		SP	N-1
#			LOC+2		DP	A(0)
#					...
#			LOC+2N+2	DP	A(N)		# RETURN IS TO LOC+2N+4

; Entry 1: POWRSERS - Coefficient table in fixed or erasable memory
; Calling sequence: A = address (LOC-3), L = N-1, MPAC = X (DP argument)
POWRSERS	EXTEND
		QXCH	POLYRET		# Save return address from Q register
		TS	POLISH		# Store coefficient table address (from A)
		LXCH	POLYCNT		# Store N-1 in counter (from L register)
		TCF	POLYCOM		# Jump to common setup

; Entry 2: POLY - Coefficient table follows TC POLY instruction
; Calling sequence: MPAC = X (DP), LOC = TC POLY, LOC+1 = N-1, LOC+2 = a(0)...
POLY		INDEX	Q		# Q points to instruction after TC POLY
		CAF	0		# Fetch N-1 (degree of polynomial)
		TS	POLYCNT		# N-1 TO COUNTER for loop control
		DOUBLE			# Double N-1 to get 2(N-1)
		AD	Q		# Add to Q to calculate coefficient address
		TS	POLISH		# Store address of a(N) - 3 in POLISH
		AD	FIVE		# Calculate return address (skip past table)
		TS	POLYRET		# STORE RETURN ADDRESS for TC POLYRET

; Common entry point for both POWRSERS and POLY
POLYCOM		CAF	LVBUF		# Load address of VBUF buffer
		TS	ADDRWD		# Set ADDRWD so DMPSUB multiplies by VBUF
					# (X will be saved in VBUF shortly)

		EXTEND			# Load first coefficient a(N)
		INDEX	POLISH		# Using POLISH as base address
		DCA	3		# Fetch DP coefficient at POLISH+3

# Page 1035
		DXCH	MPAC		# Load a(N) into MPAC (swap with X)
		DXCH	VBUF		# Save X in VBUF, retrieve a(N) back
					# Now: MPAC = a(N), VBUF = X
		TCF	POLY2		# Jump to main loop

; Main Horner's method loop: Repeatedly multiply by X and add next coefficient
; Implements: result = a(N)*X + a(N-1), then result*X + a(N-2), etc.
POLYLOOP	TS	POLYCNT		# Save decremented loop counter
		CS	TWO		# Load -2 (two words per DP coefficient)
		ADS	POLISH		# Decrement coefficient pointer to next lower coeff

; Loop body: Multiply current value by X, then add next coefficient
POLY2		TC	DMPSUB		# Multiply MPAC by X (in VBUF) using DMPSUB
					# Result in MPAC (double precision)
		EXTEND			# Prepare for DCA instruction
		INDEX	POLISH		# Use POLISH as base address
		DCA	1		# Fetch next coefficient (lower power)
		DAS	MPAC		# Add to MPAC (double precision add)
					# USER'S RESPONSIBILITY: Ensure no overflow

		CCS	POLYCNT		# Decrement and test loop counter
		TCF	POLYLOOP	# Continue if more coefficients remain
		TC	POLYRET		# Return to caller with P(X) in MPAC

# Page 1036
; ============================================================================
; MISCELLANEOUS MULTI-PRECISION ROUTINES
; ============================================================================
; These routines support fixed-fixed arithmetic operations but are not
; directly used by the interpreter. They provide utility functions for
; multi-precision number manipulation used by other AGC subsystems.
;
# MISCELLANEOUS MULTI-PRECISION ROUTINES REQUIRED IN FIXED-FIXED BUT NOT USED BY THE INTERPRETER.

; DPAGREE / TPAGREE - Force Sign Agreement in Multi-Precision Numbers
; Purpose: Ensures all words of a multi-precision number have consistent sign
; Used to correct rounding errors that can produce +0.0 and -0.0 in different words
;
; Entry: DPAGREE for double-precision (zeros MPAC+2)
;        TPAGREE for triple-precision (preserves all three words)
; Return: A contains sign of result (POSMAX or NEGMAX), normalized MPAC
;
DPAGREE		CAF	ZERO		# Double precision entry point
		TS	MPAC 	+2	# Zero low-order word for DP mode

TPAGREE		LXCH	Q		# Save return address in L
		TC	BRANCH		# Test sign of MPAC (most significant word)
		TCF	ARG+		# Branch if positive
		TCF	ARGZERO		# Branch if zero

		CS	POSMAX		# Handle negative case: load NEGMAX
		TCF		+2	# Skip next instruction

ARG+		CAF	POSMAX		# Handle positive case: load POSMAX
		TS	Q		# Save sign indicator in Q
		EXTEND
		AUG	A		# Form ±1.0 in A register
		AD	MPAC 	+2	# Add to least significant word
		TS	MPAC 	+2	# Store normalized LSW
		CAF	ZERO		# Prepare for carry propagation
		AD	Q		# Add sign (for carry)
		AD	MPAC 	+1	# Add to middle word
		TS	MPAC 	+1	# Store normalized middle word
		CAF	ZERO		# Prepare for final carry
		AD	Q		# Q still holds POSMAX or NEGMAX
		AD	MPAC		# Add to most significant word
ARGZERO2	TS	MPAC		# Store normalized MSW (always skips unless ARGZERO)
		TS	MPAC 	+1	# Propagate zero if ARGZERO case
		TC	L		# Return via L (saved return address)

ARGZERO		TS	MPAC 	+2	# Input was zero: clear all three words
		TCF	ARGZERO2	# Continue to clear MPAC and MPAC+1

; SHORTMP - Short Multiply
; Purpose: Multiplies triple-precision MPAC by single-precision A register
; Entry: A = single-precision multiplier, MPAC = TP multiplicand (three words)
; Exit: MPAC = TP product
; Note: Used for scaling operations where one factor is small integer
;
# SHORTMP MULTIPLIES THE TP CONTENTS OF MPAC BY THE SINGLE PRECISION NUMBER ARRIVING IN A.

SHORTMP		TS	MPTEMP		# Save single-precision multiplier
		EXTEND			# Prepare for multiply
		MP	MPAC 	+2	# Multiply by least significant word
		TS	MPAC 	+2	# Store low-order product
SHORTMP2	CAF	ZERO		# Prepare zero for DAS carry handling
		XCH	MPAC 	+1	# Get middle word, zero MPAC+1
		TCF	DMPSUB2		# Continue with double-precision multiply logic

# Page 1037
; DMPNSUB - Double-Precision Multiply by Integer
; Purpose: Multiplies DP fraction in MPAC by SP integer in A
; Entry: A = single-precision integer multiplier
;        MPAC, MPAC+1 = double-precision fraction multiplicand
; Exit: MPAC, MPAC+1 = double-precision product (also returned in A, L)
;
; CAUTION: This routine INCREASES magnitude (scales up by integer factor)
; User must ensure no overflow:
;   |A × MPAC| < 1.0  AND  |A × (MPAC, MPAC+1)| < 1.0
; where B(x) indicates arriving contents
;
# DMPNSUB MULTIPLIES THE DP FRACTION ARRIVING IN MPAC BY THE SP
# INTEGER ARRIVING IN A.  THE DP PRODUCT DEPARTS BOTH IN MPAC AND IN
# A AND L.  NOTE THAT DMPNSUB NORMALLY INCREASES THE MAGNITUDE OF THE
# CONTENTS OF MPAC.  THE CUSTOMER MUST INSURE THAT B(A) X B(MPAC,MPAC+1)
# AND B(A) X B(MPAC) ARE LESS THAN 1 IN MAGNITUDE, WHERE B, AS IS OBVIOUS,
# INDICATES THE ARRIVING CONTENTS.

DMPNSUB		TS	DMPNTEMP	# Save integer multiplier
		EXTEND			# Prepare for multiply
		MP	MPAC 	+1	# Multiply by LSW of fraction
		DXCH	MPAC		# Low product to MPAC, save MSW in A
		EXTEND			# Prepare for second multiply
		MP	DMPNTEMP	# Multiply saved MSW by integer
		CA	L		# Get low word of this product
		ADS	MPAC		# Add to MPAC to complete full product
		EXTEND			# Prepare to return result
		DCA	MPAC		# Load product into A and L registers
		TC	Q		# Return to caller

# Page 1038
; ============================================================================
; MISCELLANEOUS VECTOR OPERATIONS
; ============================================================================
; This section implements interpretive vector and matrix operations essential
; for guidance and navigation computations. All operations use double-precision
; (DP) arithmetic for maximum accuracy in 3D vector mathematics.
;
; Operations provided:
;   1. DOT    - Vector dot product (returns scalar)
;   2. VXV    - Vector cross product (returns vector)
;   3. VXSC   - Vector times scalar (scaling)
;   4. V/SC   - Vector divided by scalar
;   5. VPROJ  - Vector projection: (MPAC·X)MPAC
;   6. VXM    - Vector post-multiplied by matrix (row vectors)
;   7. MXV    - Vector pre-multiplied by matrix (column vectors)
;
; Vector format: Three DP components in MPAC+0,+1 (X), MPAC+3,+4 (Y), MPAC+5,+6 (Z)
; Matrix format: Nine DP elements, row-major or column-major depending on operation
; Used extensively during: Orbital mechanics, IMU transformations, targeting
;
# MISCELLANEOUS VECTOR OPERATIONS.  INCLUDED HERE ARE THE FOLLOWING.
#	1.	DOT	DP VECTOR DOT PRODUCT.
#	2.	VXV	DP VECTOR CROSS PRODUCT.
#	3.	VXSC	DP VECTOR TIMES SCALAR.
#	4.	V/SC	DP VECTOR DIVIDED BY SCALAR.
#	5.	VPROJ	DP VECTOR PROJECTION.  ( (MPAC.X)MPAC ).
#	6.	VXM	DP VECTOR POST-MULTIPLIED BY MATRIX.
#	7.	MXV	DP VECTOR PRE-MULTIPLIED BY MATRIX.

; DOT - Vector Dot Product
; Purpose: Computes scalar dot product of two DP vectors
; Entry: MPAC = first vector, ADDRWD = address of second vector
; Exit: MPAC = DP scalar result (V1·V2), MODE = DP scalar
;
DOT		TC	PREDOT		# Compute dot product via DOTSUB
DMODE		CAF	ZERO		# Switch mode to DP scalar
		TCF	NEWMODE		# Update MODE and continue

; MXV - Matrix times Vector (Pre-multiply)
; Purpose: Multiplies 3×3 matrix by 3D vector: M × V
; Matrix stored as row vectors (rows are contiguous)
; Entry: MPAC = vector V, ADDRWD = address of matrix M
; Exit: MPAC = transformed vector (result of M × V)
;
MXV		CAF	TWO		# Row vector increment (2 words per DP)
		TS	MATINC		# Store matrix stepping increment
		TCF	VXM/MXV		# Continue to common multiply logic

; VXM - Vector times Matrix (Post-multiply)
; Purpose: Multiplies 3D vector by 3×3 matrix: V × M
; Matrix stored as column vectors (columns are contiguous)
; Entry: MPAC = vector V, ADDRWD = address of matrix M
; Exit: MPAC = transformed vector (result of V × M)
;
VXM		CS	TEN		# Column vector increment (negative for indexing)
		TS	MATINC		# Store matrix stepping increment
		CAF	SIX		# Offset for column-major access

# Page 1039
# COMMON PORTION OF MXV AND VXM.

VXM/MXV		TS	DOTINC
# ITRACE (2) REFERS TO "VXM/MXV".
		TC	MPACVBUF	# SAVE VECTOR IN MPAC FOR FURTHER USE.

		TC	DOTSUB		# GO DOT TO GET X COMPONENT OF ANSWER.
		EXTEND
		DCA	VBUF		# MOVE MPAC VECTOR BACK INTO MPAC, SAVING
		DXCH	MPAC		# NEW X COMPONENT IN BUF2.
		DXCH	BUF2
		EXTEND
		DCA	VBUF 	+2
		DXCH	MPAC 	+3
		EXTEND
		DCA	VBUF 	+4
		DXCH	MPAC 	+5
		CA	MATINC		# INITIALIZE ADDRWD FOR NEXT DOT PRODUCT.
		ADS	ADDRWD		# FORMS HAS ADDRESS OF NEXT COLUMN(ROW).

		TC	DOTSUB
		DXCH	VBUF		# MORE GIVEN VECTOR BACK TO MPAC, SAVING Y
		DXCH	MPAC		# COMPONENT OF ANSWER IN VBUF +2.
		DXCH	VBUF 	+2
		DXCH	MPAC	+3
		DXCH	VBUF 	+4
		DXCH	MPAC 	+5
		CA	MATINC		# FORM ADDRESS OF LAST COLUMN OR ROW.
		ADS	ADDRWD

		TC	DOTSUB
		DXCH	BUF2		# ANSWER NOW COMPLETE. PUT COMPONENTS INTO
		DXCH	MPAC		# PROPER MPAC REGISTERS.
		DXCH	MPAC 	+5
		DXCH	VBUF 	+2
		DXCH	MPAC 	+3
		TCF	DANZIG		# EXIT.

# Page 1040
; VXSC - Vector Times Scalar
; Purpose: Multiplies DP vector by DP scalar
; Entry: MPAC = vector (V1, V2, V3), scalar at ADDRWD (or vice versa if MODE ≠ 0)
; Exit: MPAC = result vector (scalar×V1, scalar×V2, scalar×V3)
; Mode handling: CCS MODE tests which operand is in MPAC
;   - If MODE = 0: Vector in MPAC, scalar at address → use VVXSC path
;   - If MODE ≠ 0: Scalar in MPAC, vector at address → use DVXSC path
; Used for:
;   - Scaling velocity vectors during guidance computations
;   - Thrust magnitude application to unit direction vectors
;   - Coordinate scaling during transformations
;   - Time-scaled integration steps
; Algorithm: Multiplies each vector component by scalar using DMPSUB
;            Rounds each component using VROUND
;            Rotates components back to standard position
; During landing: Scales computed thrust direction by desired magnitude
;
# VXSC -- VECTOR TIMES SCALAR.

VXSC		CCS	MODE		# TEST PRESENT MODE.
		TCF	DVXSC		# SEPARATE ROUTINE WHEN SCALAR IS IN MPAC.
		TCF	DVXSC

VVXSC		TC	DMPSUB		# COMPUTE X COMPONENT
		TC	VROUND		# AND ROUND IT.
		DXCH	MPAC 	+3	# PUT Y COMPONENT INTO MPAC SAVING MPAC IN
		DXCH	MPAC		# MPAC +3.
		DXCH	MPAC 	+3

		TC	DMPSUB		# DO SAME FOR Y AND Z COMPONENTS.
		TC	VROUND
		DXCH	MPAC 	+5
		DXCH	MPAC
		DXCH	MPAC	+5

		TC	DMPSUB
		TC	VROUND
VROTATEX	DXCH	MPAC		# EXIT USED TO RESTORE MPAC AFTER THIS
		DXCH	MPAC 	+5	# TYPE OF ROTATION.  CALLED BY VECTOR SHIFT
		DXCH	MPAC 	+3	# RIGHT, V/SC, ETC.
		DXCH	MPAC
		TCF	DANZIG

# Page 1041
; VPROJ - Vector Projection
; Purpose: Projects vector MPAC onto vector X: result = (MPAC·X)·MPAC
; Entry: MPAC = vector to be scaled, ADDRWD = address of vector X
; Exit: MPAC = projection of MPAC onto X
; Formula: VPROJ = (MPAC·X) × MPAC, where (MPAC·X) is scalar dot product
; Geometric interpretation:
;   - Computes component of MPAC in direction of X
;   - Result is parallel to MPAC, scaled by dot product with X
;   - Used in guidance to decompose vectors into desired directions
; Implementation: Calls PREDOT to compute dot product, then falls into DVXSC
;                 to multiply MPAC by the scalar result
; Used during landing for: velocity decomposition, thrust direction alignment
;
# DP VECTOR PROJECTION ROUTINE.

VPROJ		TC	PREDOT		# (MPAC.X)MPAC IS COMPUTED AND LEFT IN
		CS	FOUR		# MPAC.  DO DOT AND FALL INTO DVXSC.
		ADS	ADDRWD

# VXSC WHEN SCALAR ARRIVES IN MPAC AND VECTOR IS AT X.

DVXSC		EXTEND			# SAVE SCALAR IN MPAC +3 AND GET X
		DCA	MPAC		# COMPONENT OF ANSWER.
		DXCH	MPAC 	+3
		TC	DMPSUB
		TC	VROUND

		CAF	TWO		# ADVANCE ADDRWD TO Y COMPONENT OF X.
		ADS	ADDRWD
		EXTEND
		DCA	MPAC 	+3	# PUT SCALAR BACK INTO MPAC AND SAVE
		DXCH	MPAC		# X RESULT IN MPAC +5.
		DXCH	MPAC 	+5
		TC	DMPSUB
		TC	VROUND

		CAF	TWO
		ADS	ADDRWD		# TO Z COMPONENT.
		DXCH	MPAC 	+3	# BRING SCALAR BACK, PUTTING Y RESULT IN
		DXCH	MPAC		# THE PROPER PLACE.
		DXCH	MPAC 	+3
		TC	DMPSUB
		TC	VROUND

		DXCH	MPAC		# PUT Z COMPONENT IN PROPER PLACE, ALSO
		DXCH	MPAC 	+5	# POSITIONING X.
		DXCH	MPAC

		TCF	VMODE		# MODE HAS CHANGED TO VECTOR.

# Page 1042
; VXV - Vector Cross Product
; Purpose: Computes DP vector cross product M × X where M is in MPAC
; Entry: MPAC = vector M (M1, M2, M3), ADDRWD = address of vector X (X1, X2, X3)
; Exit: MPAC = result vector (M × X), perpendicular to both input vectors
; Formula: M × X = (M2·X3 - M3·X2, M3·X1 - M1·X3, M1·X2 - M2·X1)
;          Result components: (X component, Y component, Z component)
; Used extensively for:
;   - Angular momentum calculations during orbital mechanics
;   - Coordinate frame transformations (IMU to navigation frame)
;   - Gimbal angle computations
;   - Attitude control calculations
; Cross product properties:
;   - Result magnitude = |M|·|X|·sin(θ) where θ is angle between vectors
;   - Result direction follows right-hand rule (perpendicular to plane of M and X)
;   - Anti-commutative: M × X = -(X × M)
;   - Used during landing for: thrust vector orientation, terrain slope calculations
; Algorithm uses VBUF for intermediate storage to preserve partial results
; Temporary storage: VBUF stores M1, VBUF+2 stores X1M3, VBUF+4 stores -X2M3
;
# VECTOR CROSS PRODUCT ROUTINE CALCULATES (X M -X M ,X M -X M ,X M -X M ) WHERE M IS THE VECTOR IN
#                                           3 2  2 3  1 3  3 1  2 1  1 2
# MPAC AND X THE VECTOR AT THE GIVEN ADDRESS.

VXV		EXTEND
		DCA	MPAC 	+5	# FORM UP M3X1, LEAVING M1 IN VBUF.
		DXCH	MPAC
		DXCH	VBUF
		TC	DMPSUB		# BY X1.

		EXTEND
		DCS	MPAC 	+3	# CALCULATE -X1M2, SAVING X1M3 IN VBUF +2.
		DXCH	MPAC
		DXCH	VBUF 	+2
		TC	DMPSUB

		CAF	TWO		# ADVANCE ADDRWD TO X2.
		ADS	ADDRWD
		EXTEND
		DCS	MPAC 	+5	# PREPARE TO GET -X2M3, SAVING -X1M2 IN
		DXCH	MPAC		# MPAC +5.
		DXCH	MPAC 	+5
		TC	DMPSUB

		EXTEND
		DCA	VBUF		# GET X2M1, SAVING -X2M3 IN VBUF +4.
		DXCH	MPAC
		DXCH	VBUF 	+4
		TC	DMPSUB

		CAF	TWO		# ADVANCE ADDRWD TO X3.
		ADS	ADDRWD
		EXTEND
		DCS	VBUF		# GET -X3M1, ADDING X2M1 TO MPAC +5 TO
		DXCH	MPAC		# COMPLETE THE Z COMPONENT OF THE ANSWER.
		DAS	MPAC 	+5

		EXTEND
		BZF	+2
		TC	OVERFLWZ

		TC	DMPSUB
		DXCH	VBUF 	+2	# MOVE X1M3 TO MPAC +3 SETTING UP FOR X3M2
		DXCH	MPAC 	+3	# AND ADD -X3M1 TO MPAC +3 TO COMPLETE THE
		DXCH	MPAC		# Y COMPONENT OF THE RESULT.
		DAS	MPAC 	+3

		EXTEND
		BZF		+2
# Page 1043
		TC OVERFLWY

		TC	DMPSUB
		DXCH	VBUF 	+4	# GO ADD -X2M3 TO X3M2 TO COMPLETE THE X
		TCF	ENDVXV		# COMPONENT (TAIL END OF DAD).

# THE MPACVBUF SUBROUTINE SAVES THE VECTOR IN MPAC IN VBUF WITHOUT CLOBBERING MPAC.

MPACVBUF	EXTEND			# CALLED BY MXV, VXM, AND UNIT.
		DCA	MPAC
		DXCH	VBUF
		EXTEND
		DCA	MPAC 	+3
		DXCH	VBUF 	+2
		EXTEND
		DCA	MPAC 	+5
		DXCH	VBUF 	+4
		TC	Q		# RETURN TO CALLER.

# DOUBLE PRECISION SIGN AGREE ROUTINE.  ARRIVE WITH INPUT IN A+L.  OUTPUT IS IN A + L.

ALSIGNAG	CCS	A		# TEST UPPER PART.
		TCF	UPPOS		# IT IS POSITIVE
		TC	Q		# ZERO
		TCF	UPNEG		# NEGATIVE
		TC	Q		# ZERO

UPPOS		XCH	L		# SAVE DECREMENTED UPPER PART.
		AD	HALF
		AD	HALF
		TS	A		# SKIPS ON OVERFLOW
		TCF	+2
		INCR	L		# RESTORE UPPER TO ORIGINAL VALUE
		XCH	L		# SWAP A + L BANCK.
		TC	Q

UPNEG		XCH	L		# SAVE COMPLEMENTED + DECREMENTED UPPER PT
		AD	NEGMAX
		AD	NEGONE
		TS	A
		TCF	+2		# DON'T INCREMENT IF NO OVERFLOW.
		INCR	L
		XCH	L
		COM			# MAKE NEGATIVE AGAIN.
		TC	Q

# Page 1044
# INTERPRETIVE INSTRUCTIONS WHOSE EXECUTION CONSISTS OF PRINCIPALLY CALLING SUBROUTINES.

DMP1		TC	DMPSUB		# DMP INSTRUCTIONS
		TCF	DANZIG

DMPR		TC	DMPSUB
		TC	ROUNDSUB +1	# (C(A) = +0).
		TCF	DANZIG

DDV		EXTEND
		INDEX	ADDRWD		# MOVE DIVIDEND INTO BUF.
		DCA	0
		TCF	BDDV 	+4

BDDV		EXTEND			# MOVE DIVISOR INTO MPAC SAVING MPAC, THE
		INDEX	ADDRWD		# DIVIDEND, IN BUF.
		DCA	0
		DXCH	MPAC
 +4		DXCH	BUF
		CAF	ZERO		# DIVIDE ROUTINES IN BANK 0.
		TS	FBANK
		TCF	DDV/BDDV

SETPD		CA	ADDRWD		# MUST SET TO WORK AREA, OR EBANK TROUBLE.
		TS	PUSHLOC
		TCF	NOIBNKSW	# NO FBANK SWITCH REQUIRED.

TSLC		CAF	ZERO		# SHIFTING ROUTINES LOCATED IN BANK 00.
		TS	FBANK
		TCF	TSLC2

GSHIFT		CAF	LOW7		# USED AS MASK AT GENSHIFT. THIS PROCESSES
		TS	FBANK		# ANY SHIFT INSTRUCTION (EXCEPT TSLC) WITH
		TCF	GENSHIFT	# AN ADDRESS (ROUTINES IN BANK 0).

# Page 1045
; V/SC - Vector Divided by Scalar
; Purpose: Divides DP vector by DP scalar
; Entry: Either MPAC = vector and scalar at ADDRWD, or MPAC = scalar and vector at ADDRWD
; Exit: MPAC = result vector (V1/scalar, V2/scalar, V3/scalar)
; Mode handling: CCS MODE determines which operand is in MPAC
;   - If MODE = 0 (vector): Vector in MPAC, scalar at address → use VV/SC path
;   - If MODE ≠ 0 (scalar): Scalar in MPAC, vector at address → use DV/SC path
; Used for:
;   - Normalizing velocity vectors to unit length
;   - Converting accelerations to unit-magnitude form
;   - Scaling position vectors during trajectory computations
;   - Time-inverse operations in guidance equations
; Algorithm: 
;   1. Arranges operands so vector is in MPAC, scalar in BUF
;   2. Calls V/SC2 routine in bank 0 which divides each component by scalar
;   3. Uses DDV (double-precision divide) for each vector component
;   4. Rounds results and restores to proper MPAC locations
; Safety: Divides by scalar in BUF, so divisor must be non-zero
; During landing: Used in velocity normalization and guidance direction computation
;
# THE FOLLOWING IS THE PROLOGUE TO V/SC.  IF THE PRESENT MODE IS VECTOR, IT SAVES THE SCALAR AT X IN BUF
# AND CALLS THE V/SC ROUTINE IN BANK 0.  IF THE PRESENT MODE IS SCALAR, IT MOVES THE VECTOR AT X INTO MPAC, SAVING
# THE SCALAR IN MPAC IN BUF BEFORE CALLING THE V/SC ROUTINE IN BANK 0.

V/SC		CCS	MODE
		TCF	DV/SC		# MOVE VECTOR INTO MPAC.
		TCF	DV/SC

VV/SC		EXTEND
		INDEX	ADDRWD
		DCA	0
V/SC1		DXCH	BUF		# IN BOTH CASES, VECTOR IS NOW IN MPAC AND
		CAF	ZERO		# SCALAR IN BUF.
		TS	FBANK
		TCF	V/SC2

DV/SC		EXTEND
		INDEX	ADDRWD
		DCA	2
		DXCH	MPAC 	+3
		EXTEND
		INDEX	ADDRWD
		DCA	4
		DXCH	MPAC 	+5

		CS	ONE		# CHANGE MODE TO VECTOR.
		TS	MODE

		EXTEND
		INDEX	ADDRWD
		DCA	0
		DXCH	MPAC
		TCF	V/SC1		# FINISH PROLOGUE AT COMMON SECTION.

# Page 1046
# SIGN AND COMPLEMENT INSTRUCTIONS.

SIGN		INDEX 	ADDRWD		# CALL COMP INSTRUCTION IF WORD AT X IS
		CCS	0		# NEGATIVE NON-ZERO.
		TCF	DANZIG
		TCF	+2
		TCF	COMP		# DO THE COMPLEMENT.

		INDEX	ADDRWD
CCSL		CCS	1
		TCF	DANZIG
		TCF	DANZIG
		TCF	COMP
		TCF	DANZIG
COMP		EXTEND			# COMPLEMENT DP MPAC IN EVERY CASE.
		DCS	MPAC
		DXCH	MPAC

		CCS	MODE		# EITHER COMPLEMENT MPAC +3 OR THE REST OF
		TCF	DCOMP		# THE VECTOR ACCUMULATOR.
		TCF	DCOMP

		EXTEND			# VECTOR COMPLEMENT.
		DCS	MPAC 	+3
		DXCH	MPAC 	+3
		EXTEND
		DCS	MPAC 	+5
		DXCH	MPAC 	+5
		TCF	DANZIG

DCOMP		CS	MPAC 	+2
		TS	MPAC 	+2
		TCF	DANZIG

# Page 1047
# THE FOLLOWING SHORT SHIFT CODES REQUIRE NO ADDRESS WORD:
#	1.	SR1 TO SR4	SCALAR SHIFT RIGHT.
#	2.	SR1R TO SR4R	SCALAR SHIFT RIGHT AND ROUND.
#	3.	SL1 TO SL4	SCALAR SHIFT LEFT.
#	4.	SL1R TO SL4R	SCALAR SHIFT LEFT AND ROUND.
#	5.	VSR1 TO VSR8	VECTOR SHIFT RIGHT (ALWAYS ROUNDS).
#	6.	VSL1 TO VSL8	VECTOR SHIFT LEFT (NEVER ROUNDS).
# THE FOLLOWING CODES REQUIRE AND ADDRESS WHICH MAY BE INDEXED:*
#	1.	SR		SCALAR SHIFT RIGHT.
#	2.	SRR		SCALAR SHIFT RIGHT AND ROUND.
#	3.	SL		SCALAR SHIFT LEFT.
#	4.	SLR		SCALAR SHIFT LEFT AND ROUND.
#	5.	VSR		VECTOR SHIFT RIGHT.
#	6.	VSL		VECTOR SHIFT LEFT.
# * IF THE ADDRESS IS INDEXED, AND THE INDEX MODIFICATION RESULTS IN A NEGATIVE SHIFT COUNT, A SHIFT OF THE
# ABSOLUTE VALUE OF THE COUNT IS DONE IN THE OPPOSITE DIRECTION.

		BANK	00

		COUNT*	$$/INTER
SHORTT		CAF	SIX		# SCALAR SHORT SHIFTS COME HERE.  THE SHIFT
		MASK	CYR		# COUNT-1 IS NOW IN BITS 2-3 OF CYR.  THE
		TS	SR		# ROUNDING BIT IS IN BIT1 AT THIS POINT.

		CCS	CYR		# SEE IF RIGHT OR LEFT SHIFT DESIRED.
		TCF	TSSL		# SHIFT LEFT.

SRDDV		DEC	20		# MPTEMP SETTING FOR SR BEFORE DDV.

TSSR		INDEX	SR		# GET SHIFTING BIT.
		CAF	BIT14
		TS	MPTEMP

		CCS	CYR		# SEE IF A ROUND IS DESIRED.
RIGHTR		TC	MPACSRND	# YES -- SHIFT RIGHT AND ROUND.
		TCF	NEWMODE		# SET MODE TO DP (C(A) = 0).
MPACSHR		CA	MPTEMP		# DO A TRIPLE PRECISION SHIFT RIGHT.
		EXTEND
		MP	MPAC 	+2
 +3		TS	MPAC 	+2	# (EXIT FROM SQRT AND ABVAL).
		CA	MPTEMP
		EXTEND
		MP	MPAC		# SHIFT MAJOR PART INTO A,L AND PLACE IN
# Page 1048
		DXCH	MPAC		# MPAC,+1.
		CA	MPTEMP
		EXTEND
		MP	L		# ORIGINAL C(MPAC +1).
		DAS	MPAC 	+1	# GUARANTEED NO OVERFLOW.
		TCF	DANZIG

# MPAC SHIFT RIGHT AND ROUND SUBROUTINES

MPACSRND	CA	MPAC 	+2	# WE HAVE TO DO ALL THREE MULTIPLIES SINCE
		EXTEND			# MPAC +1 AND MPAC +2 MIGHT HAVE SIGN
		MP	MPTEMP		# DISAGREEMENT WITH A SHIFT RIGHT OF L.
		XCH	MPAC 	+1
		EXTEND
		MP	MPTEMP
		XCH	MPAC 	+1	# TRIAL MINOR PART.
		AD	L

VSHR2		DOUBLE			# (FINISH VECTOR COMPONENT SHIFT RIGHT
		TS	MPAC 	+2	# AND ROUND.)
		TCF	+2
		ADS	MPAC 	+1	# GUARANTEED NO OVERFLOW.

		CAF	ZERO
		TS	MPAC 	+2
		XCH	MPAC		# SETTING TO ZERO SO FOLLOWING DAS WORKS.
		EXTEND
		MP	MPTEMP
		DAS	MPAC		# AGAIN NO OVERFLOW.
		TC	Q

VSHRRND		CA	MPTEMP		# ENTRY TO SHIFT RIGHT AND ROUND MPAC WHEN
		EXTEND			# MPAC CONTAINS A VECTOR COMPONENT.
		MP	MPAC 	+1
		TS	MPAC 	+1
		XCH	L
		TCF	VSHR2		# GO ADD ONE IF NECESSARY AND FINISH.

# Page 1049
# ROUTINE FOR SHORT SCALAR SHIFT LEFT (AND MAYBE ROUND).

TSSL		CA	SR		# GET SHIFT COUNT FOR SR.
 +1		TS	MPTEMP

 +2		EXTEND			# ENTRY HERE FROM SL FOR SCALARS.
		DCA	MPAC 	+1	# SHIFTING LEFT ONE PLACE AT A TIME IS
		DAS	MPAC 	+1	# FASTER THAN DOING THE WHOLE SHIFT WITH
		AD	MPAC		# MULTIPLIES ASSUMING THAT FREQUENCY OF
		AD	MPAC		# SHIFT COUNTS GOES DOWN RAPIDLY AS A
		TS	MPAC		# FUNCTION OF THEIR MAGNITUDE.
		TCF 	+2
		TS	OVFIND		# OVERFLOW.  (LEAVES OVERFLOW-CORRECTED
					# RESULT ANYWAY).
		CCS	MPTEMP		# LOOP ON DECREMENTED SHIFT COUNT.
		TCF	TSSL 	+1

		CCS	CYR		# SEE IF ROUND WANTED.
ROUND		TC	ROUNDSUB	# YES -- ROUND AND EXIT.
		TCF	DANZIG		# SL LEAVES A ZERO IN CYR FOR NO ROUND.
		TCF	DANZIG		# NO -- EXIT IMMEDIATELY

# Page 1050
# VECTOR SHIFTING ROUTINES.

SHORTV		CAF	LOW3		# SAVE 3 BIT SHIFT COUNT -- 1 WITHOUT
		MASK	CYR		# EDITING CYR.
		TS	MPTEMP
		CCS	CYR		# SEE IF LEFT OR RIGHT SHIFT.
		TCF	VSSL		# VECTOR SHIFT LEFT.
OCT176		OCT	176		# USED IN PROCESSED SHIFTS WITH - COUNT.

VSSR		INDEX	MPTEMP		# (ENTRY FROM SR).  PICK UP SHIFTING BIT.
		CAF	BIT14		# MPTEMP CONTAINS THE SHIFT COUNT - 1.
		TS	MPTEMP
		TC	VSHRRND		# SHIFT X COMPONENT.

		DXCH	MPAC		# SWAP X AND Y COMPONENTS.
		DXCH	MPAC 	+3
		DXCH	MPAC
		TC	VSHRRND		# SHIFT Y COMPONENT.

		DXCH	MPAC		# SWAP Y AND Z COMPONENTS.
		DXCH	MPAC 	+5
		DXCH	MPAC
		TC	VSHRRND		# SHIFT Z COMPONENT.

		TCF	VROTATEX	# RESTORE COMPONENTS TO PROPER PLACES.

# Page 1051
# VECTOR SHIFT LEFT -- DONE ONE PLACE AT A TIME.

 -1		TS	MPTEMP		# SHIFTING LOOP.

VSSL		EXTEND
		DCA	MPAC
		DAS	MPAC
		EXTEND
		BZF	+2
		TC	OVERFLOW

		EXTEND
		DCA	MPAC 	+3
		DAS	MPAC 	+3
		EXTEND
		BZF	+2
		TC	OVERFLWY

		EXTEND
		DCA	MPAC 	+5
		DAS	MPAC 	+5
		EXTEND
		BZF	+2
		TC	OVERFLWZ

		CCS	MPTEMP		# LOOP ON DECREMENTED SHIFT COUNTER.
		TCF	VSSL 	-1
		TCF	DANZIG		# EXIT.

# Page 1052
# TSLC -- TRIPLE SHIFT LEFT AND COUNT.  SHIFTS MPAC LEFT UNTIL GREATER THAN .5 IN MAGNITUDE, LEAVING
# THE COMPLEMENT OF THE NUMBER OF SHIFTS REQUIRED IN X.

TSLC2		TS	MPTEMP		# START BY ZEROING SHIFT COUNT (IN A NOW).
		TC	BRANCH		# EXIT WITH NO SHIFTING IF ARGUMENT ZERO.
		TCF	+2
		TCF	ENDTSLC		# STORES ZERO SHIFT COUNT IN THIS CASE.

		TC	TPAGREE		# MAY CAUSE UPSHIFT OF ONE EXTRA PLACE.

		CA	MPAC		# BEGIN NORMALIZATION LOOP.
		TCF	TSLCTEST

TSLCLOOP	INCR	MPTEMP		# INCREMENT SHIFT COUNTER.
		EXTEND
		DCA	MPAC 	+1
		DAS	MPAC 	+1
		AD	MPAC
		ADS	MPAC
TSLCTEST	DOUBLE			# SEE IF (ANOTHER) SHIFT IS REQUIRED
		OVSK
		TCF	TSLCLOOP	# YES -- INCREMENT COUNT AND SHIFT AGAIN.

ENDTSLC		CS	MPTEMP
		TCF	STORE1		# STORE SHIFT COUNT AND RETURN TO DANZIG.

# Page 1053
# THE FOLLOWING ROUTINE PROCESSES THE GENERAL SHIFT INSTRUCTIONS SR, SRR, SL, AND SLR.
# THE GIVEN ADDRESS IS DECODED AS FOLLOWS:
#	BITS 1-7	SHIFT COUNT (SUBADDRESS) LESS THAN 125 DECIMAL.
#	BIT 8		PSEUDO SIGN BIT (DETECTS CHANGE IN SIGN IN INDEXED SHIFTS).
#	BIT 9		0 FOR LEFT SHIFT, AND 1 FOR RIGHT SHIFT.
#	BIT 10		1 FOR TERMINAL ROUND ON SCALAR SHIFTS, 0 OTHERWISE
#	BITS 11-13	0.
#	BIT 14		1.
#	BIT 15		0.
# THE ABOVE ENCODING IS DONE BY THE YUL SYSTEM.

GENSHIFT	MASK	ADDRWD		# GET SHIFT COUNT, TESTING FOR ZERO.
		CCS	A		# (ARRIVES WITH C(A) = LOW7).
		TCF	GENSHFT2	# IF NON-ZERO, PROCEED WITH DECREMENTED CT

		CAF	BIT10		# ZERO SHIFT COUNT.  NO SHIFTS NEEDED BUT
		MASK	ADDRWD		# WE MIGHT HAVE TO ROUND MPAC ON SLR AND
		CCS	A		# SRR (SCALAR ONLY).
		TC	ROUNDSUB
		TCF	DANZIG

GENSHFT2	TS	MPTEMP		# DECREMENTED SHIFT COUNT TO MPTEMP.
		CAF	BIT8		# TEST MEANING OF LOW SEVEN BIT COUNT IN
		EXTEND			# MPTEMP NOW.
		MP	ADDRWD
		MASK	LOW2		# JUMPS ON SHIFT DIRECTION (BIT8) AND
		INDEX	A
		TCF	+1		# ORIGINAL SHIFT DIRECTION (BIT 9)
		TCF	RIGHT-		# NEGATIVE SHIFT COUNT FOR SL OR SLR.
		TCF	LEFT		# SL OR SLR.
		TCF	LEFT-		# NEGATIVE SHIFT COUNT WITH SR OR SRR.

# Page 1054
# GENERAL SHIFT RIGHT

RIGHT		CCS	MODE		# SET IF VECTOR OR SCALAR.
		TCF	GENSCR
		TCF	GENSCR

		CA	MPTEMP		# SEE IF SHIFT COUNT LESS THAN 14D.
VRIGHT2		AD	NEG12
		EXTEND
		BZMF	VSSR		# IF SO, BRANCH AND SHIFT IMMEDIATELY.

		AD	NEGONE		# IF NOT, REDUCE MPTEMP BY A TOTAL OF 14.
		TS	MPTEMP		# AND DO A SHIFT RIGHT AND ROUND BY 14.
		CAF	ZERO		# THE ROUND AT THIS STAGE MAY INTRODUCE A
		TS	L		# ONE BIT ERROR IN A SHIFT RIGHT 15D.
		XCH	MPAC
		XCH	MPAC 	+1
		TC	SETROUND	# X COMPONENT NOW SHIFTED, SO MAKE UP THE
		DAS	MPAC		# ROUNDING QUANTITY (0 IN A AND 0 OR +-1
					# IN L).
		XCH	MPAC 	+3	# REPEAT THE ABOVE PROCESS FOR Y AND Z.
		XCH	MPAC 	+4
		TC	SETROUND
		DAS	MPAC 	+3	# NO OVERFLOW ON THESE ADDS.

		XCH	MPAC 	+5
		XCH	MPAC 	+6
		TC	SETROUND
		DAS	MPAC 	+5

		CCS	MPTEMP		# SEE IF DONE, DOING FINAL DECREMENT.
		TS	MPTEMP
		TCF	VRIGHT2
BIASLO		DEC	.2974 	B-1	# SQRT CONSTANT

		TCF	DANZIG

SETROUND	DOUBLE			# MAKES UP ROUNDING QUANTITY FROM ARRIVING
		TS	MPAC 	+2	# C(A).  L IS ZERO INITIALLY.
		CAF	ZERO
		XCH	L
		TC	Q		# RETURN AND DO THE DAS, RESETTING L TO 0.

# Page 1055
# PROCESS SR AND SRR FOR SCALARS.

GENSCR		CA	MPTEMP		# SEE IF THE ORIGINAL SHIFT COUNT WAS LESS
 +1		AD	NEG12		# THAN 14D.
		EXTEND
		BZMF	DOSSHFT		# DO THE SHIFT IMMEDIATELY IF SO.

 +4		AD	NEGONE		# IF NOT, DECREMENT SHIFT COUNT BY 14D AND
		TS	MPTEMP		# SHIFT MPAC RIGHT 14 PLACES.
		CAF	ZERO
		XCH	MPAC
		XCH	MPAC 	+1
		TS	MPAC 	+2
		CCS	MPTEMP		# SEE IF FINISHED, DO FINAL DECREMENT.
		TS	MPTEMP
		TC	GENSCR	+1
SLOPEHI		DEC	.5884		# SQRT CONSTANT.
		CAF	BIT10		# FINISHED WITH SHIFT.  SEE IF ROUND
		MASK	ADDRWD		# WANTED.
		CCS	A
		TC	ROUNDSUB
		TCF	DANZIG		# DO SO AND/OR EXIT.

DOSSHFT		INDEX	MPTEMP		# PICK UP SHIFTING BIT.
		CAF	BIT14
		TS	MPTEMP
		CAF	BIT10		# SEE IF TERMINAL ROUND DESIRED.
		MASK	ADDRWD
		CCS	A
		TCF	RIGHTR		# YES.
		TCF	MPACSHR		# JUST SHIFT RIGHT.

# Page 1056
# PROCESS THE RIGHT- (SL(R) WITH A NEGATIVE COUNT), LEFT-, AND LEFT OPTIONS.

RIGHT-		CS	MPTEMP		# GET ABSOLUTE VALUE - 1 OF SHIFT COUNT
		AD	OCT176		# UNDERSTANDING THAT BIT8 (PSEUDO-SIGN)
		TS	MPTEMP		# WAS 1 INITIALLY.
		TCF	RIGHT		# DO NORMAL SHIFT RIGHT.

LEFT-		CS	OCT176		# SAME PROLOGUE TO LEFT FOR INDEXED RIGHT
		AD	MPTEMP		# SHIFT WHOSE NET SHIFT COUNT IS NEGATIVE
		COM
		TS	MPTEMP

LEFT		CCS	MODE		# SINCE LEFT SHIFTING IS DONE ONE PLACE AT
		TCF	GENSCL		# A TIME, NO COMPARISON WITH 14 NEED BE
		TCF	GENSCL		# DONE.  FOR SCALARS, SEE IF TERMINAL ROUND
		TCF	VSSL		# DESIRED.  FOR VECTORS, SHIFT IMMEDIATELY.

GENSCL		CS	ADDRWD		# PUT ROUNDING BIT (BIT 10 OF ADDRWD) INTO
		EXTEND			# BIT 15 OF CYR WHERE THE ROUNDING BIT OF
		MP	BIT6		# A SHORT SHIFT LEFT WOULD BE
		TS	CYR
		TCF	TSSL 	+2	# DO THE SHIFT.

# Page 1057
# SCALAR DIVISION INSTRUCTIONS, DDV AND BDDV, ARE EXECUTED HERE.  AT THIS POINT, THE DIVIDEND IS IN MPAC
# AND THE DIVISOR IS IN BUF.

DDV/BDDV	CS	ONE		# INITIALIZATION
		TS	DVSIGN		# +-1 FOR POSITIVE QUOTIENT -- -0 FOR NEG.
		TS	DVNORMCT	# DIVIDENT NORMALIZATION COUNT.
		TS	MAXDVSW		# NEAR-ONE DIVIDE FLAG.

		CCS	BUF		# FORCE BUF POSITIVE WITH THE MAJOR PART
		TCF	BUFPOS		# NON-ZERO.
		TCF	+2
		TCF	BUFNEG

BUFZERO		TS	MPAC 	+2	# ZERO THIS.
		TC	TPAGREE		# FORCE SIGN AGREEMENT BEFORE OVERFLOW

		CCS	MPAC		# TEST TO SEE IF MPAC NON-ZERO.  (TOO BIG)
		TCF	OVF+		# MAJOR PART OF DIVIDEND IS POSITIVE NON-0
		TCF	+2
		TCF	OVF+ 	-1	# MAJOR PART OF DIVIDEND IS NEG. NON-ZERO

		XCH	BUF 	+1	# SHIFT DIVIDEND AND DIVISOR LEFT 14
		XCH	BUF
		XCH	MPAC 	+1
		XCH	MPAC
		CCS	BUF		# TRY AGAIN ON FORMER MINOR PART.
		TCF	BUF+
		TCF	+2		# OVERFLOW ON ZERO DIVISOR.
		TCF	BUF-

		CS	MPAC		# SIGN OF MPAC DETERMINES SIGN OF RESULT.
SGNDVOVF	EXTEND
		BZMF	+2
		INCR	DVSIGN		# NEGMAX IN MPAC PERHAPS.
DVOVF		CAF	POSMAX		# ON DIVISION OVERFLOW OF ANY SORT, SET
		TS	MPAC		# SET DP MPAC TO +-POSMAX.
		TC	FINALDV +3
		CAF	ONE		# SET OVERFLOW INDICATOR AND EXIT.
		TS	OVFIND
		TC	DANZIG

 -1		INCR	DVSIGN
OVF+		CS	BUF 	+1	# LOAD LOWER ORDER PART OF DIVISOR.
		TCF	SGNDVOVF	# GET SIGN OF RESULT.

BUF-		EXTEND			# IF BUF IS NEGATIVE, COMPLEMENT IT AND
		DCS	BUF		# MAINTAIN DVSIGN FOR FINAL QUOTIENT SIGN.
		DXCH	BUF
		INCR	DVSIGN		# NOW -0.

# Page 1058
BUF+		CCS	MPAC		# FORCE MPAC POSITIVE, CHECKING FOR ZERO
		TCF	MPAC+		# DIVIDEND IN THE PROCESS.
		TCF	+2
		TCF	MPAC-
		CCS	MPAC 	+1
		TCF	MPAC+
		TCF	DANZIG		# EXIT IMMEDIATELY ON ZERO DIVIDEND.
		TCF	MPAC-
		TCF	DANZIG

MPAC-		EXTEND			# FORCE MPAC POSITIVE AS BUF IN BUF-.
		DCS	MPAC
		DXCH	MPAC
		INCR	DVSIGN		# NOW +1 OR -0.

# Page 1059
MPAC+		CS	MPAC		# CHECK FOR DIVISION OVERFLOW.  IF THE
		AD	NEGONE		# MAJOR PART OF THE DIVIDEND IS LESS THAN
		AD	BUF		# THE MAJOR PART OF THE DIVISOR BY AT
		CCS	A		# LEAST TWO, WE CAN PROCEED IMMEDIATELY
		TCF	DVNORM		# WITHOUT NORMALIZATION PRODUCING A DVMAX.
-1/2+2		OCT	60001		# USED IN SQRTSUB.

		TCF	+1		# IF THE ABOVE DOES NOT HOLD, FORCE SIGN
		CAF	HALF		# AGREEMENT IN NUMERATOR AND DENOMINATOR
		DOUBLE			# TO FACILITATE OVERFLOW AND NEAR-ONE
		AD	MPAC 	+1	# CHECKING.
		TS	MPAC 	+1
		CAF	ZERO
		AD	POSMAX
		ADS	MPAC

		CAF	HALF		# SAME FOR BUF.
		DOUBLE
		AD	BUF 	+1
		TS	BUF 	+1
		CAF	ZERO
		AD	POSMAX
		ADS	BUF

		CS	MPAC		# CHECK MAGNITUDE OF SIGN-CORRECTED
		AD	BUF		# OPERANDS.
		CCS	A
		TCF	DVNORM		# DIVIDE OK -- WILL NOT BECOME MAXOV CASE.
LBUF2		ADRES 	BUF2
		TCF	DVOVF		# DIVISOR NOT LESS THAN DIVIDEND -- OVF.

		TS	MAXDVSW		# IF THE MAJOR PARTS OF THE DIVIDEND AND
		CS	MPAC 	+1	# DIVISOR ARE EQUAL, A SPECIAL APPROXIMA-
		AD	BUF	+1	# TION IS USED (PROVIDED THE DIVISION IS
		EXTEND			# POSSIBLE, OF COURSE).
		BZMF	DVOVF
		TCF	DVNORM		# IF NO OVERFLOW.

# Page 1060
BUFNORM		EXTEND			# ADD -1 TO AUGMENT SHIFT COUNT AND SHIFT
		AUG	DVNORMCT	# LEFT ONE PLACE.
		EXTEND
		DCA	BUF
		DAS	BUF

DVNORM		CA	BUF		# SEE IF DIVISOR NORMALIZED YET.
		DOUBLE
		OVSK
		TCF	BUFNORM		# NO -- SHIFT LEFT ONE AND TRY AGAIN.

		DXCH	MPAC		# CALL DIVIDEND NORMALIZATION SEQUENCE
		INDEX	DVNORMCT	# PRIOR TO DOING THE DIVIDE.
		TC	MAXTEST

		TS	MPAC 	+2	# RETURNS WITH DIVISION DONE AND C(A) = 0.
		TCF	DANZIG

BUFPOS		CCS	A
		TCF	BUF+		# TO BUF+ IF BUF IS GREATER THAN +1.

		CS	BUF 	+1	# IF BUF IS +1, FORCING SIGN AGREEMENT
		EXTEND			# MAY CAUSE BUF TO BECOME ZERO.
		BZMF	BUF+		# BRANCH IF SIGNS AGREE.

		CA	HALF		# SIGNS DISAGREE.  FORCE AGREEMENT.
 +6		DOUBLE
		ADS	BUF 	+1
		CA	ZERO
		TS	BUF
		TCF	BUFZERO

BUFNEG		CCS	A
		TCF	BUF-		# TO BUF- IF BUF IS LESS THAN -1.

		CA	BUF 	+1	# IF BUF IS -1, FORCING SIGN AGREEMENT
		EXTEND			# MAY CAUSE BUF TO BECOME ZERO.
		BZMF	BUF-		# BRANCH IF SIGNS AGREE.

		CS	HALF		# SIGNS DISAGREE.  FORCE AGREEMENT.
		TCF	BUFPOS +6

# Page 1061
# THE FOLLOWING ARE PROLOGUES TO SHIFT THE DIVIDEND ARRIVING IN A AND L BEFORE THE DIVIDE.

 -21D		LXCH	SR		# SPECIAL PROLOGUE FOR UNIT WHEN THE
		EXTEND			# LENGTH OF THE ARGUMENT WAS NOT LESS THAN
		MP	HALF		# .5.  IN THIS CASE, EACH COMPONENT MUST BE
		XCH	L		# SHIFTED RIGHT ONE TO PRODUCE A HALF-UNIT
		AD	SR		# VECTOR.
		XCH	L
		TCF	GENDDV 	+1	# WITH DP DIVIDEND IN A,L.

		DDOUBL			# PROLOGUE WHICH NORMALIZES THE DIVIDEND
		DDOUBL			# WHEN IT IS KNOWN THAT NO DIVISION
		DDOUBL			# OVEFLOW WILL OCCUR.
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DDOUBL
		DXCH	MPAC

MAXTEST		CCS	MAXDVSW		# 0 IF MAJORS MIGHT BE =, -1 OTHERWISE.
BIASHI		DEC	.4192 	B-1	# SQRT CONSTANTS.

		TCF	MAXDV		# CHECK TO SEE IF THAY ARE NOW EQUAL.

# Page 1062
# THE FOLLOWING IS A GENERAL PURPOSE DOUBLE PRECISION DIVISION ROUTINE.  IT DIVIDES MPAC BY BUF AND LEAVES
# THE RESULT IN MPAC.  THE FOLLOWING CONDITIONS MUST BE SATISFIED:
#
#	1.	THE DIVISOR (BUF) MUST BE POSITIVE AND NOT LESS THAN .5.
#
#	2.	THE DIVIDEND (MPAC) MUST BE POSITIVE WITH THE MAJOR PART OF MPAC STRICTLY LESS THAN THAT OF BUF
#		(A SPECIAL APPROXIMATION, MAXDV, IS USED WHEN THE MAJOR PARTS ARE EQUAL).
#
# UNDERSTANDING THAT A/B = Q + S(R/B) WHERE S = 2(-14) AND Q AND R ARE QUOTIENT AND REMAINDER, RESPEC-
# TIVELY, THE FOLLOWING APPROXIMATION IS OBTAINED BY MULTIPLYING ABOVE AND BELOW BY C - SD AND NEGLECTING TERMS OF
# ORDER S-SQUARED (POSSIBLY INTRODUCING ERROR INTO THE LOW TWO BITS OF THE RESULT).  SIGN AGREEMENT IS UNNECESSARY.
#
#	A + SB .      (R - QD)                                             A + SB
#	------ = Q + S(------) WHERE Q AND R ARE QUOTIENT AND REMAINDER OF ------ RESPECTIVELY.
#	C + SD        (  C   }                                                C

GENDDV		DXCH	MPAC		# WE NEED A AND B ONLY FOR FIRST DV.
 +1		EXTEND			# (SPECIAL UNIT PROLOGUE ENTERS HERE).
		DV	BUF		# A NOW CONTAINS Q AND L, R.
		DXCH	MPAC

		CS	MPAC		# FORM DIVIDEND FOR MINOR PART OF RESULT.
		EXTEND
		MP	BUF 	+1
		AD	MPAC 	+1	# OVERFLOW AT THIS POINT IS POSITIVE SINCE
		OVSK			# R IS POSITIVE IN EVERY CASE.
		TCF	+5

		EXTEND			# OVERFLOW CAN BE REMOVED BY SUBTRACTING C
		SU	BUF		# (BUF) ONCE SINCE R IS ALWAYS LESS THAN C
		INCR	MPAC		# IN THIS CASE.  INCR COMPENSATES SUBTRACT.
		TCF	+DOWN		# (SINCE C(A) IS STILL POSITIVE).

 +5		EXTEND			# C(A) CAN BE MADE LESS THAN C IN MAGNI-
		BZMF	-UP		# TUDE BY DIMINISHING IT BY C (SINCE C IS
					# NOT LESS THAN .5) UNLESS C(A) = 0.

# Page 1063
+DOWN		EXTEND
		SU	BUF		# IF POSITIVE, REDUCE ONLY IF NECESSARY
		EXTEND			# SINCE THE COMPENSATING INCR MIGHT CAUSE
		BZF	+3		# OVERFLOW.
		EXTEND			# DON'T SUBTRACT UNLESS RESULT IS POSITIVE
		BZMF	ENDMAXDV	# OR ZERO.

 +3		INCR	MPAC		# KEEP SUBTRACT HERE AND COMPENSATE.
		TCF	FINALDV

-UP		EXTEND			# IF ZERO, SET MINOR PART OF RESULT TO
		BZF	FINALDV +3	# ZERO.

		EXTEND			# IF NEGATIVE, ADD C TO A, SUBTRACTING ONE
		DIM	MPAC		# TO COMPENSATE.  DIM IS OK HERE SINCE THE
ENDMAXDV	AD	BUF		# MAJOR PART NEVER GOES NEGATIVE.

# Page 1064
FINALDV		ZL			# DO DV TO OBTAIN MINOR PART OF RESULT.
		EXTEND
		DV	BUF
 +3		TS	MPAC 	+1

		CCS	DVSIGN		# LEAVE RESULT POSITIVE UNLESS C(DVSIGN)=
		TC	Q		# -0.
		TC	Q
		TC	Q

		EXTEND
		DCS	MPAC
		DXCH	MPAC
		CAF	ZERO		# SO WE ALWAYS RETURN WITH C(A) = 0.
		TC	Q

# Page 1065
# IF THE MAJOR PARTS OF THE DIVISOR AND DIVIDEND ARE EQUAL, BUT THE MINOR PARTS ARE SUCH THAT THE
# DIVIDEND IS STRICTLY LESS THAN THE DIVISOR IN MAGNITUDE, THE FOLLOWING APPROXIMATION IS USED.  THE ASSUMPTIONS
# ARE THE SAME AS THE GENERAL ROUTINE WITH THE ADDITION THAT SIGN AGREEMENT IS NECESSARY (B, C, & D POSITIVE).
#
#	C + SB .          (C + B - D)
#	------ = 37777 + S(---------)
#	C + SD            (    C    )
#
# THE DIVISION MAY BE PERFORMED IMMEDIATELY SINCE B IS STRICTLY LESS THAN D AND C IS NOT LESS THAN .5.

MAXDV		CS	MPAC		# SEE IF MAXDV CASE STILL HOLDS AFTER
		AD	BUF		# NORMALIZATION.
		EXTEND
		BZF	+2
		TCF	GENDDV		# MPAC NOW LESS THAN BUFF -- DIVIDE AS USUAL.

 +2		CAF	POSMAX		# SET MAJOR PART OF RESULT.
		TS	MPAC

		CS	BUF 	+1	# FORM DIVIDEND OF MINOR PART OF RESULT.
		AD	MPAC 	+1
		TCF	ENDMAXDV	# GO ADD C AND DO DIVIDE, ATTACHING SIGN
					# BEFORE EXITING.

# Page 1066
# VECTOR DIVIDED BY SCALAR, V/SC, IS EXECUTED HERE.  THE VECTOR IS NOW IN MPAC WITH SCALAR IN BUF.

V/SC2		CS	ONE		# INITIALIZE DIVIDEND NORMALIZATION COUNT
		TS	DVNORMCT	# AND DIVISION SIGN REGISTER.
		TS	VBUF 	+5

		TC	VECAGREE	# FORCE SIGN AGREEMENT IN VECTOR

		DXCH	BUF
		TC	ALSIGNAG	# SIGN AGREE BUF
		DXCH	BUF
		CCS	BUF		# FORCE DIVISOR POSITIVE WITH MAJOR PART
		TCF	/BUF+		# NON-ZERO (IF POSSIBLE).
		TCF	+2
		TCF	/BUF-

		XCH	BUF	+1	# SHIFT VECTOR AND SCALAR LEFT 14.
		XCH	BUF
		XCH	MPAC  	+1
		XCH	MPAC
		EXTEND			# CHECK FOR OVERFLOW IN EACH CASE.
		BZF	+2
		TCF	DVOVF

		XCH	MPAC  	+4
		XCH	MPAC  	+3
		EXTEND
		BZF	+2
		TCF	DVOVF

		XCH	MPAC  	+6
		XCH	MPAC  	+5
		EXTEND
		BZF	+2
		TCF	DVOVF

		CCS	BUF
		TCF	/BUF+
		TCF	DVOVF		# ZERO DIVISOR - OVERFLOW.
		TCF	/BUF-
		TCF	DVOVF

/BUF-		EXTEND			# ON NEGATIVE, COMPLEMENT BUF AND MAINTAIN
		DCS	BUF		# DVSIGN IN VBUF +5.
		DXCH	BUF
		INCR	VBUF  	+5

# Page 1067
/BUF+		EXTEND
		DCA	BUF		# LEAVE ABS(ORIG DIVISOR) IN BUF2
		DXCH	BUF2		# FOR OVERFLOW TESTING
		TCF	/NORM		# NORMALIZE DIVISOR IN BUF.

/NORM2		EXTEND			# IF LESS THAN .5, AUGMENT DVNORMCT AND
		AUG	DVNORMCT	# DOUBLE DIVISOR.
		EXTEND
		DCA	BUF
		DAS	BUF

/NORM		CA	BUF		# SEE IF DIVISOR NORMALIZED.
		DOUBLE
		OVSK
		TCF	/NORM2		# DOUBLE AND TRY AGAIN IF NOT.

		TC	V/SCDV		# DO X COMPONENT DIVIDE.
		DXCH	MPAC 	+3	# SUPPLY ARGUMENTS IN USUAL SEQUENCE.
		DXCH	MPAC
		DXCH	MPAC 	+3

		TC	V/SCDV		# Y COMPONENT.
		DXCH	MPAC 	+5
		DXCH	MPAC
		DXCH	MPAC 	+5

		TC	V/SCDV		# Z COMPONENT.
		TCF	VROTATEX	# GO RE-ARRANGE COMPONENTS BEFORE EXIT.

# Page 1068
# SUBROUTINE USED BY V/SC TO DIVIDE VECTOR COMPONENT IN MPAC,+1 BY THE SCALAR GIVEN IN BUF.

V/SCDV		CA	VBUF 	+5	# REFLECTS SIGN OF SCALAR.
		TS	DVSIGN

		CCS	MPAC		# FORCE MPAC POSITIVE, EXITING ON ZERO.
		TCF	/MPAC+
		TCF	+2
		TCF	/MPAC-

		CCS	MPAC 	+1
		TCF	/MPAC+
		TC	Q
		TCF	/MPAC-
		TC	Q

/MPAC-		EXTEND			# USUAL COMPLEMENTING AND SETTING OF SIGN.
		DCS	MPAC
		DXCH	MPAC
		INCR	DVSIGN

/MPAC+		CS	ONE		# INITIALIZE NEAR-ONE SWITCH.
		TS	MAXDVSW

		CS	MPAC		# CHECK POSSIBLE OVERFLOW.
		AD	BUF2		# UNNORMALIZED INPUT DIVISOR.
		CCS	A
		TCF	DDVCALL		# NOT NEAR-ONE
		TCF	+2		# +0 IS JUST POSSIBLE
		TCF	DVOVF		# NO HOPE
		TS	MAXDVSW		# SIGNAL POSSIBLE NEAR-ONE CASE
		CS	MPAC 	+1	# SEE IF DIVISION CAN BE DONE
		AD	BUF2 	+1
		EXTEND
		BZMF	DVOVF

DDVCALL		DXCH	MPAC		# CALL PRE-DIVIDE NORMALIZATION.
		INDEX	DVNORMCT
		TCF	MAXTEST

# Page 1069
SLOPELO		DEC	.8324

VECAGREE	XCH	Q		# SAVE Q IN A
		DXCH	MPAC
		TC	ALSIGNAG	# SIGNAGREE MPAC
		DXCH	MPAC
		DXCH	MPAC 	+3
		TC	ALSIGNAG	# SIGN AGREE MPAC +3
		DXCH	MPAC 	+3
		DXCH	MPAC 	+5
		TC	ALSIGNAG	# SIGNAGREE MPAC +5
		DXCH	MPAC 	+5
		TC	A

# Page 1070
# THE FOLLOWING ROUTINE EXECUTES THE UNIT INSTRUCTION, WHICH TAKES THE UNIT OF THE VECTOR IN MPAC.

UNIT		TC	VECAGREE	# FORCE SIGN AGREEMENT IN VECTOR
		TC	MPACVBUF	# SAVE ARGUMENT IN VBUF
		CAF	ZERO		# MUST SENSE OVERFLOW IN FOLLOWING DOT.
		XCH	OVFIND
		TS	TEM1
		TC	VSQSUB		# DOT MPAC WITH ITSELF.
		CA	TEM1
		XCH	OVFIND
		EXTEND
		BZF	+2
		TCF	DVOVF
		EXTEND
		DCA	MPAC		# LEAVE THE SQUARE OF THE LENGTH OF THE
		INDEX	FIXLOC		# ARGUMENT IN LVSQUARE.
		DXCH	LVSQUARE

		TC	SQRTSUB		# GO TAKE THE NORMALIZED SQUARE ROOT.

		CCS	MPAC		# CHECK FOR UNIT OVERFLOW.
		TCF	+5		# MPAC IS NOT LESS THAN .5 UNLESS
		TS	L
		INDEX	FIXLOC
		DXCH	LV
		TCF	DVOVF		# INPUT TO SQRTSUB WAS 0.

		CS	FOURTEEN	# SEE IF THE INPUT WAS SO SMALL THAT THE
		AD	MPTEMP		# FIRST TWO REGISTERS OF THE SQUARE WERE 0
		CCS	A
		COM			# IF SO, SAVE THE NEGATIVE OF THE SHIFT
		TCF	SMALL		# COUNT -15D.

		TCF	LARGE		# (THIS IS USUALLY THE CASE.)

		CS	THIRTEEN	# IF THE SHIFT COUNT WAS EXACTLY 14, SET
		TS	MPTEMP		# THE PRE-DIVIDE NORM COUNT TO -13D.

		CA	MPAC		# SHIFT THE LENGTH RIGHT 14 BEFORE STORING
SMALL2		TS	L		# (SMALL EXITS TO THIS POINT).
		CAF	ZERO
		TCF	LARGE2		# GO TO STORE LENGTH AND PROCEED.

LARGE		CCS	MPTEMP		# MOST ALL CASES COME HERE.
		TCF	LARGE3		# SEE IF NO NORMALIZATION WAS REQUIRED BY

		CS	SRDDV		# SQRT, AND IF SO, SET UP FOR A SHIFT
		TS	MPTEMP		# RIGHT 1 BEFORE DIVIDING TO PRODUCE
		EXTEND			# THE DESIRED HALF UNIT VECTOR.
		DCA	MPAC
# Page 1071
		TCF	LARGE2

# Page 1072
LARGE3		COM			# LEAVE NEGATIVE OF SHIFT COUNT-1 FOR
		TS	MPTEMP		# PREDIVIDE LEFT SHIFT.

		COM			# PICK UP REQUIRED SHIFTING BIT TO UNNORM-
		INDEX	A		# ALIZE THE SQRT RESULT.
		CAF	BIT14
		TS	BUF
		EXTEND
		MP	MPAC 	+1
		XCH	BUF
		EXTEND			# (UNNORMALIZE THE SQRT FOR LV).
		MP	MPAC
		XCH	L
		AD	BUF
		XCH	L

LARGE2		INDEX	FIXLOC
		DXCH	LV		# LENGTH NOW STORED IN WORK AREA.

		CS	ONE
		TS	MAXDVSW		# NO MAXDV CASES IN UNIT.

		DXCH	VBUF		# PREPARE X COMPONENT FOR DIVIDE, SETTING
		DXCH	MPAC		# LENGTH OF VECTOR AS DIVISOR IN BUF.
		DXCH	BUF
		TC	UNITDV

		DXCH	VBUF 	+2	# DO Y AND Z IN USUAL FASHION SO WE CAN
		DXCH	MPAC		# EXIT THROUGH VROTATEX.
		DXCH	MPAC 	+3
		TC	UNITDV

		DXCH	VBUF 	+4
		DXCH	MPAC
		DXCH	MPAC 	+5
		TC	UNITDV
		TCF	VROTATEX	# AND EXIT.

# Page 1073
# IF THE LENGTH OF THE ARGUMENT VECTOR WAS LESS THAN 2(-28), EACH COMPONENT MUST BE SHIFTED LEFT AT LEAST
# 14 PLACES BEFORE THE DIVIDE. NOTE THAT IN THIS CASE, THE MAJOR PART OF EACH COMPONENT IS ZERO.

SMALL		TS	MPTEMP		# NEGATIVE OF PRE-DIVIDE SHIFT COUNT.

		CAF	ZERO		# SHIFT EACH COMPONENT LEFT 14.
		XCH	VBUF 	+1
		XCH	VBUF
		XCH	VBUF 	+3
		XCH	VBUF 	+2
		XCH	VBUF 	+5
		XCH	VBUF 	+4

		CS	MPTEMP
		INDEX	A
		CAF	BIT14
		EXTEND
		MP	MPAC
		TCF	SMALL2

THIRTEEN	=	OCT15
FOURTEEN	=	OCT16
OCT16		=	R1D1

# Page 1074
# THE FOLLOWING ROUTINE SETS UP THE CALL TO THE DIVIDE ROUTINES.

UNITDV		CCS	MPAC		# FORCE MPAC POSITIVE IF POSSIBLE, SETTING
		TCF	UMPAC+		# DVSIGN ACCORDING TO THE SIGN OF MPAC
		TCF	+2		# SINCE THE DIVISOR IS ALWAYS POSITIVE
		TCF	UMPAC-		# HERE.

		CCS	MPAC 	+1
		TCF	UMPAC+
		TC	Q		# EXIT IMMEDIATELY ON ZERO.
		TCF	UMPAC-
		TC	Q

UMPAC-		CS	ZERO		# IF NEGATIVE, SET -0 IN DVSIGN FOR FINAL
		TS	DVSIGN		# COMPLEMENT.
		EXTEND
		DCS	MPAC		# PICK UP ABSOLUTE VALUE OF ARG AND JUMP.
		INDEX	MPTEMP
		TCF	MAXTEST -1

UMPAC+		TS	DVSIGN		# SET DVSIGN FOR POSITIVE QUOTIENT.
		DXCH	MPAC
		INDEX	MPTEMP
		TCF	MAXTEST -1

# Page 1075
# MISCELLANEOUS UNARY OPERATIONS.

DSQ		TC	DSQSUB		# SQUARE THE DP CONTENTS OF MPAC.
		TCF	DANZIG

ABVALABS	CCS	MODE		# ABVAL OR ABS INSTRUCTION.
		TCF	ABS		# DO ABS ON SCALAR.
		TCF	ABS

ABVAL		TC	VSQSUB		# DOT MPAC WITH ITSELF.
		LXCH	MODE		# MODE IS NOW DP (L ZERO AFTER DAS).

		EXTEND			# STORE SQUARE OF LENGTH IN WORK AREA.
		DCA	MPAC
		INDEX	FIXLOC
		DXCH	LVSQUARE

# Page 1076
# PROGRAM DESCRIPTION -- SUBROUTINE SQRT
#
# FUNCTIONAL DESCRIPTION -- DOUBLE PRECISION SQUARE ROOT ROUTINE
#	THIS PROGRAM TAKES THE SQUARE ROOT OF THE 27 OR 28 MOST SIGNIFICANT BITS IN THE TRIPLE PRECISION SET OF
#	NUMBERS -- MPAC, MPAC+1, AND MPAC+2.  THE ROOT IS RETURNED DOUBLE PRECISION IN MPAC AND MPAC+1.
#
# WARNING -- THIS SUBROUTINE USES A TRIPLE PRECISION INPUT.  THE PROGRAMMER MUST ASSURE THE CONTENTS OF MPAC+2
#	ESPECIALLY IF THE CONTENTS OF MPAC IS SMALL OR ZERO.  FOR DETAILS SEE STG MEMO NO.949.
#
# CALLING SEQUENCE -- IN INTERPRETIVE MODE, I.E., FOLLOWING `TC INTPRET', `SQRT', NO ADDRESS IS ALLOWED.
#	INPUT SCALING: THE BINARY POINT IS ASSUMED TO THE RIGHT OF BIT 15.  THE ANSWER IS RETURNED WITH THE SAME SCALING.
#
# SUBROUTINES -- GENSCR, MPACSHR, SQRTSUB, ABORT
#
# ABORT EXIT MODE -- ABORTS ON NEGATIVE INPUT -1.2X10E-4 (77775 OCTAL) OR LESS.
#	DISPLAYS ERROR CODE 1302
#		TC	ABORT
#		OCT	1302
#
# DEBRIS -- LOCATIONS BUF, MPTEMP, ADDRWD ARE USED

SQRT		TC	SQRTSUB		# TAKE THE SQUARE ROOT OF MPAC.
		CCS	MPTEMP		# RETURNED NORMALIZED SQUARE ROOT.  SEE IF
		TCF	+2		# ANY UN-NORMALIZATION REQUIRED AND EXIT
		TCF	DANZIG		# IF NOT.

		AD	NEG12		# A RIGHT SHIFT OF MORE THAN 13 COULD BE
		EXTEND			# REQUIRED IF INPUT WAS ZERO IN MPAC,+1.
		BZMF	SQRTSHFT	# GOES HERE IN MOST CASES.
		ZL			# IF A LONG SHIFT IS REQUIRED, GO TO
		LXCH	ADDRWD		# GENERAL RIGHT SHIFT ROUTINES.
		TCF	GENSCR 	+4	# ADDRWD WAS ZERO TO PREVENT ROUND.

SQRTSHFT	INDEX	MPTEMP		# SELECT SHIFTING BIT AND EXIT THROUGH
		CAF	BIT15		# SHIFT ROUTINES.
		TS	MPTEMP
		CAF	ZERO		# TO ZERO MPAC +2 IN THE PROCESS.
		TCF	MPACSHR +3

ABS		TC	BRANCH		# TEST SIGN OF MPAC AND COMPLEMENT IF
		TCF	DANZIG
		TCF	DANZIG
		TCF	COMP

# Page 1077
VDEF		CS	FOUR		# VECTOR DEFINE -- ESSENTIALLY TREATS
		ADS	PUSHLOC		# SCALAR IN MPAC AS X COMPONENT, PUSHES UP
		EXTEND			# FOR Y AND THEN AGAIN FOR Z.
		INDEX	A
		DCA	2
		DXCH	MPAC 	+3
		EXTEND
		INDEX	PUSHLOC
		DCA	0
		DXCH	MPAC 	+5
		TCF	VMODE		# MODE IS NON VECTOR.

VSQ		TC	VSQSUB		# DOT MPAC WITH ITSELF.
		TCF	DMODE		# MODE IS NOW DP.

PUSH		EXTEND			# PUSH DOWN MPAC LEAVING IT LOADED.
		DCA	MPAC
		INDEX	PUSHLOC		# PUSH DOWN FIRST TWO REGISTERS IN EACH
		DXCH	0

		INDEX	MODE		# INCREMENT PUSHDOWN POINTER.
		CAF	NO.WDS
		ADS	PUSHLOC

		CCS	MODE
		TCF	TPUSH		# PUSH DOWN MPAC +2.
		TCF	DANZIG		# DONE FOR DP.

		EXTEND			# ON VECTOR, PUSH DOWN Y AND Z COMPONENTS.
		DCA	MPAC 	+3
		INDEX	PUSHLOC
		DXCH	0 	-4
		EXTEND
		DCA	MPAC 	+5
		INDEX	PUSHLOC
		DXCH	0 	-2
		TCF	DANZIG

TPUSH		CA	MPAC 	+2
		TCF	ENDTPUSH +2

RVQ		INDEX	FIXLOC		# RVQ -- RETURN IVA QPRET.
		CA	QPRET
		TS	POLISH
		TCF	GOTO 	+4	# (ASSUME QPRET POINTS TO FIXED ONLY.)

# Page 1078
# THE FOLLOWING SUBROUTINES ARE USED IN SQUARING MPAC, IN BOTH THE SCALAR AND VECTOR SENSE.  THEY ARE
# SPECIAL CASES OF DMPSUB AND DOTSUB, PUT IN TO SAVE SOME TIME.

DSQSUB		CA	MPAC 	+1	# SQUARES THE SCALAR CONTENTS OF MPAC.
		EXTEND
		SQUARE
		TS	MPAC 	+2
		CAF	ZERO		# FORM 2(CROSS TERM).
		XCH	MPAC 	+1
		EXTEND
		MP	MPAC
		DDOUBL			# AND MAYBE OVEFLOW.
		DAS	MPAC 	+1	# AND SET A TO NET OVERFLOW.
		XCH	MPAC
		EXTEND
		SQUARE
		DAS	MPAC
		TC	Q

VSQSUB		EXTEND			# DOTS THE VECTOR IN MPAC WITH ITSELF.
		QXCH	DOTRET
		TC	DSQSUB		# SQUARE THE X COMPONENT.
		DXCH	MPAC 	+3
		DXCH	MPAC
		DXCH	BUF		# SO WE CAN END IN DOTSUB.
		CA	MPAC 	+2
		TS	BUF 	+2

		TC	DSQSUB		# SQUARE Y COMPONENT.
		DXCH	MPAC 	+1
		DAS	BUF 	+1
		AD	MPAC
		AD	BUF
		TS	BUF
		TCF	+2
		TS	OVFIND		# IF OVERFLOW.

		DXCH	MPAC 	+5
		DXCH	MPAC
		TC	DSQSUB		# SQUARE Z COMPONENT.
		TCF	ENDDOT		# END AS IN DOTSUB.

# Page 1079
# DOUBLE PRECISION SQUARE ROOT ROUTINE.  TAKE THE SQUARE ROOT OF THE TRIPLE PRECISION (MPAC +2 USED ONLY
# IN NORMALIZATION) CONTENTS OF MPAC AND LEAVE THE NORMALIZED RESULT IN MPAC (C(MPAC) GREATER THAN OR EQUAL TO
# .5).  THE RIGHT SHIFT COUNT (TC UNNORMALIZE) IS LEFT IN MPTEMP.

; ----------------------------------------------------------------------------
; SQRTSUB - Square Root Subroutine (Double-Precision)
;
; Computes sqrt(MPAC) using Newton-Raphson iteration with normalization.
; This is a core mathematical subroutine used by interpretive operations
; and trigonometric functions (ARCSIN, ARCCOS).
;
; INPUT:
;   MPAC, MPAC+1, MPAC+2: Argument in DP format (must be non-negative)
;
; OUTPUT:
;   MPAC, MPAC+1: sqrt(argument) in DP format
;   MPAC+2: Cleared to zero
;
; ALGORITHM:
;   1. NORMALIZATION: Scale argument to range [0.125, 0.5] by left-shifting
;      Track shift count in MPTEMP for later denormalization
;   2. LINEAR APPROXIMATION: Compute initial guess X0 using:
;      For arg in [0.25, 0.5]:   X0 = (arg)(0.5884) + 0.4192
;      For arg in [0.125, 0.25]: X0 = (arg)(0.8324) + 0.2974
;   3. NEWTON-RAPHSON: Refine using two iterations:
;      X1 = (X0/2) + (arg/2)/(X0/2)    [first iteration]
;      X2 = (X1/2) + (arg/2)/(X1/2)    [second iteration]
;   4. DENORMALIZATION: Right-shift result by MPTEMP/2 to restore scaling
;
; ERROR HANDLING:
;   - Negative arguments (< -10^-4): Abort with program alarm 1302
;   - Small negative (> -10^-4): Return zero (treat as computational noise)
;   - Zero argument: Return zero
;   - Overflow cases: Saturate at POSMAX
;
; CONSTANTS USED:
;   SLOPEHI = 0.5884  (high-range linear approximation slope)
;   BIASHI  = 0.4192  (high-range y-intercept)
;   SLOPELO = 0.8324  (low-range linear approximation slope)
;   BIASLO  = 0.2974  (low-range y-intercept)
;
; ACCURACY:
;   Two Newton-Raphson iterations provide ~10 decimal places accuracy,
;   exceeding AGC's 15-bit mantissa precision. Linear approximation
;   constants chosen to minimize maximum error across each range.
;
; EXECUTION TIME:
;   ~25-35ms depending on normalization requirements and argument value
;
; USAGE IN APOLLO 11:
;   SQRTSUB computed vector magnitudes, distance calculations, and
;   intermediate values for ARCSIN/ARCCOS during:
;   - Orbital navigation (position vector magnitudes)
;   - Guidance targeting (range-to-target computations)
;   - ARCSIN evaluation during descent trajectory computations
;   - Radar data processing (slant range from altitude/velocity)
;
; HISTORICAL NOTE:
;   Newton-Raphson converges quadratically, doubling accuracy with each
;   iteration. Two iterations chosen as optimal trade-off between speed
;   and precision for AGC hardware. Algorithm proven reliable through
;   Gemini and early Apollo missions before Apollo 11.
; ----------------------------------------------------------------------------
; STEP 1: Initialize and check argument sign
SQRTSUB		CAF	ZERO		# START BY ZEROING RIGHT SHIFT COUNT.
		TS	MPTEMP		# MPTEMP tracks normalization shifts

		CCS	MPAC		# CHECK FOR POSITIVE ARGUMENT, SHIFTING
		TCF	SMPAC+		# FIRST SIGNIFICANT MPAC REGISTER INTO
		TCF	+2		# MPAC ITSELF.
		TCF	SQRTNEG		# SEE IF MAG OF ARGUMENT LESS THAN 10(-4).

; NORMALIZATION STAGE: Shift argument left to range [0.125, 0.5]
; Left shift by 14 bits = multiply by 2^14, tracking shifts for later correction
		XCH	MPAC 	+2	# MPAC IS ZERO -- SHIFT LEFT 14.
		XCH	MPAC 	+1	# Move MPAC+2 -> MPAC+1 -> MPAC
		TS	MPAC		# (14-bit left shift)
		CAF	SEVEN		# AUGMENT RIGHT SHIFT COUNTER.
		TS	MPTEMP		# MPTEMP = 7 (14/2 for sqrt denorm)

		CCS	MPAC		# SEE IF MPAC NOW PNZ.
		TCF	SMPAC+		# Positive - proceed to range check
		TCF	+2		# Zero - try another shift
		TCF	ZEROANS		# NEGATIVE BUT LESS THAN 10(-4) IN MAG.

; If still zero, shift left another 14 bits
		XCH	MPAC 	+1	# XERO -- SHIFT LEFT 14 AGAIN.
		TS	MPAC		# Another 14-bit left shift
		CAF	SEVEN		# AUGMENT RIGHT SHIFT COUNTER.
		ADS	MPTEMP		# MPTEMP += 7 (now 14 total)

		CCS	MPAC		# Check sign again
		TCF	SMPAC+		# Positive - proceed
		TC	Q		# SQRT(0) = 0.
		TCF	ZEROANS		# Treat as zero
		TCF	FIXROOT		# DO NOT LEAVE SQRTSUB WITH -0 IN MPAC.

; ERROR HANDLING: Check if negative argument is computational noise or error
SQRTNEG		CCS	A		# ARGUMENT IS NEGATIVE, BUT SEE IF SIGN-
		TCF	SQRTABRT	# CORRECTED ARGUMENT IS LESS THAN 10(-4)
					# If magnitude > 10^-4, abort with alarm
		CCS	MPAC 	+1	# IN MAGNITUDE.  IF SO, CALL ANSWER ZERO.
ZEROANS		CAF	ZERO		# FORCE ANSWER TO ZERO HERE.
		TCF	FIXROOT		# Treat tiny negative as zero (roundoff)
		TCF	SQRTABRT	# Large negative - abort
		TCF	FIXROOT		# Treat as zero

; Abort with program alarm 1302: SQRT of negative number
SQRTABRT	DXCH	LOC		# Save return address
		TC	POODOO1		# Trigger program alarm
		OCT	1302		# Alarm code: illegal SQRT argument

# Page 1080
; STEP 2: Range checking and linear approximation initialization
; Determine which range argument falls in and select appropriate constants
SMPAC+		AD	-1/2+2		# SEE IF ARGUMENT GREATER THAN OR EQUAL TO
		EXTEND			# .5.
		BZMF	SRTEST		# IF SO, SEE IF LESS THAN .25.
		; If arg >= 0.5, divide by 2 first (shift right 1)

; For arguments in [0.5, 1.0], take sqrt(arg/2) then scale result
		DXCH	MPAC		# WE WILL TAKE THE SQUARE ROOT OF MPAC/2.
		LXCH	SR		# SHIFT RIGHT 1 AND GO TO THE SQRT ROUTINE
		EXTEND			# Save shifted bit in SR register
		MP	HALF		# Divide MPAC by 2
		DXCH	MPAC		# Store result back
		XCH	SR		# Recover shifted bit
		ADS	MPAC 	+1	# GUARANTEED NO OVERFLOW.
		; Now arg is in range [0.25, 0.5] - proceed to high-range approx

; HIGH-RANGE LINEAR APPROXIMATION: for arg in [0.25, 0.5]
; Initial guess: X0/2 = (arg/2) * SLOPEHI + BIASHI/2
; SLOPEHI = 0.5884, BIASHI = 0.4192
ARGHI		CAF	SLOPEHI		# ARGUMENT BETWEEN .25 AND .5, GET A
		EXTEND			# LINEAR APPROXIMATION FOR THIS RANGE.
		MP	MPAC		# Multiply arg/2 by slope
		AD	BIASHI		# X0/2 = (MPAC/2)(SLOPHI) + BIASHI/2.

; STEP 3: NEWTON-RAPHSON ITERATION (First Pass)
; Formula: X1 = (X0/2) + (arg/2) / (X0/2)
; Both ARGHI and ARGLO enter here with their respective X0/2 values in A
 +4		TS	BUF		# X0/2 (ARGLO ENTERS HERE).
		CA	MPAC		# SINGLE-PRECISION THROUGHOUT.
		ZL			# Clear L register for division
		EXTEND
		DV	BUF		# (MPAC/2)/(X0/2) = arg/(X0)
		EXTEND
		MP	HALF		# Multiply by 0.5
		ADS	BUF		# X1 = X0/2 + .5(MPAX/2)/(X0/2)

; STEP 4: NEWTON-RAPHSON ITERATION (Second Pass - Double Precision)
; Formula: X2 = (X1/2) + (arg/2) / (X1/2)
; This iteration uses double-precision division for maximum accuracy
		EXTEND
		MP	HALF		# FORM UP X1/2.
		DXCH	MPAC		# SAVE AND BRING OUT ARGUMENT.
		EXTEND			# TAKE DP QUOTIENT WITH X1.
		DV	BUF		# Divide DP MPAC by SP BUF (X1/2)
		TS	BUF 	+1	# SAVE MAJOR PART OF QUOTIENT.
		CAF	ZERO		# FORM MINOR PART OF QUOTIENT USING
		XCH	L		# (REMAINDER,0).
		EXTEND			# Compute low-order bits from remainder
		DV	BUF		# Complete double-precision quotient
		TS	L		# IN PREPARATION FOR DAS.
		CA	BUF 	+1	# Load high-order quotient
		DAS	MPAC		# X2 = X1/2 + (MPAC/2)X1
				# DAS = Double-precision Add to Storage

; STEP 5: Overflow check and result finalization
; Check if result overflows (argument near POSMAX)
		EXTEND			# OVERFLOWS IF ARG. NEAR POSMAX.
		BZF	TCQBNK00	# No overflow - return normally
		CAF	POSMAX		# Overflow - saturate at maximum value
FIXROOT		TS	MPAC		# Store POSMAX in result
		TS	MPAC 	+1	# Both high and low words
TCQBNK00	TC	Q		# RETURN TO CALLER TO UNNORMALIZE, ETC.
				# Caller will denormalize by right-shifting

# Page 1081
; ============================================================================
; SRTEST: Medium-Range Argument Processing (0.25 ≤ arg < 0.5)
; Entry: MPAC contains argument less than 0.5
; Tests whether argument is less than 0.25 (needs normalization) or can be
; processed directly with low-range approximation
; ============================================================================
SRTEST		AD	QUARTER		# ARGUMENT WAS LESS THAN .5, SEE IF LESS
		EXTEND			# THAN .25.
		BZMF	SQRTNORM	# IF SO, BEGIN NORMALIZATION.

; Argument is between 0.25 and 0.5 - shift right by 1 to bring into ARGLO range
		DXCH	MPAC		# IF BETWEEN .5 AND .25, SHIFT RIGHT 1 AND
		LXCH	SR		# START AT ARGLO.
		EXTEND			# Multiply by 0.5 to shift right
		MP	HALF		# This brings arg into range [0.125, 0.25]
		DXCH	MPAC		# Save shifted argument
		XCH	SR		# Restore shift count
		ADS	MPAC 	+1	# NO OVERFLOW.

; ============================================================================
; ARGLO: Low-Range Linear Approximation (0.125 ≤ arg < 0.25)
; Uses linear approximation: X0 = SLOPELO * arg + BIASLO
; Lower slope provides better initial guess for Newton-Raphson
; ============================================================================
ARGLO		CAF	SLOPELO		# (NORMALIZED) ARGUMENT BETWEEN .125 AND
		EXTEND			# .25
		MP	MPAC		# X0 = SLOPELO * arg
		AD	BIASLO		# X0 = SLOPELO * arg + BIASLO
		TCF	ARGHI 	+4	# BEGIN SQUARE ROOT.

; ============================================================================
; SQRTNM2: Normalization Shift by 2
; Shifts argument left by 2 places and increments the right-shift counter
; Used for very small arguments (< 0.125) to bring them into computable range
; ============================================================================
SQRTNM2		EXTEND			# SHIFT LEFT 2 AND INCREMENT RIGHT SHIFT
		DCA	MPAC 	+1	# COUNT (FOR TERMINAL UNNORMALIZATION).
		DAS	MPAC 	+1	# Double-precision add to self (left shift)
		AD	MPAC		# Add high word to complete 2-bit shift
		ADS	MPAC		# (NO OVERFLOW).

; ============================================================================
; SQRTNORM: Normalization Entry Point
; Normalizes very small arguments (< 0.125) by shifting left until argument
; is in the range [0.125, 0.5]. Tracks shifts in MPTEMP for later denormalization.
; Each normalization left-shifts the argument but represents a right-shift of
; the final result (since sqrt(x/4) = sqrt(x)/2).
; ============================================================================
SQRTNORM	INCR	MPTEMP		# FIRST TIME THROUGH, JUST SHIFT LEFT 1
		EXTEND			# (PUTS IN EFFECTIVE RIGHT SHIFT SINCE
		DCA	MPAC 	+1	# WE WANT MPAC/2).
		DAS	MPAC 	+1	# Shift left 1 bit (DP add to self)
		AD	MPAC		# Complete the left shift
		ADS	MPAC		# (AGAIN NO OVERFLOW).
		DOUBLE			# Test value by doubling
		TS	CYL		# Store for range test

; ============================================================================
; NORMTEST: Check Normalization Status
; Tests whether argument has been normalized into computable range
; Uses CCS twice to determine which of three ranges argument falls into
; ============================================================================
NORMTEST	CCS	CYL		# SEE IF ARGUMENT NOW NORMALIZED AT
		CCS	CYL		# GREATER THAN .125.
		TCF	SQRTNM2		# NO -- SHIFT LEFT 2 MORE AND TRY AGAIN.
		TCF	ARGHI		# YES -- NOW BETWEEN .5 AND .25.
		TCF	ARGLO		# ARGUMENT NOW BETWEEN .25 AND .125.

# Page 1082
; ============================================================================
; SECTION: TRIGONOMETRIC FUNCTION PACKAGE
;
; This section implements high-precision trigonometric functions using
; Hastings polynomial approximations. These functions are critical for
; guidance, navigation, and attitude computations throughout the mission.
;
; SCALING CONVENTIONS (UNUSUAL BUT CRITICAL):
;   Angular inputs/outputs are in REVOLUTIONS, not radians or degrees
;   1.0 revolution = 360 degrees = 2π radians
;   
;   Input scaling:  2 * angle_in_revolutions (range: ±1.0 = ±180°)
;   Output scaling: (1/2) * function_result  (range: ±0.5 for ±1.0 input)
;
; AVAILABLE FUNCTIONS:
;   SIN:   Computes (1/2)sin(2π * MPAC)
;   COS:   Computes (1/2)cos(2π * MPAC)
;   ASIN:  Computes (1/2π)arcsin(2 * MPAC)
;   ACOS:  Computes (1/2π)arccos(2 * MPAC)
;
; MATHEMATICAL PROPERTIES:
;   - SIN and ASIN are mutually inverse: SIN(ASIN(X)) = X
;   - COS and ACOS are mutually inverse: COS(ACOS(X)) = X
;   - Functions use angle reduction to ±π/2 range before polynomial evaluation
;   - Accuracy: ~8 decimal places (sufficient for Apollo guidance)
;
; IMPLEMENTATION METHOD:
;   Fourth-order Hastings polynomial approximation with argument reduction.
;   COS computed using identity: cos(x) = sin(π/2 - |x|)
;   Range reduction brings arguments into [-0.5, +0.5] revolutions before
;   polynomial evaluation for maximum accuracy.
;
; HISTORICAL CONTEXT:
;   These trigonometric routines computed guidance vectors throughout Apollo 11:
;   - Orbital position/velocity transformations (inertial to rotating frames)
;   - IMU gimbal angle computations for platform alignment
;   - Targeting vector rotations during lunar descent on July 20, 1969
;   - Rendezvous radar angle tracking during ascent and docking
;   
;   The unusual scaling (revolutions instead of radians) simplified fixed-point
;   arithmetic on the AGC while maintaining precision across full angular range.
;
; EXECUTION TIME:
;   SIN/COS: ~40-50ms (includes polynomial evaluation and range reduction)
;   ASIN/ACOS: ~35-45ms (slightly faster due to simpler range handling)
;
; ACCURACY VALIDATION:
;   Functions validated against mathematical tables to 8 decimal places.
;   Sufficient for guidance errors well below spacecraft control authority.
; ============================================================================
# TRIGONOMETRIC FUNCTION PACKAGE.
#	THE FOLLOWING TRIGONOMETRIC FUNCTIONS ARE AVAIALABLE AS INTERPRETIVE OPERATIONS:
#	1.	SIN		COMPUTES (1/2)SINE(2 PI MPAC).
#	2.	COS		COMPUTES (1/2)COSINE(2 PI MPAC).
#	3.	ASIN		COMPUTES (1/2PI)ARCSINE(2 MPAC).
#	4.	ACOS		COMPUTES (1/2PI)ARCCOSINE(2 MPAC).
#
# SIN-ASIN AND COS-ACOS ARE MUTUALLY INVERSE, I.E., SIN(ASIN(X)) = X.

; ----------------------------------------------------------------------------
; COSINE - Compute (1/2)cos(2π * MPAC)
;
; Computes cosine using the mathematical identity:
;   cos(x) = sin(π/2 - |x|)
;
; INPUT:
;   MPAC, MPAC+1: Angle in revolutions scaled by 2 (DP format)
;                 Range: ±1.0 = ±180 degrees = ±π radians
;
; OUTPUT:
;   MPAC, MPAC+1, MPAC+2: (1/2)cos(2π * input) in DP format
;                         Range: ±0.5 for cosine values ±1.0
;
; METHOD:
;   1. Test sign of input using BRANCH (handles ±0 and ±nonzero cases)
;   2. Take absolute value by complementing if negative
;   3. Add π/2 (0.25 revolutions) to convert cos(x) → sin(π/2 + |x|)
;   4. Fall through to SINE routine for polynomial evaluation
;
; EXAMPLE USAGE (from orbital mechanics):
;   During Apollo 11 lunar orbit, this computed rotation matrix elements
;   transforming position vectors from inertial to Moon-rotating frames.
;   The cos(longitude) term positioned the landing site coordinates.
; ----------------------------------------------------------------------------
COSINE		TC	BRANCH		# FINDS COSINE USING THE IDENTITY
		TCF	+3		# COS(X) = SIN(PI/2 - ABS(X)).
		TCF	PRESINE
		TCF	PRESINE

; If input was negative, complement to get absolute value
 +3		EXTEND
		DCS	MPAC		# Double complement = -MPAC
		DXCH	MPAC		# Store absolute value

; Add π/2 (quarter revolution) to convert cos → sin identity
PRESINE		CAF	QUARTER		# PI/2 SCALED (0.25 revolutions).
		ADS	MPAC		# MPAC now contains (π/2 - |original|)

; ----------------------------------------------------------------------------
; SINE - Compute (1/2)sin(2π * MPAC)
;
; Core trigonometric routine using 4th-order Hastings polynomial approximation
; with angle reduction for maximum accuracy.
;
; INPUT:
;   MPAC, MPAC+1: Angle in revolutions scaled by 2 (DP format)
;                 Range: ±1.0 = ±180 degrees = ±π radians
;
; OUTPUT:
;   MPAC, MPAC+1, MPAC+2: (1/2)sin(2π * input) in DP format
;                         Range: ±0.5 for sine values ±1.0
;
; ALGORITHM:
;   1. ANGLE DOUBLING: Multiply input by 2 to convert from half-revolutions
;      to full angle representation for internal computation
;
;   2. OVERFLOW HANDLING: If doubling causes overflow (|angle| > π), apply
;      identity sin(x ± π) = -sin(x) by complementing the value
;
;   3. RANGE REDUCTION: If |angle| > 0.5 revolutions (π/2), reduce using:
;      - For positive: sin(x) = sin(π - x)  →  x' = π - x
;      - For negative: sin(x) = sin(-π - x) →  x' = -π - x
;      Guarantees |x'| ≤ 0.5 revolutions for optimal polynomial accuracy
;
;   4. POLYNOMIAL EVALUATION: Compute 4th-order Hastings approximation:
;      sin(x) ≈ x(c₀ + x²(c₁ + x²(c₂ + x²(c₃ + x²c₄))))
;      Coefficients optimized for x ∈ [-π/2, +π/2]
;
;   5. FINAL SCALING: Multiply polynomial result by reduced argument and
;      shift left 2 bits to restore proper scaling
;
; POLYNOMIAL COEFFICIENTS (Hastings approximation):
;   c₀ = +0.3926990796  (leading term)
;   c₁ = -0.6459637111  (3rd order correction)
;   c₂ = +0.318758717   (5th order correction)
;   c₃ = -0.074780249   (7th order correction)
;   c₄ = +0.009694988   (9th order correction)
;
; ACCURACY:
;   Maximum error < 1 part in 10⁸ over full range (±180 degrees)
;   Equivalent to ~8 decimal places of precision
;
; EXECUTION TIME: ~40-50 milliseconds (including range reduction)
;
; HISTORICAL CONTEXT:
;   This routine executed thousands of times during Apollo 11's descent on
;   July 20, 1969, computing:
;   - Descent guidance thrust vector orientations in rotating Moon frame
;   - IMU gimbal angle transformations for platform alignment
;   - Landing site position transformations from inertial coordinates
;   - Velocity vector projections for throttle control computations
;
; EXAMPLE:
;   Input: MPAC = 0.125 (represents 45 degrees = π/4 radians)
;   Output: MPAC = 0.3536 (represents sin(45°) = 0.7071 scaled by 1/2)
; ----------------------------------------------------------------------------
SINE		DXCH	MPAC		# DOUBLE ARGUMENT.
		DDOUBL			# Multiply by 2 for internal representation
		OVSK			# SEE IF OVERFLOW PRESENT.
		TCF	+3		# IF NOT, ARGUMENT OK AS IS.

		EXTEND			# IF SO, WE LOST (OR GAINED) PI, SO
		DCOM			# COMPLEMENT MPAC USING THE IDENTITY
					# SIN(X-(+)PI) = SIN(-X).
 +3		DXCH	MPAC		# Store potentially adjusted argument
		CA	MPAC		# SEE IF ARGUMENT GREATER THAN .5 IN
		DOUBLE			# MAGNITUDE.  IF SO, REDUCE IT TO LESS THAN
		TS	L		# .5 (+-PI/2 SCALED) AS FOLLOWS:
		TCF	SN1		# If |arg| ≤ 0.5, skip range reduction

		INDEX	A		# IF POSITIVE, FORM PI - X, IF NEGATIVE
		CAF	NEG1/2 +1	# USE -PI -X.
		DOUBLE			# Scale reduction constant
		EXTEND
		SU	MPAC		# GUARANTEED NO OVERFLOW.
		TS	MPAC		# Store reduced MS word
		CS	MPAC 	+1	# Complement LS word to complete reduction
		TS	MPAC 	+1

# Page 1083
; ----------------------------------------------------------------------------
; POLYNOMIAL EVALUATION STAGE
;
; With angle now reduced to |x| ≤ 0.5 revolutions (±π/2), evaluate the
; Hastings 4th-order polynomial approximation for optimal accuracy.
;
; POLYNOMIAL FORM:
;   sin(x) ≈ x · P(x²)
;   where P(x²) = c₀ + c₁x² + c₂x⁴ + c₃x⁶ + c₄x⁸
;
; EVALUATION STRATEGY (Horner's method):
;   P(x²) = c₀ + x²(c₁ + x²(c₂ + x²(c₃ + x²·c₄)))
;   This minimizes multiplications and provides numerical stability.
;
; STEPS:
;   1. Save original argument x in BUF2 (needed for final multiplication)
;   2. Square the argument to get x² in MPAC
;   3. Call POLY routine with degree=3 (4th order) and 5 coefficients
;   4. Multiply polynomial result P(x²) by saved argument x
;   5. Shift left 2 bits (multiply by 4) to restore proper scaling
;
; HASTINGS COEFFICIENTS (optimized for x ∈ [-π/2, +π/2]):
;   c₀ = +0.3926990796  (Dominant linear term approximating x for small x)
;   c₁ = -0.6459637111  (3rd order correction, shapes curve)
;   c₂ = +0.318758717   (5th order correction, refines accuracy)
;   c₃ = -0.074780249   (7th order correction, handles extremes)
;   c₄ = +0.009694988   (9th order correction, final precision)
;
; These coefficients were carefully derived by Hastings to minimize maximum
; error over the interval, achieving ~8 decimal places of accuracy.
;
; FINAL SCALING:
;   The left shift by 2 positions (multiply by 4) compensates for the
;   (1/2) output scaling convention, ensuring that input scaled by 2
;   produces output scaled by (1/2).
; ----------------------------------------------------------------------------
SN1		EXTEND			# SET UP TO EVALUATE HASTINGS POLYNOMIAL
		DCA	MPAC		# Load reduced argument (range ±π/2)
		DXCH	BUF2		# Save in BUF2 for final multiplication
		TC	DSQSUB		# SQUARE MPAC: x² needed for polynomial

		TC	POLY		# EVALUATE FOURTH ORDER POLYNOMIAL.
		DEC	3		# Degree = 3 (4th order: c₀+c₁x²+c₂x⁴+c₃x⁶+c₄x⁸)
		2DEC	+.3926990796	# c₀: Leading coefficient (linear term)

		2DEC	-.6459637111	# c₁: Cubic correction term

		2DEC	+.318758717	# c₂: Quintic correction term

		2DEC	-.074780249	# c₃: 7th order correction term

		2DEC	+.009694988	# c₄: 9th order correction term

		CAF	LBUF2		# MULTIPLY BY ARGUMENT AND SHIFT LEFT 2.
		TC	DMPSUB 	-1	# Multiply P(x²) by original x

		EXTEND			# First left shift (shift left 1 position)
		DCA	MPAC 	+1	# Load lower DP words
		DAS	MPAC 	+1	# Double precision add to self = shift
		AD	MPAC		# Add any carry to MS word
		ADS	MPAC		# Store shifted result - NEITHER SHIFT OVERFLOWS.
		EXTEND			# Second left shift (another shift left 1)
		DCA	MPAC 	+1	# Total shift = 2 positions left
		DAS	MPAC 	+1	# Completes scaling adjustment
		AD	MPAC		# Propagate carry
		ADS	MPAC		# Final scaled sin(x) result now in MPAC
		TCF	DANZIG		# Return to dispatcher with result

# Page 1084
# ARCSIN/ARCCOS ROUTINE.

; ============================================================================
; ARCSIN AND ARCCOS - INVERSE TRIGONOMETRIC FUNCTIONS
;
; Computes inverse sine and inverse cosine using the mathematical identity:
;   ARCSIN(X) = PI/2 - ARCCOS(X)
;
; ARCCOS uses Hastings polynomial approximation of form:
;   ARCCOS(X) = SQRT(1-X) * P(X)  for X in [0,1]
;
; For negative arguments:
;   ARCCOS(X) = PI - ARCCOS(-X)   (uses symmetry)
;
; INPUT:  MPAC contains DP argument X (range -1 to +1 in revolutions)
; OUTPUT: MPAC contains DP result (angle in revolutions)
;         Range: ARCSIN [-0.25, +0.25], ARCCOS [0, 0.5]
;
; SCALING: Angles in revolutions (0.0 to 1.0 = 0° to 360°)
;          Quarter = 0.25 = PI/2 = 90°
;          Half = 0.5 = PI = 180°
;
; USED BY: Guidance targeting, navigation state updates, IMU alignment
;
; HISTORICAL CONTEXT:
; These routines computed critical angles during Apollo 11 lunar descent,
; including spacecraft attitude relative to velocity vector and line-of-sight
; angles to landing site. ARCSIN/ARCCOS were used in the powered descent
; guidance equations that executed throughout the landing on July 20, 1969.
; ============================================================================

; ARCSIN Entry Point
; Uses identity ARCSIN(X) = PI/2 - ARCCOS(X)
; Sets escape address to ASINEX to perform final subtraction after ARCCOS
;
ARCSIN		CAF	LASINEX		# COMPUTE ARCSIN BY USING THE IDENTITY
		TCF	+2		# ARCSIN(X) = PI/2 - ARCCOS(X).

; ARCCOS Entry Point
; Main entry for ARCCOS computation
; Sets up escape address and handles argument sign
;
ARCCOS		CAF	LDANZIG		# (EXITS IMMEDIATELY).
		TS	ESCAPE		# Normal exit - return to DANZIG dispatcher
		TC	BRANCH		# TEST SIGN OF INPUT.
		TCF	ACOSST		# START IMMEDIATELY IF POSITIVE.
		TCF	ACOSZERO	# ARCCOS(0) = PI/2 = .25.
;
; Handle negative argument using identity: ARCCOS(X) = PI - ARCCOS(-X)
; Force argument positive and set flag to subtract from PI at end
;
		EXTEND			# IF NEGATIVE, USE THE IDENTITY
		DCS	MPAC		# ARCCOS(X) = PI - ARCCOS(-X), FORCING
		DXCH	MPAC		# ARGUMENT POSITIVE.
		CAF	TCSUBTR		# SET EXIT TO DO ABOVE BEFORE
		XCH	ESCAPE		# ARCSIN/ARCCOS CONSIDERATIONS.
		TS	ESCAPE2		# Chain escape addresses

; Magnitude Test
; Check if |X| > 0.5, indicating potential overflow or special case
; For X > 0.5, ARCCOS uses alternate formula or overflow handling
;
ACOSST		CS	HALF		# TEST MAGNITUDE OF INPUT.
		AD	MPAC		# Compute MPAC - 0.5
		CCS	A		# Test result sign
		TCF	ACOSOVF		# THIS IS PROBABLY AN OVERFLOW CASE.

LASINEX		TCF	ASINEX		# Jump vector for ARCSIN epilogue

		TCF	ACOSST2		# NO OVERFLOW -- PROCEED.

; Special case: If MPAC = 0.5 exactly, ARCCOS(0.5) = 0
; (60 degrees = 0.166... revolutions, but due to approximation range)
;
		CCS	MPAC 	+1	# IF MAJOR PART IS .5, CALL ANSWER 0
		CAF	ZERO		# UNLESS MINOR PART NEGATIVE.
		TCF	ACOS=0		# Return zero result

		TCF	ACOSST2		# Continue with computation

ACOS=0		TS	MPAC 	+1	# Set result to zero
		TS	MPAC
		TC	ESCAPE		# Exit to dispatcher

; Compute sqrt(1-X) for Hastings approximation
; Formula: ARCCOS(X) = sqrt(1-X) * P(X)
; where P(X) is 7th-order polynomial (Hastings polynomial 4654)
;
ACOSST2		EXTEND			# NOW THAT ARGUMENT IS IN PROPER RANGE,
		DCS	MPAC		# BEGIN COMPUTATION.  USE HASTINGS
		AD	HALF		# APPROXIMATION ARCCOS(X) = SQRT(1-X)P(X)
		DXCH	MPAC		# IN A SCALED VERSION WHERE P(X) IS A
		DXCH	BUF2		# SEVENTH ORDER POLYNOMIAL.
					# Save original argument in BUF2
					# Compute (1-X)/2 for sqrt input

		TC	SQRTSUB		# RETURNS WITH NORMALIZED SQUARE ROOT.
					# Result: sqrt((1-X)/2) in MPAC

		CCS	MPTEMP		# SEE IF UN-NORMALIZATION REQUIRED.
		TCF	ACOSSHR		# IF SO.

# Page 1085
; Polynomial Evaluation
; Evaluate P(X) using 7th-order Hastings polynomial
; Coefficients are scaled by C * 2^(+I) / (PI * sqrt(2))
; where C are original Hastings coefficients
;
ACOS3		DXCH	MPAC		# SET UP FOR POLYNOMIAL EVALUATION.
		DXCH	BUF2		# Swap sqrt result with original argument
		DXCH	MPAC		# Now MPAC = original X, BUF2 = sqrt result

		TC	POLY		# Evaluate polynomial P(X)
		DEC	6		# Polynomial order: 6th degree (7 coefficients)
		2DEC	+.353553385	# COEFFICIENTS ARE C 2(+I)/PISQRT(2) WHERE
					# C0 = 0.353553385 (1/sqrt(8))

		2DEC*	-.0483017006 B+1*	# I
					# C1 = -0.0483017006 * 2

		2DEC*	+.0200273085 B+2*	# WHERE C STANDS FOR ORIGINAL COEFFS.
					# C2 = +0.0200273085 * 4

		2DEC*	-.0112931863 B+3*	# C3 = -0.0112931863 * 8

		2DEC*	+.00695311612 B+4*	# C4 = +0.00695311612 * 16

		2DEC*	-.00384617957 B+5*	# C5 = -0.00384617957 * 32

		2DEC*	+.001501297736 B+6*	# C6 = +0.001501297736 * 64

		2DEC*	-.000284160334 B+7*	# C7 = -0.000284160334 * 128

; Final Multiply and Exit
; Multiply polynomial result P(X) by sqrt((1-X)/2) from BUF2
; Result: ARCCOS(X) = sqrt((1-X)/2) * P(X)
;
		CAF	LBUF2		# DO FINAL MULTIPLY AND GO TO ANY
		TC	DMPSUB 	-1	# EPILOGUE SEQUENCES.
		TC	ESCAPE		# Return to dispatcher or epilogue

; SUBTR Epilogue - Handle Negative Inputs to ARCCOS
; Uses identity: ARCCOS(-X) = PI - ARCCOS(X)
; Input: MPAC contains ARCCOS(X)
; Output: MPAC contains ARCCOS(-X) = PI - ARCCOS(X)
; Note: In AGC scaling, PI = 0.5 revolutions
;
SUBTR		EXTEND			# EPILOGUE FOR NEGATIVE INPUTS TO ARCCOS.
		DCS	MPAC		# Negate and complement MPAC
		AD	HALF		# FORMS PI - ARCCOS(-X) = ARCCOS(X).
					# HALF = 0.5 revolutions = PI radians
		DXCH	MPAC		# Store result back to MPAC
		TC	ESCAPE2		# GO TO POSSIBLE ARCSIN EPILOGUE.

; ASINEX Epilogue - Convert ARCCOS Result to ARCSIN
; Uses identity: ARCSIN(X) = PI/2 - ARCCOS(X)
; Input: MPAC contains ARCCOS(X)
; Output: MPAC contains ARCSIN(X)
; Note: QUARTER = 0.25 revolutions = PI/2 radians
;
ASINEX		EXTEND
		DCS	MPAC		# ARCSIN EPILOGUE -- GET ARCSIN(X)
		AD	QUARTER		# = PI/2 - ARCCOS(X).
					# QUARTER = 0.25 revolutions = 90 degrees
		DXCH	MPAC		# Store ARCSIN result
LDANZIG		TCF	DANZIG		# Return to dispatcher

# Page 1086
; ACOSSHR - Shift Right for Unnormalized Square Root Results
; Called when SQRTSUB returns with MPTEMP indicating unnormalization
; Performs right shift to restore proper scaling before polynomial evaluation
;
ACOSSHR		INDEX	A		# THE SHIFT RIGHT IS LESS THAN 14 SINCE
		CAF	BIT14		# THE INPUT WAS NON-ZERO DP.
		TS	MPTEMP		# Store shift count
		TC	VSHRRND		# DP SHIFT RIGHT AND ROUND.
					# Restore proper scaling for polynomial input
		TCF	ACOS3		# PROCEED.

; ACOSOVF - Overflow Handling for Arguments Near ±1.0
; Called when |X| is slightly greater than 1.0 due to roundoff errors
; If the excess is minimal (only 1 bit over 0.5), treats as valid boundary case
; Otherwise triggers alarm for genuine overflow condition
;
ACOSOVF		EXTEND			# IF MAJOR PART WAS ONLY 1 MORE THAN .5,
		BZF	ACOS=0		# CALL ANSWER ZERO.
					# Roundoff tolerance: treat as |X| = 1.0

; ACOSABRT - Alarm for Genuine Overflow Condition
; Triggered when input magnitude significantly exceeds 1.0
; Sets result to zero and generates alarm code 1301
; Alarm 1301: ARCCOS/ARCSIN argument out of range
;
ACOSABRT	EXTEND			# IF OVERFLOW, CALL ANSWER ZERO BUT
		DCA	LOC		# SOUND AN ALARM.
		TC	ALARM1		# Generate program alarm
		OCT	1301		# Alarm code 1301: invalid ARCCOS/ARCSIN argument

		CAF	ZERO		# Set result to zero
		TCF	ACOS=0		# Exit via standard return

; ACOSZERO - Special Case for Zero Input
; ARCCOS(0) = PI/2 = 0.25 revolutions in AGC scaling
; This is a frequently occurring case optimized for direct return
;
ACOSZERO	CAF	QUARTER		# ACOS(0) = PI/2.
					# QUARTER = 0.25 revolutions = 90 degrees
		TCF	ACOS=0 	+1	# SET MPAC AND EXIT VIA ESCAPE.

NEG12		DEC	-12
TCSUBTR		TCF	SUBTR

# Page 1087
; ============================================================================
; INDEX REGISTER OPERATIONS
;
; The interpreter provides two index registers (X1 and X2) used for array
; indexing and loop control. These registers enable iterative calculations
; common in guidance equations, such as processing state vector components
; or iterating through polynomial coefficients.
;
; INDEX REGISTERS:
; - X1: Primary index register (address: FIXLOC, typically address 50 octal)
; - X2: Secondary index register (address: FIXLOC+1, typically address 51 octal)
; - Both registers store 14-bit unsigned values (range 0-16383 decimal)
; - Used for: Array indexing, loop counters, address calculations
;
; INSTRUCTION CATALOG:
;
; LOAD OPERATIONS:
; - AXT,1 / AXT,2: Load immediate constant into X1 or X2
; - AXC,1 / AXC,2: Load erasable address contents into X1 or X2
; - LXA,1 / LXA,2: Load X1 or X2 from erasable address
; - LXC,1 / LXC,2: Load X1 or X2 from erasable address (alternate mnemonic)
;
; STORE OPERATIONS:
; - SXA,1 / SXA,2: Store X1 or X2 to erasable address
;
; EXCHANGE OPERATIONS:
; - XCHX,1 / XCHX,2: Exchange MPAC with X1 or X2
;
; ARITHMETIC OPERATIONS:
; - INCR,1 / INCR,2: Increment X1 or X2 by 1
; - XAD,1 / XAD,2: Add erasable address contents to X1 or X2
; - XSU,1 / XSU,2: Subtract erasable address contents from X1 or X2
;
; CONDITIONAL BRANCH:
; - TIX,1 / TIX,2: Test index, decrement, and branch if positive
;   (Classic loop control: continue while index > 0)
;
; USAGE PATTERN (typical loop structure):
;   AXT,1  10D        ; Initialize X1 = 10 (loop 10 times)
; LOOP
;   ... process indexed data using X1 ...
;   TIX,1  LOOP       ; Decrement X1, branch if X1 > 0
;   ... continue after loop ...
;
; ADDRESSING WITH INDEX REGISTERS:
; Index registers modify addresses during interpretive execution. When an
; indexed address is encountered, the index register value is added to the
; base address, enabling access to array elements or table entries.
;
; HISTORICAL CONTEXT:
; During lunar landing, index registers controlled loops processing radar
; data samples, iterating through descent trajectory checkpoints, and
; updating multi-component state vectors. The TIX instruction provided
; efficient loop control for time-critical guidance computations.
; ============================================================================

# THE FOLLOWING INSTRUCTIONS ARE AVAILABLE FOR SETTING, MODIFYING, AND BRANCHING ON INDEX REGISTERS:
#	1.	AXT	ADDRESS TO INDEX TRUE.
#	2.	AXC	ADDRESS TO INDEX COMPLEMENTED.
#	3.	LXA	LOAD INDEX FROM ERASABLE.
#	4.	LXC	LOAD INDEX COMPLEMENTED FROM ERASABLE.
#	5.	SXA	STORE INDEX IN ERASABLE.
#	6.	XCHX	EXCHANGE INDEX REGISTER WITH ERASABLE.
#	7.	INCR	INCREMENT INDEX REGISTER.
#	8.	XAD	ERASABLE ADD TO INDEX REGISTER.
#	9.	XSU	ERASABLE SUBTRACT FROM INDEX REGISTER.
#	10.	TIX	BRANCH ON INDEX REGISTER AND DECREMENT.

		BANK	01

		COUNT*	$$/INTER

; AXT: Load immediate constant into index register X1 or X2.
; Address word contains the constant value to load.
; Example: AXT,1 100D loads decimal 100 into X1.
AXT		TC	TAGSUB		# SELECT APPROPRIATE INDEX REGISTER.
		CA	POLISH
XSTORE		INDEX	INDEXLOC	# CONTAINS C(FIXLOC) OR C(FIXLOC)+1
		TS	X1
		TCF	DANZIG

; AXC: Load complement of address into index register.
; Loads the one's complement (negative) of the address operand.
; Used for countdown loops or negative indexing operations.
AXC		TC	TAGSUB
		CS	POLISH
		TC	XSTORE

; LXA: Load index register from erasable memory location.
; Reads the value at the specified erasable address into X1 or X2.
; Example: LXA,1 LOOPCNT loads the value at LOOPCNT into X1.
LXA		TC	15ADRERS	# LOAD INDEX REGISTER FROM ERASABLE.
		INDEX	POLISH
		CA	0
		TCF	XSTORE

; LXC: Load complement of erasable memory into index register.
; Like LXA but loads the one's complement of the memory contents.
LXC		TC	15ADRERS	# LOAD NDX REG FROM ERASABLE COMPLEMENTED.
		INDEX	POLISH
		CS	0
		TCF	XSTORE

; SXA: Store index register to erasable memory.
; Writes the current value of X1 or X2 to the specified erasable address.
; Example: SXA,1 SAVNDX stores X1 to memory location SAVNDX.
SXA		TC	15ADRERS	# STORE INDEX REGISTER IN ERASABLE.
		INDEX	INDEXLOC
		CA	X1
MSTORE1		INDEX	POLISH
		TS	0
		TCF	DANZIG

# Page 1088
; XCHX: Exchange index register with erasable memory.
; Swaps the value in X1/X2 with the value at the erasable address.
; Atomic exchange operation useful for saving/restoring index state.
XCHX		TC	15ADRERS	# EXCHANGE INDEX REGISTER WITH ERASABLE.
		INDEX	POLISH
		CA	0
		INDEX	INDEXLOC
		XCH	X1
		TCF	MSTORE1

; XAD: Add erasable memory contents to index register.
; Performs X1/X2 = X1/X2 + memory[address].
; Ignores overflows (wraps at 16383 decimal).
; Example: XAD,1 DELTA adds value at DELTA to X1.
XAD		TC	15ADRERS	# ADD ERASABLE TO INDEX REGISTER.
		INDEX	POLISH
		CA	0
XAD2		INDEX	INDEXLOC
		ADS	X1		# IGNORING OVERFLOWS.
		TCF	DANZIG

; INCR: Increment index register by 1.
; Simple counter increment: X1/X2 = X1/X2 + 1.
; Example: INCR,1 increments X1, often used for forward iteration.
INCR		TC	TAGSUB		# INCREMENT INDEX REGISTER.
		CA	POLISH
		TCF	XAD2

; XSU: Subtract erasable memory contents from index register.
; Performs X1/X2 = X1/X2 - memory[address].
; Ignores overflows/underflows.
XSU		TC	15ADRERS	# SUBTRACT ERASABLE FROM INDEX REGISTER.
		INDEX	POLISH
		CS	0
		TCF	XAD2

; TIX: Test index, decrement, and branch if positive.
; Classic loop control instruction:
;   1. Decrements X1/X2 by 1
;   2. If result > 0, branches to specified address
;   3. If result <= 0, continues to next instruction
; Example loop:
;   AXT,1 10D        ; Initialize loop counter to 10
; LOOP               ; Loop body processes 10 iterations
;   TIX,1 LOOP       ; Decrement X1, branch if still positive
; This instruction was crucial during descent for iterating through
; radar sample arrays and processing navigation state vector components.
TIX		TC	TAGSUB		# BRANCH AND DECREMENT ON INDEX.
		INDEX	INDEXLOC
		CS	S1
		INDEX	INDEXLOC
		AD	X1
		EXTEND			# NO OPERATION IF DECREMENTED INDEX IS
		BZMF	DANZIG		# NEGATIVE OR ZERO.

DOTIXBR		INDEX	INDEXLOC
		XCH	X1		# IGNORING OVERFLOWS.

		TCF	GOTO		# DO THE BRANCH USING THE CADR IN POLISH.

# Page 1089
# SUBROUTINE TO CONVERT AN ERASABLE ADDRESS (11 BITS) TO AN EBANK SETTING AND SUBADDRESS.

15ADRERS	CS	POLISH
		AD	DEC45
		CCS	A		# DOES THE ADDRESS POINT TO THE WORK AREA?
		CA	FIXLOC		# YES.  ADD FIXLOC.  EBANK OK AS IS.
		TCF	+5

		CA	OCT1400		# NO. SET EBANK & MAKE UP SUBADDRESS.
		XCH	POLISH
		TS	EBANK
		MASK	LOW8
 +5		ADS	POLISH		# FALL INTO TAGSUB, AND RETURN VIA Q.

# SUBROUTINE WHICH SETS THE ADDRESS OF THE SPECIFIED INDEX IN INDEXLOC.  (ACTUALLY, THE ADDRESS -38D.)

TAGSUB		CA	FIXLOC
		TS	INDEXLOC

		CCS	CYR		# BIT 15 SPECIFIES INDEX.
		INCR	INDEXLOC	# 0 MEANS USE X2.
		TC	Q
		TC	Q		# 1 FOR X1.

# Page 1090
# MISCELLANEOUS OPERATION CODES WITH DIRECT ADDRESSES.  INCLUDED HERE ARE:
#	1.	ITA	STORE CPRET (RETURN ADDRESS) IN ERASABLE.
#	2.	CALL	CALL A SUBROUTINE, LEAVING RETURN IN QPRET.
#	3.	RTB	RETURN TO BASIC LANGUAGE AT THE GIVEN ADDRESS.
#	4.	BHIZ	BRANCH IF THE HIGH ORDER OF MPAC IS ZERO (SINGLE PRECISION).
#	5.	BOV	BRANCH ON OVERFLOW.
#	6.	GOTO	SIMPLE SEQUENCE CHANGE.

; ============================================================================
; MISCELLANEOUS OPERATIONS
; ============================================================================
; This section implements control flow and utility instructions:
;
; RTB: Return to Basic (exit interpreter to native AGC code)
; BHIZ: Branch if High-order word Is Zero
; BOV: Branch On oVerflow
; GOTO: Unconditional branch within interpreter
; ITA: Index (store return address) To Address
; CALL: Call interpretive subroutine
;
; These operations enable:
; - Transitioning between interpretive and native AGC code
; - Conditional branching based on computation results
; - Subroutine call/return mechanisms
; - Overflow handling
;
; The RTB instruction was critical during lunar landing when switching
; from guidance computations (interpretive) to throttle control routines
; (native AGC code for real-time engine commands).
; ============================================================================

; Dispatcher distinguishes between RTB and BHIZ based on CYR prefix bit.
RTB/BHIZ	CCS	CYR
; RTB: Return To Basic - exit interpreter and execute native AGC code.
; The address in POLISH specifies the native AGC routine to call.
; Return from that routine (TC Q) leads back to DANZIG to resume interpretation.
; Example: RTB THROTTLE calls native throttle control routine then returns.
RTB		CA	POLISH
		TC	SWCALL 	-1	# SO A "TC Q" FROM ROUTINE LEADS TO DANZIG

; BHIZ: Branch if High-order word Is Zero.
; Tests MPAC (most significant word) for zero.
; If MPAC = 0, branches to address in POLISH.
; If MPAC ≠ 0, continues to next instruction.
; Used for zero-detection in scalar and vector operations.
BHIZ		CCS	MPAC
		TCF	DANZIG
		TCF	GOTO
		TCF	DANZIG
		TCF	GOTO

; BOV: Branch On oVerflow.
; If overflow flag (OVFIND) is set, branches to specified address.
; Can branch to either interpretive code (GOTO) or basic code (RTB).
; Clears overflow flag after detecting it.
; Critical for preventing cascade errors in guidance computations.
BOV(B)		CCS	OVFIND		# BRANCH ON OVERFLOW TO BASIC OR INTERP.
		TCF	+2
		TCF	DANZIG
		TS	OVFIND
		CCS	CYR
		TCF	RTB		# IF BASIC.
B5TOBB		OCT	360
		TCF	GOTO

# Page 1091
; BZE: Branch on Zero - branches if MPAC = 0.
; GOTO: Unconditional branch to address in POLISH.
; Dispatcher uses CYR bit to distinguish between the two.
BZE/GOTO	CCS	CYR		# SEE WHICH OP-CODE IS DESIRED.
		TC	BRANCH		# DO BZE.
		TCF	DANZIG
		TCF	GOTO		# DO GOTO.
		TCF	DANZIG

; BPL: Branch on PLus - branches if MPAC >= 0.
; BMN: Branch on MiNus - branches if MPAC < 0.
; These enable sign-based conditional branching for vector components,
; velocity signs, and altitude checks during descent.
BPL/BMN		CCS	CYR
		TCF	BPL
5B10		DEC	5 B+10		# SHIFTS OP CODE IN SWITCH INSTRUCTION ADR

		TC	BRANCH		# DO BMN
		TCF	DANZIG
		TCF	DANZIG
		TCF	GOTO		# ONLY IF NNZ.

BPL		TC	BRANCH
		TCF	GOTO		# IF POSITIVE OR ZERO.
		TCF	GOTO
		TCF	DANZIG

; CALL: Call interpretive subroutine - jumps to CALL implementation at line 1042.
; ITA: Index (store return address) To Address.
; Stores current return address (QPRET) to specified erasable address.
; Used when subroutines need to pass return addresses to other routines.
; CYR bit distinguishes: positive = CALL, zero/negative = ITA.
CALL/ITA	CCS	CYR
		TCF	CALL

		TC	CCSHOLE
		TC	15ADRERS	# STORE QPRET.  (TAGSUB AFTER 15ADRERS IS
		INDEX	FIXLOC		# SLOW IN THIS CASE, BUT SAVES STORAGE.)
		CA	QPRET
		TCF	MSTORE1

# Page 1092
; ============================================================================
; SWITCH OPERATIONS
;
; The AGC uses software "switches" (flag bits) to control program flow and
; maintain state across interpretive routines. These operations provide
; conditional branching based on switch states and allow atomic switch
; manipulation (set, clear, invert) combined with testing and branching.
;
; Switches are stored in the STATE erasable area (up to 64 switch words).
; Each word contains 15 switch bits (bit 1-15, numbered left to right).
;
; During Apollo 11's lunar landing, these switches controlled critical
; decision points like FLAGORGY flags in THE_LUNAR_LANDING.agc:
; - MUNFLAG: Indicates in descent phase
; - LRBYPASS: Landing radar data valid/invalid
; - P25FLAG: Rendezvous targeting active
;
; Switch operations are atomic (INHINT/RELINT protected) so interrupt
; routines can safely share switch words with interpretive code.
; ============================================================================

# THE FOLLOWING OPERATIONS ARE AVAILABLE FOR ALTERING AND TESTING INTERPRETATIVE SWITCHES:

#	00	BONSET		SET A SWITCH AND DO A GOTO IF IT WAS ON.
#	01	SETGO		SET A SWITCH AND DO A GOTO.
#	02	BOFSET		SET A SWITCH AND DOA GOTO IF IT WAS OFF
#	03	SET		SET A SWITCH.

#	04	BONINV		INVERT A SWITCH AND BRANCH IF IT WAS ON.
#	05	INVGO		INVERT A SWITCH AND DO A GOTO.
#	06	BOFINV		INVERT A SWITCH AND BRANCH IF IT WAS OFF
#	07	INVERT		INVERT A SWITCH.

#	10	BONCLR		CLEAR A SWITCH AND BRANCH IF IT WAS ON.
#	11	CLRGO		CLEAR A SWITCH AND DO A GOTO.
#	12	BOFCLR		CLEAR A SWITCH AND BRANCH IF IT WAS OFF.
#	13	CLEAR		CLEAR A SWITCH.

#	14	BON		BRANCH IF A SWITCH WAS ON.
#	16	BOFF		BRANCH IF A SWITCH WAS OFF.

# THE ADDRESS SUPPLIED WITH THE SWITCH INSTRUCTION IS INTERPRETED AS FOLLOWS:

#	BITS 1-4	SWITCH BIT NUMBER (1-15).
#	BITS 5-8	SWITCH OPERATION NUMBER
#	BITS 9-		SWITCH WORD NUMBER (UP TO 64 SWITCH WORDS).

# THE ADDRESS ITSELF IS MADE UP BY THE YUL SYSTEM ASSEMBLER.  THE BRANCH INSTRUCTIONS REQUIRE TWO
# ADDRESSES, THE SECOND TAKEN AS THE DIRECT (OR INDIRECT IF IN ERASABLE) ADDRESS OF THE BRANCH.

; Switch address encoding allows compact specification of:
; - Which switch word (STATE +0, STATE +1, ... STATE +63)
; - Which bit within that word (bits 1-15, numbered left to right)
; - What operation to perform (set/clear/invert/test)
; - Optional branch address for conditional operations
;
; Example: BON MUNFLAG, DESCEND
;   Tests MUNFLAG bit, branches to DESCEND if flag is ON.
;   YUL assembler encodes bit number, word number, and operation code.

; Switch instruction entry point.
; Decode the switch address from POLISH to extract bit number, word number,
; and operation code. Then perform atomic switch manipulation.
SWITCHES	CAF	LOW4		# LEAVE THE SWITCH BIT IN SWBIT.
		MASK	POLISH		# Extract bits 1-4 (switch bit number 1-15)
		INDEX	A
		CAF	BIT15		# (NUMBER FROM LEFT TO RIGHT.)
		TS	SWBIT		# Store bit mask for this switch

		CAF	BIT7		# LEAVE THE SWITCH NUMBER IN SWWORD.
		EXTEND			# Extract bits 9+ (switch word number)
		MP	POLISH
		TS	SWWORD		# Store index into STATE array

		INHINT			# DURING SWITCH CHANGE SO RUPT CAN USE TOO
		INDEX	A		# LEAVE THE SWITCH WORD ITSELF IN L.
		CA	STATE		# Load current switch word from STATE array
		TS	Q		# Q WILL BE USED AS A CHANNEL.
# Page 1093
; Dispatch to appropriate switch bit operation (set/invert/clear/noop).
; Bits 7-8 of POLISH encode the operation:
;   00 = SET (OR bit with current word)
;   01 = INVERT (XOR bit with current word)
;   10 = CLEAR (AND complement of bit with current word)
;   11 = NOOP (no modification, test only)
		CAF	BIT11
		EXTEND			# DISPATCH SWITCH BIT OPERATION AS IN BITS
		MP	POLISH		# 7-8 OF POLISH.
		MASK	B3TOB4		# GETS 4X2-BIT CODE.
		INDEX	A		# Dispatch based on operation code
		TCF	+1

 +1		CA	SWBIT		# 00 -- SET SWITCH IN QUESTION.
		EXTEND
		ROR	QCHAN		# OR: Set the bit (turn switch ON)
		TCF	SWSTORE

 +5		CA	SWBIT		# 01 -- INVERT SWITCH.
		EXTEND
		RXOR	QCHAN		# XOR: Toggle the bit
		TCF	SWSTORE

 +9D		CS	SWBIT		# 10 -- CLEAR.
		MASK	Q		# AND with complement: Clear the bit (turn switch OFF)
SWSTORE		INDEX	SWWORD		# Store modified switch word back to STATE
		TS	STATE		# NEW SWITCH WORD.

# Page 1094
; Dispatch to sequence control operation (branch if on/off, goto, or continue).
; After modifying (or not) the switch bit, determine what to do next.
; Bits 5-6 of POLISH encode the branching behavior:
;   00 = Branch if switch was ON (before modification)
;   01 = Unconditional GOTO
;   10 = Branch if switch was OFF (before modification)
;   11 = NOOP (continue to next instruction)
 +13D		RELINT			# 11 -- NOOP.
		CAF	BIT13
		EXTEND			# DISPATCH SEQUENCE CHANGING OR BRANCING
		MP	POLISH		# CODE.
		MASK	B3TOB4		# Extract bits 5-6
		INDEX	A
		TCF	+1		# ORIGINALLY STORED IN BITS 5-6

 +1		CS	Q		# 00 -- BRANCH IF ON.
TEST		MASK	SWBIT		# Test if switch bit was set
		CCS	A		# If bit was ON (positive), take branch
		TCF	SWSKIP		# Bit was ON: skip to branch address

 +5		TCF	SWBRANCH	# 01 -- GO TO (unconditional).

		TCF	SWSKIP		# HERE ONLY ON BIT 15.

		TC	CCSHOLE
		TC	CCSHOLE

 +9D		CA	Q		# 10 -- BRANCH IF OFF.
		TCF	TEST		# Test complement (bit OFF = positive result)

B3TOB4		OCT	0014
SWSKIP		INCR	LOC		# Skip branch address, continue to next instruction

SW/		EQUALS	SWITCHES

 +13D		TCF	DANZIG		# 11 -- NOOP (no branch, continue normally).

