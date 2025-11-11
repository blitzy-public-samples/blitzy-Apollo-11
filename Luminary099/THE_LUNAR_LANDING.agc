# Copyright:	Public domain.
# Filename:	THE_LUNAR_LANDING.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Hartmuth Gutsche<hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	785-792
# Mod history:	2009-05-20 HG	Transcribed from page images.
#
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

; ============================================================================
; FILE: THE_LUNAR_LANDING.agc
; MODULE: Lunar Landing Guidance
; MISSION PHASE: descent/landing
;
; TL;DR: Implements P63 braking phase program managing powered descent from
;        10,000 feet altitude to lunar surface touchdown. Controls descent
;        engine throttle, monitors navigation state, handles transition from
;        automated guidance to semi-manual control at ~500 feet, and processes
;        program alarms including the famous 1202 alarm during Apollo 11 descent.
;
; COMMENT-ONLY READERS: This is the heart of the lunar landing sequence.
;        Read comments to follow Armstrong and Aldrin's descent to the Moon
;        on July 20, 1969, culminating in "The Eagle has landed" at 102:45:40 MET.
; CODE-ALONG READERS: Study guidance equations integration with throttle
;        control and executive scheduler to understand autonomous landing system
;        and crew semi-manual control transition.
; ============================================================================

# Page 785
		BANK	32
		SETLOC	F2DPS*32
		BANK

		EBANK=	E2DPS

#	*************************************
#	P63: THE LUNAR LANDING, BRAKING PHASE
#	*************************************

		COUNT*	$$/P63

; ============================================================================
; TRANSITION: Powered Descent Initiation to Braking Phase
;
; The Lunar Module Eagle has separated from Command Module Columbia and is in
; descent orbit 50,000 feet above the lunar surface. Powered Descent Initiation
; (PDI) occurs at mission elapsed time 102:33:05 on July 20, 1969. The P63
; program manages the critical 12-minute descent through three phases:
; 1) Braking phase (50,000 ft to 10,000 ft) - automated altitude/velocity control
; 2) Approach phase (10,000 ft to ~500 ft) - semi-manual crew control enabled
; 3) Landing phase (~500 ft to surface) - Armstrong's manual site selection
; ============================================================================

; P63LM: Main entry point for the Powered Descent program (Program 63).
; This is the code that controlled Eagle's descent to the lunar surface.
; Armstrong and Aldrin are in the LM, descending toward the Sea of Tranquility.

P63LM		TC	PHASCHNG
		OCT	04024

		TC	BANKCALL	# DO IMU STATUS CHECK ROUTINE R02
		CADR	R02BOTH

; Initialize BURNBABY master ignition routine parameters.
; BURNBABY will control the Descent Propulsion System (DPS) engine ignition
; and throttle management during the powered descent.

		CAF	P63ADRES	# INITIALIZE WHICH FOR BURNBABY
		TS	WHICH

; Initialize Delta-V monitoring thresholds for descent engine performance.
; DVMON tracks velocity changes to detect engine failures or anomalies.

		CAF	DPSTHRSH	# INITIALIZE DVMON
		TS	DVTHRUSH
		CAF	FOUR
		TS	DVCNTR

; Initialize phase tracking variables for descent sequence monitoring.
; WCHPHASE tracks which phase of descent we're in (braking/approach/landing).

		CS	ONE		# INITIALIZE WCHPHASE AND FLPASS0
		TS	WCHPHASE

		CA	ZERO
		TS	FLPASS0

; Disable the landing radar track-enable discrete.
; The radar will be re-enabled during descent when altitude is appropriate.

		CS	BIT14
		EXTEND
		WAND	CHAN12		# REMOVE TRACK-ENABLE DISCRETE.

; ============================================================================
; FLAGORGY: Mission Mode Flag Initialization
;
; This routine prepares the AGC software flags for lunar landing operations.
; The whimsical name "DIONYSIAN FLAG WAVING" was given by the original MIT
; programmers, referring to Dionysus (Greek god of wine and revelry),
; suggesting enthusiastic flag manipulation. Each flag controls critical
; aspects of the landing sequence.
; ============================================================================

FLAGORGY	TC	INTPRET		# DIONYSIAN FLAG WAVING
		CLEAR	CLEAR
			NOTHROTL	; NOTHROTL cleared: Enable throttle control
					;   Allows AGC to command DPS engine throttle
					;   during descent. Critical for altitude/velocity control.
			REDFLAG		; REDFLAG cleared: Clear landing site redesignation
					;   Allows fresh targeting of landing site coordinates.
		CLEAR	SET
			LRBYPASS	; LRBYPASS cleared: Enable landing radar data
					;   AGC will accept altitude/velocity measurements
					;   from landing radar for navigation updates.
			MUNFLAG		; MUNFLAG set: Indicate lunar landing mode active
					;   Tells guidance system we're landing on Moon (not Earth).
		CLEAR	CLEAR
			P25FLAG		; TERMINATE P25 IF IT IS RUNNING.
					; P25FLAG cleared: Terminate P25 (CSM/LM targeting)
					;   Ends any active rendezvous navigation calculations.
			RNDVZFLG	; TERMINATE P20 IF IT IS RUNNING
					; RNDVZFLG cleared: Terminate P20 (orbital nav)
					;   Ends orbit determination program if active.

					# ********************************

; ============================================================================
; IGNALG: Ignition Algorithm - Landing Site Setup and Guidance Initialization
;
; This section prepares the guidance computer for powered descent by:
; 1) Loading the targeted landing site coordinates (RLS) in lunar fixed frame
; 2) Computing the estimated time of landing (TLAND)
; 3) Transforming landing site position to reference frame for guidance
; 4) Initializing guidance parameters for iterative descent trajectory calculation
;
; The algorithm iteratively computes ignition timing (DDUMCALC loop) to achieve
; optimal fuel consumption while reaching the target landing site.
; For Apollo 11, the target was Mare Tranquillitatis (Sea of Tranquility).
; ============================================================================

IGNALG		SETPD	VLOAD		# FIRST SET UP INPUTS FOR RP-TO-R:-
# Page 786
			0		#   AT 0D LANDING SITE IN MOON FIXED FRAME
			RLS		#   AT 6D ESTIMATED TIME OF LANDING
					; RLS: Landing site coordinates in Moon-fixed frame
					;   For Apollo 11: 0.691° N, 23.473° E
					;   (Sea of Tranquility, southwest of crater Sabine D)
		PDDL	PUSH		#   MPAC NON-ZERO TO INDICATE LUNAR CASE
			TLAND		; TLAND: Estimated time of landing
					;   Initially computed from orbital mechanics
					;   Refined iteratively by IGNALG
		STCALL	TPIP		# ALSO SET TPIP FOR FIRST GUIDANCE PASS
			RP-TO-R		; RP-TO-R: Convert from planetary (Moon-fixed) to
					;   reference frame for guidance computations
		VSL4	MXV
			REFSMMAT
		STCALL	LAND
			GUIDINIT	# GUIDINIT INITIALIZES WM AND /LAND/
		DLOAD	DSU
			TLAND
			GUIDDURN
		STCALL	TDEC1		# INTEGRATE STATE FORWARD TO THAT TIME
			LEMPREC
		SSP	VLOAD
			NIGNLOOP
			40D
			UNITX
		STOVL	CG
			UNITY
		STOVL	CG +6
			UNITZ
		STODL	CG +14
			99999CON
		STOVL	DELTAH		# INITIALIZE DELTAH FOR V16N68 DISPLAY
			ZEROVECS
		STODL	UNFC/2		# INITIALIZE TRIM VELOCITY CORRECTION TERM
			HI6ZEROS
		STORE	TTF/8

; ============================================================================
; IGNALOOP: Iterative Ignition Time Computation Loop
;
; This loop iteratively refines the ignition time (TIG) to achieve optimal
; descent trajectory to the landing site. Each iteration:
; 1) Integrates the LM state vector to the current ignition time estimate
; 2) Runs guidance to compute required velocity-to-be-gained (VGU)
; 3) Calculates time correction (DDUM) based on position/velocity errors
; 4) Tests for convergence (within DDUMCRIT threshold)
; 5) If not converged, updates ignition time and repeats
;
; Typically converges in 3-5 iterations. For Apollo 11, this computation
; occurred while Eagle was in descent orbit, approximately 15 minutes before
; powered descent initiation (PDI) at 102:33:05 mission elapsed time.
; ============================================================================

IGNALOOP	DLOAD			; Begin iteration: load current time
			TAT		; TAT: Time at which computation is happening
		STOVL	PIPTIME1	; Store as integration epoch time
			RATT1		; RATT1: LM position vector at TAT
					;   (from orbital navigation state)
		VSL4	MXV		; Scale and transform position to
			REFSMMAT	;   reference coordinates for guidance
					; REFSMMAT: Reference Stable Member Matrix
		STCALL	R		; Store as current position (R)
			MUNGRAV		; MUNGRAV: Compute gravitational acceleration
					;   Includes lunar oblateness perturbations
		STCALL	GDT/2		; Store gravity as GDT/2 (half time-step)
			?GUIDSUB	# WHICH DELIVERS N PASSES OF GUIDANCE
					; ?GUIDSUB: Guidance subroutine dispatcher
					;   Calls landing guidance equations to compute
					;   desired velocity-to-be-gained vector (VGU)
					;   and required landing site position (RGU)

; ============================================================================
; DDUMCALC: Time Correction Computation
;
; COMMENT-ONLY READERS: The computer now calculates how much to adjust the
; ignition time based on the current trajectory errors. This correction (DDUM)
; accounts for altitude error, downrange error, and velocity magnitude error.
; By iteratively adjusting ignition time, the computer finds the precise
; moment to fire the descent engine that will result in arriving at the
; landing site with zero velocity.
;
; CODE-ALONG READERS: DDUM is computed using a weighted sum of position and
; velocity errors, divided by a velocity term. The formula incorporates three
; gain constants (KIGNX, KIGNY, KIGNV) tuned to achieve rapid convergence:
; ============================================================================

# DDUMCALC IS PROGRAMMED AS FOLLOWS:-
#                                         2                                           ___
#              (RIGNZ - RGU )/16 + 16(RGU  )KIGNY/B8 + (RGU - RIGNX)KIGNX/B4 + (ABVAL(VGU) - VIGN)KIGNV/B4
#                          2             1                 0
#	DDUM = -------------------------------------------------------------------------------------------
#                                                10
#                                               2   (VGU - 16 VGU KIGNX/B4)
#                                                       2        0
# Page 787 new page is actually one line earlier but this would put the indices on a separate line
# disconnected from their respective variables
# THE NUMERATOR IS SCALED IN METERS AT 2(28).  THE DENOMINATOR IS A VELOCITY IN UNITS OF 2(10) M/CS.
# THE QUOTIENT IS THUS A TIME IN UNITS OF 2(18) CENTISECONDS.  THE FINAL SHIFT RESCALES TO UNITS OF 2(28) CS.
# THERE IS NO DAMPING FACTOR.  THE CONSTANTS KIGNX/B4, KIGNY/B8 AND KIGNV/B4 ARE ALL NEGATIVE IN SIGN.

DDUMCALC	TS	NIGNLOOP	; Store remaining iteration count
		TC	INTPRET		; Enter interpretive mode for vector math
		DLOAD	DMPR		# FORM DENOMINATOR FIRST
			VGU		; VGU(0): X-component of velocity-to-be-gained
					;   (downrange component, aligned with landing site)
			KIGNX/B4	; KIGNX/B4: Gain constant for downrange velocity
					;   Negative value, scaled by 2^4
		SL4R	BDSU		; Shift left 4 bits (multiply by 16), then subtract
			VGU +4		; VGU(2): Z-component of velocity-to-be-gained
					;   (vertical component, altitude rate)
					; Denominator = 2^10 * (VGU(2) - 16*VGU(0)*KIGNX/B4)
		PDDL	DSU		; Push denominator to stack, load altitude error
			RIGNZ		; RIGNZ: Initial estimate of altitude at ignition
			RGU +4		; RGU(2): Guidance-computed altitude component
					; Altitude error = (RIGNZ - RGU(2))
		SR4R	PDDL		; Shift altitude error right 4 (divide by 16), push
			RGU +2		; RGU(1): Y-component of required position
					;   (crossrange, perpendicular to landing approach)
		DSQ	DMPR		; Square Y-component, multiply by gain
			KIGNY/B8	; KIGNY/B8: Gain for crossrange position error
					;   Negative value, scaled by 2^8
					; Term 2 = 16 * (RGU(1))^2 * KIGNY/B8
		SL4R	PDDL		; Shift left 4, push, load X-component
			RGU		; RGU(0): X-component of required position
					;   (downrange along landing approach path)
		DSU	DMPR		; Compute downrange error and scale by gain
			RIGNX		; RIGNX: Initial estimate of downrange at ignition
			KIGNX/B4	; KIGNX/B4: Gain for downrange error
					; Term 3 = (RGU(0) - RIGNX) * KIGNX/B4
		PDVL	ABVAL		; Push term 3, load VGU vector, compute magnitude
			VGU		; VGU: Complete velocity-to-be-gained vector
					; ABVAL computes sqrt(VGU(0)^2 + VGU(1)^2 + VGU(2)^2)
		DSU	DMPR		; Compute velocity magnitude error
			VIGN		; VIGN: Initial estimate of velocity magnitude needed
			KIGNV/B4	; KIGNV/B4: Gain for velocity magnitude error
					; Term 4 = (|VGU| - VIGN) * KIGNV/B4
		DAD	DAD		; Add all numerator terms together:
		DAD	DDV		;   Sum = Term1 + Term2 + Term3 + Term4
					; Divide sum by denominator (still on stack)
		SRR			; Shift result right 10 bits
			10D		;   Converts from 2^18 centiseconds to 2^28 centiseconds
					; DDUM now contains ignition time correction

		PUSH	DAD		; Push DDUM to stack, add to current time
			PIPTIME1	; PIPTIME1: Current integration epoch time
		STODL	TDEC1		# STORE NEW GUESS FOR NEXT INTEGRATION
					; TDEC1 = PIPTIME1 + DDUM (updated ignition time)
		ABS	DSU		; Take absolute value of DDUM, subtract threshold
			DDUMCRIT	; DDUMCRIT: Convergence criterion (~0.1 seconds)
					;   If |DDUM| < DDUMCRIT, solution has converged
		BMN	CALL		; Branch if result is negative (converged)
			DDUMGOOD	;   to DDUMGOOD (ignition time is acceptable)
			INTSTALL	; INTSTALL: Set up integration parameters
					;   Not converged yet, prepare for another iteration
		SET	SET		; Set flags for integration
			INTYPFLG	; INTYPFLG: Integration type flag (conic)
			MOONFLAG	; MOONFLAG: Indicates lunar gravity field
		DLOAD			; Load integration epoch time
			PIPTIME1
		STOVL	TET		# HOPEFULLY ?GUIDSUB DID NOT
			RATT1		#   CLOBBER RATT1 AND VATT1
					; TET: Time of initial state vector
					; RATT1: Position vector at that time
# Page 788
		STOVL	RCV		; Store position as RCV (integration initial position)
			VATT1		; VATT1: Velocity vector at PIPTIME1
		STCALL	VCV		; Store velocity as VCV (integration initial velocity)
			INTEGRVS	; INTEGRVS: Integrate state vector forward
					;   From PIPTIME1 (TET) to TDEC1 (updated TIG)
					;   Uses conic orbit propagation
		GOTO			; Integration complete, repeat IGNALOOP
			IGNALOOP	;   with updated state at new ignition time

; ============================================================================
; DDUMGOOD: Ignition Time Converged - Finalize Landing Parameters
;
; COMMENT-ONLY READERS: The computer has found the optimal ignition time!
; The descent engine will fire at precisely the right moment to arrive at
; the landing site. The computer now calculates the "out-of-plane" distance:
; how far to the right or left the landing site is from the LM's current
; orbital plane. This helps plan the approach trajectory.
;
; CODE-ALONG READERS: This section computes TIG (Time of Ignition) and
; OUTOFPLN (out-of-plane distance). OUTOFPLN is the dot product of the
; landing site unit vector with the orbit plane normal vector.
; ============================================================================

DDUMGOOD	SLOAD	SR		; Load zoom time (high-altitude orbit time)
			ZOOMTIME	; ZOOMTIME: Time of apoapsis in descent orbit
			14D		; Shift right 14 bits for scaling
		BDSU			; Subtract converged ignition time
			TDEC1		; TDEC1: Final computed ignition time (TIG)
		STOVL	TIG		# COMPUTE DISTANCE LANDING SITE WILL BE
			V		#   OUT OF LM'S ORBITAL PLANE AT IGNITION:
					; TIG now contains time from now until ignition
					; V: Current velocity vector
		VXV	UNIT		#   SIGN IS + IF LANDING SITE IS TO THE
			R		#   RIGHT, NORTH; - IF TO THE LEFT, SOUTH.
					; V × R = orbit plane normal vector (perpendicular)
					; UNIT normalizes to unit vector
		DOT	SL1		; Dot product with landing site direction
			LAND		; LAND: Landing site unit vector in ref frame
					; Result is sine of angle (out-of-plane component)
R60INIT		STOVL	OUTOFPLN	# INITIALIZATION FOR CALCMANU
					; OUTOFPLN: Out-of-plane distance (+ = north, - = south)
			UNFC/2		; UNFC/2: Half of unit thrust acceleration vector
		STORE	R60VSAVE	# STORE UNFC/2 TEMPORARILY IN R60SAVE
					; R60VSAVE: Temporary storage for maneuver routine
		EXIT			; Return to basic AGC mode (exit interpreter)
					# *******************************************

; ============================================================================
; TRANSITION: From Pre-Ignition Calculation to Crew Alignment
;
; COMMENT-ONLY READERS: With ignition time computed, Armstrong and Aldrin now
; perform an IMU alignment using star sightings through the Alignment Optical
; Telescope (AOT). This ensures the inertial platform is precisely oriented
; before the critical powered descent. They will sight on two stars and the
; computer will refine the platform alignment to within 0.1 degrees.
;
; CODE-ALONG READERS: IGNALGRT prevents re-entry to IGNALG after convergence.
; ASTNCLOK schedules R51 (IMU orientation determination) to be called.
; ASTNRET receives control after R51 completes, initializing burn attitude.
; ============================================================================

IGNALGRT	TC	PHASCHNG	# PREVENT REPEATING IGNALG
		OCT	04024		; Phase change: mark IGNALG complete
					; Prevents restart from re-running ignition calc

ASTNCLOK	CS	ASTNDEX		; ASTNDEX: Index for astronaut task scheduling
		TC	BANKCALL	; Cross-bank call to scheduling routine
		CADR	STCLOK2		; STCLOK2: Set up long-call to major mode
					;   Schedules R51 (IMU alignment) for crew
		TCF	ENDOFJOB	# RETURN IN NEW JOB AND IN EBANK FIVE
					; End this job, R51 will run as separate job

ASTNRET		TC	INTPRET		; Return here after R51 alignment complete
		SSP	RTB		# GO PICK UP DISPLAY AT END OF R51:
			QMAJ		#   "PROCEED" WILL DO A FINE ALIGNMENT
		FCADR	P63SPOT2	#   " ENTER " WILL RETURN TO P63SPOT2
			R51P63		; R51P63: R51 configured for P63 entry
					; Crew can choose PROCEED (fine align) or ENTER (skip)
P63SPOT2	VLOAD	UNIT		# INITIALIZE KALCMANU FOR BURN ATTITUDE
			R60VSAVE	; R60VSAVE: Contains UNFC/2 (thrust direction)
		STOVL	POINTVSM	; POINTVSM: Desired LM pointing direction
					;   Set to thrust vector direction for burn
			UNITX		; UNITX: Unit vector in X-axis direction
		STORE	SCAXIS		; SCAXIS: Spacecraft axis for attitude control
					;   X-axis (along LM centerline) will align with thrust
		EXIT			; Exit interpreter mode

; ============================================================================
; Pre-Ignition Display and Maneuver Preparation
;
; COMMENT-ONLY READERS: The computer now presents the crew with the burn
; attitude display (R60) showing the orientation the LM must achieve before
; engine ignition. The crew uses the hand controller to maneuver the LM
; into this attitude, pointing the descent engine in the correct direction.
;
; CODE-ALONG READERS: Switch to EBANK7 for display routines, enable pre-flight
; light test (PFLITEDB), then call R60LEM which displays desired attitude
; angles and current attitude error on the DSKY for crew monitoring.
; ============================================================================

		CAF	EBANK7		; Switch to erasable bank 7
		TS	EBANK		; EBANK register selects active memory bank

		INHINT			; Inhibit interrupts during bank call setup
		TC	IBNKCALL	; Inter-bank call (different bank subroutine)
		CADR	PFLITEDB	; PFLITEDB: Pre-flight light test for DSKY
# Page 789
		RELINT			; Re-enable interrupts

		TC	BANKCALL	; Call attitude maneuver display routine
		CADR	R60LEM		; R60LEM: Display desired LM attitude to crew
					;   Shows FDAI needles and DSKY angles
					;   Crew maneuvers LM to align thrust vector

		TC	PHASCHNG	# PREVENT RECALLING R60
		OCT	04024		; Phase change prevents R60 restart

; ============================================================================
; P63SPOT3: Landing Radar Antenna Position Check
;
; COMMENT-ONLY READERS: The landing radar antenna must be cranked to Position 1
; (forward-looking) before descent. If it's not there yet, the computer displays
; "CODE 500" asking the crew to manually crank it. During Apollo 11, this was
; routine procedure—Buzz Aldrin confirmed "Antenna to Position 1."
;
; CODE-ALONG READERS: Reads CHAN33 bit 6 to check landing radar antenna
; position switch. If not in Position 1, displays CODE 500 via GOPERF1,
; waits for crew PROCEED, then re-checks. When confirmed (or ENTER pressed),
; calls SETPOS1 to initialize radar and jumps to BURNBABY (master ignition).
; ============================================================================

P63SPOT3	CA	BIT6		# IS THE LR ANTENNA IN POSITION 1 YET
		EXTEND			; Extended instruction follows
		RAND	CHAN33		; Read AND Channel 33 (landing radar discrete)
					;   BIT6 = 1 if antenna in position 1
		EXTEND			; Extended instruction follows
		BZF	P63SPOT4	# BRANCH IF ANTENNA ALREADY IN POSITION 1
					;   (BZF: Branch on Zero to Following address)

		CAF	CODE500		# ASTRONAUT:	PLEASE CRANK THE
		TC	BANKCALL	#		SILLY THING AROUND
		CADR	GOPERF1		; Display "500" on DSKY, flash verb-noun display
					;   Crew must PROCEED (antenna positioned) or
					;   TERMINATE (abort P63)
		TCF	GOTOPOOH	# TERMINATE
					;   Crew selected TERMINATE, end P63
		TCF	P63SPOT3	# PROCEED	SEE IF HE'S LYING
					;   Crew pressed PROCEED, re-check antenna
					;   (Comment humor: verify crew actually moved it!)

P63SPOT4	TC	BANKCALL	# ENTER		INITIALIZE LANDING RADAR
		CADR	SETPOS1		; SETPOS1: Configure landing radar for position 1
					;   Enables radar data processing for descent

		TC	POSTJUMP	# OFF TO SEE THE WIZARD...
		CADR	BURNBABY	; Jump to BURNBABY master ignition routine
					;   (Wizard of Oz reference in original comment)
					;   Begins powered descent sequence!

; ============================================================================
; THE FAMOUS 1202 ALARM - A CRITICAL MOMENT IN HISTORY
;
; COMMENT-ONLY READERS: After control transfers to BURNBABY and the descent
; engine ignites, the guidance computer begins continuous trajectory computation,
; processing landing radar data, updating the navigation state, computing
; required velocity changes, and commanding the throttle and attitude control
; systems. During Apollo 11's actual descent on July 20, 1969, at approximately
; 102:38:26 mission elapsed time (about 5 minutes after powered descent
; initiation), the computer triggered program alarm 1202.
;
; What happened: The landing radar was sending data faster than expected,
; creating more computational tasks than the executive scheduler's job queue
; could handle. The WAITLIST and EXECUTIVE routines (in separate files)
; detected the overload condition and raised the 1202 alarm code, which
; displayed on the DSKY as "PROG 1202."
;
; Armstrong radioed: "Give us a reading on the 1202 Program Alarm."
; In Mission Control, 26-year-old flight controller Steve Bales (GUIDO -
; Guidance Officer) had only seconds to decide. His backroom support officer
; Jack Garman had memorized the alarm codes and knew 1202 meant executive
; overflow—serious, but not fatal if it didn't happen continuously. Bales
; made the call: "We're Go on that alarm."
; CapCom Charlie Duke relayed: "Roger, we got you... We're Go on that alarm."
;
; The AGC's brilliant restart protection system (designed by engineers at MIT
; Instrumentation Laboratory) preserved all critical guidance state during
; the overload. The alarm recurred several times (including a 1201 variant),
; but each time the computer recovered, and the landing continued. This was
; exactly what the restart system was designed to do—prioritize critical
; tasks, shed lower-priority work temporarily, and maintain mission-critical
; functions.
;
; Without Steve Bales' split-second decision and the AGC's fault-tolerant
; design, Apollo 11 would have aborted just minutes from the Moon's surface.
; Instead, 7 minutes later, Armstrong announced: "The Eagle has landed."
;
; CODE-ALONG READERS: The 1202 alarm is generated by EXECUTIVE.agc when the
; job queue (core sets) overflows—too many jobs requested simultaneously.
; The WAITLIST.agc overflow (1201 alarm) occurs when the timer task queue
; exceeds capacity. Both are handled by ALARM_AND_ABORT.agc which displays
; the alarm code and continues execution if the overflow clears. The restart
; protection logic (RESTART_TABLES.agc, RESTARTS_ROUTINE.agc) uses phase
; tables to record program state at key points, allowing intelligent recovery
; from interruptions. During Apollo 11, the landing radar coupled with the
; rendezvous radar (left running by mistake) created the excessive load.
; The computer's priority interrupt system ensured critical guidance, throttle,
; and attitude control computations continued while shedding less critical
; display updates and radar data processing temporarily.
; ============================================================================

#	---------------------------------

; ============================================================================
; P63 Program Constants
;
; COMMENT-ONLY READERS: These constants control the timing and precision of
; the powered descent sequence. They define how long the descent will take,
; how accurately the ignition timing must converge, and what displays the
; crew will see if manual intervention is needed.
;
; CODE-ALONG READERS: Fixed-point constants with specific scaling factors.
; P63ADRES points to the P63TABLE for program initialization. GUIDDURN is
; the estimated guidance duration (~664 seconds for complete descent).
; DDUMCRIT is the convergence criterion (8 × 2^-28) for the IGNALG iterative
; solution of ignition time—iteration stops when time delta falls below this.
; ============================================================================

#	CONSTANTS FOR P63LM AND IGNALG

P63ADRES	GENADR	P63TABLE	; Address of P63 program parameter table

ASTNDEX		=	MD1		# OCT 25:  INDEX FOR CLOKTASK
					; Mission timer index for astronaut clock
					;   display during descent

CODE500		OCT	00500		; DSKY display code: "500" = PLEASE CRANK
					;   LANDING RADAR ANTENNA TO POSITION 1

99999CON	2DEC	30479.7 B-24	; Constant 99999 scaled by 2^-24
					;   (30479.7 × 2^-24 ≈ 99999 feet)
					;   Used in altitude calculations

GUIDDURN	2DEC	+66440		# GUIDDURN	+6.64400314 E+2
					; Guidance duration: +664.40 seconds
					;   Estimated time for complete powered descent
					;   from PDI to touchdown (~11 minutes)

DDUMCRIT	2DEC	+8 B-28		# CRITERION FOR IGNALG CONVERGENCE
					; Convergence threshold: 8 × 2^-28
					;   ≈ 0.0000000298 seconds
					;   IGNALG iteration stops when delta-time
					;   between successive solutions < this value

# Page 790
#	--------------------------------

# Page 791

; ============================================================================
; TRANSITION: From Powered Descent to Landing Confirmation
;
; The Eagle has landed! After touchdown at 102:45:40 mission elapsed time
; on July 20, 1969, Neil Armstrong radioed the immortal words: "Houston,
; Tranquility Base here. The Eagle has landed." Mission Control erupted in
; celebration while CapCom Charlie Duke responded, "Roger, Tranquility, we
; copy you on the ground. You got a bunch of guys about to turn blue. We're
; breathing again. Thanks a lot."
;
; Now the P68 program executes to confirm the landing site coordinates and
; prepare for the lunar surface stay. The crew will verify their position,
; then transition to P57 (lunar surface alignment program) to maintain IMU
; alignment during their 21.5-hour stay on the Moon.
; ============================================================================

#	*************************
#	P68: LANDING CONFIRMATION
#	*************************

		BANK	31		; Switch to bank 31 for P68 code
		SETLOC	F2DPS*31	; Set location in fixed memory
		BANK

		COUNT*	$$/P6567	; Instruction counter for program metrics

; ============================================================================
; LANDJUNK: Post-Landing Initialization and Site Confirmation
;
; COMMENT-ONLY READERS: After the LM touches down, this routine zeroes the
; attitude control errors (we're on the ground now, not flying), widens the
; attitude control deadband to save RCS propellant, sets flags indicating
; we're on the lunar surface, computes the actual landing site coordinates,
; and displays them to the crew. Armstrong and Aldrin verified Eagle landed
; at approximately 0.67°N, 23.49°E in the Sea of Tranquility, about 4 miles
; downrange from the planned target site due to Armstrong's manual site
; selection to avoid a boulder field.
;
; CODE-ALONG READERS: Calls ZATTEROR to zero attitude error accumulation,
; SETMAXDB to widen DAP deadband to 5 degrees (conserve RCS fuel on surface).
; Sets SURFFLAG (on lunar surface), clears LETABORT (no abort possible now),
; sets APSFLAG (ascent engine available), LUNAFLAG (lunar operations mode).
; Computes current position RN, calls LAT-LONG to convert to latitude/longitude,
; calls R-TO-RP to compute landing site in moon-fixed frame, stores in RLS.
; Displays V06N43 (latitude, longitude, altitude) for crew verification.
; ============================================================================

LANDJUNK	TC	PHASCHNG	; Update program phase for restart protection
		OCT	04024		;   Phase code 04024

		INHINT			; Inhibit interrupts during initialization
		TC	BANKCALL	# ZERO ATTITUDE ERROR
		CADR	ZATTEROR	; ZATTEROR: Zero attitude error integrators
					;   (On surface, no active attitude control)

		TC	BANKCALL	# SET 5 DEGREE DEADBAND
		CADR	SETMAXDB	; SETMAXDB: Set maximum (5°) attitude deadband
					;   Prevents wasteful RCS firing on surface

		TC	INTPRET		# TO INTERPRETIVE AS TIME IS NOT CRITICAL
					; Enter interpreter mode (no time pressure now)
		SET	CLEAR		; Set SURFFLAG, clear LETABORT
			SURFFLAG	;   SURFFLAG: LM is on lunar surface
			LETABORT	;   LETABORT: Abort guidance no longer active
		SET	VLOAD		; Set APSFLAG, load position vector
			APSFLAG		;   APSFLAG: Ascent Propulsion System available
			RN		;   RN: Current position vector (from navigation)
		STODL	ALPHAV		; Store position in ALPHAV, load time
			PIPTIME		;   PIPTIME: Time of last PIPA reading
		SET	CALL		; Set LUNAFLAG, call LAT-LONG
			LUNAFLAG	;   LUNAFLAG: Lunar operations mode active
			LAT-LONG	;   LAT-LONG: Convert position to lat/long/alt
		SETPD	VLOAD		# COMPUTE RLS AND STORE IT AWAY
			0		;   Set push-down pointer to 0
			RN		;   Load current position vector
		VSL2	PDDL		; Vector shift left 2, push, load time
					;   (Shift scales position appropriately)
			PIPTIME		;   PIPTIME for coordinate transformation
		PUSH	CALL		; Push time, call R-TO-RP
			R-TO-RP		;   R-TO-RP: Position to moon-fixed frame
		STORE	RLS		;   RLS: Landing site in moon-fixed coordinates
					;        (Will be used for ascent targeting)
		EXIT			; Exit interpreter mode

; ============================================================================
; Landing Site Display and Crew Verification
;
; COMMENT-ONLY READERS: The computer now displays the actual landing site
; coordinates (latitude, longitude, altitude) on the DSKY using Verb 06
; Noun 43. The crew verifies these coordinates match what they observe
; through the windows. During Apollo 11, the crew confirmed their position
; and radioed Mission Control with landmark observations to refine their
; location estimate. This information was critical for planning the ascent
; trajectory to rendezvous with Michael Collins in the Command Module.
;
; CODE-ALONG READERS: Displays V06N43 via GOFLASH (flashing display awaiting
; crew response). Noun 43 format: R1 = latitude (degrees), R2 = longitude
; (degrees), R3 = altitude (feet). Crew can TERMINATE (unlikely on surface!),
; PROCEED (accept and continue to P57), or RECYCLE (re-display same data).
; ============================================================================

		CAF	V06N43*		# ASTRONAUT:  NOW LOOK WHERE YOU ENDED UP
		TC	BANKCALL	; Call flashing display routine
		CADR	GOFLASH		;   GOFLASH: Display V06N43, await crew input
		TCF	GOTOPOOH	# TERMINATE
					;   (Highly unlikely crew would terminate here!)
		TCF	+2		# PROCEED
					;   Crew accepts landing site, continue
		TCF	-5		# RECYCLE
					;   Crew wants to see display again

		TC	INTPRET		; Re-enter interpreter for final setup
# Page 792
		VLOAD			# INITIALIZE GSAV AND (USING REFMF)
			UNITX		#   YNBSAV, ZNBSAV AND ATTFLAG FOR P57
		STCALL	GSAV		;   GSAV: Gravity vector for surface alignment
			REFMF		;   REFMF: Reference frame transformation
					;   Prepares coordinate frames for P57
		EXIT			; Exit interpreter

		TCF	GOTOPOOH	# ASTRONAUT:  PLEASE SELECT P57
					; Jump to program termination
					;   Crew will manually select P57 (lunar
					;   surface alignment) to maintain IMU
					;   alignment during 21.5-hour surface stay

V06N43*		VN	0643		; Verb 06 Noun 43: Display decimal
					;   R1 = Latitude (degrees)
					;   R2 = Longitude (degrees)
					;   R3 = Altitude (feet)

