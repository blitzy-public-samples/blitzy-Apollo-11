# Copyright:	Public domain.
# Filename:	R30.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	712-722
# Mod history:	2009-05-19 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
#		2009-06-07 RSB	Removed a space between two components of
#				a 2OCT that isn't legal in yaYUL.
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
; FILE: R30.agc
; MODULE: Orbital Parameter Display (R30)
; MISSION PHASE: lunar-orbit/descent/ascent/rendezvous
;
; TL;DR: Computes and displays key orbital parameters including apogee and
;        perigee altitudes, time to perigee, and time to free fall. Invoked
;        by Verb 82, this routine converts the spacecraft state vector into
;        Keplerian orbital elements that the crew can monitor on the DSKY
;        display. Supports both Earth-centered and Moon-centered orbits with
;        appropriate scaling for each celestial body.
;
; COMMENT-ONLY READERS: This routine gives the crew real-time awareness of
;        their orbital characteristics - how high, how low, and when key
;        events will occur. Essential for mission planning and navigation.
; CODE-ALONG READERS: Study how state vectors are transformed to orbital
;        elements, reference frame conversions between Earth/Moon systems,
;        and integration with the DSKY display interface for crew monitoring.
; ============================================================================

# Page 712
; ============================================================================
; ORBITAL PARAMETER DISPLAY ROUTINE (VERB 82)
;
; The crew invokes this routine by entering Verb 82 on the DSKY keyboard.
; This provides them with critical orbital information: how high their orbit
; reaches (apogee), how low it dips (perigee), and when they'll reach
; perigee. This information is essential for mission planning, particularly
; during lunar orbit operations, rendezvous maneuvers, and descent preparation.
;
; The routine operates in two modes depending on whether the Average G
; navigation system is active. In either mode, the spacecraft's current
; state vector (position and velocity) is transformed into classical orbital
; elements that describe the conic section trajectory being followed.
; ============================================================================

# SUBROUTINE NAME:  V82CALL
# MOD NO: 0								DATE: 16 FEB 67
# MOD BY: R. R. BAIRNSFATHER						LOG SECTION:  R30
# MOD NO: 1	MOD BY:  R. R. BAIRNSFATHER	DATE: 11 APR 67		SR30.1 CHANGED TO ALLOW MONITOR OPERN
# MOD NO: 2	MOD BY:  ALONSO			DATE: 11 DEC 67		VB82 PROGRAM REWRITTEN
# MOD NO: 3	MOD BY:  ALONSO			DATE: 26 MAR 68		PROG MOD TO HANDLE DIF EARTH/MOON SCALE
#
# NEW FUNCTIONAL DESCRIPTION:	CALLED BY VERB 82 ENTER.  PRIORITY 10.
# USED THROUGHOUT.		CALCULATE AND DISPLAY ORBITAL PARAMETERS
#
# 1.	IF AVERAGE G IS OFF:
#		FLASH DISPLAY V04N06. R2 INDICATES WHICH SHIP'S STATE VECTOR IS
#		 TO BE UPDATED. INITIAL CHOICE IS THIS SHIP (R2=1). ASTRONAUT
#		 CAN CHANGE TO OTHER SHIP BY V22EXE, WHERE X NOT EQ 1.
#		SELECTED STATE VECTOR UPDATED BY THISPREC (OTHPREC).
#		CALLS SR30.1 (WHICH CALLS TFFCONMU + TFFRP/RA) TO CALCULATE
#		 RPER (PERIGEE RADIUS), RAPO (APOGEE RADIUS), HPER (PERIGEE
#		 HEIGHT ABOVE LAUNCH PAD OR LUNAR LANDING SITE), HARD (APOGEE
#		 HEIGHT AS ABOVE), TPER (TIME TO PERIGEE), TFF (TIME TO
#		 INTERSECT 300 KFT ABOVE PAD OR 35KFT ABOVE LANDING SITE).
#		FLASH MONITOR V16N44 (HAPO, HPER, TFF).TFF IS -59M59S IF IT WAS
#		 NOT COMPUTABLE, OTHERWISE IT INCREMENTS ONCE PER SECOND.
#		 ASTRONAUT HAS OPTION TO MONITOR TPER BY KEYING IN N 32 E.
#		 DISPLAY IS IN HMS, IS NEGATIVE (AS WAS TFF), AND INCREMENTS
#		 ONCE PER SECOND ONLY IF TFF DISPLAY WAS -59M59S.
#
# 2.	IF AVERAGE G IS ON:
#		CALLS SR30.1 APPROX EVERY TWO SECS.  STATE VECTOR IS ALWAYS
#		 FOR THIS VEHICLE. V82 DOES NOT DISTURB STATE VECTOR.  RESULTS
#		 OF SR30.1 ARE RAPO, RPER, HAPO, HPER, TPER, TFF.
#		FLASH MONITOR V16N44 (HAPO, HPER, TFF).
# ADDENDUM: HAPO AND HPER SHOULD BE CHANGED TO READ HAPOX AND HPERX IN THE
#		 ABOVE REMARKS.
#
# CALLING SEQUENCE:  VERB 82 ENTER.
#
# SUBROUTINES CALLED:  SR30.1, GOXDSPF
#			MAYBE - THISPREC , OTHPREC, LOADTIME, DELRSPL
# NORMAL EXIT MODES:  TC ENDEXT
#
# ALARMS:  NONE
#
# OUTPUT:  HAPOX	(-29) M
#	   HPERX	(-29) M
#	   RAPO		(-29) M EARTH
#			(-27) M MOON
#	   RPER		(-29) M EARTH
#			(-27) M MOON
#	   TFF		(-28) CS	CONTAINS NEGATIVE QUANTITY
#	   -TPER	(-28) CS	CONTAINS NEGATIVE QUANTITY
# Page 713
#
# ERASABLE INITIALIZATION REQUIRED: STATE VECTOR.
#
# DEBRIS:	QPRET, RONE, VONE,TFF/RTMU, HPERMIN, RPADTEM, V82EMFLG.
#		MAYBE: TSTART82, V82FLAGS, TDEC1.

		EBANK=	HAPOX
		BANK	31
		SETLOC	R30LOC
		BANK
		COUNT*	$$/R30

; Verb 82 Entry Point - Orbital Parameter Display Request
; When the crew enters V82 on the DSKY, execution begins here. The routine
; first determines the current navigation mode to select the appropriate
; calculation and display strategy.

V82CALL		TC	INTPRET		; Enter interpretive mode for vector math
		BON	GOTO		; Branch on flag (if set, go to first addr)
			AVEGFLAG	; Check if Average G navigation active
			V82GON		; If Average G ON: continuous monitoring mode
			V82GOFF		; If Average G OFF: single calculation mode

; ============================================================================
; AVERAGE G OFF MODE - Manual Vehicle Selection
;
; When Average G navigation is not active, the routine allows the crew to
; select which vehicle's orbital parameters to display: the Lunar Module
; (this ship) or the Command Module (other ship). This is useful during
; rendezvous operations when the crew needs to monitor both spacecraft.
;
; The routine displays V04N06, where R2 indicates the selected vehicle
; (R2=1 means this ship, any other value means the other ship). The crew
; can change their selection by entering new data via the DSKY.
; ============================================================================

V82GOFF		EXIT			# Exit interpretive mode
		CAF	TWO		# Set up display options
		TS	OPTIONX		# Load option code for V04N06 display
		CAF	ONE		# Default selection: this vehicle (LM)
		TS	OPTIONX +1	# Store in R2 display register
		CAF	OPTIONVN	# V 04 N 06 (flash display for input)
		TC	BANKCALL	; Call display interface routine
		CADR	GOXDSPF		; Display V04N06 and await crew response
		TC	ENDEXT		# TERMINATE key pressed - exit routine
		TC	+2		# PROCEED key pressed - accept selection
		TC	-5		# DATA IN - crew entered new value
					; OPTIONX+1 = 1 for this vehicle (LM)
					; OPTIONX+1 ≠ 1 for other vehicle (CSM)
; Set up a timer task to handle display updates. The TICKTEST routine will
; increment the time-to-free-fall and time-to-perigee displays once per second
; so the crew can see these countdowns updating in real-time on the DSKY.

		CAF	BIT4		; 80 milliseconds delay
		TC	WAITLIST	; Schedule TICKTEST on timer queue
		EBANK=	TFF		; Set erasable bank for TFF access
		2CADR	TICKTEST	; Address of timer task routine

		RELINT			; Re-enable interrupts

; Main recycle loop: This is where the routine returns after each display
; update when the crew presses PROCEED to continue monitoring. The state
; vector is updated and orbital parameters are recalculated.

V82GOFLP	CAF	TFFBANK		; MAJOR RECYCLE LOOP ENTRY
		TS	EBANK		; Set erasable bank for TFF variables
		CAF	ZERO
		TS	V82FLAGS	; Zero flags for TICKTEST
					; This inhibits decrementing of TFF and -TPER
					; until calculations complete
		CAF	PRIO7		; Priority 7 job
		TC	FINDVAC		; Find vacant core set for new job
		EBANK=	TFF		; Set erasable bank
		2CADR	V82GOFF1	; V82GOFF1 will execute state vector
					; update and orbit calculations for
					; selected vehicle about proper body

; Stall loop: Wait here until the state vector update and orbital calculations
; complete. This prevents displaying incomplete or inconsistent data to the crew.

		RELINT			; Re-enable interrupts
V82STALL	CAF	THREE		; STALL IN THIS LOOP AND WITHOLD V 16 N 44
# Page 714
		MASK	V82FLAGS	; Check if calculation complete
		CCS	A		; Test if any flag bit set
		TC	FLAGGON		; Flag set: exit stall loop, show results
		CAF	1SEC		; Flag not set: wait 1 second
		TC	BANKCALL	; Delay this job
		CADR	DELAYJOB	; 1-second delay routine
		TC	V82STALL	; Loop back and check again

; Display orbital parameters to the crew. V16N44 is a monitor-style display
; that shows apogee altitude, perigee altitude, and time-to-free-fall.
; The display flashes to get crew attention, then updates continuously.

FLAGGON		CAF	V16N44		; MONITOR HAPO, HPER, TFF
		TC	BANKCALL	; Call display interface
		CADR	GOXDSPF		; Display V16N44 (monitor mode)
		TC	B5OFF		; TERM key: tell TICKTEST to terminate
		TC	B5OFF		; PROCEED key: also terminate TICKTEST
		TC	V82GOFLP	; RECYCLE key: recompute state vector
					; and display updated parameters

OPTIONVN	VN	412		; V04N06 - Option selection display
V16N44		VN	1644		; V16N44 - Monitor HAPO, HPER, TFF
TFFBANK		ECADR	TFF		; Extended address for TFF variable

; ============================================================================
; STATE VECTOR UPDATE AND ORBITAL PARAMETER COMPUTATION
;
; This section updates the state vector for the selected vehicle and prepares
; the position and velocity data for orbital calculations. The AGC maintains
; separate state vectors for the LM and CSM during rendezvous operations.
; ============================================================================

V82GOFF1	TC	INTPRET		; Enter interpretive mode
		RTB			; Return to basic (load time)
			LOADTIME	; Get current mission time
		STORE	TDEC1		; TIME FOR STATE VECTOR UPDATE
		STORE	TSTART82	; TIME FOR INTERNAL USE
		EXIT			; Exit interpretive mode

; Determine which vehicle's state vector to use based on crew selection.
; This logic checks OPTIONX+1: if it equals 1, use this ship (LM);
; otherwise use other ship (CSM).

		CS	OPTIONX +1	; Load complement of selection
		AD	ONE		; Add 1 (result is zero if selection was 1)
		EXTEND			; Extended instruction follows
		BZF	THISSHIP	; Branch if zero (this vehicle selected)

; Other ship selected: Update CSM state vector and orbital parameters

OTHSHIP		TC	INTPRET		; Enter interpretive mode
		CALL			; Call state vector update for CSM
			OTHPREC		; Other precision integration routine

; Copy the updated state vector into working storage for SR30.1 calculations.
; The position vector (RATT) and velocity vector (VATT) are moved to RONE
; and VONE respectively. These are the inputs for orbital element computation.

BOTHSHIP	VLOAD			; Load vector from memory
			RATT		; Position vector (radius)
		STOVL	RONE		; Store in RONE, then load next vector
					; RATT scaled at (-29)M for Earth or Moon
			VATT		; Velocity vector
		STORE	VONE		; VATT scaled at (-7)M/CS for Earth or Moon

; Load gravitational parameters appropriate for the central body (Earth or Moon).
; X2 register contains body indicator: 0 for Earth, 2 for Moon, set by
; THISPREC or OTHPREC routines. Indexed loads select correct constants.

		DLOAD*			; Double precision load with indexing
			1/RTMUE,2	; 1/√μ for Earth (X2=0) or Moon (X2=2)
		STORE	TFF/RTMU	; Store for time-to-free-fall calculations
		DLOAD*			; Load with indexing
			MINPERE,2	; Minimum perigee for Earth or Moon
		STORE	HPERMIN		; Store minimum perigee height
		SLOAD	BHIZ		; Single load and branch if zero
			X2		; Load body indicator
			EARTHPAD	; If X2=0: use Earth pad radius
		GOTO			; Otherwise use Moon pad
			MOONPAD
# Page 715

; This ship selected: Update LM state vector and orbital parameters

THISSHIP	TC	INTPRET		; Enter interpretive mode
		CALL			; Call state vector update for LM
			THISPREC	; This precision integration routine
		GOTO			; Join common path
			BOTHSHIP	; Continue to parameter setup

; THE FOLLOWING CONSTANTS ARE PAIRWISE INDEXED. DO NOT SEPARATE PAIRS.
; These constants define minimum perigee heights: 300,000 feet for Earth
; (clearing the atmosphere for safe orbital operations) and 35,000 feet for
; the Moon (clearing terrain and providing safe abort margins during descent).

MINPERM		2DEC	10668 B-27	; 35 KFT min perigee height for Moon (-27)M
					; Clears lunar terrain and provides abort margin

MINPERE		2DEC	91440 B-29	; 300 KFT (-29)M for Earth
					; Clears atmosphere for orbital operations

; ============================================================================
; REFERENCE RADIUS SETUP FOR ALTITUDE CALCULATIONS
;
; Altitude above "pad" is computed as orbital radius minus reference radius.
; For Earth: Use Kennedy Space Center Pad 37-B radius (launch site reference).
; For Moon: Use radius to lunar landing site (RLS vector magnitude).
; ============================================================================

EARTHPAD	DLOAD	CLRGO		; Load double precision value
			RPAD		; Pad 37-B radius scaled at (-29)M
			V82EMFLG	; Clear flag: indicates Earth scaling for SR30.1
			BOTHPAD		; Continue to common calculations

MOONPAD		VLOAD	ABVAL		; Load vector and compute magnitude
			RLS		; Lunar landing site position vector (-27)M
		SET			; Set flag bit
			V82EMFLG	; Set flag: indicates Moon scaling for SR30.1
BOTHPAD		STCALL	RPADTEM		; Store reference radius, then call
			SR30.1		; CALCULATE ORBITAL PARAMETERS
					; This key subroutine computes:
					; - RAPO (apogee radius)
					; - RPER (perigee radius)
					; - HAPO/HPER (apogee/perigee altitudes)
					; - TPER (time to perigee)
					; - TFF (time to free fall)
; After SR30.1 completes, adjust times for the computation duration.
; The elapsed time since V82GOFF1 started is added to the orbital parameters
; so displayed times increment continuously without appearing to "jump back"
; during recalculations.

		RTB	DSU		; Return to basic, then subtract
			LOADTIME	; Load current mission time
			TSTART82	; Subtract start time: elapsed time delta
		STORE	TSTART82	; Save elapsed time for corrections

; Branch based on whether time-to-free-fall (TFF) was computable.
; SR30.1 sets -TPER=0 if perigee height is below minimum (300 KFT Earth
; or 35 KFT Moon), indicating the orbit doesn't achieve safe altitude.

		DLOAD	BZE		; Load and branch if zero
			-TPER		; Time to perigee (negative value)
			TICKTIFF	; If -TPER=0: TFF was computed

; Time to perigee is non-zero: The orbit achieves safe perigee altitude.
; In this case, TFF was not computed (set to -59M59S as sentinel value).
; Display both TFF and -TPER, but only increment -TPER each second.

TICKTPER	DLOAD	DAD		; Load double precision, then add
			-TPER		; Time to perigee (negative)
			TSTART82	; Add elapsed time correction
		STORE	-TPER		; Store corrected time to perigee
		EXIT			; Exit interpretive mode
		CAF	BIT1		; Load bit 1 constant
		TS	V82FLAGS	; Set flag: increment only -TPER
		TC	ENDOFJOB	; Terminate this job

; Time to perigee is zero: Perigee altitude is below minimum threshold.
; In this case, TFF was computed successfully. Display TFF but not -TPER,
; and increment only TFF each second to show time counting down to impact.

TICKTIFF	DLOAD	DAD		; Load double precision, then add
			TFF		; Time to free fall (negative)
			TSTART82	; Add elapsed time correction
		STORE	TFF		; Store corrected time to free fall
		EXIT			; Exit interpretive mode
		CAF	BIT2		; Load bit 2 constant
		TS	V82FLAGS	; Set flag: increment only TFF
		TC	ENDOFJOB	; Terminate this job

# Page 716

; ============================================================================
; TICKTEST - One-Second Display Update Routine
;
; This waitlist task runs every second to increment the displayed time values
; (TFF or -TPER), giving the crew a continuously updating countdown. The routine
; perpetuates itself until Verb 82 is terminated (BIT 5 of EXTVBACT cleared).
; ============================================================================

TICKTEST	CAF	BIT5		; Load bit 5 mask
		MASK	EXTVBACT	; Check if extended verb still active
		CCS	A		; Check, compare, skip if positive
		TC	DOTICK		; Verb active: continue ticking
		
		; Verb 82 has been terminated by crew or system. Clean up and exit.
		
		CAF	PRIO25		; Load priority 25
		TC	NOVAC		; Create new job (can't call ENDEXT in interrupt)
		EBANK=	EXTVBACT	; Set EBANK for extended verb control
		2CADR	ENDEXT		; Address of extended verb termination routine

		TC	TASKOVER	; End this waitlist task

; Verb 82 still active: Re-schedule this task for one second from now,
; then update the appropriate time display based on V82FLAGS.

DOTICK		CAF	1SEC		; Load one-second time constant
		TC	WAITLIST	; Schedule task on waitlist
		EBANK=	TFF		; Set EBANK for orbital parameter storage
		2CADR	TICKTEST	; Re-invoke this routine in one second

; Determine which time value to increment based on flags set by TICKTPER
; or TICKTIFF. The INDEX instruction causes a computed branch.

		CAF	THREE		; Load bits 1 and 2 mask
		MASK	V82FLAGS	; Mask the flag register
		INDEX	A		; Use result as branch offset
		TC	+1		; Branch based on flag state:
					; A=0: no flags set (no increment)
					; A=1: BIT1 set (increment -TPER)
					; A=2: BIT2 set (increment TFF)
		TC	TASKOVER	; No flags set: don't change TFF or -TPER
		TC	TPERTICK	# ONLY BIT 1 SET. INCR -TPER BY 1 SEC.

; ============================================================================
; TIME INCREMENT ROUTINES
;
; TFFTICK and TPERTICK are called once per second by the TICKTEST waitlist
; routine to update the displayed time values. These routines add one second
; to the displayed times, creating smooth countdown displays on the DSKY for
; time-to-free-fall and time-to-perigee during orbital parameter monitoring.
; The routines use double-precision addition (DAS) for accuracy.
; ============================================================================

TFFTICK		CAF	1SEC		; Load one second constant
		TS	L		; Store in L register (lower accumulator)
		CAF	ZERO		; Load zero into A register
		DAS	TFF		; Double-precision add to TFF
					; (adds 1 second, incrementing toward zero)
		TC	TASKOVER	; Complete task and continue loop

TPERTICK	CAF	1SEC		; Load one second constant
		TS	L		; Store in L register
		CAF	ZERO		; Load zero into A register
		DAS	-TPER		; Double-precision add to -TPER
					; (adds 1 second, incrementing toward zero)
		TC	TASKOVER	; Complete task and continue loop

# Page 717

; ============================================================================
; TRANSITION: From precision state vector updates to average-G calculations
;
; When the average-G flag is set, the AGC uses a simplified orbital mechanics
; model that approximates gravitational effects without full numerical
; integration. This mode recalculates orbital parameters approximately once
; per second using the current state vector, providing continuous monitoring
; during coast phases without the computational overhead of precision updates.
; ============================================================================

V82GON		EXIT			; Exit interpretive mode
					; AVERAGE G ON: Use current state vector
					; for orbital parameter calculations

; Set up a background job to perform continuous orbital calculations.
; This job runs at priority 7 (lower than critical guidance functions)
; and recalculates orbital parameters approximately once per second.

		CAF	PRIO7		; Load priority 7 (less than Lambert)
		TC	FINDVAC		; Find vacant core set for job
		EBANK=	TFF		; Set E-bank for TFF variable access
		2CADR	V82GON1		; Address of computation job
					; V82GON1 performs orbit calculations
					; about proper body approx once per sec

; Wait for the first orbital calculation to complete before displaying results.
; This ensures the crew sees valid data rather than stale values from a
; previous calculation.

		RELINT			; Re-enable interrupts
		CCS	NEWJOB		; Check if new job pending
		TC	CHANG1		; Wait for V82GON1 to complete
					; NOTE: V82GON1 (PRIO7, FINDVAC job)
					; completes before V82GON (PRIO7, NOVAC)

; Display the computed orbital parameters on the DSKY for crew monitoring.

V82REDSP	CAF	V16N44		; Load V16N44 display code
					; Monitor HAPO, HPER, TFF
		TC	BANKCALL	; Cross-bank subroutine call
		CADR	GOXDSPF		; Call display formatting routine
		TC	B5OFF		; TERM: Tells V82GON1 to kill itself
		TC	B5OFF		; PROC: Ditto
		TC	V82REDSP	# RECYCLE

; ============================================================================
; V82GON1: BACKGROUND JOB THAT RECALCULATES ORBITAL PARAMETERS EVERY SECOND
;
; This job was started by V82GON and perpetuates itself indefinitely until
; terminated by the astronaut. It fetches the current state vector (position
; and velocity) from the navigation system and passes it to SR30.1 for orbital
; parameter computation. Display updates once per second showing real-time
; orbital data as the spacecraft moves.
;
; COMMENT-ONLY READERS: The computer continuously monitors the spacecraft's
; orbit, recalculating and displaying altitude parameters every second during
; Average-G navigation.
; ============================================================================

V82GON1		TC	INTPRET		# THIS EXEC PROGRAM PERPETUATES ITSELF
					# ONCE A SEC UNTIL BIT 5 OF EXTVBACT =0.
		VLOAD	GOTO		# HOLDS OFF CCS NEWJOB BETWEEN RN AND
			RN		#   VN FETCH SO RN , VN ARE FROM SAME
			NEXTLINE	#   STATE VECTOR UPDATE.
; Fetch current position and velocity from navigation state vector.
; RN (position) scaled at (-29)M = 1.862645 nanometers per bit.
; VN (velocity) scaled at (-7)M/CS = 0.7629395 cm/sec per bit.
; Must fetch atomically to avoid inconsistency during state vector updates.

NEXTLINE	STOVL	RONE		#  RN AT (-29)M FOR EARTH OR MOON
			VN
		STORE	VONE		# VN AT (-7)M/CS FOR EARTH OR MOON
; Determine whether orbit calculations should use Earth or Moon as central body.
; MOONTHIS flag set during lunar operations, reset during Earth/cislunar ops.
		BON	GOTO
			MOONTHIS	# FLAG INDICATES BODY ABOUT WHICH ORBITAL
			MOONGON		#    CALCULATIONS ARE TO BE PERFORMED.
			EARTHGON	#   IF SET - MOON , IF RESET - EARTH.

; Moon-centered orbit: Load lunar gravitational parameter and reference radius.
; During lunar operations (LM descent, ascent, lunar orbit), Moon is central body.
MOONGON		SET	DLOAD
			V82EMFLG	# INDICATE MOON SCALING FOR SR30.1
			1/RTMUM		# LUNAR PARAMETERS LOADED HERE FOR SR30.1
		STODL	TFF/RTMU	# 1/sqrt(mu_moon) for time calculations
			MINPERM		# Minimum periapsis for lunar surface (35 kft)
		STOVL	HPERMIN
			RLS		#  SCALED AT (-27)M.
		ABVAL	GOTO		# Compute lunar landing site radius
			V82GON2

; Earth-centered orbit: Load terrestrial gravitational parameter and reference.
; Used during Earth orbit, translunar/transearth coast when Earth dominant.
EARTHGON	CLEAR	DLOAD
			V82EMFLG	# INDICATE EARTH SCALING FOR SR30.1
			1/RTMUE		# EARTH PARAMETERS LOADED HERE FOR SR30.1
		STODL	TFF/RTMU	# 1/sqrt(mu_earth) for time calculations
			MINPERE		# Minimum periapsis for Earth (300 kft altitude)
		STODL	HPERMIN
			RPAD		# Earth launch pad radius
V82GON2		STCALL	RPADTEM		# COMMON CODE FOR EARTH & MOON.
			SR30.1		# Call orbital parameter computation
# Page 718
		EXIT
; ============================================================================
; V82GON3: ONE-SECOND DELAY LOOP FOR CONTINUOUS ORBITAL MONITORING
;
; After SR30.1 completes and display updates, this section checks if monitoring
; should continue. If Verb 82 is still active (BIT5 set), delays one second
; and repeats V82GON1. This creates continuous orbital parameter updates
; visible to crew on DSKY display.
;
; COMMENT-ONLY READERS: The computer waits one second, then recalculates and
; updates the orbital display. Cycle repeats until astronaut terminates Verb 82.
; ============================================================================

V82GON3		CAF	BIT5
		MASK	EXTVBACT	# SEE IF ASTRONAUT HAS SIGNALLED TERMINATE
		EXTEND
		BZF	ENDEXT		# YES, TERMINATE VB 82 LOOP
; Astronaut has not terminated Verb 82. Delay one second then repeat.
		CAF	1SEC		# Load one-second delay constant (100 centiseconds)
		TC	BANKCALL	# WAIT ONE SECOND BEFORE REPEATING
		CADR	DELAYJOB	#   ORBITAL PARAMETER COMPUTATION.
		TC	V82GON1		# Restart monitoring cycle

SPLRET		=	V82GON3

# Page 719
# SUBROUTINE NAME: SR30.1
# MOD NO: 0								DATE: 16 FEB 67
# MOD BY: R. R. BAIRNSFATHER						LOG SECTION: R32
# MOD NO: 1	MOD BY: R. R. BAIRNSFATHER	DATE: 11 APR 67		SR30.1 CHANGED TO ALLOW MONITOR OPERN
# MOD NO: 2	MOD BY: R. R. BAIRNSFATHER	DATE: 14 APR 67		ADD OVFL CK FOR RAPO
# MOD NO: 3	MOD BY ALONSO			DATE: 11 DEC 67		SUBROUTINE REWRITTEN
# MOD NO: 4	MOD BY ALONSO			DATE: 26 MAR 68		PROG MOD TO HANDLE DIF EARTH/MOON SCALE
# MOD NO: 5	MOD BY: R. R. BAIRNSFATHER	DATE: 6 AUG 68		OVFL CK FOR HAPO & HPER. VOIDS MOD #2.
#
# NEW FUNCTIONAL DESCRIPTION:  ORBITAL PARAMETERS DISPLAY FOR NOUNS 32 AND 44.
# SR30.1 CALLS TFFCONMU AND TFFRP/RA TO CALCULATE RPER (PERIGEE RADIUS),
# RAPO (APOGEE RADIUS), HPER (PERIGEE HEIGHT ABOVE LAUNCH PAD OR LUNAR
# LANDING SITE), HAPO (APOGEE HEIGHT AS ABOVE), TPER (TIME TO PERIGEE),
# TFF (TIME TO INTERSECT 300 KFT ABOVE PAD OR 35KFT ABOVE LANDING SITE).
# IF HPER IS GREATER THAN OR EQUAL TO HPERMIN, CALCULATES TPER AND STORES
# NEGATIVE IN -TPER.  OTHERWISE STORES +0 IN -TPER.  WHENEVER TPER IS
# CALCULATED, TFF IS NOT COMPUTABLE AND DEFAULTS TO -59MIN 59SEC.  IF HAPO
# WOULD EXCEED 9999.9 NM, IT IS LIMITED TO THAT VALUE FOR DISPLAY.
#
# ADDENDUM:	HAPO AND HPER SHOULD BE CHANGED TO READ HAPOX AND HPERX IN THE
#		ABOVE REMARKS.
#
# CALLING SEQUENCE:	CALL
#				SR30.1
#
# SUBROUTINES CALLED:	TFFCONMU, TFFRP/RA, CALCTPER, CALCTFF
#
# NORMAL EXIT MODE:	CALLING LINE +1 (STILL IN INTERPRETIVE MODE)
#
# ALARMS:	NONE
#
# OUTPUT:	RAPO	(-29) M EARTH	APOGEE RADIUS	EARTH CENTERED COORD.
#			(-27) M MOON			MOON CENTERED COORD.
#		RPER	(-29) M EARTH	PERIGEE RADIUS	EARTH CENTERED COORD.
#			(-27) M MOON			MOON CENTERED COORD.
#		HAPOX	(-29) M		APOGEE ALTITUDE ABOVE PAD OR LAND. SITE MAX VALUE LIMITED TO 9999.9 NM.
#		HPERX	(-29) M		PERIGEE ALT. ABOVE PAD OR LAND. SITE    MAX VALUE LIMITED TO 9999.9 NM.
#		TFF	(-28) CS	TIME TO 300KFT OR 35KFT ALTITUDE
#		-TPER	(-28) CS	TIME TO PERIGEE
#
# ERASABLE INITIALIZATION REQUIRED --
#	TFF/RTMU	(+17) EARTH	RECIPROCAL OF PROPER GRAV CONSTANT FOR
#			(+14) MOON	EARTH OR MOON = 1/SQRT(MU).
#	RONE		(-29) M		STATE VECTOR
#	VONE		(-7)  M/CS	STATE VECTOR
#	RPADTEM		(-29) M EARTH	RADIUS OF LAUNCH PAD OR LUNAR LANDING
#			(-27) M MOON	SITE.
#	HPERMIN		(-29) M EARTH	(300 OR 35) KFT MINIMUM PERIGEE ALTITUDE
#			(-27) M MOON	ABOVE LAUNCH PAD OR LUNAR LANDING SITE.
#	V82EMFLG	(INT SW BIT)	RESET FOR EARTH, SET FOR MOON.
#
# DEBRIS:	QPREG, PDL, S2

# Page 720
		COUNT*	$$/SR30S

; ============================================================================
; SR30.1: ORBITAL PARAMETER COMPUTATION CORE ENGINE
;
; This subroutine computes orbital parameters from state vector (position and
; velocity). Handles both Earth-centered and Moon-centered coordinates with
; appropriate rescaling. Calls TFFCONMU for conic analysis, TFFRP/RA for
; apogee/perigee radii, then computes altitudes above surface and times.
;
; COMMENT-ONLY READERS: This is the mathematical heart of orbital display.
; It takes spacecraft position and velocity, applies orbital mechanics equations,
; and produces the apogee/perigee altitudes and times displayed to crew.
;
; CODE-ALONG READERS: Note different scaling conventions for Earth vs Moon.
; Earth uses (-29)M for position, Moon uses (-27)M. This reflects different
; planetary radii and precision requirements. Subroutine must rescale Moon
; state vectors before calling conic subroutines which expect uniform scaling.
; ============================================================================

SR30.1		SETPD	STQ		# INITIALIZE PUSHDOWN LIST.
			0
			S2		# Save return address in S2
					# SR30.1 INPUT:	RONE AT (-29)M EARTH/MOON
					#		VONE AT (-7)M/CS
					# TFFCONMU, TFFRP/RA, CALCTPER, AND CALCTFF
					# CALLS REQUIRE:
					# EARTH CENTERED (NO RESCALING REQUIRED)
					#	RONE SCALED TO B-29 M
					#	VONE SCALED TO B-7  M/CS
					# MOON CENTERED (RESCALING REQUIRED)
					#	RONE SCALED TO B-27 M
					#	VONE SCALED TO B-5  M/CS
; Rescale Moon-centered state vectors to match conic subroutine expectations.
; Moon position comes in at (-27)M (7.45 mm per bit), must be shifted to (-29)M.
; Moon velocity comes in at (-5)M/CS (30.5 cm/sec per bit), must be shifted to (-7)M/CS.
		BOFF	VLOAD
			V82EMFLG	# OFF FOR EARTH, ON FOR MOON.
			TFFCALLS	# Earth: Skip rescaling, already correct scale
			RONE		# Moon: Load position vector
		VSL2			# Shift left 2 bits: (-27)M → (-29)M
		STOVL	RONE		# Store rescaled position
			VONE		# Load velocity vector
		VSL2			# Shift left 2 bits: (-5)M/CS → (-7)M/CS
		STORE	VONE		# Store rescaled velocity
; ============================================================================
; CONIC SUBROUTINE CALLS: Compute orbital elements from state vector
;
; TFFCONMU computes time-to-free-fall and orbital elements (eccentricity,
; semi-major axis, etc.) using vis-viva equation and conic section geometry.
; TFFRP/RA computes apogee radius (RAPO) and perigee radius (RPER) from
; the conic parameters. Results scaled at (-29)M for Earth, (-27)M for Moon.
;
; Source: CONIC_SUBROUTINES.agc
; ============================================================================

TFFCALLS	CALL
			TFFCONMU	# Compute orbital elements and TFF
		CALL			# TFFRP/RA COMPUTES RAPO,RPER.
			TFFRP/RA	# Returns RAPO in MPAC, RPER saved
					# RETURNS WITH RAPO IN D(MPAC).
; Compute apogee altitude above reference surface (Earth pad or lunar site).
; RAPO is radius from central body center. Subtracting RPADTEM (reference
; radius) gives altitude above surface. For Earth, reference is launch pad
; radius (~6378 km). For Moon, reference is landing site radius (~1738 km).
		DSU
			RPADTEM		# HAPO = RAPO - reference_radius
		BOFF	SR2R		# NEED HAPO AT (-29)M FOR DISPLAY.
					# IF MOON CENTERED, RESCALE FROM (-27)M.
					# IF EARTH CENTERED ALREADY AT (-29)M.
			V82EMFLG	# OFF FOR EARTH, ON FOR MOON.
			+1		# Earth: Already at (-29)M, skip rescale
; Moon case: Shift right 2 bits to convert (-27)M → (-29)M for display.
		CALL			# IF RAPO > MAXNM, SET RAPO =9999.9 NM.
			MAXCHK		# OTHERWISE STORE (RAPO-RPADTEM) IN HAPO.
; Store apogee altitude for DSKY display (Verb 16 Noun 44 R1).
STORHAPO	STODL	HAPOX		# HAPOX = apogee altitude above surface
			RPER		# Now process perigee radius
; Compute perigee altitude above reference surface using same logic as apogee.
; RPER is radius from central body center. Subtracting reference radius gives
; altitude above surface (launch pad for Earth, landing site for Moon).
		DSU
			RPADTEM		# HPER = RPER - reference_radius
					# GIVES HPER AT (-29)M EARTH, (-27)M MOON.
		STORE	MPAC +4		# SAVE THIS FOR COMPARISON TO HPERMIN.
		BOFF	SR2R		# NEED HPER AT (-29)M FOR DISPLAY.
					# IF MOON CENTERED, RESCALE FROM (-27)M.
					# IF EARTH CENTERED ALREADY AT (-29)M.
			V82EMFLG	# OFF FOR EARTH, ON FOR MOON.
			+1		# Earth: Already at (-29)M, skip rescale
; Moon case: Shift right 2 bits to convert (-27)M → (-29)M for display.
		CALL			# IF HPER > MAXNM, SET HPER = 9999.9 NM.
			MAXCHK		# Limits display to maximum readable value
# Page 721
; Store perigee altitude for DSKY display (Verb 16 Noun 44 R2).
STORHPER	STODL	HPERX		# HPERX = perigee altitude above surface
			MPAC 	+4	# Retrieve saved HPER for threshold check
; ============================================================================
; MINIMUM ALTITUDE CHECK: Determine if perigee low enough for TPER calculation
;
; HPERMIN thresholds: Earth = 300 kft (~91 km), Moon = 35 kft (~10.7 km)
; If perigee below threshold (spacecraft in low orbit or suborbital trajectory),
; time-to-perigee (TPER) is meaningful for crew. Otherwise set TPER to zero.
;
; COMMENT-ONLY READERS: If spacecraft perigee is too high (above 300,000 feet
; for Earth or 35,000 feet for Moon), time to perigee is not computed since
; spacecraft will not pass through low altitude. Otherwise computer calculates
; how long until lowest point in orbit.
; ============================================================================

		DSU	BPL		# HPERMIN AT (-29)M FOR EARTH, (-27)M MOON
			HPERMIN		# IF HPER L/   HPERMIN (300 OR 35)KFT,
			DOTPER		#  THEN ZERO INTO -TPER.
; Perigee above minimum threshold: TPER not meaningful, set to zero.
		DLOAD	GOTO		#   OTHERWISE CALCULATE TPER.
			HI6ZEROS	# Load zero constant
			SKIPTPER	# Skip TPER calculation
; Perigee below minimum threshold: Calculate time to perigee passage.
; CALCTPER uses Kepler's equation to compute time from current position
; to perigee passage, considering orbital eccentricity and current true anomaly.
DOTPER		DLOAD	CALL
			RPER		# Load perigee radius for CALCTPER
			CALCTPER	# Compute time to perigee (positive value)
		DCOMP			# Negate: TPER IS PUT NEG INTO -TPER.
; Store negative time-to-perigee. Displayed as negative HMS on DSKY to indicate
; future event. Value increments toward zero as spacecraft approaches perigee.
SKIPTPER	STODL	-TPER		# -TPER at (-28) centiseconds
			HPERMIN		# HPERMIN AT (-29)M FOR EARTH, (-27)M MOON
; ============================================================================
; TIME TO FREE FALL (TFF) CALCULATION
;
; TFF is time until spacecraft reaches threshold altitude (300 kft Earth,
; 35 kft Moon). Used for reentry planning or abort timing. CALCTFF computes
; intersection time with reference altitude sphere. If spacecraft perigee
; above threshold, TFF set to 59m59s (maximum displayable time) indicating
; no intersection during current orbit.
;
; COMMENT-ONLY READERS: Computer calculates time until spacecraft reaches
; critical altitude threshold. For reentry from Earth orbit, this is time
; until atmospheric entry at 300,000 feet. Display shows countdown in
; minutes and seconds.
; ============================================================================

		DAD	CALL
			RPADTEM		# Sum = HPERMIN + RPADTEM = threshold radius
					# RPADTEM AT (-29)M FOR EARTH, (-27)M MOON
			CALCTFF		# Compute time to free fall
					# GIVES 59M59S FOR TFF IF HPER G/
		DCOMP			# Negate result (display convention)
					#   HPERMIN + RPADTEM. (TPER WAS NON ZERO)
		STCALL	TFF		# Store TFF at (-28) centiseconds
			S2		# Return to caller via saved address

; ============================================================================
; MAXCHK SUBROUTINE: Display value limiter
;
; PURPOSE: Clamp MPAC value to maximum displayable range for DSKY.
; MAXNM = 9999.9 nautical miles at (-29)M scaling = 18,519,400 meters.
; DSKY cannot display values above 99999 in 5-digit field. This routine
; ensures altitude displays don't overflow or show meaningless values.
;
; INPUTS:  MPAC = Altitude or radius value (any magnitude)
; OUTPUTS: MPAC = min(input_value, 9999.9 NM)
; RETURNS: RVQ (return via Q register)
;
; USAGE: Called after altitude calculations to prevent display overflow.
; Also used by P30-P37 programs for target parameter limiting.
; ============================================================================

MAXCHK		DSU	BPL		# IF C(MPAC) > 9999.9 NM. MPAC = 9999.9 NM.
			MAXNM		# Subtract maximum value
			+3		# Branch if result positive (input > max)
					# OTHERWISE C(MPAC) = B(MPAC).
; Input exceeds maximum: Add MAXNM back to restore MAXNM in MPAC.
		DAD	RVQ		# MPAC = (MPAC - MAXNM) + MAXNM = MAXNM
			MAXNM		# Clamp to maximum displayable value
; Input within range: Load MAXNM then return (effectively returns input).
 +3		DLOAD	RVQ		# (USED BY P30 - P37 ALSO)
			MAXNM		# This loads MAXNM but doesn't affect result

; Maximum displayable nautical miles for DSKY altitude fields.
; 9999.9 NM = 18,519.4 km at (-29) meter scaling for display.
MAXNM		2OCT	0106505603	# 9999.9 NM at (-29)M scaling

# Page 722 (empty page)
