# Copyright:    Public domain.
# Filename:     R30.agc
# Purpose:      Part of the source code for Colossus 2A, AKA Comanche 055.
#               It is part of the source code for the Command Module's (CM)
#               Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:    yaYUL
# Contact:      Hartmuth Gutsche <hgutsche@xplornet.com>.
# Website:      www.ibiblio.org/apollo.
# Pages:	514-524
# Mod history:  2009-05-09 HG    Started adapting from the Colossus249/ file
#               of the same name, using Comanche055 page
#               images 0514.jpg - 0524.jpg.
#
# This source code has been transcribed or otherwise adapted from digitized
# images of a hardcopy from the MIT Museum.  The digitization was performed
# by Paul Fjeld, and arranged for by Deborah Douglas of the Museum.  Many
# thanks to both.  The images (with suitable reduction in storage size and
# consequent reduction in image quality as well) are available online at
# www.ibiblio.org/apollo.  If for some reason you find that the images are
# illegible, contact me at info@sandroid.org about getting access to the
# (much) higher-quality images which Paul actually created.
#
# Notations on the hardcopy document read, in part:
#
#    Assemble revision 055 of AGC program Comanche by NASA
#    2021113-051.  10:28 APR. 1, 1969
#
#    This AGC program shall also be referred to as
#            Colossus 2A

; ============================================================================
; FILE: R30.agc
; MODULE: COMEKISS Subsystem (Orbital Navigation)
; MISSION PHASE: all-phases
;
; TL;DR: Orbit parameter display routine showing current orbital characteristics
;        to crew via DSKY. Converts state vectors to Keplerian elements (apogee,
;        perigee, period, eccentricity) with coordinate frame transformations
;        from inertial to rotating reference frames. Provides crew situation
;        awareness of orbital status throughout mission phases.
;
; COMMENT-ONLY READERS: This program displayed the spacecraft's current orbit
;        shape and altitude to help crew understand their position around Earth
;        or Moon. Think of it as the "where am I and where am I going" display.
; CODE-ALONG READERS: Study state vector to Keplerian element conversion and
;        reference frame transformation mathematics. Note Earth/Moon scaling
;        differences and DSKY display formatting techniques.
; ============================================================================

# Page 514
# SUBROUTINE NAME:  V82CALL
# MOD NO: 0					DATE: 16 FEB 67
# MOD BY: R. R. BAIRNSFATHER			LOG SECTION:  R30
# MOD NO: 1	MOD BY:  R. R. BAIRNSFATHER	DATE: 11 APR 67		SR30.1 CHANGED TO ALLOW MONITOR OPERN
# MOD NO: 2	MOD BY:  ALONSO			DATE: 11 DEC 67		VB82 PROGRAM REWRITTEN
# MOD NO: 3	MOD BY:  ALONSO			DATE: 26 MAR 68		PROG MOD TO HANDLE DIF EARTH/MOON SCALE
#
# NEW FUNCTIONAL DESCRIPTION:	CALLED BY VERB 82 ENTER.  PRIORITY 10.
# USED THROUGHOUT.		CALCULATE AND DISPLAY ORBITAL PARAMETERS

; ============================================================================
; V82CALL - VERB 82 ORBITAL PARAMETERS DISPLAY ENTRY POINT
;
; When crew members entered "VERB 82 ENTER" on the DSKY keyboard, this routine
; displayed the spacecraft's current orbital shape around Earth or Moon. The
; astronauts used this throughout the mission to verify their trajectory was
; correct after burns and to monitor their orbit during coast phases.
;
; The display showed three key numbers:
; - HAPO (apogee altitude): highest point in the orbit
; - HPER (perigee altitude): lowest point in the orbit  
; - TFF (time to fall): time until atmospheric entry (Earth only)
;
; This gave crew immediate awareness of their orbital situation without
; requiring ground control calculations.
; ============================================================================
#
; ============================================================================
; OPERATIONAL MODES - Two different operating scenarios based on guidance state
; ============================================================================
;
; MODE 1 - AVERAGE G OFF (Normal orbit display mode):
; The crew first saw display V04N06 asking which spacecraft to track:
; - R2=1 meant "this ship" (the Command Module they were flying in)
; - Any other value meant "other ship" (the Lunar Module if separated)
;
; The computer then calculated and displayed on the DSKY:
; - HAPO: Apogee height (highest point above surface)
; - HPER: Perigee height (lowest point above surface)  
; - TFF: Time to fall to 300,000 feet altitude (Earth atmospheric entry)
;
; These displays updated continuously. If TFF showed -59M59S, it meant the
; orbit would never enter atmosphere (stable orbit). The crew could also
; request N32 display to see TPER (time to perigee passage).
;
; MODE 2 - AVERAGE G ON (Continuous monitoring during powered flight):
; During engine burns, the routine automatically updated every 2 seconds
; showing the constantly changing orbit. This helped crew verify the burn
; was achieving the desired trajectory. In P11 (Earth orbit insertion),
; splashdown point was also calculated for abort planning.
;
; ============================================================================

# 1.	IF AVERAGE G IS OFF:
#		FLASH DISPLAY V04N06.  R2 INDICATES WHICH SHIP'S STATE VECTOR IS
#			TO BE UPDATED.  INITIAL CHOICE IS THIS SHIP (R2=1).  ASTRONAUT
#			CAN CHANGE TO OTHER SHIP BY V22EXE. WHERE X IS NOT EQ 1.
#		SELECTED STATE VECTOR UPDATED BY THISPREC (OTHPREC).
#		CALLS SR30.1 (WHICH CALLS TFFCONMU + TFFRP/RA) TO CALCULATE
#			RPER (PERIGEE RADIUS), RAPO (APOGEE RADIUS), HPER (PERIGEE
#			HEIGHT ABOVE LAUNCH PAD OR LAUNAR LANDING SITE), HARD (APOGEE
#			HEIGHT AS ABOVE), TPER (TIME TO PERIGEE), TFF (TIME TO
#			INTERSECT 300 KFT ABOVE PAD OR 35KFT ABOVE LANDING SITE).
#		FLASH MONITOR V16N44 (HAPO, HPER, TFF).  TFF IS -59MS59S IF IT WAS
#			NOT COMPUTABLE, OTHERWISE IT INCREMENTS ONCE PER SECOND.
#			ASTRONAUT HAS OPTION TO MONITOR TPER BY KEYING IN N 32 E.
#			DISPLAY IS IN HMS, IS NEGATIVE (AS WAS TFF), AND INCREMENTS
#			ONCE PER SECOND ONLY IF TFF DISPLAY WAS -59M59S.
# 2.	IF AVERAGE G IS ON:
#		CALLS SR30.1 APPROX EVERY TWO SECS.  STATE VECTOR IS ALWAYS
#			FOR THIS VEHICLE.  V82 DOES NOT DISTURB STATE VECTOR.  RESULTS
#			OF SR30.1 ARE RAPO, RPER, HAPO, HPER, TPER, TFF.
#		FLASH MONITOR V16N44 (HAPO, HPER, TFF).
#			IF MODE IS P11, THEN CALL DELRSPL SO ASTRONAUT CAN MONITOR
#			RESULTS BY N50E.  SPLASH COMPUTATION DONE ONCE PER TWO SECS.

# ADDENDUM:  HAPO AND HPER SHOULD BE CHANGED TO READ HAPOX AND HPERX IN THE
#	     ABOVE REMARKS.
#
# CALLING SEQUENCE: VERB 82 ENTER.
#
# SUBROUTINES CALLED:	SR30.1, GOXDSPF
#			MAYBE -- THISPREC, OTHPREC, LOADTIME, DELRSPL
#
# NORMAL EXIT MODES:  TC ENDEXT
#
# ALARMS:  NONE
#
# OUTPUT:	HAPOX	(-29) M
#		HPERX	(-29) M
#		RAPO	(-29) M EARTH
#			(-27) M MOON
# Page 515
#		RPER	(-29) M EARTH
#			(-27) M MOON
#		TFF	(-28) CS	CONTAINS NEGATIVE QUANTITY
#		-TPER	(-28) CS	CONTAINS NEGATIVE QUANTITY
#		RSP-RREC(-29) M		IF DELRSPL CALLED
#
# ERASABLE INITIALIZATION REQUIRED: STATE VECTOR.
#
# DEBRIS:	QPRET, RONE, VONE,TFF/RTMU, HPERMIN, RPADTEM, V82EMFLG.
#		MAYBE:  TSTART82, V82FLAGS, TDEC1.

		EBANK=	HAPOX
		BANK	31
		SETLOC	R30LOC
		BANK
		COUNT*	$$/R30

; ============================================================================
; VERB 82 MAIN ENTRY POINT
;
; The spacecraft enters interpretive mode and checks whether Average G
; (guidance acceleration integration) is active. This flag determines whether
; the display runs once (crew-initiated) or continuously (during burns).
; ============================================================================

; ============================================================================
; VERB 82: ORBITAL PARAMETER DISPLAY
;
; Astronauts initiated this program by pressing VERB 82 ENTER on the DSKY.
; The program calculated and displayed the spacecraft's current orbit shape,
; showing apogee altitude (highest point), perigee altitude (lowest point),
; and time to reach specific altitudes. This gave the crew critical awareness
; of their trajectory around Earth or Moon throughout the mission.
;
; Two operational modes existed:
; 1. Average G OFF - Used during coasting flight when not under thrust.
;    Updated state vector, displayed orbit parameters once, allowed monitoring.
; 2. Average G ON - Used during powered flight (engine firing).
;    Continuously recalculated orbit parameters every two seconds to show
;    how the spacecraft's trajectory was changing as the engine burned.
; ============================================================================

V82CALL		TC	INTPRET
		BON	GOTO		; Branch ON if flag set
			AVEGFLAG	; Check if guidance integration active
			V82GON		# IF AVERAGE G ON (continuous monitoring)
			V82GOFF		# IF AVERAGE G OFF (one-time display)

; ============================================================================
; V82GOFF - AVERAGE G OFF MODE (Crew-Initiated Display)
;
; The crew has requested orbital parameters at this moment. First, the DSKY
; displays V04N06 asking which vehicle's orbit to show: this spacecraft (R2=1)
; or the other vehicle if separated (R2≠1). This was used during lunar missions
; when Command Module and Lunar Module were flying separately.
; ============================================================================

V82GOFF		EXIT			# ALLOW ASTRONAUT TO SELECT VEHICLE
		CAF	TWO		# DESIRED FOR ORBITAL PARAMETERS
		TS	OPTIONX		; Set display mode options
		CAF	ONE
		TS	OPTIONX +1	; Default to "this ship" (R2=1)
		CAF	OPTIONVN	# V 04 N 06 display code
		TC	BANKCALL	; Call display interface routine
		CADR	GOXDSPF		; Generic display flasher
		TC	ENDEXT		# TERMINATE - crew pressed RSET key
		TC	+2		# PROCEED - crew accepted default
		TC	-5		# DATA IN - crew changed selection
					; OPTIONX +1 = 1 FOR THIS VEHICLE
					; OPTIONX +1 ≠ 1 FOR OTHER VEHICLE

; Set up TICKTEST routine to run in 80 milliseconds. This routine will
; decrement the TFF and TPER displays once per second so crew sees time
; counting down to perigee passage or atmospheric entry.

		CAF	BIT4		# 80 MS delay constant
		TC	WAITLIST	; Schedule task on waitlist
		EBANK=	TFF		; Set erasable memory bank
		2CADR	TICKTEST	; Address of countdown update routine

		RELINT			; Re-enable interrupts
; ============================================================================
; V82GOFLP - MAIN RECYCLE LOOP
;
; This is the heart of the calculation loop. Each time through, the computer
; updates the spacecraft's position and velocity to the current time, then
; calculates the resulting orbital parameters. The crew sees updated numbers
; reflecting the current orbit.
; ============================================================================

V82GOFLP	CAF	TFFBANK		# MAJOR RECYCLE LOOP ENTRY
		TS	EBANK		; Set memory bank for TFF variables
		CAF	ZERO
		TS	V82FLAGS	# Zero flags for TICKTEST, inhibits
					# decrementing of TFF and -TPER until
					# new calculations complete

; Schedule orbital calculation as priority 7 job. This allows higher-priority
; guidance and navigation tasks to interrupt if needed during long mission phases.

		CAF	PRIO7		; Priority level 7
		TC	FINDVAC		; Find vacant core set for new job
# Page 516
		EBANK=	TFF		# V82GOFF1 will execute state vector
		2CADR	V82GOFF1	# update and orbit calculations for
					# selected vehicle about proper central body

		RELINT			; Re-enable interrupts
V82STALL	CAF	THREE		# STALL IN THIS LOOP AND WITHOLD V 16 N 44
		MASK	V82FLAGS	# UNTIL STATE VECTOR UPDATE SETS ONE OF
		CCS	A		# OUR FLAG BITS.
		TC	FLAGGON		# EXIT FROM STALL LOOP.
		CAF	1SEC
		TC	BANKCALL
		CADR	DELAYJOB
		TC	V82STALL

FLAGGON		CAF	V16N44		# MONITOR HAPO,HPER,TFF.
		TC	BANKCALL
		CADR	GOXDSPF
		TC	B5OFF		# TERM  THIS TELLS TICKTEST TO KILL ITSELF
		TC	B5OFF		# PROCEED  DITTO
		TC	V82GOFLP	# RECYCLE  RECOMPUTE STATE VECT + DISPLAY

OPTIONVN	VN	0412
V16N44		VN	1644
TFFBANK		ECADR	TFF

; ============================================================================
; STATE VECTOR UPDATE AND PARAMETER SELECTION
;
; In Average G OFF mode, the crew could choose to display orbital parameters
; for either their own spacecraft (Command Module) or the other vehicle
; (Lunar Module when separated). This choice was made via R2 display:
; R2=1 meant "this ship", any other value meant "other ship".
;
; The program first updated the chosen spacecraft's state vector (position
; and velocity) to the current time, accounting for orbital motion since
; the last update. Then it selected the appropriate physical constants:
; - Earth parameters when orbiting Earth (translunar, transearth phases)
; - Moon parameters when orbiting Moon (lunar orbit, landing phases)
;
; This allowed the crew to monitor both vehicles' orbits during rendezvous
; operations, checking that the Lunar Module's ascent trajectory would
; properly intercept the Command Module's orbit for docking.
; ============================================================================

V82GOFF1	TC	INTPRET
		RTB
			LOADTIME
		STORE	TDEC1		# TIME FOR STATE VECTOR UPDATE.
		STORE	TSTART82	# TIME FOR INTERNAL USE.
		EXIT
		CS	OPTIONX +1	# 1 FOR THIS VEHICLE, NOT 1 FOR OTHER
		AD	ONE
		EXTEND
		BZF	THISSHIP
		
; Crew selected "other ship" - update that vehicle's state vector
OTHSHIP		TC	INTPRET
		CALL			# CALL STATE VECTOR UPDATE FOR OTHER SHIP.
			OTHPREC
			
; Common path: Copy updated state vector into calculation variables
BOTHSHIP	VLOAD			# MOVE RESULTS INTO TFFCONIC STORAGE AREAS
			RATT		# TO BE CALLED BY SR30.1.
		STOVL	RONE		# RATT AT (-29)M FOR EARTH OR MOON
			VATT
		STORE	VONE		# VATT AT (-7)M/CS FOR EARTH OR MOON
		
; Select gravitational parameter based on which body we're orbiting
; 1/RTMU = 1/sqrt(mu) where mu is gravitational parameter
		DLOAD*
			1/RTMUE,2	# X2 IS 0 FOR EARTH CENTERED STATE VEC
		STORE	TFF/RTMU	# X2 IS 2 FOR MOON
		DLOAD*			# AS LEFT BY THISPREC OR OTHPREC.
			MINPERE,2
		STORE	HPERMIN		# TFFRTMU, HPERMIN, AND RPADTEM ARE ALL
		SLOAD	BHIZ		# EARTH/MOON PARAMETERS AS SET HERE.
# Page 517
			X2
			EARTHPAD
		GOTO
			MOONPAD
			
; Crew selected "this ship" - update own vehicle's state vector
THISSHIP	TC	INTPRET
		CALL			# CALL STATE VECTOR UPDATE FOR THIS SHIP.
			THISPREC
		GOTO
			BOTHSHIP

; ============================================================================
; EARTH/MOON PHYSICAL CONSTANTS
;
; These constants accommodate the different gravitational environments and
; scaling conventions for Earth and Moon orbits:
;
; 1/RTMU: Inverse square root of gravitational parameter mu (GM)
;   - Earth: 3.986005 x 10^14 m³/s² (larger body, stronger gravity)
;   - Moon: 4.9028 x 10^12 m³/s² (smaller body, weaker gravity)
;
; MINPER: Minimum perigee altitude for trajectory intersection calculations
;   - Earth: 300,000 feet (91.44 km) - entry interface altitude
;   - Moon: 35,000 feet (10.668 km) - landing site altitude
;
; These constants are pairwise indexed (X2=0 for Earth, X2=2 for Moon).
; The indexing was set by THISPREC or OTHPREC based on which body's sphere
; of influence the spacecraft was currently in.
; ============================================================================

# THE FOLLOWING CONSTANTS ARE PAIRWISE INDEXED.  DO NOT SEPARATE PAIRS.

1/RTMUM		2DEC*	.45162595 E-4 B14*
1/RTMUE		2DEC*	.50087529 E-5 B17*

MINPERM		2DEC	10668 B-27	# 35 KFT MIN PERIGEE HEIGHT FOR MOON(-27)M
MINPERE		2DEC	91440 B-29	# 300 KFT (-29)M FOR EARTH

; ============================================================================
; LAUNCH PAD RADIUS SELECTION
;
; The program needed a reference radius for altitude calculations:
; - Earth: Cape Kennedy Launch Complex 37-B radius (Earth center to pad)
; - Moon: Landing site radius (Moon center to surface)
;
; Earth radius was pre-stored. Moon radius was calculated from the RLS
; (Radius Landing Site) vector, which pointed from Moon center to the
; planned landing site coordinates. This allowed altitude measurements
; relative to the actual landing location, not just the Moon's mean radius.
;
; The V82EMFLG flag indicated to SR30.1 which scaling to use in displays.
; ============================================================================

EARTHPAD	DLOAD	CLRGO		# PAD 37-B RADIUS.  SCALED AT (-29)M.
			RPAD
			V82EMFLG	# INDICATE EARTH SCALING FOR SR30.1
			BOTHPAD

MOONPAD		VLOAD	ABVAL		# COMPUTE MOON PAD RADIUS FROM RLS VECTOR.
			RLS		# SCALED AT (-27)M.
		SET
			V82EMFLG	# INDICATE MOON SCALING FOR SR30.1
BOTHPAD		STCALL	RPADTEM
			SR30.1		# CALCULATE ORBITAL PARAMETERS
		EXIT
		CA	MODREG		# ARE WE IN P00
		EXTEND
		BZF	CANDEL		# YES, DO DELRSPL

; ============================================================================
; COUNTDOWN TIMER INITIALIZATION AND CORRECTION
;
; After orbital parameters were calculated, the program needed to display
; time-to-event countdowns that updated once per second on the DSKY. The
; crew could watch the countdown tick: "00500" → "00459" → "00458" seconds
; until reaching a critical altitude or orbital event.
;
; Two countdown timers existed:
; 1. TFF - Time to reach 300,000 ft (Earth entry) or 35,000 ft (Moon landing)
; 2. TPER - Time to perigee (lowest point in orbit)
;
; Only one timer displayed at a time. SR30.1 determined which was relevant:
; - If perigee altitude was high enough (above HPERMIN threshold), display
;   TPER countdown and set TFF to the default value -59m59s (not computed).
; - If perigee was very low (below threshold), display TFF countdown showing
;   time until the spacecraft reached the critical altitude.
;
; Since calculation took time, TSTART82 recorded when SR30.1 began. This
; section corrected the countdown value by the elapsed calculation time,
; ensuring the display showed accurate time remaining from THIS moment.
; ============================================================================

SPLRET1		TC	INTPRET
		RTB	DSU
			LOADTIME
			TSTART82	# PRESENT TIME - TIME V82GOFF1 BEGAN
		STORE	TSTART82	#                SAVE IT
		DLOAD	BZE		# SR30.1 SETS -TPER=0 IF HPER L/
			-TPER		# HPERMIN (300 OR 35) KFT.
			TICKTFF		# (-TPER = 0)
TICKTPER	DLOAD	DAD		# (-TPER NON ZERO)  TFF WAS NOT COMPUTED,
			-TPER		# BUT WAS SET TO 59M59S.DON'T DICK TFF, DO
			TSTART82	# TICK -TPER. DISPLAY BOTH.
		STORE	-TPER           # -TPER CORRECTED FOR TIME SINCE V82GOFF1
		EXIT                    # BEGAN.

# Page 518
		CAF	BIT1
		TS	V82FLAGS	# INFORMS TICKTEST TO INCREMENT ONLY -TPER
		TC	ENDOFJOB

TICKTFF		DLOAD	DAD		# (-TPER=0)  TFF WAS COMPUTED.TICK TFF.
			TFF		# DO NOT TICK -TPER.DISPLAY TFF, BUT NOT
			TSTART82	# -TPER.
		STORE	TFF		# TFF CORRECTED FOR TIME SINCE V82GOFF1
		EXIT			# BEGAN.
		CAF	BIT2
		TS	V82FLAGS	# INFORMS TICKTEST TO INCREMENT ONLY TFF.
		TC	ENDOFJOB

; ============================================================================
; COUNTDOWN TIMER TICK TASK
;
; This waitlist task made the DSKY countdown displays decrement once per
; second, giving the crew continuously updated time-to-event information.
; The task scheduled itself perpetually: each time it ran, it requested
; another execution one second later, creating a self-sustaining countdown.
;
; The crew watched these countdowns during critical mission phases:
; - During entry from lunar return, TFF counted down to 300,000 feet where
;   Earth's atmosphere began affecting the trajectory.
; - During lunar descent, TFF counted down to 35,000 feet where the landing
;   radar would acquire the surface and provide precise altitude data.
; - In orbit, TPER counted down to perigee passage (lowest altitude point).
;
; V82FLAGS controlled which timer decremented (bit 1 = TPER, bit 2 = TFF).
; The task terminated when VERB 82 was cancelled (bit 5 of EXTVBACT cleared).
; ============================================================================

TICKTEST	CAF	BIT5		# THIS WAITLIST PROGRAM PERPETUATES ITSELF
		MASK	EXTVBACT	# ONCE A SEC UNTIL BIT 5 OF EXTVBACT =0.
		CCS	A
		TC	DOTICK
		CAF	PRIO25
		TC	NOVAC		# TERMINATE V 82.CAN'T CALL ENDEXT IN RUPT.
		EBANK=	EXTVBACT
		2CADR	ENDEXT

		TC	TASKOVER

; Schedule this task to run again one second from now, creating perpetual
; countdown. Then increment the appropriate timer based on V82FLAGS bits.

DOTICK		CAF	1SEC		# RE-REQUEST TICKTEST.
		TC	WAITLIST
		EBANK=	TFF
		2CADR	TICKTEST

; V82FLAGS determines which timer to increment: bit 1 = TPER, bit 2 = TFF.
; The INDEX A instruction acts as a computed jump, selecting the correct
; increment routine based on which flag bit is set.

		CAF	THREE
		MASK	V82FLAGS
		INDEX	A		; Computed jump: A=0→TASKOVER, A=1→TPERTICK, A=2→TFFTICK
		TC	+1
		TC	TASKOVER	# IF NO FLAGBITS SET DONT CHANGE TFF OR
					# -TPER, BUT CONTINUE LOOP.
		TC	TPERTICK	# ONLY BIT 1 SET. INCR -TPER BY 1 SEC.

; Increment TFF by 1 second (100 centiseconds). Since TFF is negative,
; incrementing it moves the countdown toward zero: -300 → -299 → -298 seconds.

TFFTICK		CAF	1SEC		# ONLY BIT 2 SET. INCR TFF BY 1 SEC.
		TS	L
		CAF	ZERO
		DAS	TFF		; Double-precision add 1 second to TFF
		TC	TASKOVER

; Increment -TPER by 1 second. Both timers stored as negative values so
; incrementing makes the countdown progress: -500 → -499 → -498 seconds.

TPERTICK	CAF	1SEC
		TS	L
		CAF	ZERO
		DAS	-TPER		; Double-precision add 1 second to -TPER
		TC	TASKOVER
# Page 519
; ============================================================================
; AVERAGE G ON MODE - CONTINUOUS ORBITAL MONITORING
;
; When "Average G" was active, the spacecraft was under powered flight,
; continuously accelerating. The state vector changed rapidly. VERB 82 in
; this mode recalculated orbital parameters approximately once per second,
; giving the crew real-time trajectory information during critical burns.
;
; During translunar injection (TLI), lunar orbit insertion (LOI), or
; transearth injection (TEI), the crew monitored these displays to verify
; the engine burn was proceeding correctly. If apogee or perigee heights
; deviated from the flight plan, Mission Control could decide to extend
; or terminate the burn early.
;
; The V82GON routine spawned a lower-priority job (V82GON1) that performed
; orbital calculations once per second. Meanwhile, the main routine displayed
; results via V16N44 (apogee height, perigee height, time to fall).
; ============================================================================

V82GON		EXIT			# AVERAGE G ON. USE CURRENT STATE VECTOR
					# FOR ORBITAL PARAMETER CALCULATIONS.
		CAF	PRIO7		# LESS THAN LAMBERT
		TC	FINDVAC		# V82GON1 WILL PERFORM ORBIT CALCULATIONS
		EBANK=	TFF		# ABOUT PROPER BODY APPROX ONCE PER SEC.
		2CADR	V82GON1

; Wait for V82GON1 to complete first orbital calculation before displaying
; results. This ensured the crew saw valid data, not uninitialized values.

		RELINT
		CCS	NEWJOB		# WITHOLD V16 N44 UNTIL FIRST ORBIT CALC
		TC	CHANG1		# IS DONE. NOTE: V82GON1 (PRIO7, FINDVAC
					# JOB) IS COMPLETED BEFORE V82GON (PRIO7,
					# NOVAC JOB).

; Display loop: show V16N44 (apogee height, perigee height, time-to-fall)
; continuously, updating with each new calculation from V82GON1. The crew
; could monitor these three values changing in real-time during burns.

V82REDSP	CAF	V16N44		# MONITOR HAPO, HPER, TFF
		TC	BANKCALL
		CADR	GOXDSPF
		TC	B5OFF		# TERM THIS TELLS V82GON1 TO KILL ITSELF.
		TC	B5OFF		# PROC DITTO.
		TC	V82REDSP	# RECYCLE

; ============================================================================
; V82GON1 - PERPETUAL ORBITAL CALCULATION TASK
;
; This job ran continuously during Average G ON mode, recalculating orbital
; parameters once per second as the state vector changed during powered flight.
; The routine perpetuated itself by waiting one second then re-invoking,
; continuing until the crew terminated VERB 82 (bit 5 of EXTVBACT cleared).
;
; State vector atomicity was critical: RN (position) and VN (velocity) had
; to be from the same navigation update. The VLOAD/GOTO sequence ensured
; both were fetched together without interruption from state vector updates.
; ============================================================================

V82GON1		TC	INTPRET		# THIS EXEC PROGRAM PERPETUATES ITSELF
					# ONCE A SEC UNTIL BIT 5 OF EXTVBACT =0.
		VLOAD	GOTO		# HOLDS OFF CCS NEWJOB BETWEEN RN AND
			RN		# VN FETCH SO RN , VN ARE FROM SAME
			NEXTLINE	# STATE VECTOR UPDATE.

; Store position and velocity vectors for orbital calculation. Scaling
; depends on gravitational body: Earth uses (-29)M and (-7)M/CS, Moon uses
; (-27)M and (-5)M/CS due to different sphere of influence sizes.

NEXTLINE	STOVL	RONE		# RN AT (-29)M FOR EARTH OR MOON
			VN
		STORE	VONE		# VN AT (-7)M/CS FOR EARTH OR MOON
; Determine gravitational body for orbital calculations. Different constants
; apply for Earth-centered vs Moon-centered orbits due to different mass,
; radius, and sphere of influence.

		BON	GOTO
			AMOONFLG	# FLAG INDICATES BODY ABOUT WHICH ORBITAL
			MOONGON		# CALCULATIONS ARE TO BE PERFORMED.
			EARTHGON	# IF SET - MOON , IF RESET - EARTH.

; Load Moon-specific parameters for SR30.1 orbital calculations:
; - Gravitational parameter (μ) for lunar sphere of influence
; - Minimum perigee threshold for time-to-fall calculation
; - Landing site radius (surface altitude reference)

MOONGON		SET	DLOAD
			V82EMFLG	# INDICATE MOON SCALING FOR SR30.1
			1/RTMUM		# LUNAR PARAMETERS LOADED HERE FOR SR30.1
		STODL	TFF/RTMU
			MINPERM		; Lunar minimum perigee threshold
		STOVL	HPERMIN
			RLS		# SCALED AT (-27)M
		ABVAL	GOTO		; Convert landing site position to radius
			V82GON2

; Load Earth-specific parameters for SR30.1 orbital calculations:
; - Gravitational parameter (μ) for Earth sphere of influence
; - Minimum perigee threshold for atmospheric entry calculations
; - Launch pad radius (surface altitude reference)

EARTHGON	CLEAR	DLOAD
			V82EMFLG	# INDICATE EARTH SCALING FOR SR30.1
			1/RTMUE		# EARTH PARAMETERS LOADED HERE FOR SR30.1
		STODL	TFF/RTMU
			MINPERE		; Earth minimum perigee threshold
		STODL	HPERMIN
			RPAD		; Launch pad radius
V82GON2		STCALL	RPADTEM		# COMMON CODE FOR EARTH & MOON.
			SR30.1		; Calculate orbital parameters
# Page 520
; SR30.1 has computed orbital parameters (RAPO, RPER, HAPO, HPER, TPER, TFF).
; Now check if we're in Mode 11 (P11 Earth Orbit Entry Monitor) which requires
; additional splash point calculation to display landing site prediction.

		EXIT
		TC	CHECKMM
		DEC	11
		TC	V82GON3		# NOT IN MODE 11.

; Mode 11 or Mode 00: Calculate splash point (landing site) prediction.
; During Earth orbit entry monitoring (P11), the crew wanted to know where
; they would land if they continued the current trajectory. DELRSPL computed
; the distance from the target landing site to the predicted splash point.

CANDEL		TC	INTPRET		# IN MODE 11 OR 00
		CALL
			INTSTALL	# DELRSPL DOES INTWAKE
		DLOAD	CALL
			TFF		; Time to fall (entry interface time)
			DELRSPL		# RETURN IS TO NEXT LINE ( SPLRET ).

; After splash calculation, check if mode is still active (not terminated).
; If mode 00 (no program running), start countdown timer display.

SPLRET		EXIT

		CA	MODREG
		EXTEND
		BZF	SPLRET1		; Mode 00: initialize timer display

; Check termination flag. If crew pressed VERB TERMINATE, exit the loop.
; Otherwise, wait one second and repeat the calculation cycle.

V82GON3		CAF	BIT5
		MASK	EXTVBACT	# SEE IF ASTRONAUT HAS SIGNALLED TERMINATE
		EXTEND
		BZF	ENDEXT		# YES, TERMINATE VB 82 LOOP

; One-second delay before next calculation cycle. This rate was chosen as
; a compromise: fast enough to show trajectory changes during burns, slow
; enough not to overload the computer with calculation workload.

		CAF	1SEC
		TC	BANKCALL	# WAIT ONE SECOND BEFORE REPEATING
		CADR	DELAYJOB	# ORBITAL PARAMETER COMPUTATION.
		TC	V82GON1		; Perpetuate the calculation loop

# Page 521
# SUBROUTINE NAME: SR30.1
# MOD NO: 0								DATE: 		16 FEB 67
# MOD BY: R. R. BAIRNSFATHER						LOG SECTION:	R32
# MOD NO: 1	MOD BY: R. R. BAIRNSFATHER	DATE: 11 APR 67		SR30.1 CHANGED TO ALLOW MONITOR OPERN
# MOD NO: 2	MOD BY: R. R. BAIRNSFATHER	DATE: 14 APR 67		ADD OVFL CK FOR RAPO
# MOD NO: 3	MOD BY ALONSO			DATE: 11 DEC 67		SUBROUTINE REWRITTEN
# MOD NO: 4	MOD BY ALONSO			DATE: 26 MAR 68		PROG MOD TO HANDLE DIF EARTH/MOON SCALE
# MOD NO: 5	MOD BY: RR BAIRNSFATHER		DATE: 6 AUG 68		OVFL CK FOR HAPO & HPER. VOIDS MOD #2.
#
# NEW FUNCTIONAL DESCRIPTION:  ORBITAL PARAMETERS DISPLAY FOR NOUNS 32 AND 44.
# SR30.1 CALLS TFFCONMU AND TFFRP/RA TO CALCULATE RPER (PERIGEE RADIUS),
# RAPO (APOGEE RADIUS), HPER (PERIGEE HEIGHT ABOVE LAUNCH PAD OR LUNAR
# LANDING SITE), HAPO (APOGEE HEIGHT AS ABOVE), TPER (TIME TO PERIGEE),
# TFF (TIME TO INTERSECT 300 KFT ABOVE PAD OR 35KFT ABOVE LANDING SITE).
# IF HPER IS GREATER THAN OR EQUAL TO HPERMIN, CALCULATES TPER AND STORES
# NEGATIVE   IN -TPER.  OTHERWISE STORES +0 IN -TPER.  WHENEVER TPER IS
# CALCULATED, TFF IS NOT COMPUTABLE AND DEFAULTS TO -59MIN 59SEC. IF HAPO
# WOULD EXCEED 9999.9 NM, IT IS LIMITED TO THAT VALUE FOR DISPLAY.
#
# ADDENDUM:	HAPO AND HPER SHOULD BE CHANGED TO READ HAPOX AND HPERX IN THE
#		ABOVE REMARKS.
#
# CALLING SEQUENCE:	CALL
#				SR30.1
#
# SUBROUTINES CALLED:	TFFCONMU, TFFRP/RA, CALCTPER, CALCTFF
# NORMAL EXIT MODE:	CALLING LINE +1 (STILL IN INTERPRETIVE MODE)
# ALARMS:	NONE
# OUTPUT:       RAPO	(-29) M EARTH	APOGEE RADIUS	EARTH CENTERED COORD.
#			(-27) M MOON			MOON CENTERED COORD.
#		RPER	(-29) M EARTH	PERIGEE RADIUS	EARTH CENTERED COORD.
#			(-27) M MOON			MOON CENTERED COORD.
#		HAPOX	(-29) M		APOGEE ALTITUDE ABOVE PAD OR LAND. SITE MAX VALUE LIMITED TO 9999.9 NM.
#		HPERX	(-29) M		PERIGEE ALT. ABOVE PAD OR LAND. SITE    MAX VALUE LIMITED TO 9999.9 NM.
#		TFF	(-28) CS	TIME TO 300KFT OR 35KFT ALTITUDE
#		-TPER	(-28) CS	TIME TO PERIGEE
# ERASABLE INITIALIZATION REQUIRED -
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
# DEBRIS:	QPRET, PDL, S2

# Page 522
		COUNT*	$$/SR30S

; ============================================================================
; SUBROUTINE: SR30.1 - Orbital Parameter Calculation
;
; This is the computational heart of the R30 (V82) orbital parameter display.
; Given a state vector (position and velocity), SR30.1 computes the Keplerian
; orbital elements and times of interest to the crew:
;
;   - RAPO, RPER: Apogee and perigee radii (highest and lowest orbital points)
;   - HAPOX, HPERX: Apogee and perigee altitudes above reference surface
;   - TPER: Time until perigee passage
;   - TFF: Time to fall through atmosphere (300 kft Earth, 35 kft Moon)
;
; The subroutine handles both Earth-centered and Moon-centered orbits, which
; require different scaling due to the different masses and radii of the two
; bodies. The V82EMFLG flag indicates which body's parameters to use.
;
; This calculation was critical during Apollo 11 mission phases when the crew
; needed to monitor their orbital trajectory, especially during trans-lunar
; coast, lunar orbit, and trans-earth coast.
; ============================================================================

SR30.1		SETPD	STQ		# INITIALIZE PUSHDOWN LIST.
			0
			S2		; Save return address
					# SR30.1 INPUT:	RONE AT (-29)M EARTH/MOON
					#		VONE AT (-7)M/CS
					# TFFCONMU,TFFRP/RA,CALCTPER,AND CALCTFF
					# CALLS REQUIRE:
					# EARTH CENTERED (NO RESCALING REQUIRED)
					#	RONE SCALED TO B-29 M
					#	VONE SCALED TO B-7 M/CS
					# MOON CENTERED (RESCALING REQUIRED)
					#	RONE SCALED TO B-27 M
					#	VONE SCALED TO B-5 M/CS

; Moon-centered orbits use different scaling than Earth-centered orbits due to
; the Moon's smaller size and mass. The called subroutines (TFFCONMU, TFFRP/RA)
; expect Moon vectors scaled at B-27 for position and B-5 for velocity.
; If V82EMFLG is set (Moon mode), multiply both vectors by 4 (shift left 2).

		BOFF	VLOAD
			V82EMFLG	# OFF FOR EARTH , ON FOR MOON.
			TFFCALLS	; Earth: skip rescaling
			RONE		; Moon: load position vector
		VSL2			; Multiply by 4: (-29)M → (-27)M
		STOVL	RONE		; Store rescaled position
			VONE		; Load velocity vector
		VSL2			; Multiply by 4: (-7)M/CS → (-5)M/CS
		STORE	VONE		; Store rescaled velocity
; Call TFFCONMU to compute conic constants (semi-major axis, eccentricity)
; from the state vector. These constants describe the orbital ellipse geometry.
; Then call TFFRP/RA to compute the actual apogee and perigee radii.

TFFCALLS	CALL
			TFFCONMU	; Compute μ, semi-major axis, eccentricity
		CALL			# TFFRP/RA COMPUTES RAPO,RPER.
			TFFRP/RA	; Compute apogee and perigee radii
					# RETURNS WITH RAPO IN D(MPAC).

; Compute apogee altitude above reference surface (launch pad or landing site).
; RAPO is the distance from the body's center; subtract the reference radius
; to get altitude above the surface where the crew launched or will land.

		DSU
			RPADTEM		; HAPO = RAPO - RPADTEM (altitude above pad)

; Rescale Moon-centered altitude for display. Moon results are at (-27)M but
; the display routine expects (-29)M. Divide by 4 (shift right 2) to convert.
; Earth-centered results are already at (-29)M and need no rescaling.

		BOFF	SR2R		# NEED HAPO AT (-29)M FOR DISPLAY.
					# IF MOON CENTERED, RESCALE FROM (-27)M.
					# IF EARTH CENTERED ALREADY AT (-29)M.
			V82EMFLG        # OFF FOR EARTH , ON FOR MOON.
			+1		; Earth: skip rescaling
					; Moon: divide by 4 to convert (-27)M → (-29)M

; Overflow protection: DSKY can only display up to 9999.9 nautical miles.
; MAXCHK limits HAPO to this maximum to prevent display overflow/wrapping.

		CALL			# IF HAPO > MAXNM, SET HAPO =9999.9 NM.
			MAXCHK		# OTHERWISE STORE (RAPO-RPADTEM) IN HAPO.
STORHAPO	STODL	HAPOX		; Store apogee altitude for display
			RPER		; Load perigee radius
; Compute perigee altitude above reference surface. This is the lowest point
; in the orbit, critical for crew safety monitoring. If perigee drops too low,
; atmospheric drag (Earth) or surface collision (Moon) becomes a concern.

		DSU
			RPADTEM		# GIVES HPER AT (-29)M EARTH, (-27)M MOON.
		STORE	MPAC +4		# SAVE THIS FOR COMPARISON TO HPERMIN.

; Rescale Moon-centered perigee altitude for display, same as for apogee.
; Convert from (-27)M to (-29)M by dividing by 4 (shift right 2).

		BOFF	SR2R		# NEED HPER AT (-29)M FOR DISPLAY.
					# IF MOON CENTERED, RESCALE FROM (-27)M.
					# IF EARTH CENTERED ALREADY AT (-29)M.
			V82EMFLG	# OFF FOR EARTH, ON FOR MOON.
			+1		; Earth: skip rescaling
					; Moon: divide by 4 to convert (-27)M → (-29)M

; Overflow protection: limit HPER to 9999.9 NM for DSKY display compatibility.
; During Apollo 11 lunar orbit, typical perigee altitudes were 50-60 NM, well
; below this limit. This check was more relevant during trans-lunar coast with
; very high perigee altitudes.

		CALL			# IF HPER > MAXNM, SET HPER = 9999.9 NM.
			MAXCHK
# Page 523

; Store perigee altitude and check if orbit dips below minimum safe threshold.
; If perigee is too low, the spacecraft is on a suborbital trajectory that
; will intersect the surface or atmosphere, so time-to-perigee is meaningless
; (set to zero). Otherwise, calculate when the spacecraft reaches perigee.

STORHPER	STODL	HPERX		# STORE (RPER - RPADTEM) INTO HPERX.
			MPAC +4		; Reload raw HPER for threshold check
		DSU	BPL		# HPERMIN AT (-29)M FOR EARTH, (-27)M MOON
			HPERMIN		# IF HPER L/ HPERMIN (300 OR 35) KFT,
			DOTPER		# THEN ZERO INTO -TPER.

; Perigee below minimum threshold: spacecraft will impact or enter atmosphere
; before reaching true perigee. Set time-to-perigee to zero (displayed as
; -00H00M00S) to indicate trajectory is suborbital/impact.

		DLOAD	GOTO		# OTHERWISE CALCULATE TPER.
			HI6ZEROS	; Load zero
			SKIPTPER	; Skip TPER calculation

; Perigee above minimum threshold: calculate time from current position to
; perigee passage using Kepler's equation and orbital mechanics. CALCTPER
; computes the time based on the elliptical orbit geometry and current
; position along the orbit.

DOTPER		DLOAD	CALL
			RPER		; Pass perigee radius to CALCTPER
			CALCTPER	; Compute time to perigee passage
		DCOMP			# TPER IS PUT NEG INTO -TPER.

; Store negative time to perigee (-TPER) for display. The DSKY increments
; this negative value once per second, counting down to perigee passage.
; After TPER storage, calculate time-to-fall (TFF) for atmospheric entry.

SKIPTPER	STODL	-TPER		; Store (negative) time to perigee
			HPERMIN		# HPERMIN AT (-29)M FOR EARTH, (-27)M MOON

; Compute time to fall below critical altitude threshold. For Earth, this is
; 300 KFT (entry interface altitude). For Moon, this is 35 KFT (landing radar
; acquisition altitude). This tells crew how long until atmospheric entry or
; landing sequence must begin.

		DAD	CALL
			RPADTEM		# RPADTEM AT (-29)M FOR EARTH, (-27)M MOON
			CALCTFF		# GIVES 59M59S FOR TFF IF RPER G/
		DCOMP			# HPERMIN + RPADTEM.  (TPER WAS NON ZERO)
		STCALL	TFF		# OTHERWISE COMPUTES TFF.	(GOTO)
			S2		; Return to caller

; ============================================================================
; MAXCHK - Maximum Value Check for DSKY Display
;
; Limits orbital parameter values to maximum displayable range on the DSKY.
; The DSKY can only show up to 9999.9 NM in its five-digit display format.
; Values exceeding this are clamped to 9999.9 to prevent overflow/wrapping.
;
; Used by V82 (this program) and P30-P37 (rendezvous programs) to ensure
; altitude and range displays remain readable during high-altitude coast
; phases or distant target tracking.
;
; Input:  MPAC contains value to check (scaled at -29 meters)
; Output: MPAC contains min(input, 9999.9 NM), limited to displayable range
; ============================================================================

MAXCHK		DSU	BPL		# IF C(MPAC) > 9999.9 NM. MPAC = 9999.9 NM.
			MAXNM		; Subtract maximum displayable value
			+3		# OTHERWISE C(MPAC) = B(MPAC).

; Value exceeds maximum: restore to 9999.9 NM by adding MAXNM back.
; This caps the display at the maximum readable value.

		DAD	RVQ
			MAXNM		; Restore to maximum displayable

; Value within range: load and return the maximum value (effectively no change
; since we're at the branch target after the DAD).

 +3		DLOAD	RVQ		# (USED BY P30 - P37 ALSO)
 			MAXNM		; Return with value capped at maximum

; MAXNM constant: 9999.9 nautical miles in scaled format (-29 meters)
; This represents the maximum value displayable on the DSKY in five digits
; with one decimal place (XXXXX.X format). Octal value 0106505603 at B-29
; scaling equals approximately 18,519,926 meters = 9999.9 NM.

MAXNM		2OCT	0106505603

# Page 524

# There is no source code on this page --- HG 2009
