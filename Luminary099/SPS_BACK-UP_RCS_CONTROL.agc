# Copyright:	Public domain.
# Filename:	SPS_BACK-UP_RCS_CONTROL.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1507-1510
# Mod history:	2009-05-27 RSB	Adapted from the corresponding
#				Luminary131 file, using page
#				images from Luminary 1A.
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
; FILE: SPS_BACK-UP_RCS_CONTROL.agc
; MODULE: Digital Autopilot (DAPS)
; MISSION PHASE: Not used in LM operations (CM functionality for code commonality)
;
; TL;DR: Implements backup RCS (Reaction Control System) jet control during 
;        SPS (Service Propulsion System) engine burns to maintain spacecraft 
;        attitude. This is Command Module functionality included in Lunar Module
;        code for software commonality but not operationally used during lunar
;        missions since the LM does not have an SPS engine.
;
; COMMENT-ONLY READERS: This code would control small attitude-control thrusters
;        during main engine burns on the Command Module. Skip this file as it
;        wasn't used during Eagle's lunar landing mission.
; CODE-ALONG READERS: Study phase plane control logic with rate deadbands,
;        coast zones, and inhibition counters. Note CM/LM code-sharing strategy.
; ============================================================================

# Page 1507
# PROGRAM NAME:		SPSRCS
# AUTHOR:		EDGAR M. OSHIKA (AC ELECTRONICS)
# MODIFIED:		TO RETURN TO ALL AXES VIA Q BY P. S. WEISSMAN, OCT 7, 1968
# MODIFIED TO IMPROVE BENDING STABILITY BY G. KALAN, FEB. 14, 1969
#
# FUNCTIONAL DESCRIPTION:
#	THE PROGRAM CONTROLS THE FIRING OF ALL RCS JETS IN THE DOCKED CONFIGURATION ACCORDING TO THE FOLLOWING PHASE
#	PLANE LOGIC.
#
#	1. JET SENSE TEST (SPSRCS)
#		IF JETS ARE FIRING NEGATIVELY, SET OLDSENSE NEGATIVE AND CONTINUE
#		IF JETS ARE FIRING POSITIVELY, SET OLDSENSE POSITIVE AND CONTINUE
#		IF JETS ARE NOT FIRING, SET OLDSENSE TO ZERO AND GO TO OUTER RATE LIMIT TEST
#
# 	2. RATE DEAD BAND TEST
#		IF JETS ARE FIRING NEGATIVELY AND RATE IS GREATER THAN TARGET RATE, LEAVE
#			JETS ON AND GO TO INHIBITION LOGIC.  OTHERWISE, CONTINUE.
#		IF JETS ARE FIRING POSITIVELY AND RATE IS   LESS  THAN TARGET RATE, LEAVE
#			JETS ON AND GO TO INHIBITION LOGIC.  OTHERWISE, CONTINUE.
#
#	3. OUTER RATE LIMIT TEST (SPSSTART)
#		IF MAGNITUDE OF EDOT IS GREATER THAN 1.73 DEG/SEC SET JET FIRING TIME
#			TO REDUCE RATE AND GO TO INHIBITION LOGIC.  OTHERWISE, CONTINUE.
#
#	4. COAST ZONE TEST
#		IF STATE (E,EDOT) IS BELOW LINE E + 4 X EDOT > -1.4 DEG AND EDOT IS LESS THAN 1.30 DEG/SEC SET JET TIME
#		 	POSITIVE AND CONTINUE.  OTHERWISE, SET JET FIRING TIME TO ZERO AND CONTINUE.
#		IF STATE IS ABOVE LINE E + 4 X EDOT > +1.4 DEG AND EDOT IS GREATER THAN -1.30 DEG/SEC, SET JET TIME NEGATIVE
#		 	AND CONTINUE.  OTHERWISE, SET JET FIRING TIME TO ZERO AND CONTINUE.
#
#	5. INHIBITION LOGIC
#		IF OLDSENSE IS NON-ZERO:
#			A) RETURN IF JET TIME AS THE SAME SIGN AS OLDSENSE
#			B) SET INHIBITION COUNTER* AND RETURN IF JET TIME IS ZERO
#			C) SET INHIBITION COUNTER,* SET JET TIME TO ZERO AND RETURN IF SIGN
#			   OF JET TIME IS OPPOSITE TO THAT OF OLDSENSE
#		IF OLDSENSE IS ZERO:
#			A) RETURN IF INHIBITION COUNTER IS NOT POSITIVE
#			B) SET JET TIME TO ZERO AND RETURN IF INHIBITION COUNTER IS POSITIVE
#		*NOTE: INHIBITION COUNTERS CAN BE SET TO 4 OR 10 FOR THE P AND UV AXES,
#		RESPECTIVELY, IN SPSRCS.  THEY ARE DECREMENTED BY ONE AT THE BEGINNING OF
# Page 1508
#		EACH DAP PASS.
#
#	THE MINIMUM PULSE WIDTH OF THIS CONTROLLER IS DETERMINED BY THE REPETITION RATE AT WHICH THIS ROUTINE IS CALLED
#	AND IS NOMINALLY 100 MS FOR ALL AXES IN DRIFTING FLIGHT.  DURING POWERED FLIGHT THE MINIMUM IS 100 MS FOR THE
#	P AXIS AND 200 MS FOR THE CONTROL OF THE U AND V AXES.
#
# CALLING SEQUENCE:
#		INHINT
#		TC	IBNKCALL
#		CADR	SPSRCE
#
# EXIT:
#		TC	Q
#
# ALARM/ABORT MODE:	NONE
#
# SUBROUTINES CALLED:	NONE
#
# INPUT:		E, EDOT
#			TJP, TJV, TJU		TJ MUST NOT BE NEGATIVE ZERO
#
# OUTPUT:		TJP, TJV, TJU

; ============================================================================
; TRANSITION: SPS Backup RCS Control Logic Implementation
;
; This section implements a sophisticated phase plane controller for RCS jets
; during SPS burns. Although this code resides in the Lunar Module software,
; it controls Command Module behavior and would not execute during lunar 
; operations. The controller prevents attitude disturbances during main engine
; burns by firing RCS jets with carefully designed deadbands and inhibition
; logic to avoid excessive jet cycling.
; ============================================================================

		BANK	21
		SETLOC	DAPS4
		BANK

		COUNT*	$$/DAPBU

		EBANK=	TJU

; Rate limit constant for coast zone boundary test (1.125 degrees per second).
; Used to define the inner boundary of the control deadband.
RATELIM2	OCT	00632		# 1.125 DEG/SEC

; Positive thrust command entry point. Sets jet firing time to +0.5 (HALF)
; to command RCS jets to fire in the positive direction for attitude correction.
POSTHRST	CA	HALF

		NDX	AXISCTR
		TS	TJU

; Inhibition logic prevents rapid jet reversals that could destabilize the
; spacecraft. OLDSENSE tracks the previous jet firing direction (positive,
; negative, or zero). This logic checks if the new command reverses direction.
		CCS	OLDSENSE
		TCF	POSCHECK	# JETS FIRING POSITIVELY
		TCF	CTRCHECK	# JETS OFF.  CHECK INHIBITION CTR

; Check if jets firing negatively should continue or be inhibited.
; If commanded jet time has opposite sign from current firing, set inhibition
; counter to prevent immediate reversal.
NEGCHECK	INDEX	AXISCTR		# JETS FIRING NEGATIVELY
		CS	TJU
		CCS	A
		TC	Q		# RETURN
		TCF	+2
		TCF	+1		# JETS COMMANDED OFF.  SET CTR AND RETURN

; Set inhibition counter when jet firing reversal is commanded.
; Counter prevents jets from reversing direction immediately, which would
; waste propellant and potentially excite structural bending modes.
SETCTR		INDEX	AXISCTR		# JET FIRING REVERSAL COMMANDED.  SET CTR,
		CA	UTIME		# SET JET TIME TO ZERO, AND RETURN
# Page 1509
		INDEX	AXISCTR
		TS	UJETCTR
		
; Zero the jet time command. Used when inhibition logic prevents firing
; or when jets need to be turned off during deadband transitions.
ZAPTJ		CA	ZERO
		INDEX	AXISCTR
		TS	TJU
		TC	Q

; Check jets firing positively for potential reversal or inhibition.
POSCHECK	INDEX	AXISCTR
		CA	TJU
		TCF	NEGCHECK +2

; Check the jet inhibition counter to see if firings should be prevented.
; Counter is decremented each DAP pass and prevents jet commands when positive.
CTRCHECK	INDEX	AXISCTR		# CHECK JET INHIBITION COUNTER
		CCS	UJETCTR
		TCF	+2
		TC	Q		# CTR IS NOT POSITIVE.  RETURN
		TCF	ZAPTJ		# CTR IS POSITIVE.  INHIBIT FIRINGS
		TC	Q		# CTR IS NOT POSITIVE.  RETURN

; Inhibition counter time constants (octal values).
; These define how many DAP cycles jets remain inhibited after reversal.
		OCT	00004		; P-axis inhibition time
UTIME		OCT	00012		; U-axis inhibition time (10 decimal)
		OCT	00012		; V-axis inhibition time (10 decimal)

OLDSENSE	EQUALS	DAPTREG1	; Previous jet firing sense storage

; ============================================================================
; Jets firing negatively: Set OLDSENSE negative and perform rate deadband test.
; The rate deadband prevents jets from firing when angular rate is close to
; the target rate, reducing propellant consumption and jet cycling.
; ============================================================================
NEGFIRE		CS	ONE		# JETS FIRING NEGATIVELY
		TS	OLDSENSE
		CA	EDOT
		TCF	+4

; Jets firing positively: Set OLDSENSE positive and perform rate deadband test.
PLUSFIRE	CA	ONE
		TS	OLDSENSE
		CS	EDOT		# RATE DEAD BAND TEST

; Rate deadband test checks if current angular rate is within acceptable
; tolerance of target rate. Target rate depends on flight mode (drifting or not).
		LXCH	A		; Save rate in L register
		CS	DAPBOOLS	# IF DRIFTBIT = 1, USE ZERO TARGET RATE
		MASK	DRIFTBIT	# IF DRIFTBIT = 0, USE 0.10 RATE TARGET
		CCS	A		; Check drift mode
		CA	RATEDB1		; Load 0.101 deg/sec target rate for powered flight
		AD	L		; Add to rate
		EXTEND
		BZMF	SPSSTART	; If within deadband, go to outer rate limit test
		TCF	POSTHRST +3	; Outside deadband, keep jets firing

; ============================================================================
; SPSRCS: Main entry point - Jet Sense Test (Phase Plane Logic Phase 1)
; Determines current jet firing direction by examining jet time command (TJU).
; Branches to appropriate control logic based on firing state.
; ============================================================================
SPSRCS		INDEX	AXISCTR		# JET SENSE TEST (axis P, U, or V)
		CCS	TJU		; Check current jet time: + / +0 / -0 / -
		TCF	PLUSFIRE	# JETS FIRING POSITIVELY (TJU > 0)
		TCF	+2		; TJU = +0, jets not firing
		TCF	NEGFIRE		# JETS FIRING NEGATIVELY (TJU < 0)
		TS	OLDSENSE	# JETS OFF (TJU = 0), store zero in OLDSENSE

; ============================================================================
; SPSSTART: Outer Rate Limit Test and Coast Zone Test
; Phase plane logic phases 3-4: Checks if angular rate exceeds safe limits
; (1.73 deg/sec) and determines coast zone boundaries for fuel-efficient control.
; ============================================================================
SPSSTART	CA	EDOT		# OUTER RATE LIMIT TEST (Phase 3)
		EXTEND
		MP	RATELIM1	; Multiply rate by 1.73 deg/sec limit constant
		CCS	A		; Check if |EDOT| > 1.73 deg/sec
		TCF	NEGTHRST	# OUTER RATE LIMIT EXCEEDED (rate too negative)
		TCF	+2		; Within limits, continue to coast test
		TCF	POSTHRST	# OUTER RATE LIMIT EXCEEDED (rate too positive)

; Coast Zone Test (Phase 4): Determines if spacecraft state (attitude error E
; and rate EDOT) lies within acceptable coast zone boundaries. The phase plane
; is divided by lines E + 4*EDOT = ±DKDB (typically ±1.4 degrees).
		CA	EDOT		# COAST ZONE TEST
# Page 1510
		AD	E		; Form E + EDOT
		EXTEND
		MP	DKDB		# PAD LOADED DEADBAND.  FRESHSTART: 1.4 DEG
		EXTEND			; Compute (E + EDOT) * DKDB
		BZF	TJZERO		; If exactly on boundary, zero jet time

; Test which side of phase plane boundaries the state lies on, and whether
; rate is within inner boundary (±1.125 deg/sec from RATELIM2).
		EXTEND
		BZMF	+7		; Branch if state below upper boundary
		CA	EDOT		; State above upper boundary line
		AD	RATELIM2	; Check if EDOT < -1.125 deg/sec
		EXTEND
		BZMF	TJZERO		; Rate too fast negative, don't fire
		
; State above line, rate OK: Command negative jet thrust to reduce error.
NEGTHRST	CS	HALF		; Load -0.5 for negative jet time
		TCF	POSTHRST +1	; Skip to storage

; State below lower boundary line: Check rate limits for positive thrust.
 +7		CS	RATELIM2	; Load -1.125 deg/sec limit
		AD	EDOT		; Check if EDOT > +1.125 deg/sec
		EXTEND
		BZMF	POSTHRST	; Rate OK, command positive thrust

; Jet time set to zero when state is in coast zone or rate limit violated.
TJZERO		CA	ZERO
		TCF	POSTHRST +1	; Skip to storage

; ============================================================================
; Phase plane controller constants assigned from other memory locations.
; These define outer rate limits and deadband parameters for the control law.
; ============================================================================
; RATELIM1: Outer rate limit (1.73 deg/sec) for emergency rate damping.
; Assigned to CALLCODE location (octal 00032) for memory efficiency.
RATELIM1	=	CALLCODE	# = 00032, CORRESPONDING TO 1.73 DEG/SEC

; RATEDB1: Target rate deadband (0.101 deg/sec) used during powered flight
; when DRIFTBIT = 0. Assigned to TBUILDFX location (octal 00045).
RATEDB1		=	TBUILDFX	# = 00045, CORRESPONDS TO 0.101 DEG/SEC

# *** END OF LMDAP  .015 ***


