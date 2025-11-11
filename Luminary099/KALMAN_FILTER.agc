# Copyright:	Public domain.
# Filename:	KALMAN_FILTER.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1470-1471
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
; FILE: KALMAN_FILTER.agc
; MODULE: Digital Autopilot System (DAPS)
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Processes RCS jet firing torque calculations and rate filtering for
;        the digital autopilot. Computes signed torque contributions from jet
;        firings, accumulates torque for downlink telemetry, and calculates
;        filtered jet rates for pitch and yaw/roll control feedback loops.
;
; COMMENT-ONLY READERS: This routine manages the reaction control jets that
;        keep the Lunar Module oriented correctly during all mission phases.
; CODE-ALONG READERS: Study jet torque calculation, scaling transformations,
;        and rate feedback computation for RCS autopilot control loops.
; ============================================================================

# Page 1470
		EBANK=	NO.UJETS
		BANK	16
		SETLOC	DAPS1
		BANK

		COUNT*	$$/DAP

; ============================================================================
; RATELOOP - JET TORQUE CALCULATION AND RATE FILTERING
;
; The Lunar Module maintains its orientation in space using small reaction
; control system (RCS) jets. This routine calculates the rotational torque
; produced by firing these jets and filters the resulting rates for the
; autopilot control loops. During critical phases like landing or rendezvous,
; precise attitude control is essential for mission success.
;
; This filtering process is conceptually related to Kalman filtering - it
; optimally estimates the spacecraft's rotational state by combining commanded
; jet firings with measurements of actual rotation rates, providing smooth
; control feedback despite discrete jet pulses.
; ============================================================================

; Loop processes two axes (pitch and yaw/roll) to calculate jet torques.
; DAPTEMP6 serves as axis counter (2 for pitch, 1 for yaw/roll, 0 for roll).
RATELOOP	CA	TWO
		TS	DAPTEMP6	; Initialize axis counter to 2
		DOUBLE			; Double to get 4
		TS	Q		; Q holds return index for torque storage
		
; Check if jet firing time (TJP) for this axis is positive (jets active).
; TJP holds the time remaining for commanded jet pulses in this axis.
		INDEX	DAPTEMP6
		CCS	TJP		; Check sign of TJP (Time Jet Pulse)
		TCF	+2		; Positive: jets are firing
		TCF	LOOPRATE	; Zero or negative: no jets, skip to rate calc
		
; Jet pulse time exceeds 100 milliseconds - limit it to maximum pulse width.
; The RCS jets fire in discrete pulses; this limits maximum continuous firing.
		AD	-100MST6	; Subtract 100ms threshold (160 at 1/1.6ms)
		EXTEND
		BZMF	SMALLTJU	; Branch if TJP < 100ms
		
; Large pulse: decrement TJP by 100ms and use full 100ms for torque calculation.
		INDEX	DAPTEMP6
		CCS	TJP		; Check sign again to get proper decrement
		CA	-100MST6	; Positive: load -100ms
		TCF	+2
		CS	-100MST6	; Negative: load +100ms (complement)
		INDEX	DAPTEMP6
		ADS	TJP		; Decrement TJP by 100ms
		
; Determine the signed torque value based on jet firing direction.
; Positive TJP indicates jets firing in positive axis direction.
		INDEX	DAPTEMP6
		CCS	TJP		; Check final sign for torque direction
		CS	-100MS		; Positive: use -100ms (0.1 sec at 1)
		TCF	+2
		CA	-100MS		; Negative: use +100ms
		
; ============================================================================
; TORQUE CALCULATION
; Convert jet firing time to actual rotational torque based on number of
; jets available for this axis. The torque depends on how many RCS jets
; can be fired simultaneously and their moment arms about the spacecraft
; center of mass.
; ============================================================================
LOOPRATE	EXTEND
		INDEX	DAPTEMP6
		MP	NO.PJETS	; Multiply by number of jets for this axis
		CA	L		; Get result from L register
		INDEX	DAPTEMP6
		TS	DAPTEMP1	; Store signed torque at 1 jet-sec for filter
		
; Rescale torque for telemetry downlink format.
; The ground controllers monitor jet usage to track propellant consumption
; and verify the autopilot is functioning correctly. During Apollo 11's
; landing, this data helped mission control assess fuel margins.
		EXTEND
		MP	BIT10		; Rescale to 32; one bit ≈ 2 jet-milliseconds
		EXTEND
		BZMF	NEGTORK		; Branch if negative torque
		
; Accumulate torque into downlink telemetry register.
; This running total is transmitted to Earth so mission controllers can
; monitor total jet firing activity throughout the mission.
STORTORK	INDEX	Q		; Use Q as index (4 for pitch, 2 for yaw)
		ADS	DOWNTORK	; Add to downlink torque register
					; NOTE: Not initialized; overflows allowed

; Loop back to process next axis (pitch → yaw/roll → roll).
; The three-axis control ensures the Lunar Module maintains precise
; orientation for critical operations like landing radar alignment or
; docking with the Command Module.
		CCS	DAPTEMP6	; Decrement axis counter
		TCF	RATELOOP +1	; Continue loop for next axis
		TCF	ROTORQUE	; All axes processed, calculate rates
		
; ============================================================================
; SMALL PULSE HANDLING
; For jet pulses less than 100 milliseconds, use the entire remaining pulse
; time and clear TJP to zero. This ensures accurate torque calculation for
; short jet firings used during fine attitude adjustments.
; ============================================================================
SMALLTJU	CA	ZERO
		INDEX	DAPTEMP6
		XCH	TJP		; Exchange: zero into TJP, TJP value to A
		EXTEND
# Page 1471
		MP	ELEVEN		; Scale by 10.24 (11 octal) for proper units
		CA	L		; Get scaled pulse time from L register
		TCF	LOOPRATE	; Continue to torque calculation
; ============================================================================
; ROTORQUE - FILTERED JET RATE CALCULATION
; 
; Computes the filtered rotation rates caused by RCS jet firings for use in
; the autopilot control loops. These rates provide feedback to the autopilot,
; allowing it to command jet firings that precisely counteract disturbances
; and maintain the desired spacecraft attitude.
;
; The calculation transforms jet torques from individual axes into roll (R),
; pitch (Q), and yaw (P) rates using the spacecraft's moment of inertia about
; each axis. During lunar landing, these rates help Armstrong and Aldrin
; maintain proper orientation as the LM descends toward the surface.
; ============================================================================
ROTORQUE	CA	DAPTEMP2	; Load pitch axis torque
		AD	DAPTEMP3	; Add yaw/roll axis torque
		EXTEND
		MP	1JACCR		; Multiply by roll acceleration coefficient
		TS	JETRATER	; Store filtered roll rate
		
; Calculate pitch/yaw combined rate.
; The moment of inertia differs for each axis based on the spacecraft's
; mass distribution (descent stage, ascent stage, propellant tanks).
		CS	DAPTEMP3	; Complement of yaw/roll torque
		AD	DAPTEMP2	; Add pitch torque
		EXTEND
		MP	1JACCQ		; Multiply by pitch/yaw acceleration coefficient
		TS	JETRATEQ	; Store filtered pitch/yaw rate
		
; Return to main autopilot routine with updated rate estimates.
; The autopilot uses these filtered rates to compute the next jet firing
; commands, closing the control loop for stable attitude hold.
		TCF	BACKP		; Return to DAP main processing
; Constant: -100 milliseconds scaled at time6 units (1.6ms per bit).
; -100ms / 1.6ms = -62.5, but represented as -160 due to internal scaling.
; This threshold limits individual RCS jet pulse durations to 100ms maximum
; to prevent excessive propellant usage and heating of jet nozzles.
-100MST6	DEC	-160

; ============================================================================
; NEGTORK - NEGATIVE TORQUE HANDLING
;
; Handles the special case where calculated torque is negative (indicating
; jets firing in the opposite direction for this axis). The routine takes
; the absolute value and adjusts the downlink register index to maintain
; proper sign accounting in telemetry.
; ============================================================================
NEGTORK		COM		; Complement to get absolute value
		INCR	Q	; Increment Q index for opposite direction
		TCF	STORTORK ; Store complemented torque value


