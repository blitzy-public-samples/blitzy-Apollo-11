# Copyright:	Public domain.
# Filename:	TVCMASSPROP.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	951-955
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
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
#	Assemble revision 055 of AGC program Comanche by NASA
#	2021113-051.  10:28 APR. 1, 1969
#
#	This AGC program shall also be referred to as
#			Colossus 2A

; ============================================================================
; FILE: TVCMASSPROP.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: trans-lunar/lunar-orbit/trans-earth (SPS burns)
;
; TL;DR: Mass properties computation tracking spacecraft center-of-gravity
;        location and inertia tensor as propellant depletes during burns.
;        Updates TVC control gains to account for changing vehicle dynamics,
;        critical for maintaining control authority throughout long Apollo 11
;        maneuvers like TLI and LOI.
;
; COMMENT-ONLY READERS: This program tracked how the spacecraft's balance point
;        changed as fuel burned, adjusting steering to compensate.
; CODE-ALONG READERS: Study center-of-gravity computation algorithms, inertia
;        tensor updates, propellant usage effects on vehicle dynamics.
; ============================================================================

# Page 951
# PROGRAM NAME....MASSPROP
# LOG SECTION....TVCMASSPROP		PROGRAMMER...MELANSON (ENGEL, SCHLUNDT)
#
#
# FUNCTIONAL DESCRIPTION:
#
#	MASSPROP OPERATES IN TWO MODES:(1)IF LEM MASS OR CONFIGURATION ARE UPDATED (MASSPROP DOES NOT TEST
# FOR THIS) THE ENTIRE PROGRAM MUST BE RUN THROUGH, BREAKPOINT VALUES AND DERIVATIVES OF THE OUTPUTS WITH
# RESPECT TO CSM MASS BEING CALCULATED PRIOR TO CALCULATION OF THE OUTPUTS.  (2)OTHERWISE, THE OUTPUTS CAN BE
# CALCULATED USING PREVIOUSLY COMPUTED BREAKPOINT VALUES AND DERIVATIVES.
#
# CALLING SEQUENCES
#
#	IF LEM MASS OR CONFIGURATION HAS BEEN UPDATED, TRANSFER TO MASSPROP, OTHERWISE TRANSFER TO FIXCW.
#		L	TC	BANKCALL OR IBNKCALL
#		L+1	CADR	MASSPROP
#			OR
#		L+1	CADR	FIXCW
#
#		L+2	RETURNS VIA Q
#
# CALLED	IN PARTICULAR BY DONOUN47 (JOB) AND TVCEXECUTIVE (TASK)
#
# JOBS OR TASKS INITIATED - NONE
#
# SUBROUTINES CALLED - NONE
#
# ERASABLE INITIALIZATION REQUIRED
#
#	LEMMASS MUST CONTAIN LEM MASS SCALED AT B+16 KILOGRAMS
#	CSMMASS MUST CONTAIN CSM MASS SCALED AT B+16 KILOGRAMS
#
#	DAPDATR1 MUST BE SET TO INDICATE VEHICLE CONFIGURATION.
#		BITS (15,14,13)  =  ( 0 , 0 , 1 )	LEM OFF
#				    ( 0 , 1 , 0 )	LEM ON (ASCNT,DSCNT)
#				    ( 1 , 1 , 0 )	LEM ON (ASCNT ONLY)
#
#
# ALARMS -	NONE
#
# EXIT -	TC	Q
#
# OUTPUTS:
#
#	(1)IXX, SINGLE PRECISION SCALED AT B+20 IN KG-M SQ.
#	(2)IAVG, SINGLE PRECISION SCALED AT B+20 IN KG-M SQ.
#	(3)IAVG/TLX, SINGLE PRECISION, SCALED AT B+2 SEC-SQD
#	THEY ARE STORED IN CONSECUTIVE REGISTERS IXX0, IXX1, IXX2
#
#	CONVERSION FACTOR :  (SLUG-FTSQ) = 0.737562 (KG-MSQ)
# Page 952
#
# OUTPUTS ARE CALCULATED AS FOLLOWS:
#
#   (1) IF LEM DOCKED, LEMMASS IS FIRST ELIMINATED AS A PARAMETER
#
#	VARST0 = INTVALUE0 + LEMMASS(SLOPEVAL0)		IXX		BREAKPOINT VALUE
#	VARST1 = INTVALUE1 + LEMMASS(SLOPEVAL1)		IAVG		BREAKPOINT VALUE
#	VARST2 = INTVALUE2 + LEMMASS(SLOPEVAL2)		IAVG/TLX	BREAKPOINT VALUE
#
#	VARST3 = INTVALUE3 + LEMMASS(SLOPEVAL3)		IAVG/TLX	SLOPE FOR CSMMASS > 33956 LBS ( SPS > 10000 LBS)
#	VARST4 = INTVALUE4 + LEMMASS(SLOPEVAL4)		IAVG		SLOPE FOR CSMMASS > 33956 LBS ( SPS > 10000 LBS)
#
#	VARST5 = INTVALUE5 + LEMMASS(SLOPEVAL5)		IXX		SLOPE FOR ALL VALUES OF CSMMASS
#
#	VARST6 = INTVALUE6 + LEMMASS(SLOPEVAL6)		IAVG		SLOPE FOR CSMMASS < 33956 LBS ( SPS < 10000 LBS)
#	VARST7 = INTVALUE7 + LEMMASS(SLOPEVAL7)		IAVG/TLX	SLOPE FOR CSMMASS < 33956 LBS ( SPS < 10000 LBS)
#
#	VARST8 = INTVALUE8 + LEMMASS(SLOPEVAL8)		IAVG		DECREMENT TO BRKPT VALUE WHEN LEM DSCNT STAGE OFF
#	VARST9 = INTVALUE9 + LEMMASS(SLOPEVAL9)		IAVG/TLX	DECREMENT TO BRKPT VALUE WHEN LEM DSCNT STAGE OFF
#
#   (2) IF LEM NOT DOCKED
#
#	VARST0 = NOLEMVAL0	WHERE THE MEANING AND SCALING OF VARST0
#		.	.	TO VARST9 ARE THE SAME AS GIVEN ABOVE
#		.	.
#		.	.	NOTE... FOR THIS CASE, VARST8,9 HAVE NO
#	VARST9 = NOLEMVAL9	MEANING (THEY ARE COMPUTED BUT NOT USED)
#
#   (3) THE FINAL OUTPUT CALCULATIONS ARE THEN DONE
#
#	IXX0 = VARST0 + (CSMMASS + NEGBPW)VARST5		IXX
#
#	IXX1 = VARST1 + (CSMMASS + NEGBPW)VARST(4 OR 6)		IAVG
#
#	IXX2 = VARST2 + (CSMMASS + NEGBPW)VARST(3 OR 7)		IAVG/TLX
#
#
# THE DATA USED CAME FROM CSM/LM SPACECRAFT OPERATIONAL DATA BOOK.
#	VOL. 3, NASA DOCUMENT SNA-8-D-027 (MARCH 1968)
#
# PERTINENT MASS DATA :		CSM WEIGHT	(FULL)	64100 LBS.
#						(EMPTY)	23956 LBS.
#				LEM WEIGHT	(FULL)	32000 LBS.
#						(EMPTY)	14116 LBS.
#
# (WEIGHTS ARE FROM AMENDMENT #1 (APRIL 24,1968) TO ABOVE DATA BOOK)
# Page 953

		BANK	25
		SETLOC	DAPMASS
		BANK
		EBANK=	BZERO
		COUNT*	$$/MASP

; ============================================================================
; MASS PROPERTIES COMPUTATION OVERVIEW
;
; As the Service Propulsion System (SPS) engine burns propellant during major
; maneuvers, the spacecraft's mass decreases and its center-of-gravity shifts.
; This routine calculates three critical outputs that the TVC digital autopilot
; needs to maintain precise attitude control:
;
; 1. IXX: Roll axis moment of inertia (how hard to rotate around X axis)
; 2. IAVG: Average pitch/yaw inertia (resistance to rotation in other axes)
; 3. IAVG/TLX: Time constant ratio for control loop responsiveness
;
; The computation uses a piecewise-linear approximation with breakpoints and
; slopes that depend on whether the Lunar Module is docked and whether the
; SPS propellant is above or below 10,000 lbs remaining. This approach allows
; fast updates during time-critical burns without complex real-time integration.
;
; During Apollo 11's Translunar Injection burn, these values changed continuously
; as 20,000+ lbs of propellant depleted, requiring the autopilot to adapt its
; control gains to prevent instability or excessive propellant consumption.
; ============================================================================

MASSPROP	CAF	NINE		# MASSPROP USES TVC/RCS INTERRUPT TEMPS
		TS	PHI333		# SET UP TEN PASSES

; ============================================================================
; BREAKPOINT VALUE CALCULATION PHASE
;
; This section computes ten intermediate "breakpoint" values (VARST0 through
; VARST9) that serve as reference points for the piecewise-linear mass property
; approximations. The computation depends on whether the Lunar Module is docked:
;
; IF LEM DOCKED: Each breakpoint = INTVALUE + (LEMMASS * SLOPEVAL)
;    The LEM's mass contributes to the combined vehicle's inertia properties.
;    The slope values account for how LEM mass distribution affects each axis.
;
; IF LEM NOT DOCKED: Each breakpoint = NOLEMVAL (precomputed constant)
;    The CSM flies alone with its own mass distribution characteristics.
;
; The loop executes ten times (PHI333 counts down from 9 to 0), computing:
; VARST0,1,2: IXX, IAVG, IAVG/TLX breakpoint values
; VARST3,4: Slopes for CSM mass > 33956 lbs (SPS propellant > 10000 lbs)
; VARST5: IXX slope (same for all CSM masses)
; VARST6,7: Slopes for CSM mass < 33956 lbs (SPS propellant < 10000 lbs)
; VARST8,9: Decrements when LM descent stage is jettisoned
; ============================================================================

LEMTEST		CAE	DAPDATR1	# DETERMINE LEM STATUS
		MASK	BIT13		# BIT13=0 means LEM attached
		EXTEND
		BZF	LEMYES		# Branch if LEM docked

LEMNO		INDEX	PHI333		# LEM NOT ATTACHED
		CAF	NOLEMVAL	# Use precomputed values for CSM-only
		TCF	STOINST		# Skip LEM mass calculations

; LEM is docked - compute breakpoint values accounting for LEM mass contribution
LEMYES		CAE	LEMMASS		# LEM IS ATTACHED (scaled B+16 kg)
		DOUBLE			# Scale adjustment for multiplication
		EXTEND
		INDEX	PHI333		# Select appropriate slope coefficient
		MP	SLOPEVAL	# LEMMASS * slope for this parameter
		DDOUBL			# Additional scaling for proper units
		INDEX	PHI333
		AD	INTVALUE	# Add intercept: breakpoint = intercept + LEM_contribution

STOINST		INDEX	PHI333		# STORAGE INST BEGIN HERE
		TS	VARST0		# Store computed breakpoint (VARST0 through VARST9)
		CCS	PHI333		# ARE ALL TEN PASSES COMPLETED
		TCF	MASSPROP +1	# NO - GO DECREMENT PHI333 and compute next

; ============================================================================
; LEM DESCENT STAGE JETTISON ADJUSTMENT
;
; After lunar landing, the LM ascent stage separates from the descent stage,
; leaving the heavy descent engine and landing gear behind. This significantly
; changes the combined vehicle's mass distribution. When BIT15 of DAPDATR1 is
; set (DAPDATR1 negative), it indicates the descent stage has been jettisoned.
;
; This section applies correction factors (VARST8, VARST9) to the breakpoint
; values to account for the ~10,000 lb mass reduction and shift in center-of-
; gravity that occurs when the descent stage is left on the lunar surface.
; For Apollo 11, this adjustment was applied after Eagle's ascent on July 21.
; ============================================================================

DXTEST		CCS	DAPDATR1	# IF NEG, BIT15 IS 1, LEM DSCNT STAGE OFF
		TCF	FIXCW		# Descent stage present - no correction needed
		TCF	FIXCW		# Descent stage present - no correction needed
		DXCH	VARST0 +8D	# Get IAVG and IAVG/TLX decrements (VARST8,9)
		DAS	VARST0 +1	# Apply corrections to breakpoints VARST1,2
		CA	DXITFIX		# Additional IAVG/TLX correction factor
		ADS	VARST0 +7	# Apply to VARST7 (low CSM mass slope)

; ============================================================================
; FINAL OUTPUT CALCULATION PHASE
;
; With breakpoint values established, now compute the three outputs by applying
; piecewise-linear corrections based on current CSM mass. The breakpoint weight
; NEGBPW (33,956 lbs = 15,402 kg) divides the SPS propellant range at 10,000 lbs
; remaining. Above this threshold, one set of slope values applies; below it,
; another set provides better approximation.
;
; The outputs IXX0, IXX1, IXX2 represent:
; - Roll axis inertia (affects roll damping and control authority)
; - Average pitch/yaw inertia (affects pitch/yaw response characteristics)  
; - Inertia/time-constant ratio (tunes control loop gains for stability)
;
; These values directly feed the TVC digital autopilot (TVCDAPS.agc) to adapt
; control gains in real-time as propellant depletes during critical burns.
; ============================================================================

; Initialization for output calculation loop - three passes compute IXX0, IXX1, IXX2
FIXCW		CAF	BIT2		# COMPUTATION PHASE BEGINS HERE.  SET UP
		TS	PHI333		# THREE PASSES (one for each output)
		TS	PSI333		# PSI333 also set to 2 for output indexing

; Determine which slope set to use based on CSM mass relative to breakpoint
; If CSM mass > 33,956 lbs (SPS propellant > 10,000 lbs), use upper-range slopes
; If CSM mass <= 33,956 lbs (SPS propellant <= 10,000 lbs), use lower-range slopes
		CAE	CSMMASS		# GET DELTA CSM WEIGHT - SIGN DETERMINES
		AD	NEGBPW		# SLOPE LOCATIONS (breakpoint at 15402 kg)
		DOUBLE			# Scale for multiplication
		TS	TEMP333		# Save delta weight for slope calculations
# Page 954
		EXTEND
		BZMF	PEGGY		# DETERMINE CORRECT SLOPE
		CAF	NEG2		# CSM mass above breakpoint - use upper slopes
		TS	PHI333		# PHI333 = -2 selects VARST3, VARST4 slopes

; Main calculation loop: Output = Breakpoint + (Delta_Mass * Slope)
; Each of three passes computes one output: IXX0, IXX1, IXX2
; PHI333 selects slope (VARST5 for IXX, VARST6/3 for IAVG, VARST7/4 for IAVG/TLX)
; PSI333 selects which output register to store result
PEGGY		INDEX	PHI333		# ALL IS READY - CALCULATE OUTPUTS NOW
		CAE	VARST5		# GET SLOPE (indexed by PHI333)
		EXTEND
		MP	TEMP333		# MULT BY DELTA CSM WEIGHT
		DOUBLE			# Scale adjustment for fixed-point math
		INDEX	PSI333
		AD	VARST0		# ADD BREAKPOINT VALUE (indexed by PSI333)
		INDEX	PSI333
		TS	IXX		# ****** OUTPUTS (IXX0, IXX1, IXX2) ******


; Check if all three outputs computed (PSI333 decrements each pass: 2, 1, 0)
		CCS	PSI333		# BOOKKEEPING - MASSPROP FINISHED OR NOT
		TCF	BOKKEP2		# NO - GO TAKE CARE OF INDEXING REGISTERS

; All outputs computed. Update total vehicle weight for display and navigation
; Includes LEM mass if docked (BIT14 of DAPDATR1 indicates LEM presence)
		CAE	DAPDATR1	# UPDATE WEIGHT/G
		MASK	BIT14		# Test if LEM is attached
		CCS	A		# If BIT14 set, include LEMMASS
		CA	LEMMASS		# LEM attached - load LEM mass
		AD	CSMMASS		# Add CSM mass (accumulator was zero if LEM off)
		TS	WEIGHT/G	# SCALED AT B+16 KILOGRAMS
ENDMASSP	TC	Q		# Return to caller

; Loop bookkeeping: decrement both index registers and return for next pass
BOKKEP2		TS	PSI333		# REDUCE PSI BY ONE (CCS result already decremented)
		EXTEND
		DIM	PHI333		# Decrement slope selector
		TCF	PEGGY		# Continue to next output calculation

# Page 955
; ============================================================================
; DATA TABLES: Mass Properties Coefficients
;
; These tables define piecewise-linear approximations for vehicle inertia
; properties as functions of CSM and LEM masses. The approximations account
; for propellant depletion during SPS burns and configuration changes when
; LEM descent stage is jettisoned.
;
; Table Organization:
; - NOLEMVAL: Coefficients when LEM not attached (CSM-only configuration)
; - INTVALUE: Intercept values for LEM-attached configuration
; - SLOPEVAL: Slope coefficients as functions of LEM mass
; - NEGBPW: Negative breakpoint weight for CSM mass range selection
; - DXITFIX: Correction factor when LEM descent stage jettisoned
;
; Units: Inertia in kg-m², masses in kg, time ratios dimensionless
; ============================================================================

; NOLEMVAL: Configuration coefficients when LEM is not docked
; Used as base values for CSM-only flight (transearth coast, entry)
NOLEMVAL	DEC	25445 B-20	# IXX breakpoint (no LEM)
		DEC	87450 B-20	# IAVG breakpoint (no LEM)
		DEC	.30715 B-2	# IAVG/TLX breakpoint (no LEM)
		DEC	1.22877 E-5 B+12	# IAVG/TLX slope upper range (CSM > 33956 lbs)
		DEC	1.6096 B-6	# IAVG slope upper range (CSM > 33956 lbs)
		DEC	1.54 B-6	# IXX slope (constant for all CSM masses)
		DEC	7.77177 B-6	# IAVG slope lower range (CSM < 33956 lbs)
		DEC	3.46458 E-5 B+12	# IAVG/TLX slope lower range (CSM < 33956 lbs)

; INTVALUE: Intercept coefficients for LEM-docked configuration
; These are the constant terms in the linear equations
; Output = INTVALUE + (LEMMASS * SLOPEVAL)
INTVALUE	DEC	26850 B-20	# IXX intercept (LEM docked)
		DEC	127518 B-20	# IAVG intercept (LEM docked)
		DEC	.54059 B-2	# IAVG/TLX intercept (LEM docked)
		DEC	.153964 E-4 B+12	# IAVG/TLX upper slope intercept
		DEC	-.742923 B-6	# IAVG upper slope intercept
		DEC	1.5398 B-6	# IXX slope intercept (constant)
		DEC	9.68 B-6	# IAVG lower slope intercept
		DEC	.647625	E-4 B+12	# IAVG/TLX lower slope intercept
		DEC	-27228 B-20	# IAVG decrement when descent stage jettisoned
		DEC	-.206476 B-2	# IAVG/TLX decrement when descent stage jettisoned

; SLOPEVAL: Slope coefficients as functions of LEM mass
; These define how inertia properties change with LEM mass
; Output = INTVALUE + (LEMMASS * SLOPEVAL)
SLOPEVAL	DEC	1.96307 B-6	# IXX vs LEM mass slope
		DEC	27.5774 B-6	# IAVG vs LEM mass slope
		DEC	2.3548 E-5 B+12	# IAVG/TLX vs LEM mass slope
		DEC	2.1777 E-9 B+26	# IAVG/TLX upper slope sensitivity to LEM
		DEC	1.044 E-3 B+8	# IAVG upper slope sensitivity to LEM
		DEC	0		# IXX slope independent of LEM mass
		DEC	2.21068 E-3 B+8	# IAVG lower slope sensitivity to LEM
		DEC	1.5166 E-9 B+26	# IAVG/TLX lower slope sensitivity to LEM
		DEC	-1.284 B-6	# IAVG descent stage contribution
		DEC	2 E-5 B+12	# IAVG/TLX descent stage contribution

; Breakpoint weight: CSM mass threshold for slope selection
; 15402.17 kg = 33,956 lbs (corresponds to SPS propellant = 10,000 lbs)
; Negative value used in calculation: CSM mass + NEGBPW determines which slope set
NEGBPW		DEC	-15402.17 B-16	# Negative breakpoint weight (kg)

; Descent stage jettison correction factor for IAVG/TLX output
; Applied when LEM descent stage separated (ascent-only configuration)
DXITFIX		DEC*	-1.88275 E-5 B+12*	# IAVG/TLX descent stage correction
