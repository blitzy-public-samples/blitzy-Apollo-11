# Copyright:	Public domain.
# Filename:	RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1037-1038
# Mod history:	2009-05-13 RSB	Adapted from the Colossus249/ file of the
#				same name, using Comanche055 page images.
#		2009-05-20 RSB	A "Page N" comment was corrected.
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
; FILE: RCS-CSM_DAP_EXECUTIVE_PROGRAMS.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: all-phases
;
; TL;DR: RCS Digital Autopilot executive integration coordinating DAP tasks
;        with main AGC scheduler. Manages autopilot cycle timing, priority
;        handling, and integration with guidance and navigation systems throughout
;        Apollo 11 mission operations.
;
; COMMENT-ONLY READERS: This program coordinated the autopilot's timing with
;        other computer tasks to ensure smooth spacecraft attitude control.
; CODE-ALONG READERS: Study DAP executive integration with EXECUTIVE scheduler,
;        task timing coordination, and priority management.
; ============================================================================

# Page 1037
; ============================================================================
; COORDINATE TRANSFORMATION MATRIX UPDATE
;
; The RCS Digital Autopilot requires periodic updates to coordinate
; transformation matrices that convert between gimbal angles and body-fixed
; coordinates. This routine executes once per second as a background task
; scheduled by the EXECUTIVE, ensuring the autopilot has current orientation
; data for commanding reaction control system jets.
; ============================================================================

# CALCULATION OF  AMGB, AMBG  ONCE EVERY SECOND
#
#	AMGB =	1	SIN(PSI)		0
#		0	COS(PSI)COS(PHI)	SIN(PHI)
#		0	-COS(PSI)SIN(PHI)	COS(PHI)
#
#	AMBG =	1	-TAN(PSI)COS(PHI)	TAN(PSI)SIN(PHI)
#		0	COS(PHI)/COS(PSI)	-SIN(PHI)/COS(PSI)
#		0	SIN(PHI)		COS(PHI)
#
# WHERE PHI AND PSI ARE CDU ANGLES

; The AMGB matrix transforms vectors from gimbal coordinates to body-fixed
; coordinates. PHI and PSI are Coupling Data Unit (CDU) angles read from the
; Inertial Measurement Unit (IMU), representing the spacecraft's orientation.
; This transformation accounts for the physical mounting of RCS jet quads at
; 7.25 degrees offset from the spacecraft axes.


		BANK	20
		SETLOC	DAPS8
		BANK

		COUNT*	$$/DAPEX
		EBANK=	KMPAC

; ============================================================================
; AMBGUPDT - COORDINATE TRANSFORMATION MATRIX UPDATE ROUTINE
;
; This routine is called once per second by the executive scheduler to update
; the AMGB coordinate transformation matrix used by the RCS Digital Autopilot.
; The Command Module used these matrices throughout the mission to maintain
; proper spacecraft attitude using the reaction control system jets.
;
; EXECUTIVE INTEGRATION: Scheduled as a periodic background job, executing
; at low priority between higher-priority navigation and guidance tasks.
; ============================================================================

AMBGUPDT	CA	FLAGWRD6	# CHECK FOR RCS AUTOPILOT
		EXTEND
		BZMF	ENDOFJOB	# BIT15 = 0, BIT14 = 1
		MASK	BIT14		# IF NOT RCS, EXIT
		EXTEND
		BZF	ENDOFJOB	# TO PROTECT TVC DAP ON SWITCHOVER

; The autopilot mode check ensures this routine only executes when the RCS
; Digital Autopilot is active. During main engine burns when the Thrust Vector
; Control (TVC) system is engaged, this routine exits immediately to prevent
; interference. FLAGWRD6 contains autopilot mode status flags updated by the
; system control programs.

; ============================================================================
; AMGB MATRIX CALCULATION - PSI ANGLE COMPONENTS
;
; The transformation begins by computing sine and cosine of the PSI angle
; (yaw gimbal angle) from the IMU. CDUZ contains the current yaw gimbal
; reading from the Coupling Data Unit, scaled as fractional revolutions
; (±0.5 represents ±180 degrees).
; ============================================================================

		CA	CDUZ
		TC	SPSIN2
		TS	AMGB1		# CALCULATE AMGB
		CA	CDUZ
		TC	SPCOS2
		TS	CAPSI		# MUST CHECK FOR GIMBAL LOCK

; AMGB1 stores SIN(PSI) for the first row of the transformation matrix.
; CAPSI stores COS(PSI) which will be used in subsequent matrix element
; calculations. Gimbal lock conditions (middle gimbal near ±90 degrees)
; require special handling to maintain attitude control authority.

; ============================================================================
; PHI ANGLE CALCULATION WITH JET QUAD OFFSET COMPENSATION
;
; The Command Module's RCS jets are physically mounted in four quads offset
; 7.25 degrees from the spacecraft body axes. This geometric offset must be
; compensated in the transformation matrix to correctly command jet firings
; for desired spacecraft rotations.
; ============================================================================

		CAF	QUADANGL	# = 7.25  DEGREES JET QUAD ANGULAR OFFSET
		EXTEND
		MSU	CDUX
		COM			# CDUX - 7.25 DEG
		TC	SPCOS1
		TS	AMGB8
		EXTEND
		MP	CAPSI
		TS	AMGB4

; CDUX is the roll gimbal angle from the IMU. The calculation (CDUX - 7.25°)
; accounts for the physical mounting angle of the jet quads. AMGB8 stores
; COS(PHI) where PHI = CDUX - 7.25°. AMGB4 stores COS(PSI)*COS(PHI), which
; becomes the (2,2) element of the AMGB transformation matrix.

		CAF	QUADANGL
		EXTEND
		MSU	CDUX
		COM			# CDUX - 7.25 DEG
		TC	SPSIN1
		TS	AMGB5
		EXTEND
		MP	CAPSI
		COM

; AMGB5 stores SIN(PHI), and the multiplication with CAPSI produces
; -COS(PSI)*SIN(PHI), which is the (3,2) element of the AMGB matrix.
; The COM (complement) instruction negates the result to match the matrix
; definition. These transformation matrix elements enable the Digital
; Autopilot to convert desired spacecraft attitude rates into the correct
; combination of RCS jet firings throughout the Apollo 11 mission.

# Page 1038
		TS	AMGB7
		TCF	ENDOFJOB

; Job complete. Return control to the EXECUTIVE scheduler, which will dispatch
; the next waiting task. This routine's once-per-second execution rate provides
; sufficiently current transformation data for the autopilot without consuming
; excessive computational resources.

; ============================================================================
; PHYSICAL CONSTANT: RCS JET QUAD ANGULAR OFFSET
;
; QUADANGL defines the 7.25-degree angular offset of the Command Module's
; four RCS jet quads from the spacecraft body axes. This physical mounting
; geometry was chosen during Command Module design to optimize thrust vector
; effectiveness while avoiding engine plume impingement on spacecraft surfaces.
;
; Value: 660 decimal = 7.25 degrees in AGC angular scaling
; (Full scale 360° = 32768 counts, so 1° = 91.022 counts approximately)
; ============================================================================

QUADANGL	DEC	660		# = 7.25 DEGREES
