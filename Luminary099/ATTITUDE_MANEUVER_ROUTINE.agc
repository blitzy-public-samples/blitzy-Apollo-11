# Copyright:	Public domain.
# Filename:	ATTITUDE_MANEUVER_ROUTINE.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	342-363
# Mod history:	2009-05-16 RSB	Adapted from the corresponding
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
; FILE: ATTITUDE_MANEUVER_ROUTINE.agc
; MODULE: Attitude Control - Automated Maneuvers
; MISSION PHASE: lunar-orbit/descent/ascent/rendezvous
;
; TL;DR: Implements KALCMANU, the automated attitude maneuver routine that
;        commands the Lunar Module to rotate from its current orientation to
;        a desired attitude during free-fall flight. Computes optimal rotation
;        paths while avoiding gimbal lock, generates steering commands for the
;        RCS Digital Autopilot, and coordinates with WAITLIST for timed updates.
;
; COMMENT-ONLY READERS: This routine enables automated spacecraft rotations
;        required for all mission phases - aligning for engine burns, pointing
;        antennas toward Earth, orienting for docking, and preparing for landing.
; CODE-ALONG READERS: Study the direction cosine matrix mathematics, gimbal
;        lock avoidance algorithms, and DAP interface protocols that enable
;        precise attitude control using reaction control thrusters.
; ============================================================================

# Page 342
; ============================================================================
; KALCMANU - AUTOMATED ATTITUDE MANEUVER ROUTINE
;
; During free-fall coast phases of the Apollo 11 mission, the Lunar Module
; must frequently reorient itself - aligning for engine burns, pointing the
; high-gain antenna toward Earth for communications, preparing for docking
; with the Command Module, or positioning for landing radar acquisition.
; This routine computes the commands that direct the spacecraft to smoothly
; rotate from its current attitude to any desired orientation.
; ============================================================================
# BLOCK 2 LGC ATTITUDE MANEUVER ROUTINE -- KALCMANU
#
# MOD 2		DATE 5/1/67	BY DON KEENE
#
# PROGRAM DESCRIPTION
#
; The spacecraft's attitude is tracked using three gimbal angles measured by
; the Inertial Measurement Unit (IMU). These angles define the spacecraft's
; orientation relative to stable member (inertial) coordinates.
;
# KALCMANU IS A ROUTINE WHICH GENERATES COMMANDS FOR THE LM DAP TO CHANGE THE ATTITUDE OF THE SPACECRAFT
# DURING FREE FALL.  IT IS DESIGNED TO MANEUVER THE SPACECRAFT FROM ITS INITIAL ORIENTATION TO SOME DESIRED
# ORIENTATION SPECIFIED BY THE PROGRAM WHICH CALLS KALCMANU, AVOIDING GIMBAL LOCK IN THE PROCESS.  IN THE
# MOD 2 VERSION, THIS DESIRED ATTITUDE IS SPECIFIED BY A SET OF OF THREE COMMANDED CDU ANGLES STORES AS 2'S COMPLEMENT
# SINGLE PRECISION ANGLES IN THE THREE CONSECUTIVE LOCATIONS, CPHI, CTHETA, CPSI, WHERE
#
#	CPHI = COMMANDED OUTER GIMBAL ANGLE
# 	CTHETA = COMMANDED INNER GIMBAL ANGLE
#	CPSI = COMMANDED MIDDLE GIMBAL ANGLE
;
; These commanded CDU (Coupling Display Unit) angles represent the target
; orientation that calling programs specify. For example, during Apollo 11's
; translunar coast, the spacecraft periodically executed passive thermal
; control (PTC) "barbecue roll" maneuvers using KALCMANU commands.
#
# WHEN POINTING A SPACECRAFT AXIS (I.E., X, Y, Z, THE AOT, THRUST AXIS, ETC.) THE SUBROUTINE VECPOINT MAY BE
# USED TO GENERATE THIS SET OF DESIRED CDU ANGLES (SEE DESCRIPTION IN R60).
#
;
; SINGLE EQUIVALENT ROTATION CONCEPT:
; Rather than sequencing separate pitch, yaw, and roll rotations, KALCMANU
; computes the mathematically optimal single rotation that transforms the
; spacecraft from initial to final attitude. This rotation is characterized
; by a unit vector COF (direction of rotation axis) and a scalar AM (angle
; of rotation about that axis). This approach minimizes propellant usage.
;
# WITH THIS INFORMATION KALCMANU DETERMINES THE DIRECTION OF THE SINGLE EQUIVALENT ROTATION (COF ALSO U) AND THE
# MAGNITUDE OF THE ROTATION (AM) TO BRING THE S/C FROM ITS INITIAL ORIENTATION TO ITS FINAL ORIENTATION.
# THIS DIRECTION REMAINS FIXED BOTH IN INERTIAL COORDINATES AND IN COMMANDED S/C AXES THROUGHOUT THE
#                  _
# MANEUVER.  ONCE COF AND AM HAVE BEEN DETERMINED, KALCMANU THEN EXAMINES THE MANEUVER TO SEE IF IT WILL BRING
#				       _
# THE S/C THROUGH GIMBAL LOCK.  IF SO, COF AND AM ARE READJUSTED SO THAT THE S/C WILL JUST SKIM THE GIMBAL
# LOCK ZONE AND ALIGN THE X-AXIS.  IN GENERAL A FINAL YAW ABOUT X WILL BE NECESSARY TO COMPLETE THE MANEUVER.
# NEEDLESS TO SAY, NEITHER THE INITIAL NOR THE FINAL ORIENTATION CAN BE IN GIMBAL LOCK.
;
; GIMBAL LOCK AVOIDANCE:
; Gimbal lock occurs when the middle gimbal angle (theta) approaches ±90
; degrees, causing loss of one degree of freedom in the IMU. If the computed
; maneuver would pass through this zone, KALCMANU recalculates the path to
; "skim" the gimbal lock boundary, aligning the spacecraft's X-axis first,
; then completing the maneuver with a final yaw rotation about X.
#
;
; DIGITAL AUTOPILOT INTERFACE:
; The RCS Digital Autopilot (DAP) controls thruster firings to achieve the
; commanded attitude. KALCMANU provides the DAP with continuously updated
; reference targets including desired gimbal angles and rotation rates.
;
# FOR PROPER ATTITUDE CONTROL THE DIGITAL AUTOPILOT MUST BE GIVEN AN ATTITUDE REFERENCE WHICH IT CAN TRACK.
# KALCMANU DOES THIS BY GENERATING A REFERENCE OF DESIRED GIMBAL ANGLES (CDUXD, CDUYD, CDUZD) WHICH ARE UPDATED
# EVERY ONE SECOND DURING THE MANEUVER.  TO ACHIEVE A SMOOTHER SEQUENCE OF COMMANDS BETWEEN SUCCESSIVE UPDATES,
# THE PROGRAM ALSO GENERATES A SET OF INCREMENTAL CDU ANGLES (DELDCDU) TO BE ADDED TO CDU DESIRED BY THE DIGITAL
# AUTOPILOT.  KALCMANU ALSO CALCULATES THE COMPONENT MANEUVER RATES (OMEGAPD, OMEGAQD, OMEGARD), WHICH CAN
#				      _
# BE DETERMINED SIMPLY BY MULTIPLYING COF BY SOME SCALAR (ARATE) CORRESPONDING TO THE DESIRED ROTATIONAL RATE.
;
; Updates occur every second via WAITLIST task scheduling. Between updates,
; incremental angle commands (DELDCDU) provide smooth interpolation. Desired
; rotation rates (OMEGAPD, OMEGAQD, OMEGARD for pitch, yaw, roll) are computed
; by scaling the rotation axis vector COF by the maneuver rate ARATE.
#
;
; MANEUVER SEQUENCING AND TIMING:
; The maneuver proceeds in phases, coordinated by WAITLIST timer tasks:
; 1. Initial rotation about the computed axis until near final attitude
; 2. Y and Z rates zeroed, desired angles set to final values
; 3. If gimbal lock avoidance required a detour, final yaw about X-axis
; 4. All rates zeroed, spacecraft settles into limit cycle about target
;
# AUTOMATIC MANEUVERS ARE TIMED WTH THE HELP OF WAITLIST SO THAT AFTER A SPECIFIED INTERVAL THE Y AND Z
# DESIRED RATES ARE SET TO ZERO AND THE DESIRED CDU ANGLES (CDUYD, CDUZD) ARE SET EQUAL TO THE FINAL DESIRED CDU
# ANGLES (CTHETA, CPSI).  IF ANY YAW REMAINS DUE TO GIMBAL LOCK AVOIDANCE, THE FINAL YAW MANEUVER IS
# CALCULATED AND THE DESIRED YAW RATE SET TO SOME FIXED VALUE (ROLLRATE = + OR - 2 DEGREES PER SEC).
# IN THIS CASE ONLY AN INCREMENTAL CDUX ANGLE (DELFROLL) IS SUPPLIED TO THE DAP.  AT THE END OF THE YAW
# MANEUVER OR IN THE EVENT THAT THERE WAS NO FINAL YAW, CDUXD IS SET EQUAL TO CPHI AND THE X-AXIS DESIRED
# RATE SET TO ZERO.  THUS, UPON COMPLETION OF THE MANEUVER THE S/C WILL FINISH UP IN A LIMIT CYCLE ABOUT THE
# DESIRED GIMBAL ANGLES.
;
; The final yaw rate is fixed at ±2 degrees per second (ROLLRATE). Upon
; completion, the spacecraft enters a stable limit cycle - small oscillations
; about the target attitude maintained by the DAP with minimal thruster use.
#
; ============================================================================
; TRANSITION: From program description to implementation logic
;
; Having established the mathematical framework and operational requirements,
; the program now describes its execution flow. KALCMANU runs as a high-
; priority executive job, ensuring attitude changes receive computational
; priority over lower-priority tasks like display updates.
; ============================================================================
;
# PROGRAM LOGIC FLOW
#
; ENTRY POINTS:
; KALCMAN3 - Primary entry for attitude maneuvers with pre-computed target CDU angles
; VECPOINT - Entry for pointing a spacecraft axis (e.g., X, Y, Z, AOT, thrust axis)
;            toward a desired direction vector; computes target CDU angles first
;
# KALCMANU IS CALLED AS A HIGH PRIORITY JOB WITH ENTRY POINTS AT KALCMAN3 AND VECPOINT.  IT FIRST PICKS
# UP THE CURRENT CDU ANGLES TO BE USED AS THE BASIS FOR ALL COMPUTATIONS INVOLVING THE INITIAL S/C ORIENTATION.
;
; The current CDU angles are read atomically from the IMU hardware registers
; to establish a consistent snapshot of the initial spacecraft orientation.
# Page 343
;
; DIRECTION COSINE MATRIX COMPUTATION:
; Three key transformation matrices are computed:
; MIS - Maps initial spacecraft body axes to stable member (inertial) axes
; MFS - Maps final spacecraft body axes to stable member axes
; MFI - Maps final spacecraft axes to initial spacecraft axes (MFI = MIS' * MFS)
;
; These matrices enable all subsequent rotation calculations.
;
# IT THEN DETERMINES THE DIRECTION COSINE MATRICES RELATING BOTH THE INITIAL AND FINAL S/C ORIENTATION TO STABLE
#               *   *                                                                               *
# MEMBER AXES (MIS,MFS).  IT ALSO COMPUTES THE MATRIX RELATING FINAL S/C AXES TO INITIAL S/C AXES (MFI).  THE
# ANGLE OF ROTATION (AM) IS THEN EXTRACTED FROM THIS MATRIX, AND TEST ARE MADE TO DETERMINE IF
#
#	A)	AM LESS THAN .25 DEGREES (MINANG)
#	B)	AM GREATER THAN 170 DEGREES (MAXANG)
;
; SMALL ANGLE OPTIMIZATION:
; If the rotation angle is less than 0.25 degrees, no sophisticated maneuver
; computation is needed - the desired CDU angles are simply set directly.
#
# IF AM IS LESS THAN .25 DEGREES, NO COMPLICATED AUTOMATIC MANEUVERING IS NECESSARY.  THEREFORE, WE CAN SIMPLY
# SET CDU DESIRED EQUAL TO THE FINAL CDU DESIRED ANGLES AND TERMINATE THE JOB.
#
; ROTATION AXIS EXTRACTION METHODS:
; For moderate rotations (0.25° to 170°), the rotation axis COF is extracted
; from the skew-symmetric components of the MFI matrix. For large rotations
; (>170°), numerical precision issues require using the symmetric components
; of MFI instead. This dual-method approach ensures accuracy across all cases.
;
# IF AM IS GREATER THAN .25 DEGREES BUT LESS THAN 170 DEGREES THE AXES OF THE SINGLE EQUIVALENT ROTATION
#   _                                                       *
# (COF) IS EXTRACTED FROM THE SKEW SYMMETRIC COMPONENTS OF MFI.
#                                                                                     *     *
# IF AM GREATER THAN 170 DEGREES AN ALTERNATE METHOD EMPLOYING THE SYMMETRIC PART OF MFI (MFISYM) IS USED
#               _
# TO DETERMINE COF.
#
# THE PROGRAM THEN CHECKS TO SEE IF THE MANEUVER AS COMPUTED WILL BRING THE S/C THROUGH GIMBAL LOCK.  IF
# SO, A NEW MANEUVER IS CALCULATED WHICH WILL JUST SKIM THE GIMBAL LOCK ZONE AND ALIGN THE S/C X-AXIS.  THIS
# METHOD ASSURES THAT THE ADDITIONAL MANEUVERING TO AVOID GIMBAL LOCK WILL BE KEPT TO A MINIMUM.  SINCE A FINAL
# P AXIS YAW WILL BE NECESSARY, A SWITCH IS RESET (STATE SWITCH 31) TO ALLOW FOR THE COMPUTATION OF THIS FINAL
# YAW.
#
# AS STATED PREVIOUSLY, KALCMANU GENERATES A SEQUENCE OF DESIRED GIMBAL ANGLES WHICH ARE UPDATED EVERY
#                                                                                              _
# SECOND.  THIS IS ACCOMPLISHED BY A SMALL ROTATION OF THE DESIRED S/C FRAME ABOUT THE VECTOR COF.  THE NEW
# DESIRED REFERENCE MATRIX IS THEN,
#	 *		 *	 *
#	MIS	=	MIS	DEL
#	   N+1		   N
#        *
# WHERE DEL IS THE MATRIX CORRESPONDING TO THIS SMALL ROTATION.  THE NEW CDU ANGLES CAN THEN BE EXTRACTED
#       *
# FROM MIS.
#
# AT THE BEGINNING OF THE MANEUVER THE AUTOPILOT DESIRED RATES (OMEGAPD, OMEGAQD, OMEGARD) AND THE
# MANEUVER TIMINGS ARE ESTABLISHED.  ON THE FIRST PASS AND ON ALL SUBSEQUENT UPDATES THE CDU DESIRED
# ANGLES ARE LOADED WITH THE APPROPRIATE VALUES AND THE INCREMENTAL CDU ANGLES ARE COMPUTED.  THE AGC CLOCKS
# (TIME1 AND TIME2) ARE THEN CHECKED TO SEE IF THE MANEUVER WILL TERMINATE BEFORE THE NEXT UPDATE.  IF
# NOT, KALCMANU CALLS FOR ANOTHER UPDATE (RUN AS A JOB WITH PRIORITY TBD) IN ONE SECOND.  ANY DELAYS IN THIS
# CALLING SEQUENCE ARE AUTOMATICALLY COMPENSATED IN CALLING FOR THE NEXT UPDATE.
#
# IF IT IS FOUND THAT THE MANEUVER IS TO TERMINATE BEFORE THE NEXT UPDATE A ROUTINE IS CALLED (AS A WAIT-
# LIST TASK) TO STOP THE MANEUVER AT THE APPROPRIATE TIME AS EXPLAINED ABOVE.

# Page 344
# CALLING SEQUENCE
#
# IN ORDER TO PERFORM A KALCMANU SUPERVISED MANEUVER, THE COMMANDED GIMBAL ANGLES MUST BE PRECOMPUTED AND
# STORED IN LOCATIONS CPHI, CTHETA, CPSI.  THE USER'S PROGRAM MUST THEN CLEAR STATE SWITCH NO 33 TO ALLOW THE
# ATTITUDE MANEUVER ROUTINE TO PERFORM ANY FINAL P-AXIS YAW INCURRED BY AVOIDING GIMBAL LOCK.  THE MANEUVER IS
# THEN INITIATED BY ESTABLISHING THE FOLLOWING EXECUTIVE JOB
#		       *
#	CAF	PRIO XX
#		     --
#	INHINT
#	TC	FINDVAC
#	2CADR	KALCMAN3
#	RELINT
#
# THE USER'S PROGRAM MAY EITHER CONTINUE OR WAIT FOR THE TERMINATION OF THE MANEUVER.  IF THE USER WISHES TO
# WAIT, HE MAY PUT HIS JOB TO SLEEP WITH THE FOLLOWING INSTRUCTIONS:
#
#	L	TC	BANKCALL
#	L+1	CADR	ATTSTALL
#	L+2	(BAD RETURN)
#	L+3	(GOOD RETURN)
#
# UPON COMPLETION OF THE MANEUVER, THE PROGRAM WILL BE AWAKENED AT L+3 IF THE MANEUVER WAS COMPLETED
# SUCCESSFULLY, OR AT L+2 IF THE MANEUVER WAS ABORTED.  THIS ABORT WOULD OCCUR IF THE INITIAL OR FINAL ATTITUDE
# WAS IN GIMBAL LOCK.
#
# *** NOTA BENE ***  IF IT IS ASSUMED THAT THE DESIRED MANEUVERING RATE (0.5, 2, 5, 10 DEG/SEC) HAS BEEN SELECTED BY
# KEYBOARD ENTRY PRIOR TO THE EXECUTION OF KALCMANU.
#
# IT IS ALSO ASSUMED THAT THE AUTOPILOT IS IN THE AUTO MODE.  IF THE MODE SWITCH IS CHANGED DURING THE
# MANEUVER, KALCMANU WILL TERMINATE VIA GOODEND WITHIN 1 SECOND SO THAT R60 MAY REQUEST A TRIM OF THE S/C ATTITUDE
# SUBROUTINES.
#
# KALCMANU USES A NUMBER OF INTERPRETIVE SUBROUTINES WHICH MAY BE OF GENERAL INTEREST.  SINCE THESE ROUTINES
# WERE PROGRAMMED EXCLUSIVELY FOR KALCMANU, THEY ARE NOT, AS YET, GENERALLY AVAILABLE FOR USE BY OTHER PROGRAMS.
#
# MXM3
# ----
#
# THIS SUBROUTINE MULTIPLIES TWO 3X3 MATRICES AND LEAVES THE RESULT IN THE FIRST 18 LOCATIONS OF THE PUSH
# DOWN LIST, I.E.,
#			[ M     M     M  ]
#			[  0     1     2 ]
#	*		[                ]		*		*
#	M	=	[ M     M     M  ]	=	M1	X	M2
#			[  3     4     5 ]
#			[                ]
#			[ M     M     M  ]
#			[  6     7     8 ]
# Page 345
#                                                                                  *
# INDEX REGISTER X1 MUST BE LOADED WITH THE COMPLEMENT OF THE STARTING ADDRESS FOR M1, AND X2 MUST BE
#                                                        *
# LOADED WITH THE COMPLEMENT OF THE STARTING ADDRESS FOR M2.  THE ROUTINE USES THE FIRST 20 LOCATIONS OF THE PUSH
# DOWN LIST.  THE FIRST ELEMENT OF THE MATRIX APPEARS IN PDO.  PUSH UP FOR M .
#                                                                           8
# TRANSPOS
# --------
#
# THIS ROUTINE TRANSPOSES A 3X3 MATRIX AND LEAVES THE RESULT IN THE PUSH DOWN LIST, I.E.,
#
#	*		* T
#	M	=	M1
#
# INDEX REGISTER X1 MUST CONTAIN THE COMPLEMENT OF THE STARTING ADDRESS FOR M1.  PUSH UP FOR THE FIRST AND SUB-
#                        *
# SEQUENT COMPONENTS OF M.  THIS SUBROUTINE ALSO USES THE FIRST 20 LOCATIONS OF THE PUSH DOWN LIST.
#
# CDU TO DCM
# ----------
#
# THIS SUBROUTINE CONVERTS THREE CDU ANGLES IN T(MPAC) TO A DIRECTION COSINE MATRIX (SCALED BY 2) RELATING
# THE CORRESPONDING S/C ORIENTATIONS TO THE STABLE MEMBER FRAME.  THE FORMULAS FOR THIS CONVERSION ARE
#
#	M	=	COSY COSZ
#	 0
#
#	M	=	-COSY SINZ COSX + SINY SINX
#	 1
#
#	M	=	COSY SINZ SINX + SINY COSX
#	 2
#
#	M	=	SINZ
#	 3
#
#	M	=	COSZ COSX
#	 4
#
#	M	=	-COSZ SINX
#	 5
#
#	M	=	-SINY COSZ
#	 6
#
#	M	=	SINY SINZ COSX + COSY SINX
#	 7
# Page 346
#	M	=	-SINY SINZ SINX + COSY COSX
#	 8
#
# WHERE		X	=	OUTER GIMBAL ANGLE
#		Y	=	INNER GIMBAL ANGLE
#		Z	=	MIDDLE GIMBAL ANGLE
#
# THE INTERPRETATION OF THIS MATRIX IS AS FOLLOWS:
#
# IF A , A , A  REPRESENT THE COMPONENTS OF A VECTOR IN S/C AXES THEN THE COMPONENTS OF THE SAME VECTOR IN
#     X   Y   Z
# STABLE MEMBER AXES (B , B , B ) ARE
#                      X   Y   Z
#
#	[ B  ]			[ A  ]
#	[  X ]			[  X ]
#	[    ]			[    ]
#	[ B  ]		  *	[ A  ]
#	[  Y ]	   =	  M	[  Y ]
#	[    ]			[    ]
#	[ B  ]			[ B  ]
#	[  Z ]			[  Z ]
#
# THE SUBROUTINE WILL STORE THIS MATRIX IN SEQUENTIAL LOCATIONS OF ERASABLE MEMORY AS SPECIFIED BY THE CALLING
#                                                                                                             *
# PROGRAM.  TO DO THIS THE CALLING PROGRAM MUST FIRST LOAD X2 WITH THE COMPLEMENT OF THE STARTING ADDRESS FOR M.
#
# INTERNALLY, THE ROUTINE USES THE FIRST 16 LOCATIONS OF THE PUSH DOWN LIST, ALSO STEP REGISTER S1 AND INDEX
# REGISTER X2.
#
# DCM TO CDU
# ----------
#								       *
# THIS ROUTINE EXTRACTS THE CDU ANGLES FROM A DIRECTION COSINE MATRIX (M SCALED BY 2) RELATING S/C AXIS TO
#                                                                                 *
# STABLE MEMBER AXES.  X1 MUST CONTAIN THE COMPLEMENT OF THE STARTING ADDRESS FOR M.  THE SUBROUTINE LEAVES THE
# CORRESPONDING GIMBAL ANGLES IN V(MPAC) AS DOUBLE PRECISION 1'S COMPLEMENT ANGLES SCALED BY 2PI.  THE FORMULAS
# FOR THIS CONVERSION ARE
#
#	Z 	=	ARCSIN (M  )
#			         3
#
#	Y	=	ARCSIN (-M /COSZ)
#			          6
#
# IF M  IS NEGATIVE, Y IS REPLACED BY PI SGN Y - Y.
#     0
# Page 347
#	X	=	ARCSIN (-M /COSZ)
#			          5
#
# IF M  IS NEGATIVE, X IS REPLACED BY PI SGN X - X.
#     4
#
# THIS ROUTINE DOES NOT SET THE PUSH DOWN POINTER, BUT USES THE NEXT 8 LOCATIONS OF THE PUSH DOWN LIST AND
# RETURNS THE POINTER TO ITS ORIGINAL SETTING.  THIS PROCEDURE ALLOWS THE CALLER TO STORE THE MATRIX AT THE TOP OF
# THE PUSH DOWN LIST.
#
# DELCOMP
# -------
#                                                     *
# THIS ROUTINE COMPUTES THE DIRECTION COSINE MATRIX (DEL) RELATING ON
#                                                                          _
# IS ROTATED WITH RESPECT TO THE FIRST BY AN ANGLE, A, ABOUT A UNIT VECTOR U.  THE FORMULA FOR THIS MATRIX IS
#
#	 *		*	 _ _T              *
#	DEL	=	I COSA + U U  (1 - COSA) + V  SINA
#			                            X
#
# WHERE		*		[ 1    0    0 ]
#		I	=	[ 0    1    0 ]
#				[ 0    0    1 ]
#
#				[    2                             ]
#				[  U           U  U          U  U  ]
#				[   X           X  Y          X  Z ]
#				[                                  ]
#		_ _T		[                 2                ]
#		U U	=	[ U  U          U            U  U  ]
#				[  Y  X          Y            Y  Z ]
#				[                                  ]
#				[                               2  ]
#				[ U  U         U  U           U    ]
#				[  Z  X         Z  Y           Z   ]
#
#
#				[   0		-U		 U  ]
#				[		  Z		  Y ]
#		*		[				    ]
#		V	=	[  U		 0		-U  ]
#		 X		[   Z                             X ]
#				[				    ]
#				[ -U		 U		 0  ]
#				[   Y 		  X		    ]
#
# Page 348
#	_
#	U	=	UNIT ROTATION VECTOR RESOLVED INTO S/C AXES.
#	A	=	ROTATION ANGLE
#
#                        *
# THE INTERPRETATION OF DEL IS AS FOLLOWS:
#
# IF A , A , A  REPRESENT THE COMPONENTS OF A VECTOR IN THE ROTATED FRAME, THEN THE COMPONENTS OF THE SAME
#     X   Y   Z
# VECTOR IN THE ORIGINAL S/C AXES (B , B , B ) ARE
#                                   X   Y   Z
#
#	[ B  ]			[ A  ]
#	[  X ]			[  X ]
#	[    ]			[    ]
#	[ B  ]		  *	[ A  ]
#	[  Y ]	   =	 DEL	[  Y ]
#	[    ]			[    ]
#	[ B  ]			[ B  ]
#	[  Z ]			[  Z ]
#
# THE ROUTINE WILL STORE THIS MATRIX (SCALED UNITY) IN SEQUENTIAL LOCATIONS OF ERASABLE MEMORY BEGINNING WITH
#                                                                                             _
# THE LOCATION CALLED DEL.  IN ORDER TO USE THE ROUTINE, THE CALLING PROGRAM MUST FIRST STORE U (A HALF UNIT
# DOUBLE PRECISION VECTOR) IN THE SET OF ERASABLE LOCATIONS BEGINNING WITH THE ADDRESS CALLED COF.  THE ANGLE, A,
# MUST THEN BE LOADED INTO D(MPAC).
#
# INTERNALLY, THE PROGRAM ALSO USES THE FIRST 10 LOCATIONS OF THE PUSH DOWN LIST.
#
# READCDUK
# --------
#
# THIS BASIC LANGUAGE SUBROUTINE LOADS T(MPAC) WITH THE THREE CDU ANGLES.
#
# SIGNMPAC
# --------
#
# THIS IS A BASIC LANGUAGE SUBROUTINE WHICH LIMITS THE MAGNITUDE OF D(MPAC) TO + OR - DPOSMAX ON OVERFLOW.
#
# PROGRAM STORAGE ALLOCATION
#
#	1)	FIXED MEMORY		1059 WORDS
#	2)	ERASABLE MEMORY		  98
#	3)	STATE SWITCHES		   3
# Page 349
#	4)	FLAGS			   1
#
# JOB PRIORITIES
#
#	1)	KALCMANU		TBD
#	2)	ONE SECOND UPDATE	TBD
#
# SUMMARY OF STATE SWITCHES AND FLAGWORDS USED BY KALCMANU.
#
#	STATE		FLAGWRD 2	SETTING		MEANING
#	SWITCH NO.	BIT NO.
#
#	  *
#	31		14		0		MANEUVER WENT THROUGH GIMBAL LOCK
#					1		MANEUVER DID NOT GO THROUGH GIMBAL LOCK
#	  *
#	32		13		0		CONTINUE UPDATE PROCESS
#					1		START UPDATE PROCESS
#
#	33		12		0		PERFORM FINAL P AXIS YAW IF REQUIRED
#					1		IGNORE ANY FINAL P-AXIS YAW
#
#	34		11		0		SIGNAL END OF KALCMANU
#					1		KALCMANU IN PROCESS.	USER MUST SET SWITCH BEFORE INITIATING
#
#	* INTERNAL TO KALCMANU
#
# SUGGESTIONS FOR PROGRAM INTEGRATION
#
# THE FOLLOWING VARIABLES SHOULD BE ASSIGNED TO UNSWITCH ERASABLE:
#
#	CPHI
#	CTHETA
#	CPSI
#	POINTVSM +5
#	SCAXIS 	 +5
#	DELDCDU
#	DELDCDU1
#	DELDCDU2
#	RATEINDX
#
# THE FOLLOWING SUBROUTINES MAY BE PUT IN A DIFFERENT BANK
#
#	MXM3
# Page 350
#	TRANSPGS
#	SIGNMPAC
#	READCDUK
#	CDUTODCM

# Page 351
		BANK	15
		SETLOC	KALCMON1
		BANK

		EBANK=	BCDU

# THE THREE DESIRED CDU ANGLES MUST BE STORED AS SINGLE PRECISION TWO'S COMPLEMENT ANGLES IN THE THREE SUCCESSIVE
# LOCATIONS, CPHI, CTHETA, CPSI.

; ============================================================================
; TRANSITION: From Program Setup to Maneuver Execution
;
; The attitude maneuver routine begins here. The Lunar Module needs to rotate
; from its current orientation to a new commanded attitude. This might be to
; point the landing radar at the lunar surface during descent, align the
; rendezvous radar toward the Command Module, or orient for a planned engine
; burn. The routine will command the RCS autopilot to fire thruster jets in
; precise sequences to achieve smooth, controlled rotation while avoiding
; gimbal lock - a condition where the IMU loses its ability to sense attitude.
; ============================================================================

		COUNT*	$$/KALC

; KALCMAN3 - Primary Entry Point for Attitude Maneuver Command
;
; COMMENT-ONLY READERS: This is where automated spacecraft rotation begins.
; The computer reads the current spacecraft orientation from the IMU gimbal
; angles, then plans the most efficient rotation path to reach the desired
; attitude commanded by the mission program. During Apollo 11's lunar orbit,
; this routine executed numerous times to orient the LM for landing radar
; checks, optical navigation, and rendezvous preparation.
;
; CODE-ALONG READERS: Entry point KALCMAN3 initiates the KALCMANU maneuver
; sequence. The routine uses interpretive language (TC INTPRET) for matrix
; operations. READCDUK subroutine loads current CDU angles (CDUX, CDUY, CDUZ)
; into MPAC. These angles represent the three-axis gimbal orientation of the
; IMU platform. The routine then validates that the desired middle gimbal
; angle (CPSI) is not near gimbal lock (±70 degrees from vertical).

KALCMAN3	TC	INTPRET		# PICK UP THE CURRENT CDU ANGLES AND
		RTB			#	COMPUTE THE MATRIX FROM INITIAL S/C
			READCDUK	#	AXES TO FINAL S/C AXES.
		STORE	BCDU		# STORE INITIAL S/C ANGLES
		SLOAD	ABS		# CHECK THE MAGNITUDE OF THE DESIRED
			CPSI		# MIDDLE GIMBAL ANGLE
		DSU	BPL
			LOCKANGL	# IF GREATER THAN 70 DEG ABORT MANEUVER
			TOOBADF
; Initial Spacecraft Orientation Matrix Computation
; This section converts the current CDU gimbal angles (BCDU) into a direction
; cosine matrix (DCM) that mathematically represents the spacecraft's current
; orientation relative to the stable member (inertial reference frame).
; The matrix MIS (Matrix Initial Spacecraft) contains 9 direction cosines.

		AXC,2	TLOAD
			MIS
			BCDU
		CALL			# COMPUTE THE TRANSFORMATION FROM INITIAL
			CDUTODCM	# S/C AXES TO STABLE MEMBER AXES

; Desired Spacecraft Orientation Matrix Computation
; Similarly, convert the commanded CDU angles (CPHI, CTHETA, CPSI) into
; matrix MFS (Matrix Final Spacecraft) representing the desired orientation.
; These commanded angles were set by the calling program (P20 rendezvous,
; P63 landing, etc.) to point the spacecraft in the required direction.

		AXC,2	TLOAD
			MFS		# PREPARE TO CALCULATE ARRAY MFS
			CPHI
		CALL
			CDUTODCM
; Matrix Rotation Computation
;
; COMMENT-ONLY READERS: The computer must now calculate the single rotation
; that transforms the spacecraft from its current orientation to the desired
; orientation. This involves complex three-dimensional matrix mathematics to
; determine both the axis around which to rotate and the angle of rotation.
;
; CODE-ALONG READERS: The routine computes MFI (Matrix Final-to-Initial) by:
; 1. Transposing MIS to get TMIS (inverse rotation from inertial to initial S/C)
; 2. Multiplying TMIS × MFS to get the net rotation matrix MFI
; 3. Creating TMFI as the transpose of MFI for subsequent calculations
; These matrix operations use interpretive instructions (VLOAD, STOVL, STORE)
; with proper scaling (×2 for transpose, ×4 for matrix product).

SECAD		AXC,1	CALL		# MIS AND MFS ARRAYS CALCULATED		$2
			MIS
			TRANSPOS
		VLOAD	STADR
		STOVL	TMIS +12D
		STADR
		STOVL	TMIS +6
		STADR
		STORE	TMIS		# TMIS = TRANSPOSE(MIS) SCALED BY 2
		AXC,1	AXC,2
			TMIS
			MFS
		CALL
			MXM3
		VLOAD	STADR
		STOVL	MFI +12D
		STADR
		STOVL	MFI +6
		STADR
		STORE	MFI		# MFI = TMIS MFS (SCALED BY 4)
		SETPD	CALL		# TRANSPOSE MFI IN PD LIST
# Page 352
			18D
			TRNSPSPD
		VLOAD	STADR
		STOVL	TMFI 	+12D
		STADR
		STOVL	TMFI 	+6
		STADR
		STORE	TMFI		# TMFI = TRANSPOSE (MFI) SCALED BY 4

; ============================================================================
; Rotation Axis Extraction (COFSKEW Computation)
;
; COMMENT-ONLY READERS: Every rotation in three-dimensional space can be
; described as a single rotation around one specific axis. The computer
; extracts this rotation axis from the mathematical difference between the
; current and desired orientations. This axis remains fixed in space
; throughout the maneuver, while the RCS jets fire to spin the spacecraft
; around it.
;
; CODE-ALONG READERS: The rotation axis (COF) is extracted from the skew-
; symmetric component of matrix MFI. The skew-symmetric matrix COFSKEW is
; computed as (MFI - TMFI)/2. The three components of COFSKEW represent
; the rotation axis scaled by 2/sin(AM). For small angles, this scaling
; approaches 2/AM. The interpretive DSU (double subtract) operations
; compute the differences between corresponding off-diagonal elements.
; ============================================================================

# CALCULATE COFSKEW AND MFISYM

		DLOAD	DSU
			TMFI 	+2
			MFI 	+2
		PDDL	DSU		# CALCULATE COF SCALED BY 2/SIN(AM)
			MFI 	+4
			TMFI 	+4
		PDDL	DSU
			TMFI 	+10D
			MFI 	+10D
		VDEF
		STORE	COFSKEW		# EQUALS MFISKEW

; Rotation Angle Calculation and Maneuver Classification
;
; COMMENT-ONLY READERS: The computer calculates how far the spacecraft must
; rotate. If the rotation is tiny (less than 0.25 degrees), no complex
; maneuvering is needed - the autopilot simply switches to holding the new
; attitude. If the rotation is moderate (0.25 to 170 degrees), the normal
; maneuver sequence executes. If the rotation is very large (over 170 degrees),
; a special calculation method is used because the standard equations become
; numerically unstable near 180-degree rotations.
;
; CODE-ALONG READERS: The rotation angle AM is extracted from the trace of
; matrix MFI using: cos(AM) = (trace(MFI) - 1) / 2. The trace is the sum of
; diagonal elements MFI(0,0) + MFI(1,1) + MFI(2,2). ARCCOS converts this to
; the angle AM (scaled by 2 for half-unit representation). Three cases:
; 1) AM < MINANG (0.25°): Too small for active maneuvering, skip to TOOBADI
; 2) MINANG ≤ AM ≤ MAXANG (170°): Normal case, use COFSKEW directly
; 3) AM > MAXANG: Large angle, use symmetric part MFISYM for stability

# CALCULATE AM AND PROCEED ACCORDING TO ITS MAGNITUDE

		DLOAD	DAD
			MFI
			MFI 	+16D
		DSU	DAD
			DP1/4TH
			MFI 	+8D
		STORE	CAM		# CAM = (MFI0+MFI4+MFI8-1)/2 HALF SCALE
		ARCCOS
		STORE	AM		# AM=ARCCOS(CAM)	(AM SCALED BY 2)
		DSU	BPL
			MINANG
			CHECKMAX
		TLOAD			# MANEUVER LESS THAN .25 DEGREES
			CPHI		# GO DIRECTLY INTO ATTITUDE HOLD
		STCALL	CDUXD		# ABOUT COMMANDED ANGLES
			TOOBADI		# STOP RATE AND EXIT

; Maneuver Angle Decision Point
;
; COMMENT-ONLY READERS: For normal rotations (under 170 degrees), the computer
; uses the rotation axis calculated earlier. But for very large rotations
; approaching 180 degrees, a different mathematical method is needed to avoid
; numerical errors. This is similar to how calculators switch methods near
; certain problem points to maintain accuracy.
;
; CODE-ALONG READERS: Compare AM against MAXANG (170 degrees). If AM ≤ MAXANG,
; the normal path loads COFSKEW, normalizes it to unit length (UNIT instruction),
; and stores as COF. If AM > MAXANG, branch to ALTCALC (alternate calculation).

CHECKMAX	DLOAD	DSU
			AM
			MAXANG
		BPL	VLOAD
			ALTCALC		# UNIT
			COFSKEW		# COFSKEW
		UNIT
		STORE	COF		# COF IS THE MANEUVER AXIS
# Page 353
		GOTO			# SEE IF MANEUVER GOES THRU GIMBAL LOCK
			LOCSKIRT

; Alternate COF Calculation for Large Angles (AM > 170°)
;
; COMMENT-ONLY READERS: When the spacecraft must rotate nearly 180 degrees,
; the standard calculation method becomes unreliable due to the mathematics
; involved. Instead, the computer uses the symmetric part of the rotation
; matrix (the part that's the same on both sides of the diagonal) to accurately
; determine the rotation axis. This ensures the maneuver remains precise even
; for these extreme rotations, which might occur during emergency abort
; procedures or unusual mission phases.
;
; CODE-ALONG READERS: ALTCALC computes the symmetric matrix MFISYM = (MFI + TMFI)/2.
; For large angles near 180°, the symmetric part dominates and provides better
; numerical conditioning. The routine then extracts COF components from MFISYM
; using: COF_i = sqrt((MFISYM_ii - CAM)/(1 - CAM)) for i=x,y,z. The SQRT
; operations compute each axis component. This method is mathematically equivalent
; but numerically stable for cos(AM) → -1 (AM → 180°).

ALTCALC		VLOAD	VAD		# IF AM GREATER THAN 170 DEGREES
			MFI
			TMFI
		VSR1
		STOVL	MFISYM
			MFI 	+6
		VAD	VSR1
			TMFI 	+6
		STOVL	MFISYM 	+6
			MFI 	+12D
		VAD	VSR1
			TMFI 	+12D
		STORE	MFISYM 	+12D	# MFISYM=(MFI+TMFI)/2	SCALED BY 4

# CALCULATE COF

		DLOAD	SR1
			CAM
		PDDL	DSU		# PDO CAM				$4
			DPHALF
			CAM
		BOVB	PDDL		# PS2 1 - CAM				$2
			SIGNMPAC
			MFISYM 	+16D
		DSU	DDV
			0
			2
		SQRT	PDDL		# COFZ = SQRT(MFISYM8-CAM)/(1-CAM)
			MFISYM 	+8D	#				$ ROOT 2
		DSU	DDV
			0
			2
		SQRT	PDDL		# COFY = SQRT(MFISYM4-CAM)/(1-CAM)  $ROOT2
			MFISYM
		DSU	DDV
			0
			2
		SQRT	VDEF		# COFX = SQRT(MFISYM-CAM)/(1-CAM)  $ROOT 2
		UNIT
		STORE	COF

; COF Sign Determination for Large-Angle Maneuvers
;
; COMMENT-ONLY READERS: The rotation axis direction has been calculated, but
; the mathematics only gives us the magnitude of each component, not the sign
; (positive or negative). The computer now determines the correct signs by
; examining which axis component is largest, then checking cross-products to
; ensure all three components point in the right direction. This is crucial
; because rotating around an axis in the wrong direction would cause the
; spacecraft to turn the long way around.
;
; CODE-ALONG READERS: The square root operations in ALTCALC produce unsigned
; magnitudes. To determine signs, the routine identifies the largest COF
; component (METHOD1 for X, METHOD2 for Y, METHOD3 for Z) and uses COFSKEW
; to determine its sign. The remaining component signs are determined from
; off-diagonal MFISYM elements, which encode the relative signs through
; products like MFISYM(0,1) = COF(X)*COF(Y)*factor. The logic flows through
; COFMAXGO → METHODn → UnPOS → OKUnn to systematically resolve all signs.

# DETERMINE LARGEST COF AND ADJUST ACCORDINGLY

COFMAXGO	DLOAD	DSU
			COF
			COF 	+2
		BMN	DLOAD		# COFY G COFX
# Page 354
			COMP12
			COF
		DSU	BMN
			COF 	+4
			METHOD3		# COFZ G COFX OR COFY
		GOTO
			METHOD1		# COFX G COFY OR COFZ
COMP12		DLOAD	DSU
			COF 	+2
			COF 	+4
		BMN
			METHOD3		# COFZ G COFY OR COFX

; METHOD2: Sign Determination When Y-Axis Component is Largest
;
; COMMENT-ONLY READERS: The Y-component of the rotation axis is the largest.
; The computer first checks if this Y-component is positive or negative by
; examining the off-diagonal matrix elements computed earlier. If negative,
; it reverses the signs of all three components. Then it examines two other
; matrix elements to determine the correct signs for the X and Z components,
; ensuring the spacecraft rotates around the intended axis direction.
;
; CODE-ALONG READERS: METHOD2 executes when |COF(Y)| > |COF(X)| and |COF(Z)|.
; First checks COFSKEW+2 (UY component from skew-symmetric MFI); if negative,
; complements entire COF vector. Then uses MFISYM+2 (encodes COF(X)*COF(Y)
; relationship) to determine COF(X) sign, and MFISYM+10D (encodes COF(Y)*COF(Z))
; to determine COF(Z) sign. Each off-diagonal element's sign indicates whether
; the corresponding axis components have matching or opposite signs.

METHOD2		DLOAD	BPL		# COFY MAX
			COFSKEW +2	# UY
			U2POS
		VLOAD	VCOMP
			COF
		STORE	COF
U2POS		DLOAD	BPL
			MFISYM 	+2	# UX UY
			OKU21
		DLOAD	DCOMP		# SIGN OF UX OPPOSITE garbled
			COF
		STORE	COF
OKU21		DLOAD	BPL
			MFISYM +10D	# UY UZ
			LOCSKIRT
		DLOAD	DCOMP		# SIGN OF UZ OPPOSITE TO UY
			COF 	+4
		STORE	COF 	+4
		GOTO
			LOCSKIRT
; METHOD1: Sign Determination When X-Axis Component is Largest
;
; COMMENT-ONLY READERS: The X-component of the rotation axis is the largest.
; Following the same logic as METHOD2, the computer checks if the X-component
; is positive or negative, reverses all signs if needed, then determines the
; correct signs for the Y and Z components. This ensures the spacecraft knows
; exactly which direction to rotate around the computed axis, avoiding a
; maneuver in the wrong direction that would waste propellant and time.
;
; CODE-ALONG READERS: METHOD1 executes when |COF(X)| > |COF(Y)| and |COF(Z)|.
; Checks COFSKEW (UX) for sign; complements COF if negative. Uses MFISYM+2
; (COF(X)*COF(Y) term) to determine COF(Y) sign, and MFISYM+4 (COF(X)*COF(Z))
; to determine COF(Z) sign. The logic parallels METHOD2 but with X as the
; reference axis. Note the original comment "SIGN OF UZ OPPOSITE TO UY" at
; OKU12 appears to be a transcription artifact; context indicates it should
; reference UX.

METHOD1		DLOAD	BPL		# COFX MAX
			COFSKEW		# UX
			U1POS
		VLOAD	VCOMP
			COF
		STORE	COF
U1POS		DLOAD	BPL
			MFISYM 	+2	# UX UY
			OKU12
		DLOAD	DCOMP
			COF 	+2	# SIGN OF UY OPPOSITE TO UX
		STORE	COF 	+2
OKU12		DLOAD	BPL
			MFISYM 	+4	# UX UZ
			LOCSKIRT
		DLOAD	DCOMP		# SIGN OF UZ OPPOSITE TO UY
			COF 	+4
# Page 355
		STORE	COF 	+4
		GOTO
			LOCSKIRT
; METHOD3: Sign Determination When Z-Axis Component is Largest
;
; COMMENT-ONLY READERS: The Z-component of the rotation axis is the largest.
; This is the third and final method for determining axis direction. Using
; the same technique as the previous two methods, the computer verifies the
; Z-component sign and adjusts the X and Y component signs to ensure they
; all work together to define the correct rotation axis. With all three
; components correctly signed, the spacecraft now has complete information
; about which direction to rotate for the most efficient maneuver.
;
; CODE-ALONG READERS: METHOD3 executes when |COF(Z)| > |COF(X)| and |COF(Y)|.
; Checks COFSKEW+4 (UZ) for sign; complements COF if negative. Uses MFISYM+4
; (COF(X)*COF(Z) relationship) to determine COF(X) sign, and MFISYM+10D
; (COF(Y)*COF(Z)) to determine COF(Y) sign. After this method completes, all
; three components of COF have correct signs and magnitude, fully defining
; the single equivalent rotation axis. Control flows to LOCSKIRT to continue
; maneuver setup.

METHOD3		DLOAD	BPL		# COFZ MAX
			COFSKEW +4	# UZ
			U3POS
		VLOAD	VCOMP
			COF
		STORE	COF
U3POS		DLOAD	BPL
			MFISYM 	+4	# UX UZ
			OKU31
		DLOAD	DCOMP
			COF		# SIGN OF UX OPPOSITE TO UZ
		STORE	COF
OKU31		DLOAD	BPL
			MFISYM 	+10D	# UY UZ
			LOCSKIRT
		DLOAD	DCOMP
			COF 	+2	# SIGN OF UY OPPOSITE TO UZ
		STORE	COF 	+2
		GOTO
			LOCSKIRT
# Page 356
; ============================================================================
; SECTION: MATRIX OPERATIONS
;
; This section contains fundamental 3x3 matrix operations used throughout
; the attitude maneuver calculations. These routines operate on direction
; cosine matrices (DCMs) that relate different coordinate frames during
; the spacecraft's rotation from initial to final attitude.
;
; The matrix operations are implemented using the AGC's interpretive language
; for vector/matrix operations, which provides efficient double-precision
; arithmetic at the cost of slower execution compared to native AGC code.
; For attitude maneuvers, precision is more critical than speed.
; ============================================================================
# MATRIX OPERATIONS

		BANK	13
		SETLOC	KALCMON2
		BANK

		EBANK=	BCDU
;
; MXM3 - 3x3 Matrix Multiply
;
; Multiplies two 3x3 matrices and stores the result in both the pushdown
; list and MPAC. This operation is essential for transforming direction
; cosine matrices between coordinate frames.
;
; COMMENT-ONLY READERS: This mathematical routine multiplies two 3x3 arrays
; of numbers (matrices) to combine coordinate system transformations. For
; example, to determine the spacecraft's rotation from its current orientation
; to its final orientation, the computer must multiply the matrices representing
; each orientation. The result tells the autopilot exactly how to rotate the
; spacecraft.
;
; CODE-ALONG READERS: This routine performs the standard 3x3 matrix multiply
; operation C = A × B using the interpretive language vector operations. Each
; row of the first matrix (address in X1) is multiplied by the entire second
; matrix (address in X2) using the VXM* (Vector times Matrix) instruction.
; The result matrix is pushed onto the pushdown list starting at location 0.
;
; Inputs:
;   X1: Address of first matrix (9 components: rows at 0, 6, 12D)
;   X2: Address of second matrix (9 components)
;
; Outputs:
;   PD List (0-17D): Result matrix C
;   MPAC: Final row of result matrix
;
; Computational Note: Each VXM* operation computes one row of the result
; by taking the dot product of that row with each column of the second matrix.
; The PUSH instruction stores the final result row and advances the pushdown
; pointer.
;
MXM3		SETPD	VLOAD*		# MXM3 MULTIPLIES 2 3X3 MATRICES
			0		# AND LEAVES RESULT IN PD LIST
			0,1		# AND MPAC
		VXM*	PDVL*		; First row: C[0] = A[0] × B
			0,2		; Multiply by second matrix
			6,1		; Load second row of first matrix
		VXM*	PDVL*		; Second row: C[1] = A[1] × B
			0,2
			12D,1		; Load third row of first matrix
		VXM*	PUSH		; Third row: C[2] = A[2] × B
			0,2
		RVQ			; Return with result in PD list

# RETURN WITH MIXM2 IN PD LIST
;
; TRANSPOS - 3x3 Matrix Transpose
;
; Transposes a 3x3 matrix by exchanging rows and columns. This operation
; is mathematically equivalent to finding the inverse of an orthonormal
; direction cosine matrix, which represents the reverse transformation.
;
; COMMENT-ONLY READERS: Matrix transpose swaps the rows and columns of the
; matrix. For the special matrices used in spacecraft attitude calculations
; (direction cosine matrices), transposing is equivalent to reversing the
; transformation. If matrix M rotates from frame A to frame B, then M-transpose
; rotates from frame B back to frame A. This is crucial for computing the
; spacecraft's path from its current position to its desired final attitude.
;
; CODE-ALONG READERS: The routine has two entry points:
;   - TRANSPOS: Loads matrix from address in X1 and transposes it
;   - TRNSPSPD: Transposes matrix already in pushdown list at location 0
;
; The transpose operation exchanges elements: result[i,j] = input[j,i]
; The interpretive portion (TRANSPOS entry) loads the three rows into the
; pushdown list. The native AGC portion (TRNSPSPD entry) uses indexed DXCH
; operations to swap the off-diagonal elements directly in memory:
;   - Swap elements (0,1) ↔ (1,0): positions 2 and 6
;   - Swap elements (0,2) ↔ (2,0): positions 4 and 12
;   - Swap elements (1,2) ↔ (2,1): positions 14 and 16
;
; The diagonal elements (0,0), (1,1), (2,2) remain unchanged.
;
; Matrix layout in memory (double-precision components):
;   Row 0: positions 0, 2, 4     (components [0,0], [0,1], [0,2])
;   Row 1: positions 6, 8, 10    (components [1,0], [1,1], [1,2])
;   Row 2: positions 12, 14, 16  (components [2,0], [2,1], [2,2])
;
; Inputs:
;   X1: Address of source matrix (if entering at TRANSPOS)
;   PD List location 0: Source matrix (if entering at TRNSPSPD)
;
; Outputs:
;   PD List (0-17D): Transposed matrix
;
TRANSPOS	SETPD	VLOAD*		# TRANSPOS TRANSPOSES A 3X3 MATRIX
			0		# 	AND LEAVES RESULT IN PD LIST
			0,1		# MATRIX ADDRESS IN XR1
		PDVL*	PDVL*		; Load all three rows into pushdown
			6,1
			12D,1
		PUSH			# MATRIX IN PD
TRNSPSPD	EXIT			# ENTER WITH MATRIX AT 0 IN PD LIST
		INDEX	FIXLOC		; Swap off-diagonal elements using
		DXCH	12		; indexed addressing. FIXLOC provides
		INDEX	FIXLOC		; base address for pushdown operations
		DXCH	16		; Swap positions 12 and 16: [2,0] ↔ [2,2]
		INDEX	FIXLOC
		DXCH	12		; Restore 12, now contains [2,2]
		INDEX	FIXLOC
		DXCH	14		; Swap 14 and accumulated value
		INDEX	FIXLOC
		DXCH	4		; Swap positions 4 and 14: [0,2] ↔ [1,2]
		INDEX	FIXLOC
		DXCH	14		; Restore 14
		INDEX	FIXLOC
		DXCH	2		; Begin swap of positions 2 and 6
		INDEX	FIXLOC
		DXCH	6		; Complete swap: [0,1] ↔ [1,0]
		INDEX	FIXLOC
		DXCH	2		; Restore position 2
# Page 357
		TC	INTPRET		; Return to interpretive mode
		RVQ			; Return to caller

		BANK	15
		SETLOC	KALCMON1
		BANK

		EBANK=	BCDU
;
; ANGLE THRESHOLD CONSTANTS
;
; These constants define the minimum and maximum rotation angles for
; different computational methods in the attitude maneuver routine.
;
; MINANG: Minimum angle threshold (0.25 degrees)
; If the rotation angle is less than this value, no complex maneuvering
; is necessary - the autopilot can simply set CDU desired equal to the
; final desired angles. This prevents computational overhead for trivial
; attitude corrections.
;
; MAXANG: Maximum angle threshold (170 degrees)  
; If the rotation angle exceeds this value, an alternate computational
; method using the symmetric part of the transformation matrix is employed
; to extract the rotation axis. Large rotations near 180 degrees require
; special handling to avoid numerical instabilities in the standard
; skew-symmetric method.
;
; Both values are stored as double-precision fractions of a revolution.
; AGC angular scaling: 1.0 = 360 degrees (one complete revolution)
;
MINANG		2DEC	0.00069375	; 0.25 degrees = 0.25/360 revolutions

MAXANG		2DEC	0.472222222	; 170 degrees = 170/360 revolutions

; ============================================================================
; GIMBAL LOCK AVOIDANCE CONSTANTS
;
; The Lunar Module's Inertial Measurement Unit (IMU) uses a three-gimbal
; system to maintain stable member orientation. When the middle gimbal angle
; approaches 90 degrees, the spacecraft enters "gimbal lock" - a condition
; where two gimbals align and the system loses one degree of freedom.
;
; These constants define the mathematical boundaries for gimbal lock avoidance.
; During attitude maneuvers, KALCMANU checks if the commanded maneuver would
; pass through the gimbal lock danger zone. If so, it recalculates the 
; maneuver path to "skim" around the lock zone with a safe buffer angle.
;
; COMMENT-ONLY READERS: Think of gimbal lock as the spacecraft getting 
; "stuck" in its ability to measure orientation. These numbers ensure the 
; computer steers the LM away from this dangerous condition during maneuvers.
;
; CODE-ALONG READERS: D=60° defines the gimbal lock threshold (middle gimbal
; angle). NGL=2° provides a buffer zone to prevent numerical instabilities
; near singularities. All trigonometric values are pre-computed for efficiency.
; ============================================================================

# GIMBAL LOCK CONSTANTS

# D = MGA CORRESPONDING TO GIMBAL LOCK = 60 DEGREES
#	NGL = BUFFER ANGLE (TO AVOID DIVISIONS BY ZERO) = 2 DEGREES

SD		2DEC	.433015		# = SIN(D)			$2

K3S1		2DEC	.86603		# = SIN(D)			$1

K4		2DEC	-.25		# = -COS(D)			$2

K4SQ		2DEC	.125		# = COS(D)COS(D)		$2

SNGLCD		2DEC	.008725		# = SIN(NGL)COS(D)		$2

CNGL		2DEC	.499695		# COS(NGL)			$2

LOCKANGL	DEC	.388889		# = 70 DEGREES

; ============================================================================
; READCDUK - Read Current CDU Angles (Interpretive Entry Point)
;
; This subroutine reads the spacecraft's current gimbal angles from the IMU's
; Coupling Data Units (CDUs) and loads them into the MPAC (Multi-Purpose
; Accumulator) for interpretive processing. The three CDU angles define the
; stable member's orientation relative to the spacecraft body.
;
; During attitude maneuvers, these angles are read frequently to track the
; current spacecraft orientation and compute the error between actual and
; desired attitude. For Apollo 11, this was critical during LM separation,
; descent orientation changes, and ascent rendezvous maneuvers.
;
; Returns: MPAC loaded with CDUX, CDUY, CDUZ in triple-precision format
;          Execution continues in interpretive mode
;
; CODE-ALONG READERS: Uses DCA (double-precision load) for efficiency and
; branches to TLOAD+6 to enter interpretive mode with CDU data loaded.
; ============================================================================

# INTERPRETIVE SUBROUTINE TO READ THE CDU ANGLES

READCDUK	CA	CDUZ		# LOAD T(MPAC) WITH CDU ANGLES
		TS	MPAC 	+2	; Store middle gimbal angle (Z-axis)
		EXTEND			; Enable extended instruction
		DCA	CDUX		# Load both outer and inner gimbal (X,Y)
		TCF	TLOAD 	+6	# Jump to interpretive loader, skip setup

; ============================================================================
; CDUTODCM - Convert CDU Angles to Direction Cosine Matrix
;
; This interpretive subroutine transforms the three CDU gimbal angles into
; a 3x3 direction cosine matrix (DCM) that fully describes the spacecraft's
; orientation. The DCM relates the stable member coordinate frame to the
; spacecraft body frame.
;
; The transformation involves computing sines and cosines of each gimbal angle
; and combining them through matrix multiplication to build the complete
; orientation matrix. This is fundamental to all attitude computations.
;
; During Apollo 11, this conversion was performed constantly during maneuvers
; to track how the LM's actual orientation compared to the desired orientation.
; The resulting matrix was used for navigation calculations, display updates,
; and autopilot commands.
;
; Input: MPAC contains three CDU angles (from READCDUK or equivalent)
; Output: 3x3 direction cosine matrix stored in pushdown list
;
; CODE-ALONG READERS: Uses interpretive opcodes for vector/matrix operations.
; The loop processes each gimbal angle, computing sin/cos pairs efficiently.
; ============================================================================

CDUTODCM	AXT,1	SSP		; Initialize index register 1 to 3
		OCT	3		; Loop will process 3 gimbal angles
			S1		; Set loop counter S1
		OCT	1		# SET XR1, S1, AND PD FOR LOOP
		STORE	7		; Store intermediate result at PD+7
		SETPD			; Set pushdown pointer to zero
			0		; Initialize pushdown list base
; 
; Main loop: For each of the three gimbal angles (PHI, THETA, PSI),
; compute sine and cosine values and store them on the pushdown stack.
; This builds the trigonometric components needed for the DCM transformation.
;
LOOPSIN		SLOAD*	RTB		; Load gimbal angle indexed by X1
			10D,1		; Indirect addressing (10D + X1 offset)
			CDULOGIC	; Convert CDU format to internal angle
# Page 358
		STORE	10D		# LOAD PD WITH 	0 SIN(PHI)
		SIN	PDDL		#		2 COS(PHI) - compute sine, push, load
			10D		#		4 SIN(THETA) - angle value
		COS	PUSH		#		6 COS(THETA) - compute cosine, push
		TIX,1	DLOAD		#		8 SIN(PSI) - loop decrement, next angle
			LOOPSIN		#		10 COS(PSI)
			6		; Load COS(THETA) to begin matrix computation
;
; ============================================================================
; DIRECTION COSINE MATRIX ELEMENT COMPUTATION
;
; With all six trigonometric values now on the pushdown stack, compute the
; nine elements of the 3x3 direction cosine matrix. Each element represents
; the projection of one stable member axis onto one spacecraft body axis.
;
; The matrix relates stable member coordinates (inertial reference) to
; spacecraft body coordinates, accounting for all three gimbal rotations.
;
; Stack contents at this point (PD offsets):
;   0: SIN(PHI)   2: COS(PHI)   4: SIN(THETA)   6: COS(THETA)
;   8: SIN(PSI)  10: COS(PSI)
;
; CODE-ALONG READERS: Standard Euler angle transformation mathematics.
; Products and sums build the rotation matrix row by row. SL1 scales results
; for AGC's fixed-point arithmetic (multiply by 2 for proper scaling).
; ============================================================================
;
		DMP	SL1		; C0: First element of first row
			10D		; COS(THETA) * COS(PSI)
		STORE	0,2		# C0 = COS(THETA)COS(PSI) - store to matrix
		DLOAD	DMP		; Begin C1 computation
			4		; SIN(THETA)
			0		; * SIN(PHI)
		PDDL	DMP		# (PD6 SIN(THETA)SIN(PHI)) - save intermediate
			6		; COS(THETA)
			8D		; * SIN(PSI)
		DMP	SL1		; Continue: COS(THETA)*SIN(PSI)*...
			2		; * COS(PHI)
		BDSU	SL1		; Subtract from intermediate (backward subtract)
			12D		; From saved SIN(THETA)*SIN(PHI)
		STORE	2,2		# C1=-COS(THETA)SIN(PSI)COS(PHI) + ... - store
		DLOAD	DMP		; Begin C2 computation
			2		; COS(PHI)
			4		; * SIN(THETA)
		PDDL	DMP		# (PD7 COS(PHI)SIN(THETA)) SCALED 4 - save
			6		; COS(THETA)
			8D		; * SIN(PSI)
		DMP	SL1		; Continue product
			0		; * SIN(PHI)
		DAD	SL1		; Add to intermediate
			14D		; Add saved COS(PHI)*SIN(THETA)
		STORE	4,2		# C2=COS(THETA)SIN(PSI)SIN(PHI) + ... - store
		DLOAD			; C3: Simple transfer of sine value
			8D		; SIN(PSI)
		STORE	6,2		# C3=SIN(PSI) - third element of first row
		DLOAD			; Begin second row of matrix
			10D		; COS(PSI)
		DMP	SL1		; Multiply and scale
			2		; * COS(PHI)
		STORE	8D,2		# C4=COS(PSI)COS(PHI) - second row, first column
		DLOAD	DMP		; C5 computation
			10D		; COS(PSI)
			0		; * SIN(PHI)
		DCOMP	SL1		; Negate and scale (double complement)
		STORE	10D,2		# C5=-COS(PSI)SIN(PHI) - second row, second column
		DLOAD	DMP		; C6 computation: last column of second row
			4		; SIN(THETA)
			10D		; * COS(PSI)
		DCOMP	SL1		; Negate and scale
		STORE	12D,2		# C6=-SIN(THETA)COS(PSI) - second row, third column
# Page 359
; 
; Third row of Direction Cosine Matrix: C7 and C8
; These elements transform the third spacecraft axis from body to inertial coordinates.
; 
		DLOAD			; Begin third row computation
		DMP	SL1		# (PUSH UP 7) - COS(PHI) operation
			8D		; SIN(THETA)
		PDDL	DMP		# (PD7 COS(PHI)SIN(THETA)SIN(PSI)) SCALE 4
			6		; SIN(PSI)
			0		; * SIN(PHI)
		DAD	SL1		# (PUSH UP 7) - Add the two terms
		STADR			# C7=COS(PHI)SIN(THETA)SIN(PSI)
		STORE	14D,2		# 	+COS(THETA)SIN(PHI) - third row, first column
		DLOAD			; C8 computation begins
		DMP	SL1		# (PUSH UP 6)
			8D		; SIN(THETA)
		PDDL	DMP		# (PD6 SIN(THETA)SIN(PHI)SIN(PSI)) SCALE 4
			6		; SIN(PSI)
			2		; * COS(PHI)
		DSU	SL1		# (PUSH UP 6) - Subtract the terms
		STADR			; Store address for C8
		STORE	16D,2		# C8=-SIN(THETA)SIN(PHI)SIN(PSI)
		RVQ			# +COS(THETA)COS(PHI) - third row, second column
					; Return with complete 3x3 Direction Cosine Matrix

# CALCULATION OF THE MATRIX DEL......
#
#	 *         *           __T           *
#	DEL = (IDMATRIX)COS(A)+UU (1-COS(A))+UX SIN(A)		SCALED 1
#	      _
#	WHERE U IS A UNIT VECTOR (DP SCALED 2) ALONG THE AXIS OF ROTATION.
#	A IS THE ANGLE OF ROTATION (DP SCALED 2)
#	                                    _
#	UPON ENTRY, THE STARTING ADDRESS OF U IS COF, AND A IS IN MPAC
;
; ============================================================================
; TRANSITION: From CDU angle conversion to rotation matrix computation
;
; With the current spacecraft orientation captured as a Direction Cosine Matrix,
; the program now computes the rotation matrix (DEL) that will transform the
; spacecraft from its initial orientation to the desired final orientation.
; This uses the Rodriguez rotation formula, which expresses any rotation as
; a combination of the identity matrix, the rotation axis outer product, and
; the skew-symmetric cross product matrix. This mathematical elegance enables
; efficient attitude maneuver computation for the RCS autopilot.
; ============================================================================
;
; DELCOMP - Compute DEL rotation matrix using Rodriguez formula
;
; This routine implements the classic Rodriguez rotation formula which computes
; a 3x3 rotation matrix from a unit rotation axis vector and rotation angle.
; The formula combines three matrix terms:
; 1) Identity matrix scaled by COS(A)
; 2) Outer product of axis vector scaled by (1-COS(A))
; 3) Skew-symmetric matrix scaled by SIN(A)
;
; Upon entry: MPAC contains rotation angle A (double precision, scaled 2)
;             COF contains rotation axis unit vector U (DP vector, scaled 2)
; Upon exit:  KEL contains 3x3 rotation matrix (scaled 1)

DELCOMP		SETPD	PUSH		# MPAC CONTAINS THE ANGLE A
 			0		; Initialize push-down stack pointer
		SIN	PDDL		# PD0 = SIN(A) - store sine for later use
		COS	PUSH		# PD2 = COS(A) - compute cosine
		SR2	PDDL		# PD2 = COS(A) - shift right 2 for scaling	$8
		BDSU	BOVB		; Compute (1 - COS(A)) term
			DPHALF		; Subtract from 0.5 double precision
			SIGNMPAC	; Branch on overflow
		PDDL			# PDA = 1-COS(A) - this term scales outer product

# COMPUTE THE DIAGONAL COMPONENTS OF DEL
;
; Diagonal elements of DEL matrix: KEL(0,0), KEL(1,1), KEL(2,2)
; Formula: UX*UX*(1-COS(A)) + COS(A) for each axis component
; These represent the rotation's effect along each principal axis.
; For small angles, diagonal elements approach 1 (no rotation).
; For 180-degree rotation, diagonal elements depend entirely on axis direction.
;
			COF		; Load UX - first component of rotation axis
		DSQ	DMP		; Square UX, then multiply by (1-COS(A))
			4		; PD4 contains (1-COS(A)) term
		DAD	SL3		; Add COS(A) from PD2, shift left 3 for proper scaling
			2		; PD2 contains COS(A)
		BOVB			; Branch on overflow to maintain precision
			SIGNMPAC	; Sign correction on overflow
# Page 360
		STODL	KEL		# UX UX(1-COS(A)) +COS(A)		$1
					; Store as KEL(0,0) - first diagonal element
			COF 	+2	; Load UY - second component of rotation axis
		DSQ	DMP		; Square UY, multiply by (1-COS(A))
			4		; Same (1-COS(A)) scaling term
		DAD	SL3		; Add COS(A), shift for scaling
			2		; COS(A) term
		BOVB			; Overflow protection
			SIGNMPAC	; Maintain sign on overflow
		STODL	KEL 	+8D	# UY UY(1-COS(A)) +COS(A)		$1
					; Store as KEL(1,1) - second diagonal element
			COF 	+4	; Load UZ - third component of rotation axis
		DSQ	DMP		; Square UZ, multiply by (1-COS(A))
			4		; Same (1-COS(A)) scaling
		DAD	SL3		; Add COS(A), shift for proper scale
			2		; COS(A) term
		BOVB			; Final overflow check
			SIGNMPAC	; Sign maintenance
		STORE	KEL 	+16D	# UZ UZ(1-COS(A)) +COS(A)		$1
					; Store as KEL(2,2) - third diagonal element
					; All three diagonal components now computed

# COMPUTE THE OFF DIAGONAL TERMS OF DEL
;
; Off-diagonal elements of DEL matrix: six antisymmetric pairs
; Formula: KEL(i,j) = Ui*Uj*(1-COS(A)) + Uk*SIN(A)
;          KEL(j,i) = Ui*Uj*(1-COS(A)) - Uk*SIN(A)
; where (i,j,k) form cyclic permutation of (x,y,z)
;
; These terms represent the coupling between different rotation axes.
; They determine how rotation about one axis affects the other axes.
; The antisymmetry (+ vs -) comes from the cross product term in Rodriguez formula.
;
; First pair: KEL(0,1) and KEL(1,0) - X-Y coupling terms
;
		DLOAD	DMP		; Load UX from COF
			COF		; First axis component
			COF	+2	; Multiply by UY (second axis component)
		DMP	SL1		; Multiply by (1-COS(A))
			4		; PD4 contains (1-COS(A))
		PDDL	DMP		# D6	 UX UY (1-COS A)		$4
			COF 	+4	; Load UZ (third axis component)
			0		; Multiply by SIN(A) from PD0
		PUSH	DAD		# D8 	UZ SIN A			$4
			6		; Add D6: UX*UY*(1-COS(A)) + UZ*SIN(A)
		SL2	BOVB		; Scale and check overflow
			SIGNMPAC	; Sign correction if overflow
		STODL	KEL 	+6	; Store as KEL(0,1) - X rotates into Y
		BDSU	SL2		; Subtract instead: UX*UY*(1-COS(A)) - UZ*SIN(A)
		BOVB			; Overflow check for opposite sign term
			SIGNMPAC	; Sign maintenance
		STODL	KEL 	+2	; Store as KEL(1,0) - Y rotates into X
;
; Second pair: KEL(0,2) and KEL(2,0) - X-Z coupling terms
;
			COF		; Load UX (first axis)
		DMP	DMP		; Multiply by UZ (third axis)
			COF 	+4	; Then by (1-COS(A))
			4		; PD4 contains (1-COS(A))
		SL1	PDDL		# D6 	UX UZ (1-COS A)			$4
			COF 	+2	; Load UY (second axis component)
		DMP	PUSH		# D8	UY SIN(A)
			0		; Multiply by SIN(A) from PD0
		DAD	SL2		; Add D6: UX*UZ*(1-COS(A)) + UY*SIN(A)
			6		; Stack reference
		BOVB			; Overflow check
			SIGNMPAC	; Sign adjustment if overflow
		STODL	KEL 	+4	# UX UZ (1-COS(A))+UY SIN(A)
# Page 361
		BDSU	SL2		; Subtract instead: UX*UZ*(1-COS(A)) - UY*SIN(A)
		BOVB			; Overflow check for opposite term
			SIGNMPAC	; Sign correction
		STODL	KEL	 +12D	# UX UZ (1-COS(A))-UY SIN(A)
;
; Third pair: KEL(1,2) and KEL(2,1) - Y-Z coupling terms
;
			COF 	+2	; Load UY (second axis)
		DMP	DMP		; Multiply by UZ (third axis)
			COF 	+4	; Then by (1-COS(A))
			4		; PD4 contains (1-COS(A))
		SL1	PDDL		# D6	UY UZ (1-COS(A))		$ 4
			COF		; Load UX (first axis component)
		DMP	PUSH		# D8	UX SIN(A)
			0		; Multiply by SIN(A) from PD0
		DAD	SL2		; Add D6: UY*UZ*(1-COS(A)) + UX*SIN(A)
			6		; Stack reference
		BOVB			; Overflow check
			SIGNMPAC	; Sign adjustment if overflow
		STODL	KEL 	+14D	# UY UZ(1-COS(A)) +UX SIN(A)
		BDSU	SL2		; Subtract instead: UY*UZ*(1-COS(A)) - UX*SIN(A)
		BOVB			; Overflow check for opposite term
			SIGNMPAC	; Sign correction
		STORE	KEL 	+10D	# UY UZ (1-COS(A)) -UX SIN(A)
		RVQ

# DIRECTION COSINE MATRIX TO CDU ANGLE ROUTINE
# X1 CONTAINS THE COMPLEMENT OF THE STARTING ADDRESS FOR MATRIX (SCALED 2).
# LEAVE CDU ANGLES SCALED 2PI IN V(MPAC).
# COS(MGA) WILL BE LEFT IN S1 (SCALED 1).
#
# THE DIRECTION COSINE MATRIX RELATING S/C AXES TO STABLE MEMBER AXES CAN BE WRITTEN AS:
#
#	C  = COS(THETA) COS(PSI
#	 0
#
#	C  = -COS(THETA) SIN(PSI) COS(PHI) + SIN(THETA) SIN(PHI)
#	 1
#
#	C  = COS(THETA) SIN(PSI) SIN(PHI) + SIN(THETA) COS(PHI)
#	 2
#
#	C  = SIN(PSI)
#	 3
#
#	C  = COS(PSI) COS(PHI)
#	 4
#
#	C  = -COS(PSI) SIN(PHI)
#	 5
#
#	C  = -SIN(THETA) COS(PSI)
#	 6
#
#	C  = SIN(THETA) SIN(PSI) COS(PHI) + COS (THETA) SIN(PHI)
#	 7
#
#	C  = -SIN(THETA) SIN(PSI) SIN(PHI) + COS(THETA)COS(PHI)
#	 8
# Page 362
#
# WHERE	PHI = OGA
#	THETA = IGA
#	PSI = MGA

;
; DCMTOCDU - Direction Cosine Matrix to CDU Angle Conversion
;
; This routine extracts gimbal angles (PHI, THETA, PSI) from a direction
; cosine matrix. It performs the inverse operation of CDUTODCM, solving
; the equations that relate the 3x3 rotation matrix to Euler angles.
;
; The extraction sequence handles gimbal angle ambiguities by checking
; the signs of cosine terms to determine the correct quadrant for each angle.
;
; Step 1: Extract PSI (middle gimbal angle) from matrix element M(1,2)
;
DCMTOCDU	DLOAD*	ARCSIN		; Load M(1,2) indexed by register 1
			6,1		; Matrix element (row 1, col 2)
		PUSH	COS		# PD +0 	PSI
;
; PSI = ARCSIN(M(1,2)) is now in MPAC
; Compute COS(PSI) to use as divisor for extracting THETA and PHI
;
		SL1	BOVB		; Scale left 1 bit, branch on overflow
			SIGNMPAC	; Sign adjustment if overflow
		STORE	S1		; Store COS(PSI) in S1 for later use
;
; Step 2: Extract THETA (inner gimbal angle) from matrix element M(2,2)
; THETA = ARCSIN(-M(2,2) / COS(PSI))
;
		DLOAD*	DCOMP		; Load M(2,2) and complement (negate)
			12D,1		; Matrix element (row 2, col 2)
		DDV	ARCSIN		; Divide by COS(PSI), then ARCSIN
			S1		; COS(PSI) is the divisor
		PDDL*	BPL		# PD +2		THETA
			0,1		# MUST CHECK THE SIGN OF COS(THETA)
			OKTHETA		# TO DETERMINE THE PROPER QUADRANT.
;
; Quadrant resolution for THETA: Check sign of M(0,0) = COS(THETA)
; If M(0,0) is negative, THETA is in 2nd or 3rd quadrant
;
		DLOAD	DCOMP		; Load and complement M(0,0)
		BPL	DAD		; Branch if positive (no adjustment needed)
			SUHALFA		; If negative, adjust THETA
			DPHALF		; Add 180 degrees (pi radians)
		GOTO
			CALCPHI		; Proceed to PHI calculation
SUHALFA		DSU			; Subtract instead of add
			DPHALF		; THETA = THETA - 180 degrees
CALCPHI		PUSH			; Push adjusted THETA to stack
;
; Step 3: Extract PHI (outer gimbal angle) from matrix element M(2,1)
; PHI = ARCSIN(-M(2,1) / COS(PSI))
;
OKTHETA		DLOAD*	DCOMP		; Load M(2,1) and complement (negate)
			10D,1		; Matrix element (row 2, col 1)
		DDV	ARCSIN		; Divide by COS(PSI), then ARCSIN
			S1		; COS(PSI) is the divisor
		PDDL*	BPL		# PUSH DOWN PHI
			8D,1		; Load M(1,1) to check sign
			OKPHI		; Branch if positive
;
; Quadrant resolution for PHI: Check sign of M(1,1) = COS(PHI)
; If M(1,1) is negative, PHI is in 2nd or 3rd quadrant
;
		DLOAD	DCOMP		# PUSH UP PHI
		BPL	DAD		; Branch if positive (no adjustment)
			SUHALFAP	; If negative, adjust PHI
			DPHALF		; Add 180 degrees
		GOTO
			VECOFANG	; Build vector of angles
SUHALFAP	DSU	GOTO		; Subtract instead of add
			DPHALF		; PHI = PHI - 180 degrees
			VECOFANG	; Build vector of angles
;
; Assembly the three gimbal angles into a vector and return
;
OKPHI		DLOAD			# PUSH UP PHI
VECOFANG	VDEF	RVQ		; Define vector (PHI, THETA, PSI) and return
# Page 363
; ============================================================================
; SECTION: MANEUVER TERMINATION ROUTINES
;
; This section provides error handling and graceful termination paths for
; the attitude maneuver routine. These routines handle cases where the 
; requested maneuver cannot be completed (gimbal lock conditions, invalid
; orientations) and ensure the spacecraft is left in a stable, controlled
; state with the autopilot properly configured.
; ============================================================================
# ROUTINES FOR TERMINATING THE AUTOMATIC MANEUVER AND RETURNING TO USER.
;
; TOOBADF - Termination routine for bad final orientation
;
; Called when the requested final attitude would place the spacecraft in
; or too close to gimbal lock. Issues alarm 00401 to alert the crew that
; the commanded attitude cannot be achieved safely. The maneuver is aborted
; and the spacecraft rates are stopped, leaving the vehicle in its current
; orientation rather than risking gimbal lock.
;
TOOBADF		EXIT
		TC	ALARM
		OCT	00401		; Alarm code: Final attitude in gimbal lock

		TCF	NOGO		# DO NOT ZERO ATTITUDE ERRORS
;
; Alternative path: Zero attitude errors before stopping rates
; (This code path appears unreachable due to TCF above)
;
		TC	BANKCALL
		CADR	ZATTEROR	# ZERO ATTITUDE ERRORS
;
; NOGO - Common termination path for aborted maneuvers
;
; This routine brings the spacecraft to a controlled stop when a maneuver
; must be terminated without reaching the desired final attitude. It stops
; all rotational rates using the RCS autopilot and schedules the GOODMANU
; routine to perform final cleanup and return control to the calling program.
;
; During Apollo 11's mission, this type of abort would have been rare, but
; the capability was essential for crew safety. If a maneuver command would
; have driven the IMU into gimbal lock, this routine ensured the spacecraft
; remained controllable.
;
NOGO		TC	BANKCALL
		CADR	STOPRATE	# STOP RATES
;
; Schedule final cleanup via WAITLIST after a 2-second delay.
; This gives the autopilot time to null out rates before final termination.
;
		CAF	TWO		; 2 seconds delay (2 x 100 centiseconds)
		INHINT			# ALL RETURNS ARE NOW MADE VIA GOODEND
		TC	WAITLIST	; Schedule task on waitlist
		EBANK=	BCDU
		2CADR	GOODMANU	; Task to execute: final cleanup

		TCF	ENDOFJOB	; Terminate current job
;
; TOOBADI - Termination routine for bad initial orientation
;
; Called when the spacecraft's current attitude is in or too close to gimbal
; lock. Since the IMU is already in a dangerous configuration, no maneuver
; can be safely initiated. The routine immediately branches to NOGO to stop
; rates and terminate gracefully.
;
; This condition would be extremely rare in normal operations, as the crew
; would have received gimbal lock warnings before reaching this state.
;
TOOBADI		EXIT
		TCF	NOGO		; Branch to common termination path


