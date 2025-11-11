# Copyright:	Public domain.
# Filename:	THROTTLE_CONTROL_ROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	HARTMUTH GUTSCHE <hgutsche@xplornet.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	793-797
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
; FILE: THROTTLE_CONTROL_ROUTINES.agc
; MODULE: Descent Propulsion System (DPS) Throttle Control
; MISSION PHASE: descent/landing
;
; TL;DR: Manages Descent Propulsion System (DPS) engine throttle commands
;        during powered descent to the lunar surface. Controls throttle
;        range from 10% (minimum) to 94% (maximum hardware stop), with
;        switching logic at 55% threshold. Implements engine response lag
;        compensation, thrust command smoothing, and interfaces with landing
;        guidance equations for desired thrust and crew displays for fuel
;        quantity monitoring. Critical for fuel-optimal descent trajectory
;        execution during Apollo 11's final 12-minute descent to landing.
;
; COMMENT-ONLY READERS: This code controlled the descent engine throttle
;        as Armstrong and Aldrin descended to the lunar surface. Follow the
;        comments to understand how the computer managed engine power while
;        monitoring fuel consumption during those critical final minutes.
; CODE-ALONG READERS: Study the throttle command computation (PIF variable),
;        engine response lag compensation (FWEIGHT calculation), and mass-
;        weighted thrust scaling to understand closed-loop engine control.
; ============================================================================

# Page 793
		BANK	31
		SETLOC	FTHROT
		BANK
		EBANK=	PIF
		COUNT*	$$/THROT

; ============================================================================
; THROTTLE CONTROL MAIN ENTRY POINT
;
; The lunar module is descending toward the surface under powered flight.
; The descent engine must be continuously adjusted to follow the guidance
; trajectory computed by LUNAR_LANDING_GUIDANCE_EQUATIONS.agc. This routine
; is called repeatedly during descent to compute the throttle command that
; will be sent to the DPS engine hardware.
;
; During Apollo 11's descent on July 20, 1969, this code executed hundreds
; of times during the 12-minute powered descent, managing engine thrust as
; Armstrong and Aldrin descended from 50,000 feet to touchdown at the Sea
; of Tranquility.
; ============================================================================

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# HERE FC, DESIRED THRUST, AND FP, PRESENT THRUST, UNWEIGHTED, ARE COMPUTED.

; THRUST COMPUTATION SECTION
; This section computes two key values:
; FC = Desired thrust commanded by guidance equations (from /AFC/)
; FP = Present thrust being produced by engine (from current acceleration)
;
; Both values are "unweighted" meaning they represent raw force before
; accounting for propellant mass depletion. The FWEIGHT compensation 
; applied later adjusts for the fact that as propellant burns, the same
; thrust produces more acceleration on the lighter vehicle.

THROTTLE	CA	ABDELV		# COMPUTE PRESENT ACCELERATION IN UNITS OF
		EXTEND			# 2(-4) M/CS/CS, SAVING SERVICER TROUBLE
		MP	/AF/CNST
 +3		EXTEND
 		QXCH	RTNHOLD
AFDUMP		TC	MASSMULT
		DXCH	FP		# FP = PRESENT THRUST
		EXTEND
		DCA	/AFC/
		TC	MASSMULT
		TS	FC		# FC = THRUST DESIRED BY GUIDANCE
		DXCH	FCODD		# FCODD = WHAT IT IS GOING TO GET

; ENGINE RESPONSE LAG COMPENSATION
; The DPS engine does not respond instantaneously to throttle commands.
; There is mechanical lag in the engine control valve and combustion 
; stabilization time. If less than 3 seconds have elapsed since the last
; throttle adjustment, we use the previously computed FWEIGHT compensation
; to augment the present thrust estimate, accounting for commands still
; "in flight" through the engine control system.
;
; This compensation is critical for smooth engine control without
; oscillations. Without it, the guidance computer would "overshoot" thrust
; commands trying to correct for lag that hasn't yet manifested.

# IF IT HAS BEEN LESS THAN 3 SECONDS SINCE THE LAST THROTTLING, AUGMENT FP USING THE FWEIGHT CALCULATED THEN.

		CS	TTHROT		# THIS CODING ASSUMES A FLATOUT WITHIN
		AD	TIME1		#   80 SECONDS BEFORE FIRST THROTTLE CALL
		MASK	POSMAX
		COM
		AD	3SECS
		EXTEND
		BZMF	WHERETO		# BRANCH IF (TIME1-TTHROT +1) > 3 SECONDS
		EXTEND
		DCA	FWEIGHT
		DAS	FP

; ============================================================================
; TRANSITION: From thrust computation to throttle range determination
;
; The computer has now calculated both the thrust the guidance system wants
; (FC) and the thrust the engine is currently producing (FP). Next, it must
; decide what throttle percentage to command to the DPS engine hardware.
;
; The DPS engine has physical throttle constraints:
; - Minimum: 10% thrust (below this, combustion becomes unstable)
; - Maximum: 94% thrust (hardware stop in throttle mechanism)
; - Critical transition: 55% threshold where control logic changes
;
; The throttle range 10%-60% was used during initial braking phase when
; full thrust wasn't needed. The range 60%-94% was used during approach
; and landing when maximum available thrust was required to arrest descent.
; ============================================================================

# THIS LOGIC DETERMINES THE THROTTLING IN THE REGION 10% - 94%.  THE MANUAL THROTTLE, NOMINALLY SET AT
# MINIMUM BY ASTRONAUT OR MISSION CONTROL PROGRAMS, PROVIDES THE LOWER BOUND.  A STOP IN THE THROTTLE HARDWARE
# PROVIDES THE UPPER.

; THROTTLE RANGE INITIALIZATION
; Load the low and high critical thrust thresholds (LOWCRIT and HIGHCRIT)
; from pad-loaded erasable memory. These values define the boundaries
; between different throttle control modes during descent.

WHERETO		CA	EBANK5		# INITIALIZE L*WCR*T AND H*GHCR*T FROM
		TS	EBANK		#   PAD LOADED ERASABLES IN W-MATRIX
# Page 794
		EBANK=	LOWCRIT
		EXTEND
		DCA	LOWCRIT
		DXCH	L*WCR*T
		CA	EBANK7
		TS	EBANK
		EBANK=	PIF
		CS	ZERO		# INITIALIZE PIFPSET
		TS	PIFPSET

; THROTTLE DECISION TREE
; The following logic determines which throttle mode to use based on
; comparing desired thrust (FC or FCODD) against threshold values:
;
; Path 1: If previous FC was high and current FC is low -> FCOMPSET mode
; Path 2: If previous FC was low and current FC is still low -> DOPIF mode  
; Path 3: If previous FC was low and current FC is high -> Throttle-up mode
;
; These paths handle the transition between descent phases, such as moving
; from high-thrust braking to lower-thrust approach phase.

		CS	H*GHCR*T
		AD	FCOLD
		EXTEND
		BZMF	LOWFCOLD	# BRANCH IF FCOLD < OR = HIGHCRIT
		CS	L*WCR*T
		AD	FCODD
		EXTEND
		BZMF	FCOMPSET	# BRANCH IF FC < OR = LOWCRIT
		CA	FP		# SEE NOTE 1
		TCF	FLATOUT1

; FCOMPSET MODE: Guidance wants less thrust than the low threshold
; In this mode, throttle is determined by comparing present thrust (FP)
; against the maximum throttle limit. This typically occurs during the
; approach phase when fine throttle control is needed.

FCOMPSET	CS	FMAXODD		# SEE NOTE 2
		AD	FP
		TCF	FLATOUT2

; LOWFCOLD MODE: Previous thrust command was below high threshold
; Check if current desired thrust is also below threshold. If so, use
; normal throttle computation (DOPIF). If not, throttle up to maximum.

LOWFCOLD	CS	H*GHCR*T
		AD	FCODD
		EXTEND
		BZMF	DOPIF		# BRANCH IF FC < OR = HIGHCRIT

		CA	FMAXPOS		# NO:  THROTTLE-UP
FLATOUT1	DXCH	FCODD
		CA	FEXTRA
FLATOUT2	TS	PIFPSET

# NOTE 1	FC IS SET EQUAL TO FP SO PIF WILL BE ZERO.  THIS IS DESIRABLE
#		AS THERE IS ACTUALLY NO THROTTLE CHANGE.
#
# NOTE 2	HERE, SINCE WE ARE ABOUT TO RETURN TO THE THROTTLEABLE REGION
#		(BELOW 55%) THE QUANTITY -(FMAXODD - FP) IS COMPUTED AND PUT
#		INTO PIFPSET TO COMPENSATE FOR THE DIFFERENCE BETWEEN THE
#		NUMBER OF BITS CORRESPONDING TO FULL THROTTLE (FMAXODD) AND THE
#		NUMBER CORRESPONDING TO ACTUAL THRUST (FP).  THUS THE TOTAL
#		THROTTLE COMMAND PIF = FC - FP - (FMAXODD - FP) = FC - FMAXODD.

; ============================================================================
; DOPIF - CORE THROTTLE COMMAND COMPUTATION
;
; This is the normal operating mode for DPS throttle control during powered
; descent. PIF (Proportional plus Integral Feedback) is calculated as:
;
;     PIF = FC - FP    (Desired Thrust - Present Thrust)
;
; COMMENT-ONLY READERS: During Apollo 11's descent on July 20, 1969, this
; routine ran continuously, adjusting the engine thrust dozens of times per
; second to follow Armstrong and Aldrin's descent path. The throttle setting
; was displayed to the crew and transmitted to Mission Control. When fuel
; warnings began in the final minute, this code continued smooth throttle
; management right through touchdown.
;
; CODE-ALONG READERS: PIF represents throttle command in units that the DPS
; engine hardware interprets. Positive PIF increases thrust, negative 
; decreases. The FASTCHNG call protects this computation from restart
; interrupts. FCOLD saves the current commanded thrust for use in the next
; throttle cycle's lag compensation.
; ============================================================================

DOPIF		TC	FASTCHNG
		EXTEND
		DCA	FCODD
		TS	FCOLD
		DXCH	PIF
		EXTEND
# Page 795
		DCS	FP
		DAS	PIF		# PIF = FC - FP, NEVER EQUALS +0

; DOIT - APPLY THROTTLE COMMAND TO ENGINE HARDWARE
; The computed throttle command (PIF + PIFPSET offset) is sent to the
; DPS engine controller through I/O channel 14. This is the moment when
; the computer's calculations become physical thrust changes.
;
; THRUST variable: Actual command sent to engine (in throttle units)
; PSEUDO55: Working copy of throttle command
; CHAN14 bit 4: Throttle data strobe signal to engine controller
; TTHROT: Timestamp of this throttle command for lag compensation next cycle

DOIT		CA	PIF
		AD	PIFPSET		# ADD IN PIFPSET, WITHOUT CHANGING PIF
		TS	PSEUDO55
		TS	THRUST
		CAF	BIT4
		EXTEND
		WOR	CHAN14
		CA	TIME1
		TS	TTHROT

# SINCE /AF/ IS NOT AN INSTANTANEOUS ACCELERATION, BUT RATHER AN "AVERAGE" OF THE ACCELERATION LEVELS DURING
# THE PRECEEDING PIPA INTERVAL, AND SINCE FP IS COMPUTED DIRECTLY FROM /AF/, FP IN ORDER TO CORRESPOND TO THE
# ACTUAL THRUST LEVEL AT THE END OF THE INTERVAL MUST BE WEIGHTED BY
#
# 	          PIF(PPROCESS + TL)     PIF /PIF/
#	FWEIGHT = ------------------ + -------------
#		       PGUID           2 PGUID FRATE
#
# WHERE PPROCESS IS THE TIME BETWEEN PIPA READING AND THE START OF THROTTLING, PGUID IS THE GUIDANCE PERIOD, AND
# FRATE IS THE THROTTLING RATE (32 UNITS PER CENTISECOND).  PGUID IS EITHER 1 OR 2 SECONDS.  THE "TL" IN THE
# FIRST TERM REPRESENTS THE ENGINE'S RESPONSE LAG.  HERE FWEIGHT IS COMPUTED FOR USE NEXT PASS.

; ============================================================================
; FWEIGHT COMPUTATION - ENGINE RESPONSE LAG COMPENSATION
;
; The DPS engine doesn't respond instantly to throttle commands. There is
; physical lag as propellant valves open/close and combustion chamber
; pressure changes. Additionally, the accelerometers (PIPAs) measure average
; acceleration over a sampling interval, not instantaneous values.
;
; FWEIGHT compensates for these timing effects by predicting what the thrust
; will be at the END of the current guidance cycle, accounting for:
; 1. Processing delay between PIPA reading and throttle command
; 2. Engine response lag (TL parameter)  
; 3. Throttle rate of change (32 units per centisecond)
;
; This compensation was critical during descent. Without it, the guidance
; would "chase" the engine response, creating oscillations in thrust that
; could waste fuel or destabilize the landing trajectory.
;
; The computation differs between P63/P64 (2-second guidance cycle) and
; P66 (1-second guidance cycle for manual mode).
; ============================================================================

		CA	THISTPIP +1		# INITIALIZE FWEIGHT COMP AS IF FOR P66
		TS	BUF

		CS	MODREG			# ARE WE IN FACT IN P66?
		AD	DEC66
		EXTEND
		BZF	FWCOMP			# YES

; P63/P64 branch: For automated descent programs, use 2-second guidance period
; PIPTIME+1 holds time of last PIPA reading
; 4SECS constant used for scaling in 2-second guidance cycle
		CA	PIPTIME +1		# NO:  INITIALIZE FOR TWO SECOND PERIOD
		TS	BUF
		CAF	4SECS
		TCF	FWCOMP +1

; P66 branch: For manual throttle mode, use 1-second guidance period  
; 2SECS constant used for scaling in 1-second guidance cycle
FWCOMP		CAF	2SECS
 +1		TS	Q
 		EXTEND
		MP	BIT6
		LXCH	BUF +1
; Calculate processing time: (Current Time - Last PIPA Time + Engine Lag)
; This accounts for all delays between acceleration measurement and throttle
; command execution. THROTLAG is the DPS engine's physical response time.
		CS	BUF		# TIME OF LAST PIPA READING.
		AD	TIME1
		AD	THROTLAG	# COMPENSATE FOR ENGINE RESPONSE LAG
		MASK	LOW8		# MAKE SURE SMALL AND POSITIVE
		ZL
		EXTEND
# Page 796
; First term of FWEIGHT: (Processing Time + Lag) * PIF / Guidance Period
; This predicts the thrust change during the delay interval
		DV	Q
		EXTEND
		MP	PIF
		DOUBLE
		DXCH	FWEIGHT
; Second term of FWEIGHT: PIF * |PIF| / (2 * Guidance Period * Throttle Rate)
; CCS instruction tests PIF sign and adds ONE to get absolute value.
; This accounts for throttle command rate of change during the guidance cycle.
; The division by BUF+1 (guidance period scaled) and implicit FRATE scaling
; converts from command rate to actual thrust change prediction.
		CCS	PIF
		AD	ONE
		TCF	+2
		AD	ONE
		EXTEND
		MP	PIF
		EXTEND
		DV	BUF +1
		ZL
		DAS	FWEIGHT

; THDUMP - Return to caller with FWEIGHT computed
; FWEIGHT is now ready for the NEXT throttle cycle. When throttle is called
; again in 1-2 seconds, FP will be augmented with this FWEIGHT to compensate
; for all timing lags, producing smooth engine response without oscillations.
THDUMP		TC	RTNHOLD

# FLATOUT THROTTLES UP THE DESCENT ENGINE, AND IS CALLED AS A BASIC SUBROUTINE.

; ============================================================================
; TRANSITION: From throttle computation to maximum thrust command
;
; When guidance demands thrust exceeding throttle range or system enters
; emergency mode, FLATOUT commands full engine power (94% throttle limit).
; During Apollo 11 descent, this routine was not invoked as throttle stayed
; within normal operating range throughout the landing sequence.
; ============================================================================

; FLATOUT - Command maximum descent engine thrust
; Entry: Invoked when thrust demand exceeds controllable range
; This sets throttle to full-on (4096 pulses = 94% mechanical limit)
; PIFPSET cleared to disable FWEIGHT augmentation during max thrust
FLATOUT		CAF	BIT13		# 4096 PULSES
WHATOUT		TS	PIFPSET		# USE PIFPSET SO FWEIGHT WILL BE ZERO
		CS	ZERO
		TS	FCOLD
		TS	PIF
		EXTEND
		QXCH	RTNHOLD
		TCF	DOIT

# MASSMULT SCALES ACCELERATION, ARRIVING IN A AND L IN UNITS OF 2(-4) M/CS/CS, TO FORCE IN PULSE UNITS.

; MASSMULT - Convert acceleration to thrust force in engine command units
; Entry: A,L contain acceleration in units of 2^-4 m/cs² 
; Exit: A,L contain force in throttle pulse units (scaled for engine commands)
; 
; This critical subroutine implements Newton's second law: F = ma
; MASS is the current LM mass (decreasing as fuel burns during descent)
; SCALEFAC converts physical units to AGC throttle pulse commands
; Used to convert both desired thrust (FC) and present thrust (FP)
;
; During Apollo 11 landing, LM mass decreased from ~15,000 kg at PDI to
; ~10,500 kg at touchdown as approximately 8,000 lbs of propellant burned.
MASSMULT	EXTEND
		QXCH	BUF
		DXCH	MPAC
		TC	DMP
		ADRES	MASS
		TC	DMP		# LEAVES PROPERLY SCALED FORCE IN MPAC
		ADRES	SCALEFAC
		TC	TPAGREE
		CA	MPAC
		EXTEND
		BZF	+3
		CAF	POSMAX
		TC	BUF
		DXCH	MPAC +1
		TC	BUF
# Page 797
# CONSTANTS:-

; FEXTRA - Maximum thrust increment (alias for BIT13 = 4096 pulses)
; Value: +5.13309020E+4 Newtons (approximately 11,540 lbs thrust)
; Used for full throttle-up commands when guidance demands maximum thrust
FEXTRA		=	BIT13		# FEXT +5.13309020E+ 4

; /AF/CNST - Acceleration-to-force conversion constant  
; Value: 0.13107 (decimal)
; Scaling factor used in MASSMULT to convert desired acceleration from
; guidance equations into force commands. Works with MASS and SCALEFAC to
; produce proper throttle pulse units for engine command system.
/AF/CNST	DEC	.13107

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
