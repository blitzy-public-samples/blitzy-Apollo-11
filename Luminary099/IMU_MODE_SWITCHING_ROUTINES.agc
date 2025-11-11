# Copyright:	Public domain.
# Filename:	IMU_MODE_SWITCHING_ROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Onno Hommes <ohommes@cmu.edu>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1309-1337
# Mod history:	2009-05-28 OH	Transcribed from page images.
#		2009-06-05 RSB	Fixed a typo.
#		2011-01-06 JL	Fixed pseudo-label indentation.
#		2011-05-08 JL	Removed workaround. Flagged SBANK=
#				workaround for future removal.

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
; FILE: IMU_MODE_SWITCHING_ROUTINES.agc
; MODULE: Navigation Sensors and IMU Control
; MISSION PHASE: All phases (launch through landing and ascent)
;
; TL;DR: Controls the Inertial Measurement Unit (IMU) through its operational
;        modes including coarse alignment, fine alignment, and gyrocompass
;        modes. Manages critical transitions between IMU states, handles PIPA
;        (accelerometer) activation and release, and controls gyro torquing
;        for platform alignment corrections during all mission phases.
;
; COMMENT-ONLY READERS: This module controls the IMU "brain" that tells the
;        spacecraft where it is and how it's oriented in space. Read to
;        understand how the computer aligns and maintains the navigation
;        platform throughout the mission.
; CODE-ALONG READERS: Study the state machine logic for IMU mode transitions,
;        PIPA management, and the low-level gyro torquing protocols that
;        maintain platform alignment accuracy.
; ============================================================================


# Page 1309
		BLOCK	02
		SETLOC	FFTAG3
		BANK

		EBANK=	COMMAND

; ============================================================================
; SECTION: IMU CDU ZEROING UTILITIES
;
; The Inertial Measurement Unit uses three Coupling Data Units (CDUs) to
; measure gimbal angles in three axes: X, Y, and Z. These routines initialize
; the CDU counters to zero as part of IMU alignment procedures.
; ============================================================================

# FIXED-FIXED ROUTINES

		COUNT*	$$/IMODE

; ZEROICDU - Zero all three IMU gimbal angle counters
; This routine clears the three CDU counters (CDUX, CDUY, CDUZ) which track
; the IMU gimbal angles. Called during IMU alignment procedures to establish
; a known reference position before applying commanded gimbal angles.
;
; Inputs: None
; Outputs: CDUX, CDUY, CDUZ all set to zero
; Returns to caller via Q register

ZEROICDU	CAF	ZERO		# ZERO ICDU COUNTERS.
		TS	CDUX		# Clear X-axis gimbal angle counter
		TS	CDUY		# Clear Y-axis gimbal angle counter
		TS	CDUZ		# Clear Z-axis gimbal angle counter
		TC	Q		# Return to caller

; SPSCODE defines the bit pattern used to indicate special processing modes
SPSCODE		=	BIT9

# Page 1310

; ============================================================================
; SECTION: IMU ZEROING WITH GIMBAL LOCK PROTECTION
;
; The IMU zeroing process resets gimbal angle counters during alignment, but
; must be protected against gimbal lock conditions where the gimbals can run
; away uncontrollably. This section implements safety checks before zeroing.
; ============================================================================

# IMU ZEROING ROUTINES

		BANK	11
		SETLOC	MODESW
		BANK

		COUNT*	$$/IMODE

; IMUZERO - Safe IMU CDU zeroing with gimbal lock protection
; This is the primary entry point for zeroing the IMU Coupling Data Unit
; counters. It checks for dangerous gimbal lock conditions before proceeding.
; If the IMU is in gimbal lock during coarse alignment, the routine aborts
; with an alarm to prevent gimbal runaway which could damage the platform.
;
; The crew initiates this through alignment programs (P51-P53) which use this
; routine to establish known gimbal angles before fine alignment.

IMUZERO		INHINT			# ROUTINE TO ZERO ICDUS.
		CS	DSPTAB +11D	# DON'T ZERO CDUS IF IMU IN GIMBAL LOCK AND
		MASK	BITS4&6		# COARSE ALIGN (GIMBAL RUNAWAY PROTECTION)
		CCS	A		# Check if dangerous condition exists
		TCF	IMUZEROA	# Safe to proceed

; Gimbal lock detected during coarse alignment - this is a dangerous condition
; where zeroing the CDUs could cause the gimbals to run away uncontrollably.
		TC	ALARM		# Alert crew of unsafe condition
		OCT	00206		# Alarm code 00206: IMU zeroing unsafe

		TCF	CAGETSTJ +4	# IMMEDIATE FAILURE - abort operation

; Safe path - no gimbal lock threat detected
IMUZEROA	TC	CAGETSTJ	# Proceed with IMU cage test and setup

; Disable Digital AutoPilot (DAP) automatic control modes to prevent
; interference while the IMU is being realigned. The DAP must not attempt
; attitude control while gimbal angles are being reset.
		CS	IMODES33	# DISABLE DAP AUTO AND HOLD MODES
		MASK	SUPER011	# BIT5 FOR GROUND
		ADS	IMODES33	# Update mode word

; Inhibit IMU and CDU failure detection during the zeroing operation.
; We're intentionally changing CDU values, which would otherwise trigger
; failure alarms designed to detect uncommanded gimbal motion.
		CS	IMODES30	# INHIBIT ICDUFAIL AND IMUFAIL (IN CASE WE
		MASK	BITS3&4		# JUST CAME OUT OF COARSE ALIGN).
		ADS	IMODES30	# Temporarily disable failure monitoring

; Send zero encode command to IMU with coarse alignment and error counter
; disabled. This prepares the IMU hardware to accept the CDU counter reset.
		CS	BITS4&6		# SEND ZERO ENCODE WITH COARSE AND ERROR
		EXTEND			# COUNTER DISABLED.
		WAND 	CHAN12		# Write command to IMU control channel

; Turn off the "NO ATT" (No Attitude) lamp on the DSKY since we are about
; to establish a valid attitude reference through IMU alignment.
		TC	NOATTOFF	# TURN OFF NO ATT LAMP.

; Enable zero encode mode in IMU
		CAF	BIT5		# Zero encode enable bit
		EXTEND
		WOR	CHAN12		# Set bit in IMU control channel

; Zero the three CDU counters (CDUX, CDUY, CDUZ)
		TC	ZEROICDU	# Clear gimbal angle counters

; Schedule completion task after delay to allow AGS to receive pulse train.
; The AGS (Abort Guidance System) needs time to process the IMU zeroing
; pulse train. The 320ms delay (BIT6 = 64 x 5ms) ensures synchronization.
		CAF	BIT6		# WAIT 320 MS TO GIVE AGS ADEQUATE TIME TO
		TC	WAITLIST	# RECEIVE ITS PULSE TRAIN.
		EBANK=	CDUIND		# Set erasable bank for scheduled task
		2CADR	IMUZERO2	# Schedule IMUZERO2 task after delay

; Verify IMU is operating before exiting. If IMU is not powered on and
; operational (BIT9 clear in IMODES30), we issue an alarm since we cannot
; proceed with alignment without a functioning IMU.
		CS	IMODES30	# SEE IF IMU OPERATING AND ALARM IF NOT.
		MASK	BIT9		# Check IMU operating status bit
		CCS	A		# Test if IMU is powered and ready
		TCF	MODEEXIT	# IMU operating - normal exit
# Page 1311
; IMU is not operating - cannot complete zeroing operation
		TC	ALARM		# Alert crew of IMU failure
		OCT	210		# Alarm code 00210: IMU not operating

; MODEEXIT - General IMU mode switching exit routine
; This is the common exit path for all IMU mode-switching operations.
; Re-enables interrupts and returns control to the calling program.
MODEEXIT	RELINT			# GENERAL MODE-SWITCHING EXIT.
		TCF	SWRETURN	# Return to mode switch caller

; IMUZERO2 - Second phase of IMU zeroing (waitlist task)
; This routine executes after the 320ms delay scheduled in IMUZERO. It
; completes the CDU zeroing process and waits for the gimbals to settle
; into their zero positions before re-enabling normal IMU operations.
;
; This is called as a waitlist task, not directly by programs.

IMUZERO2	TC	CAGETEST	# Verify IMU cage status is acceptable
		TC	ZEROICDU	# ZERO CDUX, CDUY, CDUZ again

; Remove the zero discrete signal from the IMU, ending the zero encode mode.
; The IMU can now resume normal operation with the newly zeroed CDU counters.
		CS	BIT5		# REMOVE ZERO DISCRETE.
		EXTEND			# Extended instruction follows
		WAND	CHAN12		# Clear zero encode bit in IMU channel

; Allow 10 seconds for the gimbal resolvers to find and lock onto the
; zero position. The physical gimbals need time to settle after the
; counters are zeroed. This delay prevents premature attitude computations
; while the hardware stabilizes.
		CAF	BIT11		# WAIT 10 SECS FOR CTRS TO FIND GIMBALS
		TC	VARDELAY	# Variable delay (10 seconds)

; IMUZERO3 - Final phase of IMU zeroing
; Re-enables IMU failure monitoring and DAP control modes after the gimbals
; have settled into their zero positions. The IMU is now ready for normal
; navigation operations or fine alignment procedures.

IMUZERO3	TC	CAGETEST	# Final cage status verification
		CS	BITS3&4		# REMOVE IMUFAIL AND ICDUFAIL INHIBIT.
		MASK	IMODES30	# Clear inhibit bits
		TS	IMODES30	# Re-enable failure detection

; Re-enable Digital AutoPilot automatic and hold modes now that the IMU
; has been successfully zeroed and the gimbals have stabilized.
		CS	SUPER011	# ENABLE DAP AUTO AND HOLD MODES
		MASK	IMODES33	# BIT5 FOR GROUND
		TS	IMODES33	# Update mode word

		TC	IBNKCALL	# SET ISS WARNING IF EITHER OF ABOVE ARE
		CADR	SETISSW		# PRESENT.

		TCF	ENDIMU

# Page 1312
; ============================================================================
; TRANSITION: From IMU Zeroing to Coarse Alignment
;
; With the IMU CDU counters zeroed, the system is ready for coarse alignment.
; Coarse align drives the gimbals to approximate desired angles (within 2°)
; using error counter pulses. This is the first stage of IMU alignment,
; followed by fine align for precise attitude determination.
; ============================================================================

; IMU COARSE ALIGN MODE
; 
; Coarse alignment drives the IMU gimbals to within approximately 2 degrees
; of the desired orientation angles (THETAD). This mode uses the CDU error
; counters to send pulse trains that torque the gimbals. Coarse align is
; used during initial IMU setup or after IMU cage operations.
;
; During Apollo 11's mission, coarse alignment was performed:
; - After IMU power-up and initialization
; - Following crew star sightings for platform orientation
; - Before critical maneuvers requiring precise attitude reference
;
; The coarse align process cannot provide the precision needed for navigation
; but quickly establishes approximate gimbal angles, after which fine align
; achieves the required accuracy for guidance computations.

IMUCOARS	INHINT			# Disable interrupts during mode setup
		TC	CAGETSTJ	# Verify IMU is not caged
		TC	SETCOARS	# Initialize coarse align mode

; Schedule the coarse alignment task to begin after a brief delay.
; The 30ms delay (SIX = 6 x 5ms) allows IMU hardware to stabilize in
; coarse align mode before the first pulse commands are sent.
		CAF	SIX		# 30 millisecond delay
		TC	WAITLIST	# Schedule waitlist task
		EBANK=	CDUIND		# Set erasable bank
		2CADR	COARS		# Address of coarse align task

		TCF	MODEEXIT	# Return to caller

; COARS - Coarse Alignment Task (Waitlist Task)
; This is the main coarse alignment routine that executes as a scheduled
; waitlist task. It computes the angular error between desired gimbal angles
; (THETAD) and current angles (CDUX/Y/Z), then sends pulse commands to drive
; the gimbals toward the desired orientation.
;
; The routine processes all three axes (X, Y, Z) in sequence, calculating
; required error pulses and sending them to the IMU hardware through the
; CDU error counters.

COARS		TC	CAGETEST	# Verify IMU not caged before proceeding
		CAF	BIT6		# ENABLE ALL THREE ISS CDU ERROR COUNTERS
		EXTEND			# Extended instruction
		WOR	CHAN12		# Write OR to IMU control channel

; Loop through all three CDU axes (Z, Y, X) to compute angular errors.
; CDUIND serves as the axis index: 2=Z, 1=Y, 0=X
		CAF	TWO		# SET CDU INDICATOR
COARS1		TS	CDUIND		# Store current axis index

; Compute angular error: THETAD - current CDU angle
; The error is scaled and rounded to determine the number of pulses
; required to drive the gimbal toward the desired angle.
		INDEX	CDUIND		# COMPUTE THETAD -- THETAA IN 1'S
		CA	THETAD		# COMPLEMENT FORM (desired angle)
		EXTEND			# Extended instruction follows
		INDEX	CDUIND		# Index into CDU array
		MSU	CDUX		# Subtract current angle (MSU = minus)
		EXTEND			# Extended instruction
		MP	BIT13		# SHIFT RIGHT 2 (scale adjustment)
		XCH	L		# ROUND: exchange A and L registers
		DOUBLE			# Shift left 1 (double)
		TS	ITEMP1		# Store result temporarily
		TCF	+2		# Skip next if no overflow
		ADS	L		# Add L to itself if overflow occurred

; Store the computed pulse command for this axis
		INDEX	CDUIND		# DIFFERENCE TO BE COMPUTED
		LXCH	COMMAND		# Exchange L with COMMAND array element
		CCS	CDUIND		# Check and decrement axis counter
		TC	COARS1		# Process next axis (Y, then X)

; Insert a brief delay to allow IMU hardware to stabilize before processing
; the computed commands. Minimum 4 milliseconds ensures CDU error counters
; are ready to accept pulse trains.
		CAF	TWO		# MINIMUM OF 4 MS WAIT
		TC 	VARDELAY	# Variable delay routine

# Page 1313
; COARS2 - Command Processing Loop
; This section processes the computed angular errors (COMMAND values) and
; converts them into pulse trains sent to the IMU. The routine limits the
; number of pulses per cycle to prevent overdriving the gimbals.

COARS2		TC	CAGETEST	# DON'T CONTINUE IF CAGED.
		TS	ITEMP1		# SET TO +0 (A register now zero)
		CAF	TWO		# SET CDU INDICATOR (start with Z axis)
 +3		TS	CDUIND		# Store axis index

; Process each axis command, checking if pulse count exceeds maximum allowed.
; CCS provides 4-way branch based on command value:
; +nonzero -> COMPOS, +0 -> NEXTCDU+1, -0 -> COMNEG, -nonzero -> NEXTCDU+1
		INDEX	CDUIND		# Select axis
		CCS	COMMAND		# NUMBER OF PULSES REQUIRED
		TC	COMPOS		# Positive: process, may exceed max
		TC	NEXTCDU +1	# +0: no pulses needed
		TC	COMNEG		# Negative: process negative pulses
		TC	NEXTCDU +1	# -0: no pulses needed

; COMPOS - Process Positive Pulse Command
; Limits the number of positive pulses to COMMAX per cycle. If the required
; pulse count exceeds the maximum, it is reduced and the remainder saved
; for the next cycle. This prevents gimbal overshoot and allows controlled
; incremental movement toward the desired angle.

COMPOS		AD	-COMMAX		# COMMAX = MAX NUMBER OF PULSES ALLOWED
		EXTEND			# MINUS ONE (extended instruction follows)
		BZMF	COMZERO		# Branch if result <= 0 (within limit)
		INDEX	CDUIND		# Command exceeds max
		TS	COMMAND		# REDUCE COMMAND BY MAX NUMBER OF PULSES
		CS	-COMMAX-	# ALLOWED (send max pulses this cycle)

; NEXTCDU - Advance to Next Axis
; Records that this axis has pulses to send and advances to the next axis.
; ITEMP1 counts how many axes need pulse commands sent.

NEXTCDU		INCR	ITEMP1		# Increment pulse count flag
		AD	NEG0		# Convert to proper sign
		INDEX	CDUIND		# Select axis command register
		TS	CDUXCMD		# SET UP COMMAND REGISTER (pulse count)

		CCS	CDUIND		# Check and decrement axis index
		TC	COARS2 +3	# Process next axis (Y, then X)

; Check if any axis requires pulses to be sent. If ITEMP1 is positive,
; at least one axis has a non-zero command and we proceed to send pulses.
		CCS	ITEMP1		# SEE IF ANY PULSES TO GO OUT.
		TCF	SENDPULS	# Yes, send the pulse trains

; If no pulses are needed this cycle, all axes are within tolerance.
; Wait for gimbals to settle mechanically, then verify final alignment accuracy.
		TC	FIXDELAY	# WAIT FOR GIMBALS TO SETTLE.
		DEC	150		# 750 millisecond settling time

; CHKCORS - Check Coarse Alignment Accuracy
; Verify that all three gimbal angles are within 2 degrees of desired angles.
; This validates that coarse alignment has achieved its accuracy goal before
; transitioning to fine align or ending the alignment process.

		CAF	TWO		# AT END OF COMMAND, CHECK TO SEE THAT
CHKCORS		TS	ITEMP1		# GIMBALS ARE WITHIN 2 DEGREES OF THETAD.
		INDEX	A		# Index by axis number
		CA	CDUX		# Read actual gimbal angle
		EXTEND			# Extended instruction
		INDEX	ITEMP1		# Index by axis number
		MSU	THETAD		# Subtract desired angle
		CCS	A		# Check sign and magnitude of error
		TCF	COARSERR	# Positive error: check if within tolerance
		TCF	CORSCHK2	# +0 error: perfect alignment, continue
		TCF	COARSERR	# Negative error: check if within tolerance

# Page 1314
CORSCHK2	CCS	ITEMP1		# Check if more axes to verify
		TCF	CHKCORS		# Process next axis (Y, then X)
		TCF	ENDIMU		# END OF COARSE ALIGNMENT (all axes OK)

; COARSERR - Coarse Alignment Error Check
; An axis has non-zero angular error. Check if the error is within the
; 2-degree tolerance. If not, issue alarm 211 (coarse align failure).

COARSERR	AD	COARSTOL	# Add tolerance (2 degrees, negative value)
		EXTEND			# Extended instruction
		BZMF	CORSCHK2	# If result <= 0, error is acceptable

		TC	ALARM		# COARSE ALIGN ERROR (exceeded 2° tolerance)
		OCT	211		# Alarm code 211

		TCF	IMUBAD		# IMU failed coarse align, mark as bad

COARSTOL	DEC	-.01111		# 2 DEGREES SCALED AT HALF-REVOLUTIONS
				# (-.01111 = -2/360 in half-rev units)

; COMNEG - Process Negative Pulse Command
; Similar to COMPOS but handles negative (opposite direction) gimbal commands.
; Limits magnitude to COMMAX and saves remainder for next cycle.

COMNEG		AD	-COMMAX		# Check if magnitude exceeds max
		EXTEND			# Extended instruction
		BZMF	COMZERO		# Within limit, send all pulses
		COM			# Complement (negate) for magnitude
		INDEX	CDUIND		# Select axis
		TS	COMMAND		# Save remainder for next cycle
		CA	-COMMAX-	# Send maximum negative pulses
		TC	NEXTCDU		# Continue to next axis

; COMZERO - Zero Pulse Command
; The required pulse count is within limits. Send the exact count and
; clear the command register for this axis.

COMZERO		CAF	ZERO		# No remainder
		INDEX	CDUIND		# Select axis
		XCH	COMMAND		# Exchange command with zero
		TC	NEXTCDU		# Continue to next axis

; SENDPULS - Send Pulse Trains to IMU
; Writes the computed pulse commands to the IMU control channels, causing
; the error counters to generate torquing pulses that drive the gimbals.

SENDPULS	CAF	13,14,15	# Enable pulse output channels
		EXTEND			# Extended instruction
		WOR	CHAN14		# Write OR to channel 14 (IMU control)
		CAF	600MS		# Schedule next iteration after 600ms
		TCF	COARS2 -1	# THEN TO VARDELAY (return to COARS2 loop)

; CA+ECE - Complete and Enable Error Counters
; This short routine concludes a coarse alignment cycle by enabling the
; CDU error counters and returning control to the waitlist scheduler.

CA+ECE		CAF	BIT6		# ENABLE ALL THREE ISS CDU ERROR COUNTERS
		EXTEND			# Extended instruction
		WOR	CHAN12		# Write OR to IMU control channel
		TC	TASKOVER	# End of this waitlist task

# Page 1315
; ============================================================================
; TRANSITION: From Coarse Alignment Execution to Mode Setup
;
; The previous sections executed the coarse alignment torquing loop. This
; next section (SETCOARS) prepares the IMU for entering coarse alignment
; mode by configuring hardware channels and clearing software flags.
; ============================================================================

; SETCOARS - Set Up Coarse Alignment Mode
; Configures the IMU and spacecraft systems for coarse alignment operation.
; This includes putting the ISS into coarse align mode, disabling the DAP
; (Digital Autopilot), enabling the NO ATT (No Attitude) lamp, and clearing
; various navigation reference flags.

SETCOARS	CAF	BIT4		# BYPASS IF ALREADY IN COARSE ALIGN
		EXTEND			# Extended instruction
		RAND	CHAN12		# Read AND from IMU status channel
		CCS	A		# Check if already in coarse align mode
		TC	Q		# Already set, return immediately

; IMU is not in coarse align. Proceed with mode transition sequence.
		CS	BIT6		# CLEAR ISS ERROR COUNTERS
		EXTEND			# Extended instruction
		WAND	CHAN12		# Write AND to disable error counters

; Stop any ongoing gyro torquing activity to prepare for alignment.
		CS	BIT10		# KNOCK DOWN GYRO ACTIVITY
		EXTEND			# Extended instruction
		WAND	CHAN14		# Write AND to channel 14
		CS	ZERO		# Load -0 (minus zero)
		TS	GYROCMD		# Clear gyro command register

; Enable coarse alignment mode in the IMU hardware.
		CAF	BIT4		# PUT ISS IN COARSE ALIGN
		EXTEND			# Extended instruction
		WOR	CHAN12		# Write OR to set coarse align bit

; Turn on the NO ATT lamp to alert the crew that the IMU does not have
; a valid attitude reference during coarse alignment.
		CS	DSPTAB +11D	# TURN ON NO ATT LAMP
		MASK	OCT40010	# Mask for NO ATT indicator bit
		ADS	DSPTAB +11D	# Add to display table (turns on lamp)

; Disable DAP automatic control modes since the IMU attitude is unreliable
; during coarse alignment. Manual control only.
		CS	IMODES33	# DISABLE DAP AUTO AND HOLD MODES
		MASK	BIT6		# Mask for DAP mode bits
		ADS	IMODES33	# Update mode register

; Disable IMU failure detection since expected errors occur during alignment.
		CS	IMODES30	# DISABLE IMUFAIL
		MASK	BIT4		# Mask for IMU fail inhibit bit
		ADS	IMODES30	# Update mode register

; RNDREFDR - Clear Reference Flags
; Clears navigation reference flags (TRACK, DRIFT, REFSMMAT) since the
; IMU is being realigned and previous reference data is no longer valid.

RNDREFDR	CS	TRACKBIT	# CLEAR TRACK FLAG
		MASK	FLAGWRD1	# Mask to preserve other flags
		TS	FLAGWRD1	# Update flag word 1

		CS	DRFTBIT		# CLEAR DRIFT FLAG
		MASK	FLAGWRD2	# Mask to preserve other flags
		TS	FLAGWRD2	# Update flag word 2

		CS	REFSMBIT	# CLEAR REFSMMAT FLAG (reference matrix)
		MASK	FLAGWRD3	# Mask to preserve other flags
		TS	FLAGWRD3	# Update flag word 3

		TC	Q		# Return to caller

OCT40010	OCT	40010		# Display bit constant for NO ATT lamp

# Page 1316
; ============================================================================
; TRANSITION: From Coarse Alignment to Fine Alignment
;
; The IMU has completed coarse alignment with gimbals positioned within
; 2 degrees of desired angles. This next section transitions the IMU to
; fine alignment mode, where precision gyrocompassing or optical alignment
; can achieve arc-minute accuracy needed for navigation. During Apollo 11's
; mission, fine alignment was critical for accurate trajectory navigation.
; ============================================================================

# IMU FINE ALIGN MODE SWITCH.

; IMUFINE - IMU Fine Alignment Mode Switch
; Transitions the IMU from coarse alignment to fine alignment mode. In fine
; align, the IMU uses gyrocompassing (Earth rate sensing) or optical sightings
; (star tracking) to achieve precision alignment typically better than 1 arc-
; minute. The routine disables coarse align, enables the DAP, schedules delayed
; re-enabling of IMU fail monitoring, and turns off the NO ATT lamp.

IMUFINE		INHINT			# Disable interrupts during mode switch
		TC	CAGETSTJ	# SEE IF IMU BEING CAGED (abort if so)

; Exit coarse align mode and clear zero mode bit.
		CS	BITS4-5		# RESET ZERO AND COARSE (bits 4 and 5)
		EXTEND			# Extended instruction
		WAND	CHAN12		# Write AND to IMU control channel

; Re-enable DAP automatic control modes now that IMU attitude is stable.
		CS	BIT6		# INSURE DAP AUTO AND HOLD MODES ENABLED
		MASK	IMODES33	# Clear the disable bit
		TS	IMODES33	# Update mode register

; Turn off NO ATT lamp since IMU now has valid attitude reference.
		TC	NOATTOFF	# Call subroutine to disable NO ATT indicator

; Schedule delayed re-enabling of IMU fail monitoring. During coarse align,
; IMU fail was inhibited because expected errors occur. Wait 5 seconds into
; fine align before re-enabling failure detection to allow IMU to stabilize.
		CAF	BIT10		# IMU FAIL WAS INHIBITED DURING THE
		TC	WAITLIST	# PRESUMABLY PRECEDING COARSE ALIGN.  LEAVE
		EBANK=	CDUIND		# Set E-bank for task
		2CADR	IFAILOK		# IT ON FOR THE FIRST 5 SECS OF FINE ALIGN
					# Schedule IFAILOK 5.12 seconds from now

; Schedule 2-second task to verify IMU still operational after mode switch.
		CAF	2SECS		# 2 second delay
		TC	WAITLIST	# Schedule on waitlist
		EBANK=	CDUIND		# Set E-bank for task
		2CADR	IMUFINED	# Call IMUFINED after 2 seconds

		TCF	MODEEXIT	# Exit mode switch (restore interrupts)

; IMUFINED - Fine Alignment Delay Completion
; Called 2 seconds after entering fine align mode. Verifies that the IMU
; has not been caged in the meantime, confirming mode switch was successful.

IMUFINED	TC	CAGETEST	# SEE THAT NO ONE HAS CAGED THE IMU
		TCF	ENDIMU		# IMU is valid, complete successfully

# Page 1317
; IFAILOK - Enable IMU Fail Monitoring
; Called 5 seconds after entering fine align mode. Re-enables IMU failure
; detection (IMUFAIL) which was inhibited during coarse alignment. If the
; IMU is being caged or someone has re-entered coarse align, skip enabling.

IFAILOK		TC	CAGETSTQ	# ENABLE IMU FAIL UNLESS IMU BEING CAGED
		TCF	TASKOVER	# IT IS (being caged, so leave fail inhibited)

; Check if IMU has been put back into coarse align mode (bit 4 set).
		CAF	BIT4		# DON'T RESET IMU FAIL INHIBIT IF SOMEONE
		EXTEND			# Extended instruction
		RAND	CHAN12		# Read AND from IMU status
		CCS	A		# Check if coarse align bit is set
		TCF	TASKOVER	# Yes, leave IMU fail inhibited

; Clear IMUFAIL and IMUCFAIL inhibit bits, then continue to common reset logic.
		CS	IMODES30	# RESET IMUFAIL (clear inhibit)
		MASK	BIT13		# Mask for IMUFAIL bit
		ADS	IMODES30	# Update mode register
		CS	BIT4		# Prepare to clear bit 4 (IMUFAIL inhibit)
PFAILOK2	MASK	IMODES30	# Common entry point for fail enable logic
		TS	IMODES30	# Update mode register
		TC	IBNKCALL	# THE ISS WARNING LIGHT MAY COME ON NOW
		CADR	SETISSW		# THAT THE INHIBIT HAS BEEN REMOVED
		TCF	TASKOVER	# End of waitlist task

; PFAILOK - Enable PIPA Fail Monitoring
; Re-enables PIPA (Pulsed Integrating Pendulous Accelerometer) failure
; detection after a period where it was inhibited (e.g., during alignment).
; Also clears both IMU and PIPA fail inhibit flags.

PFAILOK		TC	CAGETSTQ	# ENABLE PIP FAIL PROG ALARM
		TCF	TASKOVER	# IMU being caged, leave fail inhibited

; Clear IMUFAIL and PIPAFAIL inhibit bits in mode registers.
		CS	IMODES30	# RESET IMU AND PIPA FAIL BITS
		MASK	BIT10		# Mask for combined fail bits
		ADS	IMODES30	# Update IMODES30

		CS	IMODES33	# Clear PIPAFAIL inhibit
		MASK	BIT13		# Mask for PIPA fail bit
		ADS	IMODES33	# Update IMODES33

		CS	BIT5		# Prepare to clear bit 5 (PIPAFAIL inhibit)
		TCF	PFAILOK2	# Continue to common reset logic

; NOATTOFF - Turn Off NO ATT Lamp
; Subroutine to turn off the NO ATT (No Attitude) lamp on the DSKY.
; Called when the IMU has a valid attitude reference (after coarse or fine
; alignment completes successfully).

NOATTOFF	CS	OCT40010	# SUBROUTINE TO TURN OFF NO ATT LAMP
		MASK	DSPTAB +11D	# Mask out the NO ATT bit
		AD	BIT15		# Add bit 15 (sign adjustment)
		TS	DSPTAB +11D	# Update display table
		TC	Q		# Return to caller

# Page 1318
; ============================================================================
; TRANSITION: From IMU Mode Switching to PIPA Management
;
; The IMU provides attitude reference through gyroscopes. Equally critical
; for navigation are the PIPAs (Pulsed Integrating Pendulous Accelerometers)
; which measure velocity changes. This section manages PIPA initialization
; and failure monitoring. During Apollo 11's powered descent and ascent,
; PIPA data was essential for computing trajectory and fuel consumption.
; ============================================================================

# ROUTINES TO INITIATE AND TERMINATE PROGRAM USE OF THE PIPAS. NO IMUSTALL REQUIRED IN EITHER CASE.

; PIPUSE - Initialize PIPA Usage
; Zeros the PIPA pulse counters and enables PIPA failure monitoring. Called
; at the start of navigation programs (e.g., P20, P40, descent guidance) that
; rely on PIPA velocity measurements. The three PIPAs measure velocity changes
; along the X, Y, and Z axes of the stable member coordinate frame.

PIPUSE		CS	ZERO		# Clear minus zero
		TS	PIPAX		# Zero X-axis PIPA pulse accumulator
		TS	PIPAY		# Zero Y-axis PIPA pulse accumulator
		TS	PIPAZ		# Zero Z-axis PIPA pulse accumulator

; PIPUSE1 - Enable PIPA Fail Monitoring
; Common entry point to enable PIPA failure detection without zeroing counters.
; Checks if IMU is being caged (abort if so), then enables PIPA fail alarm.

PIPUSE1		TC	CAGETSTQ	# DO NOT ENABLE PIPA FAIL IF IMU IS CAGED
		TCF	SWRETURN	# IMU being caged, return without enabling

		INHINT			# Disable interrupts during mode change
		CS	BIT1		# IF PIPA FAILS FROM NOW ON (UNTIL
		MASK	IMODES30	# PIPFREE), LIGHT ISS WARNING
		TS	IMODES30	# Clear bit 1 (PIPAFAIL inhibit)

; PIPFREE2 - Common PIPA Status Update
; Updates the ISS warning light based on current IMU/PIPA failure status.
; Called when PIPA fail monitoring state changes (enabled or disabled).

PIPFREE2	TC	IBNKCALL	# ISS WARNING MIGHT COME ON NOW
		CADR	SETISSW		# (OR GO OFF ON PIPFREE)
					# Bank call to update ISS warning lamp

		TCF	MODEEXIT	# Exit with interrupts restored

; PIPFREE - Terminate PIPA Usage
; Called when a navigation program completes and no longer needs PIPA data.
; Disables PIPA failure monitoring to prevent nuisance alarms when PIPAs are
; not in active use. If a PIPA failure occurred during use, generates alarm 212.

PIPFREE		INHINT			# PROGRAM DONE WITH PIPAS. DON'T LIGHT
		CS	IMODES30	# ISS WARNING (disable PIPA fail alarm)
		MASK	BIT1		# Mask for PIPAFAIL inhibit bit
		ADS	IMODES30	# Set bit 1 (inhibit PIPA fail alarm)

; Check if a PIPA failure actually occurred while monitoring was enabled.
		MASK	BIT10		# IF PIP FAIL ON, DO PROG ALARM AND RESET
		CCS	A		# ISS WARNING
		TCF	MODEEXIT	# No PIPA failure, exit normally

; PIPA failed during usage. Generate program alarm 212 (PIPA FAIL).
		TC	ALARM		# Generate program alarm
		OCT	212		# Alarm code 212: PIPA failure detected

		INHINT			# Re-disable interrupts

		TCF	PIPFREE2	# Update ISS warning and exit

# Page 1319
; ============================================================================
; TRANSITION: From PIPA Management to Gyro Torquing
;
; The IMU stable platform maintains spacecraft attitude reference using three
; single-degree-of-freedom gyroscopes. To align or correct the platform
; orientation, the gyroscopes must be "torqued" (commanded to precess at
; controlled rates). This section manages the complex process of sending
; torquing commands to the gyros, generating precise pulse trains, handling
; concurrent access, and coordinating with other IMU operations. During
; Apollo 11's mission, gyro torquing was critical for platform alignment,
; gyro drift compensation, and attitude corrections throughout the flight.
; ============================================================================

#          THE FOLLOWING ROUTINE TORQUES THE IRIGS ACCORDING TO DOUBLE PRECISION INPUTS IN THE SIX REGISTERS
# BEGINNING AT THE ECADR ARRIVING IN A. THE MINIMUM SIZE OF ANY PULSE TRAIN IS 16 PULSES (.25 CDU COUNTS). THE
# UNSENT PORTION OF THE COMMAND IS LEFT INTACT IN THE INPUT COMMAND REGISTERS.

; IMUPULSE - IMU Gyro Torquing Command Routine
;
; Sends torquing pulses to the IMU gyroscopes to change stable platform
; orientation. Accepts double-precision commands for all three gyros (OGC,
; IGC, MGC) stored in six consecutive registers beginning at the ECADR
; passed in the A register. Each gyro pulse represents approximately 0.01667
; degrees of platform rotation. Minimum pulse train is 16 pulses (0.25 CDU
; counts). The routine ensures only one torquing operation occurs at a time
; and properly manages the IMU power supply during torquing.
;
; During Apollo 11's lunar landing, gyro torquing was used continuously for:
; - Platform alignment before major maneuvers
; - Compensating for gyro drift (inherent in mechanical gyroscopes)
; - Maintaining accurate attitude reference throughout descent
;
; Input: A register contains ECADR of 6-register command block
;        Registers at ECADR+0 through ECADR+5 contain double-precision
;        torquing commands for outer, inner, and middle gimbals
;
; The routine leaves unsent portions of commands intact for future torquing.

		EBANK=	1400		# VARIABLE, ACTUALLY.

IMUPULSE	TS	MPAC +5		# SAVE ARRIVING ECADR (command block address)
		TC	CAGETSTJ	# DON'T PROCEED IF IMU BEING CAGED
; If IMU is being caged (coarse aligned), abort torquing to prevent conflicts.

; Check if another job is currently using the gyros for torquing.
		CCS	LGYRO		# SEE IF GYROS BUSY (LGYRO>0 means busy)
		TC	GYROBUSY	# SLEEP (put this job to sleep until gyros free)
; Only one job at a time can torque the gyros. If busy, this job sleeps and
; will be awakened when the current torquing operation completes.

		TS	MPAC +2		# Store zero (from CCS if gyros not busy)
		CAF	BIT6		# ENABLE THE POWER SUPPLY (bit 6 = gyro power)
		EXTEND			# Extended instruction
		WOR	CHAN14		# Write OR to channel 14 (IMU control)
; Turn on IMU gyro power supply before torquing. The power supply provides
; the electrical current needed to drive the gyro torquer coils.

; Set up waitlist task to begin gyro torquing after short delay (4 centiseconds).
		CAF	FOUR		# 4 centiseconds = 40 milliseconds delay
GWAKE2		TC	WAITLIST	# Schedule task on waitlist
		EBANK=	CDUIND		# Set EBANK for task
		2CADR	STRTGYRO	# Task address: start gyro torquing
; The delay allows the power supply to stabilize before sending pulses.
; If a job was awakened from GYROBUSY sleep, power supply is already on.

; Set up EBANK for accessing gyro command registers and reserve gyros.
		CA	MPAC +5		# SET UP EBANK, SAVING CALLER'S EBANK FOR
		XCH	EBANK		# RESTORATION ON RETURN (swap with EBANK)
		XCH	MPAC +5		# Caller's EBANK now in MPAC+5
		TS	LGYRO		# RESERVES GYROS (non-zero LGYRO = busy)
; LGYRO serves dual purpose: contains ECADR and acts as busy flag.
; Non-zero LGYRO prevents other jobs from torquing gyros simultaneously.

		MASK	LOW8		# Extract low 8 bits of ECADR
		TS	ITEMP1		# Store base address for indexing commands

; Force sign agreement between high-order and low-order words of each
; double-precision gyro command. This ensures proper two's complement
; representation for positive and negative torquing angles.
		CAF	TWO		# Process 3 gyros (loop counter = 2,1,0)
GYROAGRE	TS	MPAC +3		# Store loop counter
		DOUBLE			# Multiply by 2 (each gyro = 2 registers)
		AD	ITEMP1		# Add base address
		TS	MPAC +4		# Store indexed address
		EXTEND			# Extended instruction
		INDEX	A		# Index by computed address
		DCA	1400		# Load double-precision gyro command
		DXCH	MPAC		# Store in MPAC for processing
		TC	TPAGREE		# Force sign agreement (TP = two's comp)
		DXCH	MPAC		# Retrieve result
		INDEX	MPAC +4		# Index back to command location
		DXCH	1400		# Store corrected command

		CCS	MPAC +3		# Decrement loop counter
		TCF	GYROAGRE	# Continue for next gyro (2→1, 1→0)
; Loop processes all three gyros: outer gimbal, inner gimbal, middle gimbal.

; Restore caller's EBANK and exit through mode switching exit routine.
		CA	MPAC +5		# RESTORE CALLER'S EBANK
		TS	EBANK		# Restore original EBANK setting
		TCF	MODEEXIT	# Exit and perform standard mode cleanup

# Page 1320
; ============================================================================
; GYRO TORQUING CONCURRENCY CONTROL
;
; These routines implement mutual exclusion for gyro torquing. Since the IMU
; has only one set of gyro torquer circuits, only one program can send
; torquing commands at a time. If a job attempts to torque while another job
; is already torquing, it is put to sleep and will be awakened when the gyros
; become available. This prevents conflicting torquing commands and ensures
; proper pulse train sequencing. Critical during Apollo 11 when multiple
; programs (alignment, drift compensation) might need to torque simultaneously.
; ============================================================================

# ROUTINES TO ALLOW TORQUING ONLY ONE JOB AT A TIME.

; GYROBUSY - Put Job to Sleep While Gyros Are Busy
; Called when a job wants to torque but LGYRO indicates gyros are in use.
; Saves the job's return address and puts it to sleep. The job will be
; awakened by GWAKE when the current torquing operation completes.

GYROBUSY	EXTEND			# Extended instruction
		DCA	BUF2		# SAVE RETURN 2FCADR (calling address)
		DXCH	MPAC		# Store in MPAC for preservation
REGSLEEP	CAF	LGWAKE		# Load address of wakeup routine
		TCF	JOBSLEEP	# Put this job to sleep (scheduler)
; Job scheduler will awaken this job later via GWAKE address.

; GWAKE - Wakeup Routine for Sleeping Gyro Torquing Jobs
; Called by scheduler when awakening a job that was waiting for gyros.
; Checks if gyros are now free. If still busy, goes back to sleep.
; If free, restores job state and continues with torquing.

GWAKE		CCS	LGYRO		# WHEN AWAKENED, SEE IF GYROS STILL BUSY
		TCF	REGSLEEP	# IF SO, SLEEP SOME MORE (another job took gyros)
; It's possible another job grabbed the gyros between wakeup and this check.

		TS	MPAC +2		# Store zero (gyros are free)
		EXTEND			# Extended instruction
		DCA	MPAC		# Restore return address
		DXCH	BUF2		# RESTORE SWRETURN INFO (for scheduler)
		CAF	ONE		# Load 1 (indicates awakened job)
		TCF	GWAKE2		# Continue torquing at GWAKE2 entry point
; GWAKE2 is alternate entry to torquing routine for awakened jobs.

LGWAKE		CADR	GWAKE		# Address constant for GWAKE routine

# Page 1321
# GYRO-TORQUING WAITLIST TASKS.

; ============================================================================
; GYRO TORQUING PULSE TRAIN SEQUENCING
;
; After IMUPULSE initiates a torquing operation, the STRTGYRO routine
; sequences through all three gyros (Y, Z, X), generating precise pulse
; trains for each axis. The routine uses a phase counter in LGYRO to track
; which gyro is currently being torqued, cycling through Y → Z → X → done.
; 
; COMMENT-ONLY READERS: The IMU's three gyroscopes must be torqued (nudged)
; in sequence to achieve the desired spacecraft orientation. This section
; generates the electrical pulse trains that physically rotate each gyro.
; 
; CODE-ALONG READERS: Uses LGYRO phase counter (bits 13-14) to index through
; gyro selection table. Each GSELECT call processes one axis with packed
; command word containing gyro select bits and LGYRO increment.
; ============================================================================

STRTGYRO	CS	GDESELCT	# DE-SELECT LAST GYRO.
		EXTEND
		WAND	CHAN14

; Check if IMU cage (coarse align) has been interrupted.
		TC	CAGETEST

; Phase selection logic: Determine which gyro to torque next based on
; LGYRO phase counter. Sequence is Y (phase 0) → Z (phase 1) → X (phase 2)
; → completion (phase 3).
STRTGYR2	CA	LGYRO		# JUMP ON PHASE COUNTER IN BITS 13-14.
		EXTEND
		MP	BIT4
		INDEX	A
		TCF	+1
		TC	GSELECT		# =0.  DO Y GYRO.
		OCT	00202

		TC	GSELECT		# =1.  DO Z GYRO.
		OCT	00302

		TC	GSELECT -2	# =2.  DO X GYRO.
		OCT	00100

; Phase 3: All gyros completed. Reset phase counter and wake any job
; that was waiting for gyro torquing to finish.
		CAF	ZERO		# =3.  DONE
		TS	LGYRO
		CAF	LGWAKE		# WAKE A POSSIBLE SLEEPING JOB.
		TC	JOBWAKE

NORESET		TCF	IMUFINED	# DO NOT RESET POWER SUPPLY
# Page 1322
 -2		CS	FOUR		# SPECIAL ENTRY TO REGRESS LGYRO FOR X.
		ADS	LGYRO

; ============================================================================
; GSELECT - Gyro Selection and Command Processing
;
; Selects one gyro for torquing and processes its double-precision command.
; The routine unpacks the gyro select bits from the packed word at Q+1,
; loads the DP command from erasable memory, and determines the pulse
; polarity and magnitude. Commands are split into major (high order) and
; minor (fractional) components for precise gyro positioning.
; ============================================================================

GSELECT		INDEX	Q		# SELECT GYRO.
		CAF	0		# PACKED WORD CONTAINS GYRO SELECT BITS
		TS	ITEMP4		# AND INCREMENT TO LGYRO.
		MASK	SEVEN
		AD	BIT13
		ADS	LGYRO
		TS	EBANK
		MASK	LOW8
		TS	ITEMP1

		CS	SEVEN
		MASK	ITEMP4
		TS	ITEMP4

; Load DP gyro command into RUPTREG1/RUPTREG2 for magnitude testing.
; Commands are classified as major (large) or minor (small), positive or
; negative. Threshold is 16 gyro pulses (GYROMIN constant).
		EXTEND			# MOVE DP COMMAND TO RUPTREGS FOR TESTING.
		INDEX	ITEMP1
		DCA	1400
		DXCH	RUPTREG1

; Test high-order word (RUPTREG1) to determine major positive/negative.
		CCS	RUPTREG1
		TCF	MAJ+
		TCF	+2
		TCF	MAJ-

; High-order zero: test low-order word (RUPTREG2) for minor commands.
		CCS	RUPTREG2
		TCF	MIN+
		TCF	STRTGYR2
		TCF	MIN-
		TCF	STRTGYR2

# Page 1323
; MIN+ path: Small positive command. Check if magnitude meets 16-pulse minimum.
; If below threshold, skip this gyro and advance to next phase.
MIN+		AD	-GYROMIN	# SMALL POSITIVE COMMAND.  SEE IF AT LEAST
		EXTEND			# 16 GYRO PULSES.
		BZMF	STRTGYR2

; Major positive command: Add fractional correction (GYROFRAC) and set
; positive torquing direction in channel 14.
MAJ+		EXTEND			# DEFINITE POSITIVE OUTPUT.
		DCA	GYROFRAC
		DAS	RUPTREG1

		CA	ITEMP4		# SELECT POSITIVE TORQUING FOR THIS GYRO.
		EXTEND
		WOR	CHAN14

; Extract augment count (bits 0-6 of low word) and prepare command for
; merging. Commands exceeding 16383 pulses require multiple pulse trains
; with 8192-pulse increments.
		CAF	LOW7		# LEAVE NUMBER OF POSSIBLE 8192 AUGMENTS
		MASK	RUPTREG2	# TO INITIAL COMMAND IN MAJOR PART OF LONG
		XCH	RUPTREG2	# TERM STORAGE AND TRUNCATED FRACTION
GMERGE		EXTEND			# IN MINOR PART.  THE MAJOR PART WILL BE
		MP	BIT8		# COUNTED DOWN TO ZERO IN THE COURSE OF
		TS	ITEMP2		# PUTTING OUT THE ENTIRE COMMAND.
		CA	RUPTREG1
		EXTEND
		MP	BIT9
		TS	RUPTREG1
		CA	L
		EXTEND
		MP	BIT14
		ADS	ITEMP2		# INITIAL COMMAND.

		EXTEND			# SEE IF MORE THAN ONE PULSE TRAIN NEEDED
		DCA	RUPTREG1	# (MORE THAN 16383 PULSES).
		AD	MINUS1
		CCS	A
		TCF	LONGGYRO
-GYROMIN	OCT	-176		# MAY BE ADJUSTED TO SPECIFY MINIMUM CMD
		TCF	+4

		CAF	BIT14
		ADS	ITEMP2
		CAF	ZERO

 +4		INDEX	ITEMP1
		DXCH	1400
# Page 1324
		CA	ITEMP2		# ENTIRE COMMAND.
LASTSEG		TS	GYROCMD
		EXTEND
		MP	BIT10		# WAITLIST DT
		AD	THREE		# TRUNCATION AND PHASE UNCERTAINTIES.
		TC	WAITLIST
		EBANK=	CDUIND
		2CADR	STRTGYRO

GYROEXIT	CAF	BIT10
		EXTEND
		WOR	CHAN14
		TCF	TASKOVER

LONGGYRO	INDEX	ITEMP1
		DXCH	1400		# INITIAL COMMAND OUT PLUS N AUGMENTS OF
		CAF	BIT14		# 8192.  INITIAL COMMAND IS AT LEAST 8192.
		AD	ITEMP2
		TS	GYROCMD

AUG3		EXTEND			# GET WAITLIST DT TO TIME WHEN TRAIN IS
		MP	BIT10		# ALMOST OUT.
		AD	NEG3
		TC	WAITLIST
		EBANK=	CDUIND
		2CADR	8192AUG

		TCF	GYROEXIT

8192AUG		TC	CAGETEST

		CAF	BIT4
		EXTEND
		RAND	CHAN12
		CCS	A
		TCF	IMUBAD
		CA	LGYRO		# ADD 8192 PULSES TO GYROCMD
		TS	EBANK
		MASK	LOW8
		TS	ITEMP1

		INDEX	ITEMP1		# SEE IF THIS IS THE LAST AUG.
		CCS	1400
		TCF	AUG2		# MORE TO COME.

		CAF	BIT14
		ADS	GYROCMD
		TCF	LASTSEG +1

# Page 1325
AUG2		INDEX	ITEMP1
		TS	1400
		CAF	BIT14
		ADS	GYROCMD
		TCF	AUG3		# COMPUTE DT.

# Page 1326
; ============================================================================
; NEGATIVE GYRO COMMAND HANDLING
; Commands in negative direction (opposite to positive gyro torquing).
; Same structure as positive paths: MIN- checks threshold, MAJ- processes
; definite commands. Channel 14 bit 9 controls polarity (0=positive, 1=negative).
; ============================================================================

; MIN- path: Small negative command. Check if magnitude meets 16-pulse minimum.
; If below threshold, skip this gyro and advance to next phase.
MIN-		AD	-GYROMIN	# POSSIBLE NEGATIVE OUTPUT.
		EXTEND
		BZMF	STRTGYR2

; Major negative command: Subtract fractional correction (GYROFRAC) and set
; negative torquing direction in channel 14 bit 9.
MAJ-		EXTEND			# DEFINITE NEGATIVE OUTPUT.
		DCS	GYROFRAC
		DAS	RUPTREG1

		CA	ITEMP4		# SELECT NEGATIVE TORQUING FOR THIS GYRO.
		AD	BIT9
		EXTEND
		WOR	CHAN14

; Complement command values to convert negative to positive representation.
; Channel 14 bit 9 controls actual polarity sent to IMU. All GYROCMD values
; stored as positive magnitudes; hardware interprets sign from channel bit.
		CS	RUPTREG1	# SET UP RUPTREGS TO FALL INTO GMERGE.
		TS	RUPTREG1	# ALL NUMBERS PUT INTO GYROCMD ARE
		CS	RUPTREG2	# POSITIVE -- BIT9 OF CHAN 14 DETERMINES
		MASK	LOW7		# THE SIGN OF THE COMMAND.
		COM
		XCH	RUPTREG2
		COM
		TCF	GMERGE

; Gyro deselect mask: Turns off select and activity control bits in channel 14.
GDESELCT	OCT	1700		# TURN OFF SELECT AND ACTIVITY BITS.

; Gyro fractional correction constant: 0.215 scaled at B-21 (bit position -21).
; Applied to gyro commands to compensate for residual drift and quantization.
GYROFRAC	2DEC	.215 B -21

# Page 1327
# IMU MODE SWITCHING ROUTINES COME HERE WHEN ACTION COMPLETE.

; ============================================================================
; IMU MODE TERMINATION AND VALIDATION
; Called when IMU mode switching operations complete. Validates IMU status
; by checking for cage condition and ISS warning. Returns GOOD or BAD status.
; ============================================================================

; ENDIMU: Main termination entry. Checks IMU status after mode operation.
; Reads ISS warning bit from DSALMOUT. If warning present, declares bad mode.
ENDIMU		EXTEND			# MODE IS BAD IF CAGE HAS OCCURRED OR IF
		READ	DSALMOUT	# ISS WARNING IS ON.
		MASK	BIT1
		CCS	A
		TCF	IMUBAD

; IMUGOOD exit: Mode switching completed successfully with no errors.
; Returns to caller with A register = 0 indicating good status.
IMUGOOD		TCF	GOODEND		# WITH C(A) = 0.

; IMUBAD exit: Mode switching failed or IMU in degraded state.
; Returns to caller with A register = 0 but via BADEND (different return path).
IMUBAD		CAF	ZERO
		TCF	BADEND

; CAGETEST subroutine: Checks if IMU has been caged during mode operation.
; Tests IMODES30 bit 6 (cage flag). If caged, terminates immediately as bad.
; Returns to caller if not caged.
CAGETEST	CAF	BIT6		# SUBROUTINE TO TERMINATE IMU MODE
		MASK	IMODES30	# SWITCH IF IMU HAS BEEN CAGED.
		CCS	A
		TCF	IMUBAD		# DIRECTLY.
		TC	Q		# WITH C(A) = +0.

; CAGETSTQ subroutine: Conditional return based on cage status.
; If IMU NOT being caged, increments return address (skip next instruction).
; If IMU IS being caged, returns to normal return address.
CAGETSTQ	CS	IMODES30	# SKIP IF IMU NOT BEING CAGED.
		MASK	BIT6
		CCS	A
		INCR	Q
		TC	Q

; CAGETSTJ subroutine: Cage test during mode switch initialization.
; If cage detected during initialization, sets IMUCADR to -0 to indicate
; operation failure, preventing further mode switching attempts.
CAGETSTJ	CS	IMODES30	# IF DURING MODE SWITCH INITIALIZATION
		MASK	BIT6		# IT IS FOUND THAT THE IMU IS BEING CAGED,
		CCS	A		# SET IMUCADR TO -0 TO INDICATE OPERATION
		TC	Q		# COMPLETE BUT FAILED.  RETURN IMMEDIATELY

		CS	ZERO		# TO SWRETURN.
		TS	IMUCADR
		TCF	MODEEXIT

# Page 1328
#          GENERALIZED MODE SWITCHING TERMINATION. ENTER AT GOODEND FOR SUCCESSFUL COMPLETION OF AN I/O OPERATION
# OR AT BADEND FOR A N UNSUCCESSFUL ONE. C(A) OR ARRIVAL =0 FOR IMU, 1 FOR OPTICS.

BADEND		TS	RUPTREG2	# DEVICE INDEX.
		CS	ZERO		# FOR FAILURE.
		TCF	GOODEND +2

GOODEND		TS	RUPTREG2
		CS	ONE		# FOR SUCCESS.

		TS	RUPTREG3
		INDEX	RUPTREG2	# SEE IF USING PROGRAM ASLEEP.
		CCS	MODECADR
		TCF	+4		# YES -- WAKE IT UP.
		TCF	ENDMODE		# IF 0, PROGRAM NOT IN YET.

		EXTEND
		BZF	ENDMODE +1	# BZF = TCF IF MODECADR = -0.

		CAF	ZERO		# WAKE SLEEPING PROGRAM.
		INDEX	RUPTREG2
		XCH	MODECADR
		TC	JOBWAKE

		CS	RUPTREG3	# ADVANCE LOC IF SUCCESSFUL.
		INDEX	LOCCTR
		ADS	LOC

		TCF	TASKOVER

ENDMODE		CA	RUPTREG3	# -0 INDICATES OPERATION COMPLETE BUT
 +1		INDEX	RUPTREG2	# UNSUCCESSFUL: -1 INDICATES COMPLETE AND
		TS	MODECADR	# SUCCESSFUL.
		TCF	TASKOVER

# ============================================================================
# STALLING ROUTINES SECTION - IMU/AOT/RADAR OPERATION COMPLETION POLLING
#
; These stalling routines provide a generalized mechanism for programs to
; wait until external IMU, AOT, or radar operations complete. During lunar
; descent and ascent, the guidance computer must frequently coordinate with
; the IMU for attitude updates while the IMU is processing alignment or
; gyro torquing commands. Rather than spinning in tight loops (wasting
; computational resources), these routines yield control to other tasks
; through the WAITLIST scheduler, periodically checking status flags until
; the external device signals completion.
;
; COMMENT-ONLY READERS: These routines enable the AGC to multitask efficiently
; during time-consuming IMU operations. While waiting for IMU alignment or
; gyro commands to complete, the computer continues running other mission-
; critical programs. This cooperative scheduling approach was essential to
; preventing the 1202 program alarms during Apollo 11's descent.
;
; CODE-ALONG READERS: The STALL routine implements a polling loop with timed
; delays. It repeatedly checks a flagword (IMODES30 for IMU, AOTFLAGS for AOT,
; RADMODES for radar) against a mask. If no bits match, it schedules itself
; on the WAITLIST for 20ms delay and tries again. When a flag bit is detected,
; the routine returns with that bit isolated in the A register, allowing the
; caller to determine which specific operation completed.
# ============================================================================

# Page 1329
#          GENERAL STALLING ROUTINE. USING PROGRAMS COME HERE TO WAIT FOR I/O COMPLETION.
#
# PROGRAM DESCRIPTION                                    DATE- 21 FEB 1967
#                                           LOG SECTION IMU MODE SWITCHING
# MOD BY- R.MELANSON TO ADD DOCUMENTATION       ASSEMBLY SUNDISK  REV.  82
#
# FUNCTIONAL DESCRIPTION-
#	TO DELAY FURTHER EXECUTION OF THE CALLING ROUTINE UNTIL ITS SELECTED
#	I/O FUNCTION IS COMPLETE.THE FOLLOWING CHECKS ON THE CALLING ROUTINE:S
#	MODECADR ARE MADE AND ACTED UPON.
#	  1) +0 INDICATES INCOMPLETE I/O OPERATION.CALLING ROUTINE IS PUT TO
#	     SLEEP.
#	  2) -1 INDICATES COMPLETED I/O OPERATION. STALL BYPASSES JOBSLEEP
#	     CALL AND RETURNS TO CALLING ROUTINE AT L+3
#	  3) -0 INDICATES COMPLETED I/O WITH FAILURE. STALL CLEARS MODECADR
#	     AND RETURNS TO CALLING ROUTINE AT L+2.
#	  4) VALUE GREATER THAN 0 INDICATES TWO ROUTINES CALLING FOR USE OF
#	     SAME DEVICE.  STALL EXITS TO ABORT WHICH EXECUTES A PROGRAM
#	     RESTART WHICH IN TURN CLEARS ALL MODECADR REGISTERS.
#
#  CALLING SEQUENCE-
#	L 	TC	 BANKCALL
#	L+1	CADR (ONE OF 5 STALL ADDRESSES I.E. IMUSTALL,OPTSTALL,RADSTALL,
#			  AOTSTALL,OR ATTSTALL)
#
# NORMAL-EXIT MODE-
#	TCF  JOBSLEEP OR TCF   MODEXIT
#
# ALARM OR ABORT EXIT MODE-
#	TC   ABORT
#
# OUTPUT-
#	MODECADR= CADR IF JOBSLEEP
#	MODECADR=+0    IF I/O COMPLETE
#	BUF2=L+3       IF I/O COMPLETE AND GOOD.
#	BUF2=L+2 IF I/O COMPLETE BUT FAILED.
#
# ERASABLE INITIALIZATION-
#	BUF2 CONTAINS RETURN ADDRESS PLUS 1,(L+2)
#	BUF2+1 CONTAINS FBANK VALUE OF CALLING ROUTINE.
#	MODECADR OF CALLING ROUTINE CONTAINS +0,-1,-0 OR  CADR RETURN ADDRESS.
#
# DEBRIS-
#	RUPTREG2 AND CALLING ROUTINE MODECADR.

; Device-specific stall entry points. Each loads an index (0=IMU, 1=AOT, 2=RADAR)
; and calls the common STALL routine to check the corresponding MODECADR.

AOTSTALL	CAF	ONE		# AOT.
		TC	STALL

RADSTALL	CAF	TWO
		TCF	STALL

# Page 1330
OPTSTALL	EQUALS	AOTSTALL

IMUSTALL	CAF	ZERO		# IMU.

; Core stalling routine. Checks device MODECADR status:
; +0 = operation incomplete, put calling program to sleep via JOBSLEEP
; -1 = operation complete and successful, return to caller at L+3
; -0 = operation complete but failed, return to caller at L+2
; >0 = illegal state (two routines using same device), abort to restart
;
; During Apollo 11 descent, proper use of IMUSTALL prevented IMU operations
; from monopolizing CPU time, allowing guidance and throttle control to
; continue executing while awaiting IMU alignment completion.

STALL		INHINT
		TS	RUPTREG2	# SAVE DEVICE INDEX.
		INDEX	A		# SEE IF OPERATION COMPLETE.
		CCS	MODECADR
		TCF	MODABORT	# ALLOWABLE STATES ARE +0, -1, AND -0.
		TCF	MODESLP		# OPERATION INCOMPLETE.
		TCF	MODEGOOD	# COMPLETE AND GOOD IF = -1.

MG2		INDEX	RUPTREG2	# COMPLETE AND FAILED IF -0.  RESET TO +0.
		TS	MODECADR	# RETURN TO CALLER.
		TCF	MODEEXIT

MODEGOOD	CCS	A		# MAKE SURE INITIAL STATE -1.
		TCF	MODABORT

		INCR	BUF2		# IF SO, INCREMENT RETURN ADDRESS AND
		TCF	MG2		# RETURN IMMEDIATELY, SETTING CADR = +0.

; Operation incomplete path: Store return address in device MODECADR and
; yield control through JOBSLEEP. The calling program will be reawakened
; when the device operation completes and another task updates MODECADR to -1.

MODESLP		TC	MAKECADR	# CALL FROM SWITCHABLE FIXED ONLY.
		INDEX	RUPTREG2
		TS	MODECADR
		TCF	JOBSLEEP

; Abort path: Illegal state detected (MODECADR > 0). This indicates two programs
; attempted simultaneous use of the same device. Issue program alarm 01210 and
; initiate restart to clear all MODECADR registers and reset system state.

MODABORT	DXCH	BUF2
		TC	BAILOUT1	# TWO PROGRAMS USING THE SAME DEVICE.
		OCT	1210

# Page 1331
# CONSTANTS FOR MODE SWITCHING ROUTINES

BITS3&4		=	OCT14
BITS4&6		=	OCT50
BITS4-5		OCT	00030
IMUSEFLG	EQUALS	BIT8		# INTERPRETER SWITCH 7.
-COMMAX		DEC	-191
-COMMAX-	DEC	-192
600MS		DEC	60
IMUFIN20	=	IMUFINE

; GOMANUR - Manual attitude maneuver initiation check. Verifies that the
; Kalman filter steering (KALCMANU) is not currently active. If ATTCADR is
; zero (KALCMANU free), saves return address for KALCMAN3 and continues.
; If ATTCADR is non-zero (KALCMANU busy), issues POODOO abort 01210.
; Used when crew initiates manual attitude maneuvers during mission phases.

GOMANUR		CA	ATTCADR		# IS KALCMANU FREE
		EXTEND
		BZF	+3

		TC	POODOO		# NO
		OCT	1210		# 2 TRYING TO USE SAME DEVICE

 +3		EXTEND
		DCA	BUF2
		DXCH	ATTCADR		# SAVE FINAL RETURN FOR KALCMAN3

		CA	BBANK
		MASK	SEVEN
		ADS	ATTCADR +1

		CA	PRIORITY
		MASK	PRIO37
		TS	ATTPRIO		# SAVE USERS PRIO

		CAF	KALEBCON	# SET EBANK FOR KALCMAN3
		TS	EBANK
		TC	POSTJUMP
		CADR	KALCMAN3
KALEBCON	ECADR	BCDU

# ============================================================================
# R02 - IMU STATUS CHECK ROUTINE
#
; R02 verifies that the IMU is powered on and aligned to a known orientation
; before allowing programs to proceed with navigation or guidance operations.
; If the IMU is off or its alignment state is unknown to the CMC, R02 requests
; crew selection of an appropriate program to restore IMU operational status.
;
; COMMENT-ONLY READERS: Before the computer can use the IMU for navigation, it
; must verify that the platform is spinning and aligned. If the IMU was turned
; off or lost alignment, the crew must run an alignment program (P51-P53) to
; re-establish a known orientation relative to the stars.
;
; CODE-ALONG READERS: R02BOTH checks IMODES30 flag bits to determine IMU status.
; If IMU is off (bit 9 clear) or alignment is lost (REFSMFLG clear), issues
; alarm and requests operator action through GOTOPOOH display interface.
# ============================================================================

# Page 1332
# PROGRAM DESCRIPTION
# IMU STATUS CHECK ROUTINE R02 (SUBROUTINE UTILITY)
# MOD NO - 1
# MOD BY - N.BRODEUR
# FUNCTIONAL DESCRIPTION
#
# TO CHECK WHETHER IMU IS ON AND IF ON WHETHER IT IS ALIGNED TO AN
# ORIENTATION KNOWN BY THE CMC. TO REQUEST SELECTION OF THE APPROPRIATE
# PROGRAM IF THE IMU IS OFF OR NOT ALIGNED TO AN ORIENTATION KNOWN BY THE
# CMC. CALLED THROUGH BANKCALL
# CALLING SEQUENCE-
#
# L        TC     BANKCALL
# L+1      CADR   R02BOTH
# SUBROUTINES CALLED
#
#       VARALARM
#       FLAGUP
# NORMAL EXIT MODES
#
# AT L+2 OF CALLING SEQUENCE
# ALARM OR ABORT EXIT MODES
#       GOTOPOOH, WITH ALARM
# ERASABLE INITIALIZATION REQUIRED
#
# NONE
# DEBRIS
#
# CENTRALS-A,Q,L

		BANK	34
		SETLOC	R02
		BANK
		COUNT*	$$/R02
DEC51		DEC	51

; R02BOTH entry point. First checks if REFSMMAT (Reference Stable Member Matrix)
; is flagged as valid in FLAGWRD3. If REFSMBIT is set, IMU orientation is known
; and routine sets IMUSE flag and returns successfully.

R02BOTH		CAF	REFSMBIT
		MASK	FLAGWRD3
		CCS	A
		TC	R02ZERO		# ZERO IMUS

; REFSMMAT not valid. Check if IMU is at least powered on (IMODES30 bit 9).
; If IMU is off, issue alarm 220 (REFSMM alarm - no reference orientation).
; If IMU is on but not aligned, issue alarm 210 (ISS not initialized alarm).
; Either condition requires operator intervention through GOTOPOOH.

		CA	IMODES30
		MASK	BIT9		# IS ISS INITIALIZED
		EXTEND
		BZF	+2
		CS	BIT4		# SEND IMU ALARM CODE 210
		AD	OCT220		# SEND REFSMM ALARM
		TC	VARALARM

		TC	GOTOPOOH

; IMU status acceptable. Set IMUSE flag to indicate IMU is available for
; navigation programs and return to caller.

R02ZERO		TC	UPFLAG
# Page 1333
		ADRES	IMUSE
		TCF	SWRETURN
OCT220		OCT	220

# Page 1334
# PROGRAM DESCRIPTION   P06   10FEB67
#
# TRANSFER THE ISS/CMC FROM THE OPERATE TO THE STANDBY CONDITION.
#
# THE NORMAL CONDITION OF READINESS OF THE GNCS WHEN NOT IN USE IS STANDBY. IN THIS CONDITION THE IMU
# HEATER POWER IS ON. THE IMU OPERATE POWER IS OFF. THE COMPUTER POWER IS ON. THE OPTICS POWER IS OFF. THE
# CMC  STANDBY ON THE MAIN AND LEB DISKYS IS ON.
#
# CALLING SEQUENCE:
#          ASTRONAUT REQUEST THROUGH DSKY     V37E 06E.
#
# SUBROUTINES CALLED:
#          GOPERF1
#          BANKCALL
#          FLAGDOWN
#
# Page 1335
# PRESTAND PREPARES FOR STANDBY BY SNAPSHOTTING THE SCALER AND TIME1 TIME2
# THE LOW 5 BITS OF THE SCALER ARE INSPECTED TO INSURE COMPATIBILITY
# BETWEEN THE SCALER READING AND THE TIME1 TIME2 READING.

# ============================================================================
# P06 - TRANSFER TO STANDBY MODE
#
; P06 prepares the Apollo Guidance Computer for power-down to standby mode,
; preserving critical timing and state information needed for recovery when
; the system is restored to operational status. During extended mission phases
; when guidance is not required (such as lunar surface operations or long
; coast periods), standby mode conserves spacecraft electrical power while
; maintaining minimum AGC functionality.
;
; COMMENT-ONLY READERS: The crew could place the computer into standby mode
; (V37E 06E) during non-critical mission phases to save power. The computer
; carefully recorded the exact time before shutdown and could precisely
; determine how much time elapsed during standby when power was restored.
;
; CODE-ALONG READERS: PRESTAND snapshots TIME2/TIME1 and the hardware SCALER
; register to establish a reference timestamp. SCALPREP verifies that the
; scaler low 5 bits and time registers are mutually consistent (scaler
; increments every 10 milliseconds, so compatibility check prevents race
; conditions). System flags (REFSMMAT, drift tracking, IMU use) are cleared.
; The standby enable bit (channel 13 bit 11) is set. Recovery restart is
; configured to POSTAND entry point for time restoration upon power-up.
# ============================================================================

		SETLOC	P05P06
		BANK

		EBANK=	TIME2SAV
		COUNT*	$$/P06

; P06 entry point. Set NODOV37 flag to prevent V37 (change program) operations
; during standby preparation sequence.

P06		TC	UPFLAG		# SET NODOV37 BIT
		ADRES	NODOFLAG

; PRESTAND begins standby preparation. Captures current time and scaler state,
; verifying temporal consistency before proceeding with shutdown sequence.

PRESTAND	INHINT
		EXTEND
		DCA	TIME2		# SNAPSHOT TIME1TIME2
		DXCH	TIME2SAV
		
; Call SCALPREP to read hardware scaler and verify compatibility with time
; registers. If low 5 bits of scaler indicate recent TIME1 increment (race
; condition detected), SCALPREP returns to L+1 (TC PRESTAND) to retry the
; entire snapshot. If compatible, SCALPREP returns to L+2 with adjusted
; scaler value in MPAC.

		TC	SCALPREP
		TC	PRESTAND	# T1,T2,SCALER NOT COMPATIBLE
		
; Scaler and time registers confirmed compatible. Save adjusted scaler value
; to SCALSAVE for later use during POSTAND recovery.

		DXCH	MPAC		# T1,T2 AND SCALER OK
		DXCH	SCALSAVE	# STORE SCALER
		
; Clear IMU-related flags to indicate standby state. REFSMMAT flag indicates
; IMU orientation is no longer valid. Drift and tracking flags cleared since
; IMU will be inactive during standby.

		INHINT
		TC	BANKCALL
		CADR	RNDREFDR	# REFSMM, DRIFT, TRACK FLAGS DOWN

		TC	DOWNFLAG
		ADRES	IMUSE		# IMUSE DOWN
		TC	DOWNFLAG
		ADRES	RNDVZFLG	# RNDVZFLG DOWN

; Set standby enable bit in channel 13. This signals to spacecraft power
; systems that AGC is ready for low-power standby mode.

		CAF	BIT11
		EXTEND
		WOR	CHAN13		# SET STANDBY ENABLE BIT

; Configure restart protection to resume at POSTAND when power is restored.
; POSTAND will compute elapsed standby time using saved TIME2SAV/SCALSAVE
; and update current time registers accordingly.

		TC	PHASCHNG	# SET RESTART TO POSTAND WHEN STANDBY
		OCT	07024		#	RECOVERS
		OCT	20000
		EBANK=	SCALSAVE
		2CADR	POSTAND

; Final step: Display V06N22 (program number 06, noun 22 = elapsed standby time).
; This allows crew to monitor standby status. The three TCF instructions form
; an endless loop waiting for standby power-down or crew abort of P06.

		CAF	OCT62
		TC	BANKCALL
		CADR	GOPERF1
		TCF	-3
		TCF	-4
		TCF	-5

OCT62		EQUALS	.5SEC		# DEC 50 = OCT 62

# THE LOW 5 BITS OF THE SCALER READS 10000 FOR THE FIRST INTERVAL AFTER A
# Page 1336
# T1 INCREMENT. IF SCALPREP DETECTS THIS INTERVAL THE T1,T2 AND SCALER
# DATA ARE NOT COMPATIBLE AND RETURN IS TO L+1 FOR ANOTHER READING OF THE
# DATA. OTHERWISE, THE RETURN IS TO L+2 TO PROCEED. ROUTINE ALSO PREPARES
# THE SCALER READING FOR COMPUTATION OF THE INCREMENT TO UPDATE T1T2. (THE
# 10 MS BIT (BIT 6) OF THE SCALER IS INCREMENTED 5 MS OUT OF PHASE FROM
# T1.) ADDITION OF 5 MS (BIT 5) TO THE SCALER READING HAS THE EFFECT OF
# ADJUSTING BIT 6 IN THE SCALER TO BE IN PHASE WITH BIT 1 OF T1. THE LOW 5
# BITS OF THE SCALER READING ARE THEN SET TO ZERO, TO TRUNCATE THE SCALER
# DATA TO 10 MS. RESULTS ARE STORED IN MPAC, +1.

; ============================================================================
; SCALPREP - SCALER/TIME COMPATIBILITY VERIFICATION
;
; SCALPREP validates temporal consistency between hardware SCALER register
; and TIME1/TIME2 counters to prevent race condition errors. The SCALER
; increments every 10 milliseconds. TIME1 increments every 10 milliseconds
; from SCALER overflow. Due to circuit propagation delays, there is a brief
; window (first 5ms after TIME1 increment) where SCALER/TIME1 are inconsistent.
;
; SCALPREP detects this race condition by examining the low 5 bits of SCALER.
; If these bits read 10000 (binary), the snapshot occurred in the incompatible
; window and caller must retry (return to L+1). Otherwise, SCALPREP adjusts
; the scaler value for phase alignment and returns to L+2.
;
; The adjustment adds 5ms (BIT5) to align SCALER's 10ms bit (BIT6) with TIME1's
; phase, then truncates low 5 bits to 10ms resolution. Result in MPAC/MPAC+1.
; ============================================================================

SCALPREP	EXTEND
		QXCH	MPAC +2
		
; Call FINETIME to read hardware SCALER register. FINETIME returns double-
; precision scaler value in A/L registers, representing elapsed time since
; last TIME1/TIME2 increment with 10 microsecond resolution.

		TC	FINETIME +1
		RELINT
		DXCH	MPAC
		
; Add 5 milliseconds (BIT5) to scaler reading. This phase-aligns the 10ms
; increment bit (BIT6) of SCALER with the 10ms increment of TIME1, compensating
; for the 5ms phase offset inherent in AGC timing hardware design.

		CA	BIT5		# ADD 5 MS TO THE SCALER READING.
		TS	L
		CA	ZERO
		DAS	MPAC
		
; Mask off low 5 bits to truncate scaler to 10ms resolution, matching TIME1
; precision. Store adjusted scaler value in MPAC+1 for return to caller.

		CS	LOW5		# SET LOW 5 BITS OF (SCALER+5MS) TO ZERO
		MASK 	MPAC +1		# AND STORE RESULTS IN MPAC,+1.
		XCH	MPAC +1
		
; Test low 5 bits of original scaler. After adding BIT5, these bits equal
; 00000 (binary) only if original scaler was 10000 (the first 5ms interval
; after TIME1 increment when data is incompatible).

		MASK	LOW5		# TEST LOW 5 BITS OF SCALER FOR THE FIRST
					# INTERVAL AFTER THE T1 INCREMENT
					# (NOW = 00000, SINCE BIT 5 ADDED).
		CCS	A		# IS IT 1ST INTERVAL AFTER T1 INCREMENT
		INCR	MPAC +2		# NO
		TC	MPAC +2		# YES

# POSTAND RECOVERS TIME AFTER STANDBY.THE SCALER IS SNAPSHOTTED AND THE
# TIME1 TIME2 COUNTER IS SET TO ZERO. THE LOW 5 BITS OF THE SCALER ARE
# INSPECTED TO INSURE COMPATIBILITY BETWEEN THE SCALER READING AND THE
# CLEARING OF THE TIME COUNTER.  IT THEN COMPUTES THE DIFFERENCE IN SCALER
# VALUES (IN DP) AND ADDS THIS TO THE PREVIOUSLY SNAPSHOTTED VALUES OF
# TIME1 TIME2 AND PLACES THIS NEW TIME INTO THE TIME1 TIME2 COUNTER.

; ============================================================================
; POSTAND - STANDBY RECOVERY AND TIME RESTORATION
;
; POSTAND executes when AGC power is restored after standby mode. It computes
; the elapsed time during standby by comparing the current hardware SCALER
; value with the saved pre-standby SCALER value from SCALSAVE. This delta
; is added to the saved pre-standby TIME2/TIME1 (in TIME2SAV) to restore
; the mission elapsed time counter to its correct value.
;
; COMMENT-ONLY READERS: When the crew restored power after a standby period,
; the computer needed to determine exactly how much time had elapsed while
; it was powered down. The hardware timing circuits continued running during
; standby, so by comparing the "before" and "after" values of the hardware
; clock, the computer could precisely update its mission time counter.
;
; CODE-ALONG READERS: TIME1/TIME2 are cleared to zero. SCALPREP snapshots
; current scaler with compatibility verification. Delta scaler (post minus pre)
; is computed, adjusted for scaler overflow if needed, right-shifted 5 bits
; to align with TIME1/TIME2 resolution, added to TIME2SAV, and loaded into
; TIME2/TIME1. NODOFLAG cleared to re-enable V37 program changes. Returns to
; POOH (idle loop) to await crew commands.
; ============================================================================

		COUNT*	$$/P05

POSTAND		CS	BIT11		# RECOVER TIME AFTER STANDBY.
		EXTEND
		WAND	CHAN13		# CLEAR STANDBY ENABLE BIT
		
; Clear TIME1/TIME2 to zero as reference point. The hardware SCALER continues
; running during standby, so we'll compute elapsed time from SCALER delta.

		INHINT
		CA	ZERO
		TS	L
		DXCH	TIME2		# CLEAR TIME1TIME2
		
; Snapshot current post-standby scaler value using SCALPREP for compatibility
; verification. If incompatible window detected, retry from POSTAND+3.

		TC	SCALPREP	# STORE SCALER IN MPAC, MPAC+1
		TC	POSTAND +3	# T1,T2,SCALER NOT COMPATIBLE
		
; Compute delta scaler: (post-standby scaler) minus (pre-standby SCALSAVE).
; This difference represents elapsed time during standby in scaler units.

		EXTEND			# T1,T2 AND SCALER OK
		DCS	SCALSAVE
		DAS	MPAC		# FORM DP DIFFERENCE OF POSTSTANDBY SCALER
# Page 1337
; Shift delta right 5 bits to convert from scaler resolution (10 microseconds
; per bit) to TIME1/TIME2 resolution (320 microseconds per bit). BIT10 passed
; to SHORTMP specifies the 5-bit right shift count.

		CAF	BIT10		# MINUS PRESTANDBY SCALER AND SHIFT RIGHT
		TC	SHORTMP		# 5 TO ALIGN BITS WITH TIME1TIME2.
		
; Call TPAGREE to force triple-precision sign agreement across MPAC/MPAC+1/MPAC+2.
; This handles two's complement sign extension for negative values.

		CAF	ZERO
		TS	MPAC +2		# NEEDED FOR TP AGREE
		TC	TPAGREE		# MAKE DP DIFF AGREE
		
; Check if scaler overflowed during standby (32768 scaler ticks = 327.68 seconds).
; If delta is negative, scaler wrapped around. Add BIT10 to high word to correct
; for the overflow, producing correct positive elapsed time.

		CCS	MPAC
		TC	POSTCOM		# IF DP DIFF NET +, NO SCALER OVERFLOW
		TC	POSTCOM		# BETWEEN PRE AND POST STANDBY.
		TC	+1		# IF DP DIFF NET -, SCALER OVERFLOWED. ADD
		CAF	BIT10		# BIT 10 TO HIGH DIFF TO CORRECT.
		ADS	MPAC
		
; Add elapsed time delta to pre-standby TIME2SAV snapshot. Result is current
; mission elapsed time accounting for standby period. Load into TIME2/TIME1
; to restore time continuity. Clear NODOFLAG to re-enable V37 operations.

POSTCOM		EXTEND			# C(MPAC,+1) IS MAGNITUDE OF DELTA SCALER.
		DCA	TIME2SAV	# PRESTANDBY TIME1TIME2
		DAS	MPAC
		TC	TPAGREE		# FORCE SIGN AGREEMENT
		DXCH	MPAC		# UPDATED VALUE FOR T1,T2.
		DAS	TIME2		# LOAD UPDATED VALUE INTO T1,T2, WITH
		TC	DOWNFLAG	# CLEAR NODOFLAG
		ADRES	NODOFLAG

; Time recovery complete. Return to POOH (idle loop) to await crew program
; selection. AGC is now fully operational with correct mission elapsed time.

		TC	GOTOPOOH

