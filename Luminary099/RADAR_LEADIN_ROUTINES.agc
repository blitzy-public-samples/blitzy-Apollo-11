# Copyright:	Public domain.
# Filename:	RADAR_LEADIN_ROUTINES.agc
# Purpose: 	Part of the source code for Luminary 1A build 099.
#		It is part of the source code for the Lunar Module's (LM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	490-491
# Mod history:	2009-05-17 RSB	Adapted from the corresponding
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
; FILE: RADAR_LEADIN_ROUTINES.agc
; MODULE: Radar System Interface
; MISSION PHASE: descent/landing/ascent/rendezvous
;
; TL;DR: Manages radar sampling and data acquisition for both Landing Radar
;        (LR) and Rendezvous Radar (RR). Provides cyclic sampling loop that
;        reads altitude, velocity, range, and range-rate measurements critical
;        for lunar descent navigation and rendezvous targeting with the CSM.
;
; COMMENT-ONLY READERS: This code orchestrates the radar systems that told
;        Armstrong and Aldrin how high they were and how fast they were
;        descending during the final approach to the lunar surface.
; CODE-ALONG READERS: Study the WAITLIST-based sampling architecture and
;        variable radar channel selection through indexed CADR tables.
; ============================================================================

# Page 490
		BANK	25
		SETLOC	RRLEADIN
		BANK

		EBANK=	RSTACK

; ============================================================================
; RADAR SAMPLING LOOP
;
; The Lunar Module carries two radar systems that provide critical navigation
; data. The Landing Radar (LR) measures altitude and three-axis velocity
; during descent to the lunar surface. The Rendezvous Radar (RR) measures
; range and range-rate to the Command Module during rendezvous operations.
;
; This sampling loop runs continuously via WAITLIST, reading radar data at
; regular intervals and storing measurements in the RSTACK buffer for
; navigation processing by MEASUREMENT_INCORPORATION routines.
; ============================================================================

		COUNT*	$$/RLEAD
;
; RADSAMP is the WAITLIST-scheduled entry point for periodic radar sampling.
; During Apollo 11's descent on July 20, 1969, this routine cycled every
; second to provide Armstrong and Aldrin with updated altitude and velocity
; readings displayed on the DSKY and processed by the guidance computer.
;
RADSAMP		CCS	RSAMPDT		# TIMES NORMAL ONCE-PER-SECOND SAMPLING.
		TCF	+2
		TCF	TASKOVER	# +0 INSERTED MANUALLY TERMINATES TEST.

;
; Reschedule this routine via WAITLIST for the next sampling cycle.
; The WAITLIST provides timer-driven preemptive scheduling, ensuring
; radar data is acquired at precise intervals regardless of other AGC tasks.
;
		TC	WAITLIST
		EBANK=	RSTACK
		2CADR	RADSAMP

;
; Schedule DORSAMP job with priority 25 to perform the actual radar read.
; NOVAC finds an available executive core set and initiates the job without
; waiting. This separates timing control (WAITLIST) from execution (EXEC).
;
		CAF	PRIO25
		TC	NOVAC
		EBANK=	RSTACK
		2CADR	DORSAMP

;
; Calculate radar data index for cyclic storage in RSTACK buffer.
; RTSTDEX = RTSTLOC/2 + RTSTBASE where:
;   RTSTBASE = 0 for Rendezvous Radar (RR)
;   RTSTBASE = 2 for Landing Radar (LR)
; This indexing allows alternating storage of multiple radar measurements.
;
		CAF	BIT14		# FOR CYCLIC SAMPLING, RTSTDEX =
		EXTEND			# RTSTLOC/2 + RTSTBASE
		MP	RTSTLOC
		AD	RTSTBASE	# 0 FOR RR, 2 FOR LR.
		TS	RTSTDEX
		TCF	TASKOVER

; ============================================================================
; TRANSITION: From radar sampling scheduler to data acquisition
;
; With the timing established, the AGC now performs the actual radar hardware
; read. During Apollo 11's final approach, this routine was reading Landing
; Radar altitude every second, watching the numbers count down: 5000 feet,
; 4000 feet, 3000 feet... The crew relied on these measurements as they
; descended toward the Sea of Tranquility with limited forward visibility.
; ============================================================================

;
; DORSAMP - Execute radar sample and store in navigation buffer
;
; This is where the AGC actually reads radar hardware through I/O channels.
; The radar data flows: Hardware → RADSTALL (I/O read) → SAMPLSUM (temporary)
; → RSTACK (navigation buffer) → MEASUREMENT_INCORPORATION (Kalman filter).
;
DORSAMP		TC	VARADAR		# SELECTS VARIABLE RADAR CHANNEL.
		TC	BANKCALL
		CADR	RADSTALL

;
; Even if radar data quality is questionable, increment failure counter
; but continue processing. The navigation filter will weight unreliable
; data appropriately rather than rejecting measurements entirely.
;
		INCR	RFAILCNT	# ADVANCE FAIL COUNTER BUT ACCEPT BAD DATA

;
; DORSAMP2 - Store validated radar measurement in navigation stack
;
; Interrupts disabled (INHINT) during RSTACK update to prevent navigation
; routines from reading partially-updated data. The R77 flag check prevents
; overwriting RSTACK during rendezvous radar self-test mode.
;
DORSAMP2	INHINT
		CA	FLAGWRD5	# DON'T UPDATE RSTACK IF IN R77.
		MASK	R77FLBIT
		CCS	A
		TCF	+4

;
; Transfer double-precision radar measurement from SAMPLSUM temporary storage
; into RSTACK buffer at position indexed by RTSTLOC. Navigation programs
; (P20-P25 for rendezvous, P63-P66 for landing) will retrieve this data.
;
		DXCH	SAMPLSUM
		INDEX	RTSTLOC
		DXCH	RSTACK

;
; Cycle RTSTLOC index through radar data buffer. The buffer holds multiple
; consecutive measurements for statistical averaging and trend analysis.
; When reaching RTSTMAX, wrap back to beginning. Each radar measurement
; requires two words (double-precision), so increment by 2.
;
		CS	RTSTLOC		# CYCLE RTSTLOC.
		AD	RTSTMAX
		EXTEND

# Page 491
		BZF	+3
		CA	RTSTLOC
		AD	TWO		# STORAGE IS DP
		TS	RTSTLOC
		TCF	ENDOFJOB	# CONTINUOUS SAMPLING AND 2N TRIES - GONE.

; ============================================================================
; VARIABLE RADAR CHANNEL SELECTION
;
; The Lunar Module's two radar systems provide different measurements:
;
; RENDEZVOUS RADAR (RR) - Tracks Command Module during rendezvous:
;   - RRRANGE: Range (distance) to CSM in feet
;   - RRRDOT:  Range rate (closing velocity) in feet/second
;
; LANDING RADAR (LR) - Measures descent parameters during lunar approach:
;   - LRVELX:  Horizontal velocity component X (forward) in feet/second
;   - LRVELY:  Horizontal velocity component Y (lateral) in feet/second  
;   - LRVELZ:  Vertical velocity (descent rate) in feet/second
;   - LRALT:   Altitude above lunar surface in feet
;
; During Apollo 11's landing, the Landing Radar provided Armstrong with
; critical velocity and altitude data as Eagle descended through the final
; 40,000 feet to touchdown on July 20, 1969 at 102:45:40 mission time.
; ============================================================================

;
; VARADAR - Select appropriate radar measurement routine via indexed table
;
; Uses RTSTDEX (calculated earlier from RTSTLOC and RTSTBASE) to select
; which radar channel to read. The RDRLOCS table contains CADRs (coded
; addresses) pointing to specific radar interface routines in other banks.
;
VARADAR		CAF	ONE		# WILL BE SENT TO RADAR ROUTINE IN A BY
		TS	BUF2		# SWCALL.
		INDEX	RTSTDEX
		CAF	RDRLOCS
		TCF	SWCALL		# NOT TOUCHING Q.

;
; RDRLOCS - Table of radar measurement routine addresses
;
; Each CADR points to a specific radar channel interface routine that
; performs hardware I/O to read one measurement type. The index determines
; which radar system (RR vs LR) and which measurement (range, velocity, etc).
;
RDRLOCS		CADR	RRRANGE		# =0  Rendezvous Radar range
		CADR	RRRDOT		# =1  Rendezvous Radar range rate
		CADR	LRVELX		# =2  Landing Radar velocity X
		CADR	LRVELY		# =3  Landing Radar velocity Y
		CADR	LRVELZ		# =4  Landing Radar velocity Z (descent rate)
		CADR	LRALT		# =5  Landing Radar altitude


