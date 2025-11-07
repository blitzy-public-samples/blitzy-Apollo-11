# Copyright:	Public domain.
# Filename:	JET_SELECTION_LOGIC.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1039-1062
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
; FILE: JET_SELECTION_LOGIC.agc
; MODULE: TVCDAPS Subsystem (Control Systems)
; MISSION PHASE: all-phases
;
; TL;DR: RCS jet selection algorithms determining optimal thruster pair selection
;        for commanded attitude changes. Implements thruster coupling analysis,
;        failed jet isolation, propellant balancing across quad systems, and
;        momentum management to minimize fuel consumption while achieving control
;        objectives throughout Apollo 11 mission.
;
; COMMENT-ONLY READERS: This program chose which steering jets to fire for each
;        maneuver, avoiding broken jets and saving fuel.
; CODE-ALONG READERS: Study jet selection optimization algorithms, coupling effects,
;        failed jet isolation logic, quad propellant balancing, momentum management.
; ============================================================================

# Page 1039
		BANK	21
		SETLOC	DAPS4
		BANK

		COUNT	17/DAPJS

		EBANK=	KMPAC

; ============================================================================
; TRANSLATION COMMAND PROCESSING
;
; The Command Module crew controls spacecraft orientation using the hand
; controller. When the astronauts command translations (linear movements in
; X, Y, or Z directions), this routine decodes those commands and determines
; which RCS thruster pairs to fire. During Apollo 11, this system enabled
; precise maneuvers for docking with the Lunar Module and course corrections
; during translunar and transearth flight.
;
; Technical: Reads hardware channel 31 for translation hand controller inputs,
; decodes bit patterns into X/Y/Z translation indices, handles LEM-attached vs
; LEM-detached configurations for different mass properties.
; ============================================================================

# EXAMINE CHANNEL 31 FOR TRANSLATION COMMANDS

; JETSLECT - Main entry point for jet selection logic
; Called by digital autopilot when attitude or translation commands are active.
; Decodes hand controller inputs from hardware channel 31 and initiates the
; jet selection and timing calculation process.
;
; Register usage: A = accumulator for bit manipulation and command decoding
;                 L = link register (saved BANKRUPT for interrupt return)
;                 Q = return address (saved in QRUPT)

JETSLECT	LXCH	BANKRUPT
		CAF	DELTATT3	# = 60 MS  RESET TO EXECUTE PHASE1
		AD	T5TIME
		TS	TIME5
		TCF	+3
		CAF	DELATT20	# = 20 MS  TO ASSURE A T5RUPT
		TS	TIME5
		CAF	=14MS		# RESET T6 TO INITIALIZE THE JET CHANNELS
		TS	TIME6		# IN 14 MS
		CAF	NEGMAX
		EXTEND
		WOR	CHAN13
		EXTEND
		QXCH	QRUPT
		
; Read translation hand controller from channel 31.
; The astronaut's hand controller generates bit patterns indicating desired
; translation direction: +X/-X (left/right), +Y/-Y (up/down), +Z/-Z (forward/back).
; Bit pattern 7700 octal (binary 111 111 000 000) masks the translation bits.
		
		CAF	XLNMASK		# = 7700 OCT
		EXTEND			# EXAMINE THE TRANSLATION
		RXOR	CHAN31		# HAND CONTROLLER
		MASK	XLNMASK
		EXTEND
		BZF	NOXLNCMD
; Decode translation command bits into indices for table lookup.
; Each axis (X, Y, Z) produces an index: 0 = no command, 1 = positive, 2 = negative.
; These indices select which thruster pairs fire from the four RCS quads (A, B, C, D).
; The Command Module has 4 RCS quads spaced 90° apart around the service module,
; each quad containing 4 jets for redundancy and 6-axis control authority.

		TS	T5TEMP
		EXTEND
		MP	BIT9
		MASK	THREE
		TS	XNDX1		# AC QUAD  X-TRANSLATION INDEX
		TS	XNDX2		# BD QUAD  X-TRANSLATION INDEX
		CA	T5TEMP
		EXTEND			# 1 = + XLN
		MP	BIT7		# 2 = - XLN
		MASK	THREE		# 3 = NO XLN
		TS	YNDX		# Y-TRANSLATION INDEX

		CA	T5TEMP
		EXTEND
		MP	BIT5
		MASK	THREE
		TS	ZNDX		# Z-TRANSLATION INDEX

; Check spacecraft configuration (LEM attached or detached).
; Mass properties differ significantly: CSM alone ~30,000 lbs, CSM+LM ~90,000 lbs.
; Different filter gains optimize control response for each configuration.

		CA	DAPDATR1	# SET ATTKALMN TO PICK UP FILTER GAINS FOR
		MASK	BIT14		# TRANSLATIONS.
		EXTEND			# CHECK DAPDATR1 BIT 14 FOR LEM ATTACHED.
# Page 1040
		BZF	NOLEM
		CS	THREE		# IF LEM IS ON, SET ATTKALMN = -3
		TCF	+2
NOLEM		CS	TWO		# IF LEM IS OFF, SET ATTKALMN = -2.
		TS	ATTKALMN
		CCS	XTRANS		# (+, -1, 0)
		TS	XNDX1		# USING BD-X  ZERO XNDX1
		TCF	PWORD
		TS	XNDX2		# USING AC-X  ZERO XNDX2
		TCF	PWORD
XLNMASK		OCT	7700

DELTATT3	DEC	16378		# = 60 MS
DELATT20	DEC	16382		# = 20 MS

NOXLNCMD	TS	XNDX1		# ZERO ALL REQUESTS FOR TRANSLATION
		TS	XNDX2
		TS	YNDX
		TS	ZNDX

; ============================================================================
; TRANSITION: From translation processing to rotational command processing
;
; With translation commands decoded (or zeroed if none present), the jet
; selection logic now processes rotational commands: pitch, yaw, and roll.
; During critical mission phases like docking with Eagle or Service Module
; separation for reentry, precise rotational control was essential. The
; autopilot commands torques; this logic converts torques into specific
; jet pairs and firing durations.
; ============================================================================

# PITCH COMMANDS  TIMING(NO X-TRANS, NO QUAD FAILS) 32MCT

; PITCH COMMAND PROCESSING
; The digital autopilot calculates required pitch torque (TAU1) to achieve
; commanded attitude. This section converts pitch torque into jet selection
; by looking up the optimal jet pair from PYTABLE, checking for failed jets,
; and calculating required on-time duration. The CSM RCS has multiple jets
; capable of pitch control; this logic selects the most fuel-efficient combination.

PWORD		CCS	TAU1		# CHECK FOR PITCH COMMANDS
		CAF	ONE
		TCF	+2		#  0 = NO PITCH
		CAF	TWO		# +1 =  + PITCH
		TS	PINDEX		# +2 =  - PITCH

; Failed jet isolation logic.
; The CSM RCS has four quads (A, B, C, D) spaced 90° apart. If a jet fails
; (detected by pressure sensors or crew input), the system must avoid that quad
; and use jets from the remaining three quads. This preserves control authority
; while preventing unwanted torques from failed jets. During Apollo 11, all jets
; functioned nominally, but this logic was critical for contingency operations.

		CCS	RACFAIL		# FLAG FOR REAL AC QUAD FAILURES
		TCF	AFAILP
		TCF	TABPCOM		# 0 = NO REAL AC FAILURES
		TCF	CFAILP		# + = A QUAD FAILED
		TCF	TABPCOM		# - = C QUAD FAILED
					# IF FAILURES ARE PRESENT IGNORE
					# X-TRANSLATIONS ON THIS AXIS

AFAILP		CAF	NINE		# IF FAILURE IS PRESENT 1JET OPERATION
		TCF	TABPCOM +2	# IS ASSUMED.  IGNORE X-TRANSLATION
CFAILP		CAF	TWELVE
		TCF	TABPCOM +2

XLNNDX		DEC	0		# INDICES FOR TRANSLATION COMMANDS
		DEC	3		# FOR USE IN TABLE LOOK UP
		DEC	6
		DEC	0

TWELVE		=	OCT14
# TABLE LOOK UP FOR PITCH COMMANDS WITH AND WITHOUT X-TRANSLATION AND AC QUAD FAILURES PRESENT.
# BITS 9, 10 CONTAIN THE NUMBER OF PITCH JETS USED TO PERFORM THE PITCH ROTATION
# Page 1041

; PITCH JET TABLE LOOKUP
; The PYTABLE contains pre-computed jet selection patterns for all combinations of:
; pitch direction (+/-), translation direction (+/-/0), and failed quads. The
; index is computed from PINDEX (pitch direction), XNDX1 (translation), and any
; quad failure offsets. The table entry encodes: which jets fire (bits 1-4 for
; pitch, bits 5-8 for yaw), and how many jets (bits 9-10 pitch, 11-12 yaw).
; This lookup minimizes computational load during real-time control.

TABPCOM		INDEX	XNDX1
		CA	XLNNDX
		AD	PINDEX
		INDEX	A
		CA	PYTABLE
		MASK	PJETS		# =1417 OCT
		TS	PWORD1
		EXTEND
		MP	BIT7
		TS	NPJETS		# = NO. OF PITCH JETS

# YAW JET COMMANDS  TIMING(NO X-TRANS, NO QUAD FAILURES)  32MCT

; YAW COMMAND PROCESSING
; Similar to pitch processing, but using TAU2 (yaw torque) and checking RBDFAIL
; for B/D quad failures. Yaw control was critical during transposition and docking
; (after TLI) when Columbia turned around to extract Eagle from S-IVB, and during
; reentry when precise yaw attitude ensured the crew capsule splashed down in the
; Pacific recovery zone. The logic structure parallels pitch for code efficiency.

YWORD		CCS	TAU2		# CHECK FOR YAW COMMANDS
		CAF	ONE
		TCF	+2
		CAF	TWO
		TS	YINDEX		# YAW ROTATION INDEX

		CCS	RBDFAIL		# FLAG FOR B OR D QUAD FAILURES
		TCF	BFAILY		# 0 = NO BD FAILURE
		TCF	TABYCOM		# + = B QUAD FAILED
		TCF	DFAILY		# - = D QUAD FAILED
		TCF	TABYCOM

BFAILY		CAF	NINE
		TCF	TABYCOM +2
DFAILY		CAF	TWELVE
		TCF	TABYCOM +2

# Page 1042
# TABLE FOR PITCH(YAW) COMMANDS
# BITS 4,3,2,1 = PITCH, X-TRANSLATION JETS SELECTED
# BITS     10,9 = NO. PITCH JETS USED TO PERFORM ROTATION
# BITS 8,7,6,5 = YAW, X-TRANSLATION JETS SELECTED
# BITS 12,11 : NO. YAW JETS USED TO PERFORM ROTATION

; PYTABLE - Combined Pitch/Yaw Jet Selection Table
; This 15-entry table encodes optimal jet selection for all command combinations.
; Each octal word contains packed information:
;   Bits 1-4: Pitch axis jet pattern (which of 4 pitch jets to fire)
;   Bits 5-8: Yaw axis jet pattern (which of 4 yaw jets to fire)
;   Bits 9-10: Number of pitch jets used (0, 1, or 2)
;   Bits 11-12: Number of yaw jets used (0, 1, or 2)
; Table index = BIAS + rotation direction (0, +1, -2) + translation (0, 3, 6)
; Example: Entry 0231 octal = rotation=0, translation=+, uses jets in specific pattern
; This table was optimized to minimize fuel consumption while achieving required torques.

					# ROT	TRANS	QUAD	BIAS
PYTABLE		OCT	0		# 0	0		0
		OCT	5125		# +	0		0
		OCT	5252		# -	0		0
		OCT	0231		# 0	+		3
		OCT	2421		# +	+		3
		OCT	2610		# -	+		3
		OCT	0146		# 0	-		6
		OCT	2504		# +	-		6
		OCT	2442		# -	-		6
		OCT	0		# 0		A(B)	9
		OCT	2421		# +		A(B)	9
		OCT	2442		# -		A(B)	9
		OCT	0		# 0		C(D)	12
		OCT	2504		# +		C(D)	12
		OCT	2610		# -		C(D)	12

# MASKS FOR PITCH AND YAW COMMANDS

PJETS		OCT	1417
YJETS		OCT	6360

# TABLE LOOK UP FOR YAW COMMANDS WITH AND WITHOUT X-TRANSLATION AND AC QUAD FAILURES PRESENT
# BITS 11, 12 CONTAIN THE NUMBER OF YAW JETS USED TO PERFORM THE YAW ROTATION

; YAW JET TABLE LOOKUP
; Uses the same PYTABLE as pitch, but extracts the yaw jet pattern (bits 5-8)
; and yaw jet count (bits 11-12) using YJETS mask (6360 octal). This shared
; table approach saves memory in the 36K fixed storage. The table lookup
; accounts for X-translation direction (XNDX2) and yaw rotation direction.

TABYCOM		INDEX	XNDX2
		CA	XLNNDX
		AD	YINDEX
		INDEX	A
		CA	PYTABLE
		MASK	YJETS		# = 6360 OCT
		TS	YWORD1
		EXTEND
		MP	BIT5
		TS	NYJETS		# NO. OF YAW JETS USED TO PERFORM ROTATION

# Page 1043
# ROLL COMMANDS  TIMING(NO Y,Z TRANS, NO QUAD FAILS)  45MCT

; ROLL COMMAND PROCESSING
; Roll control is more complex than pitch/yaw because the CSM can use either
; AC quads or BD quads for roll, depending on propellant balance and failures.
; The ACORBD flag determines which quad pair to use. Roll was critical during
; Passive Thermal Control ("barbecue roll") in translunar/transearth coast to
; distribute solar heating evenly across the spacecraft. During Apollo 11, this
; slow roll (3 revolutions per hour) prevented overheating on the sun-facing side.

RWORD		CCS	TAU		# CHECK FOR ROLL COMMANDS
		CAF	ONE
		TCF	+2
		CAF	TWO
		TS	RINDEX

		CCS	ACORBD		# FLAG FOR AC OR BD QUAD SELECTION FOR
		TCF	BDROLL		# ROLL COMMANDS
		TCF	BDROLL		# +, +0 = BD ROLL
		TCF	+1		# -, -0 = AC ROLL

ACROLL		CCS	RACFAIL		# CHECK FOR REAL FAILURES
		TCF	RAFAIL		# ON AC QUADS
		TCF	RXLNS
		TCF	RCFAIL
		TCF	RXLNS

RAFAIL		CAF	NINE		# QUAD FAILURE WILL GET
		TCF	TABRCOM		# 1-JET OPERATION
RCFAIL		CAF	TWELVE
		TCF	TABRCOM

; XLN1NDX - Translation Index Table
; Converts translation commands into table offsets for combined rotation+translation
; jet selection. This small lookup table avoids complex arithmetic operations.

XLN1NDX		DEC	0
		DEC	1		# INDECES FOR TRANSLATION
		DEC	2
		DEC	0

# TABLE LOOK UP FOR AC-ROLL COMMANDS WITH AND WITHOUT Y-TRANSLATION AND ACQUAD FAILURES PRESENT
# BITS 9,10,11 CONTAIN THE MAGNITUDE AND DIRECTION OF THE ROLL

; AC-ROLL JET TABLE LOOKUP
; Selects AC quad jets for roll commands, accounting for Y-translation requests.
; Uses RTABLE to determine optimal jet firing pattern. The ACRJETS mask (3760 octal)
; extracts the AC quad roll jet bit pattern. This provides primary roll control
; authority when AC quads are healthy and selected by propellant management logic.

RXLNS		INDEX	YNDX		# NO AC QUAD FAILURES
		CA	XLNNDX		# INCLUDE +,-,0, Y-TRANSLATION
TABRCOM		AD	RINDEX
		INDEX	A
		CA	RTABLE
		MASK	ACRJETS		# = 3760 OCT
		TS	RWORD1

# CHECK FOR Z-TRANSLATIONS ON BD

; BD Z-TRANSLATION COMPATIBILITY CHECK
; Z-axis translation uses BD quad jets, which can also produce roll torques.
; This section determines if Z-translation can be safely executed without
; interfering with commanded roll attitude. The logic ensures that firing
; BD jets for Z-translation won't produce unwanted roll disturbances.

BDZCHECK	CA	ZNDX
		EXTEND
		BZMF	NOBDZ		# NO Z-TRANSLATION

# Page 1044
# 	TABLE LOOK UP FOR BD Z-TRANSLATION WITH AND WITHOUT REAL BD QUAD FAILURES.  Z-TRANSLATION WILL BE POSS-
# IBLE AS LONG AS ROLL COMMANDS CAN BE SATISFIED WITH THE AC ROLL JETS.  CRITERION..  IF THE RESULTANT NET ROLL
# COMMANDS = 0 (WITH Z-TRANSLATION) AND IF TAU = 0, THEN INCLUDE THE BD Z-TRANSLATION COMMANDS.  IF THE RESULTANT
# ROLL COMMAND = 0, AND IF TAU NZ, THEN IGNORE THE BD Z-TRANSLATION

; BD Z-TRANSLATION JET SELECTION WITH ROLL COUPLING ANALYSIS
; Z-translation jets on BD quads inevitably produce some roll coupling torque.
; This logic checks if adding Z-translation would cancel out existing roll commands
; (net roll = 0). If TAU = 0 (no roll rotation commanded), Z-translation is accepted
; even if it produces some roll coupling. If TAU ≠ 0 (roll rotation active), reject
; Z-translation if it would produce unwanted roll. This prioritizes attitude control
; over translation when conflicts arise. Critical during docking when precise attitude
; maintenance was essential while translating toward the LM.

		CCS	RBDFAIL
		CAF	THREE
		TCF	+2
		CAF	SIX
		INDEX	ZNDX
		AD	XLN1NDX
		INDEX	A
		CA	YZTABLE
		MASK	BDZJETS		# = 3417 OCT
		AD	RWORD1		# ADD TO ROLL COMMANDS
		TS	T5TEMP		# IF POSSIBLE.  MUST CHECK TAU FIRST

		EXTEND
		MP	BIT7		# DETERMINE THE NET ROLL COMMAND WITH
		AD	=-4		# Z-TRANSLATION ADDED ON
		TS	NRJETS		# NET NO. OF +,- ROLL JETS ON
		EXTEND
		BZF	TAUCHECK

ACRBDZ		CA	T5TEMP		# Z-TRANSLATION ACCEPTED EVEN THO WE MAY
		TS	RWORD1		# HAVE INTRODUCED AN UNDESIREABLE ROLL
		TCF	ROLLTIME	# BRANCH TO JET ON-TIME CALCULATIONS

; TAU CHECK - Decide Z-translation acceptance based on roll command state
; TAU = 0: No roll rotation commanded, accept Z-translation with coupling torque
; TAU ≠ 0: Roll rotation active, reject Z-translation to avoid interference

TAUCHECK	CCS	TAU
		TCF	NOBDZ
		TCF	ACRBDZ
		TCF	NOBDZ
		TCF	ACRBDZ

NOBDZ		CA	RWORD1		# Z-TRANSLATION NOT ACCEPTED
		EXTEND
		MP	BIT7
		AD	=-2
		TS	NRJETS
		TCF	ROLLTIME	# BRANCH TO JET ON-TIME CALCULATION

# Page 1045
# BD QUAD SELECTION FOR ROLL COMMANDS
;
; BD ROLL QUAD SELECTION
; When BD quads are selected for roll, this section checks for failures in
; the BD quad pair. If failures exist, the logic falls back to single-jet
; operation (1-jet mode) to maintain some roll authority. The RBDFAIL flag
; indicates the health status of the BD quad pair.

BDROLL		CCS	RBDFAIL
		TCF	RBFAIL
		TCF	RZXLNS
		TCF	RDFAIL
		TCF	RZXLNS
RBFAIL		CAF	NINE
		TCF	TABRZCMD
RDFAIL		CAF	TWELVE
		TCF	TABRZCMD

; BD ROLL WITH Z-TRANSLATION
; This section handles BD roll commands when there are no BD quad failures.
; It combines roll and Z-translation commands by indexing into RTABLE with
; both the roll index and Z-translation index, then masks for BD jets only.

RZXLNS		INDEX	ZNDX		# NO BD FAILURES
		CA	XLNNDX		# +,-,0 Z-TRANSLATION PRESENT
TABRZCMD	AD	RINDEX
		INDEX	A
		CA	RTABLE
		MASK	BDRJETS		# = 34017 OCT
		TS	RWORD1

; AC Y-TRANSLATION CHECK
; When BD roll is selected, the system can also satisfy Y-translation requests
; using AC quads, provided AC quads are not failed. This section checks for
; Y-translation commands and AC quad health. If AC quads are healthy and
; Y-translation is requested, it combines BD roll jets with AC Y-translation
; jets from the YZTABLE. The net roll torque is calculated to ensure attitude
; control is not compromised by the combined firing.

ACYCHECK	CA	YNDX		# ANY Y-TRANSLATION
		EXTEND
		BZF	NOACY		# NO Y-TRANSLATION
		CCS	RACFAIL
		CAF	THREE
		TCF	+2
		CAF	SIX
		INDEX	YNDX
		AD	XLN1NDX
		INDEX	A
		CA	YZTABLE
		MASK	ACYJETS		# = 34360 OCT
		AD	RWORD1
		TS	T5TEMP
		EXTEND			# FOR EXPLANATION SEE CODING ON RTABLE
		MP	BIT4
		AD	=-4
		TS	NRJETS		# NO. OF NET ROLL JETS
		EXTEND
		BZF	TAUCHCK		# IF NRJETS = 0

; BD ROLL WITH AC Y-TRANSLATION ACCEPTED
; The combined BD roll + AC Y-translation command is accepted and stored.
; Control proceeds to calculate jet on-times for this combined maneuver.

BDRACZ		CA	T5TEMP		# Y-TRANSLATION ACCEPTED
		TS	RWORD1
		TCF	ROLLTIME	# BRANCH TO JET ON-TIME CALCULATIONS

; TAU CHECK FOR ATTITUDE PRIORITY
; Similar to BDZCHECK, this checks whether attitude control (TAU) should
; override the combined translation command. If TAU demands urgent attitude
; correction (TAU positive or zero), Y-translation is rejected to preserve
; pure roll authority. Otherwise, the combined command is accepted.

TAUCHCK		CCS	TAU
		TCF	NOACY
		TCF	BDRACZ
		TCF	NOACY
		TCF	BDRACZ

# Page 1046
; NO AC Y-TRANSLATION
; Y-translation is not accepted (either not requested or rejected by TAU check).
; Roll jets are commanded without any translation component. Net jet count
; is calculated for pure roll operation.

NOACY		CA	RWORD1		# Y-TRANSLATION NOT ACCEPTED
		EXTEND
		MP	BIT4
		AD	=-2
		TS	NRJETS
		TCF	ROLLTIME

# Page 1047
# 		TABLE FOR ROLL, Y AND Z-TRANSLATION COMMANDS

# 	EITHER AC OR BD ROLL MAY BE SELECTED.  IF AC ROLL IS SELECTED, Y-TRANSLATIONS MAY BE SATISFIED SIMULTANEOUSLY
# PROVIDED THAT THERE ARE NO AC QUAD FAILURES.  IF THERE ARE AC FAILURES, Y-TRANSLATION COMMANDS WILL BE IGNORED,
# IN WHICH CASE THE ASTRONAUT SHOULD SWITCH TO BD ROLL.
# 	IF BDROLL IS SELECTED, Z-TRANSLATIONS MAY BE SATISFIED SIMULTANEOUSLY PROVIDED THAT THERE ARE NO BD QUAD
# FAILURES.  IF THERE ARE BD FAILURES, Z-TRANSLATION COMMANDS WILL BE IGNORED, IN WHICH CASE THE ASTRONAUT SHOULD
# SWITCH TO AC ROLL.
# 	NOTE THAT IF ONE QUAD FAILS (E.G. B FAILED), Z-TRANSLATION IS STILL POSSIBLE AND THAT THE UNDESIRABLE ROLL
# INTRODUCED BY THIS TRANSLATION WILL BE COMPENSATED BY THE TWO AC ROLL JETS ACTUATED BY THE AUTOPILOT LOGIC.

# 		WORD MAKE UP....RTABLE

# 	TWO WORDS, CORRESPONDING TO AC OR BD ROLL SELECTION, HAVE BEEN COMBINED INTO ONE TABLE.  THE WORD CORRESPOND-
# ING TO AC ROLL HAS THE FOLLOWING INTERPRETATION..
#	BITS 9,10,11 ARE CODED TO GIVE THE NET ROLL TORQUE FOR THE WORD SELECTED.  THE CODING IS..
#			BIT NO. 11  10   9		NO. OF ROLL JETS

#				 0   0   0			-2
#				 0   0   1			-1
#				 0   1   0			 0
#				 0   1   1			+1
#				 1   0   0			+2

# 	THIS WORD MAY THEN BE ADDED TO THE WORD SELECTED FROM THE YZ-TRANSLATION TABLE, WHICH HAS THE SAME TYPE OF
# CODING AS ABOVE, AND THE NET ROLL DETERMINED BY SHIFTING THE RESULTANT WORD RIGHT 8 PLACES AND SUBTRACTING FOUR.

# 	THE WORD CORRESPONDING TO THE BD ROLL HAS A SIMILAR INTERPRETATION, EXCEPT THAT BITS 12, 13, 14 ARE CODED
# (AS ABOVE) TO GIVE THE NET ROLL TORQUE.

					# ROLL 		TRANS		QUADFAIL	BIAS

RTABLE		OCT	11000		#   0						  0
		OCT	22125		#   +						  0
		OCT	00252		#   -						  0
		OCT	11231		#   0		+Y(+Z)				  3
		OCT	15421		#   +		+Y(+Z)				  3
		OCT	04610		#   -		+Y(+Z)				  3
		OCT	11146		#   0		-Y(-Z)				  6
		OCT	15504		#   +		-Y(-Z)				  6
		OCT	04442		#   -		-Y(-Z)				  6
		OCT	11000		#   0				  A(B)		  9
		OCT	15504		#   +				  A(B)		  9
		OCT	04610		#   -				  A(B)		  9
		OCT	11000		#   0				  C(D)		 12
		OCT	15421		#   +				  C(D)           12
		OCT	04442		#   -				  C(D)		 12

# Page 1048
# RTABLE MASKS -

ACRJETS		OCT	03760
BDRJETS		OCT	34017

# Page 1049
#		Y, Z TRANSLATION TABLE

# 	ONCE AC OR BD ROLL IS SELECTED THE QUAD PAIR WHICH IS NOT BEING USED TO SATISFY THE ROLL COMMANDS MAY BE
# USED TO SATISFY THE REMAINING TRANSLATION COMMANDS.  HOWEVER, WE MUST MAKE SURE THAT ROLL COMMANDS ARE SATISFIED
# WHEN THEY OCCUR.  THEREFORE, THE Y-Z TRANSLATIONS FROM THIS TABLE WILL BE IGNORED IF THE NET ROLL TORQUE OF THE
# COMBINED WORD IS ZERO AND THE ROLL COMMANDS ARE NON-ZERO.  THIS SITUATION WOULD OCCUR, FOR EXAMPLE, IF WE EN-
# COUNTER SIMULTANEOUS +R +Y -Z COMMANDS AND A QUAD D FAILURE WHILE USING AC FOR ROLL.
# 	TO FACILITATE THE LOGIC, THE Y-Z TRANSLATION TABLE HAS BEEN CODED IN A MANNER SIMILAR TO THE ROLL TABLE
# ABOVE.
# 	BITS 9,10,11 ARE CODED TO GIVE THE NET ROLL TORQUE INCURRED BY Z-TRANSLATIONS.  THE WORD SELECTED CAN THEN BE
# ADDED TO THE AC-ROLL WORD AND THE RESULTANT ROLL TORQUE DETERMINED FROM THE COMBINED WORD.  SIMILIARLY BITS
# 12,13,14 ARE CODED TO GIVE THE NET ROLL TORQUE INCURRED BY Y-TRANSLATIONS WHEN BD-ROLL IS SELECTED.

					# TRANSLATION	QUADFAIL	BIAS
					#
YZTABLE		OCT	11000		# 	0			0
		OCT	11231		#    +Z(+Y)			0
		OCT	11146		#    -Z(-Y)			0
		OCT	11000		#	0	  B(A)		3
		OCT	04610		#    +Z(+Y)	  B(A)		3
		OCT	15504		#    -Z(-Y)	  B(A)		3
		OCT	11000		#  	0	  D(C)		6
		OCT	15421		#    +Z(+Y)	  D(C)		6
		OCT	04442		#    -Z(-Y)	  D(C)		6

# YZ-TABLE MASKS-

BDZJETS		OCT	03417
ACYJETS		OCT	34360

# ADDITIONAL CONSTANTS

=-2		=	NEG2
=-4		=	NEG4

# Page 1050
# 		CALCULATION OF JET ON-TIMES
#
# 	THE ROTATION COMMANDS (TAU:S), WHICH WERE DETERMINED FROM THE JET SWITCHING LOGIC ON THE BASIS OF SINGLE JET
# OPERATION, MUST NOW BE UPDATED BY THE ACTUAL NUMBER OF JETS TO BE USED IN SATISFYING THESE COMMANDS.  TAU MUST
# ALSO BE DECREMENTED ACCORDING TO THE EXPECTED TORQUE GENERATED BY THE NEW COMMANDS ACTING OVER THE NEXT T5 INT-
# ERVAL.
# 	IN ORDER TO MAINTAIN ACCURATE KNOWLEDGE OF VEHICLE ANGULAR RATES, WE MUST ALSO PROVIDE EXPECTED FIRING TIMES
# (DFT:S, ALSO IN TERMS OF 1-JET OPERATION) FOR THE RATE FILTER.
# 	NOTE THAT TRANSLATIONS CAN PRODUCE ROTATIONS EVEN THOUGH NO ROTATIONS WERE CALLED FOR.  NEVERTHELESS, WE MUST
# UPDATE DFT.
# 	WHEN THE ROTATIONS HAVE FINISHED, WE MUST PROVIDE CHANNEL INFORMATION TO THE T6 PROGRAM TO CONTINUE ON WITH
# THE TRANSLATIONS.  THIS WILL BE DONE IN THE NEXT SECTION.  HOWEVER, TO INSURE THAT JETS ARE NOT FIRED FOR LESS
# THAN A MINIMUM IMPULSE (14MS), ALL JET CHANNEL COMMANDS WILL BE HELD FIXED FROM THE START OF THE T5 PROGRAM FOR
# ATLEAST 14MS UNTIL THE INITIALIZATION OF NEW COMMANDS.  MOREOVER, A 14MS ON-TIME WILL BE ADDED TO ANY ROTATIONAL
# COMMANDS GENERATED BY THE MANUAL CONTROLS OR THE JET SWITCHING LOGIC, AND ALL TRANSLATION COMMANDS WILL BE
# ACTIVE FOR ATLEAST ONE CYCLE OF THE T5 PROGRAM (.1SEC)

# 		PITCH JET ON-TIME CALCULATION

; ============================================================================
; CALCULATION OF JET ON-TIMES
;
; Having selected the appropriate jet combinations for pitch, yaw, and roll
; maneuvers, the program now calculates how long each jet firing should last.
; These on-time calculations convert commanded torques (stored in TAU1, TAU2,
; TAU3) into jet firing durations (BLAST1, BLAST2, BLAST), accounting for
; thruster effectiveness and minimum impulse requirements.
;
; For comment-only readers: The computer now determines how long to fire each
; thruster pair to achieve the commanded rotation rates while minimizing fuel
; consumption. Each axis is handled separately.
;
; For code-along readers: The on-time calculations use the selected jet
; effectiveness values (NJET indexed by NPJETS, NYJETS, NRJETS) to convert
; torque commands into time durations. Maximum on-times are limited to 0.1
; seconds to allow smooth transitions when switching from RCS to main engine
; thrust vector control (TVC).
; ============================================================================

; PITCH JET ON-TIME CALCULATION
; Calculate firing duration for pitch axis thrusters based on commanded torque.

PITCHTIM	CCS	TAU1
		TCF	PTAUPOS
		TCF	+2
		TCF	PTAUNEG
		TS	DFT1		# NO PITCH ROTATION
		TCF	PBYPASS		# COMMANDS

; For negative pitch torque, complement jet selection to reverse thrust direction.
PTAUNEG		CS	NPJETS
		TS	NPJETS

; Calculate pitch jet on-time: multiply torque command by jet effectiveness.
; NJET table contains effectiveness values for different jet pairs.
PTAUPOS		CA	TAU1
		EXTEND
		INDEX	NPJETS		; Use selected jet pair effectiveness
		MP	NJET		; Multiply torque by 1/effectiveness
		TS	BLAST1		; Store calculated on-time
		AD	=-.1SEC
		EXTEND
		BZMF	AD14MSP		; Branch if on-time <= 0.1 seconds

; Pitch maneuver requires more than 0.1 second duration.
; Limit this cycle to 0.1 sec to allow smooth RCS-to-TVC transition.
		INDEX	NPJETS
		CA	DFTMAX		# THE PITCH ON-TIME IS GREATER THAN .1 SEC
		TS	DFT1		; Save torque for rate filter update
		COM
		ADS	TAU1		# UPDATE TAU1 (reduce remaining torque)
		CAF	=+.1SEC		# LIMIT THE LENGTH OF PITCH ROTATION
		TS	BLAST1		# COMMANDS TO 0.1 SEC SO THAT ONLY
		TCF	ASMBLWP		# X-TRANSLATIONS WILL CONTINUE ON SWITCH
					# OVER TO TVC
; Check if on-time is less than minimum impulse time (14 ms).
; RCS thrusters require minimum 14 ms firing for reliable ignition.
AD14MSP		CS	BLAST1		# SEE IF JET ON TIME IS LESS THAN
		AD	=14MS		# MINIMUM IMPULSE TIME
		EXTEND
		BZMF	PBLASTOK	# IF SO LIMIT MINIMUM ON TIME TO 14 MS
		CAF	=14MS		; Enforce 14 ms minimum for thruster reliability
# Page 1051
		TS	BLAST1

; Pitch on-time is acceptable. Calculate actual torque applied and update filters.
PBLASTOK	CA	BLAST1
		EXTEND			# THE PITCH COMMANDS WILL BE COMPLETED
		MP	NPJETS		# WITHIN THE TS-CYCLE TIME
		LXCH	DFT1		# FOR USE IN UPDATING RATE FILTER
		TS	TAU1		# ZERO TAU1 (ACC CONTAINS ZERO)
		TCF	ASMBLWP

# Page 1052
# YAW JET ON-TIME CALCULATION

; Calculate firing duration for yaw axis thrusters based on commanded torque.
; Logic mirrors pitch calculation but uses yaw-specific parameters.
YAWTIME		CCS	TAU2
		TCF	YTAUPOS
		TCF	+2
		TCF	YTAUNEG
		TS	DFT2		# NO YAW ROTATION COMMANDS
		TCF	YBYPASS

; For negative yaw torque, complement jet selection to reverse thrust direction.
YTAUNEG		CS	NYJETS
		TS	NYJETS

; Calculate yaw jet on-time: multiply torque by jet effectiveness.
YTAUPOS		CA	TAU2
		EXTEND
		INDEX	NYJETS		; Use selected yaw jet pair effectiveness
		MP	NJET		; Multiply torque by 1/effectiveness
		TS	BLAST2		; Store calculated on-time
		AD	=-.1SEC
		EXTEND
		BZMF	AD14MSY		; Branch if on-time <= 0.1 seconds

; Yaw maneuver requires more than 0.1 second duration.
; Limit this cycle to 0.1 sec for smooth RCS-to-TVC transition.
		INDEX	NYJETS
		CA	DFTMAX		# YAW COMMANDS WILL LAST LONGER THAN .1SEC
		TS	DFT2		; Save torque for rate filter update
		COM
		ADS	TAU2		# DECREMENT TAU2 (reduce remaining torque)
		CAF	=+.1SEC		# LIMIT THE LENGTH OF YAW ROTATION COMMAND
		TS	BLAST2		# TO 0.1 SEC SO THAT ONLY X-TRANSLATION
		TCF	ASMBLWY		# WILL CONTINUE ON SWITCH OVER TO TVC

; Check if yaw on-time is less than minimum impulse time (14 ms).
AD14MSY		CS	BLAST2		# SEE IF JET ON-TIME LESS THAN
		AD	=14MS		# MINIMUM IMPULSE TIME
		EXTEND
		BZMF	YBLASTOK	# IF SO, LIMIT MINIMUM ON-TIME TO 14 MS
		CAF	=14MS		; Enforce 14 ms minimum for thruster reliability
		TS	BLAST2

; Yaw on-time is acceptable. Calculate actual torque applied and update filters.
YBLASTOK	CA	BLAST2		# YAW COMMANDS WILL BE COMPLETED WITHIN
		EXTEND			# THE T5CYCLE TIME
		MP	NYJETS
		LXCH	DFT2		; Save for rate filter update
		TS	TAU2		# ZERO TAU2
		TCF	ASMBLWY

# Page 1053
# ROLL ON-TIME CALCULATION-

; Calculate firing duration for roll axis thrusters based on commanded torque.
; Roll logic differs from pitch/yaw because roll commands may continue during
; powered flight when TVC is active for pitch/yaw control.
ROLLTIME	CCS	TAU
		TCF	RBLAST
		TCF	+2
		TCF	RBLAST
		INDEX	NRJETS
		CA	DFTMAX		# UPDATE DFT EVEN THO NO ROLL COMMANDS ARE
		TS	DFT		# PRESENT (for rate filter continuity)
		TCF	RBYPASS

; Time conversion constants for on-time calculations and limits.
; AGC time units: 1 unit = 0.01 seconds (10 milliseconds)
		DEC	-480		# = -.3SEC
		DEC	-320		# = -.2SEC
=-.1SEC		DEC	-160		# = -.1SEC
DFTMAX		DEC	0		#     0
=+.1SEC		DEC	160		# = +.1SEC
		DEC	320		# = +.2SEC
		DEC	480		# = +.3SEC
=14MS		DEC	23		# =14MS (minimum impulse time)

; Calculate roll jet on-time: multiply torque by jet effectiveness.
RBLAST		CA	TAU
		EXTEND
		INDEX	NRJETS		; Use selected roll jet pair effectiveness
		MP	NJET		; Multiply torque by 1/effectiveness
		TS	BLAST		# BLAST IS AN INTERMEDIATE VARIABLE
					# USED IN DETERMINING THE JET ON-TIMES
		AD	=-.1SEC
		EXTEND
		BZMF	AD14MSR		; Branch if on-time <= 0.1 seconds

; Roll maneuver requires more than 0.1 second duration.
; Unlike pitch/yaw, roll may continue during powered flight with TVC.
		INDEX	NRJETS		# THE ROLL ROTATION WILL LAST LONGER
		CA	DFTMAX		# THAN THE T5 CYCLE TIME
		TS	DFT		; Save torque for rate filter update
		COM
		ADS	TAU		# Update TAU (reduce remaining torque)
		CAF	=+.1SEC		# LIMIT THE LENGTH OF ROLL ROTATION
		TS	BLAST		# COMMANDS TO 0.1 SEC SO THAT ONLY Y-Z
		TCF	ASMBLWR		# TRANSLATION COMMANDS CONTINUE

; Check if roll on-time is less than minimum impulse time (14 ms).
AD14MSR		CS	BLAST		# SEE IF THE JET ON-TIME LESS THAN
		AD	=14MS		# MINIMUM IMPULSE TIME
		EXTEND
		BZMF	RBLASTOK	; Branch if on-time >= 14 ms
		CAF	=14MS		# IF SO, LIMIT MINIMUM ON-TIME TO 14 MS
		TS	BLAST		; Enforce 14 ms minimum for thruster reliability

; Roll on-time is acceptable. Calculate actual torque applied and update filters.
RBLASTOK	CA	BLAST
		EXTEND
		MP	NRJETS		; Calculate actual applied torque
		LXCH	DFT		; Save for rate filter update
		TS	TAU		# ZERO TAU (ACC contains zero)
		TCF	ASMBLWR		; Proceed to assemble second roll word

# Page 1054
		DEC	-.333333	# = -1/3
		DEC	-.500000	# = -1/2
		DEC	-.999999	# = -1 (NEGMAX)
NJET		DEC	0
		DEC	.999999		# = +1 (POSMAX)
		DEC	.500000		# = +1/2
		DEC	.333333		# = +1/3

# Page 1055
# 	WHEN THE ROTATION COMMANDS ARE COMPLETED, IT IS NECESSARY TO REPLACE THESE COMMANDS BY NEW COMMANDS WHICH
# CONTINUE ON WITH THE TRANSLATIONS IF ANY ARE PRESENT.
# 	IN THIS SECTION THESE NEW COMMANDS ARE GENERATED AND STORED FOR REPLACEMENT OF THE CHANNEL COMMANDS WHEN THE
# CORRESPONDING ROTATIONS ARE COMPLETED.

# GENERATION OF THE SECOND PITCH(X-TRANS) WORD...PWORD2

; ============================================================================
; TRANSITION: From combined roll+yaw maneuver to pitch+translation assembly
;
; After handling roll and yaw axes with their translation combinations, attention
; turns to pitch axis. The second pitch word (PWORD2) is constructed to combine
; pitch rotation with X-axis translation if the AC quad jets are available.
; This completes the three-axis jet selection logic.
; ============================================================================

; Assemble second pitch word (PWORD2) to combine pitch with X translation.
; COMMENT-ONLY READERS: The computer now determines whether X translation can
; be combined with pitch rotation, checking for failed jets that might prevent
; the combined maneuver.
;
; CODE-ALONG READERS: PWORD2 construction depends on RACFAIL (AC quad failure
; status) and XNDX1 (X translation direction). If AC quad has failed, X translation
; is skipped. Otherwise, appropriate jets from PYTABLE are selected.
ASMBLWP		CCS	RACFAIL		; Check AC quad failure status
		TCF	FPX2		# IF FAILURE ON AC IGNORE X-TRANSLATION
		TCF	+2		; AC operational
		TCF	FPX2		; AC failed (negative case)
		INDEX	XNDX1		; Index by X translation direction (AC quad)
		CA	XLNNDX		; Get translation table index
		INDEX	A
FPX2		CA	PYTABLE		; Get pitch jet pattern from table
		MASK	PJETS		; Mask for pitch jet bits only
		TS	PWORD2		; Store second pitch command word
		TCF	YAWTIME		; Proceed to yaw on-time calculation

; Bypass case: no second pitch word needed (pure pitch rotation).
; PWORD2 is simply a copy of PWORD1 with BLAST1 zeroed.
PBYPASS		CA	PWORD1		# THE T6 PROGRAM WILL LOAD PWORD2
		TS	PWORD2		# UPON ENTRY (copy PWORD1 to PWORD2)
		CAF	ZERO
		TS	BLAST1		# THERE IS NO PWORD2 (no second interval needed)
		TCF	YAWTIME		; Continue to yaw timing

# Page 1056
# GENERATION OF THE SECOND ROLL (Y,Z) WORD (RWORD2)

; ============================================================================
; TRANSITION: From roll on-time calculation to combined maneuver assembly
;
; With roll jet firing time determined, the computer now constructs the second
; command word (RWORD2) to combine roll rotation with Y and/or Z translation
; commands. This allows simultaneous attitude and translation control, saving
; fuel by combining maneuvers when possible.
; ============================================================================

; Assemble second roll word with Y/Z translation if compatible with roll jets.
ASMBLWR		CCS	YNDX		# CHECK FOR Y-TRANS
		TCF	ACBD2Y		; Y translation requested
NO2Y		CAF	ZERO
		TS	RWORD2		; No Y translation, initialize RWORD2
		CCS	ZNDX		# CHECK FOR Z-TRANS
		TCF	ACBD2Z		; Z translation requested
NO2Z		CAF	ZERO
		ADS	RWORD2		; No Z translation either
		TCF	PITCHTIM	# RWORD2 ASSEMBLED, proceed to pitch

; Check if Y translation can be combined with roll maneuver.
; Logic depends on which jets (AC or BD) are performing the roll.
ACBD2Y		CCS	ACORBD
		TCF	AC2Y		# CAN DO Y-TRANS (BD doing roll)
		TCF	AC2Y		; (same result)
		TCF	+1		# USING AC FOR ROLL
		CCS	RACFAIL		; Check if AC jets have failed
		TCF	NO2Y		# USING AC AND AC HAS FAILED
		TCF	+2		; AC operational
		TCF	NO2Y		# DITTO (AC failed)

; AC jets operational and doing roll. Can combine with Y translation.
		INDEX	YNDX		# NO FAILURES, CAN DO Y
		CA	XLNNDX		; Get Y translation table index
		INDEX	A
		CA	RTABLE		; Get roll jets from table
		MASK	ACRJETS		; Mask for AC quad jets
		TCF	NO2Y +1		; Store result in RWORD2

; BD jets are doing roll, so AC jets available for Y translation.
AC2Y		CCS	RACFAIL		; Check AC jet failures
		CAF	THREE		; AC failed, use offset 3
		TCF	+2
		CAF	SIX		; AC operational, use offset 6
		INDEX	YNDX		; Add Y translation direction index
		AD	XLN1NDX
		INDEX	A		; Index into Y/Z translation table
		CA	YZTABLE
		MASK	ACYJETS		; Select AC Y-axis jets
		TS	RWORD2		; Store in second roll word
		EXTEND
		MP	BIT4		; Scale for jet count (4 jets per quad)
		AD	=-2		; Convert to effectiveness index
		TS	NRJETS		; Store jet effectiveness
		CS	BLAST		; Calculate torque reduction
		AD	=+.1SEC		; from Y translation coupling
		EXTEND
		MP	NRJETS
		CA	L		; Get product
		ADS	DFT		; Update rate filter torque
		TCF	NO2Y +2		; Continue to check Z translation
# Page 1057
; Check if Z translation can be combined with roll maneuver.
ACBD2Z		CCS	ACORBD
		TCF	BDF2Z		# USING BD-ROLL
		TCF	BDF2Z		# MUST CHECK FOR BD FAILURES
		TCF	+1		; Using AC for roll
		CCS	RBDFAIL		# USING AC FOR ROLL, CAN DO Z-TRANS
		CAF	THREE		; BD failed, use offset 3
		TCF	+2
		CAF	SIX		; BD operational, use offset 6
		INDEX	ZNDX		; Add Z translation direction index
		AD	XLN1NDX
		INDEX	A		; Index into Y/Z translation table
		CA	YZTABLE
		MASK	BDZJETS		; Select BD Z-axis jets
		ADS	RWORD2		; Add to second roll word
		EXTEND
		MP	BIT7		; Scale for jet count
		AD	=-2		; Convert to effectiveness index
		TS	NRJETS		; Store jet effectiveness
		CS	BLAST		; Calculate torque reduction
		AD	=+.1SEC		; from Z translation coupling
		EXTEND
		MP	NRJETS
		CA	L		; Get product
		ADS	DFT		; Update rate filter torque
		TCF	PITCHTIM	; Proceed to pitch time calculation

; BD jets are doing roll. Check if BD operational for Z translation.
BDF2Z		CCS	RBDFAIL
		TCF	NO2Z		# USING BD-ROLL AND BD HAS FAILED
		TCF	+2		; BD operational
		TCF	NO2Z		# DITTO (BD failed)
		INDEX	ZNDX		; Get Z translation table index
		CA	XLNNDX
		INDEX	A
		CA	RTABLE		; Get roll jets from table
		MASK	BDRJETS		; Mask for BD quad jets
		TCF	NO2Z +1		; Store result in RWORD2

; No roll commands present. Copy first roll word to second for T6 program.
RBYPASS		CA	RWORD1
		TS	RWORD2		; Duplicate for consistent structure
		CAF	ZERO
		TS	BLAST		; Zero on-time
		TCF	PITCHTIM	; Continue to pitch time calculation

# Page 1058
# GENERATION OF THE SECOND YAW (X-TRANS) WORD...YWORD2

; ============================================================================
; TRANSITION: From pitch time calculation to yaw second word assembly
;
; The yaw second word (YWORD2) combines yaw rotation with X translation. This
; represents the final step of command word generation before the T6 interrupt
; program sequences the actual jet firings.
; ============================================================================

; Assemble second yaw word with X translation if BD jets operational.
ASMBLWY		CCS	RBDFAIL
		TCF	FYX2		# IF FAILURE ON BD IGNORE X-TRANSLATION
		TCF	+2		; BD operational
		TCF	FYX2		; BD failed
		INDEX	XNDX2		; Get X translation table index
		CA	XLNNDX		; (BD quad handles X translation)
		INDEX	A
FYX2		CA	PYTABLE		; Get yaw jets from table
		MASK	YJETS		; Mask for yaw axis jets
		TS	YWORD2		; Store second yaw word
		TCF	T6SETUP		; Proceed to interrupt setup

; No yaw commands present. Copy first yaw word to second for T6 program.
YBYPASS		CA	YWORD1
		TS	YWORD2		; Duplicate for consistent structure
		CAF	ZERO
		TS	BLAST2		; Zero on-time

# Page 1059
#		SORT THE JET ON-TIMES

# 	AT THIS POINT ALL THE CHANNEL COMMANDS AND JET ON-TIMES HAVE BEEN DETERMINED.  IN SUMMARY THESE ARE-

#		RWORD1
#		RWORD2		BLAST

#		PWORD1
#		PWORD2		BLAST1

#		YWORD1
#		YWORD2		BLAST2

# 	IN THIS SECTION THE JET ON-TIMES ARE SORTED AND THE SEQUENCE OF T6 INTERRUPTS IS DETERMINED.  TO FACILITATE
# THE SORTING PROCESS AND THE T6 PROGRAM, THE VARIABLES BLAST, BLAST1, BLAST2, ARE RESERVED AS DOUBLE PRECISION
# WORDS.  THE LOWER PART OF THESE WORDS CONTAIN A BRANCH INDEX ASSOCIATED WITH THE ROTATION AXIS OF THE HIGHER
# ORDER WORD.

; ============================================================================
; TRANSITION: From command word generation to interrupt sequencing
;
; With all six command words and three on-times determined, the program now
; sorts the firing durations (BLAST, BLAST1, BLAST2) into ascending order.
; This sorting determines the sequence of T6 interrupts that will fire the jets.
; The shortest on-time fires first, then the T6 program updates channel outputs
; at each subsequent interrupt to turn off jets whose on-times have elapsed.
; ============================================================================

; Initialize branch indices in lower words for T6 program axis identification.
T6SETUP		CAF	ZERO		# BRANCH INDEX FOR ROLL
		TS	BLAST +1	; Lower word identifies roll axis
		CAF	FOUR		# BRANCH INDEX FOR PITCH
		TS	BLAST1 +1	; Lower word identifies pitch axis
		CAF	ELEVEN		# BRANCH INDEX FOR YAW
		TS	BLAST2 +1	; Lower word identifies yaw axis

; Bubble sort the three on-times into ascending order (T1 <= T2 <= T3).
; After sorting, shortest on-time in BLAST, longest in BLAST2.
		CS	BLAST		; Compare T1 and T2
		AD	BLAST1
		EXTEND
		BZMF	DXCHT12		# T1 GR T2 (swap if T1 > T2)
CHECKT23	CS	BLAST1		; Compare T2 and T3
		AD	BLAST2
		EXTEND
		BZMF	DXCHT23		; T2 > T3 (swap if T2 > T3)
		
; Calculate delta times for T6 interrupt sequence.
; T6 fires at T1, then at T1+ΔT2, then at T1+ΔT2+ΔT3.
CALCDT6		CS	BLAST1		; Calculate ΔT3 = T3 - T2
		ADS	BLAST2
		CS	BLAST		; Calculate ΔT2 = T2 - T1
		ADS	BLAST1		# END OF SORTING PROCEDURE
		EXTEND			# RESET T5LOC TO BEGIN PHASE1
		DCA	RCS2CADR	; Reload RCSATT address for next cycle
		DXCH	T5LOC
		
; Reset RCS flags and phase counter to complete this DAP cycle.
ENDJETS		CS	BIT1		# RESET BIT1 FOR INITIALIZATION OF
		MASK	RCSFLAGS	# T6 PROGRAM
		TS	RCSFLAGS	; Clear initialization flag
		CS	ZERO		# RESET T5PHASE FOR PHASE1
		TS	T5PHASE		; Ready for next T5 interrupt cycle
		TCF	RESUME		# RESUME INTERRUPTED PROGRAM

		EBANK=	KMPAC
RCS2CADR	2CADR	RCSATT

# Page 1060
; Swap T1 and T2 (BLAST and BLAST1) if T1 > T2.
DXCHT12		DXCH	BLAST		; Save T1
		DXCH	BLAST1		; Get T2
		DXCH	BLAST		; Store T2 in T1, restore T1
		TCF	CHECKT23	; Continue sorting

; Swap T2 and T3 (BLAST1 and BLAST2) if T2 > T3.
DXCHT23		DXCH	BLAST1		; Save T2
		DXCH	BLAST2		; Get T3
		DXCH	BLAST1		; Store T3 in T2, restore T2
		CS	BLAST		; Re-check T1 vs new T2
		AD	BLAST1		; (after T2/T3 swap)
		EXTEND
		BZMF	+2		; T1 <= T2, sorting complete
		TCF	CALCDT6		; Calculate delta times
		DXCH	BLAST		; T1 > T2, swap them
		DXCH	BLAST1
		DXCH	BLAST
		TCF	CALCDT6		; Calculate delta times

# Page 1061
# T6 PROGRAM AND CHANNEL SETUP

		BANK	21
		SETLOC	DAPS5
		BANK

; ============================================================================
; TRANSITION: From jet selection logic to real-time interrupt firing sequence
;
; The T6 interrupt program executes the actual jet firings calculated by
; JETSLECT. It fires at three times determined by the sorted on-times:
;   T6(1): Fire all jets at BLAST time
;   T6(2): Update jets at BLAST+BLAST1 time (turn off shortest-duration axis)
;   T6(3): Final update at BLAST+BLAST1+BLAST2 (turn off second axis)
;
; Throughout Apollo 11's mission, this routine executed thousands of times to
; maintain spacecraft attitude during maneuvers, docking, and orbital operations.
; ============================================================================

; T6 Interrupt entry point - handles time-sequenced jet channel updates.
T6START		LXCH	BANKRUPT	; Save interrupted program context
		EXTEND
		QXCH	QRUPT		; Save return address
		CCS	TIME6		# CHECK TO SEE IF TIME6 WAS RESET
		TCF	RESUME		# AFTER T6RUPT OCCURED(IN T5RUPT)
		TCF	+2		# IF SO WAIT FOR NEXT T6RUPT BEFORE
		TCF	RESUME		# TAKING ACTION (spurious interrupt)

; Check initialization flag. First T6 interrupt initializes channels with WORD1.
		CS	RCSFLAGS
		MASK	BIT1		# IF BIT1 IS 0 RESET TO 1
		EXTEND			# AND INITIALIZE CHANNEL
		BZF	T6RUPTOR	; Skip initialization if already done
		ADS	RCSFLAGS	; Set initialization flag
		CA	RWORD1		; Load first roll command word
		EXTEND			# INITIALIZE CHANNELS 5,6 WITH WORD1
		WRITE	CHAN6		; Channel 6 = roll jets
		CA	PWORD1		; Load first pitch command word
		AD	YWORD1		; Combine with first yaw command word
		EXTEND
		WRITE	CHAN5		; Channel 5 = pitch + yaw jets

; Main T6 interrupt sequence: check each on-time and update channels at expiry.
; Each axis has been assigned a sorted position (BLAST, BLAST1, or BLAST2).
; Branch indices in lower words identify which axis to update at each interrupt.
T6RUPTOR	CCS	BLAST		; Check first (shortest) on-time
		TCF	ZBLAST		# ZERO BLAST1 (positive, continue timing)
		TCF	REPLACE		# REPLACE WORD1 (zero, time expired)
		TCF	+2		; Negative should not occur
		TCF	REPLACE		; Negative zero
T6L1		CCS	BLAST1		; Check second (middle) on-time
		TCF	ZBLAST1		; Continue timing
		TCF	REPLACE1	; Time expired, update second axis
		TCF	+2
		TCF	REPLACE1
T6L2		CCS	BLAST2		; Check third (longest) on-time
		TCF	ZBLAST2		; Continue timing
		TCF	REPLACE2	; Time expired, update third axis
		TCF	RESUME		; All on-times expired, resume
		TCF	REPLACE2

; First on-time expired. Update jets for shortest-duration axis using branch index.
REPLACE		INDEX	BLAST +1	; Branch index: 0=roll, 4=pitch, 11=yaw
		TC	REPLACER	; Call axis-specific updater
		CS	ONE
		TS	BLAST		; Mark BLAST as complete (-1)
		TCF	T6L1		; Check second on-time

; Second on-time expired. Update jets for middle-duration axis.
REPLACE1	INDEX	BLAST1 +1	; Use second axis branch index
# Page 1062
		TC	REPLACER	; Call axis-specific updater
		CS	ONE
		TS	BLAST1		; Mark BLAST1 as complete (-1)
		TCF	T6L2		; Check third on-time

; Third on-time expired. Update jets for longest-duration axis, then resume.
REPLACE2	INDEX	BLAST2 +1	; Use third axis branch index
		TC	REPLACER	; Call axis-specific updater
		CS	ONE
		TS	BLAST2		; Mark BLAST2 as complete (-1)
		TCF	RESUME		; All axes updated, resume interrupted program

; Roll axis channel update: write second roll word to Channel 6.
; Entry: Branch index 0 (for roll axis)
REPLACER	CA	RWORD2		; Load second roll command word
		EXTEND			# INITIALIZE CHANNELS 5,6 WITH WORD2
		WRITE	CHAN6		; Update roll jets (Channel 6)
		TC	Q		; Return to caller

; Pitch axis channel update: preserve yaw jets, update pitch jets on Channel 5.
; Entry: Branch index 4 (for pitch axis)
REPLACEP	CA	YJETS		; Mask for yaw jet bits
		EXTEND
		RAND	CHAN5		; Read current Channel 5, preserve yaw bits
		AD	PWORD2		; Add second pitch command word
		EXTEND
		WRITE	CHAN5		; Update pitch jets, preserve yaw
		TC	Q		; Return to caller

; Yaw axis channel update: preserve pitch jets, update yaw jets on Channel 5.
; Entry: Branch index 11 (for yaw axis)
REPLACEY	CA	PJETS		; Mask for pitch jet bits
		EXTEND
		RAND	CHAN5		; Read current Channel 5, preserve pitch bits
		AD	YWORD2		; Add second yaw command word
		EXTEND
		WRITE	CHAN5		; Update yaw jets, preserve pitch
		TC	Q		; Return to caller

; ============================================================================
; ON-TIME COMPLETION AND T6 RE-ENABLE ROUTINES
;
; When a scheduled on-time expires, the corresponding BLAST variable is zeroed
; and T6RUPT is re-enabled with the next on-time. This ensures the interrupt
; fires for each of the three axis on-times in sequence.
;
; COMMENT-ONLY READERS: After each jet firing duration completes, the timer
; is reset to fire again for the next axis requiring jet pulses.
;
; CODE-ALONG READERS: The ZBLAST routines clear the completed on-time variable
; and fall through to ENABT6 to schedule the next T6 interrupt. The TIME6
; register is loaded with the next on-time value, and T6RUPT is re-enabled
; via Channel 13.
; ============================================================================

ZBLAST		CAF	ZERO
		XCH	BLAST		; Clear BLAST on-time, return old value in A
		TCF	ENABT6
ZBLAST1		CAF	ZERO
		XCH	BLAST1		; Clear BLAST1 on-time, return old value in A
		TCF	ENABT6
ZBLAST2		CAF	ZERO
		XCH	BLAST2		; Clear BLAST2 on-time, return old value in A
		
; Re-enable T6 interrupt with the next scheduled on-time.
; Entry: A register contains the cleared on-time value (used as next TIME6).
ENABT6		TS	TIME6		; Load TIME6 with next on-time interval
		CAF	NEGMAX
		EXTEND
		WOR	CHAN13		; Write-OR to Channel 13: enable T6RUPT
		TCF	RESUME		; Restore context and return from interrupt

# END OF T6 INTERRUPT

ENDSLECT	EQUALS
