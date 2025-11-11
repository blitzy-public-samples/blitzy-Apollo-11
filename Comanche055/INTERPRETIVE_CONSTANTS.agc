# Copyright:	Public domain.
# Filename:	INTERPRETIVE_CONSTANTS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Pages:	1205-1206
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
; FILE: INTERPRETIVE_CONSTANTS.agc
; MODULE: CHIEFTAN Subsystem (Core Operating System)
; MISSION PHASE: all-phases
;
; TL;DR: Defines essential constants used by the interpretive language virtual
;        machine for vector/matrix operations and mathematical computations.
;        Provides unit vectors, zero vectors, double-precision constants, and
;        special operational parameters used throughout Apollo 11 guidance,
;        navigation, and orbital mechanics calculations.
;
; COMMENT-ONLY READERS: Mathematical constants used by the computer's high-level
;        calculation language for navigation and guidance throughout the mission.
; CODE-ALONG READERS: Study interpretive language constant definitions, double-
;        precision scaling conventions, and unit vector representations used by
;        the interpreter virtual machine (see INTERPRETER.agc for usage context).
; ============================================================================

# Page 1205
; ============================================================================
; INTERPRETIVE CONSTANTS - BANK 1 (INTPRET1)
;
; This section defines constants stored in the first interpretive language
; memory bank. These values are used by interpretive instructions throughout
; guidance and navigation computations during all mission phases.
; ============================================================================
		SETLOC	INTPRET1
		BANK

		COUNT	23/ICONS

; Double-precision constant: One quarter (0.25)
; Used in scaling operations and fractional computations within interpretive
; vector and matrix operations. Interpretive language uses double-precision
; (DP) arithmetic for extended precision in navigation calculations.
DP1/4TH		2DEC	.25

; Unit vector components defining standard coordinate system axes.
; These three constants together form a unit vector in the X direction.
; Interpretive language represents 3D vectors as three consecutive memory
; locations (X, Y, Z components). Scaling: 0.5 in scaled units = 1.0 actual.
;
; UNITZ: Z component of unit X vector (0.0)
UNITZ		2DEC	0

; UNITY: Y component of unit X vector (0.0)
UNITY		2DEC	0

; UNITX: X component of unit X vector (0.5 scaled = 1.0 actual)
; The AGC's double-precision format requires scaling by 0.5 to represent
; unity in vector operations, avoiding overflow in 15-bit signed arithmetic.
UNITX		2DEC	.5

; Zero vector (3D): Three consecutive zero values representing origin point
; or null vector in interpretive coordinate transformations. Used to initialize
; vectors or represent zero displacement/velocity in orbital mechanics.
ZEROVECS	2DEC	0

		2DEC	0

		2DEC	0

; DPHALF: Alias for half value in double-precision (same as UNITX = 0.5)
; Used when 0.5 value needed in arithmetic operations distinct from unit vector.
DPHALF		=	UNITX

; DPPOSMAX: Maximum positive double-precision value (octal 37777 37777)
; Represents largest positive number in AGC's 2-word double-precision format.
; Used for range checking and overflow detection in interpretive calculations.
DPPOSMAX	OCT	37777
		OCT	37777

# Page 1206
# INTERPRETIVE CONSTANTS IN THE OTHER HALF-MEMORY

; ============================================================================
; INTERPRETIVE CONSTANTS - BANK 2 (INTPRET2)
;
; Second memory bank of interpretive constants. The AGC's bank-switched memory
; architecture requires critical constants duplicated across banks to enable
; efficient access regardless of current bank setting. This avoids costly
; bank-switching during time-critical navigation and guidance computations.
; ============================================================================
		SETLOC	INTPRET2
		BANK

		COUNT	14/ICONS

; Unit vector components in Bank 2 (duplicate of Bank 1 for banking efficiency)
; These define the same standard unit X vector but stored in alternate memory
; bank for access without bank-switching overhead during orbital calculations.
;
; ZUNIT: Z component of unit X vector (0.0)
ZUNIT		2DEC	0

; YUNIT: Y component of unit X vector (0.0)
YUNIT		2DEC	0

; XUNIT: X component of unit X vector (0.5 scaled = 1.0 actual)
XUNIT		2DEC	.5

; Zero vector (3D) in Bank 2 - duplicate for efficient access.
; Three consecutive double-precision zeros representing null vector.
ZEROVEC		2DEC	0

		2DEC	0

		2DEC	0

; Special constants for integration and numerical operations.
; CRITICAL: The following three values MUST remain in this exact order for
; proper operation of numerical integration routines (see ORBITAL_INTEGRATION.agc).
;
; Octal 77777 = -0 in AGC one's complement arithmetic (negative zero)
		OCT	77777		# -0,-6,-12 MUST REMAIN IN THIS ORDER

; DEC-6: Constant -6 used in integration step size control and
; Encke method perturbation calculations during orbit propagation.
DEC-6		DEC	-6

; DEC-12: Constant -12 used in integration rectification logic.
; These negative integer constants control numerical precision management
; in the Encke method for propagating spacecraft state vectors.
DEC-12		DEC	-12

; LODPMAX: "Lowest Order" double-precision maximum value (octal 3777737777)
; Used as integration overflow threshold. This value represents the maximum
; magnitude for position/velocity components before rectification required.
; CRITICAL: LODPMAX and LODPMAX1 MUST be adjacent and identical for the
; integration routines to function correctly during orbital mechanics computations.
LODPMAX		2OCT	3777737777	# THESE TWO CONSTANTS MUST REMAIN

LODPMAX1	2OCT	3777737777	# ADJACENT AND THE SAME FOR INTEGRATION

; Convenient aliases pointing to already-defined constants in this bank:
;
; ZERODP: Alias for zero in double-precision (points to ZEROVEC)
; Used when code semantically requires "zero value" rather than "zero vector"
ZERODP		=	ZEROVEC

; HALFDP: Alias for 0.5 in double-precision (points to XUNIT)
; Used when 0.5 value needed in scaling operations distinct from unit vector
HALFDP		=	XUNIT
