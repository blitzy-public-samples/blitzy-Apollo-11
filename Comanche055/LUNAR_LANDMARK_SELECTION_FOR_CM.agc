# Copyright:	Public domain.
# Filename:	LUNAR_LANDMARK_SELECTION_FOR_CM.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Jim Lawton <jim.lawton@gmail.com>.
# Website:	www.ibiblio.org/apollo.
# Pages:	936
# Mod history:	2009-05-11 JVL	Adapted from the Colossus249/ file
#				of the same name, using Comanche055 page
#				images.
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
#    Assemble revision 055 of AGC program Comanche by NASA
#    2021113-051.  10:28 APR. 1, 1969
#
#    This AGC program shall also be referred to as
#            Colossus 2A

# Page 936

; ============================================================================
; FILE: LUNAR_LANDMARK_SELECTION_FOR_CM.agc
; MODULE: TROUBLE Subsystem (Mission Programs)
; MISSION PHASE: lunar-orbit
;
; TL;DR: Lunar surface landmark optical tracking for Command Module navigation
;        updates. Enables crew to sight known lunar surface features through
;        optics, providing navigation state corrections during lunar orbit
;        operations while LM conducts surface mission.
;
; COMMENT-ONLY READERS: This program let Michael Collins track known features
;        on the Moon's surface to improve navigation while orbiting alone.
; CODE-ALONG READERS: Study landmark sighting processing, optical navigation
;        measurement incorporation, and navigation state update algorithms.
; ============================================================================

; PLACEHOLDER FILE NOTATION
;
; This file marks the end of the TROUBLE subsystem section 043 in the Command
; Module AGC program. The actual lunar landmark selection code would have
; enabled optical tracking of known surface features for navigation updates.
;
; OPERATIONAL CONTEXT:
; During Apollo 11's mission, while the Lunar Module Eagle descended to the
; surface with Armstrong and Aldrin, Command Module pilot Michael Collins
; remained in lunar orbit aboard Columbia. Landmark tracking allowed Collins
; to sight known lunar surface features through the Command Module optics
; (sextant), improving the spacecraft's navigation state vector accuracy.
;
; NAVIGATION MEASUREMENT PROCESS:
; 1. Crew identifies known landmark on lunar surface using star charts
; 2. Sights landmark through Command Module sextant optics
; 3. AGC records sighting angles and timing
; 4. Measurement incorporation routines (MEASUREMENT_INCORPORATION.agc) 
;    compute state vector corrections based on expected vs actual landmark
;    position
; 5. Navigation state updated with improved position/velocity accuracy
;
; MISSION SIGNIFICANCE:
; Landmark tracking provided independent navigation verification during lunar
; orbit operations, especially critical during periods when ground tracking
; was unavailable (far side of Moon passes). Collins performed these optical
; navigation tasks while maintaining solo orbit operations, demonstrating the
; Command Module's autonomous navigation capabilities.
;
; Source: This notation explains the purpose of landmark selection programs
; that would integrate with MEASUREMENT_INCORPORATION.agc navigation updates.

# *** END OF TROUBLE .043 ***

