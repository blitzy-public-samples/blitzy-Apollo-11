# Copyright:	Public domain.
# Filename:	CONTRACT_AND_APPROVALS.agc
# Purpose:	Part of the source code for Colossus 2A, AKA Comanche 055.
#		It is part of the source code for the Command Module's (CM)
#		Apollo Guidance Computer (AGC), for Apollo 11.
# Assembler:	yaYUL
# Contact:	Ron Burkey <info@sandroid.org>.
# Website:	www.ibiblio.org/apollo.
# Mod history:	2009-05-06 RSB	Transcribed from page images.
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

# Page 1

; ============================================================================
; FILE: CONTRACT_AND_APPROVALS.agc
; MODULE: INFORMATION Subsystem
; MISSION PHASE: all-phases (foundational document)
;
; TL;DR: Legal and contractual documentation establishing NASA authorization
;        for Comanche 055 Command Module flight software. Documents DSR project
;        55-23870 under MIT Instrumentation Laboratory development with NASA
;        sponsorship. Represents legally binding approval signatures authorizing
;        this code for Apollo 11 mission use.
;
; COMMENT-ONLY READERS: This file contains the official contract approvals
;        that authorized this software for the historic Apollo 11 mission.
; CODE-ALONG READERS: Examine the formal approval process and documentation
;        standards for safety-critical aerospace software.
; ============================================================================

; ============================================================================
; CONTRACT IDENTIFICATION AND PROGRAM AUTHORIZATION
;
; The following block establishes the legal and contractual foundation for
; this flight software. On March 28, 1969, just four months before the
; Apollo 11 lunar landing, MIT Instrumentation Laboratory formally submitted
; Comanche 055 (Colossus 2A) for NASA approval. This represented the culmination
; of years of software development for the Command Module's Apollo Guidance
; Computer.
;
; The contract identifiers documented here trace the chain of authority from
; the Manned Spacecraft Center (now Johnson Space Center) through MIT's
; Instrumentation Laboratory. Contract NAS 9-4065 bound MIT to deliver
; flight-proven software meeting the stringent requirements of Report R-577,
; the specification defining Command Module guidance, navigation, and control
; requirements.
;
; For readers in 2024 and beyond: This approval process exemplifies the
; rigorous documentation and authorization standards required for human
; spaceflight software. Every line of code in this repository was reviewed,
; tested, and formally approved by multiple levels of technical and
; programmatic authority before Neil Armstrong, Buzz Aldrin, and Michael
; Collins entrusted their lives to it.
; ============================================================================

# ************************************************************************
# *                                                                      *
# *	     THIS AGC PROGRAM SHALL ALSO BE REFERRED TO AS:              *
# *									 *
# *                                                                      *
# *			   COLOSSUS 2A                                   *
# *                                                                      *
# *                                                                      *
# *	  THIS PROGRAM IS INTENDED FOR USE IN THE CM AS SPECIFIED	 *
# *       IN REPORT R-577. THIS PROGRAM WAS PREPARED UNDER DSR		 *
# *       PROJECT 55-23870, SPONSORED BY THE MANNED SPACECRAFT		 *
# *       CENTER OF THE NATIONAL AERONAUTICS AND SPACE			 *
# *       ADMINISTRATION THROUGH CONTRACT NAS 9-4065 WITH THE		 *
# *       INSTRUMENTATION LABORATORY, MASSACHUSETTS INSTITUTE OF	 *
# *       TECHNOLOGY, CAMBRIDGE, MASS.					 *
# *                                                                      *
# ************************************************************************

; ============================================================================
; TRANSITION: From Contract Documentation to Formal Approval Signatures
;
; The contract block above established WHAT this software is (Colossus 2A
; for Command Module) and UNDER WHAT AUTHORITY it was developed (NASA contract
; NAS 9-4065, DSR project 55-23870). The approval signatures below establish
; WHO authorized its use for human spaceflight.
;
; This approval chain represents multiple layers of technical review:
; - Programming leadership (Hamilton)
; - Mission program development direction (Lickly)
; - Project management (Martin)
; - Mission development direction (Sears, Battin)
; - Overall program direction (Hoag)
; - Laboratory executive approval (Ragan)
;
; All signatures dated March 28, 1969 - exactly 113 days before Apollo 11's
; lunar landing on July 20, 1969. This tight timeline underscores the intense
; development and validation pressure faced by the Apollo program to meet
; President Kennedy's end-of-decade deadline.
; ============================================================================

; ============================================================================
; FORMAL APPROVAL SIGNATURES - COMANCHE 055 FLIGHT SOFTWARE
;
; The seven signatures below represent the formal authorization chain for
; Comanche 055 to be installed in Apollo 11's Command Module computer.
;
; HISTORICAL NOTE: Margaret Hamilton, whose signature appears first, is
; credited with coining the term "software engineering" and led the team that
; developed this code. Her work on error detection and recovery - including
; the restart routines documented throughout this repository - proved critical
; during Apollo 11's descent when program alarms 1201 and 1202 threatened to
; abort the landing.
;
; These approvals were not ceremonial. Each signatory took legal and
; professional responsibility for software that would control spacecraft
; trajectory, life support system interfaces, and crew safety during the
; 195-hour, 18-minute, 35-second mission from Earth launch to Pacific
; splashdown.
; ============================================================================

#         SUBMITTED:	MARGARET H. HAMILTON		  DATE:	 28 MAR 69
#             M.H.HAMILTON, COLOSSUS PROGRAMMING LEADER
#             APOLLO GUIDANCE AND NAVIGATION

; Margaret Hamilton submitted Comanche 055 as Colossus Programming Leader,
; representing the team of programmers, mathematicians, and engineers who
; developed this software. Her leadership emphasized software reliability,
; error detection, and graceful recovery from unexpected conditions - design
; principles that would prove essential during Apollo 11's descent.

#         APPROVED:	DANIEL J. LICKLY                  DATE:	 28 MAR 69
#             D.J.LICKLY, DIRECTOR, MISSION PROGRAM DEVELOPMENT
#             APOLLO GUIDANCE AND NAVIGATION PROGRAM

; Daniel Lickly's approval as Director of Mission Program Development confirmed
; that Comanche 055 met all mission-specific requirements for Apollo 11's lunar
; landing profile, including translunar injection, lunar orbit operations, and
; Earth reentry.

#	  APPROVED: FRED H. MARTIN                  	  DATE:	 28 MAR 69
#             FRED H. MARTIN, COLOSSUS PROJECT MANGER
#             APOLLO GUIDANCE AND NAVIGATION PROGRAM

; Fred Martin's approval as Colossus Project Manager certified project-level
; completion, budget compliance, and delivery schedule adherence for the
; Command Module software.

#         APPROVED: NORMAN E.SEARS                 	  DATE:	 28 MAR 69
#             N.E. SEARS, DIRECTOR, MISSION DEVELOPMENT
#             APOLLO GUIDANCE AND NAVIGATION PROGRAM

#         APPROVED: RICHARD H. BATTIN               	  DATE:	 28 MAR 69
#             R.H. BATTIN, DIRECTOR, MISSION DEVELOPMENT
#             APOLLO GUIDANCE AND NAVIGATION PROGRAM

; Norman Sears and Richard Battin both served as Directors of Mission
; Development, providing dual oversight of mission trajectory design, orbital
; mechanics algorithms, and navigation accuracy requirements critical for
; lunar mission success.

#         APPROVED: DAVID G. HOAG			  DATE:	 28 MAR 69
#             D.G. HOAG, DIRECTOR
#             APOLLO GUIDANCE AND NAVIGATION PROGRAM

; David Hoag's approval as overall Director of the Apollo Guidance and
; Navigation Program represented top-level technical authority, confirming
; that Comanche 055 met all AGC Block II hardware constraints and mission
; requirements across the entire Apollo program.

#         APPROVED: RALPH R. RAGAN			  DATE:	 28 MAR 69
#             R.R. RAGAN, DEPUTY DIRECTOR
#             INSTRUMENTATION LABORATORY

; ============================================================================
; END OF APPROVAL CHAIN
;
; With these seven signatures, Comanche 055 was formally authorized for
; flight use in Apollo 11. The software documented in the following files
; represents the final approved version that flew aboard Columbia (Command
; Module) during humanity's first lunar landing mission.
;
; From this approval on March 28, 1969, the code was loaded into core rope
; memory (a form of read-only memory where programs were physically woven into
; wire cores) and installed in the Command Module's Apollo Guidance Computer.
; Once in ROM, the code could not be modified - it had to work perfectly the
; first time, every time.
;
; The pressure was immense: software bugs could not be patched in flight.
; The restart protection, alarm handling, and fault tolerance designed into
; this code - and approved by these signatures - would prove their worth when
; unexpected computer overload conditions occurred during the lunar landing
; on July 20, 1969.
;
; COMMENT-ONLY READERS: The remaining files in this repository contain the
; actual flight code that these approvals authorized. Each file builds upon
; this contractual foundation, implementing the guidance, navigation, and
; control algorithms that made the Apollo 11 mission possible.
;
; CODE-ALONG READERS: As you examine the implementation files, remember that
; every instruction, every memory allocation, every timing constraint was
; reviewed and approved under this formal process. The engineering discipline
; and documentation rigor established here set the standard for safety-critical
; software development that continues to influence aerospace engineering today.
; ============================================================================
