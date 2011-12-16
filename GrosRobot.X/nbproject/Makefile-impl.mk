# 
# Generated Makefile - do not edit! 
# 
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a pre- and a post- target defined where you can add customization code.
#
# This makefile implements macros and targets common to all configurations.
#
# NOCDDL


# Building and Cleaning subprojects are done by default, but can be controlled with the SUB
# macro. If SUB=no, subprojects will not be built or cleaned. The following macro
# statements set BUILD_SUB-CONF and CLEAN_SUB-CONF to .build-reqprojects-conf
# and .clean-reqprojects-conf unless SUB has the value 'no'
SUB_no=NO
SUBPROJECTS=${SUB_${SUB}}
BUILD_SUBPROJECTS_=.build-subprojects
BUILD_SUBPROJECTS_NO=
BUILD_SUBPROJECTS=${BUILD_SUBPROJECTS_${SUBPROJECTS}}
CLEAN_SUBPROJECTS_=.clean-subprojects
CLEAN_SUBPROJECTS_NO=
CLEAN_SUBPROJECTS=${CLEAN_SUBPROJECTS_${SUBPROJECTS}}


# Project Name
PROJECTNAME=GrosRobot.X

# Active Configuration
DEFAULTCONF=default
CONF=${DEFAULTCONF}

# All Configurations
ALLCONFS=default 


# build
.build-impl: .build-pre
	@#echo "=> Running $@... Configuration=$(CONF)"
	${MAKE} -C /home/martin/Eurobot/PIC/CAN-USART.X SUBPROJECTS=${SUBPROJECTS} build
	${MAKE} -C /home/martin/Eurobot/PIC/PID.X SUBPROJECTS=${SUBPROJECTS} build
	${MAKE} -C /home/martin/Eurobot/PIC/odometrie.X SUBPROJECTS=${SUBPROJECTS} build


# clean
.clean-impl: .clean-pre
	@#echo "=> Running $@... Configuration=$(CONF)"
	${MAKE} -C /home/martin/Eurobot/PIC/CAN-USART.X SUBPROJECTS=${SUBPROJECTS} clean
	${MAKE} -C /home/martin/Eurobot/PIC/PID.X SUBPROJECTS=${SUBPROJECTS} clean
	${MAKE} -C /home/martin/Eurobot/PIC/odometrie.X SUBPROJECTS=${SUBPROJECTS} clean


# clobber 
.clobber-impl: .clobber-pre
	@#echo "=> Running $@..."
	for CONF in ${ALLCONFS}; \
	do \
	    ${MAKE} -C /home/martin/Eurobot/PIC/CAN-USART.X SUBPROJECTS=${SUBPROJECTS} clean; \
	    ${MAKE} -C /home/martin/Eurobot/PIC/PID.X SUBPROJECTS=${SUBPROJECTS} clean; \
	    ${MAKE} -C /home/martin/Eurobot/PIC/odometrie.X SUBPROJECTS=${SUBPROJECTS} clean; \
	done


# all 
.all-impl: .all-pre .depcheck-impl
	@#echo "=> Running $@..."
	for CONF in ${ALLCONFS}; \
	do \
	    ${MAKE} -C /home/martin/Eurobot/PIC/CAN-USART.X SUBPROJECTS=${SUBPROJECTS} build; \
	    ${MAKE} -C /home/martin/Eurobot/PIC/PID.X SUBPROJECTS=${SUBPROJECTS} build; \
	    ${MAKE} -C /home/martin/Eurobot/PIC/odometrie.X SUBPROJECTS=${SUBPROJECTS} build; \
	done


# help
.help-impl: .help-pre
	@echo "This makefile supports the following configurations:"
	@echo "    ${ALLCONFS}"
	@echo ""
	@echo "and the following targets:"
	@echo "    build  (default target)"
	@echo "    clean"
	@echo "    clobber"
	@echo "    all"
	@echo "    help"
	@echo ""
	@echo "Makefile Usage:"
	@echo "    make [CONF=<CONFIGURATION>] [SUB=no] build"
	@echo "    make [CONF=<CONFIGURATION>] [SUB=no] clean"
	@echo "    make [SUB=no] clobber"
	@echo "    make [SUB=no] all"
	@echo "    make help"
	@echo ""
	@echo "Target 'build' will build a specific configuration and, unless 'SUB=no',"
	@echo "    also build subprojects."
	@echo "Target 'clean' will clean a specific configuration and, unless 'SUB=no',"
	@echo "    also clean subprojects."
	@echo "Target 'clobber' will remove all built files from all configurations and,"
	@echo "    unless 'SUB=no', also from subprojects."
	@echo "Target 'all' will will build all configurations and, unless 'SUB=no',"
	@echo "    also build subprojects."
	@echo "Target 'help' prints this message."
	@echo ""

