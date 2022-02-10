##########################################
#### Config for Flapper Drone Nimble+ ####
##########################################

## Do not change the settings below, unless if you know what you are doing ##
#############################################################################

## Uncomment when using CF2 as flight controller
# CFLAGS += -DNIMBLE_USE_CF2

CFLAGS += -DMOTOR_SETUP_NIMBLE
# CFLAGS += -DENABLE_PWM_EXTENDED
## Keep the yaw setpoint within +/- YAW_MAX_DELTA from the current heading
CFLAGS += -DYAW_MAX_DELTA=30.0f
CFLAGS += -DPID_FILTER_ALL
CFLAGS += -DCPPM_USE_PA3 # CPPM pin, other alternatives:  PA7 (default), PA2(TX2), PA3(RX2), PB4(IO_3), PB5(IO_2) or PB8(IO_1)

# POWER_DISTRIBUTION = nimble_FD_PCB
# CFLAGS += -DMOTOR_SERVO_PITCH=1
# CFLAGS += -DMOTOR_SERVO_YAW=2
# CFLAGS += -DIMU_PHI=0.0f # IMU orientation
# CFLAGS += -DIMU_THETA=90.0f
# CFLAGS += -DIMU_PSI=180.0f

POWER_DISTRIBUTION = nimble_FD_PCB_revD
CFLAGS += -DMOTOR_SERVO_PITCH=0
CFLAGS += -DMOTOR_SERVO_YAW=2
CFLAGS += -DIMU_PHI=0.0f # IMU orientation
CFLAGS += -DIMU_THETA=-90.0f
CFLAGS += -DIMU_PSI=180.0f

## Deck selection ##
####################

## Uncomment the in-built features ("virtual" decks) you'd like to use:
# CFLAGS += -DDECK_FORCE=bcCPPM # force the CCPM deck
# CFLAGS += -DDECK_FORCE=bcUSD # force the SD card deck
# CFLAGS += -DDECK_FORCE=bcLedRing
# CFLAGS += -DDECK_FORCE=bcCurrentDeck # analog current sensor on PA2/TX2 (integrated only in PCB_revD)

## To use multiple features at a time, separate their names with colons:
# CFLAGS += -DDECK_FORCE=bcUSD:bcCPPM:bcLedRing:bcCurrentDeck
CFLAGS += -DDECK_FORCE=bcUSD:bcLedRing:bcCurrentDeck

## External receiver settings ##
################################
# CFLAGS += -DEXTRX_ALT_HOLD # enable altitude hold with external Rx
# CFLAGS += -DEXTRX_ARMING # enable arming with external Rx (setup via "Brushless handling" compile flags)
# CFLAGS += -DEXTRX_TAER # use TAER channel mapping instead of the default AETR - Aileron(Roll), Elevator(Pitch), Thrust, Rudder(Yaw)

## RGB LED settings ##
######################
# CFLAGS += -DTURN_OFF_LED
# CFLAGS += -DLEDRING_DEFAULT_EFFECT=7
# CFLAGS += -DLED_RING_NBR_LEDS=12

## Better baro hold with no additional sensors ##
#################################################
# CFLAGS += -DIMPROVED_BARO_Z_HOLD

## Limit maximal thrust (100% = 65535) ##
#########################################
# CFLAGS += -DNIMBLE_MAX_THRUST 53000.0f
