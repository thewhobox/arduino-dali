#pragma once

/** DALI commands */
enum DaliCmd {
  OFF = 0, UP = 1, DOWN = 2, STEP_UP = 3, STEP_DOWN = 4,
  RECALL_MAX = 5, RECALL_MIN = 6, STEP_DOWN_AND_OFF = 7, ON_AND_STEP_UP = 8,
  GO_TO_LAST = 10, // DALI-2
  GO_TO_SCENE = 16,
  RESET = 32, ARC_TO_DTR = 33,
  SAVE_VARS = 34, SET_OPMODE = 35, RESET_MEM = 36, IDENTIFY = 37, // DALI-2
  DTR_AS_MAX = 42, DTR_AS_MIN = 43, DTR_AS_FAIL = 44, DTR_AS_POWER_ON = 45, DTR_AS_FADE_TIME = 46, DTR_AS_FADE_RATE = 47,
  DTR_AS_EXT_FADE_TIME = 48, // DALI-2
  DTR_AS_SCENE = 64, REMOVE_FROM_SCENE = 80,
  ADD_TO_GROUP = 96, REMOVE_FROM_GROUP = 112,
  DTR_AS_SHORT = 128,
  QUERY_STATUS = 144, QUERY_BALLAST = 145, QUERY_LAMP_FAILURE = 146, QUERY_LAMP_POWER_ON = 147, QUERY_LIMIT_ERROR = 148,
  QUERY_RESET_STATE = 149, QUERY_MISSING_SHORT = 150, QUERY_VERSION = 151, QUERY_DTR = 152, QUERY_DEVICE_TYPE = 153,
  QUERY_PHYS_MIN = 154, QUERY_POWER_FAILURE = 155,
  QUERY_OPMODE = 158, QUERY_LIGHTTYPE = 159, // DALI-2
  QUERY_ACTUAL_LEVEL = 160, QUERY_MAX_LEVEL = 161, QUERY_MIN_LEVEL = 162, QUERY_POWER_ON_LEVEL = 163, QUERY_FAIL_LEVEL = 164, QUERY_FADE_SPEEDS = 165,
  QUERY_SPECMODE = 166, QUERY_NEXT_DEVTYPE = 167, QUERY_EXT_FADE_TIME = 168, QUERY_CTRL_GEAR_FAIL = 169, // DALI-2
  QUERY_SCENE_LEVEL = 176,
  QUERY_GROUPS_0_7 = 192, QUERY_GROUPS_8_15 = 193,
  QUERY_ADDRH = 194, QUERY_ADDRM = 195, QUERY_ADDRL = 196,
  SET_COORDINATE_X = 224, SET_COORDINATE_Y = 225,
  ACTIVATE = 226,
};

/** DALI special commands */
enum DaliSpecialCmd {
  TERMINATE = 256, SET_DTR = 257,
  INITIALISE = 258, RANDOMISE = 259, COMPARE = 260, WITHDRAW = 261,
  SEARCHADDRH = 264, SEARCHADDRM = 265, SEARCHADDRL = 266,
  PROGRAMSHORT = 267, VERIFYSHORT = 268, QUERY_SHORT = 269, PHYS_SEL = 270,
  ENABLE_DT = 272, SET_DTR1 = 273, SET_DTR2 = 274, WRITE_MEM_LOC = 275,
  WRITE_MEM_LOC_NOREPLY = 276 // DALI-2
};

/** DALI Extended Commands for DT8 - You shall call ENABLE_DT before*/
enum DaliCmdExtendedDT8 {
  SET_TEMP_KELVIN = 231,
  SET_TEMP_RGB = 235,
  QUERY_COLOR_TYPE_FEATURES = 249,
  QUERY_COLOR_VALUE = 250
};

/** DALI device types */
enum class DaliDevTypes {
  FLUORESCENT_LAMP,
  EMERGENCY_LIGHT,
  DISCHARGE_LAMP,
  HALOGEN_LAMP,
  INCANDESCENT_LAMP,
  DC_CONVERTER,
  LED_MODULE,
  SWITCH,
  COLOUR_CTRL,
  SEQUENCER,
  OPTICAL_CTRL
};

/** Type of address (short/group) */
enum DaliAddressTypes {
  SHORT = 0,
  GROUP = 1
};