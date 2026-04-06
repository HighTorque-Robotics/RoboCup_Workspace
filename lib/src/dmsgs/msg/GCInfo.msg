# General
bool connected

# Control Data
uint8 gameType
uint8 state
bool stopped
bool firstHalf
bool kickoff
uint8 gamePhase      
uint8 setPlay        
uint16 secsRemaining
uint16 secondaryTime

bool ourIndirectFreeKick
bool ourDirectFreeKick
bool ourPenaltyKick
bool ourCornerKick
bool ourGoalKick
bool ourThrowIn

bool enemyIndirectFreeKick
bool enemyDirectFreeKick
bool enemyPenaltyKick
bool enemyCornerKick
bool enemyGoalKick
bool enemyThrowIn

bool state2Ready
bool state2Freeze

# Robot Info
bool penalised
uint8 secsTillUnpenalised

# Team Info
bool teamCyan
uint8 ourScore
uint8 enemyScore

# STATE
uint8 STATE_INITIAL            =  0
uint8 STATE_READY              =  1
uint8 STATE_SET                =  2
uint8 STATE_PLAYING            =  3
uint8 STATE_FINISHED           =  4

# GAME_PHASE 
uint8 GAME_PHASE_NORMAL            = 0
uint8 GAME_PHASE_PENALTY_SHOOT_OUT = 1
uint8 GAME_PHASE_EXTRA_TIME        = 2
uint8 GAME_PHASE_TIMEOUT           = 3

# SET_PLAY 
uint8 SET_PLAY_NONE               = 0
uint8 SET_PLAY_DIRECT_FREE_KICK   = 1
uint8 SET_PLAY_INDIRECT_FREE_KICK = 2
uint8 SET_PLAY_PENALTY_KICK       = 3
uint8 SET_PLAY_THROW_IN           = 4
uint8 SET_PLAY_GOAL_KICK          = 5
uint8 SET_PLAY_CORNER_KICK        = 6