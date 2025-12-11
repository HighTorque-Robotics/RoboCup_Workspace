# General
bool connected

# Control Data
uint8 gameType
uint8 state
bool firstHalf
bool kickoff
uint8 secondaryState
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

# STATE2
uint8 STATE2_NORMAL            =  0
uint8 STATE2_PENALTYSHOOT      =  1
uint8 STATE2_OVERTIME          =  2
uint8 STATE2_TIMEOUT           =  3
uint8 STATE2_DIRECT_FREEKICK   =  4
uint8 STATE2_INDIRECT_FREEKICK =  5
uint8 STATE2_PENALTYKICK       =  6
uint8 STATE2_CORNER_KICK       =  7
uint8 STATE2_GOAL_KICK         =  8
uint8 STATE2_THROW_IN          =  9
