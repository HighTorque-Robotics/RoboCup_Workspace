/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file gamecontroller.cpp
 * @brief
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#include "dnetwork/gamecontroller.hpp"

namespace dnetwork {

using namespace std;
static const int FREQ = 2;

GameController::GameController(ros::NodeHandle* nh)
  : DProcess(FREQ, false)
  , nh_(nh)
  , last_valid_packet_timestamp_(0)
{
    if (!nh_->getParam("/ZJUDancer/GameControllerAddress", gameControllerAddress_))
        throw std::runtime_error("Can't get gamecontroller address!");

    if (!nh_->getParam("RobotId", playerNumber_))
        throw std::runtime_error("Can't get player number!");

    if (!nh->getParam("/ZJUDancer/TeamNumber", teamNumber_))
        throw std::runtime_error("Can't get team number!");

    ret_.team = (uint8_t)teamNumber_;
    ret_.player = (uint8_t)playerNumber_;
    ret_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

    pub_ = nh_->advertise<dmsgs::GCInfo>("/dnetwork_" + std::to_string(playerNumber_) + "/GCInfo", 1);
    // std::cout << "Hello1\n\n\n";
    transmitter_ = new dtransmit::DTransmit();
    transmitter_->addRawRecvFiltered(GAMECONTROLLER_DATA_PORT, gameControllerAddress_, [&](void* buffer, size_t size) {
        if (size == sizeof(RoboCupGameControlData)) {
            unique_lock<mutex> lock(dataLock_);
            // std::cout << "heard GC\n\n\n";
            ParseData(*(RoboCupGameControlData*)buffer);
        }
    });
    // std::cout << "Hello2\n\n\n";
    transmitter_->startService();
}

GameController::~GameController()
{
}

void
GameController::tick()
{
    unique_lock<mutex> lock(dataLock_);
    // manipulate data
    auto elapsed = (ros::Time::now() - last_valid_packet_timestamp_).toSec();
    connected_ = (elapsed < 3);

    TeamInfo *ourTeam, *enemyTeam;
    if (data_.teams[TEAM_CYAN].teamNumber == teamNumber_) {
        ourTeam = &(data_.teams[TEAM_CYAN]);
        enemyTeam = &(data_.teams[TEAM_MAGENTA]);
    } else {
        enemyTeam = &(data_.teams[TEAM_CYAN]);
        ourTeam = &(data_.teams[TEAM_MAGENTA]);
    }

    // std::cout << "secondary state: " << (int)data_.secondaryState << " "
    //           << (int)data_.secondaryStateInfo[0] << " "
    //         << (int)data_.secondaryStateInfo[1] << " "
    //         << (int)data_.secondaryStateInfo[2] << " "
    //         << (int)data_.secondaryStateInfo[3] << std::endl;

    int state2 = data_.secondaryState;
    int state2_team = (int)data_.secondaryStateInfo[0];

    // Refer to https://github.com/RoboCup-Humanoid-TC/GameController/blob/master/src/data/states/SecondaryStateInfo.java
    // and https://github.com/RoboCup-Humanoid-TC/GameController/blob/master/src/controller/ui/ui/customized/GameInterruptionButton.java
    // The team that is performing the free kick
    // - 0 for Start placing, robots should stay still, referee places the ball on the ground
    // - 1 for End placing, robots can place themselves toward the ball
    // - 2 for Execute, robots should stay still and referees ask to remove illegally positioned robots
    bool state2_ready = false;
    bool state2_freeze = false;
    if (state2 != STATE2_NORMAL) {
        state2_ready = ((int)data_.secondaryStateInfo[1] == 1);
        state2_freeze = ((int)data_.secondaryStateInfo[1] == 0) or ((int)data_.secondaryStateInfo[1] == 2);
    }

    bool ourDirectFreeKick = false;
    bool ourIndirectFreeKick = false;
    bool ourPenaltyKick = false;
    bool ourCornerKick = false;
    bool ourGoalKick = false;
    bool ourThrowIn = false;
    bool enemyDirectFreeKick = false;
    bool enemyIndirectFreeKick = false;
    bool enemyPenaltyKick = false;
    bool enemyCornerKick = false;
    bool enemyGoalKick = false;
    bool enemyThrowIn = false;

    if (state2 == STATE2_DIRECT_FREEKICK) {
        ourDirectFreeKick = (state2_team == teamNumber_);
        enemyDirectFreeKick = !ourDirectFreeKick;
    } else if (state2 == STATE2_INDIRECT_FREEKICK) {
        ourIndirectFreeKick = (state2_team == teamNumber_);
        enemyIndirectFreeKick = !ourIndirectFreeKick;
    } else if (state2 == STATE2_PENALTYKICK) {
        ourPenaltyKick = (state2_team == teamNumber_);
        enemyPenaltyKick = !ourPenaltyKick;
    } else if (state2 == STATE2_CORNER_KICK) {
        ourCornerKick = (state2_team == teamNumber_);
        enemyCornerKick = !ourCornerKick;
    } else if (state2 == STATE2_GOAL_KICK) {
        ourGoalKick = (state2_team == teamNumber_);
        enemyGoalKick = !ourGoalKick;
    } else if (state2 == STATE2_THROW_IN) {
        ourThrowIn = (state2_team == teamNumber_);
        enemyThrowIn = !ourThrowIn;
    }

    // std::cout << "\n\n\n\n\n\n";
    // std::cout << "state2: " << state2 << '\n'
    //           << "state2_team: " << state2_team << '\n'
    //           << "state2_ready: " << state2_ready << '\n'
    //           << "state2_freeze: " << state2_freeze << '\n'
    //
    //           << "ourDirectFreeKick: " << (ourDirectFreeKick ? "true" : "false" ) << '\n'
    //           << "ourIndirectFreeKick: " << (ourIndirectFreeKick ? "true" : "false" ) << '\n'
    //           << "ourPenaltyKick: " << (ourPenaltyKick ? "true" : "false" ) << '\n'
    //
    //           << "enemyDirectFreeKick: " << (enemyDirectFreeKick ? "true" : "false" ) << '\n'
    //           << "enemyIndirectFreeKick: " << (enemyIndirectFreeKick ? "true" : "false" ) << '\n'
    //           << "enemyPenaltyKick: " << (enemyPenaltyKick ? "true" : "false" ) << '\n';

    int ourScore = ourTeam->score;
    int enemyScore = enemyTeam->score;

    bool kickoff = (data_.kickOffTeam == teamNumber_);
    auto penalty = ourTeam->players[playerNumber_ - 1].penalty;
    penalised_ = (penalty != PENALTY_NONE);

    // FIXME(MWX): maybe chushiqing if the Referee misoperating
    info_.connected = connected_;
    info_.state = data_.state;
    info_.secondaryState = data_.secondaryState;
    info_.firstHalf = data_.firstHalf;
    info_.kickoff = kickoff;
    info_.secsRemaining = data_.secsRemaining < 10000 ? data_.secsRemaining : 0;
    info_.secondaryTime = data_.secondaryTime < 10000 ? data_.secondaryTime : 0;
    info_.secsTillUnpenalised = ourTeam->players[playerNumber_ - 1].secsTillUnpenalised;
    info_.ourScore = ourScore;
    info_.enemyScore = enemyScore;
    info_.teamCyan = teamCyan_;
    info_.penalised = penalised_;

    info_.ourPenaltyKick = ourPenaltyKick;
    info_.ourDirectFreeKick = ourDirectFreeKick;
    info_.ourIndirectFreeKick = ourIndirectFreeKick;
    info_.ourCornerKick = ourCornerKick;
    info_.ourGoalKick = ourGoalKick;
    info_.ourThrowIn = ourThrowIn;
    info_.enemyPenaltyKick = enemyPenaltyKick;
    info_.enemyDirectFreeKick = enemyDirectFreeKick;
    info_.enemyIndirectFreeKick = enemyIndirectFreeKick;
    info_.enemyCornerKick = enemyCornerKick;
    info_.enemyGoalKick = enemyGoalKick;
    info_.enemyThrowIn = enemyThrowIn;

    info_.state2Ready = state2_ready;
    info_.state2Freeze = state2_freeze;

    pub_.publish(info_);
    // transmitter_->sendRaw(GAMECONTROLLER_RETURN_PORT, (void*)&ret_, sizeof(ret_));
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
    inet_pton(AF_INET, "192.168.1.84", &addr.sin_addr);

    printf("Sending %ld bytes to %s:%d\n", sizeof(ret_), inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    for (int i = 0; i < sizeof(ret_); i++) {
        printf("%02X ", ((uint8_t*)&ret_)[i]);
    }
    printf("\n");

    sendto(sock, &ret_, sizeof(ret_), 0, (sockaddr*)&addr, sizeof(addr));
    close(sock);
}

void
GameController::ParseData(RoboCupGameControlData& gameData)
{
    if (!IsValidData(gameData)) {
        return;
    }

    if (gameData.teams[TEAM_CYAN].teamColour != TEAM_CYAN)
        RawSwapTeams(gameData);

    if (!GameDataEqual(gameData, data_))
        memcpy(&data_, &gameData, sizeof(RoboCupGameControlData));
}

bool
GameController::GameDataEqual(RoboCupGameControlData& gameData, RoboCupGameControlData& previous)
{
    return !memcmp((void*)&gameData, (void*)&previous, sizeof(RoboCupGameControlData));
}

bool
GameController::CheckHeader(char* header)
{
    for (int i = 0; i < 4; ++i) {
        if (header[i] != GAMECONTROLLER_STRUCT_HEADER[i]) {
            return false;
        }
    }
    return true;
}

bool
GameController::IsThisGame(RoboCupGameControlData& gameData)
{
    return !(gameData.teams[TEAM_CYAN].teamNumber != teamNumber_ && gameData.teams[TEAM_MAGENTA].teamNumber != teamNumber_);
}

bool
GameController::IsValidData(RoboCupGameControlData& gameData)
{
    if (!CheckHeader(gameData.header)) {
        ROS_WARN("Header invalid, recv: %s need: %s", gameData.header, GAMECONTROLLER_STRUCT_HEADER);
        return false;
    }

    if (gameData.version != GAMECONTROLLER_STRUCT_VERSION) {
        ROS_WARN("Version invalid, recv: %d, need: %d", gameData.version, GAMECONTROLLER_STRUCT_VERSION);
        return false;
    }

    if (!IsThisGame(gameData)) {
        ROS_WARN("Data not for this game!");
        return false;
    }

    // Data is valid OvO
    last_valid_packet_timestamp_ = ros::Time::now();
    ROS_DEBUG("Data valid OvO");
    return true;
}

void
GameController::RawSwapTeams(RoboCupGameControlData& gameData)
{
    //    auto teamSize = sizeof(TeamInfo);
    //    TeamInfo* cyanTeam = &(gameData.teams[TEAM_CYAN]);
    //    TeamInfo* magentaTeam = &(gameData.teams[TEAM_MAGENTA]);
    //
    //    TeamInfo tempTeam;
    //    memcpy(&tempTeam, cyanTeam, teamSize);
    std::swap(gameData.teams[TEAM_CYAN], gameData.teams[TEAM_MAGENTA]);
}
}
