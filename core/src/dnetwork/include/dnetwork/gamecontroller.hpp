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

#pragma once
#include "dmsgs/GCInfo.h"
#include "dnetwork/RoboCup/RoboCupGameControlData.h"
#include <dprocess/dprocess.hpp>
#include <dtransmit/dtransmit.hpp>

#include <mutex>
#include <ros/ros.h>
#include <string>

namespace dnetwork {
/**
  * @brief Send & Receive GameControll messages from GC server
  */
class GameController : public dprocess::DProcess<GameController>
{
  public:
    /**
     * @brief GameController constructor
     *
     * @param nh - node handler
     */
    explicit GameController(ros::NodeHandle* nh);
    /**
     * @brief GameController destructor
     */
    ~GameController();
    /**
     * @brief Tick
     */
    void tick() override;

  private:
    /**
     * @brief Parse data received from GC server
     *
     * @param gameData - received data buffer
     */
    void ParseData(RoboCupGameControlData& gameData);

    /**
     * @brief Check validity for received data
     *
     * @param gameData - received data buffer
     *
     * @return - whether or not received data is valid
     */
    bool IsValidData(RoboCupGameControlData& gameData);

    /**
     * @brief Check validity for header of received data
     *
     * @param header - header of received data buffer
     *
     * @return whether or not the heade of received data is valid
     */
    bool CheckHeader(char* header);

    /**
     * @brief Check whether or not received data is about this game
     *
     * @param gameData - received data buffer
     *
     * @return whether or not received data is about this game
     */
    bool IsThisGame(RoboCupGameControlData& gameData);

    /**
     * @brief Swap data about team cyan and team magenta
     *
     * @param gameData - received data buffer
     */
    void RawSwapTeams(RoboCupGameControlData& gameData);

    /**
     * @brief Check whether or not current data is equal to previous one 
     *
     * @param gameData - currently received data buffer
     * @param pervious - previously received data buffer
     *
     * @return whether or not current data is equal to previous one 
     */
    bool GameDataEqual(RoboCupGameControlData& gameData, RoboCupGameControlData& pervious);

  private:
    //! Node handler
    ros::NodeHandle* nh_;
    //! Publisher for GameControl messages
    ros::Publisher pub_;
    //! GameControl messages
    dmsgs::GCInfo info_;

    //! Transmitter
    dtransmit::DTransmit* transmitter_;
    //! IP address for GC server
    std::string gameControllerAddress_;

    //! Received GC data 
    RoboCupGameControlData data_;
    //! Timestamp when last valid packet is received
    ros::Time last_valid_packet_timestamp_;

    //! Lock for data
    std::mutex dataLock_;
    //! Return data for GC Server
    RoboCupGameControlReturnData ret_;

    //! Flag for whether or not GC server is connected
    bool connected_;
    //! Flag for whether or not this player is penalised
    bool penalised_;
    //! Flag for whether or not our team is cyan
    bool teamCyan_;

    //! Robot ID for current player
    int playerNumber_;
    //! Team ID for current team
    int teamNumber_;
};
}
