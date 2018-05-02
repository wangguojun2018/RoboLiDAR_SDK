/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 1.6.5
 * Date: 2018.3
 *
 * DESCRIPTION
 *
 * Robosense communication module, for output perception result to user computer.
 *
 */

#ifndef ROBOSENSE_COMMUNICATION_TYPE_H
#define ROBOSENSE_COMMUNICATION_TYPE_H

#include <stdint.h>

//boost
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace Robosense{

    struct PoseMsg
    {
//public:
//    PoseMsg() {}
//    PoseMsg(float & in_x, float & in_y, float & in_z, float & in_roll, float & in_pitch, float & in_yaw):
//            x( in_x ), y( in_y ), z( in_z ), roll( in_roll ), pitch( in_pitch ), yaw( in_yaw ){}

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & x;
            ar & y;
            ar & z;
            ar & roll;
            ar & pitch;
            ar & yaw;
        }
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
    };


    struct Header{
//public:
//    Header() {}
//    Header( unsigned short int& in_head, unsigned short int& in_object_num, unsigned int& in_length,
//             unsigned long int& in_frame_id, unsigned long int& in_timestamp):
//            head(in_head), object_num(in_object_num), length(in_length), frame_id(in_frame_id), timestamp(in_timestamp){}

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & head;
            ar & object_num;
            ar & length;
            ar & frame_id;
            ar & timestamp;
        }

        unsigned short int head;
        unsigned short int object_num;
        unsigned int length;
        unsigned long int frame_id;
        unsigned long int timestamp;
    };

    struct Point3fMsg{
//public:
//    Point3fMsg() {}
//    Point3fMsg(float& in_x, float& in_y, float& in_z):
//              x(in_x), y(in_y), z(in_z){}

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & x & y & z;
        }
        float x, y, z;
    };

    struct Direction3fMsg{
    public:
        Direction3fMsg() {}
        Direction3fMsg(float& in_x, float& in_y, float& in_z):
                x(in_x), y(in_y), z(in_z) {}

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & x & y & z;
        }

        float x,y,z;
    };

    struct Point2fMsg{
//public:
//    Point2fMsg() {}
//    Point2fMsg(float& in_x, float& in_y): x(in_x), y(in_y) {}

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & x & y;
        }
        float x, y;
    };

    struct boxMsg{
//public:
//    boxMsg() {}
//    boxMsg(float& in_length, float& in_width, float& in_height, Point3fMsg& in_location,
//           Direction3fMsg& in_direction, float& in_yaw, Point3fMsg& in_nearest_point):
//    length(in_length), width(in_width), height(in_height), location(in_location),
//    direction(in_direction), yaw(in_yaw), nearest_point(in_nearest_point) {}

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & length;
            ar & width;
            ar & height;
            ar & location;
            ar & direction;
            ar & yaw;
            ar & nearest_point;
        }

        float length;
        float width;
        float height;
        Point3fMsg location;
        Direction3fMsg direction;
        float yaw;
        Point3fMsg nearest_point;
    };

    struct PerceptResultMsg {
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & box;
            ar & track_id;
            ar & track_probability;
            ar & velocity;
            ar & acceleration;
            ar & velocity_abs;
            ar & acceleration_abs;
            ar & angle_velocity;
            ar & life;
            ar & visible_life;
            ar & robustness;

            ar & label;
            ar & label_confidence;

            ar & is_background;
        }

        boxMsg box;
        unsigned int track_id;
        float track_probability;
        Point2fMsg velocity;
        Point2fMsg acceleration;
        Point2fMsg velocity_abs;
        Point2fMsg acceleration_abs;
        float angle_velocity;
        float life;
        float visible_life;
        float robustness;

        signed char label;
        float label_confidence;

        bool is_background;
    };

    struct CommunicationMsg{
        friend class boost::serialization::access;
        Header header;
        PoseMsg pose;
//    char perceptions[20000];

        //boost
        std::vector<PerceptResultMsg> pcep_msg;
        template<typename Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & header;
            ar & pose;
            ar & pcep_msg;
        }
    };
}


#endif //ROBOSENSE_COMMUNICATION_TYPE_H
