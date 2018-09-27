// Software License Agreement (BSD-3-Clause)
//
// Copyright 2018 The University of North Carolina at Chapel Hill
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Jeff Ichnowski

#ifndef NAOCUP_H
#define NAOCUP_H

#include <cmath>
#include <cassert>
#include <err.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include "linear.hpp"
#include "collide.hpp"
#include "alloc.hpp"
#include "prrts.hpp"
// #include "hrtimer.hpp"
#include "naocup.hpp"

namespace nao_cup {
    constexpr unsigned DIMENSIONS = 10;
    template <typename S> constexpr S INCHES = S(0.0254);


    template <typename S> constexpr S PI = S(M_PI);

    /* cos(25 degrees) */
    template <typename S> constexpr S RHAND_UP_GOAL = S(0.90630778703665);

    /* cos(45 degrees) */
    template <typename S> constexpr S LHAND_UP_GOAL = S(0.707106781186548);

    template <typename S> constexpr S CENTER_TORSO_RADIUS = S(66.7 / 1000.0);
    constexpr unsigned CUP_BEAD_COUNT = 8;
    template <typename S> constexpr S CUP_BEAD_RADIUS = S(2.0/4.0) * INCHES<S>;
    template <typename S> constexpr S CUP_DIAMETER = (S(2.5) * INCHES<S>);
    template <typename S> constexpr S CUP_HEIGHT = ((S(4.0) + S(3.0)/S(8.0)) * INCHES<S>);
    template <typename S> constexpr S CUP_GRIP_HEIGHT = (S(1.0) * INCHES<S>);
    template <typename S> constexpr S CUP_BASE_TO_BOWL = ((S(1.0) + S(5.0)/S(8.0)) * INCHES<S>);
    template <typename S> constexpr S CUP_GRIP_DIAMETER = (S(5.0)/S(8.0) * INCHES<S>);
    template <typename S> constexpr S CUP_GRIP_CAPSULE_HEIGHT = (CUP_BASE_TO_BOWL<S> - CUP_GRIP_DIAMETER<S>*S(2.0));
    template <typename S> constexpr S CUP_BOWL_HEIGHT = (CUP_HEIGHT<S> - CUP_BASE_TO_BOWL<S>);

    template <typename S> constexpr S BALL_RADIUS = S(0.015);

    constexpr unsigned NUM_OBSTACLES = 4;
    template <typename S> constexpr S PLANAR_SPHERE_RADIUS = S(25.0);
    template <typename S> constexpr S TABLE_OFFSET_Z = S(0.09);
    template <typename S> constexpr S DISCRETIZATION = (S(1.0) * PI<S> / S(180.0));

    enum {
        SPHERE, CAPSULE
    };

    template <typename S>
    struct collision_object {
        int type;
        Transform<S> *transform;
        S radius;
        S length;
    };

    template <typename S>
    struct nao_arm {
#if 0
        /* joint position */
        S shoulder_pitch;
        S shoulder_roll;
        S elbow_yaw;
        S elbow_roll;
        S wrist_yaw;
#endif

        /* computed */
        Transform<S> upper_capsule_transform;
        Transform<S> lower_capsule_transform;
        Transform<S> transform_to_hand;
    };

#ifdef COMPUTE_LEG
    template <typename S>
    struct nao_leg {
        S hip_roll;
        S hip_pitch;
        S knee_pitch;
        S ankle_pitch;
        S ankle_roll;
    };
#endif

    template <typename S>
    struct nao_robot {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Transform<S> transform;

        S head_yaw;
        S head_pitch;
        struct nao_arm<S> left_arm;
        struct nao_arm<S> right_arm;
        S hip_yaw_pitch;
#ifdef COMPUTE_LEG
        struct nao_leg<S> left_leg;
        struct nao_leg<S> right_leg;
#endif
        Transform<S> head_center;
        Transform<S> head_capsule;
        /* Transform<S> center_torso; */
        Transform<S> upper_torso;
        Transform<S> lower_torso;

        /* bool in_self_collision; */
    };

    template <typename S>
    struct nao_collision {
        struct collision_object<S> left_arm[3];
        struct collision_object<S> right_arm[3];
        struct collision_object<S> torso[3];
        struct collision_object<S> head[2];
        struct collision_object<S> cup[CUP_BEAD_COUNT * 2 + 2];
        struct collision_object<S> obstacles[NUM_OBSTACLES];
        struct collision_object<S> ball[1];
        Transform<S> coke_transform;
        Transform<S> pepsi_transform;
        Transform<S> table_transform;
        Transform<S> backwall_transform;
        Transform<S> ball_transform;
        Transform<S> cup_stem_transform;
        Transform<S> cup_bowl_transform;
        Transform<S> cup_bead_transform[CUP_BEAD_COUNT * 2];
        S bead_x[CUP_BEAD_COUNT];
        S bead_y[CUP_BEAD_COUNT];
    };

    template <typename S>
    struct nao_world {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        struct nao_robot<S> robot;

        Vec3<S> cup_up_norm;

        bool in_collision;
        bool in_goal;
        bool cup_is_up;
        bool is_clear;
        S dist_to_goal;

        struct nao_collision<S> collide;
    };

    enum nao_joint {
        R_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        R_ELBOW_YAW,
        R_ELBOW_ROLL,
        R_WRIST_YAW,

        L_SHOULDER_PITCH,
        L_SHOULDER_ROLL,
        L_ELBOW_YAW,
        L_ELBOW_ROLL,
        L_WRIST_YAW,
    };

    template <typename S>
    constexpr S RAD(S x) { return ((x) * S(PI<S> / S(180.0))); }

    template <typename S> constexpr S HEAD_YAW_MIN =           RAD(S(-119.5));
    template <typename S> constexpr S HEAD_YAW_MAX =           RAD( S(119.5));
    template <typename S> constexpr S HEAD_PITCH_MIN =         RAD( S(-38.5));
    template <typename S> constexpr S HEAD_PITCH_MAX =         RAD(  S(29.5));

    template <typename S> constexpr S L_SHOULDER_PITCH_MIN =   RAD(S(-119.5));
    template <typename S> constexpr S L_SHOULDER_PITCH_MAX =   RAD( S(119.5));
    template <typename S> constexpr S L_SHOULDER_ROLL_MIN =    RAD(   S(0.5));
    template <typename S> constexpr S L_SHOULDER_ROLL_MAX =    RAD(  S(94.5));
    template <typename S> constexpr S L_ELBOW_YAW_MIN =        RAD(S(-119.5));
    template <typename S> constexpr S L_ELBOW_YAW_MAX =        RAD( S(119.5));
    template <typename S> constexpr S L_ELBOW_ROLL_MIN =       RAD( S(-89.5));
    template <typename S> constexpr S L_ELBOW_ROLL_MAX =       RAD(  S(-0.5));
    template <typename S> constexpr S L_WRIST_YAW_MIN =        RAD(S(-104.5));
    template <typename S> constexpr S L_WRIST_YAW_MAX =        RAD( S(104.5));

    template <typename S> constexpr S R_SHOULDER_PITCH_MIN =   RAD(S(-119.5));
    template <typename S> constexpr S R_SHOULDER_PITCH_MAX =   RAD( S(119.5));
    template <typename S> constexpr S R_SHOULDER_ROLL_MIN =    RAD( S(-94.5));
    template <typename S> constexpr S R_SHOULDER_ROLL_MAX =    RAD(  S(-0.5));
    template <typename S> constexpr S R_ELBOW_YAW_MIN =        RAD(S(-119.5));
    template <typename S> constexpr S R_ELBOW_YAW_MAX =        RAD( S(119.5));
    template <typename S> constexpr S R_ELBOW_ROLL_MIN =       RAD(   S(0.5));
    template <typename S> constexpr S R_ELBOW_ROLL_MAX =       RAD(  S(89.5));
    template <typename S> constexpr S R_WRIST_YAW_MIN =        RAD(S(-104.5));
    template <typename S> constexpr S R_WRIST_YAW_MAX =        RAD( S(104.5));

/* These are from the spec sheets */
    template <typename S> constexpr S NECK_OFFSET_Z = S(126.50 / 1000.0);
    template <typename S> constexpr S SHOULDER_OFFSET_Y = S(98.00 / 1000.0);
    template <typename S> constexpr S UPPER_ARM_LENGTH = S(90.00 / 1000.0);
    template <typename S> constexpr S LOWER_ARM_LENGTH = S(50.55 / 1000.0);
    template <typename S> constexpr S SHOULDER_OFFSET_Z = S(100.00 / 1000.0);
    template <typename S> constexpr S HAND_OFFSET_X = S(58.00 / 1000.0);
    template <typename S> constexpr S HIP_OFFSET_Z = S(85.00 / 1000.0);
    template <typename S> constexpr S HIP_OFFSET_Y = S(50.00 / 1000.0);
    template <typename S> constexpr S THIGH_LENGTH = S(100.00 / 1000.0);
    template <typename S> constexpr S TIBIA_LENGTH = S(102.74 / 1000.0);
    template <typename S> constexpr S FOOT_HEIGHT = S(45.11 / 1000.0);
    template <typename S> constexpr S HAND_OFFSET_Z = S(15.90 / 1000.0);

/* These are made up and need verification */
    template <typename S> constexpr S FOOT_WIDTH = S(90.0 / 1000.0);
    template <typename S> constexpr S FOOT_LENGTH = S(150.0 / 1000.0);
    template <typename S> constexpr S FOOT_OFFSET_X = S(25.0 / 1000.0);
    template <typename S> constexpr S FOOT_OFFSET_Y = S(10.0 / 1000.0);
    template <typename S> constexpr S FOOT_OFFSET_Z = FOOT_HEIGHT<S> / S(2.0);

    template <typename S> constexpr S HEAD_RADIUS = S(115.0/2.0 / 1000.0);
    template <typename S> constexpr S EAR_RADIUS = S(90.0/2.0 / 1000.0);
    template <typename S> constexpr S HEAD_WIDTH = S(133.0 / 1000.0);
    template <typename S> constexpr S TORSO_RADIUS = S(75.0 / 1000.0);
    template <typename S> constexpr S ARM_RADIUS = S(66.7/2.0 / 1000.0);
    template <typename S> constexpr S LEG_RADIUS = S(88.9/2.0 / 1000.0);
    template <typename S> constexpr S HAND_RADIUS = S(20.0 / 1000.0);
    template <typename S> constexpr S HAND_WIDTH = S(50.0 / 1000.0);


#define check_angle(joint, angle) assert((joint##_MIN) <= (angle) && (angle) <= (joint##_MAX));

    template <typename S>
    const S* nao_init_config() {
        static const S array[] = {
            S(1.125998),
            S(-0.691876),
            S(1.888312),
            S(0.776246),
            S(0.245398),

            S(1.259372),
            S(0.279146),
            S(-1.587732),
            S(-0.510780),
            S(-1.823800),
        };
        return array;
    }

    template <typename S>
    const S* nao_min_config() {
        static const S array[] = {
            R_SHOULDER_PITCH_MIN<S>,
            R_SHOULDER_ROLL_MIN<S>,
            R_ELBOW_YAW_MIN<S>,
            R_ELBOW_ROLL_MIN<S>,
            R_WRIST_YAW_MIN<S>,
            L_SHOULDER_PITCH_MIN<S>,
            L_SHOULDER_ROLL_MIN<S>,
            L_ELBOW_YAW_MIN<S>,
            L_ELBOW_ROLL_MIN<S>,
            L_WRIST_YAW_MIN<S>,
        };
        return array;
    }

    template <typename S>
    const S* nao_max_config() {
        static const S array[] = {
            R_SHOULDER_PITCH_MAX<S>,
            R_SHOULDER_ROLL_MAX<S>,
            R_ELBOW_YAW_MAX<S>,
            R_ELBOW_ROLL_MAX<S>,
            R_WRIST_YAW_MAX<S>,
            L_SHOULDER_PITCH_MAX<S>,
            L_SHOULDER_ROLL_MAX<S>,
            L_ELBOW_YAW_MAX<S>,
            L_ELBOW_ROLL_MAX<S>,
            L_WRIST_YAW_MAX<S>,
        };
        return array;
    }

    template <typename S>
    const S* nao_target_config() {
        static const S array[] = {
            S(0.258284303377494),
            S(-0.2699099199363406),
            S(-0.01113121187052224),
            S(1.2053012757652763),
            S(1.2716626717484503),
            S(-0.9826967097045605),
            S(0.07355836822937814),
            S(0.25450053440459897),
            S(-0.9512909033938429),
            S(-0.5297424293532234),
        };
        return array;
    }

    template <typename S>
    bool collide_objects(struct collision_object<S> *a, struct collision_object<S> *b) {
        switch ((a->type << 1) | b->type) {
        case (SPHERE<<1) | SPHERE:
            return collide_sphere_sphere(
                a->transform, a->radius,
                b->transform, b->radius);

        case (SPHERE<<1) | CAPSULE:
            return collide_sphere_capsule(
                a->transform, a->radius,
                b->transform, b->length, b->radius);

        case (CAPSULE<<1) | SPHERE:
            return collide_sphere_capsule(
                b->transform, b->radius,
                a->transform, a->length, a->radius);

        case (CAPSULE<<1) | CAPSULE:
            return collide_capsule_capsule(
                a->transform, a->length, a->radius,
                b->transform, b->length, b->radius);

        default:
            assert(0);
            return 1;
        }
    }

    template <typename S>
    bool collide_object_lists(
        struct collision_object<S> *a, size_t n,
        struct collision_object<S> *b, size_t m)
    {
        size_t i, j;
        bool collision = false;

        for (i=0 ; i<n ; ++i) {
            for (j=0 ; j<m ; ++j) {
                if (collide_objects(a + i, b + j)) {
                    collision = true;
                }
            }
        }
        return collision;
    }

    template <typename S>
    void compute_head(Transform<S> *transform, nao_robot<S> *robot) {
        Transform<S> head_yaw;
        Transform<S> head_pitch;
        Transform<S> head_capsule_rotate;

        m4_rotate<S>(&head_yaw, transform, robot->head_yaw, S(0), S(0), S(1));
        m4_rotate<S>(&head_pitch, &head_yaw, robot->head_pitch, S(0), S(1), S(0));
        m4_translate<S>(&robot->head_center, &head_pitch, S(0), S(0), HEAD_RADIUS<S>);

        // head_sphere
        //   center = head_center
        //   radius: HEAD_RADIUS

        m4_rotate<S>(&head_capsule_rotate, &robot->head_center, PI<S>/S(2.0), S(1), S(0), S(0));
        m4_translate<S>(&robot->head_capsule, &head_capsule_rotate,
                        S(0), S(0), -(HEAD_WIDTH<S> - EAR_RADIUS<S>*2)/S(2));

        // head_capsule
        //   center = head_capsule_transform
        //   radius: EAR_RADIUS
        //   height: HEAD_WIDTH - EAR_RADIUS*2.0
    }

    template <typename S>
    void compute_arm(Transform<S> *transform, nao_arm<S> *arm, const S *config) {
        Transform<S>  shoulder_pitch_matrix,
                shoulder_roll_matrix,
                translate_to_elbow,
                elbow_yaw_matrix,
                elbow_roll_matrix,
                wrist_yaw_matrix,
                translate_to_hand,
                rotate_hand;

        m4_rotate<S>(&shoulder_pitch_matrix, transform, config[R_SHOULDER_PITCH], S(0.0), S(1.0), S(0.0));
        m4_rotate<S>(&shoulder_roll_matrix, &shoulder_pitch_matrix, config[R_SHOULDER_ROLL], S(0.0), S(0.0), S(1.0));
        m4_rotate<S>(&arm->upper_capsule_transform, &shoulder_roll_matrix, PI<S>/S(2.0), S(0.0), S(1.0), S(0.0));
        m4_translate<S>(&translate_to_elbow, &shoulder_roll_matrix, UPPER_ARM_LENGTH<S>, S(0.0), S(0.0));
        m4_rotate<S>(&elbow_yaw_matrix, &translate_to_elbow, config[R_ELBOW_YAW], S(1.0), S(0.0), S(0.0));
        m4_rotate<S>(&elbow_roll_matrix, &elbow_yaw_matrix, config[R_ELBOW_ROLL], S(0.0), S(0.0), S(1.0));
        m4_rotate<S>(&arm->lower_capsule_transform, &elbow_roll_matrix, PI<S>/S(2.0), S(0.0), S(1.0), S(0.0));
        m4_rotate<S>(&wrist_yaw_matrix, &elbow_roll_matrix, config[R_WRIST_YAW], S(1.0), S(0.0), S(0.0));

        /*
         * the 0.01 added below is to account for the object's offset
         * within the grasp. it should probably be moved to a constant
         * somewhere else
         */
        m4_translate<S>(&translate_to_hand, &wrist_yaw_matrix,
                        LOWER_ARM_LENGTH<S> + HAND_OFFSET_X<S> + S(0.01),
                        S(0),
                        -HAND_OFFSET_Z<S> - S(0.01));

        m4_rotate<S>(&rotate_hand, &translate_to_hand, -PI<S>/S(2.0), S(1.0), S(0.0), S(0.0));
        m4_translate<S>(&arm->transform_to_hand, &rotate_hand, S(0.0), S(0.0), (HAND_WIDTH<S> - HAND_RADIUS<S>*S(2))/S(2));
    }

#ifdef COMPUTE_LEG
    template <typename S>
    void compute_leg(Transform<S> *transform, nao_leg<S> *leg) {
        Transform<S>  hip_roll_matrix,
                hip_pitch_matrix,
                translate_to_knee,
                knee_matrix,
                translate_to_ankle,
                ankle_pitch_matrix,
                ankle_roll_matrix;

        m4_rotate(&hip_roll_matrix, transform, leg->hip_roll, S(1.0), S(0.0), S(0.0));
        m4_rotate(&hip_pitch_matrix, &hip_roll_matrix, leg->hip_pitch, S(0.0), S(1.0), S(0.0));

        m4_translate(&translate_to_knee, &hip_pitch_matrix, S(0.0), S(0.0), -THIGH_LENGTH<S>);

        m4_rotate(&knee_matrix, &translate_to_knee, leg->knee_pitch, S(0.0), S(1.0), S(0.0));
        m4_translate(&translate_to_ankle, &knee_matrix, S(0.0), S(0.0), -TIBIA_LENGTH<S>);
        m4_rotate(&ankle_pitch_matrix, &translate_to_ankle, leg->ankle_pitch, S(0.0), S(1.0), S(0.0));
        m4_rotate(&ankle_roll_matrix, &ankle_pitch_matrix, leg->ankle_roll, S(1.0), S(0.0), S(0.0));
    }
#endif

    inline bool dbool(const char *name, bool value) {
        /* if (value) printf("%s: %s\n", name, value ? "true" : "false"); */
        return value;
    }

    template <typename S>
    void init_collisions(nao_world<S> *world) {
        nao_collision<S> *c = &world->collide;

        c->right_arm[0].type = CAPSULE;
        c->right_arm[0].transform = &world->robot.right_arm.lower_capsule_transform;
        c->right_arm[0].radius = ARM_RADIUS<S>;
        c->right_arm[0].length = LOWER_ARM_LENGTH<S> + HAND_OFFSET_X<S> - ARM_RADIUS<S>;

        c->left_arm[0].type = CAPSULE;
        c->left_arm[0].transform = &world->robot.left_arm.lower_capsule_transform;
        c->left_arm[0].radius = ARM_RADIUS<S>;
        c->left_arm[0].length = LOWER_ARM_LENGTH<S> + HAND_OFFSET_X<S> - ARM_RADIUS<S>;

        c->torso[0].type = SPHERE;
        c->torso[0].transform = &world->robot.transform; /* center_torso */
        c->torso[0].radius = S(55.6 / 1000.0);

        c->torso[1].type = SPHERE;
        c->torso[1].transform = &world->robot.upper_torso;
        c->torso[1].radius = CENTER_TORSO_RADIUS<S>;

        c->torso[2].type = SPHERE;
        c->torso[2].transform = &world->robot.lower_torso;
        c->torso[2].radius = (HIP_OFFSET_Z<S>/S(2.0));

        c->head[0].type = SPHERE;
        c->head[0].transform = &world->robot.head_center;
        c->head[0].radius = HEAD_RADIUS<S>;

        c->head[1].type = CAPSULE;
        c->head[1].transform = &world->robot.head_capsule;
        c->head[1].radius = EAR_RADIUS<S>;
        c->head[1].length = HEAD_WIDTH<S> - EAR_RADIUS<S>/S(2.0);

        // 12oz coke bottle
        c->obstacles[0].type = CAPSULE;
        c->obstacles[0].transform = &c->coke_transform;
        c->obstacles[0].radius = S(2.5)/S(2.0) * INCHES<S>;
        c->obstacles[0].length = (S(6.75) - S(2.5)/S(2.0)) * INCHES<S>;

        // 20oz pepsi near right hand
        c->obstacles[1].type = CAPSULE;
        c->obstacles[1].transform = &c->pepsi_transform;
        c->obstacles[1].radius = S(3.0)/S(2.0) * INCHES<S>;
        c->obstacles[1].length = (S(8.5) - S(3.0)/S(2.0)) * INCHES<S>;

        // table
        c->obstacles[2].type = SPHERE;
        c->obstacles[2].transform = &c->table_transform;
        c->obstacles[2].radius = PLANAR_SPHERE_RADIUS<S>;

        // back wall
        c->obstacles[3].type = SPHERE;
        c->obstacles[3].transform = &c->backwall_transform;
        c->obstacles[3].radius = PLANAR_SPHERE_RADIUS<S>;

        // ball in hand
        c->ball[0].type = SPHERE;
        c->ball[0].transform = &c->ball_transform;
        c->ball[0].radius = BALL_RADIUS<S>;

        // cup stem
        c->cup[0].type = CAPSULE;
        c->cup[0].transform = &c->cup_stem_transform;
        c->cup[0].radius = CUP_GRIP_DIAMETER<S> / S(2.0);
        c->cup[0].length = -CUP_GRIP_CAPSULE_HEIGHT<S>;

        // cup bowl
        c->cup[1].type = CAPSULE;
        c->cup[1].transform = &c->cup_bowl_transform;
        c->cup[1].radius = CUP_DIAMETER<S> / S(2.0);
        c->cup[1].length = CUP_BOWL_HEIGHT<S> - CUP_DIAMETER<S>;

        // cup beads (representing base and cap)
        for (unsigned i=0 ; i<CUP_BEAD_COUNT ; ++i) {
                S a = PI<S> * S(2.0) * (S)i / (S)CUP_BEAD_COUNT;
                c->bead_x[i] = cos(a) * (CUP_DIAMETER<S> / S(2.0) - CUP_BEAD_RADIUS<S>);
                c->bead_y[i] = sin(a) * (CUP_DIAMETER<S> / S(2.0) - CUP_BEAD_RADIUS<S>);
                c->cup[i*2+2].type = SPHERE;
                c->cup[i*2+2].transform = &c->cup_bead_transform[i*2];
                c->cup[i*2+2].radius = CUP_BEAD_RADIUS<S>;

                c->cup[i*2+3].type = SPHERE;
                c->cup[i*2+3].transform = &c->cup_bead_transform[i*2+1];
                c->cup[i*2+3].radius = CUP_BEAD_RADIUS<S>;
        }

        // self-collision defined as:
        //   torso-cells(3) & [ left-arm-cells, right-arm-cells ]
        //   left-arm-cells & right-arm-cells
        //   head-cells & [ left-arm-cells, right-arm-cells ]

        // other-collisions
        //   obstacles & [ left-arm-cells, right-arm-cells ]
        //   cup  & [ left-arm-cells, head-cells, torso-cells, obstacles ]
        //   ball & [ right-arm-cells, head-cells, torso-cells, obstacles ]
    }

    template <typename S>
    bool check_collisions(nao_world<S> *world) {
        nao_collision<S> *c = &world->collide;

        m4_translate<S>(&c->coke_transform, &world->robot.transform, S(0.12), S(0.08), -TABLE_OFFSET_Z<S>);
        m4_translate<S>(&c->pepsi_transform, &world->robot.transform, S(0.12) + S(2.5) * INCHES<S>, S(-0.12), -TABLE_OFFSET_Z<S>);
        m4_translate<S>(&c->table_transform, &world->robot.transform, S(0.0), S(0.0), -PLANAR_SPHERE_RADIUS<S> - TABLE_OFFSET_Z<S>);
        m4_translate<S>(&c->backwall_transform, &world->robot.transform, -PLANAR_SPHERE_RADIUS<S> - CENTER_TORSO_RADIUS<S>, S(0.0), S(0.0));
        m4_translate<S>(&c->ball_transform, &world->robot.left_arm.transform_to_hand, S(0.0), BALL_RADIUS<S> / S(2.0) + S(0.01), S(0.0));
        m4_translate<S>(&c->cup_stem_transform, &world->robot.right_arm.transform_to_hand,
                     S(0.0), S(0.0), CUP_GRIP_HEIGHT<S> / S(2.0));
        m4_translate<S>(&c->cup_bowl_transform, &world->robot.right_arm.transform_to_hand,
                     S(0.0), S(0.0), CUP_GRIP_HEIGHT<S> / S(2.0) + CUP_DIAMETER<S> / S(2.0));

        for (unsigned i=0 ; i<CUP_BEAD_COUNT ; ++i) {
            S x = c->bead_x[i];
            S y = c->bead_y[i];

            m4_translate<S>(&c->cup_bead_transform[i*2], &world->robot.right_arm.transform_to_hand,
                         x, y, CUP_BOWL_HEIGHT<S> + CUP_GRIP_HEIGHT<S> / S(2.0) - CUP_BEAD_RADIUS<S>);
            m4_translate<S>(&c->cup_bead_transform[i*2+1], &world->robot.right_arm.transform_to_hand,
                         x, y, CUP_GRIP_HEIGHT<S> / S(2.0) - CUP_BASE_TO_BOWL<S> + CUP_BEAD_RADIUS<S>);
        }

        /*
         * we're forcing unconditional evaluation here to make sure
         * there's no per-sampling-region bias on computation time.
         */
        world->in_collision = 0 != (
            /* self collision */
            (dbool("torso<->right arm", collide_object_lists(c->torso, 3, c->right_arm, 1))?1:0) +
            (dbool("torso<->left arm", collide_object_lists(c->torso, 3, c->left_arm, 1))?1:0) +
            (dbool("left<->right arms", collide_object_lists(c->left_arm, 1, c->right_arm, 1))?1:0) +
            (dbool("head<->right arm", collide_object_lists(c->head, 2, c->right_arm, 1))?1:0) +
            (dbool("head<->left arm", collide_object_lists(c->head, 2, c->left_arm, 1))?1:0) +

            /* environmental collisions */
            (dbool("right arm<->obstacles", collide_object_lists(c->right_arm, 1, c->obstacles, NUM_OBSTACLES))?1:0) +
            (dbool("left arm<->obstacles", collide_object_lists(c->left_arm, 1, c->obstacles, NUM_OBSTACLES))?1:0) +
            (dbool("cup<->torso", collide_object_lists(c->cup, CUP_BEAD_COUNT * 2 + 2, c->torso, 3))?1:0) +
            (dbool("cup<->obstacles", collide_object_lists(c->cup, CUP_BEAD_COUNT * 2 + 2, c->obstacles, NUM_OBSTACLES))?1:0) +
            (dbool("cup<->left arm", collide_object_lists(c->cup, CUP_BEAD_COUNT * 2 + 2, c->left_arm, 1))?1:0) +
            (dbool("cup<->head", collide_object_lists(c->cup, CUP_BEAD_COUNT * 2 + 2, c->head, 2))?1:0) +
            (collide_object_lists(c->right_arm, 1, c->ball, 1)?1:0) +
            (collide_object_lists(c->head, 2, c->ball, 1)?1:0) +
            (collide_object_lists(c->torso, 3, c->ball, 1)?1:0) +
            (collide_object_lists(c->obstacles, NUM_OBSTACLES, c->ball, 1)?1:0));

        return world->in_collision;
    }

    template <typename S>
    void compute(nao_world<S> *world, const S *config)
    {
        Transform<S> head_transform;
        Transform<S> right_arm_transform;
        Transform<S> left_arm_transform;
#ifdef COMPUTE_LEG
        Transform<S> right_hip_offset;
        Transform<S> right_leg_transform;
        Transform<S> left_hip_offset;
        Transform<S> left_leg_transform;
#endif
        Vec3<S> r_pos;
        Vec3<S> cup_bowl_center;
        Vec3<S> l_pos;
        Vec3<S> cup_up;
        Vec3<S> cup_up_norm;
        Vec3<S> left_hand_ball_down;
        Vec3<S> ball_down;
        Vec3<S> ball_down_norm;
        bool cup_is_up;

#if 0
        /*
         * - config is 10 x sizeof(S) = 80 bytes
         * - cache line is 64 bytes
         *
         * the first and last elements are guaranteed to be on
         * different cache lines, we issue two prefetches to get both
         * lines.
         */
        __builtin_prefetch(config);
        __builtin_prefetch(((void *)config) + CACHE_LINE_SIZE);
#endif

        world->robot.transform = Transform<S>::Identity();

        m4_translate<S>(&head_transform, &world->robot.transform, S(0), S(0), S(NECK_OFFSET_Z<S>));
        compute_head(&head_transform, &world->robot);

        /* robot->center_torso = robot->transform; */
        m4_translate<S>(&world->robot.upper_torso, &world->robot.transform, S(0), S(0), NECK_OFFSET_Z<S> - S(66.7) / S(1000.0));
        m4_translate<S>(&world->robot.lower_torso, &world->robot.transform, S(0), S(0), -HIP_OFFSET_Z<S> / S(2.0));

        m4_translate<S>(&right_arm_transform, &world->robot.transform, S(0), -SHOULDER_OFFSET_Y<S>, SHOULDER_OFFSET_Z<S>);
        compute_arm(&right_arm_transform, &world->robot.right_arm, &config[R_SHOULDER_PITCH]);

        m4_translate<S>(&left_arm_transform, &world->robot.transform, S(0), SHOULDER_OFFSET_Y<S>, SHOULDER_OFFSET_Z<S>);
        compute_arm(&left_arm_transform, &world->robot.left_arm, &config[L_SHOULDER_PITCH]);

#ifdef COMPUTE_LEG
        m4_translate(&right_hip_offset, &world->robot.transform, S(0), -HIP_OFFSET_Y<S>, -HIP_OFFSET_Z<S>);
        m4_rotate(&right_leg_transform, &right_hip_offset, world->robot.hip_yaw_pitch,
                  S(0.0), S(0.70710678118655), S(0.70710678118655));
        compute_leg(&right_leg_transform, &world->robot.right_leg);

        m4_translate(&left_hip_offset, &world->robot.transform, S(0), HIP_OFFSET_Y<S>, -HIP_OFFSET_Z<S>);
        m4_rotate(&left_leg_transform, &left_hip_offset, world->robot.hip_yaw_pitch,
                  S(0.0), S(0.70710678118655), S(-0.70710678118655));
        compute_leg(&left_leg_transform, &world->robot.left_leg);
#endif

        check_collisions(world);

        m4_extract_translation(&r_pos, &world->robot.right_arm.transform_to_hand);
        m4_extract_translation(&l_pos, &world->robot.left_arm.transform_to_hand);
        m4_transform_i<S>(&cup_bowl_center, &world->robot.right_arm.transform_to_hand,
                          0, 0, S(CUP_BOWL_HEIGHT<S> + CUP_GRIP_HEIGHT<S>/S(2.0)));

        S hand_dist = (l_pos.template head<2>(), cup_bowl_center.template head<2>()).squaredNorm();
        bool xy_align = hand_dist < (CUP_DIAMETER<S> / S(2.0) - BALL_RADIUS<S>);

        bool left_above_right = cup_bowl_center.z() < l_pos.z();

        bool hands_align = xy_align && left_above_right;

        v3_sub(&cup_up, &cup_bowl_center, &r_pos);
        v3_norm(&cup_up_norm, &cup_up);

        cup_is_up = cup_up_norm.z() > RHAND_UP_GOAL<S>;

        m4_transform_i<S>(&left_hand_ball_down, &world->robot.left_arm.transform_to_hand, S(0.0), S(-1.0), S(0.0));
        v3_sub(&ball_down, &left_hand_ball_down, &l_pos);
        v3_norm(&ball_down_norm, &ball_down);

        bool ball_is_down = ball_down_norm.z() > LHAND_UP_GOAL<S>;

        world->in_goal = hands_align && cup_is_up && ball_is_down;

        world->dist_to_goal = hand_dist
            + fmax(S(0.0), l_pos.z() - cup_bowl_center.z())
            + (1 - cup_up_norm.z())
            + (1 - ball_down_norm.z());

        world->is_clear = cup_is_up && !world->in_collision;
    }

#if 0
    template <typename S>
    void set_config(nao_world<S> *world, const S *config)
    {
        hrtimer_t start_time = hrtimer_get();
        hrtimer_t set_config_time;
        hrtimer_t compute_time;

        check_angle(R_SHOULDER_PITCH, config[0]);
        world->robot.right_arm.shoulder_pitch = config[0];

        check_angle(R_SHOULDER_ROLL, config[1]);
        world->robot.right_arm.shoulder_roll = config[1];

        check_angle(R_ELBOW_YAW, config[2]);
        world->robot.right_arm.elbow_yaw = config[2];

        check_angle(R_ELBOW_ROLL, config[3]);
        world->robot.right_arm.elbow_roll = config[3];

        check_angle(R_WRIST_YAW, config[4]);
        world->robot.right_arm.wrist_yaw = config[4];

        check_angle(L_SHOULDER_PITCH, config[5]);
        world->robot.left_arm.shoulder_pitch = config[5];

        check_angle(L_SHOULDER_ROLL, config[6]);
        world->robot.left_arm.shoulder_roll = config[6];

        check_angle(L_ELBOW_YAW, config[7]);
        world->robot.left_arm.elbow_yaw = config[7];

        check_angle(L_ELBOW_ROLL, config[8]);
        world->robot.left_arm.elbow_roll = config[8];

        check_angle(L_WRIST_YAW, config[9]);
        world->robot.left_arm.wrist_yaw = config[9];

        /* set_right_shoulder_pitch(nao, config[0]); */
        /* set_right_shoulder_roll(nao, config[1]); */
        /* set_right_elbow_yaw(nao, config[2]); */
        /* set_right_elbow_roll(nao, config[3]); */
        /* set_right_wrist_yaw(nao, config[4]); */

        /* set_left_shoulder_pitch(nao, config[5]); */
        /* set_left_shoulder_roll(nao, config[6]); */
        /* set_left_elbow_yaw(nao, config[7]); */
        /* set_left_elbow_roll(nao, config[8]); */
        /* set_left_wrist_yaw(nao, config[9]); */

        // set_config_time = hrtimer_get();

        compute(world);

        // compute_time = hrtimer_get();

        compute_time -= set_config_time;
        set_config_time -= start_time;

        if (compute_time < set_config_time) {
                printf("blargh! %lld < %lld (%lld)\n",
                       compute_time,
                       set_config_time,
                       compute_time + set_config_time);
        }
    }
#endif

    template <typename S>
    S nao_dist(const S *a, const S *b)
    {
        S sum = 0;
        S d;

        for (unsigned i=0 ; i<DIMENSIONS ; ++i) {
                d = b[i] - a[i];
                sum += d*d;
        }

        return std::sqrt(sum);
    }

    template <typename S>
    bool nao_in_goal(void *system_arg, const S *config)
    {
        nao_world<S> *world = (nao_world<S>*)system_arg;

        compute(world, config);

        return world->in_goal;
    }

    template <typename S>
    bool nao_clear(void *system_arg, const S *config)
    {
        nao_world<S> *world = (nao_world<S>*)system_arg;

        compute(world, config);

        return world->is_clear;

        /* return validate_cup_is_up(world) &&
           !validate_is_self_collision(&world->nao); */
    }

    template <typename S>
    bool nao_link_impl(nao_world<S> *world, const S *a, const S *b) {
        S d = nao_dist(a, b);
        S m[DIMENSIONS];

        if (d < DISCRETIZATION<S>) {
                return true;
        }

        for (unsigned i=0 ; i<DIMENSIONS ; ++i) {
            m[i] = (a[i] + b[i]) / S(2.0);
        }

        return nao_clear(world, m)
            && nao_link_impl(world, a, m)
            && nao_link_impl(world, m, b);
    }

    template <typename S>
    bool nao_link(void *system_arg, const S *a, const S *b)
    {
        nao_world<S> *world = (nao_world<S>*)system_arg;

        /*
         * clear(a) and clear(b) are assumed since the calling PRRT
         * calls clear on all samples before attempting to link
         * them.
         */

        return nao_link_impl(world, a, b);
    }

    template <typename S>
    void *nao_system_data_alloc(int thread_no, const S *sample_min, const S *sample_max) {
        // nao_world<S> *world = struct_alloc(nao_world<S>);
        nao_world<S> *world = new nao_world<S>();

        init_collisions(world);

        return world;
    }

    template <typename S>
    void nao_system_data_free(void *system) {
        delete static_cast<nao_world<S>*>(system);
        // free(system);
    }

    template <typename S>
    prrts_system<S> *naocup_create_system() {
        prrts_system<S> *system;

        if ((system = struct_alloc(prrts_system<S>)) == NULL)
                return NULL;

        memset(system, 0, sizeof(prrts_system<S>));

        system->dimensions = DIMENSIONS;

        system->init = nao_init_config<S>();
        system->min = nao_min_config<S>();
        system->max = nao_max_config<S>();
        system->target = nao_target_config<S>();

        system->system_data_alloc_func = nao_system_data_alloc<S>;
        system->system_data_free_func = nao_system_data_free<S>;
        system->dist_func = nao_dist;
        system->in_goal_func = nao_in_goal;
        system->clear_func = nao_clear;
        system->link_func = nao_link;

        return system;
    }

    template <typename S>
    void naocup_free_system(prrts_system<S> *system) {
        free(system);
    }

// static void
// usage(char *prog_name)
// {
//         printf("usage: %s -t thread_count -n sample_count -r\n", prog_name);
//         exit(1);
// }

// static void
// print_solution(prrts_solution_t *solution)
// {
//         unsigned i, j;
//         S sum;

//         printf("angleList = [\\\n");
//         for (i=0 ; i<DIMENSIONS ; ++i) {
//                 printf("  [ ");
//                 for (j=1 ; j<solution->path_length ; ++j) {
//                         printf("%f, ", solution->configs[j][i]);
//                 }
//                 printf("], \\\n");
//         }
//         printf("]\n");
//         printf("times = [\\\n");
//         for (i=0 ; i<DIMENSIONS ; ++i) {
//                 sum = S(0.0);
//                 printf("  [ ");
//                 for (j=1 ; j<solution->path_length ; ++j) {
//                         sum += nao_dist(solution->configs[j-1], solution->configs[j]);
//                         printf("%f, ", sum);
//                 }
//                 printf("], \\\n");
//         }
//         printf("]\n");
// }


} // namespace nao_cup
#endif
