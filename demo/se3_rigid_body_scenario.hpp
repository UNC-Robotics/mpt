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

#pragma once

#include <Eigen/Dense>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <functional>
#include <memory>
#include <mpt/discrete_motion_validator.hpp>
#include <mpt/goal_state.hpp>
#include <mpt/log.hpp>
#include <mpt/se3_space.hpp>
#include <mpt/uniform_sampler.hpp>
#include <nigh/kdtree_batch.hpp>

namespace mpt_demo::impl {
    static constexpr std::intmax_t SO3_WEIGHT = 50;

    // silence the warnings "taking address of packed member 'a1' of
    // class or structure 'aiMatrix4x4t<float>' may result in an
    // unaligned pointer value" We use static_asserts to ensure
    // unaligned structures will not happen.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
    template <typename Scalar>
    auto mapToEigen(const aiMatrix4x4t<Scalar>& m) {
        using EigenType = const Eigen::Matrix<Scalar, 4, 4, Eigen::RowMajor>;
        static_assert(sizeof(EigenType) == sizeof(m));
        return Eigen::Map<EigenType>(&m.a1);
    }

    template <typename Scalar>
    auto mapToEigen(const aiVector3t<Scalar>& v) {
        using EigenType = const Eigen::Matrix<Scalar, 3, 1>;
        static_assert(sizeof(EigenType) == sizeof(v));
        return Eigen::Map<const EigenType>(&v.x);
    }
#pragma GCC diagnostic pop

    template <typename Scalar, typename Fn>
    std::size_t visitVertices(
        const aiScene* scene, const aiNode *node,
        Eigen::Transform<Scalar, 3, Eigen::Affine> transform,
        Fn&& visitor)
    {
        std::size_t count = 0;
        transform *= mapToEigen(node->mTransformation).template cast<Scalar>();
        for (unsigned i=0 ; i < node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            count += mesh->mNumVertices;
            for (unsigned j=0 ; j < mesh->mNumVertices ; ++j)
                visitor(transform * mapToEigen(mesh->mVertices[j]).template cast<Scalar>());
        }
        for (unsigned i=0 ; i < node->mNumChildren ; ++i)
            count += visitVertices(scene, node->mChildren[i], transform, std::forward<Fn>(visitor));
        return count;
    }

    template <typename Scalar, typename Fn>
    static std::size_t visitTriangles(
        const aiScene *scene, const aiNode *node,
        Eigen::Transform<Scalar, 3, Eigen::Affine> transform,
        Fn&& visitor)
    {
        std::size_t count = 0;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

        transform *= mapToEigen(node->mTransformation).template cast<Scalar>();
        for (unsigned i=0 ; i<node->mNumMeshes ; ++i) {
            const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned j=0 ; j<mesh->mNumFaces ; ++j) {
                const aiFace& face = mesh->mFaces[j];
                if (face.mNumIndices < 3)
                    continue;

                // Support trangular decomposition by fanning out
                // around vertex 0.  The indexing follows as:
                //
                //   0---1   0 1 2
                //  /|\ /    0 2 3
                // 4-3-2     0 3 4
                //
                Vec3 v0 = transform * mapToEigen(mesh->mVertices[face.mIndices[0]]).template cast<Scalar>();
                Vec3 v1 = transform * mapToEigen(mesh->mVertices[face.mIndices[1]]).template cast<Scalar>();
                for (unsigned k=2 ; k<face.mNumIndices ; ++k) {
                    Vec3 v2 = transform * mapToEigen(mesh->mVertices[face.mIndices[k]]).template cast<Scalar>();
                    visitor(v0, v1, v2);
                    v1 = v2;
                }
                count += face.mNumIndices - 2;
            }
        }
        for (unsigned i=0 ; i<node->mNumChildren ; ++i)
            count += visitTriangles(scene, node->mChildren[i], transform, std::forward<Fn>(visitor));

        return count;
    }

    // static auto sceneBounds(const aiScene* scene, double scale = 0, double inc = 0) {
    //     Eigen::Matrix<double, 3, 2> bounds;
    //     bounds.col(0).fill(std::numeric_limits<double>::infinity());
    //     bounds.col(1).fill(-std::numeric_limits<double>::infinity());
    //     visitVertices(scene, [&] (const Vec3& v) {
    //         // TODO: convert v to eigen
    //         bounds.col(0) = bounds.col(0).cwiseMin(v);
    //         bounds.col(1) = boudns.col(1).cwiseMax(v);
    //     });

    //     scale -= 1;
    //     assert(scale >= 0);
    //     assert(inc >= 0);
    //     Eigen::Matrix<double, 3, 1> growth = (bounds.col(1) - bounds.col(0)) * scale + inc;
    //     bounds.col(0) = bounds.col(0) - growth;
    //     bounds.col(1) = bounds.col(1) + growth;
    //     return bounds;
    // }

    template <typename Scalar>
    class Mesh {
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using Transform = Eigen::Transform<Scalar, 3, Eigen::Affine>;

        std::string name_;
        fcl::BVHModel<fcl::OBBRSS<Scalar>> model_;

    public:
        Mesh(const std::string& name, bool shiftToCenter)
            : name_(name)
        {
            Assimp::Importer importer;
            // these options are the same as from OMPL app to strive
            // for parity
            static constexpr auto readOpts =
                aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes;

            const aiScene *scene = importer.ReadFile(name, readOpts);
            if (scene == nullptr)
                throw std::invalid_argument("could not load mesh file '" + name + "'");

            if (!scene->HasMeshes())
                throw std::invalid_argument("mesh file '" + name + "' does not contain meshes");

            Vec3 center = Vec3::Zero();
            std::size_t nVertices = visitVertices(
                scene,
                scene->mRootNode,
                Transform::Identity(),
                [&] (const Vec3& v) { center += v; });
            center /= nVertices;
            // TODO: scene::inferBounds(bounds, vertices, factor_, add_);

            Transform rootTransform = Transform::Identity();

            if (shiftToCenter)
                rootTransform *= Eigen::Translation<Scalar, 3>(-center);

            model_.beginModel();
            std::size_t nTris = visitTriangles(
                scene,
                scene->mRootNode,
                rootTransform,
                [&] (const Vec3& a, const Vec3& b, const Vec3& c) {
                    model_.addTriangle(a, b, c);
                });
            model_.endModel();
            model_.computeLocalAABB();

            MPT_LOG(INFO) << "Loaded mesh '" << name << "' (" << nVertices << " vertices, " << nTris
                          << " triangles, center=" << center << ")";
        }

        const auto& minBounds() const {
            return model_.aabb_local.min_;
        }

        const auto& maxBounds() const {
            return model_.aabb_local.max_;
        }

        auto extents() const {
            return (maxBounds() - minBounds()).norm() + Scalar(impl::SO3_WEIGHT*M_PI/2);
        }

        const fcl::CollisionGeometry<Scalar>* geom() const { return &model_; }
    };

    template <auto>
    class member_function;

    template <typename T, typename R, R T::* fn>
    class member_function<fn> {
        T& obj_;
    public:
        member_function(T& obj) : obj_(obj) {}
        template <typename ... Args>
        decltype(auto) operator() (Args&& ... args) { return (obj_.*fn)(std::forward<Args>(args)...); }
        template <typename ... Args>
        decltype(auto) operator() (Args&& ... args) const { return (obj_.*fn)(std::forward<Args>(args)...); }
    };
}

namespace mpt_demo {
    using namespace unc::robotics;

    template <typename Scalar, int nParts = 1, bool selfCollision = false>
    class SE3RigidBodyScenario {

        static_assert(nParts == 1, "only single body motions are supported currently (TODO: add multibody)");

    public:

        using Space = mpt::SE3Space<Scalar, impl::SO3_WEIGHT>; // weight SO(3) by 50
        using Bounds = std::tuple<mpt::Unbounded, mpt::BoxBounds<Scalar, 3>>;
        using State = typename Space::Type;
        using Distance = typename Space::Distance;
        using Goal = mpt::GoalState<Space>;
        using TravelTime = Scalar;

        // TODO: remove explicit Nearest and use default
        using Nearest = unc::robotics::nigh::KDTreeBatch<8>;

    private:
        using Config = typename Space::Type;
        // fcl::Transform3<Scalar> is an alias for
        // Eigen::Transform<Scalar, 3, Eigen::Isometry> though in a
        // previous version the last template parameter was
        // Eigen::AffineCompact.  Isometry seems like a better option,
        // so it seems unlikely to change, but regardless, we use
        // fcl's alias for it instead of directly using Eigen's type.
        using Transform = fcl::Transform3<Scalar>;

        // The meshes are immutable and can be rather large.  Since
        // scenarios are copied to each thread, we use a shared
        // pointer to avoid copying the environment and robot meshes.
        std::shared_ptr<impl::Mesh<Scalar>> environment_;
        std::shared_ptr<std::vector<impl::Mesh<Scalar>>> robot_;

        Space space_;
        Bounds bounds_;

        static constexpr Distance goalRadius = 1e-6;
        Goal goal_;

        static Transform stateToTransform(const Config& q) {
            return Eigen::Translation<Scalar, 3>(std::get<1>(q))
                * Eigen::Quaternion(std::get<0>(q));
        }

    public:
        bool valid(const Config& q) const {
            fcl::CollisionRequest<Scalar> req;
            fcl::CollisionResult<Scalar> res;

            for (const auto& robot : *robot_) {
                // for (std::size_t i=0 ; i<robot_->size() ; ++i) {
                Transform tf = stateToTransform(q); // TODO: get the ith state
                if (fcl::collide(robot.geom(), tf, environment_->geom(), Transform::Identity(), req, res))
                    return false;
            }

            // TODO: when multibody is supported
            // if (selfCollision) {
            //     for (std::size_t i=0 ; i<robot_.size() ; ++i) {
            //         Transform tfi = stateToTransform(q);
            //         for (std::size_t j=i ; ++j<robot_.size() ; ) {
            //             Transform tfj = stateToTransform(q);
            //             if (fcl::collide(robot_[i], tfi, robot_[j], tfj, req, res) > 0)
            //                 return false;
            //         }
            //     }
            // }

            return true;
        }

    private:
        using Validator = impl::member_function<&SE3RigidBodyScenario::valid>;

    public:
        const mpt::DiscreteMotionValidator<Space, Validator> link_;

        template <typename Min, typename Max>
        SE3RigidBodyScenario(
            const std::string& envMesh,
            const std::vector<std::string>& robotMeshes,
            const Config& goal,
            const Eigen::MatrixBase<Min>& min,
            const Eigen::MatrixBase<Max>& max,
            Scalar checkResolution)
            : environment_(std::make_shared<impl::Mesh<Scalar>>(envMesh, false))
            , robot_(std::make_shared<std::vector<impl::Mesh<Scalar>>>())
            , bounds_(mpt::Unbounded{}, mpt::BoxBounds<Scalar, 3>(min, max)) // environment_.minBounds(), environment_.maxBounds())),
            , goal_(goalRadius, goal)
            // , link_(space_, environment_->extents()*checkResolution, Validator(*this))
            , link_(space_, ((max - min).norm() + Scalar(impl::SO3_WEIGHT*M_PI/2))*checkResolution, Validator(*this))
        {
            robot_->reserve(robotMeshes.size());
            for (const std::string& mesh : robotMeshes)
                robot_->emplace_back(mesh, true);

            MPT_LOG(DEBUG) << "Volume min: " << min.transpose();
            MPT_LOG(DEBUG) << "Volume max: " << max.transpose();
        }

        const Space& space() const {
            return space_;
        }

        const Bounds& bounds() const {
            return bounds_;
        }

        // template <typename RNG>
        // std::optional<State> sample(RNG& rng) {
        //     return {};
        // }

        const Goal& goal() const {
            return goal_;
        }

        bool link(const Config& a, const Config& b) const {
            return link_(a, b);
        }

        // TODO: this shouldn't be necessary
        TravelTime travelTime(Distance dist, bool) const { return dist; }
    };
}

