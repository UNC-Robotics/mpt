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
#ifndef MPT_IMPL_DJIKSTRAS_HPP
#define MPT_IMPL_DJIKSTRAS_HPP

#include <unordered_map>
#include <vector>

namespace unc::robotics::mpt::impl {
    
    template <typename T, typename PathCost>
    struct DjikstrasNode {
        using Item = std::pair<const T, DjikstrasNode>;
        
        Item *next_;
        PathCost pathCost_;
        std::size_t heapIndex_;
    };

    template <typename T, typename PathCost>
    class DjikstrasPathIterator {
        using Node = DjikstrasNode<T, PathCost>;
        using Item = typename Node::Item;

        Item* item_;

    public:
        DjikstrasPathIterator(Item *item = nullptr)
            : item_{item}
        {
        }
        
        bool operator != (DjikstrasPathIterator other) const {
            return item_ != other.item_;
        }
        
        bool operator == (DjikstrasPathIterator other) const {
            return item_ == other.item_;
        }

        const T& operator * () {
            return item_->first;
        }

        const T* operator -> () {
            return &item_->first;
        }

        DjikstrasPathIterator& operator ++ () {
            item_ = item_->second.next_;
            return *this;
        }

        DjikstrasPathIterator operator ++ (int) {
            DjikstrasPathIterator prev = *this;
            item_ = item_->second.next_;
            return prev;
        }
    };

    template <typename T, typename PathCost, typename Goal, typename Edges>
    class Djikstras {
        using Index = std::size_t;
        using Node = DjikstrasNode<T, PathCost>;
        using Item = typename Node::Item;
        using PathIter = DjikstrasPathIterator<T, PathCost>;
        using Map = std::unordered_map<T, Node>;

        Map map_;
        std::vector<Item*> heap_;

        Goal goal_;
        Edges edges_;
        
    public:
        Djikstras(const Goal& goal = Goal(), const Edges& edges = Edges())
            : goal_(goal)
            , edges_(edges)
        {
            // index 0 is reserved to mean nullptr
            heap_.push_back(nullptr);
        }

        void reset() {
            map_.clear();
            heap_.resize(1);
        }

        void addStart(const T& t, PathCost pathCost = 0) {
            Index c = heap_.size();
            auto [ it, inserted ] = map_.emplace(t, Node());
            if (inserted) {
                heap_.push_back(nullptr);
                for (Index p ; c != 1 ; c = p) {
                    if (heap_[p = c / 2]->second.pathCost_ <= pathCost)
                        break;
                    (heap_[c] = heap_[p])->second.heapIndex_ = c;
                }
                it->second.next_ = nullptr;
                it->second.pathCost_ = pathCost;
                (heap_[c] = &*it)->second.heapIndex_ = c;
            }
        }

        bool solved() const {
            return heap_.size() > 1 && goal_(heap_[1]->first);
        }

        bool operator() () {
            while (heap_.size() > 1) {
                Item *min = heap_[1];

                if (goal_(min->first))
                    return true;

                // pop from the heap.  keep in mind that index 0 is reserved.
                heap_[1] = heap_.back();
                heap_.pop_back();
                for (Index p = 1, c ; (c = p*2) < heap_.size() ; p = c) {
                    PathCost pc = heap_[c]->second.pathCost_;
                    if (c+1 < heap_.size() && heap_[c+1]->second.pathCost_ < pc)
                        pc = heap_[++c]->second.pathCost_;
                    if (heap_[p]->second.pathCost_ < pc)
                        break;
                    std::swap(heap_[p], heap_[c]);
                    heap_[p]->second.heapIndex_ = p;
                    heap_[c]->second.heapIndex_ = c;
                }

                edges_(min->first, [&, minCost = min->second.pathCost_] (PathCost w, T to) {
                    PathCost pathCost = minCost + w;
                    auto it = map_.find(to);
                    Index c;
                    if (it == map_.end()) {
                        // std::cout << "  PUSH " << to->id_ << std::endl;
                        c = heap_.size();
                        it = map_.emplace(to, Node()).first;
                        heap_.push_back(nullptr);
                    } else if (pathCost < it->second.pathCost_) {
                        // std::cout << " REDUCE " << to->id_ << std::endl;
                        c = it->second.heapIndex_;
                    } else {
                        // std::cout << "  SKIP " << to->id_ << std::endl;
                        return;
                    }
                    it->second.next_ = min;
                    it->second.pathCost_ = pathCost;

                    // std::cout << "  c = " << c << std::endl;

                    // move the inserted/updated value up in the queue
                    for (Index p ; c != 1 ; c = p) {
                        if (heap_[p = c / 2]->second.pathCost_ <= pathCost)
                            break;
                        // std::cout << "  p = " << p << std::endl;
                        (heap_[c] = heap_[p])->second.heapIndex_ = c;
                    }
                    (heap_[c] = &*it)->second.heapIndex_ = c;

                    // for (Index i=1 ; i<heap.size() ; ++i) {
                    //     std::cout << "[" << i << "] = " << heap[i]->first->id_ << " " << heap[i]->second.pathCost_ << std::endl;
                    // }
                });
            }
            
            return false;
        }

        template <typename Result>
        decltype(auto) solution(Result result) const {
            if (heap_.size() < 2)
                return result(std::size_t(0), PathIter(), PathIter());
            
            // reverse the pointers in the linked list so that the
            // result goes from start to goal.
            Item* prev = nullptr;
            Item* curr = heap_[1];
            std::size_t n = 1;
            for ( ;; ++n) {
                Item *next = curr->second.next_;
                curr->second.next_ = prev;
                if (next == nullptr)
                    return result(n, PathIter(curr), PathIter());
                prev = curr;
                curr = next;
            }
        }

        template <typename Result>
        decltype(auto) operator() (Result result) {
            if ((*this)()) {
                return solution(result);
            } else {
                return result(std::size_t(0), PathIter(), PathIter());
            }
        }
    };

    template <
        typename T, typename PathCost,
        typename StartIter,
        typename Goal, typename Edges, typename Result>
    decltype(auto) djikstras(
        StartIter first, StartIter last,
        Goal goal, Edges edges, Result result)
    {
        Djikstras<T, PathCost, Goal, Edges> shortestPath{goal, edges};
        for (StartIter it = first ; it != last ; ++it)
            shortestPath.addStart(*it);

        return shortestPath(result);
    }

    template <
        typename T, typename PathCost,
        typename Goal, typename Edges, typename Result>
    decltype(auto) djikstras(
        const T& start, Goal goal, Edges edges, Result result)
    {
        Djikstras<T, PathCost, Goal, Edges> shortestPath{goal, edges};
        shortestPath.addStart(start);
        return shortestPath(result);
    }
}

#endif
