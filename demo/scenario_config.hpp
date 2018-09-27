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
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <regex>
#include <string>
#include <tuple>

namespace mpt_demo {

    template <
        typename Char = char,
        typename Traits = std::char_traits<Char>,
        typename Allocator = std::allocator<Char>>
    class ScenarioConfig {
        using String = std::basic_string<Char, Traits, Allocator>;
        using Regex = std::basic_regex<Char, std::regex_traits<Char>>;

        using KeyValueMap = std::map<String, String>;

        std::map<String, KeyValueMap> properties_;

        template <typename T, typename Convert> // T (*Convert)(const String&, std::size_t*)>
        void loadConvert(T& prop, const String& section, const String& name, Convert convert) const {
            String str;
            load(str, section, name);
            std::size_t pos;
            prop = convert(str, &pos);

            // check that all characters were processed
            if (pos != str.length())
                throw std::invalid_argument("invalid numeric value: " + str);
        }

    public:
        ScenarioConfig(const String& fileName) {
            std::basic_ifstream<Char, Traits> in(fileName);
            if (!in)
                throw std::system_error(errno, std::system_category(), "failed to open '" + fileName + "'");

            Regex sectionRe("\\s*\\[\\s*([^\\]]+?)\\s*\\]\\s*");
            Regex keyValueRe("\\s*([^=]*?)\\s*=\\s*(.*?)\\s*");
            std::match_results<typename String::const_iterator> m;
            String line;

            String sectionName;

            for (int lineNo=1 ; std::getline(in, line) ; ++lineNo) {
                if (std::all_of(line.begin(), line.end(), isspace)) {
                    // skip empty lines
                } else if (std::regex_match(line, m, sectionRe)) {
                    if (properties_.find(sectionName = m[1]) != properties_.end())
                        MPT_LOG(WARN) << fileName << ":" << lineNo << ": duplicate section";
                } else if (std::regex_match(line, m, keyValueRe)) {
                    if (sectionName.empty()) {
                        MPT_LOG(WARN) << fileName << ":" << lineNo << ": property without section header";
                    } else {
                        properties_[sectionName][m[1]] = m[2];
                    }
                } else {
                    MPT_LOG(WARN) << fileName << ":" << lineNo << ": invalid line";
                }
            }
        }

        bool hasProp(const String& section, const String& name) const {
            auto it = properties_.find(section);
            if (it == properties_.end())
                return false;

            return it->second.find(name) != it->second.end();
        }

        void load(String& prop, const String& section, const String& name) const {
            auto it = properties_.find(section);
            if (it == properties_.end())
                throw std::invalid_argument("missing section [" + section + "]");


            auto kv = it->second.find(name);
            if (kv == it->second.end())
                throw std::invalid_argument("missing property [" + section + "] " + name);

            prop = kv->second;
        }

        void load(float& prop, const String& section, const String& name) const {
            loadConvert(prop, section, name, [](const String& str, std::size_t* pos) { return std::stof(str, pos); });
        }

        void load(double& prop, const String& section, const String& name) const {
            loadConvert(prop, section, name, [](const String& str, std::size_t* pos) { return std::stod(str, pos); } );
        }

        // void load(long double& prop, const String& section, const String& name) const {
        //     loadConvert(prop, section, name, [](const String& str, std::size_t* pos) { return std::stold(str, pos); } );
        // }

        template <typename Scalar>
        void load(Eigen::Matrix<Scalar, 3, 1>& v, const String& section, const String& name) const {
            load(v[0], section, name + ".x");
            load(v[1], section, name + ".y");
            load(v[2], section, name + ".z");
        }

        template <typename Scalar>
        void load(Eigen::AngleAxis<Scalar>& a, const String& section, const String& name) const {
            load(a.angle(), section, name + ".theta");
            load(a.axis(), section, name + ".axis");
        }

        template <typename Scalar>
        void load(Eigen::Quaternion<Scalar>& q, const String& section, const String& name) const {
            Eigen::AngleAxis<Scalar> a;
            load(a, section, name);
            q = a;
        }

        template <typename A, typename B>
        void load(std::tuple<A, B>& q, const String& section, const String& name) const {
            load(std::get<0>(q), section, name);
            load(std::get<1>(q), section, name);
        }
    };
}
