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

//! @author William Lee

#ifndef SHAPE_HIERARCHY
#define SHAPE_HIERARCHY

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>

namespace shape
{
    class Color
    {
    public:
        Color(const int r, const int g, const int b)
            : r(r), g(g), b(b) {}
        Color()
            : Color(255, 255, 255) {}
    private:
        const int r;
        const int g;
        const int b;
        friend std::ostream &operator<<(std::ostream &, const Color &);
    };

    inline std::string startTag(std::string tag)
    {
        return "<" + tag;
    }

    inline std::string closeTag()
    {
        return "/>\n";
    }

    inline std::string closeTag(std::string tag)
    {
        return "</" + tag + ">\n";
    }

    template <typename T>
    inline std::string addAttr(std::string name, T value, std::string unit = "")
    {
        std::stringstream ss;
        ss << ' ' << name << "=" <<  "\"" << value << unit << "\"";
        return ss.str();
    }

    inline void startSvg(std::ofstream &file, const int width, const int height)
    {
        file << "<?xml version=\"1.0\" standalone=\"no\" ?>" << std::endl
             << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" << std::endl
             << startTag("svg")
             << addAttr("width", width, "px")
             << addAttr("height", height, "px")
             << addAttr("xmlns", "http://www.w3.org/2000/svg")
             << addAttr("xmlns:xlink", "http://www.w3.org/1999/xlink")
             << addAttr("version", "1.1")
             << ">\n";
    }

    inline void addImage(std::ofstream &file, const std::string &path)
    {
        file << '\t' << startTag("image") << addAttr("xlink:href", path) << closeTag();
    }

    inline void endSvg(std::ofstream &file)
    {
        file << closeTag("svg");
        file.close();
    }

    inline void addEdge(std::ofstream &file, double x1, double y1, double x2, double y2, double width, Color c) {
        file << "\t"
             << startTag("line")
             << addAttr("x1", x1)
             << addAttr("y1", y1)
             << addAttr("x2", x2)
             << addAttr("y2", y2)
             << addAttr("stroke", c)
             << addAttr("stroke-width", width)
             << addAttr("stroke-linecap", "round")
             << closeTag();       
    }

    inline void addSolutionEdge(std::ofstream &file, double x1, double y1, double x2, double y2, double width = 4.0)
    {
        addEdge(file, x1, y1, x2, y2, width, Color(250, 50, 50));
    }

    inline void addVisitedEdge(std::ofstream &file, double x1, double y1, double x2, double y2, double width = 1.0)
    {
        addEdge(file, x1, y1, x2, y2, width, Color(125, 125, 125));
    }

    inline void addState(std::ofstream &file, double x, double y, double r, char c) {
        file << "\t"
             << startTag("circle")
             << addAttr("cx", x)
             << addAttr("cy", y)
             << addAttr("r", r)
             << addAttr("fill", "rgb(230, 230, 230)")
             << addAttr("stroke", "red")
             << addAttr("stroke-width", r / 5)
             << closeTag();

        // add the label
        file << startTag("text")
             << addAttr("x", x)
             << addAttr("y", y + r / 3.0)
             << addAttr("fill", "black")
             << addAttr("text-anchor", "middle")
             << addAttr("font-size", r)
             << ">"
             << ' ' << c << ' '
             << closeTag("text");
    }

    inline void addStartState(std::ofstream &file, double x, double y, double r = 20)
    {
        addState(file, x, y, r, 'S');
    }

    inline void addGoalState(std::ofstream &file, double x, double y, double r = 20)
    {
        addState(file, x, y, r, 'G');
    }

    template <typename Scalar>
    class Rect
    {
    public:
        using State = Eigen::Matrix<Scalar, 2, 1>;
        // TODO: check parameter in the constructor
        Rect(const Scalar x0, const Scalar y0, const Scalar x1, const Scalar y1, const Color color = Color(0, 0, 0))
            : p0_(State(x0, y0)), p1_(State(x1, y1)), color_(color) {}

        bool pointIsValid(const State &p) const
        {
            Scalar px = p[0];
            Scalar py = p[1];
            return !(px >= p0_[0] && px <= p1_[0] && py >= p0_[1] && py <= p1_[1]);
        }

        bool segmentIsValid(const State &a, const State &b) const
        {
            if(!pointIsValid(a) || !pointIsValid(b))
                return false;
            return bisectSegment(a, b);
        }

        bool bisectSegment(const State &a, const State &b) const
        {
            State mid = (a + b) / 2;
            Scalar distSquared = (b - a).squaredNorm();
            Scalar tolerance = 1;
            if(distSquared < tolerance * tolerance)
                return true;
            if(!pointIsValid(mid))
                return false;
            bool left = bisectSegment(a, mid);
            if(!left)
                return false;
            bool right = bisectSegment(mid, b);
            return right;
        }

    private:
        const State p0_;
        const State p1_;
        const Color color_;
        template <typename Scalar_>
        friend std::ostream &operator<<(std::ostream &, const Rect<Scalar_> &);
    };

    template <typename Scalar>
    class Circle
    {
    public:
        using State = Eigen::Matrix<Scalar, 2, 1>;

        Circle(Scalar rx, Scalar ry, const Scalar radius, const Color color = Color(0, 0, 0))
            : center_(State(rx, ry)), radius_(radius), color_(color) {}

        bool pointIsValid(const State &p) const
        {
            State dist = p - center_;
            return dist.squaredNorm() > radius_ * radius_;
        }

        bool segmentIsValid(const State &a, const State &b) const
        {
            return distPointSegmentSquared(center_, a, b) > radius_ * radius_;
        }

        // for robot with non trivial size (e.g. robot arm)
        bool segmentIsValid(const State &a, const State &b, Scalar robotRadius) const
        {
            return distPointSegmentSquared(center_, a, b) > (radius_ + robotRadius) * (radius_ + robotRadius);
        }

    private:
        const State center_;
        const Scalar radius_;
        const Color color_;

        Scalar distPointSegmentSquared (const State &pt, const State &s0, const State &s1) const
        {
            State v = s1 - s0;
            State w = pt - s0;
            Scalar c1 = v.dot(w);
            if (c1 <= 0)
                return w.squaredNorm();
            Scalar c2 = v.squaredNorm();
            if (c2 <= c1)
                return (pt - s1).squaredNorm();
            return (s0 - pt + v * (c1 / c2)).squaredNorm();
        }
        template <typename Scalar_>
        friend std::ostream &operator<<(std::ostream &, const Circle<Scalar_> &);
    };

    std::ostream &operator<<(std::ostream &strm, const Color &c)
    {
        return strm << "rgb(" << c.r << "," << c.g << "," << c.b << ")";
    }

    template <typename Scalar>
    std::ostream &operator<<(std::ostream &strm, const Circle<Scalar> &c)
    {
        return strm << '\t'
               << startTag("circle")
               << addAttr("cx", c.center_[0])
               << addAttr("cy", c.center_[1])
               << addAttr("r", c.radius_)
               << addAttr("fill", c.color_)
               << addAttr("stroke", "black")
               << addAttr("stroke-width", "1px")
               << closeTag();
    }

    template <typename Scalar>
    std::ostream &operator<<(std::ostream &strm, const Rect<Scalar> &r)
    {
        return strm << '\t'
               << startTag("rect")
               << addAttr("x", r.p0_[0])
               << addAttr("y", r.p0_[1])
               << addAttr("width", r.p1_[0] - r.p0_[0])
               << addAttr("height", r.p1_[1] - r.p0_[1])
               << addAttr("fill", r.color_)
               << addAttr("stroke", "black")
               << addAttr("stroke-width", "1px")
               << closeTag();
    }
}
#endif