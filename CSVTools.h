/** @file
    @brief Header

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDED_CSVTools_h_GUID_82FA298C_196A_46AA_B2D6_059F2A035687
#define INCLUDED_CSVTools_h_GUID_82FA298C_196A_46AA_B2D6_059F2A035687

// Internal Includes
// - none

// Library/third-party includes
// - none

// Standard includes
#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

namespace csvtools {

static const char COMMA_CHAR = ',';

static const char DOUBLEQUOTE_CHAR = '"';

inline std::string getCleanLine(std::istream &is) {
    std::string ret;
    std::getline(is, ret);
    while (!ret.empty() && (ret.back() == '\n' || ret.back() == '\r')) {
        ret.pop_back();
    }
    return ret;
}

namespace string_fields {

    inline std::size_t getBeginningOfField(std::string const &line,
                                           std::size_t field) {
        if (0 == field) {
            return 0;
        }
        std::size_t pos = 0;
        for (std::size_t i = 0; i < field && pos != std::string::npos; ++i) {
            pos = line.find(COMMA_CHAR, pos + 1);
        }
        if (pos != std::string::npos) {
            if (pos + 1 < line.size()) {
                // must advance past the comma.
                pos++;
            } else {
                // if we can't advance past the comma, it's as though we
                // couldn't
                // find the field.
                pos = std::string::npos;
            }
        }
        return pos;
    }
}

std::vector<std::string> getFields(std::string const &line,
                                   std::size_t numFields,
                                   std::size_t first = 0) {
    std::vector<std::string> ret;
    /// "begin" iterator/position
    std::size_t b = string_fields::getBeginningOfField(line, first);
    /// initial "one past the end" iterator/position
    auto e = b;
    std::size_t len = 0;
    bool done = false;
    const auto n = line.size();
    /// the condition on b < n is because we update b = e + 1, and e might be
    /// the last character in the string.
    for (std::size_t i = 0; i < numFields && !done && b < n; ++i) {
        e = line.find(COMMA_CHAR, b);
        if (e == std::string::npos) {
            // indicate to substring we want the rest of the line.
            len = std::string::npos;
            // quit after this field
            done = true;
        } else {
            len = e - b;
        }

        ret.emplace_back(line.substr(b, len));
        if (!done) {
            b = e + 1;
        }
    }
    return ret;
}

inline void stripQuotes(std::string &field) {
    if (field.size() > 1 && field.front() == DOUBLEQUOTE_CHAR &&
        field.back() == DOUBLEQUOTE_CHAR) {
        /// pop back first, so we have less to "slide up"
        field.pop_back();
        /// then remove the first character
        field.erase(0, 1);
    }
}

inline void stripQuotes(std::vector<std::string> &fields) {
    for (auto &field : fields) {
        stripQuotes(field);
    }
}
} // csvtools

#endif // INCLUDED_CSVTools_h_GUID_82FA298C_196A_46AA_B2D6_059F2A035687
