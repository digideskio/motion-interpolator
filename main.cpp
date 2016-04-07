/** @file
    @brief Implementation

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

// Internal Includes
// - none

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <osvr/Util/TimeValue.h>

// Standard includes
#include <array>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <ratio>
#include <sstream>
#include <string>
#include <vector>

static const char COMMA_CHAR = ',';

static const char DOUBLEQUOTE_CHAR = '"';

using osvr::util::time::TimeValue;

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
            // if we can't advance past the comma, it's as though we couldn't
            // find the field.
            pos = std::string::npos;
        }
    }
    return pos;
}
}
void usage() {
    std::cerr << "Must pass the CSV file containing the tracker reports, then "
                 "the CSV file containing other data that you'd like to "
                 "interpolate the tracker based on."
              << std::endl;
    std::cerr << "Press enter to exit..." << std::endl;
}

int errorExitAfterUsagePrint() {
    std::cerr << "\n";
    usage();
    return -1;
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

static const auto NUM_TIMESTAMP_FIELDS = 2;
static const std::array<std::string, NUM_TIMESTAMP_FIELDS> TIMESTAMP_HEADERS = {
    "sec", "usec"};
static const std::vector<std::string> TRACKER_HEADERS = {TIMESTAMP_HEADERS[0],
                                                         TIMESTAMP_HEADERS[1],
                                                         "x",
                                                         "y",
                                                         "z",
                                                         "qw",
                                                         "qx",
                                                         "qy",
                                                         "qz"};
namespace {
using MicrosecIntType = std::int32_t;
inline MicrosecIntType microsecondsDifference(TimeValue const &a,
                                              TimeValue const &b) {
    return static_cast<MicrosecIntType>(a.seconds - b.seconds) *
               std::micro::den +
           (a.microseconds - b.microseconds);
}

inline std::ostream &operator<<(std::ostream &os, TimeValue const &tv) {
    os << tv.seconds << ":" << tv.microseconds;
    return os;
}

enum class Status {
    BeforeRecordedTrackerData,
    Successful,
    OutOfData,
    OtherUnexpectedFailure
};
class MotionSynthesizer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MotionSynthesizer(std::istream &trackerData) : trackerData_(trackerData) {
        if (!readTrackerPose(start_, startXlate_, startRot_)) {
            throw std::runtime_error("Could not read the initial data row "
                                     "from the tracker data!");
        }
        if (!readTrackerPose(end_, endXlate_, endRot_)) {
            throw std::runtime_error("Could not read the second data row "
                                     "from the tracker data!");
        }
        updateCachedIntervalData();
    }
    bool outOfData() const { return done_; }

    /// Feed me with SEQUENTIAL TimeValue structs and I'll give you interpolated
    /// data for them, modulo some caveats.
    Status operator()(TimeValue const &tv, Eigen::Vector3d &outXlate,
                      Eigen::Quaterniond &outRot) {
        if (isBeforeTrackerData(tv)) {
            return Status::BeforeRecordedTrackerData;
        }
        /// Might need to be advanced several times...
        while (trackerDataNeedsAdvancing(tv)) {
            std::cerr << "Advanced the tracker data!" << std::endl;
            if (!advanceTrackerData()) {
                return Status::OutOfData;
            }
        }
        if (outOfData()) {
            return Status::OutOfData;
        }
        auto result = getInterpolation(tv, outXlate, outRot);
        if (!result) {
            return Status::OtherUnexpectedFailure;
        }
        return Status::Successful;
    }

    TimeValue const &getStartTime() const { return start_; };
    TimeValue const &getEndTime() const { return end_; };

  private:
    bool isBeforeTrackerData(TimeValue const &tv) const { return tv < start_; }
    bool trackerDataNeedsAdvancing(TimeValue const &tv) const {
        return end_ < tv;
    }
    void updateCachedIntervalData() {
        intervalDuration_ = microsecondsDifference(end_, start_);
        incXlate_ = endXlate_ - startXlate_;
    }
    bool getInterpolation(TimeValue const &tv, Eigen::Vector3d &outXlate,
                          Eigen::Quaterniond &outRot) const {
        if (tv == start_) {
            /// right on the start.
            outXlate = startXlate_;
            outRot = startRot_;
            return true;
        }
        if (tv == end_) {
            /// right on the end.
            outXlate = endXlate_;
            outRot = endRot_;
            return true;
        }
        if (isBeforeTrackerData(tv) || trackerDataNeedsAdvancing(tv)) {
            /// can't interpolate here.
            return false;
        }
        auto tvSinceStart = microsecondsDifference(tv, start_);
        auto t = static_cast<double>(tvSinceStart) / intervalDuration_;

        /// Slerp the rotation
        outRot = startRot_;
        outRot.slerp(t, endRot_);

        /// Lerp the translation
        outXlate = startXlate_ + t * incXlate_;
        return true;
    }
    /// move us along another row - false if no such thing possible.
    bool advanceTrackerData() {
        start_ = end_;
        startXlate_ = endXlate_;
        startRot_ = endRot_;
        if (!readTrackerPose(end_, endXlate_, endRot_)) {
            // couldn't read another line - out of data
            done_ = true;
            return false;
        }
        updateCachedIntervalData();
        return true;
    }
    /// utility
    bool readTrackerPose(TimeValue &tv, Eigen::Vector3d &xlate,
                         Eigen::Quaterniond &rot) {
        if (!trackerData_) {
            return false;
        }
        auto line = getCleanLine(trackerData_);
        if (!trackerData_) {
            return false;
        }

        fieldsTemp_ = getFields(line, FIELDS_IN_TRACKER_DATA);
        enum {
            Sec = 0,
            Usec = 1,
            TX = 2,
            TY = 3,
            TZ = 4,
            QW = 5,
            QX = 6,
            QY = 7,
            QZ = 8
        };

        getField(Sec, tv.seconds);
        getField(Usec, tv.microseconds);
        xlate.x() = getFieldAs<double>(TX);
        xlate.y() = getFieldAs<double>(TY);
        xlate.z() = getFieldAs<double>(TZ);
        rot.x() = getFieldAs<double>(QX);
        rot.y() = getFieldAs<double>(QY);
        rot.z() = getFieldAs<double>(QZ);
        rot.w() = getFieldAs<double>(QW);
        return true;
    }

    template <typename T> inline bool getField(std::size_t field, T &output) {
        iss_.clear();
        iss_.str(fieldsTemp_[field]);
        return static_cast<bool>(iss_ >> output);
    }

    template <typename T> inline T getFieldAs(std::size_t field) {
        T ret = 0;
        iss_.clear();
        iss_.str(fieldsTemp_[field]);
        iss_ >> ret;
        return ret;
    }

    static const auto FIELDS_IN_TRACKER_DATA = 9;
    std::istream &trackerData_;

    TimeValue start_;
    Eigen::Vector3d startXlate_;
    Eigen::Quaterniond startRot_;

    TimeValue end_;
    Eigen::Vector3d endXlate_;
    Eigen::Quaterniond endRot_;

    bool done_ = false;

    /// @name Cached interval data
    /// @{
    MicrosecIntType intervalDuration_ = 0;
    Eigen::Vector3d incXlate_;
    /// @}

    std::vector<std::string> fieldsTemp_;

    std::istringstream iss_;
};
} // namespace

int main(int argc, char *argv[]) {
    if (argc < 3) {
        return errorExitAfterUsagePrint();
    }
    std::ifstream trackerData(argv[1]);
    if (!trackerData) {
        std::cerr << "Could not open tracker data file " << argv[1]
                  << std::endl;
        return errorExitAfterUsagePrint();
    }

    static const auto FIELDS_IN_TRACKER_DATA = TRACKER_HEADERS.size();

    // Verify at least the first line of the tracker file to make sure it's what
    // we expect.
    {
        auto trackerHeaders =
            getFields(getCleanLine(trackerData), FIELDS_IN_TRACKER_DATA);
        if (trackerHeaders.size() != FIELDS_IN_TRACKER_DATA) {
            std::cerr
                << "Couldn't get " << FIELDS_IN_TRACKER_DATA
                << " headings from the first line of the tracker data file."
                << std::endl;
            return errorExitAfterUsagePrint();
        }

        stripQuotes(trackerHeaders);
#if 0
        for (auto &header : trackerHeaders) {
            std::cout << "Header: " << header << std::endl;
        }
#endif
        for (std::size_t i = 0; i < FIELDS_IN_TRACKER_DATA; ++i) {
            if (trackerHeaders[i] != TRACKER_HEADERS[i]) {
                std::cerr << "Heading mismatch in tracker data file, column "
                          << i << ", expected " << TRACKER_HEADERS[i]
                          << ", found " << trackerHeaders[i] << std::endl;
                return errorExitAfterUsagePrint();
            }
        }
    }

    std::ifstream timeRefData(argv[2]);
    if (!timeRefData) {
        std::cerr << "Could not open time reference data file " << argv[2]
                  << std::endl;
        return errorExitAfterUsagePrint();
    }
    // Verify the first line of the other file to look for at least sec,usec
    // headers.
    {
        auto timestampHeaders =
            getFields(getCleanLine(timeRefData), NUM_TIMESTAMP_FIELDS);
        if (timestampHeaders.size() != NUM_TIMESTAMP_FIELDS) {
            std::cerr << "Couldn't get " << NUM_TIMESTAMP_FIELDS
                      << " headings from the first line of the time reference "
                         "data file."
                      << std::endl;
            return errorExitAfterUsagePrint();
        }

        stripQuotes(timestampHeaders);

        for (auto &header : timestampHeaders) {
            std::cout << "Header: " << header << std::endl;
        }

        for (std::size_t i = 0; i < NUM_TIMESTAMP_FIELDS; ++i) {
            if (timestampHeaders[i] != TIMESTAMP_HEADERS[i]) {
                std::cerr << "Heading mismatch in tracker data file, column "
                          << i << ", expected " << TIMESTAMP_HEADERS[i]
                          << ", found " << timestampHeaders[i] << std::endl;
                return errorExitAfterUsagePrint();
            }
        }
    }

    try {
        MotionSynthesizer app(trackerData);
        std::istringstream iss;
        Eigen::Vector3d xlate;
        Eigen::Quaterniond rot;
        bool done = false;
        std::uint64_t rows = 0;
        do {
            auto data = getCleanLine(timeRefData);
            if (!timeRefData) {
                std::cerr << "Out of time ref data, all done." << std::endl;
                std::cerr << "Rows: " << rows << std::endl;
                break;
            }

            auto timestampFields = getFields(data, NUM_TIMESTAMP_FIELDS);
            if (timestampFields.size() != NUM_TIMESTAMP_FIELDS) {
                std::cerr << "Got only " << timestampFields.size()
                          << " fields, wanted " << NUM_TIMESTAMP_FIELDS
                          << std::endl;
                std::cerr << "Line was '" << data << "'" << std::endl;
                std::cerr << "Rows: " << rows << std::endl;
                break;
            }
            rows++;
            TimeValue tv;
            iss.clear();
            iss.str(timestampFields[0]);
            iss >> tv.seconds;
            iss.clear();
            iss.str(timestampFields[1]);
            iss >> tv.microseconds;
            switch (app(tv, xlate, rot)) {
            case Status::BeforeRecordedTrackerData:
                std::cout << tv << " not in [ " << app.getStartTime() << " , "
                          << app.getEndTime() << " ]" << std::endl;
                // std::cout << "Skip!" << std::endl;
                break;
            case Status::Successful:
                std::cout << xlate.transpose() << std::endl;
                break;
            case Status::OutOfData:
                std::cout << "Out of data from the tracker." << std::endl;
                done = true;
                break;
            default:
                std::cerr << "Bad things happened!" << std::endl;
                break;
            }
        } while (!done);
    } catch (std::exception const &e) {
        std::cerr << "Got exception: " << e.what() << std::endl;
        return -2;
    }

    return 0;
}
