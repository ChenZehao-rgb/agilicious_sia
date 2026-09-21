#include "agilib/reference/trajectory_csv.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace agi::trajectory_csv {
namespace {

constexpr char kShortHeader[] = "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z";
constexpr char kFullHeader[] =
        "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,"
        "a_lin_x,a_lin_y,a_lin_z,a_rot_x,a_rot_y,a_rot_z,u_1,u_2,u_3,u_4,jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z";

class TemporaryCsv {
public:
	explicit TemporaryCsv(const std::string& content) {
		std::string pattern = (std::filesystem::temp_directory_path() / "agi_trajectory_header_XXXXXX").string();
		std::vector<char> buffer(pattern.begin(), pattern.end());
		buffer.push_back('\0');
		const int descriptor = mkstemp(buffer.data());
		if (descriptor < 0) throw std::runtime_error("Could not create trajectory test file");
		close(descriptor);
		_path = buffer.data();
		std::ofstream stream(_path);
		stream << content;
		if (!stream) throw std::runtime_error("Could not write trajectory test file");
	}
	~TemporaryCsv() {
		std::error_code error;
		std::filesystem::remove(_path, error);
	}
	const std::filesystem::path& path() const { return _path; }

private:
	std::filesystem::path _path;
};

std::string numericRow(double time, int columns = 30) {
	std::ostringstream row;
	row << time;
	for (int i = 1; i < columns; ++i) row << ',' << i;
	return row.str();
}

TEST(TrajectoryCsv, AcceptsShortHeaderWithLfOrCrLf) {
	for (const auto* header : {kShortHeader}) {
		for (const auto* newline : {"\n", "\r\n"}) {
			const TemporaryCsv csv(std::string(header) + newline + numericRow(0.) + newline + numericRow(.01) + newline);
			const auto rows = readTrajectoryRows(csv.path());
			ASSERT_EQ(rows.size(), 2u);
			ASSERT_EQ(rows.front().size(), 30u);
			EXPECT_DOUBLE_EQ(rows.front()[14], 14.);
			EXPECT_DOUBLE_EQ(rows.back()[29], 29.);
		}
	}
}

TEST(TrajectoryCsv, RejectsReorderedUnknownAndIncompleteHeaders) {
	std::string reordered = kShortHeader;
	reordered.replace(reordered.find("p_x,p_y"), 7, "p_y,p_x");
	for (const auto& header :
	     {reordered, std::string(kFullHeader), std::string(kShortHeader) + ",extra", std::string("time,p_x,p_y")}) {
		const TemporaryCsv csv(header + '\n' + numericRow(0.) + '\n' + numericRow(.01) + '\n');
		EXPECT_THROW(readTrajectoryRows(csv.path()), std::runtime_error);
	}
}

TEST(TrajectoryCsv, LegacyShortHeaderStillRequiresThirtyNumericColumns) {
	const TemporaryCsv csv(std::string(kShortHeader) + '\n' + numericRow(0., 14) + '\n' + numericRow(.01, 14) + '\n');
	EXPECT_THROW(readTrajectoryRows(csv.path()), std::runtime_error);
}

TEST(TrajectoryCsv, ReadsAllThreeRequestedDatasetsWithUnifiedHeader) {
	const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
	const auto directory = root / "miscellaneous/datasets/ref_trajs/open_source";
	for (const auto* filename : {"aggressive_50mps.csv", "HELIX_FWD20_50mps.csv", "CPC33_Z1.csv"}) {
		const auto rows = readTrajectoryRows(directory / filename);
		ASSERT_GE(rows.size(), 2u) << filename;
		EXPECT_EQ(rows.front().size(), 30u) << filename;
		EXPECT_GT(rows.back()[0], rows.front()[0]) << filename;
	}
}

}  // namespace
}  // namespace agi::trajectory_csv
