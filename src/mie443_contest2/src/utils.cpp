#include <boost/filesystem/path.hpp>
#include <contest2/utils.h>
#include <unordered_map>

int findMode(const std::vector<int> &nums) {
  // Return 0 if the vector is empty (or handle error as needed)
  if (nums.empty()) {
    return 0;
  }

  // Map to store frequency of each number
  std::unordered_map<int, int> frequency;
  for (int num : nums) {
    frequency[num]++;
  }

  // Find the mode by checking which number has the highest frequency
  int mode = nums[0];
  int maxCount = 0;
  for (const auto &entry : frequency) {
    if (entry.second > maxCount) {
      maxCount = entry.second;
      mode = entry.first;
    }
  }

  return mode;
}

std::string getFileName(int boxGuess) {
  if (boxGuess == -1) {
    return "Blank";
  } else {
    boost::filesystem::path p(TEMPLATE_FILES[boxGuess]);
    return p.stem().string();
  }
}
