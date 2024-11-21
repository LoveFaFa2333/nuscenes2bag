#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/NuScenes2Bag.hpp"
#include <boost/program_options.hpp>
#include <iostream>
#include <algorithm> 

using namespace boost::program_options;
using namespace nuscenes2bag;

int
main(const int argc, const char* argv[])
{
  try {
    std::string dataroot;
    std::string scenes_list;
    std::string version = "v1.0-mini";
    std::string outputBagName;
    int32_t threadNumber = -1;
    int32_t sceneNumber = -1;
    std::vector<uint32_t> sceneNumberVec;
    bool en_lidar = true;
    bool en_radar = false;
    bool en_cam = false;

    options_description desc{ "Options" };
    desc.add_options()("help,h", "show help");

    options_description inputDesc{ "input" };
    inputDesc.add_options()
    ("scenes_list,sl", value<std::string>(&scenes_list), "scenes number list to convert")
    ("lidar,l", value<bool>(&en_lidar), "enbale lidar data converting")
    ("cam,c", value<bool>(&en_cam), "enbale camera data converting")
    ("radar,r", value<bool>(&en_radar), "enbale radar data converting")
    ("scene_number,n", value<int32_t>(&sceneNumber), "only convert a given scene")
    ("dataroot,s", value<std::string>(&dataroot)->required(), "Path to root of dataset containing 'maps', 'samples', 'sweeps'")
    ("version", value<std::string>(&version), "Version string (default = 'v1.0-mini')")
    ("out,o", value<std::string>(&outputBagName), "output bag name")
    ("jobs,j", value<int32_t>(&threadNumber), "number of jobs (thread number)");

    variables_map vm;

    desc.add(inputDesc);
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
    } else {
      // open scenes number txt
      std::ifstream fin(scenes_list);
      if(!fin.is_open()) {
        std::cout << "Cannot open scenes number .txt file from " << scenes_list 
                  << ", Failed to conduct batch bags processing.";
      }
      std::string line;
      while (std::getline(fin, line)) {
        std::istringstream iss(line); // 创建字符串流以解析每一行
        int number;
        iss >> number;
        sceneNumberVec.push_back(number);
      }
      fin.close();

      fs::path sampleDirPath(dataroot);

      if (sceneNumber > 0) {
        if(std::find(sceneNumberVec.begin(), sceneNumberVec.end(), sceneNumber) == sceneNumberVec.end()) {
          sceneNumberVec.push_back(sceneNumber);
        }
      }

      std::vector<std::vector<uint32_t>> splitResult;
      for (size_t i = 0; i < sceneNumberVec.size(); i += 20) {
          size_t end = (i + 20 < sceneNumberVec.size()) ? (i + 20) : sceneNumberVec.size();
          std::vector<uint32_t> chunk(sceneNumberVec.begin() + i, sceneNumberVec.begin() + end);
          splitResult.push_back(chunk);
      }
      
      for(auto subSceneNumberVec : splitResult) {
        NuScenes2Bag converter{};
        converter.convertDirectory(sampleDirPath, version, outputBagName, threadNumber, subSceneNumberVec, en_lidar, en_cam, en_radar);
      }

    }
  } catch (const error& ex) {
    std::cerr << ex.what() << '\n';
  }

  return 0;
}