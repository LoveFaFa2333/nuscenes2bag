#include <nuscenes2bag/MetaDataReader.hpp>
#include "nuscenes2bag/utils.hpp"

#include <regex>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <map>
#include <algorithm>

using namespace std;
namespace fs = std::filesystem;
namespace json = nlohmann;

void MetaDataReader::loadFromDirectory(const fs::path& directoryPath) {
    const fs::path sceneFile = directoryPath / "scene.json";
    const fs::path sampleFile = directoryPath / "sample.json";
    const fs::path sampleDataFile = directoryPath / "sample_data.json";

    scenes = loadScenesFromFile(sceneFile);
    scene2Samples = loadSampleInfos(sampleFile);
    sample2SampleData = loadSampleDataInfos(sampleDataFile);

    loadFromDirectoryCalled = true;
}


json::json MetaDataReader::slurpJsonFile(const std::filesystem::path& filePath) {
    std::ifstream file(filePath.string());
    if(!file.is_open()) {
        std::string errMsg = string("Unable to open ") + filePath.string();
        throw std::runtime_error(errMsg);
    }
    json::json newJson;
    file >> newJson;
    return newJson;
}

std::vector<SceneInfo> MetaDataReader::loadScenesFromFile(const fs::path& filePath) {
    auto sceneJsons = slurpJsonFile(filePath);
    std::vector<SceneInfo> sceneInfos;

    std::regex sceneIdRegex("scene-(\\d+)");

    for(const auto& sceneJson : sceneJsons) {
        std::string sceneIdStr = sceneJson["name"];
        std::smatch match;
        std::regex_search(sceneIdStr, match, sceneIdRegex);
        SceneId sceneId = std::stoi(match.str(1));
        sceneInfos.push_back(SceneInfo{
            sceneJson["token"],
            sceneJson["nbr_samples"],
            sceneId,
            sceneJson["name"],
            sceneJson["description"],
            sceneJson["first_sample_token"],
        });
    }

    return sceneInfos;
}

std::map<Token, std::vector<SampleInfo>> MetaDataReader::loadSampleInfos(const std::filesystem::path& filePath) {
    auto sampleInfos = slurpJsonFile(filePath);
    std::map<Token, std::vector<SampleInfo>> token2Samples;

    for(const auto& sampleInfo : sampleInfos) {
        Token sampleToken = sampleInfo["token"];
        Token sceneToken = sampleInfo["scene_token"];
        std::vector<SampleInfo>& samples = getExistingOrDefault(token2Samples, sceneToken);
        samples.push_back(SampleInfo{
            sampleToken, 
            sampleInfo["timestamp"]
        });
    }

    return token2Samples;
}

std::map<Token, std::vector<SampleDataInfo>> MetaDataReader::loadSampleDataInfos(const std::filesystem::path& filePath) {
    auto sampleDataJsons = slurpJsonFile(filePath);
    std::map<Token, std::vector<SampleDataInfo>> sample2SampleData;

    for(const auto& sampleDataJson : sampleDataJsons) {
        Token sampleToken  = sampleDataJson["sample_token"];
        Token sampleDataToken = sampleDataJson["token"];
        std::vector<SampleDataInfo>& sampleDatas = getExistingOrDefault(sample2SampleData, sampleToken);
        sampleDatas.push_back(SampleDataInfo{
            sampleDataToken,
            sampleDataJson["timestamp"],
            sampleDataJson["ego_pose_token"],
            sampleDataJson["calibrated_sensor_token"],
            sampleDataJson["fileformat"],
            sampleDataJson["is_key_frame"],
            sampleDataJson["filename"],
        });
    }

    return sample2SampleData;
}

std::map<Token, std::vector<EgoPoseInfo>> MetaDataReader::loadEgoPoseInfos(const std::filesystem::path& filePath) {
    std::map<Token, std::vector<EgoPoseInfo>> egoPoseInfos;
    return egoPoseInfos;
}


std::vector<Token> MetaDataReader::getAllSceneTokens() const {
    assert(loadFromDirectoryCalled);
    std::vector<Token> tokens;
    std::transform(scenes.begin(), scenes.end(), std::back_inserter(tokens), [](const SceneInfo& sceneInfo) {
        return sceneInfo.token;
    });
    return tokens;
}


std::optional<SceneInfo> MetaDataReader::getSceneInfo(const Token& sceneToken) const {
    auto it = std::find_if(scenes.begin(), scenes.end(), [&sceneToken](const SceneInfo& sceneInfo) {
        return sceneInfo.token == sceneToken;
    });
    if (it == scenes.end()) {
        return std::nullopt;
    }

    return std::optional(*it);
}

std::vector<SampleDataInfo> MetaDataReader::getSceneSampleData(const Token& sceneToken) const {
    std::vector<SampleDataInfo> sampleDataInfos;

    const auto sceneSamplesIt = scene2Samples.find(sceneToken);
    if(sceneSamplesIt == scene2Samples.end()) {
        assert(false);
        return sampleDataInfos;
    }
    const auto& sceneSamples = sceneSamplesIt->second;
    for(const auto& sceneSample : sceneSamples) {
        const Token& sceneSampleToken = sceneSample.token;
        const auto sceneSampleDatasIt = sample2SampleData.find(sceneSampleToken);
        if(sceneSampleDatasIt == sample2SampleData.end()) {
            return sampleDataInfos;
            assert(false);
        }

        const auto& sceneSampleDatas = sceneSampleDatasIt->second;
        for(const SampleDataInfo& sampleData: sceneSampleDatas) {
            sampleDataInfos.push_back(sampleData);
        }
    }

    return sampleDataInfos;
};