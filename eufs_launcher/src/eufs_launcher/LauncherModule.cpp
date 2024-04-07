#include "LauncherModule.h"
#include <QString>
#include <algorithm>
#include <yaml-cpp>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

class EUFSLauncher(qt_gui_cpp::Plugin aPlugin)
 : aPlugin(plugin)
{
    // Get share directories
    mLauncherShare = ament_index_cpp::get_package_share_directory("eufs_launcher");
    mTracksShare = ament_index_cpp::get_package_share_directory("eufs_tracks");
    mConfigShare = ament_index_cpp::get_package_share_directory("eufs_config");

    // Grab the files
    std::filesystem::path defaultConfigPath = std::filesystem::path(mConfigShare) + std::filesystem::path(mLauncherOpsYaml);

    // Initialise plugin defaults that may have been changed by the user
    mPluginYamlPath = std::filesystem::path(mConfigShare) + std::filesystem::path(mPluginUserYaml);

    std::filesystem::path defaultYamlPath = std::filesystem::path(mConfigShare) + std::filesystem::path(mPluginYaml);

    // Parse YAMLs
    YAML::Node configYAML = YAML::LoadFile(defaultConfigPath.c_str());
    YAML::Node pluginYAML = YAML::LoadFile(mPluginYamlPath.c_str());

    // Parameters file
    EUFSLauncher::InitPluginConfig(defaultYamlPath);
}

static void 
EUFSLauncher::SetupQComboBox(QComboBox aQComboBox, const std::string& aDefaultMode, std::vector<std::string> aModes)
{
    aQComboBox->clear();

    // If the provided default mode is not in the other modes, add it
    if std::find(aModes.begin(), aModes.end(), aDefaultMode) != aModes.end() {
        QString qmode = QString::fromUtf8(aDefaultMode.c_str());
        aQComboBox->addItem(qmode);
    }

    // Loop over the provided modes and add any others we missed
    for (const auto& mode: aModes) {
        if mode != aDefaultMode {
            QString qmode = QString::fromUtf8(mode.c_str());
            aQComboBox->addItem(qmode);
        }
    }

}

void 
EUFSLauncher::LoadTrackDropdowns(void)
{
    // Peruses file system for files to add to the drop-down menus of the launcher

    # Clear the dropdowns
    mTrackSelector->clear();
    
    # Get tracks from eufs_tracks package
    const std::string worlds = "worlds";
    const std::filesystem::path worldDir = std::filesystem::path(mTracksShare) + std::filesystem::path(worlds);

    // world_files = [
    //     f for f in listdir(world_dir_path) if isfile(join(world_dir_path, f))
    // ]

    // Remove "blacklisted" files (ones that don't define tracks)
    const std::string blacklistFile = "blacklist.txt";
    const std::filesystem::path blacklistDir = worldDir + std::filesystem::path(blacklistFile);

    // Open the file and get the blacklist and worlds files
    with open(blacklist_filepath, "r") as f:
        blacklist = [f.strip() for f in f.readlines()]  # remove \n
        world_files = [f for f in world_files if f not in blacklist]

    // Add Tracks to Track Selector
    base_track = self.default_config["eufs_launcher"]["base_track"]
    if base_track in world_files:
        self.TRACK_SELECTOR.addItem(base_track.split(".")[0])
    for f in world_files:
        if f != base_track:
            self.TRACK_SELECTOR.addItem(f.split(".")[0])

}

void 
EUFSLauncher::LaunchButtonPressed(void)
{
            """
        Launches Gazebo and spawns the car.
        """
        self.logger.info("Launching Nodes...")

        // Calculate parameters to pass
        track_layout = f"track:={self.TRACK_SELECTOR.currentText()}"
        vehicle_model = f"vehicle_model:={self.VEHICLE_MODEL_MENU.currentText()}"
        command_mode = f"command_mode:={self.COMMAND_MODE_MENU.currentText()}"
        model_config = self.MODEL_CONFIGS[self.MODEL_PRESET_MENU.currentText()]
        vehicle_config = f"vehicle_config:={model_config}"
        robot_name = f"robot_name:={self.ROBOT_NAME_MENU.currentText()}"
        base_frame = f"base_frame:={self.BASE_FRAME}"
        parameters_to_pass = [
            track_layout,
            vehicle_model,
            command_mode,
            vehicle_config,
            robot_name,
            base_frame,
        ]

        // Get vehicle model information
        self.logger.info(f"Vehicle model: {self.VEHICLE_MODEL_MENU.currentText()}")
        self.logger.info(f"Command mode: {self.COMMAND_MODE_MENU.currentText()}")
        self.logger.info(f"Preset: {model_config}")
        self.logger.info(
            f"Robot description file: {self.ROBOT_NAME_MENU.currentText()}"
        )

        // Get checkbox parameter information
        for checkbox, param_if_on, param_if_off in self.checkbox_parameter_mapping:
            if checkbox.isChecked():
                self.logger.info(f"Checkbox enabled: {param_if_on}")
                parameters_to_pass.extend(param_if_on)
            else:
                self.logger.info(f"Checkbox disabled: {param_if_off}")
                parameters_to_pass.extend(param_if_off)

        // Rewrite yaml file with new parameters
        for parameter in parameters_to_pass:
            self.rewrite_yaml_config(parameter)

        noise_config = join(
            get_package_share_directory("eufs_config"),
            "config",
            "motionNoise.yaml",
        )
        self.rewrite_yaml_config(str("noise_config:=" + noise_config))

        vehicle_config = join(
            get_package_share_directory("eufs_config"),
            "config",
            model_config,
        )
        self.rewrite_yaml_config(str("vehicle_config:=" + vehicle_config))

        // Here we launch `simulation.launch.py`.
        self.launch_with_args(
            "eufs_launcher", "simulation.launch.py", parameters_to_pass
        )

        // Trigger launch files hooked to checkboxes
        for checkbox, effect_on, effect_off in self.checkbox_effect_mapping:
            if checkbox.isChecked():
                effect_on()
            else:
                effect_off()

        self.LAUNCH_BUTTON.setEnabled(False)

}

void 
EUFSLauncher::RewriteYamlConfig(const std::string& aParameter)
{
    """Rewrites a yaml file with a new key-value pair."""
    // extract key and value from parameter, form: key:=value
    key, value = parameter.split(":=")
    with open(self.plugin_yaml, "r") as f:
        data = yaml.safe_load(f)

    // iterate through the three nodes in the yaml file
    for node in data:
        // check if key already exists
        if key in data[node]["ros__parameters"]:
            self.logger.info(f"Overwriting {key} in {node} with {value}")
            data[node]["ros__parameters"][key] = value
            // check if boolean
            if value == "true":
                data[node]["ros__parameters"][key] = True
            elif value == "false":
                data[node]["ros__parameters"][key] = False

    with open(self.plugin_yaml, "w") as f:
        yaml.safe_dump(data, f)

}

void 
EUFSLauncher::InitPluginConfig(std::filesystem::path aDefaultYamlFile)
{
    YAML::Node defaultYAML = YAML::LoadAllFromFile(aDefaultYamlFile.c_str());
    YAML::Emitter yamloverwrite;

    yamloverwrite << defaultYAML;
    std::ofstream yamlout(mPluginYamlPath.c_str(), std::ofstream::out);
    yamlout << yamloverwrite.c_str();
    yamlout.close();
}

void 
EUFSLauncher::GeneratorButtonPressed(void)
{

}

void 
EUFSLauncher::ConverterButtonPressed(void)
{

}

void 
EUFSLauncher::RunWithoutArgs(const std::string& package, const std::string& program_name)
{
    std::string cmd = "stdbuf -o L ros2 run" + package + program_name;
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
}

void 
EUFSLauncher::LaunchWithArgs(const std::string& package, const std::string& launch_file, const std::vector<std::string> args)
{
    std::string cmd = "stdbuf -o L ros2 run" + package + launch_file + "use_sim_time:=True log_level:=debug";
    for (const auto& arg: args) {
        cmd = cmd + arg;
    }

    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }

}