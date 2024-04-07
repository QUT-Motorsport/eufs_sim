#include <QLabel>
#include <QtCore>
#include <QtGui>
#include <QApplication>  
#include <QCheckBox>
#include <QComboBox>
#include <QWidget>
#include <QPushButton>
#include <QFont>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <qt_gui_cpp>
#include <QMainWindow>
#include <filesystem>

class EUFSLauncher()
  : qt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    ///
    /// @brief Construct a new EUFSLauncher object
    /// 
    /// @param aPlugin QT GUI C++ Plugin element
    ///
    explicit EUFSLauncher(qt_gui_cpp::Plugin aPlugin);

    ///
    /// @brief Static function to define options for a combo box in the UI
    /// 
    /// @param aQComboBox The combo box to configure
    /// @param aDefaultMode Mode strings which defines default modes to select
    /// @param aModes A list of selectable modes
    ///
    static void SetupQComboBox(QComboBox aQComboBox, const std::string& aDefaultMode, std::vector<std::string> aModes);

    ///
    /// @brief Function to load in track dropdown options
    /// 
    ///
    void LoadTrackDropdowns(void);

    ///
    /// @brief Function to detect launch button presses
    /// 
    ///
    void LaunchButtonPressed(void);

    ///
    /// @brief Function to update yaml configuration files
    /// 
    /// @param aParameter The file to configure
    ///
    void RewriteYamlConfig(const std::string& aParameter);

    ///
    /// @brief Load in a yaml file with some configuration parameters
    /// 
    /// @param aDefaultYamlFile The full path to that yaml file
    ///
    void InitPluginConfig(std::filesystem::path aDefaultYamlFile);

    ///
    /// @brief Function to detect generator button presses
    /// 
    ///
    void GeneratorButtonPressed(void);

    ///
    /// @brief Function to detect converter button presses
    /// 
    ///
    void ConverterButtonPressed(void);

    ///
    /// @brief Execute a command without arguments
    /// 
    /// @param package Ros node name to execute
    /// @param program_name The specfic program name to run
    ///
    void RunWithoutArgs(const std::string& package, const std::string& program_name);

    ///
    /// @brief Execute a command with additional arguments
    /// 
    /// @param package Ros node name to execute
    /// @param program_name The specfic program name to run
    /// @param args Additional arguments to provide
    ///
    void LaunchWithArgs(const std::string& package, const std::string& launch_file, const std::vector<std::string> args);

private:

    // QWidget class variable
    mWidget = QWidget();

    // Folder for the launcher
    const std::string mLauncherShare;
    
    // Folder for tracks
    const std::string mTracksShare;
    
    // Folder for config
    const std::string mConfigShare;

    // YAML filenames
    const std::string mPluginYaml = "config/pluginParams.yaml";
    const std::string mPluginUserYaml = "config/pluginUserParams.yaml";
    const std::string mLauncherOpsYaml = "config/launcherOptions.yaml";

    // Frame ID
    const std::string mBaseFrame;

    // Plugin YAML Path
    std::filesystem::path mPluginYamlPath;

    // Give widget components permanent names
    QComboBox* mTrackSelector;
    QPushButton* mLaunchButton;
    QPushButton* mGeneratorButton;
    QPushButton* mConverterButton:
    QPushButton* mRefreshTrackButton:
    QComboBox* mVehicleModelMenu:
    QComboBox* mCommandModeMenu:
    QComboBox* mModelPresetMenu;
    QComboBox* mRobotNameMenu;

}