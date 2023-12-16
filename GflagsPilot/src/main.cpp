#include <gflags/gflags.h>

#include <iostream>

DEFINE_string(host, "localhost", "Server host address");
DEFINE_int32(port, 8080, "Server port");

// possible to have arguments validation
static bool validatePort(const char* flag, google::int32 value) {
    if (value > 0 && value < 32768)
        return true;
    std::cerr << "Invalid value for --" << flag << ": " << value << std::endl;
    return false;
}
static const bool port_dummy =
    google::RegisterFlagValidator(&FLAGS_port, &validatePort);

int main(int argc, char** argv) {
    google::SetVersionString("1.1.0");
    google::SetUsageMessage("./GflagsPilot");
    google::CommandLineFlagInfo info;
    // gflags::ParseCoomandLineFlags(&argc, &argv, true);
    google::ParseCommandLineFlags(&argc, &argv, true);

    // check if parameters set
    if (google::GetCommandLineFlagInfo("port", &info) && info.is_default) {
        std::cout << "port is not set" << std::endl;
    } else {
        std::cout << "port is set: " << FLAGS_port << std::endl;
    }

    // dynamic setting
    std::cout << google::SetCommandLineOption("port", "9909") << std::endl;
    // introduction from env setting
    std::cout << "Got '" << FLAGS_host << ":" << FLAGS_port << "'."
              << std::endl;

    return EXIT_SUCCESS;
}

/// usage
// ./GflagsPilot -host www.foobar.com -port 80
// use `--help` check help info
// ./GflagsPilot --help
// error of validating port
// ./GflagsPilot -host www.foobar.com -port -80
// introduce file `--flagfile=FileName`
// ./GflagsPilot --flagfile ../flags.txt
