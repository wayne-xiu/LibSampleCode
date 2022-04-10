#include <glog/logging.h>
#include <iostream>

int main(int argc, char** argv )
{
    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "This is an info  message";
    LOG(WARNING) << "This is a warning message";
    LOG(ERROR) << "This is an error message";
    LOG(FATAL) << "This is a fatal message";

    std::cout << "e" << std::endl;
	
    return 0;
}