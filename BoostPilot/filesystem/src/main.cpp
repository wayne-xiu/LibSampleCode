#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    /// pwd
    bfs::path full_path(bfs::current_path());
    cout << "pwd: " << full_path << endl;

    std::cout << "parent directory: " << full_path.parent_path() << std::endl;
    
    /// check file size
    cout << argv[0] << " file size: " << bfs::file_size(argv[0]) << " bytes" << endl;

    /// check, create and remove directory
    std::string testdata_directory = "test_data";
    if (!bfs::exists(testdata_directory)) {
        bfs::create_directories(testdata_directory);
    }
    bfs::ofstream ofs{testdata_directory + "/test.txt"};
    ofs << "hello boost::filesystem\n";

    try
    {
        bfs::remove_all(bfs::path(testdata_directory));
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    

    return 0;
}
