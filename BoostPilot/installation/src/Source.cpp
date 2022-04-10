#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

int main()
{
    int major_ver = BOOST_VERSION / 100000,
        minor_ver = (BOOST_VERSION / 100) % 1000,
		sub_minor_ver = BOOST_VERSION % 100;
	std::cout << "boost version: " << major_ver << "." << minor_ver << "."
        << sub_minor_ver << std::endl;
    // header only library example
    using namespace boost::lambda;
    typedef std::istream_iterator<int> in;

    // std::for_each(
    //     in(std::cin), in(), std::cout << (_1 * 3) << " ");

    using boost::lexical_cast;
    int s = 42;
    std::string s_str = lexical_cast<std::string>(s);
    std::cout << "intger to string cast: " << s_str << std::endl;
    double num = lexical_cast<double>("12.34");
    std::cout << "string to double cast: " << num << std::endl;
    try
    {
        int i = lexical_cast<int>("HelloBoost");
    }
    catch (boost::bad_lexical_cast &e)
    {
        std::cout << "Exception caught: " << e.what() << std::endl;
    }

    // library that needs to be linked
    // std::string line;
    // boost::regex pat( "^Subject: (Re: |Aw: )*(.*)" );

    // while (std::cin)
    // {
    //     std::getline(std::cin, line);
    //     boost::smatch matches;
    //     if (boost::regex_match(line, matches, pat))
    //         std::cout << matches[2] << std::endl;
    // }
}