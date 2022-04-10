#include <iostream>

#include "MathUtility/addition.h"
#include "MathUtility/division.h"
#include "PrintUtility/print_result.h"

using namespace std;

int main(int argc, char **argv)
{
    float first_no, second_no, result_add, result_div;

    std::cout << "Enter first number\t";
    std::cin >> first_no;
    std::cout << "Enter second number\t";
    std::cin >> second_no;

    result_add = addition(first_no, second_no);
    result_div = division(first_no, second_no);

    print_result("Addition", result_add);
    print_result("Division", result_div);
    // std::cout<< "Addition result:\t"<< result_add<< "\nDivision result:\t"<< result_div<< "\n";

    return 0;
}