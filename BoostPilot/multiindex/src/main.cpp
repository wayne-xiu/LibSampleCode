#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>

#include <iostream>
#include <string>
// example of boost::multi_index_container

using namespace boost::multi_index;

struct Employee
{
    int id;
    std::string name;
    int age;

    friend std::ostream &operator<<(std::ostream &os, const Employee &e)
    {
        os << e.id << " " << e.name << " " << e.age;
        return os;
    }
};

typedef multi_index_container<
    Employee,
    indexed_by<
        ordered_unique<member<Employee, int, &Employee::id>>,
        ordered_non_unique<member<Employee, std::string, &Employee::name>>,
        ordered_non_unique<member<Employee, int, &Employee::age>>
    >
> employee_set;

void TestMultiIndex() {
    employee_set es;

    es.insert(Employee{1, "Robert", 30});
    es.insert(Employee{2, "Mike", 20});
    es.insert(Employee{3, "John", 40});

    // listing all employees
    std::cout << "all employees:" << std::endl;
    for (auto &e : es) {
        std::cout << e << std::endl;
    }

    // listing all employees by id
    std::cout << "all employees by id:" << std::endl;
    for (const Employee &e : es.get<0>()) {
        std::cout << e << std::endl;
    }

    // listing all employees by name
    std::cout << "all employees by name:" << std::endl;
    for (const Employee &e : es.get<1>()) {
        std::cout << e << std::endl;
    }

    // listing all employees by age
    std::cout << "all employees by age:" << std::endl;
    for (const Employee &e : es.get<2>()) {
        std::cout << e << std::endl;
    }
}

int main()
{
    TestMultiIndex();
    return 0;
}